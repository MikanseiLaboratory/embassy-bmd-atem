//! `atem-usb-control` — desktop probe / CLI for the Blackmagic Design ATEM USB-C
//! "8.6+" control link (host side, `nusb` backend).
//!
//! ```text
//! atem-usb-control --list
//! atem-usb-control --verbose
//! atem-usb-control --send "CPgI 0 3" --send "DAut 0"
//! atem-usb-control --pid 0xbe7c --iface 1 --out-ep 0x02 --in-ep 0x82
//! atem-usb-control --replay captures/switcher.hex
//! ```
//!
//! On Windows the control interface must be bound to WinUSB before `nusb` can claim
//! it (see the crate docs); on Linux udev write access to `/dev/bus/usb` is required.

use std::io;
use std::io::Write as _;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use bmd_atem_atom::{
    AtomIter, NextTransitionStyle, encode_atom_change_preview_input,
    encode_atom_change_program_input, encode_atom_change_transition_next, encode_atom_do_ftb_auto,
    encode_atom_do_ftb_cut, encode_atom_do_transition_auto, encode_atom_do_transition_cut,
};
use clap::{Parser, ValueEnum};
use embassy_bmd_atem::FrameError;
use embassy_bmd_atem::usb::protocol::{DataLenLayout, StreamParser, build_flow_host};
use embassy_bmd_atem::usb::session::{
    AtemUsbSession, SessionEvent, UsbSessionConfig, UsbSessionError, UsbSessionState,
};
use embassy_bmd_atem_std as backend;

/// Decode of the device → host `0x04` length field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum LayoutArg {
    /// Little-endian right after the header (pyatem RX parse). The parser tolerates
    /// the mirrored layout on top of this.
    #[value(name = "le-start")]
    LeStart,
    /// Big-endian at the body end (docs table / symmetric to host → device).
    #[value(name = "be-end")]
    BeEnd,
}

impl From<LayoutArg> for DataLenLayout {
    fn from(v: LayoutArg) -> Self {
        match v {
            LayoutArg::LeStart => Self::LeAtStart,
            LayoutArg::BeEnd => Self::BeAtEnd,
        }
    }
}

#[derive(Parser, Debug)]
#[command(
    name = "atem-usb-control",
    version,
    about = "Control a Blackmagic Design ATEM switcher over USB-C (8.6+ protocol)"
)]
struct Cli {
    /// Enumerate connected Blackmagic Design ATEM USB devices and exit.
    #[arg(long)]
    list: bool,
    /// Product id to select (hex, e.g. 0xbe7c). Auto-detects among known models when omitted.
    #[arg(long, value_name = "PID")]
    pid: Option<String>,
    /// Claim this interface instead of auto-detecting the control interface.
    #[arg(long, value_name = "N")]
    iface: Option<u8>,
    /// Bulk OUT endpoint override (stock ATEMs: 0x02).
    #[arg(long, value_name = "ADDR")]
    out_ep: Option<u8>,
    /// Bulk IN endpoint override (stock ATEMs: 0x82).
    #[arg(long, value_name = "ADDR")]
    in_ep: Option<u8>,
    /// Handshake timeout in ms.
    #[arg(long, default_value_t = 3000)]
    timeout_ms: u64,
    /// Stop after this many ms of listening (0 = run until Ctrl-C).
    #[arg(long, default_value_t = 0)]
    listen_ms: u64,
    /// Wait this long (ms) after connect before sending; also capped by the first atoms block.
    #[arg(long, default_value_t = 1000)]
    settle_ms: u64,
    /// Send an atom after connecting; repeatable. Forms:
    /// `CPgI <me> <src>`, `CPvI <me> <src>`, `DAut <me>`, `DCut <me>`, `FtbA <me>`,
    /// `FCut <me> <bool>`, `CTTp <me> <style 0-4>`, `raw <hex>`.
    #[arg(long = "send", value_name = "SPEC")]
    sends: Vec<String>,
    /// Delay between `--send` presets, ms.
    #[arg(long, default_value_t = 250)]
    send_delay_ms: u64,
    /// Record received atom blocks (hex, one per line) to this file.
    #[arg(long, value_name = "FILE")]
    dump: Option<PathBuf>,
    /// Record every raw bulk transfer (`>` host→device / `<` device→host, hex
    /// per line) to this file for protocol analysis.
    #[arg(long, value_name = "FILE")]
    raw_dump: Option<PathBuf>,
    /// Parse a recorded hex replay instead of talking to a live device.
    #[arg(long, value_name = "FILE")]
    replay: Option<PathBuf>,
    /// Device → host `0x04` length layout.
    #[arg(long, default_value = "le-start")]
    layout: LayoutArg,
    /// Log raw bytes and full atom details.
    #[arg(long)]
    verbose: bool,
    /// Skip the host → device 0x03 receive-credit grant (protocol probing:
    /// the switcher will not start the initial dump).
    #[arg(long)]
    no_flow: bool,
    /// Disable the automatic 0x03 credit re-grant after each consumed chunk
    /// (protocol probing: the dump stops after the initially granted blocks).
    #[arg(long)]
    no_credit: bool,
    /// Send bare zero-length keepalive packets every N ms. **Stalls on real
    /// hardware** — probing only.
    #[arg(long, value_name = "MS")]
    keepalive_ms: Option<u32>,
    /// Send a raw tunnel frame (hex, e.g. `00050001` for the undocumented
    /// host → device `0x05` packet) after connecting; probing only.
    #[arg(long = "tunnel", value_name = "HEX")]
    tunnels: Vec<String>,
    /// Send `00 05 00 01` every N ms during the listen phase (heartbeat
    /// probing for the undocumented `0x05` packet).
    #[arg(long, value_name = "MS")]
    poll_05_ms: Option<u32>,
    /// Send a small (1-block) `0x03` credit grant every N ms during the listen
    /// phase, bypassing the idle-pause — probing the poll-for-updates model.
    #[arg(long, value_name = "MS")]
    poll_credit_ms: Option<u32>,
    /// Send a `TiRq` (NOP/time-request) poll every N ms during the listen
    /// phase, flushing a fresh credit grant first — the ASC/pyatem-style
    /// state poll.
    #[arg(long, value_name = "MS")]
    poll_tirq_ms: Option<u32>,
    /// Receive credit in 1024 B blocks granted to the switcher (initial grant
    /// and per-chunk re-grant). Default 4 (live-verified).
    #[arg(long, default_value_t = 4, value_name = "N")]
    credit: u16,
}

fn main() {
    let cli = Cli::parse();
    let layout = DataLenLayout::from(cli.layout);
    let max_atoms_len = 0x1_0000;

    if cli.list {
        match backend::list_devices(None) {
            Ok(devices) => {
                if devices.is_empty() {
                    println!("no Blackmagic Design (VID 0x1EDB) USB devices connected");
                }
                for d in devices {
                    println!(
                        "device: vid=0x{:04x} pid=0x{:04x} address={} known_atem={}",
                        d.vendor_id(),
                        d.product_id(),
                        d.device_address(),
                        backend::KNOWN_PIDS.contains(&d.product_id()),
                    );
                }
            }
            Err(e) => {
                eprintln!("enumeration failed: {e}");
                std::process::exit(1);
            }
        }
        return;
    }

    if let Some(path) = cli.replay.as_deref() {
        run_replay(path, layout, max_atoms_len, cli.dump, cli.verbose);
        return;
    }

    if let Err(e) = run_live(&cli, layout, max_atoms_len) {
        eprintln!("error: {e}");
        std::process::exit(1);
    }
}

// ---------------------------------------------------------------- live path

fn run_live(cli: &Cli, layout: DataLenLayout, max_atoms_len: usize) -> Result<(), String> {
    let pid_filter = cli.pid.as_deref().map(parse_pid).transpose()?;

    let devices = backend::list_devices(pid_filter).map_err(|e| e.to_string())?;
    if devices.is_empty() {
        return Err(format!(
            "no ATEM USB device found{} (VID 0x1EDB; run with --list)",
            match pid_filter {
                Some(p) => format!(" with pid 0x{p:04x}"),
                None => String::new(),
            }
        ));
    }
    if devices.len() > 1 {
        eprintln!(
            "warning: {} ATEM devices connected, using the first (pass --pid to disambiguate)",
            devices.len()
        );
    }
    let device_info = devices[0].clone();
    println!(
        "opening vid=0x{:04x} pid=0x{:04x} address={}…",
        device_info.vendor_id(),
        device_info.product_id(),
        device_info.device_address()
    );

    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build().unwrap();
    rt.block_on(run_live_async(cli, device_info, layout, max_atoms_len))
}

async fn run_live_async(
    cli: &Cli,
    device_info: nusb::DeviceInfo,
    layout: DataLenLayout,
    max_atoms_len: usize,
) -> Result<(), String> {
    let host = backend::open_device(device_info, cli.iface, cli.out_ep, cli.in_ep)
        .await
        .map_err(|e| e.to_string())?;
    let mut host = host;
    if let Some(path) = cli.raw_dump.as_deref() {
        match std::fs::File::create(path) {
            Ok(f) => host.set_raw_log(Some(f)),
            Err(e) => eprintln!("warning: cannot open raw dump file {}: {e}", path.display()),
        }
    }
    let (out_ep, in_ep) = host.endpoints();
    println!("control link ready (iface claimed, bulk OUT 0x{out_ep:02x} / IN 0x{in_ep:02x})");

    let credit = if cli.no_flow || cli.no_credit { None } else { Some(cli.credit) };
    let config = UsbSessionConfig {
        data_len_layout: layout,
        max_atoms_len: max_atoms_len.min(u16::MAX as usize & 0xFFFF) as u16,
        initial_flow_blocks: if cli.no_flow { None } else { credit },
        auto_credit_blocks: if cli.no_credit { None } else { credit },
        keepalive_ms: cli.keepalive_ms.unwrap_or(u32::MAX),
        ..UsbSessionConfig::default()
    };
    let mut session = AtemUsbSession::new(host, config);
    let t0 = Instant::now();
    let now = || t0.elapsed().as_millis() as u32;

    // Handshake with a caller-side timeout (the crate leaves timeouts to the runtime).
    let connect = session.connect(now());
    match tokio::time::timeout(Duration::from_millis(cli.timeout_ms), connect).await {
        Ok(Ok(())) => println!("sent 0x01 init packet (00 01 10 92), waiting for switcher…"),
        Ok(Err(e)) => return Err(format!("connect failed: {e}")),
        Err(_) => Err(format!(
            "switcher did not acknowledge within {} ms — this firmware may predate the 8.6+ USB protocol (unsupported)",
            cli.timeout_ms
        ))?,
    }

    let mut printer = EventPrinter::new(cli.dump.clone(), cli.verbose);
    // Large enough for a whole dump chunk (live: 0x2834) plus carried atoms.
    let mut buf = vec![0u8; 0x1_0000];
    let ctrl_c = tokio::signal::ctrl_c();
    tokio::pin!(ctrl_c);

    // Consume the initial state dump until the first atoms block (or the settle window).
    let settle = Instant::now() + Duration::from_millis(cli.settle_ms);
    while session.state() != UsbSessionState::Running {
        tokio::select! {
            r = session.poll(now(), &mut buf) => {
                if let Err(e) = handle_poll(r, &mut printer) {
                    println!("poll error ({e}) — continuing");
                }
                if session.state() == UsbSessionState::New {
                    break;
                }
            }
            _ = tokio::time::sleep(settle.saturating_duration_since(Instant::now())) => {}
            _ = &mut ctrl_c => return Err("interrupted".into()),
        }
        if session.state() != UsbSessionState::Running && Instant::now() >= settle {
            break;
        }
    }

    // Send the requested presets.
    for (index, spec) in cli.sends.iter().enumerate() {
        let preset = parse_send(spec).map_err(|e| format!("--send #{index}: {e}"))?;
        session.send_atoms(&preset, now()).await.map_err(|e| format!("send {spec}: {e}"))?;
        println!("sent [{spec}] -> {} bytes", preset.len());
        let delay = tokio::time::sleep(Duration::from_millis(cli.send_delay_ms));
        tokio::select! {
            () = delay => {}
            _ = &mut ctrl_c => return Err("interrupted".into()),
        }
    }

    // Send raw tunnel probing frames (e.g. the undocumented 0x05).
    for (index, spec) in cli.tunnels.iter().enumerate() {
        let raw = parse_send_hex(spec).map_err(|e| format!("--tunnel #{index}: {e}"))?;
        session.write_raw(&raw, now()).await.map_err(|e| format!("tunnel {spec}: {e}"))?;
        println!("sent tunnel [{spec}] -> {} bytes", raw.len());
    }

    // Listen until the deadline or Ctrl-C, optionally polling with 0x05
    // and/or periodic small credit grants.
    let deadline =
        (cli.listen_ms > 0).then(|| Instant::now() + Duration::from_millis(cli.listen_ms));
    let mut next_poll_05 = cli
        .poll_05_ms
        .map(|ms| Instant::now() + Duration::from_millis(u64::from(ms)))
        .unwrap_or(Instant::now());
    let mut next_poll_credit = cli
        .poll_credit_ms
        .map(|ms| Instant::now() + Duration::from_millis(u64::from(ms)))
        .unwrap_or(Instant::now());
    let mut next_poll_tirq = cli
        .poll_tirq_ms
        .map(|ms| Instant::now() + Duration::from_millis(u64::from(ms)))
        .unwrap_or(Instant::now());
    loop {
        let timer = async {
            if let Some(d) = deadline {
                tokio::time::sleep_until(tokio::time::Instant::from_std(d)).await;
            } else {
                std::future::pending::<()>().await;
            }
        };
        let credit_timer = async {
            if cli.poll_credit_ms.is_some() {
                tokio::time::sleep_until(tokio::time::Instant::from_std(next_poll_credit)).await;
            } else {
                std::future::pending::<()>().await;
            }
        };
        let tirq_timer = async {
            if cli.poll_tirq_ms.is_some() {
                tokio::time::sleep_until(tokio::time::Instant::from_std(next_poll_tirq)).await;
            } else {
                std::future::pending::<()>().await;
            }
        };
        tokio::select! {
            r = session.poll(now(), &mut buf) => {
                // Long listens survive transient errors (IN stalls are already
                // absorbed by the backend; log anything else and keep going).
                if let Err(e) = handle_poll(r, &mut printer) {
                    println!("poll error ({e}) — continuing");
                }
            }
            _ = timer => break,
            _ = tirq_timer, if cli.poll_tirq_ms.is_some() => {
                match session.poll_ping(now()).await {
                    Ok(()) => println!("poll: TiRq sent (+credit)"),
                    Err(e) => println!("poll: TiRq failed ({e}) — continuing"),
                }
                next_poll_tirq += Duration::from_millis(u64::from(cli.poll_tirq_ms.unwrap_or(500)));
            }
            _ = credit_timer, if cli.poll_credit_ms.is_some() => {
                // Grant one fresh block regardless of the idle-pause: probes
                // whether queued state (e.g. button events) flushes on grant.
                match session.write_raw(&build_flow_host(1), now()).await {
                    Ok(()) => println!("poll: granted 1 credit block"),
                    Err(e) => println!("poll: credit write failed ({e}) — continuing"),
                }
                next_poll_credit += Duration::from_millis(u64::from(cli.poll_credit_ms.unwrap_or(3000)));
            }
            _ = tokio::time::sleep_until(tokio::time::Instant::from_std(next_poll_05)), if cli.poll_05_ms.is_some() => {
                match session.write_raw(&[0x00, 0x05, 0x00, 0x01], now()).await {
                    Ok(()) => println!("poll: sent 0x05"),
                    Err(e) => println!("poll: 0x05 write failed ({e}) — cleared, continuing"),
                }
                next_poll_05 += Duration::from_millis(u64::from(cli.poll_05_ms.unwrap_or(1000)));
            }
            _ = &mut ctrl_c => { println!("interrupted"); break; }
        }
    }
    let _ = io::stdout().flush();
    Ok(())
}

/// Handle one `poll` result, printing events. Returns `true` when an event occurred.
fn handle_poll<E>(
    result: Result<Option<SessionEvent<'_>>, UsbSessionError<E>>,
    printer: &mut EventPrinter,
) -> Result<bool, String>
where
    E: core::fmt::Debug + core::fmt::Display,
{
    match result {
        Ok(Some(SessionEvent::InitAck)) => {
            println!("[tunn] 0x02 init-ack received");
            Ok(true)
        }
        Ok(Some(SessionEvent::Atoms { block, first })) => {
            printer.print_atoms(block, first);
            Ok(true)
        }
        Ok(Some(SessionEvent::FlowCtrl { blocks })) => {
            println!("[tunn] 0x03 flow: switcher announced {blocks} RX-block(s) (1024 B each)");
            Ok(true)
        }
        Ok(Some(SessionEvent::Unknown { pkt_type })) => {
            println!("[tunn] 0x{pkt_type:02x}? unknown packet type (treated as keepalive/noise)");
            Ok(true)
        }
        Ok(None) => Ok(false),
        Err(UsbSessionError::Protocol(FrameError::InvalidLength)) => {
            eprintln!("[tunn] oversized atoms block: parser resynchronised, continuing");
            Ok(false)
        }
        Err(UsbSessionError::Protocol(e)) => Err(format!("protocol error: {e}")),
        Err(UsbSessionError::NotConnected) => Err("session not connected".into()),
        Err(UsbSessionError::Backend(e)) => Err(format!("USB failure: {e} (device lost?)")),
    }
}

// ---------------------------------------------------------------- replay path

fn run_replay(
    path: &std::path::Path,
    layout: DataLenLayout,
    max_atoms_len: usize,
    dump: Option<PathBuf>,
    verbose: bool,
) {
    let transfers = match backend::load_replay_file(path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("cannot read {}: {e}", path.display());
            std::process::exit(1);
        }
    };
    println!("loaded {} transfer(s) from {}", transfers.len(), path.display());

    let mut parser = StreamParser::new(layout, max_atoms_len);
    let mut printer = EventPrinter::new(dump, verbose);
    let mut out = [0u8; 4096];
    for (index, transfer) in transfers.iter().enumerate() {
        parser.push(transfer);
        loop {
            match parser.try_next(&mut out) {
                Ok(Some(embassy_bmd_atem::usb::protocol::ParsedDeviceFrame::InitAck)) => {
                    println!("[tunn] 0x02 init-ack received");
                }
                Ok(Some(embassy_bmd_atem::usb::protocol::ParsedDeviceFrame::Data {
                    atoms_len,
                })) => {
                    printer.print_atoms(&out[..atoms_len], index == 0);
                }
                Ok(Some(embassy_bmd_atem::usb::protocol::ParsedDeviceFrame::FlowCtrl {
                    blocks,
                })) => {
                    println!("[tunn] 0x03 flow: {blocks} block(s)");
                }
                Ok(Some(embassy_bmd_atem::usb::protocol::ParsedDeviceFrame::Unknown {
                    pkt_type,
                })) => {
                    println!("[tunn] 0x{pkt_type:02x}? unknown type");
                }
                Ok(None) => break,
                Err(embassy_bmd_atem::FrameError::InvalidLength) => {
                    eprintln!("[tunn] oversized atoms blob in replay (resynced)");
                    break;
                }
                Err(e) => {
                    eprintln!("framing error in replay: {e}");
                    break;
                }
            }
        }
    }
}

// ---------------------------------------------------------------- printing

struct EventPrinter {
    dump: Option<std::fs::File>,
    verbose: bool,
    atoms_blocks: usize,
    atoms_bytes: usize,
}

impl EventPrinter {
    fn new(dump: Option<PathBuf>, verbose: bool) -> Self {
        let dump = dump.and_then(|path| match std::fs::File::create(&path) {
            Ok(f) => Some(f),
            Err(e) => {
                eprintln!("cannot open dump file {}: {e}", path.display());
                None
            }
        });
        Self { dump, verbose, atoms_blocks: 0, atoms_bytes: 0 }
    }

    fn print_atoms(&mut self, block: &[u8], first: bool) {
        self.atoms_blocks += 1;
        self.atoms_bytes += block.len();
        if let Some(f) = self.dump.as_mut() {
            let _ = writeln!(f, "{}", to_hex(block));
        }
        let tag = if first { "first" } else { "next" };
        println!(
            "[recv] {} atoms block: {} bytes (total: {} blocks / {} bytes)",
            tag,
            block.len(),
            self.atoms_blocks,
            self.atoms_bytes
        );
        let mut names = Vec::new();
        let mut count = 0usize;
        let mut trunc_err = None;
        for item in AtomIter::new(block) {
            match item {
                Ok(atom) => {
                    count += 1;
                    if names.len() < 8 {
                        let shown = if self.verbose {
                            format!("{}({}B)", atom.name.display_lossy(), atom.data.len())
                        } else {
                            atom.name.display_lossy().to_string()
                        };
                        names.push(shown);
                    }
                }
                Err(e) => {
                    trunc_err = Some(e);
                    break;
                }
            }
        }
        match (count, trunc_err) {
            (0, _) => println!("       (empty block)"),
            (n, None) => println!("       fields[{n}]: {}", names.join(", ")),
            (n, Some(e)) => {
                println!("       fields[{n}]: {} … malformed end ({e})", names.join(", "))
            }
        }
        if self.verbose && !block.is_empty() {
            let head = &block[..block.len().min(48)];
            println!("       hex: {}", to_hex(head));
        }
    }
}

fn to_hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}

fn parse_pid(raw: &str) -> Result<u16, String> {
    let trimmed = raw.trim();
    let value = trimmed.trim_start_matches("0x").trim_start_matches("0X");
    u16::from_str_radix(value, 16).map_err(|_| format!("pid `{raw}` is not a hex u16"))
}

/// Parse a `--send` spec into atom records.
fn parse_send(spec: &str) -> Result<Vec<u8>, String> {
    let parts: Vec<&str> = spec.split_whitespace().collect();
    let Some(cmd) = parts.first().map(|c| c.to_ascii_uppercase()) else {
        return Err("empty spec".into());
    };
    let args = &parts[1..];
    let num = |idx: usize, what: &str| -> Result<u8, String> {
        args.get(idx)
            .map(|v| v.parse::<u8>())
            .transpose()
            .map_err(|_| format!("{what} must be a u8"))?
            .ok_or_else(|| format!("{what} missing"))
    };
    match cmd.as_str() {
        "CPGI" => {
            let me = num(0, "me")?;
            let src = src_arg(args.get(1))?;
            Ok(encode_atom_change_program_input(me, src).into())
        }
        "CPVI" => {
            let me = num(0, "me")?;
            let src = src_arg(args.get(1))?;
            Ok(encode_atom_change_preview_input(me, src).into())
        }
        "DAUT" => Ok(encode_atom_do_transition_auto(num(0, "me")?).into()),
        "DCUT" => Ok(encode_atom_do_transition_cut(num(0, "me")?).into()),
        "FTBA" => Ok(encode_atom_do_ftb_auto(num(0, "me")?).into()),
        "FCUT" => Ok(encode_atom_do_ftb_cut(num(0, "me")?, bool_arg(args.get(1))).into()),
        "CTTP" => {
            let me = num(0, "me")?;
            let style = num(1, "style")?;
            if style > 4 {
                return Err("style must be 0..=4 (Mix, Dip, Wipe, Dve, Sting)".into());
            }
            let style = match style {
                0 => NextTransitionStyle::Mix,
                1 => NextTransitionStyle::Dip,
                2 => NextTransitionStyle::Wipe,
                3 => NextTransitionStyle::Dve,
                _ => NextTransitionStyle::Sting,
            };
            Ok(encode_atom_change_transition_next(me, style).into())
        }
        "RAW" => {
            if let Some(hex) = args.first() {
                parse_send_hex(hex)
            } else if let Some((_, rest)) = spec.split_once(':') {
                parse_send_hex(rest)
            } else {
                Err("raw requires a hex argument (e.g. `raw 000c...`)".into())
            }
        }
        other => Err(format!("unknown preset `{other}` (see --help)")),
    }
}

fn src_arg(arg: Option<&&str>) -> Result<u16, String> {
    arg.map(|v| v.parse::<u16>())
        .transpose()
        .map_err(|_| "source must be a u16".to_string())?
        .ok_or_else(|| "source missing".to_string())
}

fn bool_arg(arg: Option<&&str>) -> bool {
    arg.map(|v| {
        v.eq_ignore_ascii_case("1")
            || v.eq_ignore_ascii_case("true")
            || v.eq_ignore_ascii_case("yes")
    })
    .unwrap_or(true)
}

fn parse_send_hex(raw: &str) -> Result<Vec<u8>, String> {
    let clean: String = raw.chars().filter(|c| c.is_ascii_hexdigit()).collect();
    if !clean.len().is_multiple_of(2) {
        return Err("hex payload has odd length".into());
    }
    (0..clean.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&clean[i..i + 2], 16).map_err(|e| e.to_string()))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_parse_to_wire_atoms() {
        assert_eq!(
            parse_send("CPgI 0 3").unwrap(),
            bmd_atem_atom::encode_atom_change_program_input(0, 3).to_vec()
        );
        assert_eq!(parse_send("DAut 1").unwrap()[8], 1);
        assert_eq!(parse_send("FCut 2 false").unwrap()[9], 0);
        assert_eq!(
            parse_send("raw 000c00004443757400000000").unwrap(),
            vec![0, 0x0c, 0, 0, b'D', b'C', b'u', b't', 0, 0, 0, 0]
        );
        assert_eq!(
            parse_send("DCut 0").unwrap(),
            bmd_atem_atom::encode_atom_do_transition_cut(0).to_vec()
        );
    }
}
