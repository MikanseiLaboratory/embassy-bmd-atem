# embassy-bmd-atem

[![CI](https://github.com/MikanseiLaboratory/embassy-bmd-atem/actions/workflows/ci.yml/badge.svg)](https://github.com/MikanseiLaboratory/embassy-bmd-atem/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/embassy-bmd-atem.svg)](https://crates.io/crates/embassy-bmd-atem)

Blackmagic Design ATEM switcher control protocol for embedded Rust and std,
in one workspace:

- **`bmd-atem-atom`** — transport-independent atom (command record) codec
  ([crates.io](https://crates.io/crates/bmd-atem-atom)).
- **`embassy-bmd-atem`** — `no_std` protocol core with two transports:
  - `udp` — the LAN **BURP** protocol (port 9910): 12-byte packet header,
    connect handshake, cumulative-ACK queue, retransmission helpers, and
    `edge-nal`-style async UDP traits (continuation of the archived
    [edge-bmd-atem](https://github.com/MikanseiLaboratory/edge-bmd-atem)).
  - `usb` — the **USB-C "8.6+" control link, host side**: plug your
    microcontroller (or PC) into the switcher's USB port and speak the tunnel
    protocol. See [`docs/PROTOCOL_USB.md`](docs/PROTOCOL_USB.md).
- **`embassy-bmd-atem-std`** — Tokio + [nusb](https://crates.io/crates/nusb)
  desktop backend and the `atem-usb-control` CLI
  ([crates.io](https://crates.io/crates/embassy-bmd-atem-std)).
- **`examples/rp2040-usb-atem`** — RP2040 example wiring the USB host session
  (transport transport integration lands with real hardware bring-up).

## Quick start

### Control an ATEM over USB from a PC

```sh
cargo install embassy-bmd-atem-std   # or: cargo run -p embassy-bmd-atem-std --bin atem-usb-control

atem-usb-control --list                # enumerate VID 0x1EDB devices
atem-usb-control --verbose             # connect, dump initial state
atem-usb-control --send "CPgI 0 3"     # put input 3 on program of ME1
atem-usb-control --send "DCut 0"       # CUT
atem-usb-control --send "DAut 0"       # AUTO transition
atem-usb-control --send "FtbA 0"       # fade to black
```

- **Windows**: the switcher's vendor control interface must be bound to
  **WinUSB** (e.g. with [Zadig](https://zadig.akeo.ie/)) for nusb access.
  Replacing the driver stops ATEM Software Control from using that interface
  until reverted. Linux/macOS need no driver changes (Linux needs udev write
  access on `/dev/bus/usb`).
- Only one process may claim the control interface: close ATEM Software Control
  first.
- Protocol support is the **8.6+ USB generation** (ATEM Mini family on
  firmware ≥ 8.6). Older firmware simply times out on the handshake.

### One command, either transport

Both transports carry **identical atom records**; only the wrapper differs.
Build the payload once with `AtomWriter` and frame it for whichever link is up:

```rust,ignore
use embassy_bmd_atem::{AtomWriter, FourCc, frame_command, FrameContext, FramePlan, TransportKind};

let mut atoms = [0u8; 64];
let mut w = AtomWriter::new(&mut atoms);
w.push_source(FourCc(*b"CPgI"), 0, 3).unwrap(); // program input 3 on ME1

let mut wire = [0u8; 128];
match frame_command(TransportKind::Usb, FrameContext::usb(), w.as_slice(), &mut wire)? {
    FramePlan::AnnounceThenPayload { announce, payload } => {
        // USB: two bulk writes — the 0x04 announce, then the payload.
        let _ = (announce, payload);
    }
    FramePlan::Datagram(datagram) => {
        // UDP: one datagram (the arm used with TransportKind::Udp).
        let _ = datagram;
    }
}
```

| | UDP / BURP | USB-C 8.6+ |
|---|---|---|
| wrapper | 12-byte header (flags, session, packet ids) | `0x04` announce + payload |
| connection | `0x01` Connect / `0x02` ConnectAck | `0x01` init / `0x02` ack |
| reliability | ACK queue + retransmission | none; flow-control windows |
| payload | **atom records** | **atom records** |

### UDP from an MCU (edge-nal style)

```rust,ignore
let mut session = embassy_bmd_atem::AtemSession::connect(
    &mut udp, remote, 0x2970, SessionConfig::default(),
    || t.elapsed().as_millis() as u32,
).await?;

// Commands go through the session so packet ids and the ACK queue are tracked:
let mut atoms = [0u8; 32];
let mut w = embassy_bmd_atem::AtomWriter::new(&mut atoms);
w.push_source(embassy_bmd_atem::FourCc(*b"CPgI"), 0, 3).unwrap();
let pid = session.take_sender_packet_id();
let pkt = session.command_packet(pid, w.as_slice(), 0);
udp.send(remote, &pkt.to_bytes()).await?;
```

### USB host session (embedded)

```rust,ignore
let mut session = embassy_bmd_atem::AtemUsbSession::new(bulk_backend, UsbSessionConfig::default());
session.connect(now_ms()).await?;
loop {
    match session.poll(now_ms(), &mut buf).await? {
        Some(SessionEvent::Atoms { block, first }) => { /* parse AtomIter::new(block) */ }
        Some(SessionEvent::FlowCtrl { blocks }) => { … }
        Some(SessionEvent::InitAck) | Some(SessionEvent::Unknown { .. }) => {}
        None => {}
    }
}
session.send_atoms(&cmd_atoms, now_ms()).await?;
```

The USB session only needs a [`UsbBulkHost`](crates/embassy-bmd-atem/src/usb/io.rs)
implementation (async bulk read/write + clear-halt + delay). The desktop
implementation lives in `embassy-bmd-atem-std` (`nusb`); embedded backends
(e.g. `cotton-usb-host` on RP2040) wrap the same trait.

## Status

- [x] UDP/BURP transport (carried over from the archived `edge-bmd-atem`, tested)
- [x] USB 8.6+ framing, session FSM, flow-control windows —
      **verified against real hardware** (ATEM Mini `0x1EDB:0xBE49`):
      handshake, full initial state dump (continuous atom stream across
      chunks), command round-trips (`DCut` → `PrgI`/`TPrvI` responses),
      bulk-IN stall = idle-signal recovery, and the undocumented
      `0x05` → `0x06` heartbeat pair (see
      [`docs/PROTOCOL_USB.md`](docs/PROTOCOL_USB.md))
- [x] Cross-checked against `pyatem` on the same switcher with Linux `usbmon`:
      ZLP keepalive at ~510 ms and a 1-block flow window are the reference
      client's behaviour and are now this crate's defaults
- [x] Shared atom layer: `AtomWriter` (build once) + `link::frame_command`
      (wrap for either transport), with tests proving both framings carry
      byte-identical payloads
- [x] Desktop `nusb` backend + CLI (with `--raw-dump` protocol capture)
- [x] RP2040 firmware example (RTIC 2 + cotton-usb-host, `thumbv6m` CI build)
- [x] CI on every push/PR; tag-driven release workflow publishing all three
      crates to crates.io
- [ ] Joint hardware debug of the RP2040 firmware over SWD / defmt-RTT
      (needs a powered USB-C OTG hub so the switcher sees VBUS)
- [ ] `0x05` heartbeat cadence, and whether spontaneous switcher events
      need a poll (open questions in
      [`docs/PROTOCOL_USB.md`](docs/PROTOCOL_USB.md#open-questions))

### Migrating from `edge-bmd-atem`

That repository is **archived**; its UDP/BURP implementation lives on here as
`embassy_bmd_atem::udp` and every item is re-exported at the crate root, so the
move is a rename:

```toml
# before
edge_bmd_atem = { git = "https://github.com/MikanseiLaboratory/edge-bmd-atem", default-features = false }
# after
embassy-bmd-atem = { version = "0.1", default-features = false }
```

```rust
// before
use edge_bmd_atem::io::{ErrorType, UdpReceive, UdpReceiveBounded, UdpSend};
// after
use embassy_bmd_atem::{ErrorType, UdpReceive, UdpReceiveBounded, UdpSend};
```

Module paths: `io`/`packet`/`session`/`udp` → `embassy_bmd_atem::udp::{io, packet, session, mock}`.

### Releasing

Releases use crates.io **Trusted Publishing** — no long-lived token is stored
in the repository.

One-time setup, per crate on <https://crates.io/crates/>*name*/settings/trusted-publishing:

| field | value |
|---|---|
| Repository | `MikanseiLaboratory/embassy-bmd-atem` |
| Workflow file | `release.yml` |

Apply it to `bmd-atem-atom`, `embassy-bmd-atem` and `embassy-bmd-atem-std`.
The workflow then exchanges a GitHub OIDC token (`id-token: write`) for a
short-lived crates.io token via `rust-lang/crates-io-auth-action`, publishes the
three crates in dependency order and opens a GitHub release. The token is
revoked automatically when the job ends.

Cut a release by tagging `main`:

```sh
git tag v0.1.1 -m "v0.1.1"
git push origin v0.1.1
```

## License

Apache-2.0. Protocol facts were derived from the OpenSwitcher/pyatem
(LGPL-3.0) documentation and necromancer (Apache-2.0); no source code was
copied — see [`NOTICE`](NOTICE).
