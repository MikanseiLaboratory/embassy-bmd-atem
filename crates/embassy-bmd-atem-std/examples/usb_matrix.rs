//! Clean-room USB probe matrix for the ATEM 8.6+ link. Runs the whole
//! verification sequence sequentially with explicit pacing — no mid-read
//! cancellation, no interleaved timers.
//!
//! ```text
//! cargo run -p embassy-bmd-atem-std --example usb_matrix
//! ```
use std::time::{Duration, Instant};

use embassy_bmd_atem::usb::session::{
    AtemUsbSession, SessionEvent, UsbSessionConfig, UsbSessionState,
};
use embassy_bmd_atem_std as backend;

fn main() {
    let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build().unwrap();
    rt.block_on(run());
}

async fn run() {
    let devices = backend::list_devices(None).expect("list");
    let Some(device) = devices.first().cloned() else {
        eprintln!("no ATEM connected");
        return;
    };
    let host = backend::open_device(device, None, None, None).await.expect("open");
    let mut session = AtemUsbSession::new(host, UsbSessionConfig::default());
    let t0 = Instant::now();
    let now = || (t0.elapsed().as_millis() & 0xFFFF_FFFF) as u32;
    let mut buf = vec![0u8; 0x1_0000];

    println!("== P1: connect + initial dump ==");
    session.connect(now()).await.expect("connect");
    let deadline = Instant::now() + Duration::from_secs(8);
    while session.state() != UsbSessionState::Running && Instant::now() < deadline {
        match session.poll(now(), &mut buf).await {
            Ok(Some(SessionEvent::InitAck)) => println!("[t] 0x02 ack"),
            Ok(Some(SessionEvent::Atoms { block, first })) => {
                println!("[t] atoms{}: {} B", if first { "(first)" } else { "" }, block.len());
            }
            Ok(Some(SessionEvent::FlowCtrl { blocks })) => println!("[t] 0x03 dev: {blocks} blk"),
            Ok(_) => {}
            Err(e) => println!("[t] poll err: {e}"),
        }
    }
    println!("state: {:?}", session.state());

    println!("== P2: idle (expect IN stall -> credit pause) ==");
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        match session.poll(now(), &mut buf).await {
            Ok(None) => {
                println!("[idle] read empty (stall={})", session_is_stalled(&mut session));
                break;
            }
            Ok(Some(ev)) => println!("[idle] ev: {ev:?}"),
            Err(e) => println!("[idle] err: {e}"),
        }
    }

    println!("== P3: TiRq probe x4 (2s spacing) ==");
    for i in 1..=4 {
        println!("[tirq {i}] send…");
        match session.poll_ping(now()).await {
            Ok(()) => println!("[tirq {i}] announce+atoms+credit OK"),
            Err(e) => println!("[tirq {i}] FAILED: {e}"),
        }
        // Wait up to 2s for a response (stall ticks return early).
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match session.poll(now(), &mut buf).await {
                Ok(Some(SessionEvent::Atoms { block, .. })) => {
                    let mut names = Vec::new();
                    for atom in bmd_atem_atom::AtomIter::new(block).flatten() {
                        names.push(atom.name.display_lossy().to_string());
                    }
                    println!("[tirq {i}] RESP: {} B atoms {:?}", block.len(), names);
                }
                Ok(Some(ev)) => println!("[tirq {i}] ev: {ev:?}"),
                Ok(None) => {
                    println!("[tirq {i}] idle-stall (no response)");
                    break;
                }
                Err(e) => {
                    println!("[tirq {i}] poll err: {e}");
                    break;
                }
            }
        }
        backend_sleep(2000).await;
    }

    println!("== P4: BUTTON WATCH 25s — press ATEM buttons now! ==");
    let deadline = Instant::now() + Duration::from_secs(25);
    let mut next_ping = Instant::now() + Duration::from_secs(2);
    while Instant::now() < deadline {
        if Instant::now() >= next_ping {
            let _ = session.poll_ping(now()).await;
            println!("[watch] ping");
            next_ping += Duration::from_secs(2);
        }
        match session.poll(now(), &mut buf).await {
            Ok(Some(SessionEvent::Atoms { block, .. })) => {
                let mut names = Vec::new();
                for atom in bmd_atem_atom::AtomIter::new(block).flatten() {
                    names.push(atom.name.display_lossy().to_string());
                }
                println!("[watch] EVENT: {} B atoms {:?}", block.len(), names);
            }
            Ok(Some(ev)) => println!("[watch] ev: {ev:?}"),
            Ok(None) => {}
            Err(e) => println!("[watch] err: {e}"),
        }
    }

    println!("== P5: command battery ==");
    for cmd in [bmd_atem_atom::encode_atom_change_program_input(0, 1)] {
        match session.send_atoms(&cmd, now()).await {
            Ok(()) => println!("[cmd] sent {} B", cmd.len()),
            Err(e) => println!("[cmd] FAILED: {e}"),
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            match session.poll(now(), &mut buf).await {
                Ok(Some(SessionEvent::Atoms { block, .. })) => {
                    let mut names = Vec::new();
                    for atom in bmd_atem_atom::AtomIter::new(block).flatten() {
                        names.push(atom.name.display_lossy().to_string());
                    }
                    println!("[cmd] RESP: {} B atoms {:?}", block.len(), names);
                    break;
                }
                Ok(Some(ev)) => println!("[cmd] ev: {ev:?}"),
                Ok(None) => {
                    println!("[cmd] idle-stall (no response)");
                    break;
                }
                Err(e) => {
                    println!("[cmd] poll err: {e}");
                    break;
                }
            }
        }
    }
    println!("== done ==");
}

fn session_is_stalled<H: embassy_bmd_atem::UsbBulkHost>(_s: &mut AtemUsbSession<H>) -> bool {
    // The idle-stall is signalled through Ok(None) reads; nothing extra to query.
    true
}

async fn backend_sleep(ms: u64) {
    tokio::time::sleep(Duration::from_millis(ms)).await;
}
