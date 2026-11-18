//! Diagnostic: dump the USB interface/endpoint layout of connected Blackmagic
//! Design devices and probe which interfaces can be claimed (i.e. are bound to
//! WinUSB on Windows). Read-only; does not submit any transfer.
//!
//! ```text
//! cargo run -p embassy-bmd-atem-std --example usb_probe
//! cargo run -p embassy-bmd-atem-std --example usb_probe -- --all
//! ```
use std::time::Duration;

use clap::Parser;
use nusb::MaybeFuture as _;

#[derive(Parser, Debug)]
#[command(
    name = "usb_probe",
    version,
    about = "Dump BMD USB device interface layout / claimability"
)]
struct Cli {
    /// Probe every USB device, not just Blackmagic (VID 0x1EDB).
    #[arg(long)]
    all: bool,
}

fn main() {
    let cli = Cli::parse();
    let devices = nusb::list_devices().wait().expect("list_devices");
    let mut found = 0;
    for info in devices.filter(|d| cli.all || d.vendor_id() == 0x1EDB) {
        found += 1;
        println!(
            "device vid=0x{:04x} pid=0x{:04x} addr={} class={:?} serial={:?}",
            info.vendor_id(),
            info.product_id(),
            info.device_address(),
            info.class(),
            info.serial_number()
        );
        let Ok(device) = info.open().wait() else {
            println!("  open: FAILED (not WinUSB-bound on Windows / no permission on Linux)");
            continue;
        };
        println!("  open: OK");
        let Ok(config) = device.active_configuration() else {
            println!("  active_configuration: none (unconfigured device)");
            continue;
        };
        for iface in config.interfaces() {
            for desc in iface.alt_settings() {
                println!(
                    "  if{} alt{} class=0x{:02x}/0x{:02x}/0x{:02x} endpoints={}",
                    iface.interface_number(),
                    desc.alternate_setting(),
                    desc.class(),
                    desc.subclass(),
                    desc.protocol(),
                    desc.num_endpoints()
                );
                for ep in desc.endpoints() {
                    let is_bulk = ep.transfer_type() == nusb::descriptors::TransferType::Bulk;
                    println!(
                        "    ep 0x{:02x} {} mps={}",
                        ep.address(),
                        if is_bulk { "bulk" } else { "other" },
                        ep.max_packet_size()
                    );
                }
                let bulk = desc
                    .endpoints()
                    .any(|ep| ep.transfer_type() == nusb::descriptors::TransferType::Bulk);
                if !bulk {
                    continue;
                }
                match device.claim_interface(iface.interface_number()).wait() {
                    Ok(_handle) => println!(
                        "    claim if{}: OK (claimable → WinUSB/driverless, usable by nusb)",
                        iface.interface_number()
                    ),
                    Err(e) => println!(
                        "    claim if{}: FAILED ({e:?}) — bound to a kernel/vendor driver",
                        iface.interface_number()
                    ),
                }
            }
        }
    }
    if found == 0 {
        println!(
            "no matching USB devices{}",
            if cli.all { "" } else { " (VID 0x1EDB); try --all" }
        );
    }
    let _ = Duration::from_secs(0);
}
