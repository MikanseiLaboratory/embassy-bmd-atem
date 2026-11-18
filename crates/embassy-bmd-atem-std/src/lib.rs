//! Desktop (std / Tokio) host backend for the Blackmagic Design ATEM USB-C
//! control link, built on [`nusb`].
//!
//! Implements [`UsbBulkHost`](embassy_bmd_atem::UsbBulkHost) over one claimed
//! bulk endpoint pair of the switcher's vendor-specific control interface
//! (bulk OUT `0x02` / bulk IN `0x82` on stock ATEM Minis; otherwise discovered
//! from the interface descriptors).
//!
//! # Driver notes
//!
//! - **Linux**: needs write access to `/dev/bus/usb/**` (udev rules). Kernel drivers for
//!   the composite's other interfaces (UVC / UAC) stay bound; only the control interface
//!   is claimed (with kernel-driver detach as a fallback).
//! - **Windows**: the vendor control interface must be bound to **WinUSB** for `nusb`
//!   access. If Blackmagic's `switchers.inf` / `blackmagicusbio.inf` driver is installed,
//!   switch only the control interface (e.g. with Zadig) — this stops ATEM Software
//!   Control from using that interface until it is reverted.
//! - **macOS**: works out of the box; a configuration is set automatically when the
//!   device is unconfigured (vendor-class composite).

use std::io;
use std::io::Write as _;
use std::time::Duration;

use embassy_bmd_atem::usb::io::UsbBulkHost;
use nusb::MaybeFuture as _;
use nusb::descriptors::{InterfaceDescriptor, TransferType};
use nusb::transfer::{Buffer, Bulk, In, Out, TransferError};

/// Blackmagic Design USB vendor id.
pub const USB_VID_BLACKMAGIC: u16 = 0x1EDB;

/// Known ATEM switcher USB product ids (see `NOTICE` and `docs/PROTOCOL_USB.md`).
///
/// `0xBE49` ATEM Mini, `0xBE55` ATEM Mini Pro, `0xBE56` ATEM Mini Pro ISO,
/// `0xBE7C` ATEM Mini Extreme, `0xBE83` ATEM Mini Extreme ISO,
/// `0xBE9E` ATEM Television Studio HD8 ISO.
pub const KNOWN_PIDS: &[u16] = &[0xBE49, 0xBE55, 0xBE56, 0xBE7C, 0xBE83, 0xBE9E];

/// Errors from the desktop USB backend.
#[derive(Debug)]
pub enum HostError {
    /// `nusb` non-transfer failure (open / claim / clear-halt).
    Nusb(nusb::Error),
    /// Bulk transfer failure (Stall, Disconnected, ...).
    Transfer(nusb::transfer::TransferError),
    /// No suitable vendor-class bulk endpoint pair was found.
    NoControlInterface,
}

impl core::fmt::Display for HostError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Nusb(e) => write!(f, "usb error: {e:?}"),
            Self::Transfer(e) => write!(f, "usb transfer error: {e}"),
            Self::NoControlInterface => write!(
                f,
                "no ATEM control interface (vendor-class bulk pair) found; pass --iface/--out-ep/--in-ep to override"
            ),
        }
    }
}

impl std::error::Error for HostError {}

/// A claimed control interface with exclusive access to its bulk endpoint pair.
#[derive(Debug)]
pub struct NusbBulkHost {
    ep_in: nusb::Endpoint<Bulk, In>,
    ep_out: nusb::Endpoint<Bulk, Out>,
    _interface: nusb::Interface,
    /// Optional capture sink: every bulk transfer is appended as one hex line
    /// (`>` host→device, `<` device→host) for protocol analysis.
    raw_log: Option<std::fs::File>,
}

impl NusbBulkHost {
    /// Endpoint addresses `(OUT, IN)` used by the control link.
    #[must_use]
    pub fn endpoints(&self) -> (u8, u8) {
        (self.ep_out.endpoint_address(), self.ep_in.endpoint_address())
    }

    /// Maximum packet size of the IN endpoint (full-speed bulk = 64 B).
    #[must_use]
    pub fn in_max_packet_size(&self) -> usize {
        self.ep_in.max_packet_size()
    }

    /// Attach a raw-transfer capture file (one hex line per transfer).
    pub fn set_raw_log(&mut self, file: Option<std::fs::File>) {
        self.raw_log = file;
    }

    fn log_raw(&mut self, dir: char, data: &[u8]) {
        if let Some(f) = self.raw_log.as_mut() {
            let hex: String = data.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join("");
            let _ = writeln!(f, "{dir}{hex}");
        }
    }
}

/// List connected Blackmagic Design ATEM USB devices, optionally filtered by PID.
pub fn list_devices(pid: Option<u16>) -> Result<Vec<nusb::DeviceInfo>, HostError> {
    let devices = nusb::list_devices().wait().map_err(HostError::Nusb)?;
    Ok(devices
        .filter(|d| d.vendor_id() == USB_VID_BLACKMAGIC && pid.is_none_or(|p| d.product_id() == p))
        .collect())
}

/// Open the control interface of a discovered device and return a bulk host handle.
///
/// With `interface` unset the control interface is auto-detected: every claimable
/// interface is scanned and the one carrying the classic ATEM pair (bulk OUT
/// `0x02` / bulk IN `0x82`) is preferred; otherwise the first interface with any
/// bulk pair is used.
///
/// # Errors
/// [`HostError::NoControlInterface`] when no candidate is found, or backend failures.
pub async fn open_device(
    device_info: nusb::DeviceInfo,
    interface: Option<u8>,
    out_ep: Option<u8>,
    in_ep: Option<u8>,
) -> Result<NusbBulkHost, HostError> {
    let device = device_info.open().await.map_err(HostError::Nusb)?;
    // macOS only sets a configuration automatically for composite-class devices;
    // vendor-class composites may need it explicitly. Not supported on Windows.
    #[cfg(not(target_os = "windows"))]
    if device.active_configuration().is_err() {
        let _ = device.set_configuration(1).await;
    }

    let explicit = interface.is_some();
    let scan = match interface {
        Some(iface) => iface..=iface,
        None => 0u8..=8u8,
    };

    // Pass 1: find every claimable interface with a bulk pair, ranking the
    // classic ATEM control pair (0x02/0x82) first. Claims are released by drop.
    let mut best: Option<(i32, u8, u8, u8)> = None; // (rank, iface, out, in)
    for iface in scan {
        let Some((out_addr, in_addr)) = scan_interface(&device, iface, out_ep, in_ep).await else {
            continue;
        };
        let rank = i32::from(out_addr == 0x02 && in_addr == 0x82);
        if explicit {
            best = Some((rank, iface, out_addr, in_addr));
            break;
        }
        if best.is_none_or(|(best_rank, ..)| rank > best_rank) {
            best = Some((rank, iface, out_addr, in_addr));
        }
    }
    let Some((_, iface, out_addr, in_addr)) = best else {
        return Err(HostError::NoControlInterface);
    };

    // Pass 2: re-claim the chosen interface and open its endpoints.
    let interface_handle = claim_interface(&device, iface).await?;
    let ep_out = interface_handle.endpoint::<Bulk, Out>(out_addr).map_err(HostError::Nusb)?;
    let ep_in = interface_handle.endpoint::<Bulk, In>(in_addr).map_err(HostError::Nusb)?;
    Ok(NusbBulkHost { ep_in, ep_out, _interface: interface_handle, raw_log: None })
}

/// Claim an interface. On Linux a kernel driver may hold it (usbfs); detach
/// and retry once in that case.
async fn claim_interface(device: &nusb::Device, iface: u8) -> Result<nusb::Interface, HostError> {
    if let Ok(handle) = device.claim_interface(iface).await {
        return Ok(handle);
    }
    #[cfg(target_os = "linux")]
    {
        device.detach_and_claim_interface(iface).await.map_err(HostError::Nusb)
    }
    #[cfg(not(target_os = "linux"))]
    {
        // Re-run the claim to surface its original error.
        device.claim_interface(iface).await.map_err(HostError::Nusb)
    }
}

/// Claim `iface` and return its bulk endpoint pair (hints honoured when given).
async fn scan_interface(
    device: &nusb::Device,
    iface: u8,
    out_hint: Option<u8>,
    in_hint: Option<u8>,
) -> Option<(u8, u8)> {
    let handle = claim_interface(device, iface).await.ok()?;
    let desc = handle.descriptor()?;
    let pair = pick_bulk_pair(&desc, out_hint, in_hint);
    drop(handle); // release the claim
    pair
}

/// Pick the bulk OUT/IN pair from an interface descriptor, honouring explicit hints.
fn pick_bulk_pair(
    desc: &InterfaceDescriptor<'_>,
    out_hint: Option<u8>,
    in_hint: Option<u8>,
) -> Option<(u8, u8)> {
    if let (Some(out_addr), Some(in_addr)) = (out_hint, in_hint) {
        return Some((out_addr, in_addr));
    }
    let mut bulk_out = None;
    let mut bulk_in = None;
    for ep in desc.endpoints() {
        if ep.transfer_type() != TransferType::Bulk {
            continue;
        }
        let address = ep.address();
        if address & 0x80 != 0 {
            if in_hint == Some(address) {
                bulk_in = Some(address);
                break;
            }
            // Prefer the 0x8x2 control pair seen on ATEMs, otherwise keep the lowest.
            if bulk_in.is_none() || address == 0x82 {
                bulk_in = Some(address);
            }
        } else {
            if out_hint == Some(address) {
                bulk_out = Some(address);
                break;
            }
            if bulk_out.is_none() || address == 0x02 {
                bulk_out = Some(address);
            }
        }
    }
    match (out_hint.or(bulk_out), in_hint.or(bulk_in)) {
        (Some(out_addr), Some(in_addr)) => Some((out_addr, in_addr)),
        _ => None,
    }
}

impl UsbBulkHost for NusbBulkHost {
    type Error = HostError;

    async fn read_bulk(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.is_empty() {
            return Ok(0);
        }
        let mps = self.ep_in.max_packet_size().max(1);
        let requested = buf.len().next_multiple_of(mps);
        self.ep_in.submit(Buffer::new(requested));
        let completion = self.ep_in.next_complete().await;
        if completion.status == Err(TransferError::Stall) {
            // Device signalled a protocol violation; clear the halt and report
            // an empty read so the session can attempt recovery.
            self.ep_in.clear_halt().await.map_err(HostError::Nusb)?;
            eprintln!("[usb] bulk IN stalled (cleared halt)");
            return Ok(0);
        }
        let result = completion.status;
        if result.is_ok() {
            let n = completion.actual_len.min(buf.len());
            buf[..n].copy_from_slice(&completion.buffer[..n]);
            self.log_raw('<', &buf[..n]);
            Ok(n)
        } else {
            Err(HostError::Transfer(result.expect_err("checked above")))
        }
    }

    async fn write_bulk(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.log_raw('>', data);
        self.ep_out.submit(Buffer::from(data));
        let completion = self.ep_out.next_complete().await;
        if completion.status == Err(TransferError::Stall) {
            self.ep_out.clear_halt().await.map_err(HostError::Nusb)?;
            eprintln!("[usb] bulk OUT stalled (cleared halt)");
            return Err(HostError::Transfer(TransferError::Stall));
        }
        completion.status.map_err(HostError::Transfer)
    }

    async fn reset_state(&mut self) -> Result<(), Self::Error> {
        self.ep_out.clear_halt().await.map_err(HostError::Nusb)?;
        self.ep_in.clear_halt().await.map_err(HostError::Nusb)?;
        Ok(())
    }

    async fn delay_ms(&mut self, ms: u32) {
        tokio::time::sleep(Duration::from_millis(u64::from(ms))).await;
    }
}

/// Load a replay file: one bulk transfer per line, arbitrary hex with any separators
/// (e.g. `tshark -T fields -e usb.capdata` output). Blank lines and comments (`#`) skipped.
pub fn load_replay_file(path: &std::path::Path) -> io::Result<Vec<Vec<u8>>> {
    let content = std::fs::read_to_string(path)?;
    Ok(load_replay_str(&content))
}

/// See [`load_replay_file`].
#[must_use]
pub fn load_replay_str(content: &str) -> Vec<Vec<u8>> {
    content
        .lines()
        .filter(|line| !line.trim().is_empty() && !line.trim_start().starts_with('#'))
        .map(parse_hex_loose)
        .filter(|bytes| !bytes.is_empty())
        .collect()
}

/// Decode a hex string with arbitrary separators; odd trailing nibbles are dropped.
#[must_use]
pub fn parse_hex_loose(line: &str) -> Vec<u8> {
    let mut nibbles = Vec::with_capacity(line.len() / 2);
    for ch in line.chars() {
        match ch {
            '0'..='9' | 'a'..='f' | 'A'..='F' => {
                nibbles.push(ch.to_digit(16).unwrap_or(0) as u8);
            }
            _ => continue,
        }
    }
    nibbles.as_chunks::<2>().0.iter().map(|pair| (pair[0] << 4) | pair[1]).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_hex_loose_variants() {
        assert_eq!(parse_hex_loose("00:01:00:01"), vec![0x00, 0x01, 0x00, 0x01]);
        assert_eq!(parse_hex_loose("0001 0001"), vec![0, 1, 0, 1]);
        assert_eq!(parse_hex_loose("!"), Vec::<u8>::new());
        assert_eq!(parse_hex_loose("00-01"), vec![0x00, 0x01]);
        assert_eq!(parse_hex_loose("010"), vec![0x01]); // odd trailing nibble dropped
    }

    #[test]
    fn load_replay_skips_empty_lines() {
        let out = load_replay_str("\n02 00 01 00\n# noise\n\n04 00 01 00 0c 00\n");
        assert_eq!(out, vec![vec![2, 0, 1, 0], vec![4, 0, 1, 0, 0x0c, 0]]);
    }
}
