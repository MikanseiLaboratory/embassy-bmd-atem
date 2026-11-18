//! RP2040 example: act as a USB host to a Blackmagic Design ATEM switcher
//! (USB-C control link, "8.6+" protocol).
//!
//! Wire the ATEM's USB port to the board's USB connector (an A-to-Micro-B
//! "OTG" cable/adapter on a Pico; the W5500-EVB-Pico exposes it directly)
//! and supply **5 V on VBUS** — the switcher enumerates as a USB *device*
//! and will not appear without host-side VBUS, even when externally powered.
//!
//! The firmware waits for the switcher to enumerate, claims its vendor-class
//! control interface (bulk OUT `0x02` / IN `0x82`), runs the
//! `embassy-bmd-atem` 8.6+ handshake with credit-based flow control, prints
//! the initial state dump over defmt/RTT, then sends one CUT command on ME1
//! as a demo and logs subsequent state updates.
//!
//! Program/debug over SWD (Pico Debug Probe: `probe-rs run --chip rp2040`
//! from this directory) — the board's USB port belongs to the ATEM.
//!
//! Based on the `rp2040-usb-msc` example of the
//! [cotton](https://github.com/pdh11/cotton) project (CC0-1.0).
#![no_std]
#![no_main]
#![allow(unsafe_code)]
#![allow(unsafe_op_in_unsafe_fn)]

use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::pin;

use bmd_atem_atom::encode_atom_do_transition_cut;
use cotton_usb_host::device::identify::IdentifyFromDescriptors;
use cotton_usb_host::host::rp2040::{Rp2040HostController, UsbShared, UsbStatics};
use cotton_usb_host::host_controller::{HostController, TransferType, UsbError};
use cotton_usb_host::usb_bus::{BulkIn, BulkOut, DeviceEvent, HubState, UsbBus};
use cotton_usb_host::wire::{ConfigurationDescriptor, DescriptorVisitor, InterfaceDescriptor};
use defmt_rtt as _;
use embassy_bmd_atem::usb::io::UsbBulkHost;
use embassy_bmd_atem::usb::session::{
    AtemUsbSession, SessionEvent, UsbSessionConfig, UsbSessionState,
};
use futures_util::StreamExt;
use panic_probe as _;
use rp_pico as _; // includes boot2
use rtic_monotonics::rp2040::prelude::*;
use static_cell::ConstStaticCell;

/// Blackmagic Design USB vendor id.
const USB_VID_BMD: u16 = 0x1EDB;
/// Preferred control bulk endpoints of the ATEM (pyatem layout).
const PREFER_EP_IN: u8 = 0x82;
const PREFER_EP_OUT: u8 = 0x02;
/// Upper bound for one announced atoms chunk (see docs/PROTOCOL_USB.md).
const MAX_ATOMS_LEN: usize = 0x4000;

const HEAP_SIZE: usize = 48 * 1024;

static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

static HEAP: critical_section::Mutex<core::cell::RefCell<linked_list_allocator::Heap>> =
    critical_section::Mutex::new(core::cell::RefCell::new(linked_list_allocator::Heap::empty()));

struct HostHeap;

unsafe impl core::alloc::GlobalAlloc for HostHeap {
    unsafe fn alloc(&self, layout: core::alloc::Layout) -> *mut u8 {
        critical_section::with(|cs| {
            HEAP.borrow_ref_mut(cs)
                .allocate_first_fit(layout)
                .map(|n| n.as_ptr())
                .unwrap_or(core::ptr::null_mut())
        })
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: core::alloc::Layout) {
        if let Some(n) = core::ptr::NonNull::new(ptr) {
            critical_section::with(|cs| HEAP.borrow_ref_mut(cs).deallocate(n, layout));
        }
    }
}

#[global_allocator]
static ALLOCATOR: HostHeap = HostHeap;

#[rtic::app(device = rp_pico::hal::pac, dispatchers = [ADC_IRQ_FIFO])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        shared: &'static UsbShared,
    }

    #[local]
    struct Local {
        resets: rp_pico::pac::RESETS,
        regs: Option<rp_pico::pac::USBCTRL_REGS>,
        dpram: Option<rp_pico::pac::USBCTRL_DPRAM>,
    }

    rp2040_timer_monotonic!(Mono); // 1 MHz

    #[init()]
    fn init(c: init::Context) -> (Shared, Local) {
        let device = c.device;
        let mut resets = device.RESETS;
        let mut watchdog = rp_pico::hal::watchdog::Watchdog::new(device.WATCHDOG);

        let _clocks = rp_pico::hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            device.XOSC,
            device.CLOCKS,
            device.PLL_SYS,
            device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        Mono::start(device.TIMER, &resets);

        // The heap must be live before any task allocates
        // (`embassy-bmd-atem` uses `alloc::Vec` for stream reassembly).
        critical_section::with(|cs| {
            let heap: &mut [MaybeUninit<u8>] = unsafe { &mut *core::ptr::addr_of_mut!(HEAP_MEM) };
            HEAP.borrow_ref_mut(cs).init_from_slice(heap);
        });

        usb_task::spawn().unwrap();

        static USB_SHARED: UsbShared = UsbShared::new();

        (
            Shared { shared: &USB_SHARED },
            Local { resets, regs: Some(device.USBCTRL_REGS), dpram: Some(device.USBCTRL_DPRAM) },
        )
    }

    pub(crate) fn rtic_delay(ms: usize) -> impl Future<Output = ()> {
        Mono::delay(<Mono as rtic_monotonics::Monotonic>::Duration::millis(ms as u64))
    }

    /// Millisecond clock for the ATEM session (wrapping `u32`).
    pub(crate) fn now_ms() -> u32 {
        // The RP2040 monotonic ticks at 1 MHz.
        (Mono::now().ticks() / 1000) as u32
    }

    #[task(local = [regs, dpram, resets], shared = [&shared], priority = 2)]
    async fn usb_task(cx: usb_task::Context) {
        static USB_STATICS: ConstStaticCell<UsbStatics> = ConstStaticCell::new(UsbStatics::new());
        let statics = USB_STATICS.take();

        let driver = Rp2040HostController::new(
            cx.local.resets,
            cx.local.regs.take().unwrap(),
            cx.local.dpram.take().unwrap(),
            cx.shared.shared,
            statics,
        );
        let hub_state = HubState::default();
        let stack = UsbBus::new(driver);

        let mut p = pin!(stack.device_events(&hub_state, rtic_delay));

        loop {
            match p.next().await {
                Some(DeviceEvent::EnumerationError(hub, port, e)) => {
                    defmt::warn!(
                        "enumeration error {} on hub {} port {} (check VBUS 5V and cabling)",
                        e,
                        hub,
                        port
                    );
                }
                Some(DeviceEvent::Connect(udevice, info)) => {
                    if info.vid != USB_VID_BMD {
                        defmt::info!(
                            "ignoring USB device vid=0x{:x} pid=0x{:x}",
                            info.vid,
                            info.pid
                        );
                        continue;
                    }
                    defmt::info!("BMD device connected: pid=0x{:x}", info.pid);

                    let mut ident = IdentifyAtem::default();
                    if stack.get_configuration(&udevice, &mut ident).await.is_err() {
                        defmt::warn!("could not read configuration descriptors");
                        continue;
                    }
                    let Some(config) = ident.identify() else {
                        defmt::warn!("no vendor-class bulk interface: not an ATEM control link");
                        continue;
                    };

                    let Ok(mut usb) = stack.configure(udevice, config).await else {
                        defmt::warn!("configure failed");
                        continue;
                    };

                    let in_ep = pick_ep(usb.in_endpoints(), PREFER_EP_IN);
                    let out_ep = pick_ep(usb.out_endpoints(), PREFER_EP_OUT);
                    let Ok(bulk_in) = usb.open_in_endpoint(in_ep) else {
                        defmt::warn!("open bulk IN 0x{:x} failed", in_ep);
                        continue;
                    };
                    let Ok(bulk_out) = usb.open_out_endpoint(out_ep) else {
                        defmt::warn!("open bulk OUT 0x{:x} failed", out_ep);
                        continue;
                    };

                    defmt::info!("ATEM control link ready (in=0x{:x} out=0x{:x})", in_ep, out_ep);
                    run_atem_session(&stack, bulk_in, bulk_out).await;
                }
                Some(DeviceEvent::Disconnect(_))
                | Some(DeviceEvent::HubConnect(_))
                | Some(DeviceEvent::None) => {}
                None => break,
            }
        }
    }

    #[task(binds = USBCTRL_IRQ, shared = [&shared], priority = 2)]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.shared.shared.on_irq();
    }
}

/// Find an endpoint in a `BitSet`, preferring `prefer` (matched as a raw
/// address such as `0x82` or a plain endpoint number such as `2`).
fn pick_ep(set: cotton_usb_host::bitset::BitSet, prefer: u8) -> u8 {
    set.iter()
        .find(|&e| e == prefer || e == (prefer & 0x7f))
        .unwrap_or_else(|| set.iter().next().unwrap_or_default())
}

/// Record the configuration containing the ATEM's vendor-class control
/// interface.
#[derive(Default)]
struct IdentifyAtem {
    current: Option<u8>,
    vendor: Option<u8>,
}

impl DescriptorVisitor for IdentifyAtem {
    fn on_configuration(&mut self, c: &ConfigurationDescriptor) {
        self.current = Some(c.bConfigurationValue);
    }

    fn on_interface(&mut self, i: &InterfaceDescriptor) {
        if i.bInterfaceClass == 0xFF {
            self.vendor = self.current;
        }
    }
}

impl IdentifyFromDescriptors for IdentifyAtem {
    fn identify(&self) -> Option<u8> {
        self.vendor
    }
}

/// [`UsbBulkHost`] adapter over cotton-usb-host's bulk endpoint pair.
struct CottonBulk<'a, HC: HostController> {
    bus: &'a UsbBus<HC>,
    bulk_in: BulkIn,
    bulk_out: BulkOut,
}

impl<HC: HostController> core::fmt::Debug for CottonBulk<'_, HC> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CottonBulk").finish_non_exhaustive()
    }
}

/// Wrapper satisfying the trait's `Debug + Display` bounds on `no_std`
/// (cotton's `UsbError` only implements defmt's `Format` there).
#[derive(Clone, Copy, PartialEq)]
struct CottonErr(UsbError);

impl core::fmt::Debug for CottonErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "UsbError")
    }
}

impl core::fmt::Display for CottonErr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "usb error")
    }
}

impl<HC: HostController> UsbBulkHost for CottonBulk<'_, HC> {
    type Error = CottonErr;

    async fn read_bulk(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        match self.bus.bulk_in_transfer(&self.bulk_in, buf, TransferType::VariableSize).await {
            Ok(n) => Ok(n),
            Err(UsbError::Stall) => {
                // The switcher STALLs the IN endpoint when it has nothing more
                // to send (idle signal, see docs/PROTOCOL_USB.md). Clear the
                // halt and report an empty read; the session keeps running.
                if self.bus.clear_halt(&self.bulk_in).await.is_ok() {
                    defmt::debug!("bulk IN idle-stall (halt cleared)");
                    Ok(0)
                } else {
                    Err(CottonErr(UsbError::Stall))
                }
            }
            Err(e) => Err(CottonErr(e)),
        }
    }

    async fn write_bulk(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        let n = self
            .bus
            .bulk_out_transfer(&self.bulk_out, data, TransferType::VariableSize)
            .await
            .map_err(CottonErr)?;
        if n != data.len() {
            defmt::warn!("short bulk OUT write: {}/{}", n, data.len());
        }
        Ok(())
    }

    async fn reset_state(&mut self) -> Result<(), Self::Error> {
        // cotton-usb-host exposes clear_halt for BulkIn only; an OUT halt
        // would need a device re-enumeration (the switcher only stalls OUT on
        // protocol violations, which the session avoids).
        self.bus.clear_halt(&self.bulk_in).await.map_err(CottonErr)?;
        Ok(())
    }

    async fn delay_ms(&mut self, ms: u32) {
        app::rtic_delay(ms as usize).await;
    }
}

/// Run the ATEM 8.6+ session until the device disappears.
///
/// After the initial state dump, one CUT command is sent on ME1 as a demo;
/// afterwards the firmware logs state updates (and the `0x05`/`0x06`
/// heartbeat can be probed from here later).
async fn run_atem_session<HC: HostController>(
    stack: &UsbBus<HC>,
    bulk_in: BulkIn,
    bulk_out: BulkOut,
) {
    static RX_BUF: ConstStaticCell<[u8; MAX_ATOMS_LEN]> =
        ConstStaticCell::new([0u8; MAX_ATOMS_LEN]);
    let buf = RX_BUF.take();

    let backend = CottonBulk { bus: stack, bulk_in, bulk_out };
    let mut session = AtemUsbSession::new(
        backend,
        UsbSessionConfig { read_chunk: 256, ..UsbSessionConfig::default() },
    );

    if let Err(_e) = session.connect(app::now_ms()).await {
        defmt::error!("connect failed (backend/protocol error; see log above)");
        return;
    }
    defmt::info!("0x01 init sent (00 01 10 92), waiting for switcher…");

    let mut atoms_blocks = 0u32;
    let mut atoms_bytes = 0u32;
    let mut cut_sent = false;

    loop {
        match session.poll(app::now_ms(), buf).await {
            Ok(Some(SessionEvent::InitAck)) => {
                defmt::info!("0x02 init ack received");
            }
            Ok(Some(SessionEvent::Atoms { block, first })) => {
                atoms_blocks += 1;
                atoms_bytes += block.len() as u32;
                let mut names = [[0u8; 4]; 8];
                let mut count = 0usize;
                for atom in bmd_atem_atom::AtomIter::new(block).flatten() {
                    if count < names.len() {
                        names[count] = atom.name.0;
                    }
                    count += 1;
                }
                defmt::info!(
                    "{} atoms block {}: {} B (total {} B), {} fields, first {:a}",
                    if first { "first" } else { "next" },
                    atoms_blocks,
                    block.len(),
                    atoms_bytes,
                    count,
                    &names[..count.min(names.len())]
                );
                // Demo command once the first atoms have landed: CUT on ME1.
                if !cut_sent && count > 0 {
                    cut_sent = true;
                    app::rtic_delay(3000).await;
                    let cut = encode_atom_do_transition_cut(0);
                    match session.send_atoms(&cut, app::now_ms()).await {
                        Ok(()) => defmt::info!("sent DCut (CUT) on ME1"),
                        Err(_) => defmt::error!("send DCut failed"),
                    }
                }
            }
            Ok(Some(SessionEvent::FlowCtrl { blocks })) => {
                defmt::info!("0x03 flow: switcher announced {} block(s)", blocks);
            }
            Ok(Some(SessionEvent::Unknown { pkt_type })) => {
                // 0x06 = heartbeat response to a host 0x05 probe.
                defmt::info!("0x{:x}? device frame (0x06 = heartbeat response)", pkt_type);
            }
            Ok(None) => {}
            Err(_) => {
                defmt::error!("session error — waiting for re-enumeration");
                return;
            }
        }
        let _ = session.state() == UsbSessionState::Running;
    }
}
