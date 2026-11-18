//! RP2040 example: control an ATEM switcher plugged into the Pico's USB-C/USB-A
//! port using the `embassy-bmd-atem` USB 8.6+ host session.
//!
//! This is the S7 placeholder of `docs/PROTOCOL_USB.md`; the full
//! `cotton-usb-host`-based transport lands in the next commit. Target hardware:
//! the MikanseiLaboratory RP2040 + W5500 EVB (VBUS 5 V must be supplied to the
//! ATEM, see the README).
#![no_std]
#![no_main]
#![allow(unsafe_code)]

use core::alloc::{GlobalAlloc, Layout};
use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::ptr::{self, NonNull};

use cortex_m_rt::entry;
use critical_section::Mutex;
use linked_list_allocator::Heap;

const HEAP_SIZE: usize = 32 * 1024;

static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

static HEAP: Mutex<RefCell<Heap>> = Mutex::new(RefCell::new(Heap::empty()));

struct HostHeap;

unsafe impl GlobalAlloc for HostHeap {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        critical_section::with(|cs| {
            HEAP.borrow_ref_mut(cs)
                .allocate_first_fit(layout)
                .map(|n| n.as_ptr())
                .unwrap_or(ptr::null_mut())
        })
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if let Some(n) = NonNull::new(ptr) {
            critical_section::with(|cs| HEAP.borrow_ref_mut(cs).deallocate(n, layout));
        }
    }
}

#[global_allocator]
static ALLOCATOR: HostHeap = HostHeap;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

#[entry]
unsafe fn main() -> ! {
    critical_section::with(|cs| {
        let heap: &mut [MaybeUninit<u8>] = &mut *core::ptr::addr_of_mut!(HEAP_MEM);
        HEAP.borrow_ref_mut(cs).init_from_slice(heap);
    });

    // TODO(S7): drive `AtemUsbSession` here over a cotton-usb-host bulk transport.
    let _ = bmd_atem_atom::encode_atom_do_transition_cut(0);
    loop {
        cortex_m::asm::wfi();
    }
}
