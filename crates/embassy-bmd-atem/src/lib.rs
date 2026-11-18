//! Blackmagic Design ATEM control protocol for embedded Rust.
//!
//! Two transports, one atom layer:
//!
//! - **UDP / BURP** ([`udp`]): the LAN protocol on port 9910. 12-byte packet header
//!   (flags / session id / packet ids) plus connect handshake, ACK queue and
//!   retransmission helpers, aligned with [`edge-nal`] stacks.
//! - **USB-C 8.6+** ([`usb`]): the host side of the switcher's USB control link
//!   (vendor-class bulk endpoints on the switcher). 4-byte type headers
//!   (`0x01` init / `0x02` ack / `0x03` flow control / `0x04` data announce / `0x05`)
//!   around a stream of atom records. See [`usb`] and `docs/PROTOCOL_USB.md`.
//!
//! The common atom record codec lives in the separate
//! [`bmd-atem-atom`](https://crates.io/crates/bmd-atem-atom) crate and is re-exported here.
//!
//! The crate is `#![no_std]` + `alloc`. Enable the `std` feature for the Tokio-based
//! unit tests and examples.
//!
//! [`edge-nal`]: https://docs.rs/edge-nal
#![cfg_attr(not(feature = "std"), no_std)]
extern crate alloc;

pub mod error;
pub mod udp;
pub mod usb;

pub use bmd_atem_atom::{
    encode_atom_change_preview_input, encode_atom_change_program_input,
    encode_atom_change_transition_next, encode_atom_do_ftb_auto, encode_atom_do_ftb_cut,
    encode_atom_do_transition_auto, encode_atom_do_transition_cut, parse_atoms, AtomIter, FourCc,
    NextTransitionStyle, RawAtom, RawAtomSlice,
};
pub use error::Error;

pub use udp::{
    AtemControl, AtemPacket, AtemPacketFlags, AtemPacketPayload, AtemSession, EdgeNalUdp, MockUdp,
    MockUdpExhausted, PendingPacket, SessionConfig, SessionError, SessionState, UdpReceive,
    UdpReceiveBounded, UdpSend, ATEM_UDP_PORT,
};

pub use usb::{
    build_data_host, build_flow_host, build_init, AtemUsbSession, DataLenLayout, FrameError,
    SessionEvent, StreamParser, UsbBulkHost, UsbSessionConfig, UsbSessionError, UsbSessionState,
};
