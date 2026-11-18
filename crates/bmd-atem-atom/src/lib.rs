//! Blackmagic Design ATEM atom (command record) codec.
//!
//! An "atom" is the four-byte-command record used by every ATEM control transport
//! (UDP/BURP "high-level protocol", USB-C 8.6+ tunnel, TCP proxy): a big-endian
//! `u16` record length (≥ 8, including the header), two padding bytes, then the
//! 4-byte ASCII command name, then `length − 8` payload bytes.
//!
//! This crate is transport-independent; the UDP/BURP framing lives in
//! [`embassy_bmd_atem`](https://crates.io/crates/embassy-bmd-atem) `udp`, and the
//! USB-C 8.6+ framing in the same crate's `usb` module.
//!
//! Protocol layout notes are aligned with the
//! [necromancer](https://github.com/micolous/necromancer) project (Apache-2.0); see
//! the workspace `NOTICE` file.
#![no_std]
extern crate alloc;

mod atoms;
mod error;

pub use atoms::{
    AtomIter, FourCc, NextTransitionStyle, RawAtom, RawAtomSlice, encode_atom_change_preview_input,
    encode_atom_change_program_input, encode_atom_change_transition_next, encode_atom_do_ftb_auto,
    encode_atom_do_ftb_cut, encode_atom_do_transition_auto, encode_atom_do_transition_cut,
    parse_atoms,
};
pub use error::Error;
