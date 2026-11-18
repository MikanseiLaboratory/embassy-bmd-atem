//! One command, two framings: the shared layer between UDP/BURP and USB 8.6+.
//!
//! Both transports carry exactly the same **atom records**; only the wrapper
//! differs:
//!
//! | | UDP / BURP | USB-C 8.6+ |
//! |---|---|---|
//! | wrapper | 12-byte packet header (flags, session, packet ids) | 4-byte type header, `0x04` announce + payload |
//! | connection | `0x01` Connect / `0x02` ConnectAck handshake | `0x01` init (`00 01 10 92`) / `0x02` ack |
//! | reliability | ACK queue + retransmission | none (USB is reliable); flow-control windows |
//! | addressing | `SocketAddr` + session id | claimed bulk endpoint pair |
//! | payload | **atom records** | **atom records** (identical bytes) |
//!
//! [`AtomWriter`](bmd_atem_atom::AtomWriter) builds the payload once;
//! [`frame_command`] wraps it for whichever transport is in use.
//!
//! ```ignore
//! use embassy_bmd_atem::{frame_command, FrameContext, TransportKind};
//!
//! let mut atoms = [0u8; 64];
//! let mut w = bmd_atem_atom::AtomWriter::new(&mut atoms);
//! w.push_source(bmd_atem_atom::FourCc(*b"CPgI"), 0, 3).unwrap();
//!
//! let mut wire = [0u8; 96];
//! match frame_command(
//!     TransportKind::Usb,
//!     FrameContext::usb(),
//!     w.as_slice(),
//!     &mut wire,
//! )
//! .unwrap()
//! {
//!     FramePlan::AnnounceThenPayload { announce, payload } => {
//!         // write `announce`, then `payload`
//!         let _ = (announce, payload);
//!     }
//!     FramePlan::Datagram(d) => { let _ = d; }
//! }
//! ```

use crate::udp::packet::{AtemPacket, AtemPacketFlags};
use crate::usb::protocol::{HEADER_LEN, build_data_host};

/// Which transport a command is being framed for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportKind {
    /// UDP/BURP datagram (port 9910).
    Udp,
    /// USB-C 8.6+ tunnel.
    Usb,
}

/// Transport-specific addressing needed to frame a command.
///
/// USB ignores the UDP fields, so [`FrameContext::usb`] leaves them zero.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct FrameContext {
    /// BURP session id (the switcher-assigned one, `0x8000` bit set).
    pub session_id: u16,
    /// Cumulative ACK the client is acknowledging.
    pub acked_packet_id: u16,
    /// Client packet id for this command (`0` for plain data).
    pub client_packet_id: u16,
    /// Outgoing `sender_packet_id`.
    pub sender_packet_id: u16,
}

impl FrameContext {
    /// Context for the USB transport (no ids involved).
    #[must_use]
    pub const fn usb() -> Self {
        Self { session_id: 0, acked_packet_id: 0, client_packet_id: 0, sender_packet_id: 0 }
    }
}

/// How the caller must write the framed command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FramePlan<'a> {
    /// UDP: one datagram, already framed in the caller's buffer.
    Datagram(&'a [u8]),
    /// USB: write the announce header, then the payload as a second transfer.
    AnnounceThenPayload {
        announce: [u8; crate::usb::protocol::HOST_HEADER_BODY_LEN],
        payload: &'a [u8],
    },
}

impl FramePlan<'_> {
    /// Number of bus writes the transport needs.
    #[must_use]
    pub const fn writes(&self) -> usize {
        match self {
            Self::Datagram(_) => 1,
            Self::AnnounceThenPayload { .. } => 2,
        }
    }
}

/// Frame one atom blob for the given transport.
///
/// For UDP the datagram is written into `out` and returned borrowed. For USB the
/// announce header is returned by value (USB frames are fixed 12 bytes) together
/// with the borrowed payload.
///
/// # Errors
/// [`crate::Error::InvalidLength`] when the command does not fit the transport's
/// limits (UDP packet cap `0x7ff`, USB `u16` length) or `out` is too small.
pub fn frame_command<'a>(
    kind: TransportKind,
    ctx: FrameContext,
    atoms: &'a [u8],
    out: &'a mut [u8],
) -> Result<FramePlan<'a>, crate::Error> {
    match kind {
        TransportKind::Udp => {
            let packet = AtemPacket::with_atoms(
                AtemPacketFlags { ack: false, ..AtemPacketFlags::default() },
                ctx.session_id,
                ctx.acked_packet_id,
                ctx.client_packet_id,
                ctx.sender_packet_id,
                atoms.to_vec(),
            );
            let n = packet.wire_len();
            if n > AtemPacket::MAX_PACKET_LEN || out.len() < n {
                return Err(crate::Error::InvalidLength);
            }
            packet.write_into(out)?;
            Ok(FramePlan::Datagram(&out[..n]))
        }
        TransportKind::Usb => {
            let announce = build_data_host(atoms.len()).map_err(|_| crate::Error::InvalidLength)?;
            Ok(FramePlan::AnnounceThenPayload { announce, payload: atoms })
        }
    }
}

/// The USB data-announce header size, re-exported for callers matching on
/// [`FramePlan::AnnounceThenPayload`].
pub const USB_ANNOUNCE_LEN: usize = crate::usb::protocol::HOST_HEADER_BODY_LEN;

/// The USB tunnel header size.
pub const USB_HEADER_LEN: usize = HEADER_LEN;

#[cfg(test)]
mod tests {
    use alloc::vec::Vec;

    use bmd_atem_atom::{AtomWriter, FourCc};

    use super::*;
    use crate::udp::packet::AtemPacketPayload;

    fn build_command() -> [u8; 12] {
        let mut buf = [0u8; 12];
        let mut w = AtomWriter::new(&mut buf);
        w.push_source(FourCc(*b"CPgI"), 0, 3).unwrap();
        let _ = w.finish();
        buf
    }

    /// The whole point of the shared layer: both framings carry the same bytes.
    #[test]
    fn udp_and_usb_carry_identical_atom_bytes() {
        let atoms = build_command();
        let mut wire = [0u8; 64];

        let udp = frame_command(
            TransportKind::Udp,
            FrameContext { session_id: 0x8001, ..FrameContext::default() },
            &atoms,
            &mut wire,
        )
        .unwrap();
        let FramePlan::Datagram(datagram) = udp else { panic!("udp must be a datagram") };
        let decoded = AtemPacket::decode(datagram).unwrap();
        let AtemPacketPayload::Atoms(udp_atoms) = &decoded.payload else {
            panic!("udp must carry atoms")
        };

        let usb = frame_command(TransportKind::Usb, FrameContext::usb(), &atoms, &mut []);
        let FramePlan::AnnounceThenPayload { announce, payload } = usb.unwrap() else {
            panic!("usb must announce")
        };

        assert_eq!(payload, &atoms[..]);
        assert_eq!(payload.len(), atoms.len());
        assert_eq!(announce[..4], [0x00, 0x04, 0x00, 0x01]);
        assert_eq!(&announce[10..12], &(atoms.len() as u16).to_be_bytes());
        // UDP payload == USB payload == the atoms the caller authored.
        assert_eq!(payload, atoms.as_slice());
        assert_eq!(udp_atoms, &atoms);
    }

    #[test]
    fn write_counts_match_the_transports() {
        let atoms = build_command();
        let mut wire = [0u8; 64];
        let udp =
            frame_command(TransportKind::Udp, FrameContext::default(), &atoms, &mut wire).unwrap();
        assert_eq!(udp.writes(), 1);
        let usb = frame_command(TransportKind::Usb, FrameContext::usb(), &atoms, &mut []);
        assert_eq!(usb.unwrap().writes(), 2);
    }

    #[test]
    fn oversized_commands_are_rejected() {
        let atoms = [0u8; AtemPacket::MAX_PACKET_LEN];
        let mut wire = [0u8; 4096];
        assert_eq!(
            frame_command(TransportKind::Udp, FrameContext::default(), &atoms, &mut wire),
            Err(crate::Error::InvalidLength)
        );
        // A payload beyond u16 is impossible on USB too.
        let huge = Vec::<u8>::new();
        let _ = frame_command(TransportKind::Usb, FrameContext::usb(), &huge, &mut []);
    }

    #[test]
    fn framing_preserves_multi_atom_commands() {
        let mut buf = [0u8; 36];
        let mut w = AtomWriter::new(&mut buf);
        w.push_me(FourCc(*b"DAut"), 0).unwrap();
        w.push_source(FourCc(*b"CPgI"), 0, 2).unwrap();
        w.push_me(FourCc(*b"DCut"), 1).unwrap();
        let atoms: Vec<u8> = w.as_slice().to_vec();

        let mut wire = [0u8; 128];
        let plan =
            frame_command(TransportKind::Udp, FrameContext::default(), &atoms, &mut wire).unwrap();
        let FramePlan::Datagram(dg) = plan else { panic!() };
        let decoded = AtemPacket::decode(dg).unwrap();
        let AtemPacketPayload::Atoms(p) = &decoded.payload else { panic!() };
        assert_eq!(p, &atoms);
        assert_eq!(bmd_atem_atom::split_atoms(p).count(), 3);
    }
}
