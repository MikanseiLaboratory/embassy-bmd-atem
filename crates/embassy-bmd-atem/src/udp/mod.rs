//! UDP/BURP transport: packet codec, connect handshake, ACK queue and edge-nal-style
//! UDP traits.
//!
//! Field layout and handshake behaviour are aligned with the
//! [necromancer](https://github.com/micolous/necromancer) project (Apache-2.0); see
//! the workspace `NOTICE` file.
//!
//! - Implement [`io::UdpSend`], [`io::UdpReceive`] and [`io::UdpReceiveBounded`] for
//!   your stack (for example wrap `edge-nal-embassy` types in a newtype and delegate;
//!   API matches `edge-nal` 0.6). `UdpReceiveBounded` is typically `select(recv, timer)`
//!   on the wait slice.
//! - Call [`packet::AtemPacket::decode`] / [`packet::AtemPacket::write_into`] for
//!   framing, or [`session::AtemSession::connect`] with a monotonic `now_ms` closure.

pub mod io;
pub mod mock;
pub mod packet;
pub mod session;

/// Default UDP control port of ATEM switchers.
pub const ATEM_UDP_PORT: u16 = 9910;

pub use io::{ErrorType, UdpReceive, UdpReceiveBounded, UdpSend};
pub use mock::{EdgeNalUdp, MockUdp, MockUdpExhausted};
pub use packet::{AtemControl, AtemPacket, AtemPacketFlags, AtemPacketPayload};
pub use session::{AtemSession, PendingPacket, SessionConfig, SessionError, SessionState};
