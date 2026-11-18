//! Blackmagic ATEM USB-C control link (8.6+ protocol), host/controller side.
//!
//! The switcher exposes a vendor-specific bulk endpoint pair (OUT `0x02`, IN `0x82`)
//! on its USB control interface. This module implements the host side of the
//! "8.6+" tunnel:
//!
//! 1. [`protocol::build_init`] — host sends a type `0x01` packet (`00 01 10 92` payload).
//! 2. The switcher replies with type `0x02` (4-byte acknowledge).
//! 3. The switcher pushes the full initial state dump as type `0x04` announcements
//!    followed by raw atom records; the host consumes them through
//!    [`AtemUsbSession::poll`].
//! 4. Commands are sent with [`AtemUsbSession::send_atoms`] (type `0x04` + atoms).
//! 5. Type `0x03` carries RX-buffer usage in 1024-byte blocks (flow control);
//!    the host sends zero-length OUT transfers as keepalives when idle.
//!
//! Wire format reference: `docs/PROTOCOL_USB.md` and the OpenSwitcher / pyatem
//! sources cited there. This module is transport-agnostic: implement
//! [`UsbBulkHost`] for your stack (Tokio + `nusb` on the host,
//! `cotton-usb-host`-style bulk endpoints on firmware).

pub mod io;
pub mod protocol;
pub mod session;

pub use io::UsbBulkHost;
pub use protocol::{
    build_data_host, build_flow_host, build_init, DataLenLayout, FrameError, ParsedDeviceFrame,
    StreamParser, PKT_TYPE_DATA, PKT_TYPE_FLOW, PKT_TYPE_HOST_MISC, PKT_TYPE_INIT,
    PKT_TYPE_INIT_ACK,
};
pub use session::{
    AtemUsbSession, SessionEvent, UsbSessionConfig, UsbSessionError, UsbSessionState,
};
