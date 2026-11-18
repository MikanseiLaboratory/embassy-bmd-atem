//! Controller-side session FSM over [`UsbBulkHost`].
//!
//! Mirrors the live-verified ATEM USB 8.6+ flow (see `docs/PROTOCOL_USB.md`):
//! send the `0x01` init packet, wait for the `0x02` acknowledgement, then grant
//! the switcher receive credit with a `0x03` frame (blocks of 1024 B). The
//! switcher streams its initial state dump as `0x04`-announced chunks of a
//! **continuous atom stream** (records straddle chunk boundaries); every
//! consumed chunk is re-credited so the dump keeps flowing. Commands are sent
//! with [`AtemUsbSession::send_atoms`] paced ~4 ms per USB payload. Timeouts
//! are the caller's concern (wrap the futures in your runtime's
//! `timeout`/`select`).
//!
//! Bare zero-length OUT packets **stall** on real hardware, so the keepalive is
//! disabled by default; the tolerated `0x05` packet is the suspected
//! Software-Control idle frame (open question).

use alloc::vec::Vec;

use crate::usb::io::UsbBulkHost;
use crate::usb::protocol::{
    DataLenLayout, FrameError, ParsedDeviceFrame, StreamParser, build_data_host, build_flow_host,
    build_init,
};

/// Tunables for the USB 8.6+ controller session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbSessionConfig {
    /// Delay after the `0x04` header write and after the atoms write, in ms
    /// (pyatem waits 4 ms around every USB send).
    pub send_pace_ms: u32,
    /// Idle interval for host-initiated zero-length keepalive packets, in ms.
    ///
    /// Live-verified against `pyatem` on Linux/usbmon: the reference client
    /// writes a ZLP roughly every 510 ms while idle and never stalls, so the
    /// keepalive is **on** by default at the same cadence. (A zero-length
    /// write is rejected by WinUSB through `nusb`, so Windows backends may need
    /// to raise this or use a 1-byte payload.)
    pub keepalive_ms: u32,
    /// Upper bound of a device-announced chunk, in bytes. Larger announcements
    /// are a protocol error and resynchronise the stream.
    pub max_atoms_len: u16,
    /// Size of the backend read scratchpad (largest expected bulk IN transfer).
    pub read_chunk: usize,
    /// Blocks (× 1024 B) of receive credit granted to the switcher once after
    /// the `0x02` ack. Live-verified: the switcher sends exactly this much of
    /// the initial dump before waiting for more credit. `None` skips the grant
    /// (the switcher then never starts the dump).
    pub initial_flow_blocks: Option<u16>,
    /// Blocks (× 1024 B) of receive credit re-granted after every consumed
    /// chunk, keeping the dump flowing. `None` disables auto-credit.
    pub auto_credit_blocks: Option<u16>,
    /// How the device → host `0x04` length field is encoded.
    pub data_len_layout: DataLenLayout,
}

impl Default for UsbSessionConfig {
    fn default() -> Self {
        Self {
            send_pace_ms: 4,
            keepalive_ms: 510,
            max_atoms_len: 0x4000,
            read_chunk: 2048,
            initial_flow_blocks: Some(1),
            auto_credit_blocks: Some(1),
            data_len_layout: DataLenLayout::LeAtStart,
        }
    }
}

/// State of [`AtemUsbSession`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UsbSessionState {
    /// Created; `0x01` not yet sent.
    New,
    /// `0x01` sent, waiting for the switcher's `0x02` acknowledge.
    InitSent,
    /// Switcher acknowledged; initial dump pending.
    Acked,
    /// First atoms block received; fully operational.
    Running,
}

/// One event surfaced by [`AtemUsbSession::poll`]. `Atoms` blocks borrow the caller's
/// buffer that was passed to `poll`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionEvent<'a> {
    /// Switcher acknowledged the init packet (session state `InitSent → Acked`).
    InitAck,
    /// One announced atoms blob. `first` marks the start of the initial state dump
    /// (state `→ Running`). A zero-length block may mark the end of a dump.
    Atoms { block: &'a [u8], first: bool },
    /// Switcher RX-buffer usage announcement in blocks of 1024 B.
    FlowCtrl { blocks: u16 },
    /// Tolerated but uninterpreted device packet type (e.g. `0x05`), surfaced for
    /// diagnostics only.
    Unknown { pkt_type: u8 },
}

/// Errors from [`AtemUsbSession`] operations.
#[derive(Debug)]
pub enum UsbSessionError<E> {
    /// USB-tunnel framing / protocol failure.
    Protocol(FrameError),
    /// Operation requires an established session (`connect` first, or lost link).
    NotConnected,
    /// Backend transport failure.
    Backend(E),
}

impl<E: core::fmt::Debug> core::fmt::Display for UsbSessionError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Protocol(e) => write!(f, "ATEM USB protocol error: {e}"),
            Self::NotConnected => write!(f, "ATEM USB session not connected"),
            Self::Backend(e) => write!(f, "USB backend failure: {e:?}"),
        }
    }
}

#[cfg(feature = "std")]
impl<E: core::fmt::Debug + 'static> std::error::Error for UsbSessionError<E> {}

/// Live USB 8.6+ controller session.
///
/// `poll` is the main entry point: each call may produce at most one event
/// ([`Ok(None)`]s mean "nothing complete yet"). Events borrow the `buf` passed in, so
/// consume them (or `to_vec` the block) before the next call. `send_atoms` may be
/// interleaved with polling; over USB there is no ACK/retransmission machinery.
#[derive(Debug)]
pub struct AtemUsbSession<H: UsbBulkHost> {
    host: H,
    config: UsbSessionConfig,
    parser: StreamParser,
    scratch: Vec<u8>,
    state: UsbSessionState,
    said_first_atoms: bool,
    last_tx_ms: u32,
    pending_flow_announce: Option<u16>,
    /// Credit re-grant scheduled by the last consumed chunk.
    pending_credit: Option<u16>,
    announced_blocks: u16,
    /// Set when the backend reports an idle IN stall: the switcher has
    /// nothing queued, so further credit re-grants are skipped.
    credit_paused: bool,
}

/// Internal frame kind without a payload borrow.
enum ReadyFrame {
    InitAck,
    Flow { blocks: u16 },
    Data(usize),
    Unknown(u8),
}

impl<H: UsbBulkHost> AtemUsbSession<H> {
    /// Create a session around an already-claimed bulk endpoint pair.
    #[must_use]
    pub fn new(host: H, config: UsbSessionConfig) -> Self {
        let parser = StreamParser::new(config.data_len_layout, usize::from(config.max_atoms_len));
        let scratch = alloc::vec![0u8; config.read_chunk.max(64)];
        Self {
            host,
            config,
            parser,
            scratch,
            state: UsbSessionState::New,
            said_first_atoms: false,
            last_tx_ms: 0,
            pending_flow_announce: None,
            pending_credit: None,
            announced_blocks: 0,
            credit_paused: false,
        }
    }

    /// Current session state.
    #[must_use]
    pub fn state(&self) -> UsbSessionState {
        self.state
    }

    /// Session configuration.
    #[must_use]
    pub fn config(&self) -> &UsbSessionConfig {
        &self.config
    }

    /// Open the control link: reset endpoint state, then send the `0x01` init packet
    /// (`00 01 10 92` payload) as one bulk OUT transfer.
    ///
    /// # Errors
    /// Backend failures only; handshake timeouts belong to the caller.
    pub async fn connect(&mut self, now_ms: u32) -> Result<(), UsbSessionError<H::Error>> {
        self.host.reset_state().await.map_err(UsbSessionError::Backend)?;
        self.parser.clear();
        self.host.write_bulk(&build_init()).await.map_err(UsbSessionError::Backend)?;
        self.state = UsbSessionState::InitSent;
        self.said_first_atoms = false;
        self.pending_flow_announce = None;
        self.pending_credit = None;
        self.announced_blocks = 0;
        self.credit_paused = false;
        self.last_tx_ms = now_ms;
        Ok(())
    }

    /// Return to [`UsbSessionState::New`] (e.g. after the device disconnected and
    /// re-appeared). Call [`connect`](Self::connect) afterwards.
    pub fn reset(&mut self) {
        self.parser.clear();
        self.state = UsbSessionState::New;
        self.said_first_atoms = false;
        self.pending_flow_announce = None;
        self.pending_credit = None;
        self.announced_blocks = 0;
        self.credit_paused = false;
    }

    /// Drive the session: may emit at most **one** event per call.
    ///
    /// Sends keepalives (ZLP) and pending flow-control announcements as needed, then
    /// reads from the backend until a complete tunnel packet is parsed or the stream
    /// is exhausted. Returns `Ok(None)` when nothing complete was available.
    ///
    /// # Errors
    /// Backend failures and protocol framing errors (the parser resynchronises itself,
    /// so transient garbage does not surface as errors).
    pub async fn poll<'a>(
        &mut self,
        now_ms: u32,
        buf: &'a mut [u8],
    ) -> Result<Option<SessionEvent<'a>>, UsbSessionError<H::Error>> {
        if self.state == UsbSessionState::New {
            return Err(UsbSessionError::NotConnected);
        }
        // Flush pending credit first, then the initial grant, then keepalives.
        let announce = self.pending_credit.take().or_else(|| {
            matches!(self.state, UsbSessionState::Acked | UsbSessionState::Running)
                .then(|| self.pending_flow_announce.take())
                .flatten()
        });
        if let Some(blocks) = announce {
            self.host
                .write_bulk(&build_flow_host(blocks))
                .await
                .map_err(UsbSessionError::Backend)?;
            self.announced_blocks = self.announced_blocks.max(blocks);
            self.last_tx_ms = now_ms;
        } else if self.config.keepalive_ms != u32::MAX
            && matches!(self.state, UsbSessionState::Acked | UsbSessionState::Running)
            && self.parser.is_idle()
            && now_ms.wrapping_sub(self.last_tx_ms) >= self.config.keepalive_ms
        {
            self.host.write_bulk(&[]).await.map_err(UsbSessionError::Backend)?;
            self.last_tx_ms = now_ms;
        }

        if let Some(ready) = self.take_ready_frame(buf).map_err(UsbSessionError::Protocol)? {
            return Ok(Some(self.finish_event(ready, buf)));
        }
        loop {
            let n =
                self.host.read_bulk(&mut self.scratch).await.map_err(UsbSessionError::Backend)?;
            if n == 0 {
                if self.host.took_idle_stall() {
                    // The switcher is idle (live-verified stall semantics):
                    // stop re-granting credit until the session is reset.
                    self.credit_paused = true;
                }
                return Ok(None); // device ZLP: not a frame
            }
            self.parser.push(&self.scratch[..n]);
            if let Some(ready) = self.take_ready_frame(buf).map_err(UsbSessionError::Protocol)? {
                return Ok(Some(self.finish_event(ready, buf)));
            }
        }
    }

    /// Send command atom records to the switcher as a `0x04`-announced blob
    /// (header transfer, then the payload transfer(s), paced per config).
    ///
    /// # Errors
    /// [`UsbSessionError::NotConnected`] before the handshake, backend failures,
    /// oversized payloads.
    pub async fn send_atoms(
        &mut self,
        atoms: &[u8],
        now_ms: u32,
    ) -> Result<(), UsbSessionError<H::Error>> {
        if !matches!(self.state, UsbSessionState::Acked | UsbSessionState::Running) {
            return Err(UsbSessionError::NotConnected);
        }
        let hdr = build_data_host(atoms.len()).map_err(UsbSessionError::Protocol)?;
        self.host.write_bulk(&hdr).await.map_err(UsbSessionError::Backend)?;
        if !atoms.is_empty() {
            self.host.delay_ms(self.config.send_pace_ms).await;
            self.host.write_bulk(atoms).await.map_err(UsbSessionError::Backend)?;
            self.host.delay_ms(self.config.send_pace_ms).await;
        }
        self.last_tx_ms = now_ms;
        Ok(())
    }

    /// Send a raw tunnel frame bypassing the session's framing (protocol
    /// probing: e.g. the undocumented `0x05` host → device packet).
    ///
    /// # Errors
    /// Backend failures only.
    pub async fn write_raw(
        &mut self,
        bytes: &[u8],
        now_ms: u32,
    ) -> Result<(), UsbSessionError<H::Error>> {
        self.host.write_bulk(bytes).await.map_err(UsbSessionError::Backend)?;
        self.last_tx_ms = now_ms;
        Ok(())
    }

    /// Poll the switcher with a `TiRq` (time-request / NOP) command — pyatem's
    /// read-only "are you there" probe. A fresh credit grant is flushed first
    /// so the queued `Time`/state response can come back.
    ///
    /// # Errors
    /// [`UsbSessionError::NotConnected`] before the handshake, backend failures.
    pub async fn poll_ping(&mut self, now_ms: u32) -> Result<(), UsbSessionError<H::Error>> {
        if !matches!(self.state, UsbSessionState::Acked | UsbSessionState::Running) {
            return Err(UsbSessionError::NotConnected);
        }
        // `TiRq` atom: u16 len (8), u16 pad, FourCC, no payload.
        const TIRQ: [u8; 8] = [0x00, 0x08, 0x00, 0x00, b'T', b'i', b'R', b'q'];
        self.write_atoms(&TIRQ, now_ms).await
    }

    /// Send an arbitrary command atom followed by a fresh credit grant.
    ///
    /// # Errors
    /// [`UsbSessionError::NotConnected`] before the handshake, backend failures.
    pub async fn poll_ping_stod(
        &mut self,
        atoms: &[u8],
        now_ms: u32,
    ) -> Result<(), UsbSessionError<H::Error>> {
        if !matches!(self.state, UsbSessionState::Acked | UsbSessionState::Running) {
            return Err(UsbSessionError::NotConnected);
        }
        self.write_atoms(atoms, now_ms).await
    }

    /// Grant receive credit and wait for whatever is queued to flush.
    ///
    /// Live-verified ordering: the switcher **stalls the OUT endpoint** if the
    /// host sends a new command after a response without first re-granting
    /// receive credit (observed: 2nd poll of a pair always failed until the
    /// grant moved in front of the command). So the grant goes **before** the
    /// command, with a second grant after it for the longer responses.
    async fn write_atoms(
        &mut self,
        atoms: &[u8],
        now_ms: u32,
    ) -> Result<(), UsbSessionError<H::Error>> {
        let blocks = self.config.auto_credit_blocks.unwrap_or(1);
        // Pre-grant: reopen the device's transmit window.
        self.host
            .write_bulk(&crate::usb::protocol::build_flow_host(blocks))
            .await
            .map_err(UsbSessionError::Backend)?;
        self.send_atoms(atoms, now_ms).await?;
        // Post-grant: cover the response (may be multiple 0x04 chunks).
        self.host
            .write_bulk(&crate::usb::protocol::build_flow_host(blocks))
            .await
            .map_err(UsbSessionError::Backend)?;
        self.credit_paused = false;
        self.last_tx_ms = now_ms;
        Ok(())
    }

    /// Report our RX backlog (`bytes` not yet consumed by the application) to the
    /// switcher. Returns `Ok(true)` when a `0x03` frame was written (only announced
    /// for **increasing** block counts, matching pyatem).
    ///
    /// # Errors
    /// Backend failures only.
    pub async fn set_rx_backlog(
        &mut self,
        bytes: usize,
        now_ms: u32,
    ) -> Result<bool, UsbSessionError<H::Error>> {
        if !matches!(self.state, UsbSessionState::Acked | UsbSessionState::Running) {
            return Ok(false);
        }
        let blocks = u16::try_from(bytes / 1024).unwrap_or(u16::MAX);
        if blocks > self.announced_blocks {
            self.host
                .write_bulk(&build_flow_host(blocks))
                .await
                .map_err(UsbSessionError::Backend)?;
            self.announced_blocks = blocks;
            self.last_tx_ms = now_ms;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    fn take_ready_frame(&mut self, buf: &mut [u8]) -> Result<Option<ReadyFrame>, FrameError> {
        let parsed = self.parser.try_next(buf)?;
        // Any consumed chunk (even one that completed no atom) earns the
        // switcher fresh receive credit so the dump keeps flowing — unless the
        // switcher already signalled idle, in which case further grants are
        // pointless (and stall-prone).
        let consumed = self.parser.take_consumed_chunk_bytes();
        if consumed > 0 && !self.credit_paused {
            self.pending_credit = self.config.auto_credit_blocks;
        }
        match parsed {
            None => Ok(None),
            Some(ParsedDeviceFrame::InitAck) => {
                if self.state == UsbSessionState::InitSent {
                    self.state = UsbSessionState::Acked;
                    self.pending_flow_announce = self.config.initial_flow_blocks;
                }
                Ok(Some(ReadyFrame::InitAck))
            }
            Some(ParsedDeviceFrame::FlowCtrl { blocks }) => Ok(Some(ReadyFrame::Flow { blocks })),
            Some(ParsedDeviceFrame::Data { atoms_len }) => Ok(Some(ReadyFrame::Data(atoms_len))),
            Some(ParsedDeviceFrame::Unknown { pkt_type }) => {
                Ok(Some(ReadyFrame::Unknown(pkt_type)))
            }
        }
    }

    fn finish_event<'a>(&mut self, ready: ReadyFrame, buf: &'a mut [u8]) -> SessionEvent<'a> {
        match ready {
            ReadyFrame::InitAck => SessionEvent::InitAck,
            ReadyFrame::Flow { blocks } => SessionEvent::FlowCtrl { blocks },
            ReadyFrame::Data(atoms_len) => {
                let first = !self.said_first_atoms;
                if first {
                    self.said_first_atoms = true;
                    if self.state != UsbSessionState::Running {
                        self.state = UsbSessionState::Running;
                    }
                }
                SessionEvent::Atoms { block: &buf[..atoms_len], first }
            }
            ReadyFrame::Unknown(pkt_type) => SessionEvent::Unknown { pkt_type },
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use alloc::collections::VecDeque;
    use alloc::vec;
    use alloc::vec::Vec;

    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockErr;

    impl core::fmt::Display for MockErr {
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            write!(f, "mock error")
        }
    }

    #[derive(Debug, Default)]
    struct MockBulk {
        inbound: VecDeque<Vec<u8>>,
        outbound: Vec<Vec<u8>>,
        delays: Vec<u32>,
        /// Whether the next `took_idle_stall` should report a stall.
        stall_flag: bool,
    }

    impl MockBulk {
        fn new(inbound: impl IntoIterator<Item = Vec<u8>>) -> Self {
            Self {
                inbound: VecDeque::from(inbound.into_iter().collect::<Vec<_>>()),
                ..Self::default()
            }
        }
    }
    impl UsbBulkHost for MockBulk {
        type Error = MockErr;

        fn took_idle_stall(&mut self) -> bool {
            core::mem::take(&mut self.stall_flag)
        }

        async fn read_bulk(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            match self.inbound.pop_front() {
                None => Ok(0),
                Some(data) => {
                    let n = data.len().min(buf.len());
                    buf[..n].copy_from_slice(&data[..n]);
                    Ok(n)
                }
            }
        }

        async fn write_bulk(&mut self, data: &[u8]) -> Result<(), Self::Error> {
            self.outbound.push(data.to_vec());
            Ok(())
        }

        async fn reset_state(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        async fn delay_ms(&mut self, ms: u32) {
            self.delays.push(ms);
        }
    }

    async fn finish_handshake(s: &mut AtemUsbSession<MockBulk>, now: &mut u32, buf: &mut [u8]) {
        s.connect(*now).await.unwrap();
        *now += 1;
        let ev = s.poll(*now, buf).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::InitAck));
        assert_eq!(s.state(), UsbSessionState::Acked);
    }

    #[tokio::test]
    async fn idle_stall_pauses_credit() {
        // After the switcher signals idle (IN stall), further credit grants
        // are suppressed — matching the live capture where the pointless
        // grant after the final chunk served no purpose.
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let dc = bmd_atem_atom::encode_atom_do_transition_cut(0);
        let hdr = |len: u16| {
            let [l, h] = len.to_le_bytes();
            vec![0x04, 0x00, 0x01, 0x00, l, h, 0, 0, 0, 0, 0, 0]
        };
        let mut session = AtemUsbSession::new(
            MockBulk::new(vec![
                ack.to_vec(),
                hdr(12).into_iter().chain(dc.iter().copied()).collect(),
            ]),
            UsbSessionConfig::default(),
        );
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        now += 1;

        // Chunk consumed → credit scheduled; the idle stall is flagged on the
        // same poll (chunk read returns data, then the poll's next read hits
        // the empty queue). Simulate by flagging the stall manually.
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::Atoms { block: dc.as_slice(), first: true }));
        session.host.stall_flag = true;

        // The stalled poll pauses credit: the auto-grant scheduled by the
        // chunk was already flushed (it precedes the stall detection), but no
        // *further* grants follow.
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, None);
        assert_eq!(session.host.outbound.len(), 3); // init + grant + final auto-grant
        assert_eq!(session.host.outbound[2], build_flow_host(1));
        assert!(session.credit_paused);

        // Nothing new: no more grants.
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, None);
        assert_eq!(session.host.outbound.len(), 3);
    }

    #[tokio::test]
    async fn credit_refill_keeps_dump_flowing() {
        let cp = bmd_atem_atom::encode_atom_change_program_input(0, 3);
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let hdr = |len: u16| {
            let [l, h] = len.to_le_bytes();
            vec![0x04, 0x00, 0x01, 0x00, l, h, 0, 0, 0, 0, 0, 0]
        };
        let mut session = AtemUsbSession::new(
            MockBulk::new(vec![
                ack.to_vec(),
                hdr(24).into_iter().chain(cp.iter().copied()).chain(cp.iter().copied()).collect(),
                hdr(24).into_iter().chain(cp.iter().copied()).chain(cp.iter().copied()).collect(),
            ]),
            UsbSessionConfig::default(),
        );
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        now += 1;

        // Chunk 1: initial credit grant flushed, then the first atoms.
        let ev = session.poll(now, &mut rxb).await.unwrap();
        let both: Vec<u8> = cp.iter().chain(cp.iter()).copied().collect();
        assert_eq!(ev, Some(SessionEvent::Atoms { block: both.as_slice(), first: true }));
        assert_eq!(session.host.outbound.len(), 2);
        assert_eq!(session.host.outbound[1], build_flow_host(1));

        // Chunk 2: the auto-credit re-grant is flushed before reading it.
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::Atoms { block: both.as_slice(), first: false }));
        assert_eq!(session.host.outbound.len(), 3);
        assert_eq!(session.host.outbound[2], build_flow_host(1));
    }

    #[tokio::test]
    async fn chunk_completing_no_atom_still_earns_credit() {
        // A chunk that is one giant partial record: consumed but no event yet.
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let partial: Vec<u8> =
            [0x00u8, 0x20, 0x00, 0x00].into_iter().chain(core::iter::repeat_n(0u8, 8)).collect();
        let hdr = |len: u16| {
            let [l, h] = len.to_le_bytes();
            vec![0x04, 0x00, 0x01, 0x00, l, h, 0, 0, 0, 0, 0, 0]
        };
        let mut session = AtemUsbSession::new(
            MockBulk::new(vec![
                ack.to_vec(),
                hdr(12).into_iter().chain(partial.iter().copied()).collect(),
            ]),
            UsbSessionConfig::default(),
        );
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        now += 1;

        // The chunk is consumed (no complete atom → no event), and the next
        // poll still re-grants credit for it.
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, None);
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, None);
        assert!(session.host.outbound.iter().any(|w| w == &build_flow_host(1)));
    }

    #[tokio::test]
    async fn handshake_init_dump_and_command() {
        let dc = bmd_atem_atom::encode_atom_do_transition_cut(0);
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let mut data = Vec::new();
        data.extend_from_slice(&[0x04, 0x00, 0x01, 0x00, 0x0c, 0x00, 0, 0, 0, 0, 0, 0]);
        data.extend_from_slice(&dc);

        let mock = MockBulk::new(vec![ack.to_vec(), data]);
        let mut session = AtemUsbSession::new(mock, UsbSessionConfig::default());
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        session.connect(now).await.unwrap();
        assert_eq!(session.state(), UsbSessionState::InitSent);
        assert_eq!(session.host.outbound[0], build_init());
        now += 1;

        // InitAck arrives; the initial flow announcement (4 blocks) is written on the
        // *next* poll before reading.
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::InitAck));
        assert_eq!(session.state(), UsbSessionState::Acked);
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::Atoms { block: &dc, first: true }));
        assert_eq!(session.state(), UsbSessionState::Running);
        // outbound: [init, flow(4)] and the pending announce does not double-send.
        assert_eq!(session.host.outbound.len(), 2);
        assert_eq!(session.host.outbound[1], build_flow_host(1));
        assert!(session.host.delays.is_empty());

        // Sending a command is header + payload with pacing around the payload.
        session.send_atoms(&dc, now).await.unwrap();
        assert_eq!(session.host.outbound.len(), 4);
        assert_eq!(session.host.outbound[2], build_data_host(12).unwrap());
        assert_eq!(session.host.outbound[3], dc);
        assert_eq!(session.host.delays, vec![4, 4]);
    }

    #[tokio::test]
    async fn split_blob_across_reads() {
        let cp = bmd_atem_atom::encode_atom_change_program_input(0, 3);
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let cfg = UsbSessionConfig { initial_flow_blocks: None, ..UsbSessionConfig::default() };
        let mock = MockBulk::new(vec![
            ack.to_vec(),
            vec![0x04, 0x00, 0x01, 0x00, 0x18, 0x00, 0, 0, 0, 0, 0, 0],
            cp.to_vec(),
            cp.to_vec(),
        ]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        finish_handshake(&mut session, &mut now, &mut [0u8; 64]).await;
        now += 1;
        let mut rxb = vec![0u8; 64];
        let ev = session.poll(now, &mut rxb).await.unwrap();
        let expected: Vec<u8> = cp.iter().chain(cp.iter()).copied().collect();
        assert_eq!(ev, Some(SessionEvent::Atoms { block: expected.as_slice(), first: true }));
        assert_eq!(session.state(), UsbSessionState::Running);
    }

    #[tokio::test]
    async fn keepalive_zlp_when_idle() {
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let cfg = UsbSessionConfig {
            initial_flow_blocks: None,
            keepalive_ms: 100,
            ..UsbSessionConfig::default()
        };
        let mock = MockBulk::new(vec![ack.to_vec()]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        let ev = session.poll(now + 200, &mut rxb).await.unwrap();
        assert_eq!(ev, None); // ZLP was sent, then the read produced nothing.
        assert!(session.host.outbound.iter().any(|w| w.is_empty()));
    }

    #[tokio::test]
    async fn flow_ctrl_from_device() {
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let mut raw = [0u8; 12];
        raw[..4].copy_from_slice(&[0x03, 0x00, 0x01, 0x00]);
        raw[4] = 0x00;
        raw[5] = 0x02; // big-endian u16 at body start (pyatem RX: '>BBBBH6x')
        let cfg = UsbSessionConfig { initial_flow_blocks: None, ..UsbSessionConfig::default() };
        let mock = MockBulk::new(vec![ack.to_vec(), raw.to_vec()]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::FlowCtrl { blocks: 2 }));
    }

    #[tokio::test]
    async fn unknown_type_is_surfaced() {
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let cfg = UsbSessionConfig { initial_flow_blocks: None, ..UsbSessionConfig::default() };
        let mock = MockBulk::new(vec![ack.to_vec(), vec![0x05, 0x00, 0x01, 0x00]]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        now += 1;
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::Unknown { pkt_type: 0x05 }));
    }

    #[tokio::test]
    async fn garbage_does_not_tear_down_the_stream() {
        // pyatem's typo'd flow frame: `write(0x03, b'x00\x00\x00\x00\x00\x00\x10\x00\x00\x00')`
        let garbage = b"x00\x00\x00\x00\x00\x00\x10\x00\x00\x00".to_vec();
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let cfg = UsbSessionConfig { initial_flow_blocks: None, ..UsbSessionConfig::default() };
        let mock = MockBulk::new(vec![garbage, ack.to_vec()]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        session.connect(now).await.unwrap();
        now += 1;
        // First poll resyncs past the garbage (no event) and continues reading.
        let ev = session.poll(now, &mut rxb).await.unwrap();
        assert_eq!(ev, Some(SessionEvent::InitAck));
    }

    #[tokio::test]
    async fn backlog_announcements_are_monotonic() {
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let cfg = UsbSessionConfig { initial_flow_blocks: None, ..UsbSessionConfig::default() };
        let mock = MockBulk::new(vec![ack.to_vec()]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        let mut rxb = [0u8; 64];
        finish_handshake(&mut session, &mut now, &mut rxb).await;
        now += 1;
        assert!(session.set_rx_backlog(4096, now).await.unwrap());
        assert_eq!(*session.host.outbound.last().unwrap(), build_flow_host(4));
        assert!(!session.set_rx_backlog(2048, now).await.unwrap());
        assert!(session.set_rx_backlog(9000, now).await.unwrap());
        assert_eq!(*session.host.outbound.last().unwrap(), build_flow_host(8));
    }

    #[tokio::test]
    async fn not_connected_before_connect() {
        let mut session = AtemUsbSession::new(MockBulk::default(), UsbSessionConfig::default());
        let err = session.send_atoms(bmd_atem_atom::encode_atom_do_ftb_auto(0).as_slice(), 0).await;
        assert!(matches!(err, Err(UsbSessionError::NotConnected)));
        let mut rxb = [0u8; 8];
        let err = session.poll(0, &mut rxb).await;
        assert!(matches!(err, Err(UsbSessionError::NotConnected)));
    }

    #[tokio::test]
    async fn oversized_announcement_is_a_protocol_error() {
        let ack = [0x02u8, 0x00, 0x01, 0x00];
        let cfg = UsbSessionConfig {
            initial_flow_blocks: None,
            max_atoms_len: 16,
            ..UsbSessionConfig::default()
        };
        let mock = MockBulk::new(vec![
            ack.to_vec(),
            vec![0x04, 0x00, 0x01, 0x00, 0x40, 0x00, 0, 0, 0, 0, 0, 0], // 0x0040 bytes announced
        ]);
        let mut session = AtemUsbSession::new(mock, cfg);
        let mut now = 0u32;
        let mut rxb = [0u8; 128];
        finish_handshake(&mut session, &mut now, &mut [0u8; 64]).await;
        now += 1;
        let err = session.poll(now, &mut rxb).await.unwrap_err();
        assert!(matches!(err, UsbSessionError::Protocol(FrameError::InvalidLength)));
    }
}
