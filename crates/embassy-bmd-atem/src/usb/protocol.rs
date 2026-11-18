//! ATEM USB-C "8.6+" tunnel framing (host/controller side).
//!
//! Packet layout (see `docs/PROTOCOL_USB.md` for provenance):
//!
//! ```text
//! host  → device:  00 <type> 00 01  [type-specific body, 0..8 bytes]
//! device→ host:  <type>  00 01 00  [type-specific body, 0..8 bytes]
//! ```
//!
//! Types: `0x01` init (host→device, body `00 01 10 92`), `0x02` init-ack
//! (device→host, no body), `0x03` flow control (RX-buffer blocks of 1024 B),
//! `0x04` data announce (the atoms blob follows in subsequent bulk transfers),
//! `0x05` host-keepalive variant (unknown purpose, tolerated).
//!
//! Byte-order notes (pyatem ground truth, `git.sr.ht/~martijnbraam/pyatem`):
//! host→device length fields are **big-endian at the body end**; the
//! device→host `0x03` block count is parsed **big-endian at the body start**,
//! and the `0x04` atoms length **little-endian at the body start**
//! ([`DataLenLayout::LeAtStart`]). Because the OpenSwitcher documentation table
//! contradicts the shipped parser, [`StreamParser`] tolerates both encodings and
//! [`DataLenLayout`] selects the primary one.

use alloc::vec::Vec;

/// USB-tunnel packet type: start a new connection (host → device).
pub const PKT_TYPE_INIT: u8 = 0x01;
/// USB-tunnel packet type: acknowledge the init packet (device → host).
pub const PKT_TYPE_INIT_ACK: u8 = 0x02;
/// USB-tunnel packet type: RX-buffer usage / flow control (both directions).
pub const PKT_TYPE_FLOW: u8 = 0x03;
/// USB-tunnel packet type: announce an atoms blob (both directions).
pub const PKT_TYPE_DATA: u8 = 0x04;
/// USB-tunnel packet type with no documented purpose (host → device); tolerated.
pub const PKT_TYPE_HOST_MISC: u8 = 0x05;

/// Payload of the initiating `0x01` packet. Per OpenSwitcher docs the meaning is unknown.
pub const INIT_PAYLOAD: [u8; 4] = [0x00, 0x01, 0x10, 0x92];

/// Header size shared by every USB-tunnel packet.
pub const HEADER_LEN: usize = 4;
/// Length of host → device flow/data header packets (header + 8-byte body).
pub const HOST_HEADER_BODY_LEN: usize = 12;

/// Encoding of the device → host `0x04` atoms length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DataLenLayout {
    /// `u16` **little-endian** directly after the 4-byte header, atoms follow immediately
    /// (what pyatem's shipped RX parser reads). Zero-length fields fall back to
    /// [`DataLenLayout::BeAtEnd`] decoding when the transfer is long enough.
    #[default]
    LeAtStart,
    /// 8-byte body of six zero bytes followed by a `u16` **big-endian** length
    /// (symmetric to the host → device encoding, per the OpenSwitcher docs table).
    BeAtEnd,
}

/// Framing errors surfaced by the device → host parser.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Too few bytes for the header or body of the announced packet.
    TooShort,
    /// The announced blob exceeds the configured limit (stream is resynchronised).
    InvalidLength,
    /// The flow-control body had no usable block count.
    InvalidFlowBody,
    /// A `0x04`/`0x03`/atom operation was attempted outside an established session.
    NotConnected,
}

impl core::fmt::Display for FrameError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooShort => write!(f, "USB tunnel frame too short"),
            Self::InvalidLength => write!(f, "USB tunnel frame length out of range"),
            Self::InvalidFlowBody => write!(f, "malformed USB tunnel flow-control body"),
            Self::NotConnected => write!(f, "USB tunnel session not connected"),
        }
    }
}

/// Host → device header bytes: `00 <type> 00 01`.
#[must_use]
pub const fn host_header_host_to_device(t: u8) -> [u8; 4] {
    [0x00, t, 0x00, 0x01]
}

/// Device → host header bytes: `<type> 00 01 00`.
#[must_use]
pub const fn host_header_device_to_host(t: u8) -> [u8; 4] {
    [t, 0x00, 0x01, 0x00]
}

/// The complete `0x01` init packet (sent as one bulk OUT transfer).
#[must_use]
pub fn build_init() -> [u8; HEADER_LEN + 4] {
    let mut out = [0u8; HEADER_LEN + 4];
    out[..HEADER_LEN].copy_from_slice(&host_header_host_to_device(PKT_TYPE_INIT));
    out[HEADER_LEN..].copy_from_slice(&INIT_PAYLOAD);
    out
}

/// Host → device `0x04`: announces that an atoms blob of `atoms_len` bytes follows as the
/// next bulk OUT transfer(s). Length is encoded big-endian at the body end
/// (`pyatem` `send_packet` behaviour).
///
/// # Errors
/// [`FrameError::InvalidLength`] when `atoms_len` does not fit in a `u16`.
pub fn build_data_host(atoms_len: usize) -> Result<[u8; HOST_HEADER_BODY_LEN], FrameError> {
    let len = u16::try_from(atoms_len).map_err(|_| FrameError::InvalidLength)?;
    let [hi, lo] = len.to_be_bytes();
    let mut out = [0u8; HOST_HEADER_BODY_LEN];
    out[..HEADER_LEN].copy_from_slice(&host_header_host_to_device(PKT_TYPE_DATA));
    out[HOST_HEADER_BODY_LEN - 2] = hi;
    out[HOST_HEADER_BODY_LEN - 1] = lo;
    Ok(out)
}

/// Host → device `0x03`: announce our RX-buffer usage (`blocks` × 1024 B). Big-endian at
/// the body end, matching pyatem's `write(0x03, struct.pack('>6xH', blocks))`.
#[must_use]
pub fn build_flow_host(blocks: u16) -> [u8; HOST_HEADER_BODY_LEN] {
    let [hi, lo] = blocks.to_be_bytes();
    let mut out = [0u8; HOST_HEADER_BODY_LEN];
    out[..HEADER_LEN].copy_from_slice(&host_header_host_to_device(PKT_TYPE_FLOW));
    out[HOST_HEADER_BODY_LEN - 2] = hi;
    out[HOST_HEADER_BODY_LEN - 1] = lo;
    out
}

/// One parsed device → host tunnel packet. Atom payloads are copied into the caller's
/// buffer by [`StreamParser::try_next`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParsedDeviceFrame {
    /// `0x02` acknowledgement of the init packet.
    InitAck,
    /// `0x03` device → host flow control: RX-buffer usage in 1024 B blocks.
    FlowCtrl { blocks: u16 },
    /// `0x04` announce whose atoms blob was copied into the caller's buffer.
    Data { atoms_len: usize },
    /// Any other tolerated type (e.g. `0x05`), reported for diagnostics only.
    Unknown { pkt_type: u8 },
}

/// Reassembling parser for the device → host bulk transfer stream.
///
/// Feed every bulk IN transfer ([`push`](StreamParser::push), including zero-length
/// transfers) and drain events with [`try_next`](StreamParser::try_next) until it
/// returns `Ok(None)`. The parser tolerates payloads split across any number of
/// transfers and header/payload packing within one transfer, and resynchronises
/// by skipping bytes after an invalid header (pyatem ships a corrupted flow-control
/// frame upstream, so strictness would be counter-productive).
#[derive(Debug)]
pub struct StreamParser {
    layout: DataLenLayout,
    max_atoms_len: usize,
    buf: Vec<u8>,
    data_remain: usize,
    data: Vec<u8>,
    data_ready: Option<usize>,
}

impl StreamParser {
    /// Create a parser with the given length layout and maximum accepted atoms blob size.
    #[must_use]
    pub fn new(layout: DataLenLayout, max_atoms_len: usize) -> Self {
        Self {
            layout,
            max_atoms_len: max_atoms_len.max(8),
            buf: Vec::new(),
            data_remain: 0,
            data: Vec::new(),
            data_ready: None,
        }
    }

    /// `true` while no `0x04` blob is being reassembled.
    #[must_use]
    pub fn is_idle(&self) -> bool {
        self.data_remain == 0 && self.data_ready.is_none()
    }

    /// Bytes still needed for the in-flight blob.
    #[must_use]
    pub fn pending_bytes(&self) -> usize {
        self.data_remain
    }

    /// Queue freshly read bytes (one bulk IN transfer or a prefix thereof).
    pub fn push(&mut self, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        if self.data_remain > 0 {
            let take = bytes.len().min(self.data_remain);
            self.data.extend_from_slice(&bytes[..take]);
            self.data_remain -= take;
            if self.data_remain == 0 {
                self.data_ready = Some(self.data.len());
            }
            if take == bytes.len() {
                return;
            }
            self.buf.extend_from_slice(&bytes[take..]);
        } else {
            self.buf.extend_from_slice(bytes);
        }
    }

    /// Parse the next complete frame. [`ParsedDeviceFrame::Data`] payloads are copied into
    /// `out` (which must be at least `max_atoms_len` bytes).
    ///
    /// # Errors
    /// [`FrameError::InvalidLength`] when the blob exceeds the configured limit or the
    /// caller's buffer is too small (the parser is resynchronised / delivery deferred).
    pub fn try_next(&mut self, out: &mut [u8]) -> Result<Option<ParsedDeviceFrame>, FrameError> {
        loop {
            // Deliver a completed blob first (or when the caller's buffer is big enough).
            if let Some(n) = self.data_ready {
                if out.len() < n {
                    return Err(FrameError::InvalidLength);
                }
                out[..n].copy_from_slice(&self.data);
                self.data_ready = None;
                self.data.clear();
                return Ok(Some(ParsedDeviceFrame::Data { atoms_len: n }));
            }

            if self.buf.len() < HEADER_LEN {
                return Ok(None);
            }
            let b = &self.buf[..HEADER_LEN];
            let device_form = b[1] == 0x00 && b[2] == 0x01 && b[3] == 0x00;
            if !device_form {
                // Tolerate garbage: resynchronise by skipping one byte.
                self.buf.remove(0);
                continue;
            }
            match b[0] {
                PKT_TYPE_INIT_ACK => {
                    self.buf.drain(..HEADER_LEN);
                    return Ok(Some(ParsedDeviceFrame::InitAck));
                }
                PKT_TYPE_FLOW => {
                    if self.buf.len() < HOST_HEADER_BODY_LEN {
                        return Ok(None);
                    }
                    let mut blocks = u16::from_be_bytes([self.buf[4], self.buf[5]]);
                    if blocks == 0 {
                        // Tolerance for the mirrored layout.
                        let alt = u16::from_le_bytes([self.buf[10], self.buf[11]]);
                        if alt != 0 {
                            blocks = alt;
                        }
                    }
                    self.buf.drain(..HOST_HEADER_BODY_LEN);
                    return Ok(Some(ParsedDeviceFrame::FlowCtrl { blocks }));
                }
                PKT_TYPE_DATA => {
                    let Some((prefix, atoms_len)) = self.peek_data_len() else {
                        return Ok(None);
                    };
                    if atoms_len == 0 {
                        self.buf.drain(..prefix);
                        return Ok(Some(ParsedDeviceFrame::Data { atoms_len: 0 }));
                    }
                    if atoms_len > self.max_atoms_len {
                        self.buf.clear();
                        return Err(FrameError::InvalidLength);
                    }
                    self.buf.drain(..prefix);
                    self.data_remain = atoms_len;
                    self.data.clear();
                    // Route any bytes already buffered into the atoms blob.
                    if !self.buf.is_empty() {
                        let rest = core::mem::take(&mut self.buf);
                        self.push(&rest);
                    }
                    continue; // deliver the blob (possibly already complete) from the loop head
                }
                t => {
                    // Unknown device-packet type (e.g. 0x05): tolerated, header only.
                    self.buf.drain(..HEADER_LEN);
                    return Ok(Some(ParsedDeviceFrame::Unknown { pkt_type: t }));
                }
            }
        }
    }

    /// Clear all buffered state (after an unrecoverable framing error or on reconnect).
    pub fn clear(&mut self) {
        self.buf.clear();
        self.data.clear();
        self.data_remain = 0;
        self.data_ready = None;
    }

    /// Determine the header prefix length and the announced blob size from the buffer.
    /// `None` when more bytes are needed.
    fn peek_data_len(&self) -> Option<(usize, usize)> {
        match self.layout {
            DataLenLayout::LeAtStart => {
                if self.buf.len() < HEADER_LEN + 2 {
                    return None;
                }
                let mut len = usize::from(u16::from_le_bytes([self.buf[4], self.buf[5]]));
                if len == 0 && self.buf.len() >= HOST_HEADER_BODY_LEN {
                    // Tolerance for the mirrored layout.
                    let alt = usize::from(u16::from_be_bytes([self.buf[10], self.buf[11]]));
                    if alt != 0 {
                        len = alt;
                    }
                    return Some((HOST_HEADER_BODY_LEN, len));
                }
                Some((HEADER_LEN + 2, len))
            }
            DataLenLayout::BeAtEnd => {
                if self.buf.len() < HOST_HEADER_BODY_LEN {
                    return None;
                }
                Some((
                    HOST_HEADER_BODY_LEN,
                    usize::from(u16::from_be_bytes([self.buf[10], self.buf[11]])),
                ))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use bmd_atem_atom::encode_atom_do_transition_cut;

    use super::*;

    fn init_ack() -> [u8; 4] {
        host_header_device_to_host(PKT_TYPE_INIT_ACK)
    }

    fn data_le(len: u16) -> [u8; 6] {
        let [l, h] = len.to_le_bytes();
        host_header_device_to_host(PKT_TYPE_DATA)
            .iter()
            .copied()
            .chain([l, h])
            .collect::<Vec<u8>>()
            .try_into()
            .unwrap()
    }

    fn data_be(len: u16) -> [u8; 12] {
        let [h, l] = len.to_be_bytes();
        let mut out = [0u8; 12];
        out[..4].copy_from_slice(&host_header_device_to_host(PKT_TYPE_DATA));
        out[10] = h;
        out[11] = l;
        out
    }

    #[test]
    fn build_init_matches_pyatem() {
        assert_eq!(build_init(), [0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x10, 0x92]);
    }

    #[test]
    fn build_data_host_len_be_at_end() {
        let hdr = build_data_host(300).unwrap();
        assert_eq!(hdr[..4], [0x00, 0x04, 0x00, 0x01]);
        assert_eq!(&hdr[4..10], &[0u8; 6]);
        assert_eq!(&hdr[10..12], &300u16.to_be_bytes());
        assert!(build_data_host(0x10000).is_err());
    }

    #[test]
    fn build_flow_host_matches_pyatem_pack() {
        assert_eq!(build_flow_host(4), [0x00, 0x03, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0x00, 0x04]);
    }

    #[test]
    fn parses_init_ack() {
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 4096);
        p.push(&init_ack());
        assert_eq!(p.try_next(&mut [0u8; 8]).unwrap(), Some(ParsedDeviceFrame::InitAck));
        assert_eq!(p.try_next(&mut [0u8; 8]).unwrap(), None);
    }

    #[test]
    fn parses_flow_be_at_start() {
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 4096);
        let mut raw = [0u8; 12];
        raw[..4].copy_from_slice(&host_header_device_to_host(PKT_TYPE_FLOW));
        // pyatem RX uses '>BBBBH6x': big-endian u16 at body start.
        raw[4] = 0x00;
        raw[5] = 0x03;
        p.push(&raw);
        assert_eq!(
            p.try_next(&mut [0u8; 8]).unwrap(),
            Some(ParsedDeviceFrame::FlowCtrl { blocks: 3 })
        );
    }

    #[test]
    fn parses_data_le_with_payload_in_same_transfer() {
        let dc = encode_atom_do_transition_cut(0);
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 4096);
        p.push(&data_le(12));
        p.push(&dc);
        let mut out = [0u8; 32];
        assert_eq!(p.try_next(&mut out).unwrap(), Some(ParsedDeviceFrame::Data { atoms_len: 12 }));
        assert_eq!(&out[..12], &dc);
    }

    #[test]
    fn parses_data_split_across_transfers() {
        let cp = bmd_atem_atom::encode_atom_change_program_input(0, 3);
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 4096);
        p.push(&data_le(24));
        p.push(&cp);
        assert_eq!(p.try_next(&mut [0u8; 32]).unwrap(), None);
        p.push(&cp);
        let mut out = [0u8; 32];
        assert_eq!(p.try_next(&mut out).unwrap(), Some(ParsedDeviceFrame::Data { atoms_len: 24 }));
        assert_eq!(&out[..12], &cp);
        assert_eq!(&out[12..24], &cp);
    }

    #[test]
    fn parses_data_be_at_end_layout() {
        let dc = encode_atom_do_transition_cut(0);
        let raw = data_be(12);
        let mut full = raw.to_vec();
        full.extend_from_slice(&dc);
        let mut p = StreamParser::new(DataLenLayout::BeAtEnd, 4096);
        p.push(&full);
        let mut out = [0u8; 32];
        assert_eq!(p.try_next(&mut out).unwrap(), Some(ParsedDeviceFrame::Data { atoms_len: 12 }));
        assert_eq!(&out[..12], &dc);
    }

    #[test]
    fn tolerates_unknown_types_and_garbage() {
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 4096);
        p.push(&[0x05, 0x00, 0x01, 0x00]);
        assert_eq!(
            p.try_next(&mut [0u8; 8]).unwrap(),
            Some(ParsedDeviceFrame::Unknown { pkt_type: 0x05 })
        );
        // pyatem's typo'd flow frame (`b'x00\x00...'`) must not tear the stream down.
        p.push(&b"ABCDEF\x00\x00\x00"[..]);
        p.push(&init_ack());
        assert_eq!(p.try_next(&mut [0u8; 8]).unwrap(), Some(ParsedDeviceFrame::InitAck));
    }

    #[test]
    fn oversized_blob_resyncs_and_errors() {
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 32);
        p.push(&data_le(0x0100));
        assert_eq!(p.try_next(&mut [0u8; 64]), Err(FrameError::InvalidLength));
        // After the error the parser has resynchronised.
        p.push(&init_ack());
        assert_eq!(p.try_next(&mut [0u8; 8]).unwrap(), Some(ParsedDeviceFrame::InitAck));
    }

    #[test]
    fn zero_length_blob_is_delivered() {
        let mut p = StreamParser::new(DataLenLayout::LeAtStart, 4096);
        p.push(&data_le(0));
        assert_eq!(
            p.try_next(&mut [0u8; 8]).unwrap(),
            Some(ParsedDeviceFrame::Data { atoms_len: 0 })
        );
    }
}
