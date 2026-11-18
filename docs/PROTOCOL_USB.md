# ATEM USB-C control link (8.6+ protocol)

This document records the wire format of the Blackmagic Design ATEM switcher's
USB-C control link ("8.6+" protocol generation) as implemented by this crate,
with provenance. It is the reference for `embassy_bmd_atem::usb`.

## Sources

- **OpenSwitcher / pyatem** ([git.sr.ht/~martijnbraam/pyatem](https://git.sr.ht/~martijnbraam/pyatem),
  docs at [docs.openswitcher.org/usbtransport.html](https://docs.openswitcher.org/usbtransport.html)):
  the only public, hardware-verified implementation of the USB link (LGPL-3.0;
  only wire-format **facts** are used here — no code was copied, see `NOTICE`).
- **necromancer** ([micolous/necromancer](https://github.com/micolous/necromancer),
  Apache-2.0): UDP/BURP counterpart and atom definitions.
- Actual enumeration dump of an ATEM Mini Pro
  ([linux-usb mailing list, 2024](https://www.spinics.net/lists/linux-usb/msg254858.html)).

## Physical / enumeration facts

The switcher is a **composite USB device** (IAD, device class `0xEF/0x02/0x01`),
not a CDC-ACM serial port:

| Interface | Class      | Role                                   |
|-----------|------------|----------------------------------------|
| 0..2      | `0xFF`     | vendor-specific (one of them is control) |
| (next)    | `0xFE/01/01` | DFU firmware update                   |
| (next)    | `0x0E/*`   | UVC webcam control / stream             |
| (last)    | `0x01/*`   | UAC soundcard control / stream          |

- Control = a **vendor-class bulk endpoint pair**. Stock ATEM Minis use bulk OUT
  `0x02` and bulk IN `0x82` (pyatem hardcodes them), but the interface **index
  moved between firmware generations** (2 on ~8.x, 1 on ~9.x), so implementations
  should discover the pair from descriptors rather than hardcoding — this is
  what `embassy-bmd-atem-std` does.
- No baud rate, no DTR/RTS, no CDC line coding: those ideas apply to the
  *converter* products, not the ATEM switchers.
- VID `0x1EDB` (Blackmagic design). Known switcher PIDs:
  `0xBE49` ATEM Mini, `0xBE55` ATEM Mini Pro, `0xBE56` ATEM Mini Pro ISO,
  `0xBE7C` ATEM Mini Extreme, `0xBE83` ATEM Mini Extreme ISO,
  `0xBE9E` ATEM Television Studio HD8 ISO.
  (Micro Converters `0xBE89–0xBE91` use a *different*, control-transfer-based
  protocol and are **not supported** by this crate.)

## Protocol generations

Two generations exist ([OpenSwitcher docs](https://docs.openswitcher.org/usbtransport.html)):

- **≤ firmware 8.5**: a vendor class control transfer
  (`bmRequestType 0x21, bRequest 0x00, wValue 0x0000, wIndex = control interface,
  wLength 0`) opens the link, then bulk transfers carry
  `[u32 LE chunk length][concatenated atoms]` chunks.
- **8.6+** (ATEM Mini Extreme era, 2021): **this crate's target**. No control
  transfer; a 4-byte type header scheme with flow control. Devices running
  8.5-era firmware never answer the 8.6+ handshake, so callers see a timeout
  and can treat the device as unsupported.

## 8.6+ tunnel framing

Every tunnel packet starts with 4 bytes:

```
byte 0: packet type for ATEM → PC direction, otherwise 0
byte 1: packet type for PC → ATEM direction, otherwise 0
byte 2: 1 for ATEM → PC packets, 0 otherwise        (observed, undocumented)
byte 3: 1 for PC → ATEM packets, 0 otherwise        (observed, undocumented)
```

pyatem builds all PC → ATEM headers as `00 <type> 00 01`; device → host packets
therefore start with `<type> 00 01 00`. Types:

| Type | Direction   | Body                                     | Meaning |
|------|-------------|------------------------------------------|---------|
| `0x01` | PC → ATEM | `00 01 10 92` (meaning unknown)          | start a new connection |
| `0x02` | ATEM → PC | none (4-byte header only)                | acknowledge the init packet |
| `0x03` | both      | RX-buffer usage                          | flow control (see below) |
| `0x04` | both      | length of the *next* bulk transfer(s)    | announce an atoms blob |
| `0x05` | PC → ATEM | none                                     | unknown (tolerated as noise) |

### Handshake sequence (host side, as implemented)

```
PC(host)                                     ATEM(device)
  |-- BULK OUT 00 01 00 01 | 00 01 10 92 -->     (0x01 init)
  |<-- BULK IN  02 00 01 00                    -- (0x02 ack)
  |-- BULK OUT 00 03 00 01 | 00×6 + 00 04 -->   (0x03: announce our RX buffer, optional)
  |<-- BULK IN  04 00 01 00 | LL LL            -- (0x04 announce)
  |<-- BULK IN  <atoms stream, LL LL bytes>    -- (full initial state dump, unprompted)
  … 0x04/0x03 repeats; host sends ZLP when idle …
```

Notes:

- The **initial state dump is pushed by the switcher** right after the ack —
  there is no USB-side equivalent of the UDP `0xd4` request.
- The host sends **zero-length bulk OUT packets** as keepalive when idle
  (pyatem: whenever its receive wait times out, observed 0.2–0.5 s cadence).
- pyatem paces every USB payload write with ~4 ms sleeps before and after.

### Flow control (`0x03`)

Both directions encode an RX-buffer usage in **1024-byte blocks**. The
OpenSwitcher documentation table and pyatem's shipped code disagree on layout
and byte order, so this crate treats the *code* as primary and tolerates the
mirrored variant:

- host → device: `00 03 00 01 | 00×6 + u16 BE` (pyatem `struct.pack('>6xH', blocks)`),
  written only when the backlog *increases* past a previous announcement.
- device → host: parsed as `u16 BE` right after the header (pyatem's RX parse
  `struct.unpack('>BBBBH6x', ...)` reads the big-endian value at body start),
  with a fallback to the mirrored little-endian-at-offset-10 encoding when the
  primary read yields 0.

### Data blobs (`0x04`)

- host → device: the host writes a 12-byte announce
  (`00 04 00 01 | 00×6 + u16 BE length`), then the atoms blob as the next bulk
  OUT transfer(s).
- device → host: the device announces a length, then streams the blob over one
  or more bulk IN transfers. **Endianness is not pinned by public sources** —
  pyatem reads it as `u16 LE` right after the header
  (`struct.unpack_from('<H', packet, 4)`), while its own *sending* direction
  uses BE-at-end. [`DataLenLayout`](../crates/embassy-bmd-atem/src/usb/protocol.rs)
  selects the primary decoding (`LeAtStart`) and the parser falls back to the
  mirrored encoding; a real capture should settle this (see §Open questions).

The blob content is plain **atom records**, identical to the UDP
"high-level protocol": `u16 BE total length (≥8) | 2 bytes padding | 4-byte
ASCII command name | payload (length−8 bytes)`, records concatenated. The
[`bmd-atem-atom`](../crates/bmd-atem-atom) crate parses/encodes these.

## Reliability model

USB bulk transfers are reliable and ordered, so — unlike UDP/BURP — there is
**no session id, no ACK queue, no retransmission, no packet numbering** on the
USB link. Loss recovery is delegated to re-connecting (`0x01` handshake again
after a transport error / unplug).

## Open questions (to be settled with a hardware capture)

1. Byte order/position of the device → host `0x04` length (LE-at-start vs
   BE-at-end mirrored) — currently both are tolerated.
2. Whether device → host `0x03` frames appear unprompted and at what cadence.
3. The purpose of host → device `0x05` (suspected idle keepalive variant of
   ATEM Software Control).
4. The meaning of the `00 01 10 92` init payload (documented as unknown).
5. Exact `wMaxPacketSize` of the bulk endpoints (64 B full-speed vs 512 B
   high-speed) on specific models.
6. Whether the switcher re-sends the full dump on re-handshake without a
   device reset (expected, per pyatem's `mark_next_connected` behaviour).
