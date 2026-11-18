# ATEM USB-C control link (8.6+ protocol)

Wire format of the Blackmagic Design ATEM switcher's USB-C control link
("8.6+" protocol generation) as implemented by `embassy_bmd_atem::usb`, with
provenance. Statements marked **[LIVE]** were verified against real hardware
(ATEM Mini, `0x1EDB:0xBE49`, firmware 9.x, 2026-11, Windows + WinUSB).

## Sources

- **OpenSwitcher / pyatem** ([git.sr.ht/~martijnbraam/pyatem](https://git.sr.ht/~martijnbraam/pyatem),
  docs at [docs.openswitcher.org/usbtransport.html](https://docs.openswitcher.org/usbtransport.html)):
  the only public implementation of the USB link (LGPL-3.0; only wire-format
  **facts** are used here — no source code was copied, see `NOTICE`).
  Where its shipped code and its documentation table disagree, the code was
  used (and the live capture confirmed the code).
- **necromancer** ([micolous/necromancer](https://github.com/micolous/necromancer),
  Apache-2.0): UDP/BURP counterpart and atom record definitions.
- Enumeration dump of an ATEM Mini Pro
  ([linux-usb list, 2024](https://www.spinics.net/lists/linux-usb/msg254858.html)).
- **Live captures** in `captures/` (gitignored) taken with the crate's own
  `--raw-dump` facility during bring-up.

## Physical / enumeration facts **[LIVE]**

Composite USB device (IAD `0xEF/0x02/0x01`), **not** CDC-ACM. Verified layout
on an ATEM Mini (matches the 2024 linux-usb dump modulo interface ordering):

| Interface | Class        | Role                                        |
|-----------|--------------|---------------------------------------------|
| 0         | `0xFF/03/00` | vendor-specific (no endpoints)              |
| 1         | `0xFF/02/00` | vendor-specific, bulk `0x01`/`0x81`        |
| **2**     | `0xFF/05/ff` | **ATEM control**: bulk OUT `0x02` / IN `0x82` |
| 3         | `0xFE/01/01` | DFU firmware update                         |
| 4–5       | `0x0E/*`     | UVC webcam control / stream                 |
| 6–7       | `0x01/*`     | UAC soundcard control / stream              |

- VID `0x1EDB`. Known switcher PIDs: `0xBE49` ATEM Mini, `0xBE55` Mini Pro,
  `0xBE56` Mini Pro ISO, `0xBE7C` Mini Extreme, `0xBE83` Mini Extreme ISO,
  `0xBE9E` TVS HD8 ISO. Micro Converters (`0xBE89–0xBE91`) use a different,
  control-transfer-only protocol — **unsupported**.
- No baud/DTR/RTS; a *host* supplies VBUS and claims the control interface.
- **Windows**: the vendor driver (`switchers.inf` family) binds the control
  interface out of the box and rejects user-space claims ("incompatible
  driver is installed"); bind the interface to **WinUSB** (Zadig) for
  nusb/libusb access. Linux needs udev write access; macOS works as-is.

## Protocol generations

Two generations exist ([OpenSwitcher docs](https://docs.openswitcher.org/usbtransport.html)):

- **≤ firmware 8.5**: vendor class control transfer opens the link, bulk
  transfers carry `[u32 LE chunk length][atoms]`. **Not supported by this
  crate** (the 8.6+ handshake times out on such firmware).
- **8.6+**: no control transfer; 4-byte type headers with credit-based flow
  control. **This crate's target**, verified live.

## 8.6+ tunnel framing

Every tunnel packet starts with a 4-byte header; host → device packets are
`00 <type> 00 01` (pyatem's `write()`), device → host packets are
`<type> 00 01 00`:

| Type | Direction | Body | Meaning |
|------|-----------|------|---------|
| `0x01` | host → dev | `00 01 10 92` (meaning unknown) | start a new connection **[LIVE]** |
| `0x02` | dev → host | none (4-byte header) | acknowledge the init **[LIVE]** |
| `0x03` | both | RX-buffer status in 1024 B blocks (see below) | flow control **[LIVE]** |
| `0x04` | both | length of the chunk that follows | announce an atoms chunk **[LIVE]** |
| `0x05` | host → dev | none | heartbeat/poll (suspected ASC keepalive) **[LIVE]** |
| `0x06` | dev → host | 2 zero bytes (6-byte frame) | response to `0x05` **[LIVE]** |

Host → device `0x03`/`0x04` frames are 12 bytes: header + six zero bytes +
big-endian `u16` at offset 10 (pyatem `'>6xH'`) **[LIVE]**.

Device → host `0x03`/`0x04` frames are 12 bytes with the `u16` **big-endian
at offset 4** (`0x03`, block count) / **little-endian at offset 4** (`0x04`,
chunk length), the remaining bytes zero — exactly pyatem's shipped parser,
contradicting the docs table **[LIVE]**.

## Connection sequence **[LIVE]**

```
host                                         ATEM
  |-- OUT 00 01 00 01 | 00 01 10 92 ---------->|  (0x01 init)
  |<-- IN  02 00 01 00 ----------------------|  (0x02 ack)
  |<-- IN  03 00 01 00 | 00 01 00 00 … ------|  (0x03: switcher status, 1 block)
  |-- OUT 00 03 00 01 | 00… 00 04 ----------->|  (0x03: grant 4×1024 B of receive credit)
  |<-- IN  04 00 01 00 | 00 10 00 00 … ------|  (0x04: chunk of 0x1000 B)
  |<-- IN  <2048 B>…<2048 B><ZLP> ------------|  (chunk payload, short-packet terminated)
  |-- OUT 00 03 00 01 | 00… 00 04 ----------->|  (re-grant credit after consuming)
  |<-- IN  04 … (next chunk) …                |
  … repeat until the dump's last chunk (short, no ZLP) …
  |<-- IN  STALL -----------------------------|  (nothing more to send right now)
  |-- (host clears the halt; link stays fully operational)
  |-- OUT 00 04 00 01 | 00… 00 0c ----------->|  (command: DCut announce)
  |-- OUT  <12 B atoms> --------------------->|  (DCut record)
  |<-- IN  04 … <PrgI/TPrvI state atoms> … ---|  (switcher response)
  |-- OUT 00 05 00 01 ----------------------->|  (heartbeat probe)
  |<-- IN  06 00 01 00 00 00 -----------------|  (heartbeat response)
```

### Flow control (credit model) **[LIVE]**

The host's `0x03` announces **receive credit** in 1024-byte blocks. Verified
behaviour: granting 4 blocks makes the switcher stream exactly 4096 bytes
(one `0x04` chunk) and wait; re-granting after each consumed chunk keeps the
dump flowing. Without any grant the switcher never starts the dump (its
pending read eventually STALLs). The switcher also sends its own `0x03`
(observed: 1 block) right after the ack — semantics unknown, currently
logged only.

### The initial state dump is a continuous atom stream **[LIVE]**

Each `0x04` announces one chunk (observed: `0x1000` bytes, delivered as
2048+2048 bulk IN transfers, terminated by a ZLP because 4096 is an exact
multiple of the 512-byte max packet size; the final chunk was short and had
no ZLP). **Atom records straddle chunk boundaries** — the dump is a
continuous stream and must be reassembled across chunks (`StreamParser` does
this; a mid-record boundary was observed live at byte 4096). Observed dump
size: ~10.5 KB over three chunks (`_ver`, `_pin`, `_top`, `_MeC`, `AEBP`,
`AIXP`, `CCdP`, … plus command-response atoms arriving inline).

### Bulk-IN STALL means "idle", not failure **[LIVE]**

When the switcher has nothing more to send, a pending bulk IN read STALLs.
The correct recovery is `CLEAR_FEATURE(ENDPOINT_HALT)` + continue: after the
clear, command writes and their state responses still flow (verified:
credit `0x03`, `0x05`→`0x06`, and full command round-trips post-stall).
pyatem instead treats a read stall as fatal — that is why its USB rx thread
dies after the dump.

Bare **zero-length OUT packets also STALL** on real hardware (live-verified)
— the keepalive must never be a ZLP. The suspected real keepalive is the
`0x05`/`0x06` pair; its pacing is still an open question (a 1 s interval
stalled the OUT endpoint on the second probe in one run — likely needs the
right cadence or sequencing).

## Reliability model

USB bulk transfers are reliable and ordered, so — unlike UDP/BURP — there is
no session id, no ACK queue, no retransmission and no packet numbering. Loss
recovery is `0x01` re-handshake after unplug/replug.

## Open questions

1. `0x05` heartbeat semantics and safe cadence (ASC's idle behaviour).
2. Whether the switcher pushes spontaneous state events (e.g. panel button
   presses) with only an outstanding read, or only after a poll/command.
3. The meaning of the switcher's own `0x03` (1 block) after the ack.
4. Whether the remaining switcher state (media pool, keys, …) arrives on
   demand or only in later dumps.
5. The meaning of the `00 01 10 92` init payload.

## Capture attempt log (2026-11-18)

An attempt to record ATEM Software Control's own USB traffic (the definitive
way to settle the polling discipline) hit two blockers worth recording:

1. **USBPcap captured 0 packets on every root hub** — this PC exposes five
   `USB\ROOT_HUB30` (xHCI/USB 3.0) controllers and no USB 2.0 companion
   controller. USBPcap's xHCI support is known to be unreliable, so the tool
   enumerated `\\.\USBPcap1..4` and produced header-only pcap files. Captures
   must be taken on a machine with a USB 2.0 root hub, or on Linux with
   `usbmon`.
2. **ATEM Software Control connected over the network, not USB** — with both
   cables attached it opened UDP sockets (ephemeral ports) and generated no USB
   traffic at all. To capture its USB behaviour the ATEM's Ethernet cable has
   to be disconnected (or the USB device selected explicitly in ASC), so that
   USB is the only control path.

Both are environmental, not protocol, issues. The recovery procedure in
`capture_asc.ps1` (elevated `pnputil /remove-device` + `/scan-devices`)
successfully rebinds the Blackmagic driver after a Zadig WinUSB override, which
is the prerequisite for ASC-side captures.

### pyatem's transmit discipline (to emulate if captures stay unavailable)

`pyatem`'s `UsbProtocol` never leaves the OUT pipe idle:

- idle loop: write a **zero-length OUT packet** and sleep 20 ms;
- while traffic is recent (`< 10 ms`) or more than 1 KiB is buffered: write a
  `0x03` flow frame every ~10 ms;
- commands: `0x04` announce, then the payload, with a 4 ms sleep after each.

This "always keep writing" pattern is the most likely reason the switcher never
idles out on real software, and is worth implementing as an explicit discipline
mode once captures confirm it.
