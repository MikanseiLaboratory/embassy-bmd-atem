# rp2040-usb-atem

RP2040 (Pico / W5500-EVB-Pico) firmware: USB **host** to a Blackmagic Design
ATEM switcher over the USB-C control link (8.6+ protocol), using
[cotton-usb-host](https://github.com/pdh11/cotton) for the RP2040 host
controller and `embassy-bmd-atem` for the protocol session.

## What it does

1. Waits for the ATEM to enumerate (VID `0x1EDB`) and claims its vendor-class
   control interface (bulk OUT `0x02` / IN `0x82`).
2. Runs the 8.6+ handshake (`0x01` init → `0x02` ack) and grants receive
   credit (`0x03`, 4×1024 B), re-granting after every consumed chunk.
3. Logs the initial state dump over defmt/RTT (atom names per chunk).
4. Sends one `DCut` (CUT on ME1) three seconds after the first atoms arrive,
   then logs switcher state updates (`PrgI`, `TPrvI`, …).

## Wiring

- **ATEM USB → board USB port** (the board's USB connector is the *host*
  port in this firmware; OTG adapter/cable as needed).
- **VBUS 5 V must be present** on the board's USB port — the ATEM enumerates
  as a USB device and won't appear without host VBUS even though it is
  externally powered. If the board is powered only via SWD (3V3), feed 5 V
  to the VBUS pin from the debug setup or a separate supply.
- **Program/debug over SWD** (Pico Debug Probe or any CMSIS-DAP probe):
  the USB port belongs to the ATEM while running.

## Flash & debug (probe-rs)

```sh
# from this directory (uses .cargo/config.toml: thumbv6m target + probe-rs runner)
cargo run --features firmware

# or explicitly:
cargo build --features firmware
probe-rs run --chip rp2040 target/thumbv6m-none-eabi/debug/rp2040-usb-atem
```

defmt logs appear on the RTT channel (`DEFMT_LOG=info` default). Expected
sequence on a live ATEM:

```
BMD device connected: pid=0xbe49
ATEM control link ready (in=0x82 out=0x02)
0x01 init sent (00 01 10 92), waiting for switcher…
0x02 init ack received
0x03 flow: switcher announced 1 block(s)
first atoms block 1: 4084 B (total 4084 B), 130 fields, first fields [_ver, _pin, _top, _MeC, _mpl, _FAC, _FEC, _VMC]
sent DCut (CUT) on ME1
next atoms block 2: … (AEBP, AIXP, AICP, …)
…
```

If enumeration reports errors, check VBUS first (the ATEM will not enumerate
without 5 V on its USB VBUS).

## Notes

- cotton-usb-host drives the RP2040 USB controller in host mode; the ATEM
  Mini enumerates at Full Speed (12 Mbit/s) — plenty for the control link.
- `rp2040-hal` is pinned to 0.10 on purpose: cotton-usb-host's `rp2040`
  feature is built against `rp2040-pac` 0.6 register types.
- A bulk-IN **STALL is normal** when the switcher has nothing more to send
  (idle signal); the transport clears the halt and continues — see
  `docs/PROTOCOL_USB.md`.
