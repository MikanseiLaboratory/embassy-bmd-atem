# embassy-bmd-atem

[![CI](https://github.com/MikanseiLaboratory/embassy-bmd-atem/actions/workflows/ci.yml/badge.svg)](https://github.com/MikanseiLaboratory/embassy-bmd-atem/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/embassy-bmd-atem.svg)](https://crates.io/crates/embassy-bmd-atem)

Blackmagic Design ATEM switcher control protocol for embedded Rust and std,
in one workspace:

- **`bmd-atem-atom`** — transport-independent atom (command record) codec
  ([crates.io](https://crates.io/crates/bmd-atem-atom)).
- **`embassy-bmd-atem`** — `no_std` protocol core with two transports:
  - `udp` — the LAN **BURP** protocol (port 9910): 12-byte packet header,
    connect handshake, cumulative-ACK queue, retransmission helpers, and
    `edge-nal`-style async UDP traits (continuation of the archived
    [edge-bmd-atem](https://github.com/MikanseiLaboratory/edge-bmd-atem)).
  - `usb` — the **USB-C "8.6+" control link, host side**: plug your
    microcontroller (or PC) into the switcher's USB port and speak the tunnel
    protocol. See [`docs/PROTOCOL_USB.md`](docs/PROTOCOL_USB.md).
- **`embassy-bmd-atem-std`** — Tokio + [nusb](https://crates.io/crates/nusb)
  desktop backend and the `atem-usb-control` CLI
  ([crates.io](https://crates.io/crates/embassy-bmd-atem-std)).
- **`examples/rp2040-usb-atem`** — RP2040 example wiring the USB host session
  (transport transport integration lands with real hardware bring-up).

## Quick start

### Control an ATEM over USB from a PC

```sh
cargo install embassy-bmd-atem-std   # or: cargo run -p embassy-bmd-atem-std --bin atem-usb-control

atem-usb-control --list                # enumerate VID 0x1EDB devices
atem-usb-control --verbose             # connect, dump initial state
atem-usb-control --send "CPgI 0 3"     # put input 3 on program of ME1
atem-usb-control --send "DCut 0"       # CUT
atem-usb-control --send "DAut 0"       # AUTO transition
atem-usb-control --send "FtbA 0"       # fade to black
```

- **Windows**: the switcher's vendor control interface must be bound to
  **WinUSB** (e.g. with [Zadig](https://zadig.akeo.ie/)) for nusb access.
  Replacing the driver stops ATEM Software Control from using that interface
  until reverted. Linux/macOS need no driver changes (Linux needs udev write
  access on `/dev/bus/usb`).
- Only one process may claim the control interface: close ATEM Software Control
  first.
- Protocol support is the **8.6+ USB generation** (ATEM Mini family on
  firmware ≥ 8.6). Older firmware simply times out on the handshake.

### UDP from an MCU (edge-nal style)

```rust,ignore
let mut session = embassy_bmd_atem::AtemSession::connect(
    &mut udp, remote, 0x2970, SessionConfig::default(),
    || t.elapsed().as_millis() as u32,
).await?;
session.send_atoms(&embassy_bmd_atem::encode_atom_change_program_input(0, 3), now);
```

### USB host session (embedded)

```rust,ignore
let mut session = embassy_bmd_atem::AtemUsbSession::new(bulk_backend, UsbSessionConfig::default());
session.connect(now_ms()).await?;
loop {
    match session.poll(now_ms(), &mut buf).await? {
        Some(SessionEvent::Atoms { block, first }) => { /* parse AtomIter::new(block) */ }
        Some(SessionEvent::FlowCtrl { blocks }) => { … }
        Some(SessionEvent::InitAck) | Some(SessionEvent::Unknown { .. }) => {}
        None => {}
    }
}
session.send_atoms(&cmd_atoms, now_ms()).await?;
```

The USB session only needs a [`UsbBulkHost`](crates/embassy-bmd-atem/src/usb/io.rs)
implementation (async bulk read/write + clear-halt + delay). The desktop
implementation lives in `embassy-bmd-atem-std` (`nusb`); embedded backends
(e.g. `cotton-usb-host` on RP2040) wrap the same trait.

## Status

- [x] UDP/BURP transport (inherited from edge-bmd-atem, tested)
- [x] USB 8.6+ framing, session FSM, flow control, keepalives (unit-tested
      against pyatem wire shapes)
- [x] Desktop `nusb` backend + CLI
- [x] RP2040 example scaffold (`thumbv6m-none-eabi` CI build)
- [ ] Hardware capture to pin the open framing questions in
      [`docs/PROTOCOL_USB.md`](docs/PROTOCOL_USB.md#open-questions-to-be-settled-with-a-hardware-capture)
      (device→host `0x04` byte order, `0x05` semantics)
- [ ] RP2040 full transport bring-up (`cotton-usb-host`) — requires VBUS 5 V
      supply to the ATEM on the target board

`edge-bmd-atem` is archived; its code lives on here under `embassy_bmd_atem::udp`.

## License

Apache-2.0. Protocol facts were derived from the OpenSwitcher/pyatem
(LGPL-3.0) documentation and necromancer (Apache-2.0); no source code was
copied — see [`NOTICE`](NOTICE).
