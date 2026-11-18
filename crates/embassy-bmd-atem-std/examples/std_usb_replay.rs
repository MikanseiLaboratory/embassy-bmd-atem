//! Host example: parse a recorded USB tunnel capture without hardware.
//!
//! Input is a text file with one bulk transfer per line (hex, any separators —
//! e.g. the `usb.capdata` column of a tshark export); blank lines and `#`
//! comments are ignored.
//!
//! ```text
//! cargo run -p embassy-bmd-atem-std --example std_usb_replay -- captures/switcher.hex
//! ```
use std::path::PathBuf;

use clap::{Parser, ValueEnum};
use embassy_bmd_atem::usb::protocol::{DataLenLayout, ParsedDeviceFrame, StreamParser};

/// Decode of the device → host `0x04` length field.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum LayoutOpt {
    /// Little-endian right after the header (pyatem RX parse).
    #[value(name = "le-start")]
    LeStart,
    /// Big-endian at the body end (docs table).
    #[value(name = "be-end")]
    BeEnd,
}

#[derive(Parser, Debug)]
#[command(name = "std_usb_replay", version, about = "Parse a recorded ATEM USB tunnel capture")]
struct Cli {
    /// Replay file (one bulk transfer per line).
    file: PathBuf,
    /// Device → host `0x04` length layout (`le-start` = pyatem RX parse).
    #[arg(long, default_value = "le-start")]
    layout: LayoutOpt,
    /// Maximum accepted blob size.
    #[arg(long, default_value_t = 0x1_0000)]
    max_atoms_len: usize,
}

fn main() {
    let cli = Cli::parse();
    let layout = match cli.layout {
        LayoutOpt::LeStart => DataLenLayout::LeAtStart,
        LayoutOpt::BeEnd => DataLenLayout::BeAtEnd,
    };
    let transfers = embassy_bmd_atem_std::load_replay_file(&cli.file)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", cli.file.display()));
    println!("loaded {} transfer(s)", transfers.len());

    let mut parser = StreamParser::new(layout, cli.max_atoms_len);
    let mut out = vec![0u8; cli.max_atoms_len];
    for transfer in &transfers {
        parser.push(transfer);
        loop {
            match parser.try_next(&mut out) {
                Ok(Some(ParsedDeviceFrame::InitAck)) => println!("0x02 init-ack"),
                Ok(Some(ParsedDeviceFrame::Data { atoms_len })) => {
                    let mut names = Vec::new();
                    for atom in bmd_atem_atom::AtomIter::new(&out[..atoms_len]).flatten() {
                        names.push(atom.name.display_lossy().to_string());
                    }
                    println!("0x04 atoms({atoms_len} B): {}", names.join(", "));
                }
                Ok(Some(ParsedDeviceFrame::FlowCtrl { blocks })) => {
                    println!("0x03 flow: {blocks} block(s)");
                }
                Ok(Some(ParsedDeviceFrame::Unknown { pkt_type })) => {
                    println!("0x{pkt_type:02x}? unknown type");
                }
                Ok(None) => break,
                Err(e) => {
                    eprintln!("framing error: {e}");
                    break;
                }
            }
        }
    }
}
