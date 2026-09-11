//! Two VDL Mode 2 transmissions recorded off the air, replayed through the
//! whole receiver.
//!
//! # Why this file exists
//!
//! Issue #265: the decoder found bursts, locked onto them, and produced not one
//! frame — while its own synthetic generator round-tripped perfectly. It had
//! to, because that generator and this receiver were written together and
//! agreed with each other about two things that were both wrong: which end of a
//! D8PSK symbol its three bits come out of, and whether the data field is
//! wrapped in HDLC flags. A generator can never catch that. Only somebody
//! else's signal can, so here is somebody else's signal.
//!
//! # What is stored, and what it does and does not prove
//!
//! The *symbols* of each burst — one phase increment per symbol, as they came
//! off the air — rather than its samples, which would be a hundred kilobytes
//! per burst instead of a hundred and fifty bytes. `tx::modulate_increments_at`
//! puts them back on a carrier, so this exercises the receive filter, the
//! symbol clock, the synchronisation correlator, the descrambler, the header
//! code, the HDLC unwrapping, the AVLC parse and the frame check sequence.
//!
//! The pulse shape is this program's; everything *carried* is the real
//! transmitter's. So it cannot catch a mistake in the receive filter, and it is
//! exactly what catches a mistake in what the bits mean.
//!
//! Source: a 24-second 2.4 Msps recording centred on 136.8135 MHz, contributed
//! on the issue.

use sdroxide_dsp::Complex32;
use sdroxide_types::Vdl2AddrKind;
use sdroxide_vdl2::channel::ChannelRx;
use sdroxide_vdl2::gate::Burst;
use sdroxide_vdl2::tx::{Shape, TxParams, modulate_increments_at};

/// The channel rate an RTL-SDR at 2.4 Msps lands on: 9.14 samples a symbol.
const RATE: f64 = 96_000.0;

/// One recorded transmission.
struct Recorded {
    channel_hz: f64,
    /// Phase increments in eighths of a turn, from the first symbol of the
    /// synchronisation word to the last of the data field.
    inc: &'static [u8],
    /// The AVLC frame it carries, check sequence included.
    frame: &'static [u8],
}

/// 136.675 MHz, 52 dB above the channel floor: a 41-octet frame
/// in a 352-bit data field, 8 of those bits stuffed.
static BURST_1: Recorded = Recorded {
    channel_hz: 136_675_000.0,
    inc: &[
        0, 3, 2, 4, 0, 1, 6, 4, 1, 7, 2, 5, 6, 5, 7, 3, 0, 7, 5, 7, 6, 5, 1, 4, 0, 4, 7, 1, 6, 4,
        6, 6, 0, 3, 6, 3, 7, 6, 1, 5, 5, 3, 6, 2, 2, 5, 4, 6, 2, 0, 4, 6, 5, 3, 2, 5, 6, 1, 4, 4,
        1, 2, 0, 5, 1, 2, 6, 1, 1, 4, 6, 2, 3, 0, 5, 4, 2, 7, 2, 5, 1, 7, 3, 2, 7, 2, 1, 6, 7, 4,
        6, 4, 6, 6, 1, 5, 6, 5, 5, 2, 7, 7, 6, 6, 6, 7, 6, 5, 4, 0, 7, 4, 6, 1, 4, 5, 5, 1, 3, 6,
        3, 7, 4, 7, 6, 3, 4, 5, 3, 1, 4, 5, 7, 5, 5, 2, 3, 4, 7, 6, 1, 0, 1, 0, 7, 2, 3, 5, 5, 6,
        1, 4, 4,
    ],
    frame: &[
        0x14, 0x42, 0xfc, 0x58, 0x50, 0x80, 0xb6, 0x0d, 0xac, 0xff, 0xff, 0x01, 0x32, 0xae, 0xc7,
        0xad, 0x54, 0xc1, 0x57, 0xd9, 0x45, 0xdf, 0x7f, 0x31, 0x02, 0xd3, 0xb5, 0xb9, 0xc1, 0xc2,
        0xd9, 0xb0, 0x34, 0x4c, 0xd6, 0x83, 0x68, 0x45, 0x7f, 0xb7, 0xe2,
    ],
};

/// 136.975 MHz, 29 dB above the channel floor: a 14-octet frame
/// in a 129-bit data field, 1 of those bits stuffed.
static BURST_2: Recorded = Recorded {
    channel_hz: 136_975_000.0,
    inc: &[
        0, 3, 2, 4, 0, 1, 6, 4, 1, 7, 2, 5, 6, 5, 7, 3, 0, 0, 4, 2, 6, 5, 1, 2, 7, 4, 7, 1, 6, 4,
        6, 6, 0, 1, 5, 7, 1, 2, 1, 1, 4, 7, 7, 6, 7, 2, 0, 6, 3, 5, 3, 7, 5, 7, 3, 6, 7, 4, 7, 6,
        7, 7, 6, 7, 0, 5, 7, 0, 5, 1, 5, 5, 7, 7, 6,
    ],
    frame: &[0x14, 0x42, 0x24, 0x5a, 0x90, 0x8e, 0x16, 0x1d, 0x66, 0x1b, 0xff, 0x21, 0x67, 0x44],
};

/// Put a recorded burst back on a carrier and hand it to the receiver.
fn replay(r: &Recorded) -> Vec<sdroxide_vdl2::channel::Decoded> {
    let p = TxParams {
        sample_rate: RATE,
        ramp_syms: 0,
        shape: Shape::Rc,
        amplitude: 0.5,
        ..TxParams::default()
    };
    let mut iq = Vec::new();
    modulate_increments_at(r.inc, &p, 40.0, &mut iq);
    iq.resize(iq.len() + 200, Complex32::default());
    let mut rx = ChannelRx::new(r.channel_hz, RATE, 9.0);
    let burst =
        Burst { iq, rate_hz: RATE, center_hz: r.channel_hz, snr_db: 30.0, peak_dbfs: -20.0 };
    let mut out = Vec::new();
    rx.decode_burst(&burst, &mut out);
    out
}

/// 136.675 MHz. Every octet has to come out exactly as recorded, and the
/// transmitter's own frame check sequence over them is what says it did.
#[test]
fn a_recorded_transmission_on_136675_khz_decodes() {
    let out = replay(&BURST_1);
    assert_eq!(out.len(), 1, "136.675 MHz: {} frames", out.len());
    assert_eq!(out[0].raw, BURST_1.frame, "the frame came out changed");
    assert_eq!(out[0].frame.dst.addr, 0x109F9A);
    assert_eq!(out[0].frame.dst.kind, Vdl2AddrKind::GroundDelegated);
    assert_eq!(out[0].frame.src.addr, 0x4076B0);
    assert_eq!(out[0].frame.src.kind, Vdl2AddrKind::Aircraft);
}

/// 136.975 MHz. Every octet has to come out exactly as recorded, and the
/// transmitter's own frame check sequence over them is what says it did.
#[test]
fn a_recorded_transmission_on_136975_khz_decodes() {
    let out = replay(&BURST_2);
    assert_eq!(out.len(), 1, "136.975 MHz: {} frames", out.len());
    assert_eq!(out[0].raw, BURST_2.frame, "the frame came out changed");
    assert_eq!(out[0].frame.dst.addr, 0x10925A);
    assert_eq!(out[0].frame.dst.kind, Vdl2AddrKind::GroundDelegated);
    assert_eq!(out[0].frame.src.addr, 0x3C7438);
    assert_eq!(out[0].frame.src.kind, Vdl2AddrKind::Aircraft);
}
