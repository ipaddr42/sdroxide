//! Synthesise a VDL Mode 2 recording, so the whole chain can be exercised
//! without an aerial.
//!
//! ```text
//! cargo run --release -p sdroxide-vdl2 --example vdl2_iq -- /tmp/vdl2.iq
//! sdroxide --file /tmp/vdl2.iq --rate 2400000 --freq 136825000 --mode VDL2
//! ```
//!
//! Interleaved little-endian `f32` — what `--file` reads and what an RTL-SDR
//! delivers — at 2.4 Msps centred on 136.825 MHz, so every channel of the plan
//! is inside it. The file loops, so a few seconds is a band that keeps talking.
//!
//! # It proves the chain, not the decoder
//!
//! The transmitter here and the receiver it feeds were written by the same
//! hand, so agreeing with each other proves only that they agree. What this
//! *is* good for is proving the plumbing — the engine's window placement, the
//! per-channel downconverters, the panel, the wire protocol — and for making
//! the decoder fail in the ways a real band does. Every station below is given
//! a fractional start time, a carrier offset, a symbol clock error and a signal
//! level of its own, and one transmits with the other pulse shape, because that
//! turns up on real front ends. Spectral inversion is *not* one of the things
//! varied here: a mirrored front end mirrors the whole stream, which moves every
//! station to the other side of the centre and onto a different channel — it is
//! a property of the receiver, not of a transmission, and the decoder's handling
//! of it is proved in `channel`'s own tests instead.
//!
//! For evidence about the decoder itself there is no substitute for an aerial
//! and somebody else's recording.

use std::io::Write;

use sdroxide_dsp::Complex32;
use sdroxide_types::{VDL2_CHANNELS_HZ, VDL2_PLAN_CENTER_HZ, Vdl2Acars, Vdl2AddrKind, Vdl2Frame};
use sdroxide_vdl2::tx::{Noise, Shape, TxParams, modulate_at};
use sdroxide_vdl2::{acars, avlc, xid};

const RATE: f64 = 2_400_000.0;
const SECONDS: f64 = 4.0;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "/tmp/vdl2.iq".to_string());
    let n = (RATE * SECONDS) as usize;
    let mut iq = vec![Complex32::default(); n];
    let mut rng = Noise::new(0x5644_4c32);

    // Ground stations first: on the air often, strong, and the thing that makes
    // a band look alive before any aircraft answers.
    let gs_a = 0x10_20_30;
    let gs_b = 0x10_20_31;
    let mut at = 0.03 * RATE;

    // The Common Signalling Channel beacon, twice.
    for k in 0..2 {
        let x = ground_beacon(gs_a, 48.1, 16.6, &[136.975, 136.875]);
        put(&mut iq, &x, VDL2_CHANNELS_HZ[6], at, 0.60, 1200.0, 4.0, Shape::Rc, false);
        at += (0.42 + 0.05 * f64::from(k)) * RATE;
    }

    // An aircraft establishing a link, and the ground station accepting.
    let ac_a = 0x44_0F_31; // an Austrian registration
    put(
        &mut iq,
        &link_request(ac_a, gs_a),
        VDL2_CHANNELS_HZ[6],
        0.20 * RATE + 0.37,
        0.30,
        -2100.0,
        -12.0,
        Shape::Rc,
        false,
    );
    put(
        &mut iq,
        &link_accept(gs_a, ac_a),
        VDL2_CHANNELS_HZ[6],
        0.26 * RATE + 0.81,
        0.55,
        900.0,
        3.0,
        Shape::Rc,
        false,
    );

    // Company messages on the ground channels, at descending signal levels: the
    // weakest is near where the correlator gives up, which is where a change to
    // the sync threshold shows itself.
    let flights: [(u32, &str, &str, &str, f32, f64); 4] = [
        (0x44_0F_31, "OE-LWA", "AUA123", "REQUEST DESCENT FL240", 0.50, 136.725),
        (0x3C_65_A2, "D-AIZP", "DLH456", "/POS N48123E016456,FL350,M082", 0.30, 136.775),
        (0x40_63_1E, "G-EUUU", "BAW789", "ETA LOWW 1432Z FUEL 4.2", 0.16, 136.875),
        (0x48_41_7B, "PH-BGA", "KLM012", "WX REQ LOWW LOWG", 0.08, 136.925),
    ];
    let mut t = 0.55 * RATE;
    for (i, &(addr, reg, flight, text, amp, mhz)) in flights.iter().enumerate() {
        let ch = mhz * 1e6;
        let downlink = downlink_acars(addr, gs_b, reg, flight, text);
        // One of them shaped the other way, because a transmitter that splits
        // the root across both ends is a thing a receiver meets.
        let shape = if i == 2 { Shape::Rrc } else { Shape::Rc };
        put(
            &mut iq,
            &downlink,
            ch,
            t + 0.31,
            amp,
            -1500.0 + 900.0 * i as f64,
            -18.0 + 9.0 * i as f64,
            shape,
            false,
        );
        // ...and the ground station's acknowledgement a moment later.
        put(
            &mut iq,
            &uplink_ack(gs_b, addr, reg),
            ch,
            t + 0.09 * RATE + 0.63,
            0.55,
            400.0,
            2.0,
            Shape::Rc,
            false,
        );
        t += 0.6 * RATE;
    }

    // Two stations colliding on the Common Signalling Channel, a hundred
    // symbols apart: the search has to carry on past the first.
    let a = downlink_acars(0x44_0F_31, gs_a, "OE-LWA", "AUA123", "COLLISION ONE");
    let b = downlink_acars(0x3C_65_A2, gs_a, "D-AIZP", "DLH456", "COLLISION TWO");
    let c1 = 2.9 * RATE;
    put(&mut iq, &a, VDL2_CHANNELS_HZ[6], c1 + 0.17, 0.42, 300.0, 0.0, Shape::Rc, false);
    put(&mut iq, &b, VDL2_CHANNELS_HZ[6], c1 + 0.06 * RATE, 0.38, -700.0, 0.0, Shape::Rc, false);

    // ...and one frame long enough to need more than one Reed-Solomon block, so
    // the interleaving is exercised at all. Real traffic almost never is.
    let long = downlink_acars(
        0x48_41_7B,
        gs_b,
        "PH-BGA",
        "KLM012",
        &format!("FLIGHT PLAN {}", "WPT/N48123E016456/FL350 ".repeat(24)),
    );
    put(
        &mut iq,
        &long,
        VDL2_CHANNELS_HZ[3],
        3.3 * RATE + 0.44,
        0.45,
        250.0,
        -6.0,
        Shape::Rc,
        false,
    );

    // The noise floor last, over everything.
    rng.add(&mut iq, 0.006);

    let mut f = std::io::BufWriter::new(std::fs::File::create(&path).expect("create output"));
    let mut buf = Vec::with_capacity(iq.len() * 8);
    for z in &iq {
        buf.extend_from_slice(&z.re.to_le_bytes());
        buf.extend_from_slice(&z.im.to_le_bytes());
    }
    f.write_all(&buf).expect("write");
    f.flush().expect("flush");
    println!(
        "wrote {path}: {SECONDS:.0} s of complex f32 at {:.1} Msps, centred on {:.3} MHz",
        RATE / 1e6,
        VDL2_PLAN_CENTER_HZ / 1e6
    );
    println!("  sdroxide --file {path} --rate {} --freq 136825000 --mode VDL2", RATE as u64);
}

/// Modulate one frame onto one channel of the plan.
#[allow(clippy::too_many_arguments)]
fn put(
    iq: &mut Vec<Complex32>,
    frame: &[u8],
    channel_hz: f64,
    at: f64,
    amplitude: f32,
    freq_err_hz: f64,
    clock_ppm: f64,
    shape: Shape,
    inverted: bool,
) {
    let p = TxParams {
        sample_rate: RATE,
        // The channel's own offset from the recording's centre, plus whatever
        // this station's crystal is doing.
        freq_offset_hz: channel_hz - VDL2_PLAN_CENTER_HZ + freq_err_hz,
        clock_ppm,
        amplitude,
        shape,
        inverted,
        ramp_syms: 5,
    };
    let len = iq.len();
    modulate_at(frame, &p, at, iq);
    // Never grow the file: a burst that would run off the end is simply
    // truncated, the way one straddling the end of a recording is.
    iq.truncate(len);
}

fn ground(addr: u32) -> avlc::Address {
    avlc::Address { addr, kind: Vdl2AddrKind::GroundAdmin, cr: false }
}

fn aircraft(addr: u32) -> avlc::Address {
    avlc::Address { addr, kind: Vdl2AddrKind::Aircraft, cr: false }
}

/// A ground station announcing itself to everybody.
fn ground_beacon(gs: u32, lat: f64, lon: f64, freqs: &[f64]) -> Vec<u8> {
    let mut params: Vec<u8> = Vec::new();
    params.extend_from_slice(&[0x01, 0x01, 0b0011]); // connection management: GSIF
    params.extend_from_slice(&[0x81, 0x01, 0x02]); // modulation support: VDL-M2
    params.push(0xc8); // ground station location
    params.push(3);
    params.extend_from_slice(&location_octets(lat, lon));
    params.push(0xc0); // frequency support
    params.push((freqs.len() * 6) as u8);
    for &f in freqs {
        params.extend_from_slice(&xid::frequency_octets(f, 2));
        params.extend_from_slice(&avlc::address_octets(ground(gs), false));
    }
    let glen = params.len() as u16;
    let mut payload = vec![xid::FMT_ID, xid::GID_PRIVATE, (glen >> 8) as u8, (glen & 0xff) as u8];
    payload.extend_from_slice(&params);

    avlc::build(
        avlc::Address { addr: 0xff_ff_ff, kind: Vdl2AddrKind::AllStations, cr: false },
        ground(gs),
        avlc::control_octet(Vdl2Frame::Xid { pf: false }),
        &payload,
    )
}

/// An aircraft asking a ground station for a link.
fn link_request(ac: u32, gs: u32) -> Vec<u8> {
    let params = [0x01u8, 0x01, 0b0000, 0x03, 0x01, 0x01, 0x81, 0x01, 0x02];
    let glen = params.len() as u16;
    let mut payload = vec![xid::FMT_ID, xid::GID_PRIVATE, (glen >> 8) as u8, (glen & 0xff) as u8];
    payload.extend_from_slice(&params);
    avlc::build(
        avlc::Address { addr: gs, kind: Vdl2AddrKind::GroundAdmin, cr: false },
        aircraft(ac),
        avlc::control_octet(Vdl2Frame::Xid { pf: true }),
        &payload,
    )
}

/// ...and the ground station saying yes.
fn link_accept(gs: u32, ac: u32) -> Vec<u8> {
    let params = [0x01u8, 0x01, 0b0000, 0x03, 0x01, 0x01];
    let glen = params.len() as u16;
    let mut payload = vec![xid::FMT_ID, xid::GID_PRIVATE, (glen >> 8) as u8, (glen & 0xff) as u8];
    payload.extend_from_slice(&params);
    avlc::build(
        avlc::Address { addr: ac, kind: Vdl2AddrKind::Aircraft, cr: true },
        ground(gs),
        avlc::control_octet(Vdl2Frame::Xid { pf: true }),
        &payload,
    )
}

fn downlink_acars(ac: u32, gs: u32, reg: &str, flight: &str, text: &str) -> Vec<u8> {
    let a = Vdl2Acars {
        mode: '2',
        registration: reg.to_string(),
        ack: '\x15',
        label: "H1".to_string(),
        block_id: '1',
        msn: "M01A".to_string(),
        flight: flight.to_string(),
        text: text.to_string(),
        ..Vdl2Acars::default()
    };
    avlc::build(
        avlc::Address { addr: gs, kind: Vdl2AddrKind::GroundAdmin, cr: false },
        aircraft(ac),
        avlc::control_octet(Vdl2Frame::I { ns: 0, nr: 0, p: false }),
        &acars::build(&a, true),
    )
}

fn uplink_ack(gs: u32, ac: u32, reg: &str) -> Vec<u8> {
    let a = Vdl2Acars {
        mode: '2',
        registration: reg.to_string(),
        ack: '1',
        label: "_d".to_string(),
        block_id: '1',
        ..Vdl2Acars::default()
    };
    avlc::build(
        avlc::Address { addr: ac, kind: Vdl2AddrKind::Aircraft, cr: true },
        ground(gs),
        avlc::control_octet(Vdl2Frame::I { ns: 0, nr: 1, p: false }),
        &acars::build(&a, false),
    )
}

/// The three octets an XID location parameter carries: twelve signed bits of
/// latitude then twelve of longitude, both in tenths of a degree.
fn location_octets(lat: f64, lon: f64) -> [u8; 3] {
    let la = ((lat * 10.0).round() as i32) & 0xfff;
    let lo = ((lon * 10.0).round() as i32) & 0xfff;
    [(la >> 4) as u8, (((la & 0xf) << 4) | ((lo >> 8) & 0xf)) as u8, (lo & 0xff) as u8]
}
