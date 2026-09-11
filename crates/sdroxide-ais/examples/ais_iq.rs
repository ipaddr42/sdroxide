//! Synthesise a stretch of water to a `.iq` file, so the decoder and the panel
//! can be exercised without an antenna.
//!
//! ```text
//! cargo run --release -p sdroxide-ais --example ais_iq -- /tmp/sea.iq
//! sdroxide --file /tmp/sea.iq --rate 240000 --freq 162000000 --mode AIS
//! ```
//!
//! Interleaved little-endian `f32` pairs (CF32) at 240 kHz, which is what
//! `FileSource` reads and an RTL-SDR's slowest rate. The file loops, so a
//! minute of it is a harbour that keeps moving.
//!
//! # What it is for, and what it is not
//!
//! It proves the plumbing: that a slot modulated the way a transponder
//! modulates one comes back out of the whole chain as a row on the panel and a
//! hull on the chart, with the name and the position it was built from. It does
//! **not** prove the decoder works on air — the transmitter here and the
//! receiver there were written from the same standard by the same hand, and a
//! field offset misread at both ends agrees with itself. For that there is no
//! substitute for an aerial and a comparison against another AIS decoder, which
//! is what the `!AIVDM` line on every card in the panel is for.
//!
//! The ships move: each is given a position, a course and a speed, and its
//! position report is re-encoded every few seconds from where it has got to.
//! They alternate between the two channels, exactly as the standard has them
//! do, and each has a carrier offset and an amplitude of its own — the weakest
//! is near where the gate gives up, which is the part of the picture worth
//! watching. There is also a buoy that does not move, a shore station, and a
//! small craft with a Class B unit that reports its name in two halves.
//!
//! Half a minute at twelve knots is two hundred metres, so the *trails* are
//! there but a whole loop of the file is a pixel or two at the zoom that frames
//! the fleet — zoom the chart in on one ship to see one. Real traffic over a
//! real afternoon is what fills a ten-minute trail.

use std::io::Write;

use sdroxide_ais::tx::{Noise, Payload, TxParams, modulate_bits, shift};
use sdroxide_dsp::Complex32;
use sdroxide_types::{AIS_CHANNEL_A_HZ, AIS_CHANNEL_B_HZ, AIS_PLAN_CENTER_HZ};

const RATE: f64 = 240_000.0;
/// Seconds of traffic before the file loops.
const SECONDS: f64 = 30.0;

/// One vessel, and where it is going.
struct Ship {
    mmsi: u32,
    name: &'static str,
    call: &'static str,
    destination: &'static str,
    ship_type: u64,
    lat: f64,
    lon: f64,
    course_deg: f64,
    speed_kt: f64,
    /// Bow, stern, port, starboard, metres.
    dim: (u64, u64, u64, u64),
    draught_tenths: u64,
    /// A Class B unit: message 18 and the two halves of message 24 instead of
    /// messages 1 and 5.
    class_b: bool,
    /// Amplitude of its slots, full scale being 1.0.
    amp: f32,
    /// Its own carrier error, Hz.
    offset_hz: f64,
}

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "sea.iq".to_string());

    let ships = [
        Ship {
            mmsi: 244_660_123,
            name: "NORDIC LIGHT",
            call: "PBCD",
            destination: "ROTTERDAM",
            ship_type: 80, // tanker
            lat: 51.95,
            lon: 4.05,
            course_deg: 95.0,
            speed_kt: 12.4,
            dim: (240, 40, 20, 24),
            draught_tenths: 128,
            class_b: false,
            amp: 0.55,
            offset_hz: 120.0,
        },
        Ship {
            mmsi: 235_098_765,
            name: "STENA BRITANNICA",
            call: "MDXY",
            destination: "HARWICH",
            ship_type: 60, // passenger
            lat: 51.99,
            lon: 3.90,
            course_deg: 250.0,
            speed_kt: 19.8,
            dim: (180, 60, 15, 15),
            draught_tenths: 68,
            class_b: false,
            amp: 0.30,
            offset_hz: -700.0,
        },
        Ship {
            mmsi: 205_331_000,
            name: "ZEEBRUGGE TRADER",
            call: "ONAB",
            destination: "ANTWERPEN",
            ship_type: 70, // cargo
            lat: 51.88,
            lon: 3.98,
            course_deg: 40.0,
            speed_kt: 9.1,
            dim: (120, 20, 10, 12),
            draught_tenths: 74,
            class_b: false,
            amp: 0.12,
            offset_hz: 1_900.0,
        },
        Ship {
            mmsi: 244_012_345,
            name: "ZEEHOND",
            call: "PDQR",
            destination: "",
            ship_type: 37, // pleasure craft
            lat: 51.93,
            lon: 4.02,
            course_deg: 310.0,
            speed_kt: 5.2,
            dim: (8, 4, 2, 2),
            draught_tenths: 12,
            class_b: true,
            amp: 0.07,
            offset_hz: -2_400.0,
        },
    ];

    let total = (RATE * SECONDS) as usize;
    let mut buf = vec![Complex32::default(); total];
    let mut noise = Noise::new(0x4149_5320);
    noise.add(&mut buf, 0.003);

    // The slot clock. AIS divides the minute into 2250 slots per channel; the
    // exact slot a station claims is its own business, so the placement here is
    // only "one transmission at a time, spread out" rather than a simulation of
    // the access scheme.
    let slot = (RATE * 0.02667) as usize;
    let mut at = slot * 4;
    let mut channel_a = true;
    let mut t = 0.0f64;

    while at + slot * 3 < total {
        // Every ship reports its position on each pass, alternating channels;
        // every fourth pass it also sends the message that names it.
        let statics = ((t / 6.0) as usize).is_multiple_of(4);
        for s in &ships {
            let (lat, lon) = advanced(s, t);
            place(&mut buf, at, &position(s, lat, lon), s, channel_a);
            at += slot * 2;
            channel_a = !channel_a;
            if at + slot * 3 >= total {
                break;
            }
            if statics {
                for bits in static_reports(s) {
                    place(&mut buf, at, &bits, s, channel_a);
                    at += slot * 3;
                    channel_a = !channel_a;
                    if at + slot * 3 >= total {
                        break;
                    }
                }
            }
        }
        // ...and the shore: a base station with the time, and a buoy that has
        // not moved since it was laid.
        if at + slot * 3 < total {
            place(&mut buf, at, &base_station(t), &SHORE, channel_a);
            at += slot * 2;
            channel_a = !channel_a;
        }
        if at + slot * 3 < total {
            place(&mut buf, at, &buoy(), &SHORE, channel_a);
            at += slot * 2;
            channel_a = !channel_a;
        }
        t += (slot * 2) as f64 * ships.len() as f64 / RATE + 0.4;
    }

    let mut out = std::io::BufWriter::new(std::fs::File::create(&path).expect("create the file"));
    for z in &buf {
        out.write_all(&z.re.to_le_bytes()).expect("write");
        out.write_all(&z.im.to_le_bytes()).expect("write");
    }
    out.flush().expect("flush");
    println!(
        "wrote {path}: {SECONDS:.0} s of {} ships, a base station and a buoy at {} sps\n\
         try it with:\n  sdroxide --file {path} --rate {} --freq {} --mode AIS",
        ships.len(),
        RATE,
        RATE,
        AIS_PLAN_CENTER_HZ
    );
}

/// The shore stations' stand-in ship, for the fields `place` needs.
const SHORE: Ship = Ship {
    mmsi: 0,
    name: "",
    call: "",
    destination: "",
    ship_type: 0,
    lat: 0.0,
    lon: 0.0,
    course_deg: 0.0,
    speed_kt: 0.0,
    dim: (0, 0, 0, 0),
    draught_tenths: 0,
    class_b: false,
    amp: 0.45,
    offset_hz: 60.0,
};

/// Where a ship has got to after `t` seconds on its course.
fn advanced(s: &Ship, t: f64) -> (f64, f64) {
    // Knots are nautical miles an hour and a nautical mile is a minute of
    // latitude.
    let d = s.speed_kt * (t / 3600.0) / 60.0;
    let r = s.course_deg.to_radians();
    (s.lat + d * r.cos(), s.lon + d * r.sin() / s.lat.to_radians().cos())
}

/// Modulate one data field into the band at `at`, on the given channel.
fn place(buf: &mut [Complex32], at: usize, bits: &[bool], s: &Ship, channel_a: bool) {
    let p = TxParams { sample_rate: RATE, amplitude: s.amp, ..TxParams::default() };
    let mut burst = modulate_bits(bits, &p);
    let channel = if channel_a { AIS_CHANNEL_A_HZ } else { AIS_CHANNEL_B_HZ };
    shift(&mut burst, channel - AIS_PLAN_CENTER_HZ + s.offset_hz, RATE);
    for (k, z) in burst.iter().enumerate() {
        if let Some(d) = buf.get_mut(at + k) {
            *d += *z;
        }
    }
}

/// A position report: message 1 for Class A, message 18 for Class B.
fn position(s: &Ship, lat: f64, lon: f64) -> Vec<bool> {
    let lon_raw = (lon * 600_000.0) as i64;
    let lat_raw = (lat * 600_000.0) as i64;
    let sog = (s.speed_kt * 10.0) as u64;
    let cog = (s.course_deg * 10.0) as u64;
    if s.class_b {
        Payload::new(168)
            .put(0, 6, 18)
            .put(8, 30, u64::from(s.mmsi))
            .put(46, 10, sog)
            .put(56, 1, 1)
            .put_signed(57, 28, lon_raw)
            .put_signed(85, 27, lat_raw)
            .put(112, 12, cog)
            .put(124, 9, s.course_deg as u64)
            .bits()
    } else {
        Payload::new(168)
            .put(0, 6, 1)
            .put(8, 30, u64::from(s.mmsi))
            .put(38, 4, 0) // under way using engine
            .put_signed(42, 8, 0)
            .put(50, 10, sog)
            .put(60, 1, 1) // DGNSS
            .put_signed(61, 28, lon_raw)
            .put_signed(89, 27, lat_raw)
            .put(116, 12, cog)
            .put(128, 9, s.course_deg as u64)
            .bits()
    }
}

/// The message that names a ship: one for Class A, two halves for Class B.
fn static_reports(s: &Ship) -> Vec<Vec<bool>> {
    if s.class_b {
        vec![
            Payload::new(168)
                .put(0, 6, 24)
                .put(8, 30, u64::from(s.mmsi))
                .put(38, 2, 0)
                .put_text(40, 20, s.name)
                .bits(),
            Payload::new(168)
                .put(0, 6, 24)
                .put(8, 30, u64::from(s.mmsi))
                .put(38, 2, 1)
                .put(40, 8, s.ship_type)
                .put_text(90, 7, s.call)
                .put(132, 9, s.dim.0)
                .put(141, 9, s.dim.1)
                .put(150, 6, s.dim.2)
                .put(156, 6, s.dim.3)
                .bits(),
        ]
    } else {
        vec![
            Payload::new(424)
                .put(0, 6, 5)
                .put(8, 30, u64::from(s.mmsi))
                .put(40, 30, 9_100_000 + u64::from(s.mmsi % 1000))
                .put(232, 8, s.ship_type)
                .put(240, 9, s.dim.0)
                .put(249, 9, s.dim.1)
                .put(258, 6, s.dim.2)
                .put(264, 6, s.dim.3)
                .put(274, 4, 6) // ETA 06-14 09:30 UTC
                .put(278, 5, 14)
                .put(283, 5, 9)
                .put(288, 6, 30)
                .put(294, 8, s.draught_tenths)
                .put_text(70, 7, s.call)
                .put_text(112, 20, s.name)
                .put_text(302, 20, s.destination)
                .bits(),
        ]
    }
}

/// A shore station's report, with the time of day it keeps.
fn base_station(t: f64) -> Vec<bool> {
    let secs = t as u64 % 60;
    Payload::new(168)
        .put(0, 6, 4)
        .put(8, 30, 2_442_017) // 00MIDXXXX — a Netherlands coast station
        .put(38, 14, 2026)
        .put(52, 4, 9)
        .put(56, 5, 3)
        .put(61, 5, 11)
        .put(66, 6, 42)
        .put(72, 6, secs)
        .put(78, 1, 1)
        .put_signed(79, 28, (4.10 * 600_000.0) as i64)
        .put_signed(107, 27, (51.98 * 600_000.0) as i64)
        .bits()
}

/// An aid to navigation: a special mark that has not moved in years.
fn buoy() -> Vec<bool> {
    Payload::new(272)
        .put(0, 6, 21)
        .put(8, 30, 992_471_026)
        .put(38, 5, 30) // special mark
        .put(163, 1, 1)
        .put_signed(164, 28, (3.96 * 600_000.0) as i64)
        .put_signed(192, 27, (51.97 * 600_000.0) as i64)
        .put_text(43, 20, "MAAS APPROACH MN1")
        .bits()
}
