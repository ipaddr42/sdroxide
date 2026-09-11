//! Decode a recording and say what happened, in enough detail to tell a deaf
//! aerial from a broken decoder.
//!
//! ```text
//! cargo run --release -p sdroxide-ais --example ais_replay -- /tmp/sea.iq 240000 162000000
//! ```
//!
//! The file is interleaved little-endian `f32`, the format `sdroxide --file`
//! reads and `--record-iq` writes. `rate` and `center` are the recording's, in
//! hertz.
//!
//! # Reading the summary
//!
//! The counters are printed per channel, in the order the chain fails in, and
//! the first one that stops counting names the problem:
//!
//! - **no slots** — nothing is arriving. The aerial, or the receiver is not
//!   looking here. AIS is line-of-sight VHF and an indoor wire hears none of
//!   it, however well it hears broadcast FM.
//! - **slots, all of them without an eye** — something is arriving and it is
//!   not GMSK. A carrier, a repeater's tail, a pager.
//! - **slots with an eye, no frames, no bad FCS** — bursts that never framed:
//!   no flag pair was found in them at all. A decoder problem, or a
//!   transmission so damaged that the flags themselves went.
//! - **bad FCS and no frames** — real bursts arriving damaged, or a bit order
//!   backwards. On a marginal path a handful of bad check sequences alongside
//!   good frames is ordinary; nothing but bad ones is not.
//! - **frames, and unsupported messages** — the decoder is working and the
//!   traffic includes types it does not model (binary broadcasts, safety text,
//!   interrogations). Nothing is wrong.
//!
//! The **offset** line is the one to read first on a receiver that has never
//! decoded anything. It is how far off frequency the transmissions were, and
//! every ship being kilohertz off in the same direction is the receiver rather
//! than the fleet.
//!
//! Each decoded message is printed as the `!AIVDM` sentence it would have gone
//! out as, so the whole run can be piped into another AIS decoder and compared
//! — which for a decoder written from the standard rather than from a recording
//! is the check that matters.

use std::io::Read;

use sdroxide_ais::channel::ChannelRx;
use sdroxide_ais::plan;
use sdroxide_ais::track::Tracker;
use sdroxide_dsp::{Complex32, Ddc};
use sdroxide_types::{AIS_PLAN_CENTER_HZ, AisSettings};

fn main() {
    let mut args = std::env::args().skip(1);
    let mut path = None;
    let mut rate = 0f64;
    let mut center = AIS_PLAN_CENTER_HZ;
    let mut threshold = f64::from(sdroxide_types::AIS_THRESHOLD_DB) as f32;
    let mut sentences = false;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--threshold" => threshold = args.next().and_then(|v| v.parse().ok()).unwrap_or(8.0),
            "--sentences" => sentences = true,
            _ if path.is_none() => path = Some(a),
            _ if rate == 0.0 => rate = a.parse().unwrap_or(0.0),
            _ => center = a.parse().unwrap_or(center),
        }
    }
    let Some(path) = path else {
        eprintln!(
            "usage: ais_replay <file.iq> <rate> [center] [--threshold dB] [--sentences]\n\
             the file is interleaved little-endian f32, as `--record-iq` writes"
        );
        std::process::exit(2);
    };
    if rate <= 0.0 {
        eprintln!("a sample rate is needed: the file does not carry one");
        std::process::exit(2);
    }

    let mut raw = Vec::new();
    std::fs::File::open(&path)
        .unwrap_or_else(|e| {
            eprintln!("cannot open {path}: {e}");
            std::process::exit(1);
        })
        .read_to_end(&mut raw)
        .expect("read the recording");
    let iq: Vec<Complex32> = raw
        .chunks_exact(8)
        .map(|c| {
            Complex32::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect();
    let seconds = iq.len() as f64 / rate;
    println!("{path}: {:.1} s at {rate} sps about {:.3} MHz", seconds, center / 1e6);

    // The window the engine would place, and then the same split into channels
    // the worker makes.
    let window_target = plan::WINDOW_TARGET_RATE_HZ.min(rate);
    let mut window = Ddc::new(rate, window_target);
    let window_center = plan::window_center_for(center, rate, window.out_rate());
    window.set_offset_hz(window_center - center);
    let window_rate = window.out_rate();
    let reachable = plan::channels_in_window(window_center, window_rate);
    if reachable.is_empty() {
        println!(
            "no AIS channel is inside a {:.0} kHz window about {:.3} MHz — \
             the recording does not contain either of them",
            window_rate / 1e3,
            window_center / 1e6
        );
        return;
    }
    let both = reachable.len() == plan::CHANNELS.len();
    let chan_rate = plan::channel_rate_for(window_rate, both);
    println!(
        "window {:.0} kHz about {:.3} MHz; channels at {:.1} kHz ({:.2} samples a bit)",
        window_rate / 1e3,
        window_center / 1e6,
        chan_rate / 1e3,
        chan_rate / sdroxide_types::AIS_BIT_RATE
    );
    if !both {
        println!(
            "only AIS {} is inside it — a ship alternates, so it will be heard at half rate",
            plan::CHANNELS[reachable[0]].label
        );
    }

    let mut wide = Vec::new();
    window.process(&iq, &mut wide);

    let cfg = AisSettings::default();
    let mut tracker = Tracker::new(cfg);
    let mut decoded = Vec::new();
    let mut rxs: Vec<(usize, Ddc, ChannelRx)> = reachable
        .iter()
        .map(|&i| {
            let ch = &plan::CHANNELS[i];
            let mut ddc = Ddc::new(window_rate, chan_rate);
            ddc.set_offset_hz(ch.center_hz - window_center);
            let rate = ddc.out_rate();
            let label = ch.label.chars().next().unwrap_or('A');
            (i, ddc, ChannelRx::new(ch.center_hz, label, rate, threshold))
        })
        .collect();

    let mut buf = Vec::new();
    let mut lines = Vec::new();
    for (_, ddc, rx) in &mut rxs {
        buf.clear();
        ddc.process(&wide, &mut buf);
        decoded.clear();
        rx.push(&buf, &mut decoded);
        for d in &decoded {
            // A synthetic clock: the recording has no wall time, and the
            // tracker only needs something monotonic.
            tracker.absorb(d, 0);
            let m = &d.message;
            lines.push(format!(
                "  AIS {} type {:>2}  MMSI {:>9}  {:>5.0} dBFS  {:>4.1} dB  {:+6.0} Hz{}",
                d.channel,
                m.kind,
                m.mmsi,
                d.rssi_dbfs,
                d.snr_db,
                d.freq_offset_hz,
                if m.known { "" } else { "  (type not modelled)" }
            ));
            if sentences {
                for s in sdroxide_ais::sixbit::nmea(&d.bits, d.channel, 0) {
                    lines.push(format!("    {s}"));
                }
            }
        }
    }

    println!("\nper channel:");
    for (i, _, rx) in &rxs {
        let n = rx.counters();
        println!(
            "  AIS {:<2} {:.3} MHz  floor {:>6.1} dBFS  slots {:<5} no-eye {:<5} bad-FCS {:<5} \
             frames {:<5} messages {:<5} unsupported {}",
            plan::CHANNELS[*i].label,
            plan::CHANNELS[*i].center_hz / 1e6,
            rx.floor_dbfs(),
            n.bursts,
            n.no_eye,
            n.bad_fcs,
            n.frames,
            n.messages,
            n.unsupported
        );
        match rx.offset_hz() {
            Some(o) => {
                println!("      offset {o:+.0} Hz ({:+.1} ppm)", f64::from(o) / center * 1e6)
            }
            None => println!("      offset  — nothing decoded here"),
        }
    }

    if !lines.is_empty() {
        println!("\n{} messages:", lines.len());
        for l in &lines {
            println!("{l}");
        }
    }

    let vessels = tracker.snapshot();
    println!("\n{} stations:", vessels.len());
    let mut vessels = vessels;
    vessels.sort_by_key(|v| v.mmsi);
    for v in &vessels {
        let pos = match v.lat.zip(v.lon) {
            Some((la, lo)) => format!("{la:9.5}, {lo:10.5}"),
            None => "        —,          —".to_string(),
        };
        println!(
            "  {:>9}  {:<20} {:<5} {}  {:>5} kt  {:>3}°  {} msg",
            v.mmsi,
            v.label(),
            v.kind.short(),
            pos,
            v.fmt_speed(),
            v.fmt_course(),
            v.messages
        );
    }

    // The one sentence that says what to do next.
    let total: u64 = rxs.iter().map(|(_, _, r)| r.counters().messages).sum();
    let slots: u64 = rxs.iter().map(|(_, _, r)| r.counters().bursts).sum();
    let no_eye: u64 = rxs.iter().map(|(_, _, r)| r.counters().no_eye).sum();
    let bad: u64 = rxs.iter().map(|(_, _, r)| r.counters().bad_fcs).sum();
    println!();
    if total > 0 {
        println!("the decoder is working: {total} messages from {} stations.", vessels.len());
    } else if slots == 0 {
        println!(
            "nothing arrived at all. The aerial, or the receiver is not looking here — \
             AIS is line-of-sight VHF and needs 46 cm of wire with a view of the water."
        );
    } else if no_eye >= slots {
        println!(
            "{slots} slots opened and none of them held a GMSK signal. Something is on \
             the channel and it is not AIS."
        );
    } else if bad == 0 {
        println!(
            "{slots} slots opened and none framed. No flag pair was found in any of them: \
             either the transmissions are far more damaged than the levels suggest, or the \
             bit order is wrong."
        );
    } else {
        println!(
            "{slots} slots, {bad} failed check sequences and no good frames. Marginal \
             signal, or a decoder fault one layer in. Check the offset line above first."
        );
    }
}
