//! Decode a recording and say what happened, in enough detail to tell a deaf
//! aerial from a broken decoder.
//!
//! ```text
//! cargo run --release -p sdroxide-vdl2 --example vdl2_replay -- /tmp/vdl2.iq 2400000 136825000
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
//! - **no bursts** — nothing is arriving. The aerial, or the receiver is not
//!   looking here.
//! - **bursts, no syncs** — something is arriving and it is not VDL2.
//! - **syncs, no headers** — it *is* VDL2 and the bits are being read wrongly.
//!   A decoder problem, not a propagation one.
//! - **headers, Reed-Solomon failures** — real frames, arriving damaged.
//! - **Reed-Solomon good, frame check bad** — the error correction is fine and
//!   something above it is not, which is this decoder's fault and not the
//!   band's. On a real recording this figure is the sharpest evidence there is.
//!
//! `--syndromes` prints the fraction of Reed-Solomon blocks that were already
//! clean before any correction. On a decent recording that is nearly all of
//! them; if the code's parameters were wrong it would be none, and the check
//! needs no reference decoder and no aerial of one's own.

use std::io::Read;

use sdroxide_dsp::{Complex32, Ddc};
use sdroxide_types::{Vdl2Payload, Vdl2Settings};
use sdroxide_vdl2::block::InterleaveOrder;
use sdroxide_vdl2::channel::ChannelRx;
use sdroxide_vdl2::{plan, station};

fn main() {
    let mut args = std::env::args().skip(1);
    let mut path = None;
    let mut rate = 0f64;
    let mut center = sdroxide_types::VDL2_PLAN_CENTER_HZ;
    let mut threshold = 9.0f32;
    let mut order = InterleaveOrder::RoundRobin;
    let mut positional = 0;
    while let Some(a) = args.next() {
        match a.as_str() {
            "--threshold" => threshold = args.next().and_then(|v| v.parse().ok()).unwrap_or(9.0),
            "--sequential" => order = InterleaveOrder::Sequential,
            "--syndromes" => {}
            _ => {
                match positional {
                    0 => path = Some(a),
                    1 => rate = a.parse().unwrap_or(0.0),
                    _ => center = a.parse().unwrap_or(center),
                }
                positional += 1;
            }
        }
    }
    let Some(path) = path else {
        eprintln!(
            "usage: vdl2_replay <file.iq> <rate_hz> [center_hz] [--threshold dB] [--sequential]"
        );
        std::process::exit(2);
    };
    if rate <= 0.0 {
        eprintln!("a sample rate is needed; the file does not carry one");
        std::process::exit(2);
    }

    let iq = read_cf32(&path);
    println!(
        "{path}: {} samples, {:.3} s at {:.3} Msps centred on {:.3} MHz",
        iq.len(),
        iq.len() as f64 / rate,
        rate / 1e6,
        center / 1e6
    );

    // The same window the engine would place, so the answer here is the answer
    // there.
    let window_rate = Ddc::rate_for(rate, plan::WINDOW_TARGET_RATE_HZ.min(rate));
    let window_center = plan::window_center_for(center, rate, window_rate);
    let reachable = plan::channels_in_window(window_center, window_rate);
    println!(
        "window {:.3} MHz / {:.0} kHz reaches {} of {} channels",
        window_center / 1e6,
        window_rate / 1e3,
        reachable.len(),
        plan::CHANNELS.len()
    );
    if reachable.is_empty() {
        println!("nothing to decode: no VDL2 channel is inside this recording");
        return;
    }

    let mut window = Ddc::new(rate, plan::WINDOW_TARGET_RATE_HZ.min(rate));
    window.set_offset_hz(window_center - center);
    let mut win_buf = Vec::new();
    window.process(&iq, &mut win_buf);

    let mut tracker = station::Tracker::new(Vdl2Settings::default());
    let mut total = Totals::default();
    for &i in &reachable {
        let ch = &plan::CHANNELS[i];
        let mut ddc = Ddc::new(window_rate, plan::CHANNEL_TARGET_RATE_HZ);
        ddc.set_offset_hz(ch.center_hz - window_center);
        let mut buf = Vec::new();
        ddc.process(&win_buf, &mut buf);

        let mut rx = ChannelRx::new(ch.center_hz, ddc.out_rate(), threshold);
        rx.set_interleave_order(order);
        let mut out = Vec::new();
        rx.push(&buf, &mut out);
        let c = *rx.counters();
        println!(
            "{:.3} MHz  {:.2} sps  floor {:6.1} dBFS  bursts {:4}  syncs {:5}  headers {:4} \
             (bad {:3}, insane {:3})  RS ok {:4} fail {:4} fixed {:4}  bad FCS {:3}  frames {:3}",
            ch.center_hz / 1e6,
            rx.samples_per_symbol(),
            rx.floor_dbfs(),
            c.bursts,
            c.syncs,
            c.header_ok,
            c.header_bad,
            c.length_insane,
            c.rs_ok,
            c.rs_fail,
            c.rs_corrected,
            c.fcs_bad,
            c.frames,
        );
        total.add(&c);
        for d in &out {
            tracker.absorb(d, 0);
        }
    }

    println!();
    for m in tracker.messages() {
        let payload = match &m.payload {
            Vdl2Payload::Acars(a) => format!("ACARS {}", a.summary()),
            Vdl2Payload::Xid(x) => {
                let extra = if let (Some(la), Some(lo)) = (x.lat, x.lon) {
                    format!(" at {la:.1} {lo:.1}")
                } else {
                    String::new()
                };
                format!("XID {}{extra}", x.kind)
            }
            Vdl2Payload::Other { note, .. } => note.clone(),
            Vdl2Payload::None => String::new(),
        };
        println!(
            "{:.3} {} {} -> {} {:>10}  SNR {:4.1}  EVM {:4.1}°  {:+6.0} Hz  {payload}",
            m.freq_hz / 1e6,
            if m.command { "cmd" } else { "rsp" },
            m.src_hex(),
            m.dst_hex(),
            m.frame.label(),
            m.snr_db,
            m.evm_deg,
            m.freq_err_hz,
        );
    }

    println!();
    println!("{} stations heard:", tracker.stations().len());
    let mut st = tracker.stations();
    st.sort_by_key(|s| s.addr);
    for s in &st {
        println!(
            "  {} {:<8} {:<8} {:>4} messages  {:.3} MHz",
            s.hex(),
            s.kind.short(),
            s.label(),
            s.messages,
            s.last_freq_hz / 1e6
        );
    }

    println!();
    total.report();
}

#[derive(Default)]
struct Totals {
    bursts: u64,
    syncs: u64,
    headers: u64,
    rs_ok: u64,
    rs_fail: u64,
    fcs_bad: u64,
    frames: u64,
}

impl Totals {
    fn add(&mut self, c: &sdroxide_vdl2::channel::Counters) {
        self.bursts += c.bursts;
        self.syncs += c.syncs;
        self.headers += c.header_ok;
        self.rs_ok += c.rs_ok;
        self.rs_fail += c.rs_fail;
        self.fcs_bad += c.fcs_bad;
        self.frames += c.frames;
    }

    /// The one sentence that says what to do next.
    fn report(&self) {
        if self.bursts == 0 {
            println!(
                "no bursts at all. Nothing rose above the noise on any channel — the aerial, \
                 or a receiver that is not looking here."
            );
        } else if self.syncs == 0 {
            println!(
                "{} bursts and no synchronisation words. Something is on these channels and \
                 it is not VDL2.",
                self.bursts
            );
        } else if self.headers == 0 {
            println!(
                "{} synchronisation words and no valid headers. The bursts are VDL2 and the \
                 bits are being read wrongly — a decoder problem, not a propagation one.",
                self.syncs
            );
        } else if self.frames == 0 && self.rs_fail > 0 {
            println!(
                "{} headers, {} Reed-Solomon failures and no frames. Real transmissions \
                 arriving too damaged to repair: a weak signal, or the wrong aerial.",
                self.headers, self.rs_fail
            );
        } else if self.fcs_bad > self.frames {
            println!(
                "{} blocks repaired and {} frames failed their check sequence against {} \
                 that passed. The error correction is working and something above it is not, \
                 which is this decoder's fault rather than the band's.",
                self.rs_ok, self.fcs_bad, self.frames
            );
        } else {
            println!(
                "{} frames from {} bursts. Reed-Solomon repaired {} blocks and refused {}; \
                 {} frames failed their check sequence.",
                self.frames, self.bursts, self.rs_ok, self.rs_fail, self.fcs_bad
            );
        }
    }
}

fn read_cf32(path: &str) -> Vec<Complex32> {
    let mut f = std::fs::File::open(path).unwrap_or_else(|e| {
        eprintln!("{path}: {e}");
        std::process::exit(1);
    });
    let mut raw = Vec::new();
    f.read_to_end(&mut raw).expect("read");
    raw.chunks_exact(8)
        .map(|c| {
            Complex32::new(
                f32::from_le_bytes([c[0], c[1], c[2], c[3]]),
                f32::from_le_bytes([c[4], c[5], c[6], c[7]]),
            )
        })
        .collect()
}
