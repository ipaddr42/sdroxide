//! A self-contained session trace, so a user with a radio can produce a report
//! that a developer without one can read.
//!
//! Everything here also goes to `tracing`, but `tracing` is the wrong shape for
//! a bug report: it needs `RUST_LOG` set before the run, it interleaves with
//! every other subsystem, and asking somebody to reproduce a connect failure
//! with the right filter is asking them to reproduce it twice. A [`Trace`] is
//! always on, bounded, and dumps as one text block.
//!
//! # What it records
//!
//! Every control line in both directions, the first packet of each VITA-49
//! stream with its decoded header, and a periodic counter summary per stream.
//! Full packet bodies are *not* recorded — a 192 kHz IQ stream is 1.5 MB/s, and
//! the header plus the counters is what actually diagnoses a fault.
//!
//! # What it deliberately drops
//!
//! The line buffer is a ring: a long session keeps the most recent
//! [`Trace::CAPACITY`] lines and reports how many it discarded. A trace that
//! grew without bound would either exhaust memory on a radio that reconnects in
//! a loop — which is exactly the fault worth tracing — or force a size cap that
//! silently truncated the interesting end.

use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Per-stream packet counters, kept for the diagnostic summary.
#[derive(Debug, Clone, Copy, Default)]
pub struct StreamCounters {
    pub packets: u64,
    pub bytes: u64,
    /// Packets the 4-bit VITA counter says went missing.
    pub lost: u64,
    /// Class code of the most recent packet — a stream that changes it (a DAX
    /// IQ rate change, say) is worth seeing.
    pub class_code: u16,
    /// I/Q sample pairs actually decoded off this stream, where it is a DAX IQ
    /// one. Zero for every other stream type.
    ///
    /// Here so the dump can state the rate the radio is *really* streaming at,
    /// measured, rather than the rate its class code claims. A receive chain
    /// running at a rate the samples do not arrive at sounds like a badly
    /// detuned receiver rather than like an error — "unintelligible" is the
    /// word every report of it has used — and there is nothing else in a trace
    /// that would tell the two apart (issue #368).
    pub iq_pairs: u64,
    /// When the first packet of this stream arrived, so the measurement above
    /// has an interval and not just a count.
    pub first_at: Option<Instant>,
}

#[derive(Debug, Default)]
struct Inner {
    lines: VecDeque<String>,
    dropped: u64,
    streams: BTreeMap<u32, StreamCounters>,
    /// Every datagram the VITA-49 sockets received, whatever it turned out to
    /// be. Counted separately from [`Self::streams`] because the difference
    /// between the two is a diagnosis: streams empty *and* this zero means not
    /// one byte of UDP arrived from the radio, which is a network fault and not
    /// a protocol one. Streams empty with this non-zero would mean datagrams are
    /// arriving and we are failing to make sense of them, which is ours.
    datagrams: u64,
}

/// A shared, bounded record of one radio session.
///
/// Cloning shares the same buffer — the TCP thread, the UDP thread and the
/// `IqSource` all hold one.
#[derive(Debug, Clone)]
pub struct Trace {
    inner: Arc<Mutex<Inner>>,
    started: Instant,
}

impl Default for Trace {
    fn default() -> Self {
        Trace::new()
    }
}

impl Trace {
    /// How many lines are kept before the oldest are discarded. At roughly 100
    /// bytes a line this is a few hundred kilobytes — small enough to hold
    /// permanently, long enough to cover a connect plus several minutes of
    /// status churn.
    pub const CAPACITY: usize = 4000;

    pub fn new() -> Trace {
        Trace { inner: Arc::new(Mutex::new(Inner::default())), started: Instant::now() }
    }

    /// Record a free-form event.
    pub fn note(&self, what: impl AsRef<str>) {
        self.push(format!("{:>9.3}  {}", self.started.elapsed().as_secs_f64(), what.as_ref()));
    }

    /// Record a control line we sent.
    pub fn tx_line(&self, line: &str) {
        self.note(format!("→ {}", line.trim_end()));
        tracing::trace!(target: "smartsdr::control", line = %line.trim_end(), "sent");
    }

    /// Record a control line the radio sent.
    pub fn rx_line(&self, line: &str) {
        self.note(format!("← {}", line.trim_end()));
        tracing::trace!(target: "smartsdr::control", line = %line.trim_end(), "received");
    }

    /// Record that a datagram arrived, before anything is known about it.
    ///
    /// Deliberately separate from [`Self::packet`]: a "no spectrum" report has
    /// to distinguish "the radio's UDP never reached this machine" from "it
    /// reached us and we dropped it", and only a count taken before parsing can.
    pub fn datagram(&self) {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).datagrams += 1;
    }

    /// Record a VITA-49 packet's arrival. The first packet of each stream is
    /// written out in full; later ones only move the counters.
    pub fn packet(&self, stream_id: u32, class_code: u16, bytes: usize, lost: u8) {
        let first = {
            let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
            let e = g.streams.entry(stream_id).or_default();
            let first = e.packets == 0;
            if first {
                e.first_at = Some(Instant::now());
            }
            e.packets += 1;
            e.bytes += bytes as u64;
            e.lost += lost as u64;
            e.class_code = class_code;
            first
        };
        if first {
            self.note(format!(
                "▷ first packet stream=0x{stream_id:08X} class=0x{class_code:04X} bytes={bytes}"
            ));
            tracing::debug!(
                target: "smartsdr::vita",
                stream = format!("0x{stream_id:08X}"),
                class = format!("0x{class_code:04X}"),
                bytes,
                "first packet on stream"
            );
        }
    }

    /// Record how many I/Q pairs came out of one DAX IQ packet, which is what
    /// [`StreamCounters::iq_pairs`] measures the real rate from.
    pub fn iq_pairs(&self, stream_id: u32, pairs: usize) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        g.streams.entry(stream_id).or_default().iq_pairs += pairs as u64;
    }

    /// A snapshot of the per-stream counters.
    pub fn stream_counters(&self) -> BTreeMap<u32, StreamCounters> {
        self.inner.lock().unwrap_or_else(|e| e.into_inner()).streams.clone()
    }

    /// Render the whole trace as one text block, ready to paste into an issue.
    pub fn dump(&self) -> String {
        let g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        let mut out = String::with_capacity(64 * 1024);
        out.push_str("=== sdroxide SmartSDR session trace ===\n");
        out.push_str(&format!("elapsed: {:.1} s\n", self.started.elapsed().as_secs_f64()));
        if g.dropped > 0 {
            out.push_str(&format!(
                "note: {} earlier lines were discarded (ring holds {})\n",
                g.dropped,
                Trace::CAPACITY
            ));
        }
        out.push_str("\n--- streams ---\n");
        out.push_str(&format!("datagrams received on the VITA-49 socket(s): {}\n", g.datagrams));
        if g.streams.is_empty() {
            out.push_str(if g.datagrams == 0 {
                "(no VITA-49 packets received — and no UDP at all, so nothing the radio \
                 sent reached this machine)\n"
            } else {
                "(no VITA-49 packets received, though datagrams did arrive — they were \
                 too short to be VITA-49 headers)\n"
            });
        }
        for (id, c) in &g.streams {
            out.push_str(&format!(
                "0x{id:08X}  class=0x{:04X}  packets={}  bytes={}  lost={}\n",
                c.class_code, c.packets, c.bytes, c.lost
            ));
            // The measured rate against the declared one. A DAX IQ stream whose
            // two disagree is a receive chain running at the wrong rate, which
            // is heard as unintelligible audio and is invisible everywhere else
            // (issue #368).
            let (Some(first), Some(declared)) =
                (c.first_at, crate::protocol::dax_iq_rate(c.class_code))
            else {
                continue;
            };
            let secs = first.elapsed().as_secs_f64();
            if c.iq_pairs == 0 || secs < 1.0 {
                continue;
            }
            let measured = c.iq_pairs as f64 / secs;
            let off = (measured - f64::from(declared)) / f64::from(declared);
            out.push_str(&format!(
                "              {:.1} kHz measured over {secs:.1} s ({} pairs), declared {:.0} \
                 kHz{}\n",
                measured / 1000.0,
                c.iq_pairs,
                f64::from(declared) / 1000.0,
                if off.abs() > 0.02 {
                    format!(
                        "  ⚠ {:+.0}% — the receive chain is running at the wrong rate",
                        off * 100.0
                    )
                } else {
                    String::new()
                },
            ));
        }
        out.push_str("\n--- control ---\n");
        for line in &g.lines {
            out.push_str(line);
            out.push('\n');
        }
        out
    }

    fn push(&self, line: String) {
        let mut g = self.inner.lock().unwrap_or_else(|e| e.into_inner());
        if g.lines.len() >= Trace::CAPACITY {
            g.lines.pop_front();
            g.dropped += 1;
        }
        g.lines.push_back(line);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The dump states the rate a DAX IQ stream is *really* running at, and
    /// says so loudly when that is not the rate its class code claims.
    ///
    /// Nothing else in a trace can tell a receive chain running at the wrong
    /// rate from one being sent nonsense, and both are heard as unintelligible
    /// audio (issue #368).
    #[test]
    fn a_dax_iq_stream_reports_the_rate_it_is_really_running_at() {
        let t = Trace::new();
        // A 192 kHz stream that is actually delivering 96 kHz. One packet to
        // start the clock, then the pairs a second and a bit would carry.
        t.packet(0x2000_0000, crate::protocol::pcc::DAX_IQ_192K, 1440, 0);
        std::thread::sleep(std::time::Duration::from_millis(1100));
        t.iq_pairs(0x2000_0000, 96_000 * 11 / 10);
        let d = t.dump();
        assert!(d.contains("declared 192 kHz"), "{d}");
        assert!(d.contains("the receive chain is running at the wrong rate"), "{d}");

        // A stream that is not DAX IQ has no rate to measure and says nothing.
        let t2 = Trace::new();
        t2.packet(0x4000_0001, crate::protocol::pcc::FFT, 144, 0);
        assert!(!t2.dump().contains("declared"));
    }

    #[test]
    fn dump_reports_both_directions_and_streams() {
        let t = Trace::new();
        t.tx_line("C1|client program sdroxide\n");
        t.rx_line("R1|0|");
        t.packet(0x0400_0001, 0x02E6, 1032, 0);
        t.packet(0x0400_0001, 0x02E6, 1032, 2);

        let d = t.dump();
        assert!(d.contains("→ C1|client program sdroxide"));
        assert!(d.contains("← R1|0|"));
        assert!(d.contains("0x04000001"));
        assert!(d.contains("packets=2"));
        assert!(d.contains("lost=2"));
        // Only the first packet of a stream gets its own line.
        assert_eq!(d.matches("first packet").count(), 1);
    }

    #[test]
    fn the_ring_bounds_a_long_session_and_says_so() {
        let t = Trace::new();
        for i in 0..(Trace::CAPACITY + 50) {
            t.note(format!("line {i}"));
        }
        let d = t.dump();
        assert!(d.contains("50 earlier lines were discarded"));
        // The recent end is what survives — that's the end a fault is at.
        assert!(d.contains(&format!("line {}", Trace::CAPACITY + 49)));
        assert!(!d.contains("line 0\n"));
    }

    #[test]
    fn an_empty_trace_still_dumps() {
        assert!(Trace::new().dump().contains("no VITA-49 packets received"));
    }

    #[test]
    fn silence_says_whether_any_udp_arrived_at_all() {
        // The two silences have different causes and different fixes, and a
        // report that cannot tell them apart sends everybody to the wrong one.
        let quiet = Trace::new();
        assert!(quiet.dump().contains("no UDP at all"));

        let noisy = Trace::new();
        noisy.datagram();
        noisy.datagram();
        let d = noisy.dump();
        assert!(d.contains("datagrams received on the VITA-49 socket(s): 2"));
        assert!(d.contains("though datagrams did arrive"));
    }
}
