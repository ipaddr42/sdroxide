//! Threading wrapper around [`crate::bpsk::acquire`], mirroring
//! `sdroxide_skimmer::SkimmerController`: the realtime engine thread ships IQ
//! blocks to a worker over a bounded channel and drains status updates
//! non-blocking via [`Qo100Controller::poll`]. All the DSP runs on the worker
//! thread.
//!
//! Two things it does *not* copy from `SkimmerController`, because that one's
//! unit of work is a single block's FFT and this one's is a whole
//! [`bpsk::acquire`] sweep that can run for seconds:
//!
//! * dropping an IQ block on backpressure would punch a hole into a buffer a
//!   10.36 s frame has to sit inside contiguously — the same rule the DeepCW
//!   window follows. So a dropped block instead *restarts* the rolling
//!   buffer: fewer search windows under sustained backpressure, but every one
//!   of them is a contiguous span of air.
//!
//!   Which block to restart from is the whole difficulty, and it is why every
//!   block carries a sequence number rather than the realtime side merely
//!   raising a flag. A block can only be dropped when the queue is *full*, so
//!   at that instant the queue still holds a full depth of blocks that do
//!   join up with the buffer. A flag would therefore be consumed by one of
//!   *those* — clearing the buffer before the gap, throwing away good air,
//!   and then splicing the real gap in with the flag already spent.
//!   [`Iq::seq`] moves the decision onto the block itself, so the restart
//!   lands exactly where the hole is.
//! * `Drop` cannot simply join the worker — a sweep in progress would hold
//!   the engine thread for as long as the sweep takes. A shared cancel flag,
//!   polled between candidates inside `acquire`, brings that back to at most
//!   one candidate's work.
//!
//! Settings ride a separate unbounded channel: rare, and must never be
//! dropped even while the IQ queue is backed up.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;

use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
use sdroxide_dsp::Complex32;
use sdroxide_types::Qo100Settings;

use crate::bpsk::{self, FRAME_SECONDS};

/// Realtime data, dropped on backpressure.
struct Iq {
    /// Where this block sits in the realtime side's own stream, counting the
    /// blocks it had to drop as well as the ones that fit. The worker reads
    /// the gaps off these numbers — see the module doc for why a bare "a drop
    /// happened" flag could not do it.
    seq: u64,
    samples: Vec<Complex32>,
}

/// Control traffic, never dropped.
enum Ctl {
    Config(Qo100Settings),
    Stop,
}

/// How long a rolling buffer is kept before each search — comfortably more
/// than two frame times, so a frame beginning anywhere in the buffer is
/// always captured whole at least once, regardless of where the buffer
/// happens to be cut relative to the beacon's own, unrelated, transmit
/// timing.
fn window_seconds() -> f64 {
    FRAME_SECONDS * 2.3
}

/// How much of the window survives each search, so consecutive windows
/// overlap by more than one frame time — the reason a frame can never land
/// exactly on a cut.
fn keep_seconds() -> f64 {
    FRAME_SECONDS * 1.15
}

/// Coarse frequency-grid step the search tries, in Hz. The delay-and-multiply
/// chip detector `bpsk::acquire` runs at each candidate tolerates a residual
/// carrier error of well over 100 Hz (its own tests decode an uncorrected
/// 100 Hz offset), and `bpsk::refine_offset_hz` then measures the true offset
/// to about a hertz — so the grid only has to land *inside* that capture
/// range, not resolve it. 150 Hz keeps the nearest candidate within 75 Hz of
/// any real signal while keeping the candidate count — and so the sweep
/// time — an order of magnitude below a 10 Hz grid's.
const FREQ_STEP_HZ: f64 = 150.0;

/// The rate every candidate is mixed down to before the chip search, whatever
/// the capture rate above it. The beacon is 400 baud; 16 kHz is heavily
/// oversampled for it and is the floor `Engine::qo100_target_rate_hz` uses
/// too. Fixing it here is what stops the sweep cost growing with the square
/// of the search width — see [`bpsk::acquire`].
pub(crate) const DEMOD_RATE_HZ: f64 = 16_000.0;

/// Whether a block numbered `seq` carries straight on from the run the worker
/// has already buffered, `want_seq` being the number the next contiguous block
/// would have. Anything else is a gap the realtime side dropped, and the
/// buffer has to restart from `seq` rather than splice across it.
///
/// A free function so the rule is pinned by name: the decision has to be made
/// per *block*, and making it off a shared "a drop happened" flag instead is
/// the subtle way to get it wrong — see the module doc.
fn continues_run(want_seq: Option<u64>, seq: u64) -> bool {
    want_seq == Some(seq)
}

/// Bounded IQ queue depth. Roughly a second and a half of channel-rate audio
/// at a typical device read, enough that an ordinary scheduling hiccup does
/// not cost a window; sustained backpressure past this restarts the buffer
/// rather than splicing it (see the module doc).
const IQ_QUEUE_DEPTH: usize = 256;

pub struct Qo100Controller {
    iq_tx: Sender<Iq>,
    ctl_tx: Sender<Ctl>,
    res_rx: Receiver<sdroxide_types::Qo100Status>,
    /// Numbers the blocks handed to [`Self::on_rx_iq`], dropped ones
    /// included, so the worker can tell a gap from an ordinary hand-off.
    next_seq: AtomicU64,
    /// Set true by `Drop` so a sweep in progress returns at the next
    /// candidate instead of running to completion under the engine thread's
    /// `join`.
    cancel: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl Qo100Controller {
    pub fn new(rate_hz: f64, cfg: Qo100Settings) -> Self {
        let (iq_tx, iq_rx) = bounded::<Iq>(IQ_QUEUE_DEPTH);
        let (ctl_tx, ctl_rx) = unbounded::<Ctl>();
        let (res_tx, res_rx) = unbounded::<sdroxide_types::Qo100Status>();
        let cancel = Arc::new(AtomicBool::new(false));
        let window_len = (rate_hz * window_seconds()).round() as usize;
        let keep_len = (rate_hz * keep_seconds()).round() as usize;
        let worker = std::thread::Builder::new()
            .name("sdroxide-qo100".into())
            .spawn({
                let cancel = Arc::clone(&cancel);
                move || {
                    let mut cfg = cfg;
                    let mut buf: Vec<Complex32> = Vec::with_capacity(window_len);
                    let (mut tried, mut locked) = (0u64, 0u64);
                    let mut last: Option<(f64, String, i64)> = None; // offset, text, unix
                    // The `seq` the next contiguous block would carry; `None`
                    // before the first one has arrived.
                    let mut want_seq: Option<u64> = None;
                    loop {
                        select! {
                            recv(ctl_rx) -> msg => match msg {
                                Ok(Ctl::Config(next)) => cfg = next,
                                Ok(Ctl::Stop) | Err(_) => break,
                            },
                            recv(iq_rx) -> msg => match msg {
                                Ok(Iq { seq, samples }) => {
                                    if !continues_run(want_seq, seq) {
                                        buf.clear();
                                    }
                                    want_seq = Some(seq.wrapping_add(1));
                                    buf.extend_from_slice(&samples);
                                    if buf.len() < window_len {
                                        continue;
                                    }
                                    tried += 1;
                                    let lock = bpsk::acquire(
                                        &buf,
                                        rate_hz,
                                        cfg.search_half_width_hz,
                                        FREQ_STEP_HZ,
                                        DEMOD_RATE_HZ,
                                        &cancel,
                                    );
                                    if let Some(l) = lock {
                                        locked += 1;
                                        let unix = std::time::SystemTime::now()
                                            .duration_since(std::time::UNIX_EPOCH)
                                            .map(|d| d.as_secs() as i64)
                                            .unwrap_or(0);
                                        last = Some((l.offset_hz, l.text, unix));
                                    }
                                    // Keep the newest slice — enough overlap
                                    // that no frame can fall in the gap —
                                    // rather than clearing outright, so a
                                    // frame that straddles this cut is still
                                    // whole in the *next* window instead of
                                    // being thrown away twice.
                                    let start = buf.len().saturating_sub(keep_len);
                                    buf.drain(..start);
                                    let (offset_hz, text, locked_unix) =
                                        last.clone().unwrap_or_default();
                                    let _ = res_tx.send(sdroxide_types::Qo100Status {
                                        running: true,
                                        locked: lock_is_fresh(locked_unix),
                                        offset_hz,
                                        text,
                                        locked_unix,
                                        blocks_tried: tried,
                                        blocks_locked: locked,
                                    });
                                }
                                Err(_) => break,
                            },
                        }
                    }
                }
            })
            .expect("spawn qo100 worker");
        Qo100Controller {
            iq_tx,
            ctl_tx,
            res_rx,
            next_seq: AtomicU64::new(0),
            cancel,
            worker: Some(worker),
        }
    }

    /// Realtime path: hand a block of channel-rate IQ to the worker.
    /// Non-blocking; a block that will not fit is dropped, and the sequence
    /// number it consumed is what later tells the worker to restart its
    /// buffer rather than search one with a hole in it.
    pub fn on_rx_iq(&self, iq: &[Complex32]) {
        let seq = self.next_seq.fetch_add(1, Ordering::Relaxed);
        let _ = self.iq_tx.try_send(Iq { seq, samples: iq.to_vec() });
    }

    /// Apply new settings (currently just the search width) to the running
    /// worker.
    pub fn set_config(&self, cfg: Qo100Settings) {
        let _ = self.ctl_tx.send(Ctl::Config(cfg));
    }

    /// Drain the latest status, if a search finished since the last poll.
    /// Non-blocking. Only the newest matters — a status is a full snapshot,
    /// like `IsmStatus`.
    pub fn poll(&self) -> Option<sdroxide_types::Qo100Status> {
        let mut out = None;
        while let Ok(s) = self.res_rx.try_recv() {
            out = Some(s);
        }
        out
    }
}

/// Whether a lock reported at `locked_unix` is still worth showing as
/// "locked" — the beacon alternates an uncoded frame (this decoder) with a
/// coded one (not attempted) roughly every 10.36 s, so a gap of a bit over
/// twice that is expected and not evidence the beacon went away.
fn lock_is_fresh(locked_unix: i64) -> bool {
    if locked_unix == 0 {
        return false;
    }
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    (now - locked_unix) as f64 <= FRAME_SECONDS * 3.0
}

impl Drop for Qo100Controller {
    fn drop(&mut self) {
        // Cancel first: a sweep already running inside `acquire` polls this
        // between candidates, so the `join` below waits out at most one
        // candidate rather than a whole search.
        self.cancel.store(true, Ordering::Relaxed);
        let _ = self.ctl_tx.send(Ctl::Stop);
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::Qo100Settings;

    fn now_unix() -> i64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0)
    }

    #[test]
    fn a_lock_is_fresh_for_about_three_frame_times_then_stale() {
        assert!(!lock_is_fresh(0), "0 means the decoder has never locked");
        assert!(lock_is_fresh(now_unix()), "a lock from just now is fresh");
        // The beacon alternates an uncoded frame (this decoder) with a coded
        // one it skips, so a real gap runs a bit over two frame times; three
        // is the grace. Just inside it, then well outside:
        assert!(lock_is_fresh(now_unix() - (FRAME_SECONDS * 2.5) as i64));
        assert!(!lock_is_fresh(now_unix() - (FRAME_SECONDS * 4.0) as i64));
    }

    /// The rule that keeps a search window whole. The case that matters is the
    /// last one: a block arriving after a drop must restart the buffer *at
    /// itself*, which is what carrying the number on the block buys over a
    /// shared flag — a flag is consumed by whichever block is dequeued next,
    /// and since a drop can only happen with the queue full, that is one of
    /// the blocks still ahead of the gap.
    #[test]
    fn only_the_next_block_in_sequence_continues_the_buffered_run() {
        assert!(!continues_run(None, 0), "nothing buffered yet is a fresh start");
        assert!(continues_run(Some(7), 7), "the expected block carries straight on");
        assert!(!continues_run(Some(7), 8), "one dropped block is still a gap");
        assert!(!continues_run(Some(7), 260), "a queue's worth of drops likewise");
        assert!(!continues_run(Some(7), 6), "and so is anything out of order");
    }

    #[test]
    fn the_rolling_window_holds_a_whole_frame_and_overlaps_by_more_than_one() {
        // A frame beginning anywhere in the buffer has to be captured whole at
        // least once regardless of where the cut lands, so the window must
        // exceed two frame times ...
        assert!(window_seconds() > 2.0 * FRAME_SECONDS);
        // ... and consecutive windows must overlap by more than a frame, or a
        // frame could fall exactly on a cut and be lost from both.
        assert!(keep_seconds() > FRAME_SECONDS);
        assert!(keep_seconds() < window_seconds());
    }

    /// Poll `c` until `pred` holds on a status, or ~10 s pass. Returns the last
    /// status seen either way.
    fn wait_for(
        c: &Qo100Controller,
        pred: impl Fn(&sdroxide_types::Qo100Status) -> bool,
    ) -> Option<sdroxide_types::Qo100Status> {
        let mut latest = None;
        for _ in 0..500 {
            if let Some(s) = c.poll() {
                let hit = pred(&s);
                latest = Some(s);
                if hit {
                    return latest;
                }
            }
            std::thread::sleep(std::time::Duration::from_millis(20));
        }
        latest
    }

    /// The plumbing end to end on a signal that cannot lock: blocks accumulate
    /// to a full window, a search runs, a complete status snapshot comes back
    /// through `poll`, and — pure noise — nothing locks. The test finishing is
    /// also the assertion that dropping the controller with a search behind it
    /// returns promptly.
    #[test]
    fn the_worker_accumulates_a_window_searches_and_reports_through_poll() {
        let rate = 16_000.0;
        let c = Qo100Controller::new(
            rate,
            Qo100Settings { enabled: true, search_half_width_hz: 300.0 },
        );
        let n = (rate * FRAME_SECONDS * 2.4) as usize; // a hair over one window
        let noise: Vec<Complex32> = (0..n)
            .map(|i| {
                let (a, b) = ((i as f32 * 0.7).sin(), (i as f32 * 1.9 + 1.0).sin());
                Complex32::new(a, b)
            })
            .collect();
        c.on_rx_iq(&noise);
        let s = wait_for(&c, |s| s.blocks_tried >= 1).expect("a search should be attempted");
        assert!(s.running);
        assert!(!s.locked);
        assert_eq!(s.blocks_locked, 0, "pure noise must never lock");
    }

    /// A synthesized frame fed through the controller comes back out of `poll`
    /// as a lock, with the decoded text and the offset the search assumed —
    /// the same contract `bpsk::acquire`'s tests check, but exercised through
    /// the worker thread, the rolling buffer and the status channel.
    #[test]
    fn a_synthesized_frame_locks_through_the_worker() {
        let rate = 16_000.0;
        let c = Qo100Controller::new(
            rate,
            Qo100Settings { enabled: true, search_half_width_hz: 300.0 },
        );
        // One synth frame is ~10 s of signal and a window is ~24 s, so stack
        // three; the frame in the first copy lands wholly inside the buffer.
        let one = crate::bpsk::tests::synth_signal("CONTROLLER E2E", rate, 150.0, 0.02, 3);
        let block: Vec<Complex32> = one.iter().chain(&one).chain(&one).copied().collect();
        c.on_rx_iq(&block);
        let s = wait_for(&c, |s| s.blocks_locked >= 1).expect("the frame should lock");
        assert!(s.locked);
        assert!((s.offset_hz - 150.0).abs() <= 3.0, "offset {}", s.offset_hz);
        assert!(s.text.starts_with("CONTROLLER E2E"), "{:?}", s.text);
    }
}
