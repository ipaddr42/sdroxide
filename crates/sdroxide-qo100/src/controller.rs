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

use std::collections::VecDeque;
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

/// The spectral tracker's own short window — long enough for a stable Welch
/// average of the beacon's twin-lobe shape, short enough to follow an LNB
/// still drifting as it warms up. Independent of the frame decoder's much
/// longer window: the tracker reads the beacon's *shape*, not its bits.
fn track_seconds() -> f64 {
    3.0
}

/// How much fresh IQ to gather between tracker passes — a new estimate on the
/// screen about once a second.
fn track_hop_seconds() -> f64 {
    1.0
}

/// How long a tracker estimate stays on the status as current before it is
/// dropped to `None` — a couple of tracker windows.
const EST_FRESH_SECS: i64 = 12;

/// Half-width of the band the tracker scans once it has the beacon and it is
/// no longer out in the parking window — wide enough to hold both lobes plus
/// context and to not lose a beacon drifting a few hundred Hz between passes.
const TRACK_BAND_HZ: f64 = 3_000.0;

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
        let track_len = (rate_hz * track_seconds()).round().max(1.0) as usize;
        let track_hop = (rate_hz * track_hop_seconds()).round().max(1.0) as usize;
        let frame_len = (rate_hz * FRAME_SECONDS).round().max(1.0) as usize;
        let worker = std::thread::Builder::new()
            .name("sdroxide-qo100".into())
            .spawn({
                let cancel = Arc::clone(&cancel);
                move || {
                    let started = std::time::Instant::now();
                    let mut cfg = cfg;
                    let mut buf: Vec<Complex32> = Vec::with_capacity(window_len);
                    let (mut tried, mut locked) = (0u64, 0u64);
                    let mut last: Option<(f64, String, i64)> = None; // offset, text, unix
                    let (mut est_updates, mut est_misses) = (0u64, 0u64);
                    // Newest tracker estimate and the unix second it landed.
                    let mut last_est: Option<(bpsk::CarrierEstimate, i64)> = None;
                    // Recent (elapsed_secs, offset_hz) estimates, for the
                    // drift-rate fit the frame decoder is handed.
                    let mut est_hist: VecDeque<(f64, f64)> = VecDeque::new();
                    let mut drift_hz_s = 0.0f64;
                    let mut drift_accel = 0.0f64;
                    let mut progress = bpsk::DecodeProgress::default();
                    // Samples gathered since the last tracker pass.
                    let mut since_track = 0usize;
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
                                        since_track = 0;
                                    }
                                    want_seq = Some(seq.wrapping_add(1));
                                    buf.extend_from_slice(&samples);
                                    since_track += samples.len();
                                    let now = now_unix();

                                    // Fast tracker: the newest `track_len`
                                    // samples, about once a second.
                                    //
                                    // Band: a narrow window around the last
                                    // fresh estimate when there is one, so the
                                    // tracker follows the beacon wherever it
                                    // last sat — including down toward centre
                                    // after the closed loop has corrected. On
                                    // a cold start or after losing it: the
                                    // operator's parking window, unless
                                    // auto-correct is armed, in which case the
                                    // loop's target is 0 so the sweep has to
                                    // reach down there too. Anything that is
                                    // not two symmetric lobes still does not
                                    // win.
                                    if cfg.enabled
                                        && since_track >= track_hop
                                        && buf.len() >= track_len
                                    {
                                        since_track = 0;
                                        let tail = &buf[buf.len() - track_len..];
                                        let (lo, hi) = match last_est {
                                            Some((e, t)) if now - t <= EST_FRESH_SECS => {
                                                (e.hz - TRACK_BAND_HZ, e.hz + TRACK_BAND_HZ)
                                            }
                                            _ if cfg.auto_apply => {
                                                (-TRACK_BAND_HZ, cfg.park_hi_hz)
                                            }
                                            _ => (cfg.park_lo_hz, cfg.park_hi_hz),
                                        };
                                        match bpsk::estimate_carrier(tail, rate_hz, lo, hi) {
                                            Some(e) => {
                                                est_updates += 1;
                                                last_est = Some((e, now));
                                                let t = started.elapsed().as_secs_f64();
                                                est_hist.push_back((t, e.hz));
                                                while est_hist
                                                    .front()
                                                    .is_some_and(|&(t0, _)| t - t0 > DRIFT_FIT_SECS)
                                                {
                                                    est_hist.pop_front();
                                                }
                                                let (s, a) = drift_fit(&est_hist);
                                                drift_hz_s = s;
                                                drift_accel = a;
                                            }
                                            None => {
                                                est_misses += 1;
                                                est_hist.clear();
                                                drift_hz_s = 0.0;
                                                drift_accel = 0.0;
                                            }
                                        }
                                        let _ = res_tx.send(status_snapshot(
                                            &cfg, tried, locked, est_updates, est_misses,
                                            &last, &last_est, drift_hz_s, drift_accel, &progress,
                                            buf.len(), frame_len, now,
                                        ));
                                    }

                                    if buf.len() < window_len {
                                        continue;
                                    }

                                    // Frame decoder: a whole window, only when
                                    // the operator asked for the telemetry.
                                    if cfg.decode_telemetry {
                                        tried += 1;
                                        let (lock, prog) = bpsk::acquire_debug(
                                            &buf,
                                            rate_hz,
                                            cfg.search_half_width_hz,
                                            FREQ_STEP_HZ,
                                            DEMOD_RATE_HZ,
                                            drift_hz_s,
                                            drift_accel,
                                            &cancel,
                                        );
                                        progress = prog;
                                        match lock {
                                            Some(l) => {
                                                locked += 1;
                                                last = Some((l.offset_hz, l.text, now));
                                            }
                                            // The beacon alternates uncoded
                                            // and coded frames, so a window
                                            // the uncoded decoder made nothing
                                            // of may simply have held the
                                            // other kind — and on a station
                                            // whose phase noise the 514-byte
                                            // CRC cannot survive, the coded
                                            // frame is the only one that will
                                            // ever come back. Tried second
                                            // rather than first because it is
                                            // the more expensive of the two
                                            // and the uncoded path has already
                                            // done the cheap thing.
                                            None => {
                                                let seed =
                                                    last_est.map(|(e, _)| e.hz).unwrap_or(0.0);
                                                if let Some((f, carrier)) =
                                                    crate::fec::decode_iq(&buf, rate_hz, seed)
                                                {
                                                    locked += 1;
                                                    progress.carrier = true;
                                                    progress.crc_ok = true;
                                                    last = Some((
                                                        carrier,
                                                        crate::fec::payload_text(&f.payload),
                                                        now,
                                                    ));
                                                }
                                            }
                                        }
                                    } else {
                                        progress = bpsk::DecodeProgress::default();
                                    }

                                    // Keep the newest slice — enough overlap
                                    // that no frame can fall in the gap —
                                    // rather than clearing outright, so a
                                    // frame that straddles this cut is still
                                    // whole in the *next* window instead of
                                    // being thrown away twice.
                                    let start = buf.len().saturating_sub(keep_len);
                                    buf.drain(..start);
                                    let _ = res_tx.send(status_snapshot(
                                        &cfg, tried, locked, est_updates, est_misses,
                                        &last, &last_est, drift_hz_s, drift_accel, &progress,
                                        buf.len(), frame_len, now,
                                    ));
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

fn now_unix() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0)
}

/// How long a stretch of tracker estimates the drift fit uses.
const DRIFT_FIT_SECS: f64 = 12.0;

/// Least-squares fit of `hz ≈ a + b·t + c·t²` to the recent `(t, hz)` tracker
/// estimates, returned as `(drift_rate, curvature)` in Hz/s and Hz/s² — what
/// [`bpsk::acquire_debug`] de-rotates before it looks for a frame.
///
/// A warming LNB's local oscillator does not walk at a constant rate, so a
/// straight-line fit leaves a residual chirp across the decoder's ~24 s window
/// even when the last few seconds looked linear. The second-order term catches
/// that. The drift rate is reported at the *middle* of the fit window, which
/// is ≈ the centre of the decode buffer `dechirp` pivots on — so the value can
/// be handed straight through without a further time shift.
///
/// `(0.0, 0.0)` until there are enough points over a long enough span for the
/// fit to mean anything; each term clamped so one wild estimate cannot send
/// the decoder chasing a huge chirp.
fn drift_fit(hist: &VecDeque<(f64, f64)>) -> (f64, f64) {
    if hist.len() < 5 {
        return (0.0, 0.0);
    }
    let span = hist.back().unwrap().0 - hist.front().unwrap().0;
    if span < 0.5 * DRIFT_FIT_SECS {
        return (0.0, 0.0);
    }
    // Normal equations for [a, b, c] against 1, x, x² with x = t − mean(t);
    // centring keeps the 3×3 well-conditioned for a dozen points over ~12 s.
    let n = hist.len() as f64;
    let tm = hist.iter().map(|&(t, _)| t).sum::<f64>() / n;
    let (mut s1, mut s2, mut s3, mut s4) = (0.0, 0.0, 0.0, 0.0);
    let (mut r0, mut r1, mut r2) = (0.0, 0.0, 0.0);
    for &(t, h) in hist {
        let x = t - tm;
        let (x2, x3, x4) = (x * x, x * x * x, x * x * x * x);
        s1 += x;
        s2 += x2;
        s3 += x3;
        s4 += x4;
        r0 += h;
        r1 += h * x;
        r2 += h * x2;
    }
    let Some([_, b, c]) = solve3([[n, s1, s2], [s1, s2, s3], [s2, s3, s4]], [r0, r1, r2]) else {
        return (0.0, 0.0);
    };
    (b.clamp(-120.0, 120.0), (2.0 * c).clamp(-15.0, 15.0))
}

/// Cramer's rule for a 3×3 system; `None` when it is singular.
fn solve3(m: [[f64; 3]; 3], v: [f64; 3]) -> Option<[f64; 3]> {
    let det = |a: &[[f64; 3]; 3]| {
        a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
            - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
            + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])
    };
    let d = det(&m);
    if d.abs() < 1e-9 {
        return None;
    }
    let mut out = [0.0f64; 3];
    for (i, o) in out.iter_mut().enumerate() {
        let mut mi = m;
        for row in 0..3 {
            mi[row][i] = v[row];
        }
        *o = det(&mi) / d;
    }
    Some(out)
}

/// Build a full status snapshot from the worker's running counters — the one
/// place the wire type is assembled, so the tracker pass and the decode pass
/// cannot drift apart in what they report.
#[allow(clippy::too_many_arguments)]
fn status_snapshot(
    cfg: &Qo100Settings,
    tried: u64,
    locked: u64,
    est_updates: u64,
    est_misses: u64,
    last: &Option<(f64, String, i64)>,
    last_est: &Option<(bpsk::CarrierEstimate, i64)>,
    drift_hz_s: f64,
    drift_accel_hz_s2: f64,
    progress: &bpsk::DecodeProgress,
    buf_len: usize,
    frame_len: usize,
    now: i64,
) -> sdroxide_types::Qo100Status {
    let (offset_hz, text, locked_unix) = last.clone().unwrap_or_default();
    let fresh = last_est.as_ref().filter(|(_, t)| now - t <= EST_FRESH_SECS);
    sdroxide_types::Qo100Status {
        running: cfg.enabled || cfg.decode_telemetry,
        locked: lock_is_fresh(locked_unix),
        offset_hz,
        text,
        locked_unix,
        blocks_tried: tried,
        blocks_locked: locked,
        tracking: cfg.enabled,
        est_offset_hz: fresh.map(|(e, _)| e.hz),
        est_null_depth_db: fresh.map(|(e, _)| e.null_depth_db).unwrap_or(0.0),
        est_symmetry: fresh.map(|(e, _)| e.symmetry).unwrap_or(0.0),
        est_snr_db: fresh.map(|(e, _)| e.snr_db).unwrap_or(0.0),
        est_updates,
        est_misses,
        est_drift_hz_s: if fresh.is_some() { drift_hz_s as f32 } else { 0.0 },
        est_drift_accel_hz_s2: if fresh.is_some() { drift_accel_hz_s2 as f32 } else { 0.0 },
        decoding: cfg.decode_telemetry,
        carrier_seen: progress.carrier,
        sync_seen: progress.sync,
        sync_bit_errors: progress.sync_bit_errors,
        sync_matches: progress.sync_matches,
        frame_fill: if frame_len > 0 { (buf_len as f32 / frame_len as f32).min(1.0) } else { 0.0 },
        crc_ok: progress.crc_ok,
        // The closed-loop fields are the engine's to fill in — the worker
        // only measures, it does not touch the converter offset.
        ..Default::default()
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
    fn drift_fit_recovers_rate_and_curvature_and_ignores_a_short_or_flat_run() {
        // Too few points, or too short a span: no fit.
        let mut h: VecDeque<(f64, f64)> = [(0.0, 100.0), (1.0, 106.0), (2.0, 112.0)].into();
        assert_eq!(drift_fit(&h), (0.0, 0.0), "three points is not enough");

        // A clean 6 Hz/s straight walk over a full fit window: rate 6, no
        // curvature. The rate is reported at the middle of the window, which
        // for a straight line is the same everywhere.
        h = (0..=12).map(|i| (i as f64, 100.0 + 6.0 * i as f64)).collect();
        let (rate, accel) = drift_fit(&h);
        assert!((rate - 6.0).abs() < 0.05 && accel.abs() < 0.05, "rate {rate}, accel {accel}");

        // hz(t) = 100 + 2t + 0.5·t² → curvature 1.0 Hz/s², and rate at the
        // window middle (t = 6) is 2 + 1·6 = 8 Hz/s.
        h = (0..=12).map(|i| (i as f64, 100.0 + 2.0 * i as f64 + 0.5 * (i * i) as f64)).collect();
        let (rate, accel) = drift_fit(&h);
        assert!((accel - 1.0).abs() < 0.02, "curvature {accel}");
        assert!((rate - 8.0).abs() < 0.05, "mid-window rate {rate}");

        // Flat: both near zero.
        h = (0..=12).map(|i| (i as f64, 200.0)).collect();
        let (rate, accel) = drift_fit(&h);
        assert!(rate.abs() < 0.01 && accel.abs() < 0.01);

        // A single wild estimate cannot drag either term past its clamp.
        h = (0..=12).map(|i| (i as f64, 0.0)).collect();
        h.push_back((12.5, 1_000_000.0));
        let (rate, accel) = drift_fit(&h);
        assert!(rate.abs() <= 120.0 && accel.abs() <= 15.0);
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
            Qo100Settings {
                enabled: true,
                decode_telemetry: true,
                search_half_width_hz: 300.0,
                ..Default::default()
            },
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
            Qo100Settings {
                enabled: true,
                decode_telemetry: true,
                search_half_width_hz: 300.0,
                ..Default::default()
            },
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

    /// The fast tracker reports where the beacon sits from its spectral shape
    /// alone, with the telemetry decoder switched off — the split the QO-100
    /// page is built around.
    #[test]
    fn the_tracker_places_the_beacon_without_decoding_telemetry() {
        let rate = 60_000.0;
        let c = Qo100Controller::new(
            rate,
            Qo100Settings {
                enabled: true,
                decode_telemetry: false,
                park_lo_hz: 5_000.0,
                park_hi_hz: 20_000.0,
                ..Default::default()
            },
        );
        // A few seconds of the beacon parked at +12 kHz.
        let one = crate::bpsk::tests::synth_signal("TRACKED", rate, 12_000.0, 0.05, 3);
        c.on_rx_iq(&one);
        let s = wait_for(&c, |s| s.est_offset_hz.is_some())
            .expect("the tracker should place the beacon");
        assert!(s.tracking);
        assert_eq!(s.blocks_locked, 0, "the telemetry decoder was off");
        let est = s.est_offset_hz.unwrap();
        assert!((est - 12_000.0).abs() <= 350.0, "est {est}");
        assert!(s.est_updates >= 1 && s.est_null_depth_db >= 2.5, "{s:?}");
    }
}
