//! QO-100 narrowband beacon calibration: what the operator asks the engine
//! for, and what it reports back. Pure data + serde, shared by the native
//! engine and the UI (native + WASM) — the demodulation itself lives in the
//! native `sdroxide-qo100` crate, same split as [`crate::ism`].

use serde::{Deserialize, Serialize};

/// The QO-100 (Es'hail-2) narrowband transponder's 400 baud BPSK telemetry
/// beacon. Confirmed against the satellite's own published parameters (see
/// `sdroxide_qo100::bpsk` for the citation) — not a guess, and not one of the
/// satellite's other two beacons, which is why there is only one constant
/// here rather than a list.
pub const QO100_BEACON_HZ: f64 = 10_489_750_000.0;

/// What the operator asks the beacon decoder to do.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Qo100Settings {
    /// Whether the spectral tracker runs at all. Off by default: it is a
    /// calibration tool reached for occasionally, not something every station
    /// pays a downconverter and a worker thread for by default. When on, the
    /// engine watches the parking window ([`Self::park_lo_hz`]..
    /// [`Self::park_hi_hz`]) for the beacon's twin-lobe shape and reports
    /// where it sits, cycle after cycle.
    pub enabled: bool,
    /// Half the width, in Hz, of the frequency range the *telemetry* decoder
    /// ([`Self::decode_telemetry`]) sweeps around [`QO100_BEACON_HZ`]. The
    /// tracker does not use this — it searches the parking window instead.
    pub search_half_width_hz: f64,
    /// The low and high edge, in Hz above [`QO100_BEACON_HZ`], of the window
    /// the operator parks the beacon in before switching the tracker on. A
    /// positive lane clear of the DC spike and of most transponder activity:
    /// the operator nudges the dial until the beacon's two lobes sit inside
    /// it, so the tracker is confirming a shape already on screen rather than
    /// hunting blind. Default +5 kHz..+12 kHz — kept modest because the
    /// receiver has to sample fast enough to see the top of it, and a wider
    /// window is real load. Raise it only if the LNB is far enough off that
    /// the beacon will not sit inside the default.
    pub park_lo_hz: f64,
    pub park_hi_hz: f64,
    /// Whether to also run the AO-40 uncoded frame decoder (sync word + CRC +
    /// telemetry text). Separate from [`Self::enabled`]: the tracker keeps the
    /// dial honest continuously off the beacon's *shape*, while decoding the
    /// telemetry itself is a heavier, once-in-a-while thing the operator asks
    /// for explicitly.
    pub decode_telemetry: bool,
    /// Whether the tracker corrects `RadioConfig::converter_offset_hz` by
    /// itself: a slow closed loop that, on a clean and steady estimate,
    /// nudges the offset so the beacon (and the receiver behind it) lands
    /// back on [`QO100_BEACON_HZ`], then holds it there as the LNB drifts
    /// with temperature. Off by default — it reopens the front end each time
    /// it acts.
    pub auto_apply: bool,
}

impl Default for Qo100Settings {
    fn default() -> Self {
        Self {
            enabled: false,
            search_half_width_hz: 25_000.0,
            park_lo_hz: 5_000.0,
            park_hi_hz: 12_000.0,
            decode_telemetry: false,
            auto_apply: false,
        }
    }
}

/// What the engine tells the window about the decoder's own state. Re-sent
/// whenever it changes, the same convention [`crate::IsmStatus`] follows.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Qo100Status {
    /// Whether the downconverter and worker are actually running — mirrors
    /// [`Qo100Settings::enabled`], but from the engine's side, so a client
    /// that opens mid-session sees the true state rather than assuming it.
    pub running: bool,
    /// Whether the most recent search block found a CRC-valid frame.
    pub locked: bool,
    /// How far the beacon actually sits from [`QO100_BEACON_HZ`], in Hz —
    /// only meaningful while `locked`. This *is* the calibration answer: the
    /// frequency the search had to assume before a frame's sync word and CRC
    /// both checked out.
    pub offset_hz: f64,
    /// The most recently decoded telemetry text, kept across a lock that
    /// later drops so the window does not go blank between frames (the
    /// beacon sends an uncoded frame roughly every 20 s, alternating with a
    /// coded one this decoder does not attempt — see the crate doc).
    pub text: String,
    /// Unix time of the last successful lock. 0 if there has never been one.
    pub locked_unix: i64,
    /// Search blocks attempted and how many produced a valid frame, since the
    /// decoder was switched on — the same reason [`crate::IsmStatus::bursts`]
    /// and `decodes` exist: a high `blocks_tried` with `blocks_locked` still
    /// at 0 says plainly that the search is running but the beacon has not
    /// been found yet, rather than looking merely idle.
    pub blocks_tried: u64,
    pub blocks_locked: u64,

    // --- spectral tracker (fast loop, off the beacon's twin-lobe shape) ---
    /// Whether the tracker loop is running its short-window spectrum checks.
    pub tracking: bool,
    /// The tracker's current estimate of how far the beacon sits from
    /// [`QO100_BEACON_HZ`], in Hz — `None` when the last cycle saw nothing
    /// twin-lobe-shaped in the parking window.
    pub est_offset_hz: Option<f64>,
    /// Depth of the central null between the two lobes, in dB — how
    /// convincingly the last estimate looked like the beacon rather than a
    /// plain carrier.
    pub est_null_depth_db: f32,
    /// Left/right evenness of the two lobes, 0..1 (1 = perfectly symmetric).
    pub est_symmetry: f32,
    /// Lobe level over the parking-window noise floor, in dB.
    pub est_snr_db: f32,
    /// Tracker cycles that produced an estimate, and cycles that found
    /// nothing, since the tracker was switched on.
    pub est_updates: u64,
    pub est_misses: u64,
    /// Least-squares drift rate of the last stretch of estimates, in Hz/s —
    /// what the frame decoder de-rotates before looking for a frame. 0 until
    /// there is a long enough run of estimates to fit. Reported at the middle
    /// of the fit window, so it lines up with the decode buffer's own centre.
    pub est_drift_hz_s: f32,
    /// Curvature of that same fit, in Hz/s² — a warming LNB's drift rate is
    /// not constant, and this second-order term is de-rotated on top of
    /// [`Self::est_drift_hz_s`]. 0 until the fit is long enough to mean
    /// anything.
    pub est_drift_accel_hz_s2: f32,

    // --- AO-40 uncoded decoder progress (only while `decoding`) ---
    /// Whether [`Qo100Settings::decode_telemetry`] is on and the decoder is
    /// actually being run.
    pub decoding: bool,
    /// The last decode pass found chip-rate energy to work on.
    pub carrier_seen: bool,
    /// The last decode pass matched the 32-bit AO-40 sync word somewhere
    /// (within the error threshold).
    pub sync_seen: bool,
    /// Fewest sync-word bit errors seen in the last decode pass, 0..=32;
    /// [`u8::MAX`] when the pass ran no bit stream at all.
    pub sync_bit_errors: u8,
    /// How many separate sync-word matches (within the error threshold) the
    /// last pass's best bitstream carried. One or two, at very few errors, is
    /// a real frame the payload demod is then failing; a lone match near the
    /// full error budget is the chance hit a 32-bit pattern makes in a buffer
    /// this long — which is how "sync lit, CRC dark" should be read.
    pub sync_matches: u8,
    /// How much of one whole AO-40 frame the rolling buffer currently spans,
    /// 0..1 — the decoder needs a full frame inside one window to have any
    /// chance, and this says how close it is.
    pub frame_fill: f32,
    /// The last decode pass's CRC check result.
    pub crc_ok: bool,

    // --- closed loop (only while `Qo100Settings::auto_apply`) ---
    /// The tracker's closed loop is armed and watching the estimate.
    pub auto_applying: bool,
    /// Signed Hz the loop has written into the converter offset since the
    /// tracker came on, and how many separate corrections that took.
    pub auto_total_hz: f64,
    pub auto_applies: u64,
    /// The most recent single correction, and the unix second it was made
    /// (0 if the loop has not acted yet).
    pub auto_last_hz: f64,
    pub auto_last_unix: i64,
}
