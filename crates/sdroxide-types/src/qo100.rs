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
    /// Whether the decoder runs at all. Off by default: it is a calibration
    /// tool reached for occasionally, not something every station pays a
    /// downconverter and a worker thread for by default.
    pub enabled: bool,
    /// Half the width, in Hz, of the frequency range searched around
    /// [`QO100_BEACON_HZ`] — set from the QO-100 window's own width buttons.
    pub search_half_width_hz: f64,
}

impl Default for Qo100Settings {
    fn default() -> Self {
        Self { enabled: false, search_half_width_hz: 5_000.0 }
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
}
