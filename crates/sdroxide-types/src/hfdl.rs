//! HFDL (ARINC 635) ground-network decode: what the operator asks the engine
//! for, and what it reports back. Pure data + serde, shared by the native
//! engine and the UI (native + WASM) — the demodulation itself lives in the
//! native `sdroxide-hfdl` crate, same split as [`crate::qo100`] and
//! [`crate::ism`].
//!
//! As with the other not-a-mode lanes (ADS-B, VDL2, AIS, QO-100), HFDL rides
//! its own downconversion of the raw I/Q rather than the audio chain: the
//! engine hands the lane the complex baseband around the tuned channel, and
//! the worker's per-channel decoder does the rest.

use serde::{Deserialize, Serialize};

/// The default channel to listen on: Riverhead's primary 21 931 kHz — the
/// frequency the reference off-air capture that validated the whole chain was
/// recorded on, and the strongest single HFDL ground signal most of the north
/// Atlantic/North America can reach.
pub const HFDL_DEFAULT_HZ: f64 = 21_931_000.0;

/// The engine's HFDL lane decimates the raw I/Q to this rate before handing
/// it to [`crate::HfdlSettings`]-configured worker: the same 24 kS/s the
/// off-air validation used (a 2.8 kHz USB channel inside a ±12 kHz lane, with
/// the +1440 Hz subcarrier shift handled inside the decoder core). Keeping
/// the lane rate fixed at the validated one is deliberate — HFDL's demod
/// constants live at 12 kS/s ≈ 6.67 samples per 1800-Bd symbol, and this is
/// the rate the whole chain was proven against.
pub const HFDL_LANE_RATE_HZ: f64 = 24_000.0;

/// What the operator asks the HFDL decoder to do.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct HfdlSettings {
    /// Whether the decoder runs at all. Off by default: it is a
    /// sometimes-reached utility lane, not something every station pays a
    /// downconverter and a worker thread for by default, the same reason
    /// [`crate::Qo100Settings::enabled`] is off.
    pub enabled: bool,
    /// The USB channel frequency (the HFDL "assigned frequency") the lane
    /// centres on, in Hz. The decoder's subcarrier sits at +1440 Hz, and the
    /// lane is a fixed 24 kS/s channel centred on this — so it works wherever
    /// the radio itself is tuned, exactly like the QO-100 beacon lane.
    pub frequency_hz: f64,
}

impl Default for HfdlSettings {
    fn default() -> Self {
        Self { enabled: false, frequency_hz: HFDL_DEFAULT_HZ }
    }
}

/// One aircraft position an HFDL downlink carried.
///
/// The normalized object xng lifts out of a performance-data (0xD1) or
/// frequency-data (0xD5) HFNPDU: a 20-bit lat/lon pair and whatever identity
/// the payload held — the GS-local downlink alias, and the ICAO address when
/// the logon-confirm cache resolved the alias to one. The all-zero
/// not-yet-acquired fix is dropped upstream, so a `Some` here is a real
/// position.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HfdlFix {
    pub lat: f64,
    pub lon: f64,
    /// The GS-local downlink alias (8-bit). *Not* the ICAO address.
    pub aircraft_id: Option<u32>,
    /// The ICAO address, when the logon-confirm cache resolved the alias.
    pub icao: Option<String>,
    /// The flight identifier the payload carried, when it carried one.
    pub flight: Option<String>,
}

impl HfdlFix {
    /// The identity the map keys an aircraft by: the ICAO when it is known,
    /// else the GS-local alias, else the flight — most stable first, so an
    /// aircraft that resolves its ICAO mid-session keeps one plot rather than
    /// splitting into two.
    pub fn key(&self) -> String {
        if let Some(icao) = self.icao.as_deref().filter(|s| !s.is_empty()) {
            return format!("icao:{icao}");
        }
        if let Some(id) = self.aircraft_id {
            return format!("ac:{id}");
        }
        if let Some(flt) = self.flight.as_deref().map(str::trim).filter(|s| !s.is_empty()) {
            return format!("flt:{flt}");
        }
        // No identity at all (an un-resolved downlink): key by the rounded
        // position so the fix still plots, even if it cannot be followed.
        format!("pos:{:.2},{:.2}", self.lat, self.lon)
    }

    /// What the map labels the aircraft: the flight, then the ICAO, then the
    /// alias, and a bare `#?` when the payload named nothing.
    pub fn label(&self) -> String {
        if let Some(flt) = self.flight.as_deref().map(str::trim).filter(|s| !s.is_empty()) {
            return flt.to_string();
        }
        if let Some(icao) = self.icao.as_deref().filter(|s| !s.is_empty()) {
            return icao.to_string();
        }
        match self.aircraft_id {
            Some(id) => format!("#{id}"),
            None => "#?".to_string(),
        }
    }
}

/// One decoded HFDL event, as it appears in the panel's log.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HfdlDecode {
    /// Unix second it was decoded.
    pub unix: i64,
    /// The event kind, in xng's terms: "squitter", "logon", "logoff",
    /// "position", "performance-data", "frequency-data", "acars", ... — the
    /// string the SPDU/MPDU parser produced.
    pub kind: String,
    /// The ground station the event names, when the payload carried one
    /// (squitters and frequency data always do) — `name (GS n)`.
    pub gs: Option<String>,
    /// The channel frequency, in kHz.
    pub freq_khz: u32,
    /// EVM-derived burst SNR in dB, where the demod measured one.
    pub snr_db: Option<f32>,
    /// Measured carrier-frequency offset in Hz.
    pub freq_skew_hz: Option<f32>,
    /// Coded symbols the Viterbi decoder corrected for the burst.
    pub fec_corrected: Option<u32>,
    /// The payload details as compact JSON — every field xng's parser
    /// surfaced for the kind (aircraft id, position, UTC, ground station id,
    /// frequencies in use, ...).
    pub details: String,
    /// The aircraft position this event carried, where it carried one. Both
    /// the performance-data and frequency-data records do; squitters, logons
    /// and the rest do not. Lifted out of [`Self::details`] so the panel and
    /// the map need not parse JSON. Appended last: the wire is positional.
    pub position: Option<HfdlFix>,
}

/// What the engine tells the window about the decoder's own state. Re-sent
/// whenever it changes, the same convention [`crate::Qo100Status`] follows.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HfdlStatus {
    /// Whether the downconverter and worker are actually running — mirrors
    /// [`HfdlSettings::enabled`], but from the engine's side, so a client
    /// that opens mid-session sees the true state rather than assuming it.
    pub running: bool,
    /// Burst average level of the channel, in dBFS, as the demod sees it —
    /// a low figure while tuned to dead air says plainly that the lane is up
    /// but no station is in it, rather than looking merely idle.
    pub level_dbfs: f32,
    /// Bursts captured since the decoder was switched on — a burst is the
    /// demod-level unit; most carry no recognized payload, so a high burst
    /// count with few [`Self::decodes`] is a station up but idle.
    pub bursts: u64,
    /// Bursts that produced at least one decoded event.
    pub decodes: u64,
    /// The most recent decoded events, newest first, bounded to
    /// [`HFDL_LOG_DEPTH`] — the panel's log. A ground station's squitter
    /// repeats every ~32 s, so a fresh entry about that often is the norm,
    /// not evidence of noise; aircraft traffic between squitters is the
    /// interesting part. Carried whole on every status, the same snapshot
    /// convention [`crate::AdsbStatus`] uses: a dropped status costs nothing
    /// because the next one carries the same information.
    pub log: Vec<HfdlDecode>,
}

impl Default for HfdlStatus {
    fn default() -> Self {
        Self { running: false, level_dbfs: 0.0, bursts: 0, decodes: 0, log: Vec::new() }
    }
}

/// How many [`HfdlStatus::log`] entries a worker keeps in its rolling log —
/// a session's worth of HFDL is chatty (a squitter every ~32 s plus the
/// aircraft traffic), and a panel has no need of more than a screenful.
pub const HFDL_LOG_DEPTH: usize = 64;
