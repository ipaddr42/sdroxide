//! PI4 vocabulary shared by the engine, the wire protocol and the panel.
//!
//! **Interface facts only**, the same split [`crate::WsprSpot`]'s module doc
//! draws: the protocol itself — the sync vector, the message alphabet, the
//! convolutional code — lives in `sdroxide-digi::pi4`, which already carries
//! the `mfsk-core` GPL dependency the FEC is built on. What is here is what
//! the UI and the wire need in order to talk about a PI4 reception at all.
//!
//! Like a [`crate::WsprSpot`], a PI4 decode is **not** a message addressed to
//! anyone — there is no QSO, no exchange, nobody to answer. Unlike a WSPR
//! spot it carries no grid or power level either: the message is up to eight
//! characters from a 38-symbol alphabet, and a beacon spends nearly all of
//! that on its callsign (occasionally a status string instead — see
//! `sdroxide_digi::pi4::spec`'s module doc for the syntax, which this crate
//! does not attempt to parse further than the plain text).

use serde::{Deserialize, Serialize};

/// Slot length: one minute, the IARU Region 1 VHF Committee's mixed-mode
/// beacon cycle (PI4, then CW identification, then an unmodulated carrier).
pub const SLOT_S: f64 = 60.0;

/// On-air length of the PI4 portion: 146 symbols at 166.667 ms each.
pub const BURST_S: f64 = 146.0 * 2000.0 / 12_000.0;

/// One PI4 reception: a beacon this station decoded.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pi4Spot {
    /// Unix seconds at the start of the one-minute slot this was decoded
    /// against.
    pub slot_utc: i64,
    /// The message, trimmed of its trailing space padding — ordinarily a
    /// callsign.
    pub text: String,
    /// Which beacon-spacing variant this decode matched: `"PI4"` for the
    /// standard 1 kHz-spaced beacons, `"PI4-80"` / `"PI4-96"` / `"PI4-120"`
    /// for the wider ones. A label rather than a typed enum, so the wire
    /// format does not have to carry `sdroxide-digi`'s type across the
    /// engine/UI boundary for what is, on this side, decoration.
    pub variant: String,
    /// Absolute RF frequency of tone 0 (Hz): dial plus its audio frequency.
    pub tone0_hz: f64,
    /// Offset of the message's first symbol from the nominal slot boundary,
    /// in seconds.
    pub dt_sec: f32,
    /// A per-6-Hz-bin signal-to-noise estimate, in dB — see
    /// `sdroxide_digi::pi4::decode::Pi4Decode::snr_db` for what it is
    /// measured against and why it is not the 2500 Hz-referenced figure WSPR
    /// reports.
    pub snr_db: f32,
    /// How much of the received tone energy the message accounts for, `0..1`
    /// — the same anti-hallucination measure
    /// `sdroxide_digi::wspr::decode::fit_of` uses for WSPR, needed here for
    /// the same reason: this FEC carries no CRC.
    pub fit: f32,
}

/// What the PI4 engine is doing, for the panel's status strip.
///
/// `None` on [`crate::DigiStatus`] in every other mode — the same arrangement
/// [`crate::WsprStatus`] and [`crate::Js8Status`] use.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Pi4Status {
    /// Unix seconds at the start of the slot now in progress.
    pub slot_utc: i64,
    /// True while this slot's audio is still being searched. The decode is a
    /// grid search over start time, beacon variant and tone frequency and
    /// takes a real fraction of a second, so — as with WSPR — "nothing yet"
    /// and "nothing at all" are genuinely different states worth showing
    /// apart.
    pub decoding: bool,
    /// Receptions decoded in the last completed slot — a count, not the
    /// list, which travels as its own event.
    pub last_slot_spots: u32,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_pi4_burst_fits_inside_its_slot() {
        assert!(BURST_S < SLOT_S, "{BURST_S} does not fit inside {SLOT_S}");
        // 24 1/3 seconds is the figure the protocol page quotes.
        assert!((BURST_S - 24.333).abs() < 0.01, "burst is {BURST_S} s");
    }

    #[test]
    fn a_spot_serialises_and_a_default_status_is_not_decoding() {
        let s = Pi4Spot {
            slot_utc: 0,
            text: "OZ7IGY".into(),
            variant: "PI4".into(),
            tone0_hz: 144_470_682.8,
            dt_sec: 0.1,
            snr_db: -10.0,
            fit: 0.6,
        };
        let json = serde_json::to_string(&s).expect("Pi4Spot serialises");
        let back: Pi4Spot = serde_json::from_str(&json).expect("Pi4Spot deserialises");
        assert_eq!(back, s);
        assert!(!Pi4Status::default().decoding);
    }
}
