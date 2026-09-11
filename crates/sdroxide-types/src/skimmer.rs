//! Skimmer domain types, shared by the native engine, the wire protocol, and
//! the UI (native + WASM). Pure data + serde — the actual decoding lives in the
//! native `sdroxide-skimmer` crate. Designed to be skimmer-kind-agnostic so
//! future skimmers (RTTY/PSK/…) reuse the same event, wire, and overlay path.

use serde::{Deserialize, Serialize};

/// What kind of skimmer produced a spot. The wire event, UI overlay, and
/// engine seam are generic over this.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SkimmerKind {
    Cw,
    Psk,
    Rtty,
}

impl SkimmerKind {
    /// Every kind, in UI order. Also the index order of [`SkimmerSettings`].
    pub const ALL: [SkimmerKind; 3] = [SkimmerKind::Cw, SkimmerKind::Psk, SkimmerKind::Rtty];

    /// The operating mode a spot of this kind tunes to on click.
    pub fn mode(self) -> crate::Mode {
        match self {
            SkimmerKind::Cw => crate::Mode::Cw,
            SkimmerKind::Psk => crate::Mode::Psk,
            SkimmerKind::Rtty => crate::Mode::Rtty,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            SkimmerKind::Cw => "CW",
            SkimmerKind::Psk => "PSK",
            SkimmerKind::Rtty => "RTTY",
        }
    }

    /// Position in the per-kind arrays of [`SkimmerSettings`].
    pub fn index(self) -> usize {
        match self {
            SkimmerKind::Cw => 0,
            SkimmerKind::Psk => 1,
            SkimmerKind::Rtty => 2,
        }
    }
}

/// Which of the two decoders copies CW — for the skimmer, and for the CW panel
/// (see [`crate::DigiConfig::cw_engine`]).
///
/// Finding the signals and reading them are separate jobs, and only the second
/// one is expensive. The neural decoder copies several dB below where a timing
/// fit falls apart and reads hand-sent CW that no fit would accept, but it is a
/// Conformer run once per station per interval, and on a busy band that is real
/// work. The timing decoder reads the envelope the detector has already
/// computed, so it costs almost nothing on top of finding the signal at all —
/// and it needs about 8 dB SNR, where the model reaches −6.
///
/// They also do not copy the same alphabet. DeepCW's output layer has 41
/// classes — the letters, the digits, four marks and the space — and no room
/// for anything else, so the accented letters ITU-R M.1677-1 lists (Ä, Ö, Å, Ü,
/// É …) can only ever come out of the timing decoder, which reads the element
/// string and looks it up (issue #382).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum CwEngine {
    /// DeepCW, the neural decoder.
    #[default]
    Neural,
    /// The envelope-timing decoder in `sdroxide-dsp`.
    Timing,
}

impl CwEngine {
    /// Every decoder, in UI order.
    pub const ALL: [CwEngine; 2] = [CwEngine::Neural, CwEngine::Timing];

    /// The tag the chips wear.
    pub fn label(self) -> &'static str {
        match self {
            CwEngine::Neural => "NEURAL",
            CwEngine::Timing => "TIMING",
        }
    }

    /// What the hover text says the choice costs.
    pub fn hint(self) -> &'static str {
        match self {
            CwEngine::Neural => {
                "Reads weak and hand-sent CW; costs real CPU per station, and copies no \
                 accented letters"
            }
            CwEngine::Timing => {
                "Nearly free and copies the accented letters, but needs a clean signal \
                 (~8 dB SNR)"
            }
        }
    }
}

/// Most stations the neural decoder reads at once, by name.
///
/// The detector tracks far more than any of these; the ones past the cap keep
/// their marker and their frequency and simply carry no text.
pub const CW_SLOT_CHOICES: [u8; 4] = [8, 16, 32, 64];

/// Which skimmers run, and how hard each squelches its own spots. Owned by the
/// engine (it lives in [`crate::RadioState`]), edited from the SKIM popup and
/// persisted across restarts (`skimmer.json`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct SkimmerSettings {
    /// Per [`SkimmerKind::index`]: whether that skimmer decodes at all. A
    /// switched-off skimmer costs no DSP and emits no spots.
    pub enabled: [bool; 3],
    /// Per [`SkimmerKind::index`]: the minimum SNR (dB) a track must reach
    /// before it is reported as a spot. `0` reports whatever decodes.
    pub squelch_db: [i16; 3],
    /// Which decoder reads the CW skimmer's signals.
    pub cw_decoder: CwEngine,
    /// How many stations the neural decoder reads at once. Ignored by the
    /// timing decoder, which reads every track it is given.
    pub cw_slots: u8,
}

/// Default for [`SkimmerSettings::cw_slots`], and the value the old fixed cap
/// had.
pub const CW_SLOTS_DEFAULT: u8 = 32;

impl Default for SkimmerSettings {
    fn default() -> Self {
        SkimmerSettings {
            enabled: [true; 3],
            squelch_db: [0; 3],
            cw_decoder: CwEngine::Neural,
            cw_slots: CW_SLOTS_DEFAULT,
        }
    }
}

impl SkimmerSettings {
    /// Nothing running — the state for devices without a wideband IQ stream.
    ///
    /// The decoder choice matches [`Default`] rather than being meaningful here:
    /// this is what the engine substitutes when the source has no IQ to skim, and
    /// an operator swapping to such a radio and back should find their setting
    /// where they left it.
    pub const OFF: SkimmerSettings = SkimmerSettings {
        enabled: [false; 3],
        squelch_db: [0; 3],
        cw_decoder: CwEngine::Neural,
        cw_slots: CW_SLOTS_DEFAULT,
    };

    pub fn enabled(&self, kind: SkimmerKind) -> bool {
        self.enabled[kind.index()]
    }

    pub fn set_enabled(&mut self, kind: SkimmerKind, on: bool) {
        self.enabled[kind.index()] = on;
    }

    pub fn squelch_db(&self, kind: SkimmerKind) -> i16 {
        self.squelch_db[kind.index()]
    }

    pub fn set_squelch_db(&mut self, kind: SkimmerKind, db: i16) {
        self.squelch_db[kind.index()] = db;
    }

    /// True while at least one skimmer is running — what the engine starts and
    /// stops the shared skim window on.
    pub fn any_enabled(&self) -> bool {
        self.enabled.iter().any(|&on| on)
    }
}

/// One decoded signal from a skimmer: a station heard at a frequency, with a
/// (possibly not-yet-known) callsign and a rolling tail of decoded text.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SkimmerSpot {
    /// Stable id for this track, so the UI can update a box in place (and keep
    /// its message scrolling) rather than recreating it each update.
    pub id: u64,
    pub kind: SkimmerKind,
    /// Absolute RF frequency of the signal (Hz).
    pub freq_hz: f64,
    /// Best-guess callsign extracted from the decoded text, if any.
    pub callsign: Option<String>,
    /// Rolling tail of decoded text (most recent characters).
    pub text: String,
    /// Signal-to-noise estimate (dB).
    pub snr_db: i16,
    /// Estimated speed in words per minute.
    pub wpm: u16,
    /// True while the signal is currently keying (recently active).
    pub active: bool,
}
