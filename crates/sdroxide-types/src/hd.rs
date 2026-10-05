//! HD Radio (NRSC-5) receive status.
//!
//! What the decoder knows about the multiplex it is listening to, as one
//! latest-wins snapshot — the same shape as [`crate::DrmStatus`], and for the
//! same reason. The sync state, the sideband error ratios and the station's
//! own name are simply *current*: none of them is a delta, and each is worth
//! showing as the present state of its stage.

use serde::{Deserialize, Serialize};

/// One audio service announced by the multiplex's station information.
///
/// FM carries up to eight; in practice a station broadcasts its main programme
/// on HD-1 and one or two subchannels (HD-2, HD-3) beside it. The AM band has
/// only the one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct HdAudioService {
    /// The programme number the decoder uses, 0-based — HD-1 is 0.
    #[serde(default)]
    pub program: u8,
    /// `NRSC5_ACCESS_PUBLIC` (0) or `NRSC5_ACCESS_RESTRICTED` (1).
    #[serde(default)]
    pub access: u8,
    /// Audio codec mode, per SY_IDD_1017s Table 5-2. Low nibble is the codec
    /// (1 = MP1, 2 = MP2, …), the rest the bitrate class.
    #[serde(default)]
    pub codec_mode: u8,
}

impl HdAudioService {
    /// The programme as an operator reads it: the number is one-based on the
    /// air, so HD-1 is programme 1.
    pub fn program_number(self) -> u8 {
        self.program + 1
    }

    /// Whether the service is scrambled.
    pub fn restricted(self) -> bool {
        self.access != 0
    }

    /// The audio codec mode, as the four-bit field the frame header carries
    /// (NRSC-5 1017s Table 5-2, "Audio codec mode definitions"). Real FM
    /// broadcasts use one value throughout, so this is a diagnostic rather
    /// than something an operator acts on; it is shown as the raw code rather
    /// than guessed at, because the table's names are the codec family's and
    /// naming the wrong one would be worse than showing the number.
    pub fn codec_label(self) -> String {
        format!("codec mode {}", self.codec_mode)
    }
}

/// Everything the HD Radio decoder knows right now.
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct HdRadioStatus {
    /// A lock is held: the OFDM frame is being read, whether or not the audio
    /// is coming out cleanly. Everything below is only meaningful while this
    /// holds.
    #[serde(default)]
    pub locked: bool,
    /// The selected programme is producing sound: its latest audio frame was
    /// decoded, not the silence the decoder fills a missing or failed packet
    /// with.
    #[serde(default)]
    pub audio: bool,
    /// Residual carrier frequency offset, in Hz, as the sync reported it.
    #[serde(default)]
    pub freq_offset_hz: f32,
    /// Primary Service Mode Indicator (1, 2, 3, 5, 6 or 11 in FM).
    #[serde(default)]
    pub psmi: i32,
    /// Modulation error ratio of the lower digital sideband, in dB.
    #[serde(default)]
    pub mer_lower_db: f32,
    /// Modulation error ratio of the upper digital sideband, in dB.
    #[serde(default)]
    pub mer_upper_db: f32,
    /// Channel bit-error ratio, 0.0 to 1.0.
    #[serde(default)]
    pub cber: f32,

    /// Which programme the decoder is playing, 0-based.
    #[serde(default)]
    pub program: u8,
    #[serde(default)]
    pub audio_services: Vec<HdAudioService>,
    /// The broadcaster's station name, e.g. "Q107".
    #[serde(default)]
    pub station_name: String,
    /// The station slogan, e.g. "You're Listening to Q".
    #[serde(default)]
    pub station_slogan: String,
    /// A short text the broadcaster is currently airing.
    #[serde(default)]
    pub station_message: String,

    /// Why no decoder is running, when none can: the sentence names the number
    /// it is about. `None` while decoding is possible, locked or not.
    #[serde(default)]
    pub unavailable: Option<String>,
}

impl HdRadioStatus {
    /// The receiver is decoding audio, not merely holding sync on a carrier.
    pub fn decoding(&self) -> bool {
        self.locked && self.audio
    }

    /// A one-line summary for a status bar: the station's name if the
    /// multiplex has named itself, else how far the chain has got.
    pub fn summary(&self) -> String {
        if self.unavailable.is_some() {
            return "unavailable".to_string();
        }
        if !self.station_name.is_empty() {
            return self.station_name.clone();
        }
        if self.locked { "acquiring service".to_string() } else { "no signal".to_string() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The programme number is one-based on the air but zero-based in the
    /// decoder, and a panel that showed the raw field would label HD-1 as
    /// "HD-0".
    #[test]
    fn the_programme_number_is_one_based() {
        let svc = HdAudioService { program: 0, access: 0, codec_mode: 3 };
        assert_eq!(svc.program_number(), 1);
        assert!(!svc.restricted());
        assert_eq!(svc.codec_label(), "codec mode 3");
    }

    /// Sync without audio is not decoding — a muting station, or a programme
    /// nobody selected, still holds the frame.
    #[test]
    fn a_carrier_with_no_audio_is_not_decoding() {
        let mut s = HdRadioStatus { locked: true, ..Default::default() };
        assert!(!s.decoding());
        s.audio = true;
        assert!(s.decoding());
    }
}
