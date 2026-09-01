//! Working a repeater: the transmit shift, the sub-audible tone that goes out
//! under the voice, and the 1750 Hz burst that opens a carrier-access machine.
//!
//! A repeater listens on one frequency and answers on another, so a station
//! working one transmits somewhere it is not listening. That is the whole of
//! the "duplex" setting: a direction and a magnitude, applied to the dial on
//! transmit and nowhere else. It rides alongside — not instead of — split and
//! XIT, which are the other two things that move the transmit frequency; see
//! [`crate::RadioState::tx_freq_hz`].
//!
//! Access is the other half. Most repeaters want a continuous sub-audible tone
//! under the voice (CTCSS, or DCS where the code is a data stream rather than a
//! tone); much of Region 1 still opens on a 1750 Hz burst at the start of the
//! over. Both are transmit-side, and both are separate from the *receive* tone
//! squelch in [`crate::RxState::tone_sql`], which is what a station arms to
//! ignore everything not addressed to it. They are set together in one place
//! because in practice they are set together — a repeater directory gives the
//! output, the shift and the tone as one line — but they are three independent
//! settings and the state keeps them that way.

use serde::{Deserialize, Serialize};

use crate::Region;
use crate::region::mask;

/// Which side of the output frequency a repeater's input sits on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum Shift {
    /// No shift: transmit where you listen. The default, and what every
    /// non-repeater contact runs on.
    #[default]
    Simplex,
    /// Transmit below the dial — the direction nearly every repeater plan in
    /// the world uses.
    Minus,
    /// Transmit above the dial.
    Plus,
}

impl Shift {
    pub const ALL: [Shift; 3] = [Shift::Simplex, Shift::Minus, Shift::Plus];

    /// How a radio's display writes it.
    pub fn label(self) -> &'static str {
        match self {
            Shift::Simplex => "SIMPLEX",
            // A real minus sign rather than a hyphen: beside the plus below,
            // a hyphen reads a good deal smaller than it should.
            Shift::Minus => "−",
            Shift::Plus => "+",
        }
    }

    /// What multiplies the offset magnitude to give a signed shift.
    pub fn sign(self) -> f64 {
        match self {
            Shift::Simplex => 0.0,
            Shift::Minus => -1.0,
            Shift::Plus => 1.0,
        }
    }
}

/// What rides under the voice on an outgoing FM over.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
pub enum ToneMode {
    /// Nothing: a carrier-access repeater, or simplex.
    #[default]
    Off,
    /// A continuous CTCSS tone from the standard table.
    Ctcss,
    /// A continuous DCS data stream carrying one of the standard codes.
    Dcs,
}

impl ToneMode {
    pub const ALL: [ToneMode; 3] = [ToneMode::Off, ToneMode::Ctcss, ToneMode::Dcs];

    pub fn label(self) -> &'static str {
        match self {
            ToneMode::Off => "OFF",
            ToneMode::Ctcss => "CTCSS",
            ToneMode::Dcs => "DCS",
        }
    }
}

/// The sub-audible signalling to transmit, with everything the modulator needs
/// to build it — which is more than [`crate::SubTone`] carries, because that
/// type describes what a *receiver* could establish and a DCS receiver here
/// deliberately does not claim to read the code.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum TxSubTone {
    /// A CTCSS tone, in tenths of a Hz — one of [`crate::CTCSS_TONES`].
    Ctcss(u16),
    /// A DCS code, written the way radios write it: the three octal digits as
    /// a decimal number, so `023` is `23` and `754` is `754`. `invert` is the
    /// "I" polarity, where every transmitted bit is complemented.
    Dcs { code: u16, invert: bool },
}

impl TxSubTone {
    /// How a radio's display and a repeater directory write it: `88.5`, or
    /// `D023N`.
    pub fn label(self) -> String {
        match self {
            TxSubTone::Ctcss(tenths) => format!("{}.{}", tenths / 10, tenths % 10),
            TxSubTone::Dcs { code, invert } => {
                format!("D{code:03}{}", if invert { "I" } else { "N" })
            }
        }
    }
}

/// The 104 standard DCS codes, written as radios write them — the three octal
/// digits read as a decimal number.
///
/// Octal-as-decimal rather than the 9-bit value they encode because every
/// place an operator meets one of these — the radio's display, a repeater
/// directory, the local club's web page — writes them this way, and converting
/// at the edge would mean the number on screen and the number in the file were
/// different. [`dcs_bits`] does the conversion where the modulator needs it.
pub const DCS_CODES: [u16; 104] = [
    23, 25, 26, 31, 32, 36, 43, 47, 51, 53, 54, 65, 71, 72, 73, 74, 114, 115, 116, 122, 125, 131,
    132, 134, 143, 145, 152, 155, 156, 162, 165, 172, 174, 205, 212, 223, 225, 226, 243, 244, 245,
    246, 251, 252, 255, 261, 263, 265, 266, 271, 274, 306, 311, 315, 325, 331, 332, 343, 346, 351,
    356, 364, 365, 371, 411, 412, 413, 423, 431, 432, 445, 446, 452, 454, 455, 462, 464, 465, 466,
    503, 506, 516, 523, 526, 532, 546, 565, 606, 612, 624, 627, 631, 632, 654, 662, 664, 703, 712,
    723, 731, 732, 734, 743, 754,
];

/// The nine data bits a DCS code carries, from the three octal digits the code
/// is written as. Returns `None` for a number that is not three octal digits.
pub fn dcs_bits(code: u16) -> Option<u16> {
    let (d2, d1, d0) = (code / 100, (code / 10) % 10, code % 10);
    if d2 > 7 || d1 > 7 || d0 > 7 || code > 777 {
        return None;
    }
    Some((d2 << 6) | (d1 << 3) | d0)
}

/// The 1750 Hz tone that opens a carrier-access repeater in much of Region 1.
pub const TONE_BURST_HZ: f64 = 1750.0;

/// The range a tone burst's length is clamped to. Shorter than 100 ms and no
/// repeater's decoder has time to see it; past two seconds it has stopped
/// being a burst and is just a carrier with a whistle on it.
pub const BURST_MS_RANGE: std::ops::RangeInclusive<u32> = 100..=2000;

/// The largest shift offered, in Hz.
///
/// Set by the widest standard plan rather than by anything in the radio: the
/// Americas work 33 cm on a 25 MHz shift, which is the biggest figure any
/// published band plan asks for. Wide enough for a hand-entered oddity, and
/// narrow enough that a corrupted file cannot transmit half a band away from
/// where the operator is listening.
pub const MAX_OFFSET_HZ: u32 = 50_000_000;

/// Everything about working a repeater, as one setting.
///
/// `Copy` and small — it travels inside [`crate::RadioState`] on every state
/// broadcast, and a stored memory keeps one of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct RepeaterState {
    pub shift: Shift,
    /// Magnitude of the shift, in Hz. Always positive; [`Shift`] carries the
    /// direction, so switching a repeater from minus to plus cannot silently
    /// lose the offset that was already set.
    pub offset_hz: u32,
    /// Take the shift from the band plan as the dial moves, instead of holding
    /// whatever was set by hand.
    ///
    /// The engine *resolves* this: with `auto` on it writes the plan's answer
    /// into `shift` and `offset_hz` themselves, so the state always says what
    /// will actually happen and every client — and the memory that stores it —
    /// reads a settled figure rather than a rule it would have to re-run. See
    /// [`standard_shift`].
    pub auto: bool,
    /// What goes out under the voice.
    pub tone: ToneMode,
    /// The CTCSS tone to transmit, in tenths of a Hz. Kept across a switch to
    /// DCS and back, like `offset_hz` across a change of direction.
    pub ctcss_tenths: u16,
    /// The DCS code to transmit, as three octal digits read as a decimal
    /// number — see [`DCS_CODES`].
    pub dcs_code: u16,
    /// DCS "I" polarity: every transmitted bit complemented.
    pub dcs_invert: bool,
    /// Send the 1750 Hz burst at the start of every over, rather than only
    /// when the operator asks for one ([`crate::Command::ToneBurst`]).
    pub burst_auto: bool,
    /// How long that burst lasts, in ms.
    pub burst_ms: u32,
}

impl Default for RepeaterState {
    fn default() -> Self {
        RepeaterState {
            shift: Shift::Simplex,
            // The 2 m shift the whole world uses, so the first press of the
            // minus chip on 2 m is already right.
            offset_hz: 600_000,
            auto: false,
            tone: ToneMode::Off,
            // 88.5 Hz — the most-used CTCSS tone there is.
            ctcss_tenths: 885,
            dcs_code: 23,
            dcs_invert: false,
            burst_auto: false,
            // Half a second: long enough for every repeater decoder, short
            // enough not to be rude on a busy channel.
            burst_ms: 500,
        }
    }
}

impl RepeaterState {
    /// The signed transmit shift in Hz — zero on simplex.
    pub fn shift_hz(self) -> f64 {
        self.shift.sign() * self.offset_hz as f64
    }

    /// The sub-audible signalling this setting transmits, or `None` with the
    /// tone off.
    pub fn tx_tone(self) -> Option<TxSubTone> {
        match self.tone {
            ToneMode::Off => None,
            ToneMode::Ctcss => Some(TxSubTone::Ctcss(self.ctcss_tenths)),
            ToneMode::Dcs => Some(TxSubTone::Dcs { code: self.dcs_code, invert: self.dcs_invert }),
        }
    }

    /// Whether anything here changes what goes on the air.
    pub fn is_active(self) -> bool {
        self.shift != Shift::Simplex || self.tone != ToneMode::Off || self.burst_auto
    }

    /// The shift as a radio's display writes it: `−600 kHz`, or `SIMPLEX`.
    pub fn shift_label(self) -> String {
        if self.shift == Shift::Simplex {
            return "SIMPLEX".to_string();
        }
        // kHz below a megahertz and MHz above it, the way every band plan
        // writes these — and with the trailing zeros off the fraction, so the
        // common whole figures read "−600 kHz" rather than "−600.0000 kHz"
        // while an odd one keeps every digit it needs.
        let (n, unit) = if self.offset_hz >= 1_000_000 {
            (self.offset_hz as f64 / 1e6, "MHz")
        } else {
            (self.offset_hz as f64 / 1e3, "kHz")
        };
        let mut digits = format!("{n:.4}");
        if digits.contains('.') {
            digits = digits.trim_end_matches('0').trim_end_matches('.').to_string();
        }
        format!("{}{digits} {unit}", self.shift.label())
    }

    /// This setting with every field forced into the range the controls offer.
    ///
    /// Three doors lead into this type that the controls do not guard: a
    /// `SetRepeater` from a remote client, the `session.json` restored at
    /// startup, and a `memories.json` a memory was recalled from. All three are
    /// files or peers an operator may have edited, and what comes out of here
    /// decides a transmit frequency and drives a modulator — so an offset of
    /// half a band or a CTCSS "tone" that is not in the table is refused here
    /// rather than at each of the places that would act on it.
    pub fn clamped(mut self) -> Self {
        self.offset_hz = self.offset_hz.min(MAX_OFFSET_HZ);
        if !crate::CTCSS_TONES.contains(&self.ctcss_tenths) {
            self.ctcss_tenths = RepeaterState::default().ctcss_tenths;
        }
        if !DCS_CODES.contains(&self.dcs_code) {
            self.dcs_code = RepeaterState::default().dcs_code;
        }
        self.burst_ms = self.burst_ms.clamp(*BURST_MS_RANGE.start(), *BURST_MS_RANGE.end());
        self
    }
}

/// One entry of the standard repeater plan: a stretch of repeater *outputs*,
/// and the shift a transceiver applies inside it.
struct RepeaterPlan {
    lo: f64,
    hi: f64,
    shift: Shift,
    offset_hz: u32,
    /// Which regions this entry belongs to — see [`crate::region::mask`].
    regions: u8,
}

/// The conventional repeater shifts, by the sub-band the *output* falls in.
///
/// ⚠️ Transcribed from published band plans — the IARU regional VHF/UHF plans
/// and the national society plans that implement them — and never checked
/// against a repeater. It exists so that tuning to a repeater's output usually
/// puts the transmitter on its input without the operator looking anything up;
/// it is not a substitute for the local plan, and the operator can always set
/// the shift by hand.
///
/// The table is deliberately sparse. Only the sub-bands whose shift is settled
/// across a whole region are listed, because the failure mode of a wrong entry
/// is worse than the failure mode of a missing one: a missing entry leaves the
/// radio simplex, which is obvious the moment nobody comes back, while a wrong
/// one transmits confidently onto somebody else's channel. Everywhere the
/// table says nothing, [`standard_shift`] answers `None` and the operator sets
/// the shift themselves.
///
/// Every entry is a range of repeater OUTPUTS, not of the whole band: a shift
/// that applied everywhere on 2 m would put the transmitter 600 kHz down while
/// the operator called on the simplex calling channel.
const REPEATER_PLAN: &[RepeaterPlan] = &[
    // 10 m FM. The one repeater sub-band the whole world shares.
    RepeaterPlan {
        lo: 29_620_000.0,
        hi: 29_700_000.0,
        shift: Shift::Minus,
        offset_hz: 100_000,
        regions: mask::ALL,
    },
    // 6 m, Region 1.
    RepeaterPlan {
        lo: 51_210_000.0,
        hi: 51_390_000.0,
        shift: Shift::Minus,
        offset_hz: 500_000,
        regions: mask::R1,
    },
    // 4 m, Region 1's alone — 70 MHz is not an amateur band anywhere else.
    RepeaterPlan {
        lo: 70_425_000.0,
        hi: 70_487_500.0,
        shift: Shift::Minus,
        offset_hz: 500_000,
        regions: mask::R1,
    },
    // 2 m, Region 1.
    RepeaterPlan {
        lo: 145_587_500.0,
        hi: 145_800_000.0,
        shift: Shift::Minus,
        offset_hz: 600_000,
        regions: mask::R1,
    },
    // 2 m, Region 2 — two output sub-bands, and the upper one is the reason
    // this table keys on the sub-band rather than the band: 147 MHz and up is
    // the one common allocation anywhere that shifts the OTHER way.
    RepeaterPlan {
        lo: 145_200_000.0,
        hi: 145_500_000.0,
        shift: Shift::Minus,
        offset_hz: 600_000,
        regions: mask::R2,
    },
    RepeaterPlan {
        lo: 146_610_000.0,
        hi: 147_000_000.0,
        shift: Shift::Minus,
        offset_hz: 600_000,
        regions: mask::R2,
    },
    RepeaterPlan {
        lo: 147_000_000.0,
        hi: 147_400_000.0,
        shift: Shift::Plus,
        offset_hz: 600_000,
        regions: mask::R2,
    },
    // 2 m, Region 3 (the WIA plan, which New Zealand follows; Japan's is its
    // own and is not covered here). Two output sub-bands shifting opposite
    // ways, like Region 2's: outputs up to 147.000 take their inputs 600 kHz
    // down, the ones above it 600 kHz up (issue #233). The old single entry
    // spanned 146–148 MHz on a minus shift, which put the transmitter on a
    // 147 MHz repeater's *output* — and covered 146.500, the VK simplex
    // calling channel, which no shift belongs on at all.
    RepeaterPlan {
        lo: 146_600_000.0,
        hi: 147_025_000.0,
        shift: Shift::Minus,
        offset_hz: 600_000,
        regions: mask::R3,
    },
    RepeaterPlan {
        lo: 147_025_000.0,
        hi: 147_400_000.0,
        shift: Shift::Plus,
        offset_hz: 600_000,
        regions: mask::R3,
    },
    // 1.25 m — Region 2's alone.
    RepeaterPlan {
        lo: 223_850_000.0,
        hi: 224_980_000.0,
        shift: Shift::Minus,
        offset_hz: 1_600_000,
        regions: mask::R2,
    },
    // 70 cm, Region 1. The big one: outputs at 438 MHz, inputs 7.6 MHz down at
    // 431 MHz.
    RepeaterPlan {
        lo: 438_400_000.0,
        hi: 439_600_000.0,
        shift: Shift::Minus,
        offset_hz: 7_600_000,
        regions: mask::R1,
    },
    // 70 cm, Region 2 — again two output sub-bands shifting opposite ways.
    RepeaterPlan {
        lo: 442_000_000.0,
        hi: 445_000_000.0,
        shift: Shift::Plus,
        offset_hz: 5_000_000,
        regions: mask::R2,
    },
    RepeaterPlan {
        lo: 447_000_000.0,
        hi: 450_000_000.0,
        shift: Shift::Minus,
        offset_hz: 5_000_000,
        regions: mask::R2,
    },
    // 70 cm, Region 3.
    RepeaterPlan {
        lo: 438_025_000.0,
        hi: 439_000_000.0,
        shift: Shift::Minus,
        offset_hz: 5_000_000,
        regions: mask::R3,
    },
    // 33 cm — Region 2's alone.
    RepeaterPlan {
        lo: 927_000_000.0,
        hi: 928_000_000.0,
        shift: Shift::Minus,
        offset_hz: 25_000_000,
        regions: mask::R2,
    },
    // 23 cm.
    RepeaterPlan {
        lo: 1_297_000_000.0,
        hi: 1_297_400_000.0,
        shift: Shift::Minus,
        offset_hz: 6_000_000,
        regions: mask::R1,
    },
    RepeaterPlan {
        lo: 1_282_000_000.0,
        hi: 1_288_000_000.0,
        shift: Shift::Minus,
        offset_hz: 12_000_000,
        regions: mask::R2,
    },
];

/// The conventional shift for a repeater whose output is `hz`, in `region`, or
/// `None` where the plan has nothing to say — see [`REPEATER_PLAN`].
pub fn standard_shift_in(hz: f64, region: Region) -> Option<(Shift, u32)> {
    REPEATER_PLAN
        .iter()
        .find(|p| region.in_mask(p.regions) && hz >= p.lo && hz < p.hi)
        .map(|p| (p.shift, p.offset_hz))
}

/// [`standard_shift_in`] for the station's configured region.
pub fn standard_shift(hz: f64) -> Option<(Shift, u32)> {
    standard_shift_in(hz, crate::region())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_shift_carries_its_sign_and_nothing_else() {
        let r = RepeaterState { shift: Shift::Minus, offset_hz: 600_000, ..Default::default() };
        assert_eq!(r.shift_hz(), -600_000.0);
        assert_eq!(RepeaterState { shift: Shift::Plus, ..r }.shift_hz(), 600_000.0);
        // Simplex ignores the magnitude rather than zeroing it, so a repeater
        // switched off and back on comes back on the same offset.
        assert_eq!(RepeaterState { shift: Shift::Simplex, ..r }.shift_hz(), 0.0);
        assert_eq!(RepeaterState { shift: Shift::Simplex, ..r }.offset_hz, 600_000);
    }

    /// The plan answers on repeater outputs and stays quiet everywhere else —
    /// which is what keeps AUTO off the simplex calling channels.
    #[test]
    fn the_plan_only_speaks_where_the_repeaters_are() {
        // R1 2 m: the repeater sub-band shifts, the calling channel does not.
        assert_eq!(standard_shift_in(145_712_500.0, Region::R1), Some((Shift::Minus, 600_000)));
        assert_eq!(standard_shift_in(145_500_000.0, Region::R1), None);
        // …and 145.500 in the Americas is a repeater output, on the same 600 kHz.
        assert_eq!(standard_shift_in(145_400_000.0, Region::R2), Some((Shift::Minus, 600_000)));
        // The one common allocation that shifts upwards.
        assert_eq!(standard_shift_in(147_210_000.0, Region::R2), Some((Shift::Plus, 600_000)));
        assert_eq!(standard_shift_in(146_940_000.0, Region::R2), Some((Shift::Minus, 600_000)));
        // 10 m is the same everywhere.
        for region in Region::ALL {
            assert_eq!(
                standard_shift_in(29_680_000.0, region),
                Some((Shift::Minus, 100_000)),
                "{region:?}",
            );
        }
        // Region 3's 2 m plan turns over at 147 MHz the way Region 2's does
        // (issue #233), and the simplex calling channel below both is left
        // alone.
        assert_eq!(standard_shift_in(147_000_000.0, Region::R3), Some((Shift::Minus, 600_000)));
        assert_eq!(standard_shift_in(146_875_000.0, Region::R3), Some((Shift::Minus, 600_000)));
        assert_eq!(standard_shift_in(147_275_000.0, Region::R3), Some((Shift::Plus, 600_000)));
        assert_eq!(standard_shift_in(146_500_000.0, Region::R3), None, "VK calling channel");
        // Nothing on HF, and nothing on the 2 m SSB end.
        assert_eq!(standard_shift_in(14_070_000.0, Region::R1), None);
        assert_eq!(standard_shift_in(144_300_000.0, Region::R1), None);
    }

    /// No two entries of one region may claim the same frequency, or which
    /// shift AUTO picked would depend on the order of the table.
    #[test]
    fn the_plan_has_no_overlapping_entries() {
        for region in Region::ALL {
            let mut ranges: Vec<(f64, f64)> = REPEATER_PLAN
                .iter()
                .filter(|p| region.in_mask(p.regions))
                .map(|p| (p.lo, p.hi))
                .collect();
            ranges.sort_by(|a, b| a.0.total_cmp(&b.0));
            for w in ranges.windows(2) {
                assert!(w[0].1 <= w[1].0, "{region:?}: {:?} overlaps {:?}", w[0], w[1]);
            }
            for (lo, hi) in ranges {
                assert!(lo < hi, "{region:?}: {lo} is not below {hi}");
            }
        }
    }

    #[test]
    fn dcs_codes_are_three_octal_digits() {
        for code in DCS_CODES {
            assert!(dcs_bits(code).is_some(), "{code:03} is not octal");
        }
        assert_eq!(dcs_bits(23), Some(0o23));
        assert_eq!(dcs_bits(754), Some(0o754));
        // A hand-edited file's nonsense, refused rather than encoded.
        assert_eq!(dcs_bits(789), None);
        assert_eq!(dcs_bits(1234), None);
    }

    /// A hand-edited file cannot hand the modulator a tone that is not in the
    /// table, or the transmitter an offset half a band wide.
    #[test]
    fn clamping_refuses_what_the_controls_could_not_produce() {
        let wild = RepeaterState {
            offset_hz: u32::MAX,
            ctcss_tenths: 1234,
            dcs_code: 999,
            burst_ms: 60_000,
            ..Default::default()
        };
        let ok = wild.clamped();
        assert_eq!(ok.offset_hz, MAX_OFFSET_HZ);
        assert_eq!(ok.ctcss_tenths, RepeaterState::default().ctcss_tenths);
        assert_eq!(ok.dcs_code, RepeaterState::default().dcs_code);
        assert_eq!(ok.burst_ms, *BURST_MS_RANGE.end());
        // …and a setting the controls could produce comes back untouched.
        let good = RepeaterState {
            shift: Shift::Minus,
            offset_hz: 7_600_000,
            tone: ToneMode::Ctcss,
            ctcss_tenths: 1273,
            dcs_code: 754,
            burst_ms: 500,
            ..Default::default()
        };
        assert_eq!(good.clamped(), good);
    }

    #[test]
    fn labels_read_the_way_radios_write_them() {
        let r = RepeaterState { shift: Shift::Minus, offset_hz: 600_000, ..Default::default() };
        assert_eq!(r.shift_label(), "−600 kHz");
        assert_eq!(RepeaterState { offset_hz: 7_600_000, ..r }.shift_label(), "−7.6 MHz");
        assert_eq!(
            RepeaterState { shift: Shift::Plus, offset_hz: 5_000_000, ..r }.shift_label(),
            "+5 MHz"
        );
        assert_eq!(RepeaterState::default().shift_label(), "SIMPLEX");
        assert_eq!(TxSubTone::Ctcss(885).label(), "88.5");
        assert_eq!(TxSubTone::Dcs { code: 23, invert: false }.label(), "D023N");
        assert_eq!(TxSubTone::Dcs { code: 754, invert: true }.label(), "D754I");
    }
}
