//! SSTV (slow-scan TV) sub-mode vocabulary shared by the engine, the wire
//! protocol, and the UI. The concrete tone timing lives in the native DSP crate;
//! this module only carries the identity, dimensions, and VIS codes so the UI
//! (native + wasm) can label modes and pick TX image sizes.

use serde::{Deserialize, Serialize};

/// One SSTV transmission mode. `Mode::Sstv` is the radio mode; this picks the
/// specific line format used for encode/decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SstvMode {
    Scottie1,
    Scottie2,
    ScottieDx,
    Martin1,
    Martin2,
    Robot72,
    Robot36,
    /// Wraase SC-2 180 — 320×256 RGB, 711 ms a line. Very widely used on
    /// 3.730 MHz and 14.230 MHz, and the first of the modes added for issue
    /// #421: until then a station sending one met a receiver that showed a
    /// perfect signal on the waterfall and said "waiting for a signal".
    WraaseSc2_180,
    /// Wraase SC-2 120 — the same format at 476 ms a line.
    WraaseSc2_120,
    // The PD family (Martin Bruchanov OK2MNM's, after the "PD" in the original
    // program). Four scans between syncs, carrying *two* image lines: luma for
    // each of them either side of one shared pair of chroma scans. That is why
    // they are cheap for the bandwidth — half the chroma — and why they need a
    // line structure none of the older modes do.
    Pd50,
    Pd90,
    Pd120,
    Pd160,
    Pd180,
    Pd240,
    Pd290,
}

impl Default for SstvMode {
    fn default() -> Self {
        SstvMode::Scottie1
    }
}

impl SstvMode {
    /// All modes, in a sensible menu order.
    pub const ALL: [SstvMode; 16] = [
        SstvMode::Scottie1,
        SstvMode::Scottie2,
        SstvMode::ScottieDx,
        SstvMode::Martin1,
        SstvMode::Martin2,
        SstvMode::Robot72,
        SstvMode::Robot36,
        SstvMode::WraaseSc2_180,
        SstvMode::WraaseSc2_120,
        SstvMode::Pd50,
        SstvMode::Pd90,
        SstvMode::Pd120,
        SstvMode::Pd160,
        SstvMode::Pd180,
        SstvMode::Pd240,
        SstvMode::Pd290,
    ];

    /// Short human label for buttons/menus.
    pub fn label(self) -> &'static str {
        match self {
            SstvMode::Scottie1 => "Scottie 1",
            SstvMode::Scottie2 => "Scottie 2",
            SstvMode::ScottieDx => "Scottie DX",
            SstvMode::Martin1 => "Martin 1",
            SstvMode::Martin2 => "Martin 2",
            SstvMode::Robot72 => "Robot 72",
            SstvMode::Robot36 => "Robot 36",
            SstvMode::WraaseSc2_180 => "SC2-180",
            SstvMode::WraaseSc2_120 => "SC2-120",
            SstvMode::Pd50 => "PD50",
            SstvMode::Pd90 => "PD90",
            SstvMode::Pd120 => "PD120",
            SstvMode::Pd160 => "PD160",
            SstvMode::Pd180 => "PD180",
            SstvMode::Pd240 => "PD240",
            SstvMode::Pd290 => "PD290",
        }
    }

    /// Transmitted image size in pixels, `(width, height)`.
    pub fn dimensions(self) -> (u16, u16) {
        match self {
            // Scottie/Martin/Wraase are 320×256.
            SstvMode::Scottie1
            | SstvMode::Scottie2
            | SstvMode::ScottieDx
            | SstvMode::Martin1
            | SstvMode::Martin2
            | SstvMode::WraaseSc2_180
            | SstvMode::WraaseSc2_120 => (320, 256),
            // Robot modes are 320×240.
            SstvMode::Robot72 | SstvMode::Robot36 => (320, 240),
            // The PD family carries a different size per mode — this is the
            // one family where the picture really does get bigger with the
            // air time, which is most of the point of it.
            SstvMode::Pd50 | SstvMode::Pd90 => (320, 256),
            SstvMode::Pd160 => (512, 400),
            SstvMode::Pd120 | SstvMode::Pd180 | SstvMode::Pd240 => (640, 496),
            SstvMode::Pd290 => (800, 616),
        }
    }

    /// How many *image* lines one transmitted line carries.
    ///
    /// One for every mode but the PD family, which sends luma for two lines
    /// either side of a single pair of chroma scans and so covers two rows per
    /// sync pulse.
    pub fn rows_per_line(self) -> u16 {
        match self {
            SstvMode::Pd50
            | SstvMode::Pd90
            | SstvMode::Pd120
            | SstvMode::Pd160
            | SstvMode::Pd180
            | SstvMode::Pd240
            | SstvMode::Pd290 => 2,
            _ => 1,
        }
    }

    /// The 7-bit VIS code identifying this mode in the calibration header.
    ///
    /// The codes are the published ones — JL Barber N7CXI's 2000 mode
    /// specification, as implemented by every decoder on the air. Checked
    /// against `windytan/slowrx`'s `modespec.c` VIS map.
    pub fn vis_code(self) -> u8 {
        match self {
            SstvMode::Robot36 => 0x08,
            SstvMode::Robot72 => 0x0C,
            SstvMode::Martin2 => 0x28,
            SstvMode::Martin1 => 0x2C,
            SstvMode::WraaseSc2_180 => 0x37,
            SstvMode::Scottie2 => 0x38,
            SstvMode::Scottie1 => 0x3C,
            SstvMode::WraaseSc2_120 => 0x3F,
            SstvMode::ScottieDx => 0x4C,
            SstvMode::Pd50 => 0x5D,
            SstvMode::Pd290 => 0x5E,
            SstvMode::Pd120 => 0x5F,
            SstvMode::Pd180 => 0x60,
            SstvMode::Pd240 => 0x61,
            SstvMode::Pd160 => 0x62,
            SstvMode::Pd90 => 0x63,
        }
    }

    /// Map a decoded VIS code back to a mode, if recognised.
    pub fn from_vis(code: u8) -> Option<SstvMode> {
        SstvMode::ALL.into_iter().find(|m| m.vis_code() == code)
    }

    /// The name of a mode this build does **not** decode, for a VIS code that
    /// came through with good parity.
    ///
    /// A receiver that has read a valid header knows exactly what it is being
    /// sent and exactly why it is about to draw nothing. Saying so is the
    /// difference between "sdroxide's SSTV is broken" and "that station is
    /// using Pasokon P3" — issue #421, where a picture arrived in a mode this
    /// decoder did not have and the panel went on saying *waiting for a
    /// signal* with a textbook signal on the waterfall.
    ///
    /// Only the codes that are actually assigned; an unassigned one is a
    /// misread header rather than a mode, and claiming otherwise would turn
    /// noise into a confident wrong answer.
    pub fn unsupported_name(code: u8) -> Option<&'static str> {
        Some(match code {
            0x02 => "Robot 8 B/W",
            0x04 => "Robot 24",
            0x06 => "Robot 12 B/W",
            0x0A => "Robot 24 B/W",
            0x20 => "Martin 4",
            0x24 => "Martin 3",
            0x71 => "Pasokon P3",
            0x72 => "Pasokon P5",
            0x73 => "Pasokon P7",
            _ => return None,
        })
    }
}

/// Broadcast status for the SSTV panel: what's being sent/received and how far
/// along. Rides the wire as part of the digital-mode event stream.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SstvStatus {
    /// The mode selected for the next transmission.
    pub tx_mode: SstvMode,
    /// True while an image is being transmitted.
    pub tx_active: bool,
    /// True while a signal is being received/decoded.
    pub rx_active: bool,
    /// The mode detected from the incoming VIS header, if any.
    pub detected: Option<SstvMode>,
    /// Fraction of the current image completed, 0.0..=1.0 (RX while receiving,
    /// TX while transmitting).
    pub progress: f32,
    /// Smoothed in-band receive signal level (~0..1), for an activity meter so
    /// the operator can confirm audio is reaching the decoder.
    pub signal: f32,
    /// The last header that arrived for a mode this build cannot draw, as
    /// something to show the operator ("PD120", or "VIS $5F" for a code with
    /// no name). Cleared when a picture this decoder *can* draw starts.
    ///
    /// The whole point of it is that an unimplemented mode and a broken
    /// receiver look identical from the panel — a strong signal, a healthy
    /// level meter, and no picture (issue #421).
    #[serde(default)]
    pub unsupported: Option<String>,
    /// The callsign the last station to transmit sent in its FSK ID, if it sent
    /// one.
    ///
    /// The identification arrives in tones a fraction of a second *after* the
    /// picture, so it cannot ride on the image it belongs to; it lands here and
    /// stays until the next station sends one. Held rather than shown once
    /// because that is how it is read — the operator looks at the picture, then
    /// looks for who sent it.
    #[serde(default)]
    pub rx_id: Option<String>,
}

impl Default for SstvStatus {
    fn default() -> Self {
        SstvStatus {
            tx_mode: SstvMode::default(),
            tx_active: false,
            rx_active: false,
            detected: None,
            progress: 0.0,
            signal: 0.0,
            unsupported: None,
            rx_id: None,
        }
    }
}
