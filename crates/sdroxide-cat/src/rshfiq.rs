//! RS-HFIQ CAT (ASCII, `*`-introduced, CR-terminated) — HobbyPCB's RS-HFIQ
//! 5 W HF transceiver.
//!
//! Not a Kenwood dialect and not related to any other family here. The whole
//! command set is two dozen lines of Arduino sketch, every command begins with
//! `*` and ends with a carriage return, and only three of them have anything to
//! do with operating: `*F` sets the tuned frequency, `*F?` reads it back, and
//! `*X0` / `*X1` unkey and key. There is no mode command, no power control, no
//! meter and no filter, because the radio has none of those things.
//!
//! # It is an I/Q radio, not a transceiver with a sound card
//!
//! This is the point, and it is what makes the profile short. An RS-HFIQ is a
//! quadrature sampling detector and a quadrature modulator either side of an
//! Si5351 that runs at four times the tuned frequency; what comes out of the
//! sound card is complex baseband centred on the dial, and what goes into it is
//! the same thing on the way back. So sdroxide does the demodulating, the
//! modulating, the filtering and the mode — everything that on a Kenwood or an
//! Icom is a CAT command — and the serial port is left with the two jobs the
//! radio actually has: where to put the oscillator, and whether to transmit.
//!
//! Set the sound format to **I/Q (stereo)** and the CAT PTT method to **CAT**;
//! selecting this family fills both in, along with the baud rate, because none
//! of the three is a preference (see `settings::radio`).
//!
//! # Things about the firmware that bite
//!
//! * **The port is 57600 baud and nothing else.** "The serial port is running
//!   at 57600 Baud, N, 8, 1." There is no menu to change it, so an operator
//!   cannot have set it to something else and any other rate is simply a link
//!   that will not work — which is why this family pins it the way ELAD's four
//!   rates are pinned (`with_family_serial_limits`).
//! * **`*F` covers 3–30 MHz and nothing outside it.** "Must be in the range
//!   3,000,000 and 30,000,000 or the sketch will report `Frq out of range.`"
//!   A command outside that is answered with that string and the oscillator
//!   does not move, so the dial poll puts the frequency back where the radio
//!   still is — but the log says what happened, because otherwise a dial that
//!   springs back has no explanation anywhere.
//! * **`*X2` must never be written.** It is the third value of the transmit
//!   command and the sketch's own documentation says "DO NOT USE": it keys the
//!   transmitter from an onboard CW generator rather than from the modulator.
//!   Nothing here writes it, and nothing here should.
//! * **A command is at most 15 characters** including the `*` and the CR, and
//!   there is no provision for two on a line. Everything written here is well
//!   inside that; a frequency is eleven characters at the widest.
//! * **The frequency parser reads backwards from the end.** It takes the last
//!   ASCII numeral in the string as the 1 Hz digit and ignores everything
//!   non-numeric, so `*f7,100,000` and `*F007100000` are the same command. This
//!   writes plain digits, which every version accepts.
//!
//! Written from HobbyPCB's own *Interface Commands* page (the sketch's
//! documented command set, as quoted in issue #383). **Not verified against a
//! radio**, and one thing in particular is a guess this end cannot settle: the
//! *shape of the replies*. The page says what each query reports but not how it
//! is framed, so [`RsHfiq::parse`] accepts a bare decimal number, one with the
//! command echoed in front of it, and either line ending — which covers every
//! framing the sketch could plausibly be printing.

use crate::{CatUpdate, Protocol};
use sdroxide_types::Mode;
use tracing::{info, warn};

/// Lowest frequency `*F` accepts, in Hz — "must be in the range 3,000,000 and
/// 30,000,000".
const MIN_HZ: f64 = 3_000_000.0;
/// Highest frequency `*F` accepts, in Hz.
const MAX_HZ: f64 = 30_000_000.0;

/// What the sketch answers a `*F` it cannot honour with.
const OUT_OF_RANGE: &str = "Frq out of range";

pub struct RsHfiq {
    /// Bytes arrived and not yet split into whole lines.
    buf: String,
    /// Whether the out-of-range refusal has been logged for the frequency
    /// currently being asked for, so a poll running against a dial the radio
    /// will not accept says so once rather than twice a second.
    warned_out_of_range: bool,
    /// The firmware string from `*W`, once it has answered. Held so the line is
    /// logged on the change rather than on every reply.
    firmware: Option<String>,
}

impl RsHfiq {
    pub fn new() -> Self {
        RsHfiq { buf: String::new(), warned_out_of_range: false, firmware: None }
    }
}

impl Protocol for RsHfiq {
    fn set_freq(&mut self, hz: f64) -> Vec<u8> {
        // Sent even when it is outside what the sketch takes. Clamping would
        // put the oscillator somewhere nobody asked for and report success;
        // this leaves the radio where it is, and the dial poll below brings the
        // readout back to the truth within a poll interval.
        if !(MIN_HZ..=MAX_HZ).contains(&hz) {
            if !self.warned_out_of_range {
                self.warned_out_of_range = true;
                warn!(
                    mhz = hz / 1e6,
                    "RS-HFIQ CAT: {:.6} MHz is outside the 3–30 MHz its frequency command \
                     accepts; the radio will refuse it and stay where it is",
                    hz / 1e6
                );
            }
        } else {
            self.warned_out_of_range = false;
        }
        format!("*F{}\r", hz.round().max(0.0) as u64).into_bytes()
    }

    /// Nothing to say. Every mode this radio can be worked in is one sdroxide
    /// builds out of the I/Q itself, and the sketch has no mode command at all.
    fn set_mode(&mut self, _m: Mode) -> Vec<u8> {
        Vec::new()
    }

    /// `*X1` transmits through the quadrature modulator, `*X0` receives. The
    /// third value, `*X2`, keys from the radio's own CW generator and the
    /// firmware's documentation says not to use it; it is deliberately
    /// unreachable from here.
    fn ptt(&self, on: bool) -> Vec<u8> {
        if on { b"*X1\r".to_vec() } else { b"*X0\r".to_vec() }
    }

    /// The dial, and only the dial: there is no mode to ask about.
    fn poll_requests(&self) -> Vec<Vec<u8>> {
        vec![b"*F?\r".to_vec()]
    }

    /// What firmware is on the other end, asked once so the log carries it. A
    /// radio that answers this at all is one whose port is open, talking and at
    /// the right baud rate, which is worth a line of its own when the rest of
    /// the profile is two commands.
    fn open_requests(&self) -> Vec<Vec<u8>> {
        vec![b"*W\r".to_vec()]
    }

    fn parse(&mut self, buf: &mut Vec<u8>) -> Vec<CatUpdate> {
        self.buf.push_str(&String::from_utf8_lossy(buf));
        buf.clear();
        let mut out = Vec::new();
        // Either line ending, because the page does not say which the sketch
        // prints — see the module comment.
        while let Some(idx) = self.buf.find(['\r', '\n']) {
            let line: String = self.buf.drain(..=idx).collect();
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            if line.starts_with(OUT_OF_RANGE) {
                warn!("RS-HFIQ CAT: the radio refused the frequency — \"{line}\"");
                continue;
            }
            if let Some(rest) = line.strip_prefix("RS-HFIQ") {
                let fw = rest.trim().to_string();
                if self.firmware.as_deref() != Some(fw.as_str()) {
                    info!("RS-HFIQ CAT: firmware {fw}");
                    self.firmware = Some(fw);
                }
                continue;
            }
            // A reply that is a frequency, with or without the command echoed
            // in front of it. Only inside the range the radio can actually
            // tune: the temperature reply is a bare number too, and reading
            // "31" as 31 Hz would drag the dial to the bottom of the band.
            let digits: String = line
                .trim_start_matches(['*', 'F', 'f', '?', ':', '=', ' '])
                .chars()
                .take_while(char::is_ascii_digit)
                .collect();
            if let Ok(hz) = digits.parse::<u64>() {
                let hz = hz as f64;
                if (MIN_HZ..=MAX_HZ).contains(&hz) {
                    out.push(CatUpdate::Freq(hz));
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn feed(p: &mut RsHfiq, s: &str) -> Vec<CatUpdate> {
        let mut b = s.as_bytes().to_vec();
        p.parse(&mut b)
    }

    #[test]
    fn the_frequency_command_is_the_documented_shape() {
        let mut p = RsHfiq::new();
        assert_eq!(p.set_freq(14_195_000.0), b"*F14195000\r".to_vec());
        // The one command a dial move is allowed to be. 15 characters is the
        // sketch's hard limit, `*` and CR included.
        assert!(p.set_freq(29_999_999.0).len() <= 15);
    }

    /// The transmit command, and the value that must never be written: `*X2`
    /// keys the radio's own CW generator, which its own documentation marks
    /// "DO NOT USE".
    #[test]
    fn transmit_keys_the_modulator_and_never_the_cw_generator() {
        let p = RsHfiq::new();
        assert_eq!(p.ptt(true), b"*X1\r".to_vec());
        assert_eq!(p.ptt(false), b"*X0\r".to_vec());
        for on in [true, false] {
            assert!(!p.ptt(on).windows(3).any(|w| w == b"*X2"), "the CW generator was keyed");
        }
    }

    /// The reply framing is the part of this profile that is a guess, so the
    /// parser takes every shape the sketch could plausibly be printing rather
    /// than one of them.
    #[test]
    fn a_frequency_reply_is_read_whatever_it_is_wrapped_in() {
        for line in ["14195000\r\n", "*F14195000\r", "F:14195000\n", " 14195000 \r\n"] {
            let mut p = RsHfiq::new();
            assert_eq!(
                feed(&mut p, line),
                vec![CatUpdate::Freq(14_195_000.0)],
                "{line:?} was not read as a frequency"
            );
        }
    }

    /// `*T` answers with a bare number too, and the sketch's other queries can
    /// answer with numbers outside the tuning range. None of them is a dial.
    #[test]
    fn a_number_that_cannot_be_a_dial_is_not_taken_for_one() {
        let mut p = RsHfiq::new();
        // A temperature.
        assert!(feed(&mut p, "31\r\n").is_empty());
        // The BIT generator's range reaches below 3 MHz and above 30.
        assert!(feed(&mut p, "1024000\r\n").is_empty());
        assert!(feed(&mut p, "112500000\r\n").is_empty());
        // The refusal, and the firmware line, are not frequencies either.
        assert!(feed(&mut p, "Frq out of range.\r\n").is_empty());
        assert!(feed(&mut p, "RS-HFIQ FW1.0\r\n").is_empty());
        assert_eq!(p.firmware.as_deref(), Some("FW1.0"));
    }

    #[test]
    fn a_reply_split_across_two_reads_is_still_one_reply() {
        let mut p = RsHfiq::new();
        assert!(feed(&mut p, "141").is_empty());
        assert_eq!(feed(&mut p, "95000\r\n"), vec![CatUpdate::Freq(14_195_000.0)]);
    }

    /// Nothing this profile can be asked for reaches a control the radio has
    /// not got: no mode, no power, no filter, no squelch, no keyer.
    #[test]
    fn it_claims_only_what_the_radio_has() {
        let mut p = RsHfiq::new();
        assert!(p.set_mode(Mode::Usb).is_empty());
        assert!(p.set_power(1.0).is_empty());
        assert!(p.set_filter(Mode::Usb, 300.0, 2700.0).is_empty());
        assert!(p.set_squelch(0.5).is_empty());
        assert_eq!(p.cw_chunk_len(), 0);
        assert!(!p.commands_power());
        assert!(!p.commands_filter());
        assert!(!p.commands_squelch());
    }
}
