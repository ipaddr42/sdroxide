//! AIS — the Automatic Identification System: what every ship of any size
//! transmits about itself, on two 25 kHz channels either side of 162.000 MHz.
//!
//! Native-only, like the ADS-B and VDL2 decoders: it runs in the engine off the
//! raw I/Q and reaches the UI as [`sdroxide_types::AisStatus`] over the
//! ordinary event path.
//!
//! # Shape of the thing
//!
//! ```text
//!  150 kHz window ──┬──▶ Ddc ──▶ ChannelRx (161.975, "A") ──┐
//!                   └──▶ Ddc ──▶ ChannelRx (162.025, "B") ──┤
//!                                                            ▼
//!                                              vessel table ◀── track::Tracker
//!
//!  ChannelRx: Gate ─▶ Demod ─▶ slice ─▶ hdlc::Deframer ─▶ message::parse
//! ```
//!
//! [`plan`] says where the channels are and how far the stream may be
//! decimated; [`gate`] turns a running channel into discrete slots; [`demod`]
//! measures each slot's bit timing and slices it; [`hdlc`] finds the frame and
//! decides whether to believe it; [`message`] says what a believed one
//! contains; [`sixbit`] reads its text and writes the `!AIVDM` sentence;
//! [`track`] folds everything into one entry per ship. `controller` wraps the
//! lot in a worker thread. [`tx`] is a transmitter that exists only to test the
//! receiver.
//!
//! # Provenance
//!
//! Written from ITU-R M.1371-5 — the AIS technical characteristics: §2 for the
//! channels, §3.2 for the GMSK modulation, §3.3 for the packet, its NRZI coding
//! and its bit stuffing, Annex 8 for the message tables, and Tables 45, 47, 50
//! and 51 for the code lists — with IEC 61162-1 for the `!AIVDM` sentence and
//! IALA Guideline 1082 for the aid-to-navigation types. Sources are cited per
//! module. Cross-checked against the observable behaviour of `gnuais` and
//! `rtl-ais`, but no code came from either, which also keeps their GPL-2.0 out
//! of this tree.
//!
//! # What is proven and what is not
//!
//! Every layer is tested against a transmitter built independently from the
//! same standard ([`tx`]), so the framing, the timing recovery, the check
//! sequence and the field offsets all round-trip — including on a receiver four
//! kilohertz off frequency, and with two ships in adjacent slots.
//!
//! What that cannot prove is that the *standard* was read correctly: a field
//! offset wrong in the same way in both halves agrees with itself. **No part of
//! this decoder has been run against a recording of real AIS traffic.** The
//! `!AIVDM` sentence on every row in the panel is the answer to that — it is a
//! form every other AIS tool in the world accepts, so what sdroxide made of a
//! transmission can be compared against what anything else makes of the same
//! one.
//!
//! # The receiver this needs
//!
//! Two channels 50 kHz apart, each 25 kHz wide, so a window of 100 kHz reaches
//! both. Below [`sdroxide_types::AIS_MIN_RATE_HZ`] there is not enough stream
//! for even one, and the honest answer is to say so rather than to run and find
//! nothing. Between that and 100 kHz the lane runs on whichever channel the
//! window is over and reports which — half the traffic, from ships that
//! alternate channels, so every vessel is still seen at half its reporting
//! rate.
//!
//! There is one arithmetic trap peculiar to this plan and it is worth knowing
//! about before changing anything: the channels are 50 kHz apart, so an
//! obvious-looking channel rate of 48 kHz — five samples a bit — folds one
//! channel 2 kHz from the centre of the other. See [`plan::channel_decimation`].

pub mod channel;
mod controller;
pub mod demod;
pub mod gate;
pub mod hdlc;
pub mod message;
pub mod plan;
pub mod sixbit;
pub mod track;
pub mod tx;

pub use channel::{ChannelRx, Counters, Decoded};
pub use controller::{AisAction, AisController};
pub use message::{Fix, Message, Statics, Voyage};
pub use track::Tracker;

/// Whether a stream of this rate, centred here, reaches either AIS channel.
///
/// The engine's "can this receiver do it" predicate, the same shape the ADS-B
/// and VDL2 lanes' is. One channel is enough to say yes, because a vessel
/// alternates between the two and is heard on whichever is being listened to.
pub fn window_covers(center_hz: f64, rate_hz: f64) -> bool {
    !plan::channels_in_window(center_hz, rate_hz).is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::{AIS_CHANNEL_A_HZ, AIS_PLAN_CENTER_HZ};

    /// The two ways a receiver can fail to carry AIS are different problems
    /// with different fixes, and both have to be recognised.
    #[test]
    fn a_window_has_to_be_both_wide_enough_and_in_the_right_place() {
        assert!(window_covers(AIS_PLAN_CENTER_HZ, 150_000.0));
        assert!(!window_covers(AIS_PLAN_CENTER_HZ, 20_000.0), "too narrow for either channel");
        assert!(!window_covers(868_880_000.0, 150_000.0), "the right rate in the wrong place");
        // One channel is enough: a ship alternates, so it is still heard.
        assert!(window_covers(AIS_CHANNEL_A_HZ, 50_000.0));
        // A wide front end parked elsewhere in the band may still reach it.
        assert!(window_covers(160_000_000.0, 8_000_000.0));
    }
}
