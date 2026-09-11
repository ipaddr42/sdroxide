//! The one frequency control a rig with no DDC has, and the four things that
//! have to take turns on it.
//!
//! A CAT-controlled transceiver and a networked Icom are the same shape here:
//! the radio tunes, sdroxide does not, so VFO, RIT, XIT and split cannot be
//! software offsets the way they are on an SDR. They are all the same knob, and
//! the rules for sharing it are an interlock rather than arithmetic — which is
//! why this lives on its own, where it can be tested without a serial port or a
//! socket.

use std::time::{Duration, Instant};

/// How long a frequency this end has just commanded outranks the one the radio
/// reports.
///
/// Every CAT link here is polled, so a read of the dial that went out a moment
/// before the operator picked a new frequency is answered *after* the write
/// that moved it — carrying the frequency the radio was on a moment ago. Folded
/// in, that snaps the dial straight back off the band the operator has just
/// chosen, and the only way through is to click again and hope the timing falls
/// differently: the reported symptom of issue #285, where changing band for FT8
/// took three to five attempts and landed on frequencies nobody asked for
/// (each failed attempt also writing the wrong entry into the band stack).
///
/// Long enough to cover several poll periods on a LAN or WiFi link, and no
/// longer: a radio that will not go where it was told has to be allowed to win,
/// or the two stay out of step for the rest of the session. In practice the
/// radio confirms the new frequency one poll later, and from then on only the
/// dial it was moved off is held back — the timeout is what covers a radio that
/// never confirms at all.
pub(crate) const FREQ_SETTLE: Duration = Duration::from_millis(1000);

/// How long to wait for the radio to agree before asking it again.
///
/// A tune is a command with no acknowledgement above it. On a serial port that
/// hardly matters — the bytes arrive or the cable is out — but a networked Icom
/// is three UDP conversations, and one datagram lost on the way to the radio is
/// a band change that simply never happened. Nothing notices: the frequency
/// read that follows says the radio is where it always was, [`FREQ_SETTLE`]
/// runs out, and the dial snaps back to it. That is the second half of issue
/// #297, and the operator's own workaround for it was to click the band again —
/// three to five times — which is exactly what this does for them.
///
/// One poll period plus a round trip, so a radio that is merely being slow is
/// never asked twice, and at most three resends fit inside [`FREQ_SETTLE`].
const RETRY_AFTER: Duration = Duration::from_millis(300);

/// A tune this end has asked for, until the radio agrees or [`FREQ_SETTLE`]
/// runs out. See [`Dial::report`] and [`Dial::retry`].
#[derive(Debug, Clone, Copy, PartialEq)]
struct Commanded {
    /// The dial that was asked for.
    dial: f64,
    /// The dial it was asked to leave. Every reply already in flight when the
    /// tune went out is carrying this one.
    left: f64,
    /// When it was first asked for — the clock [`FREQ_SETTLE`] runs on.
    at: Instant,
    /// When it last went out, which is what paces [`RETRY_AFTER`].
    sent: Instant,
    /// Whether the radio has since reported this dial back.
    confirmed: bool,
}

/// Where the rig's dial has to be, and who currently owns it.
///
/// A CAT rig has exactly one frequency control and no DDC behind it, so the VFO,
/// RIT, XIT and split all have to take turns on the dial: it sits on the receive
/// frequency (VFO + RIT) while receiving, and an over that transmits somewhere
/// else — XIT, or split onto the other VFO — borrows it until unkey. That makes
/// the dial an interlock rather than a number, so it lives here where it can be
/// tested without a serial port. Each method returns the frequency to command,
/// or `None` when the dial should stay where it is.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub(crate) struct Dial {
    /// The operator's VFO — the dial with RIT taken back out.
    pub(crate) vfo: f64,
    /// How much RIT the dial carries while receiving (0 when RIT is off).
    rit: f64,
    /// Where an over has parked the dial, or `None` while receiving.
    tx: Option<f64>,
    /// The frequency the last over parked the dial on, until the rig has
    /// reported something else. See [`Self::report`].
    stale_tx: Option<f64>,
    /// The tune this end last asked for, until the rig has confirmed it and
    /// stopped repeating the dial it was moved off, or [`FREQ_SETTLE`] has run
    /// out. See [`Self::report`] and [`Self::retry`].
    commanded: Option<Commanded>,
}

impl Dial {
    /// A dial parked on `vfo`, with no RIT and no over in progress.
    pub(crate) fn at(vfo: f64) -> Dial {
        Dial { vfo, ..Dial::default() }
    }

    /// Where the dial belongs while receiving.
    pub(crate) fn rx_hz(&self) -> f64 {
        self.vfo + self.rit
    }

    /// Move the VFO. Nothing is commanded while an over owns the dial —
    /// retuning then would drag the transmitter off the frequency it was
    /// cleared to use; [`Self::end_tx`] picks the new VFO up on unkey.
    pub(crate) fn set_vfo(&mut self, hz: f64) -> Option<f64> {
        let leaving = self.rx_hz();
        self.vfo = hz;
        self.commanding(leaving)
    }

    /// Change the RIT offset (0 = RIT off). Same deferral as [`Self::set_vfo`].
    pub(crate) fn set_rit(&mut self, hz: f64) -> Option<f64> {
        if self.rit == hz {
            return None;
        }
        let leaving = self.rx_hz();
        self.rit = hz;
        self.commanding(leaving)
    }

    /// The receive frequency to command now, remembered as ours until the radio
    /// confirms it — or `None` while an over owns the dial.
    ///
    /// The remembering is what [`Self::report`] needs: a write and a read cross
    /// on every polled link, and without a record of what was just asked for
    /// there is no way to tell the radio's answer to the old question from its
    /// answer to the new one.
    fn commanding(&mut self, leaving: f64) -> Option<f64> {
        if self.tx.is_some() {
            return None;
        }
        let dial = self.rx_hz();
        let now = Instant::now();
        self.commanded =
            Some(Commanded { dial, left: leaving, at: now, sent: now, confirmed: false });
        Some(dial)
    }

    /// The tune to send again, because the radio has not said it took the last
    /// one. `None` when there is nothing outstanding, when the radio has
    /// already agreed, or when it is not yet time to ask again.
    ///
    /// Driven from the source's own poll rather than from a reply, because the
    /// case it exists for is the one where no reply is coming: a set-frequency
    /// frame lost on the way to a networked radio produces no error, no
    /// refusal and no missing answer — just a radio that is still where it was.
    /// See [`RETRY_AFTER`].
    pub(crate) fn retry(&mut self) -> Option<f64> {
        if self.tx.is_some() {
            return None;
        }
        let c = self.commanded.as_mut()?;
        // Past the settling time this end has stopped insisting, and
        // [`Dial::report`] is about to let the radio have the last word.
        if c.confirmed || c.sent.elapsed() < RETRY_AFTER || c.at.elapsed() >= FREQ_SETTLE {
            return None;
        }
        c.sent = Instant::now();
        Some(c.dial)
    }

    /// Take the dial for an over on `tx_hz`. `None` when transmit already lands
    /// where we listen (no split, no XIT), so the common case costs no retune.
    pub(crate) fn begin_tx(&mut self, tx_hz: f64) -> Option<f64> {
        if (tx_hz - self.rx_hz()).abs() < 1.0 {
            return None;
        }
        self.tx = Some(tx_hz);
        self.stale_tx = None;
        Some(tx_hz)
    }

    /// Give the dial back, including any retune deferred during the over.
    /// `None` when the over never moved it.
    pub(crate) fn end_tx(&mut self) -> Option<f64> {
        self.tx.take().map(|tx| {
            self.stale_tx = Some(tx);
            self.rx_hz()
        })
    }

    /// Fold a dial frequency the *rig* reported back into the VFO, returning the
    /// operator's new VFO. `None` while an over owns the dial: what the rig
    /// reports then is our own transmit frequency, which says nothing about
    /// where the operator wants to listen.
    ///
    /// The over is not over the moment the key comes up. A frequency read
    /// issued during the transmission is answered a round trip later, and on a
    /// polled link (CI-V over a serial port or a LAN session) that answer
    /// routinely arrives *after* [`Self::end_tx`] has already handed the dial
    /// back. Folded in, it would move the operator's VFO onto the transmit
    /// frequency — a repeater over would leave the receiver sitting on the
    /// machine's input, one shift away from the channel that was being worked
    /// (issue #233). So the frequency the over parked on is remembered and
    /// ignored until the rig reports something else, which the restore write
    /// makes it do on the very next poll.
    ///
    /// A frequency this end has just *commanded* is guarded the same way and
    /// for the same reason. The read that answers here may have gone out before
    /// the write did, in which case it carries where the radio used to be, and
    /// adopting it undoes the tune the operator just asked for — issue #285,
    /// where picking an FT8 band took several attempts because most of them
    /// were being cancelled by the radio's answer to the previous question. So
    /// a report that disagrees with the outstanding command is dropped until
    /// the radio confirms the command or [`FREQ_SETTLE`] runs out, whichever
    /// comes first. The timeout is the important half of that: a radio that
    /// will not go where it was told — a band it cannot reach, a lock switch —
    /// gets the last word rather than being argued with forever.
    ///
    /// **The radio confirming is not the end of the stale answers.** That was
    /// issue #297: the same band change, on the same networked Icom, still
    /// taking three to five clicks with the guard above in place. A LAN Icom's
    /// CI-V link is a UDP conversation with a sequence number and a resend
    /// request on top, and a datagram recovered that way is handed up *when it
    /// arrives* rather than in the order it was sent — as is one the network
    /// merely delivered late. So the reply that crossed the tune can land a few
    /// hundred milliseconds behind the reply that confirmed it, by which time
    /// the guard had lifted and the old frequency was adopted as news. There is
    /// only one frequency an answer to a question asked before the tune can be
    /// carrying, and that is the dial the tune moved off, so that one alone
    /// stays guarded for the rest of the settling time. Anything else the radio
    /// reports once it has confirmed is the operator's hand on its knob and is
    /// followed at once.
    pub(crate) fn report(&mut self, dial_hz: f64) -> Option<f64> {
        if self.tx.is_some() {
            return None;
        }
        if let Some(stale) = self.stale_tx {
            if (dial_hz - stale).abs() < 1.0 {
                return None;
            }
            // Anything else is the rig answering for where it is now — the
            // restore write's own echo, or the operator's hand on the knob.
            self.stale_tx = None;
        }
        if let Some(mut c) = self.commanded {
            if (dial_hz - c.dial).abs() < 1.0 {
                // The radio confirming where it was sent: stop asking again,
                // but keep holding back the dial it was moved off until the
                // settling time is up.
                c.confirmed = true;
                self.commanded = Some(c);
            } else if c.at.elapsed() >= FREQ_SETTLE {
                // Long enough. The radio is somewhere else and means it.
                self.commanded = None;
            } else if !c.confirmed || (dial_hz - c.left).abs() < 1.0 {
                return None;
            } else {
                // Confirmed, and now reporting a third frequency: the operator
                // has their hand on the radio's own knob, and they outrank a
                // tune that has already been carried out.
                self.commanded = None;
            }
        }
        self.vfo = dial_hz - self.rit;
        Some(self.vfo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(vfo: f64) -> Dial {
        Dial { vfo, ..Dial::default() }
    }

    /// Push the outstanding tune `by` further into the past, so a test can
    /// reach the settling deadline without sleeping for it.
    fn age(d: &mut Dial, by: Duration) {
        d.commanded = d.commanded.map(|c| Commanded { at: c.at - by, sent: c.sent - by, ..c });
    }

    #[test]
    fn rit_rides_on_the_dial_and_comes_back_out_of_what_the_rig_reports() {
        let mut d = at(14_074_000.0);
        // RIT on: the dial moves, the VFO does not.
        assert_eq!(d.set_rit(700.0), Some(14_074_700.0));
        assert_eq!(d.vfo, 14_074_000.0);
        // The rig echoes that same dial back on its next poll. Folding the
        // offset out has to land on the VFO we started from — otherwise every
        // poll would walk the VFO up by the RIT offset.
        assert_eq!(d.report(14_074_700.0), Some(14_074_000.0));
        // The operator turns the rig's own dial 1 kHz up: their VFO moved with
        // it, and RIT still sits on top.
        assert_eq!(d.report(14_075_700.0), Some(14_075_000.0));
        assert_eq!(d.rx_hz(), 14_075_700.0);
        // Clearing RIT puts the dial back on the VFO.
        assert_eq!(d.set_rit(0.0), Some(14_075_000.0));
        // Re-asserting the same offset is not a retune.
        assert_eq!(d.set_rit(0.0), None);
    }

    #[test]
    fn transmitting_where_we_listen_never_touches_the_dial() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.begin_tx(14_074_000.0), None, "no split, no XIT: nothing to do");
        assert_eq!(d.end_tx(), None);
        // Sub-hertz differences are rounding, not an offset worth a CAT write.
        assert_eq!(d.begin_tx(14_074_000.4), None);
    }

    #[test]
    fn split_borrows_the_dial_for_the_over_and_gives_it_back() {
        let mut d = at(14_074_000.0);
        d.set_rit(700.0);
        // Split onto the other VFO: transmit takes the dial…
        assert_eq!(d.begin_tx(14_200_000.0), Some(14_200_000.0));
        // …and holds it. A report while it does is our own transmit frequency,
        // so it must not be mistaken for the operator moving the dial.
        assert_eq!(d.report(14_200_000.0), None);
        assert_eq!(d.vfo, 14_074_000.0, "the VFO is untouched by the over");
        // Unkey: back to the receive frequency, RIT included.
        assert_eq!(d.end_tx(), Some(14_074_700.0));
        assert_eq!(d.end_tx(), None, "the dial is only given back once");
    }

    /// Issue #233: a repeater over on an Icom left the receiver on the
    /// machine's input. The read that went out mid-over is answered after the
    /// key comes up, and folding that answer in moves the VFO onto the
    /// transmit frequency — which is where the *next* over then applies the
    /// shift again.
    #[test]
    fn the_answer_to_a_mid_over_read_does_not_move_the_vfo() {
        let mut d = at(147_000_000.0);
        // A 2 m repeater, DUP− 600: the over borrows the dial.
        assert_eq!(d.begin_tx(146_400_000.0), Some(146_400_000.0));
        assert_eq!(d.end_tx(), Some(147_000_000.0));
        // The poll issued during the over lands now, carrying the transmit
        // frequency. It says nothing about where the operator is listening.
        assert_eq!(d.report(146_400_000.0), None);
        assert_eq!(d.vfo, 147_000_000.0);
        // The restore write's own echo clears the guard…
        assert_eq!(d.report(147_000_000.0), Some(147_000_000.0));
        // …so the operator really tuning to the input afterwards is adopted.
        assert_eq!(d.report(146_400_000.0), Some(146_400_000.0));
    }

    /// Issue #285: changing band for FT8 on an Icom over its LAN port landed
    /// on frequencies nobody had picked, and took three to five attempts to
    /// stick. Every one of these links is polled, so the read that was already
    /// in flight when the operator clicked is answered *after* the write — and
    /// the old frequency, folded in, cancels the new one.
    #[test]
    fn the_answer_to_a_read_that_crossed_our_own_tune_does_not_undo_it() {
        let mut d = at(14_074_000.0);
        // The operator picks 40 m FT8. That goes to the radio…
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        // …and the very next thing the radio says is where it was when the read
        // went out, one poll period ago. It is an answer to the old question.
        assert_eq!(d.report(14_074_000.0), None);
        assert_eq!(d.vfo, 7_074_000.0, "the band the operator chose is still the band");
        // Then the radio catches up and confirms.
        assert_eq!(d.report(7_074_000.0), Some(7_074_000.0));
        // With nothing outstanding, the operator's own hand on the knob is
        // followed again immediately.
        assert_eq!(d.report(7_075_000.0), Some(7_075_000.0));
    }

    /// The guard is not a way to overrule the radio. A rig that will not go
    /// where it was sent — a transmit-only band, a lock switch, a model that
    /// rounds — has the last word once the settling time is up, because the
    /// alternative is a readout that disagrees with the radio for the rest of
    /// the session.
    #[test]
    fn a_radio_that_refuses_the_tune_wins_in_the_end() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        assert_eq!(d.report(14_074_000.0), None);
        // Age the outstanding command past the settling time.
        age(&mut d, FREQ_SETTLE * 2);
        assert_eq!(d.report(14_074_000.0), Some(14_074_000.0));
        assert_eq!(d.vfo, 14_074_000.0);
    }

    /// Issue #297: the same FT8 band change, on the same networked Icom, still
    /// took three to five clicks with the [`FREQ_SETTLE`] guard in place.
    ///
    /// A LAN Icom's CI-V link is UDP with a sequence number and a resend
    /// request over it, and a datagram recovered that way arrives *after* the
    /// ones that overtook it while it was missing. So the answer to the read
    /// that crossed the tune can land behind the reply that confirmed the tune
    /// — and by then the guard had lifted and nothing was left to tell that old
    /// frequency from the operator reaching for the radio's knob.
    #[test]
    fn a_stale_answer_that_arrives_after_the_confirmation_is_still_stale() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        // The radio confirms first: this reply left the radio after the tune.
        assert_eq!(d.report(7_074_000.0), Some(7_074_000.0));
        // And *now* the one that crossed the tune turns up, recovered by a
        // resend a few hundred milliseconds late.
        assert_eq!(d.report(14_074_000.0), None, "the band the operator chose has to stand");
        assert_eq!(d.vfo, 7_074_000.0);
        // Which is not a licence to ignore the radio: anything else it says
        // once it has confirmed is the operator's hand on its own knob.
        assert_eq!(d.report(7_075_000.0), Some(7_075_000.0));
        // …and from there the old dial is just a frequency like any other.
        assert_eq!(d.report(14_074_000.0), Some(14_074_000.0));
    }

    /// The hold on the frequency that was left is not for ever either — an
    /// operator who tunes back by hand has to be followed like anybody else.
    #[test]
    fn the_dial_that_was_left_is_only_held_back_while_the_tune_settles() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        assert_eq!(d.report(7_074_000.0), Some(7_074_000.0));
        assert_eq!(d.report(14_074_000.0), None);
        age(&mut d, FREQ_SETTLE * 2);
        assert_eq!(d.report(14_074_000.0), Some(14_074_000.0), "the operator went back by hand");
    }

    /// The other half of issue #297: a set-frequency frame lost on the way to a
    /// networked radio produces no error and no missing answer, just a radio
    /// that is still where it was. Asking again is what the operator was doing
    /// by hand — three to five times.
    #[test]
    fn a_tune_the_radio_never_acknowledges_is_sent_again() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        assert_eq!(d.retry(), None, "not before the radio has had time to answer");
        // The frame was lost: the radio keeps reporting where it always was.
        assert_eq!(d.report(14_074_000.0), None);
        d.commanded = d.commanded.map(|c| Commanded { sent: c.sent - RETRY_AFTER, ..c });
        assert_eq!(d.retry(), Some(7_074_000.0), "ask again rather than give up");
        assert_eq!(d.retry(), None, "and then wait again before asking a third time");
        // A resend is not a new tune: it must not push the settling deadline
        // out, or a radio that never answers would be argued with for ever.
        age(&mut d, FREQ_SETTLE * 2);
        assert_eq!(d.retry(), None);
        assert_eq!(d.report(14_074_000.0), Some(14_074_000.0));
    }

    /// Nothing is asked twice once the radio has said it went there — the
    /// common case has to cost exactly one frame.
    #[test]
    fn a_tune_the_radio_confirms_is_never_sent_again() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        assert_eq!(d.report(7_074_000.0), Some(7_074_000.0));
        d.commanded = d.commanded.map(|c| Commanded { sent: c.sent - RETRY_AFTER, ..c });
        assert_eq!(d.retry(), None);
    }

    /// And nothing is asked at all while an over owns the dial: the transmit
    /// frequency went out with the key-down, and a resend of the receive dial
    /// would drag the transmitter off it mid-transmission.
    #[test]
    fn nothing_is_re_asked_while_an_over_owns_the_dial() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_vfo(7_074_000.0), Some(7_074_000.0));
        d.begin_tx(7_100_000.0);
        d.commanded = d.commanded.map(|c| Commanded { sent: c.sent - RETRY_AFTER, ..c });
        assert_eq!(d.retry(), None);
    }

    /// RIT rides on the same dial, so a report answering the write *before* the
    /// offset was applied is the same stale answer and gets the same treatment.
    #[test]
    fn the_guard_covers_a_rit_write_too() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.set_rit(700.0), Some(14_074_700.0));
        assert_eq!(d.report(14_074_000.0), None, "the dial before the offset went out");
        assert_eq!(d.rx_hz(), 14_074_700.0);
        assert_eq!(d.report(14_074_700.0), Some(14_074_000.0));
    }

    /// Nothing is commanded during an over, so nothing is guarded either: the
    /// [`Dial::stale_tx`] rule is what covers that window, and the two must not
    /// tread on each other.
    #[test]
    fn a_retune_deferred_by_an_over_arms_nothing() {
        let mut d = at(14_074_000.0);
        d.begin_tx(14_200_000.0);
        assert_eq!(d.set_vfo(14_080_000.0), None);
        assert_eq!(d.commanded, None);
    }

    /// The guard lasts one over, not for good: a fresh over re-arms it, and
    /// nothing survives from the previous one.
    #[test]
    fn the_stale_guard_belongs_to_the_over_that_set_it() {
        let mut d = at(145_600_000.0);
        d.begin_tx(145_000_000.0);
        d.end_tx();
        assert_eq!(d.report(145_000_000.0), None);
        // Another over, somewhere else. The first one's frequency is no longer
        // special.
        assert_eq!(d.begin_tx(144_000_000.0), Some(144_000_000.0));
        assert_eq!(d.end_tx(), Some(145_600_000.0));
        assert_eq!(d.report(145_000_000.0), Some(145_000_000.0));
    }

    #[test]
    fn a_retune_during_an_over_waits_for_unkey() {
        let mut d = at(14_074_000.0);
        assert_eq!(d.begin_tx(14_200_000.0), Some(14_200_000.0));
        // Retuning mid-over would drag the transmitter off the frequency it was
        // cleared to use, so nothing is commanded…
        assert_eq!(d.set_vfo(14_080_000.0), None);
        assert_eq!(d.set_rit(-500.0), None);
        // …but both are remembered, and unkey lands on the new receive frequency.
        assert_eq!(d.end_tx(), Some(14_079_500.0));
    }
}
