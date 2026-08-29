//! The radio's state, and the *orderings* that keep it correct.
//!
//! Everything here is about sequence. The individual control transfers are
//! trivial — [`crate::protocol`] encodes them and [`crate::usb`] sends them —
//! and none of them is where this driver can go wrong. What can go wrong is the
//! order, and every one of the SoapyHackRF defects that motivated a native
//! backend is an ordering bug:
//!
//! * the receive amplifier going dead after the first over, because a direction
//!   change resets it and nothing put it back;
//! * the transmit amplifier never working at all, for the same reason;
//! * one 14 dB switch driven as if it were two independent stages.
//!
//! So the rule this module is built on: **a change of direction reprograms the
//! entire front end, unconditionally.** Not the amplifier, not the stage that
//! is known to have been reset — all of it. Three extra control transfers per
//! over is nothing against the cost of being wrong about which ones survive,
//! and it means this driver does not depend on a claim about the firmware that
//! was transcribed rather than measured.
//!
//! [`Device`] is generic over [`Transport`] so all of that is testable with
//! nothing plugged in; see the tests at the bottom of this file, which assert
//! the orderings directly.

use sdroxide_types::HackRfConfig;

use crate::error::Result;
use crate::protocol::{
    BoardKind, GainSetter, M0_STATE_LEN, M0State, PartIdSerial, Request, TransceiverMode,
    auto_filter_bw, compute_filter_bw, encode_sample_rate, encode_set_freq, parse_ascii,
    rate_params,
};
use crate::usb::{Transport, TransportExt};

/// The radio, its current settings, and the sequences that change them.
pub struct Device<T: Transport> {
    io: T,
    kind: BoardKind,
    version: String,
    part_serial: Option<PartIdSerial>,
    mode: TransceiverMode,

    center_hz: f64,
    tx_center_hz: f64,
    rate_hz: f64,
    filter_bw: u32,
    /// `0.0` means "follow the rate"; anything else is an explicit override.
    filter_pref_hz: f64,
    ppm: f64,

    lna_db: f64,
    vga_db: f64,
    txvga_db: f64,
    amp: bool,
    tx_amp: bool,
    bias_tee: bool,
}

impl<T: Transport> Device<T> {
    /// Identify the radio, make the transmitter safe, then program it for
    /// receive.
    ///
    /// The order of the first two steps is the point. Nothing is tuned, no
    /// stream is opened and no gain is applied until the transmit stages have
    /// been driven to their minimum — so however this function fails, and
    /// whatever state a previous program left the radio in, it cannot be left
    /// able to emit power.
    ///
    /// The requested rate is held unclamped until [`Self::identify`] has run,
    /// because the floor is the board's: a HackRF Pro takes rates a decade
    /// below anything the other boards will do, and clamping to the MAX5864's
    /// 2 Msps before knowing which radio this is would quietly take that away.
    pub fn open(io: T, cfg: &HackRfConfig, center_hz: f64) -> Result<Device<T>> {
        let mut dev = Device {
            io,
            kind: BoardKind::HackRfOne,
            version: String::new(),
            part_serial: None,
            // Unknown until the first `set_mode`, which is what forces the
            // opening sequence to state it rather than assume it.
            mode: TransceiverMode::Ss,
            center_hz,
            tx_center_hz: center_hz,
            rate_hz: cfg.sample_rate_hz,
            filter_bw: 0,
            filter_pref_hz: cfg.filter_bw_hz,
            ppm: cfg.ppm,
            lna_db: cfg.lna_db,
            vga_db: cfg.vga_db,
            txvga_db: cfg.txvga_db,
            amp: cfg.amp,
            tx_amp: cfg.tx_amp,
            bias_tee: cfg.bias_tee,
        };

        dev.safety_rail()?;
        dev.identify();
        dev.rate_hz = dev.snap_rate(cfg.sample_rate_hz);
        dev.apply_rate()?;
        dev.retune()?;
        dev.set_mode(TransceiverMode::Receive)?;
        dev.apply_rx_front_end()?;
        Ok(dev)
    }

    /// Drive every transmit stage to its minimum, before anything else happens.
    ///
    /// `sdroxide-radio`'s SoapySDR path does the same thing at open and for the
    /// same reason — "so keying up can never emit unexpected power" — but it
    /// matters more here, because this driver owns the mode register too. A
    /// radio left in `Transmit` by a program that crashed is still transmitting
    /// when we find it, so the mode goes to `Off` first and the gain after.
    fn safety_rail(&mut self) -> Result<()> {
        self.io.trace().direction("open: making the transmitter safe");
        self.set_mode(TransceiverMode::Off)?;
        self.io.set_gain(GainSetter::TxVga, 0.0)?;
        self.io.out(Request::AmpEnable, 0, 0)?;
        // The bias tee is asked for and not insisted on, and this is the one
        // line here that is allowed to fail (issue #220).
        //
        // `usb_vendor_request_set_antenna_enable` in the firmware
        // (`hackrf_usb/usb_api_transceiver.c`) begins by switching on
        // `detected_platform()` and **stalls** on anything that is not a
        // HackRF One OG, a HackRF One r9 or a Pro — a Jawbreaker, a rad1o, and
        // any board whose platform detection does not answer. A stall there is
        // therefore the radio saying it has no bias tee to switch, which is
        // exactly the state this line was asking it to be in.
        //
        // Insisting cost a whole radio: the stall came back as a fatal
        // `control write failed` and the open was abandoned, three requests
        // before the board had even been identified. libhackrf never sends this
        // request unless the operator asks for the bias tee, which is why the
        // same radio opens in every other program.
        //
        // `identify` cannot help: it runs *after* this, and deliberately —
        // nothing may talk to a radio that might still be transmitting until
        // the transmitter has been made safe.
        if self.io.out(Request::AntennaEnable, 0, 0).is_err() {
            self.io.trace().note(
                "the radio refused SET_ANTENNA_ENABLE — it has no bias tee, so there is \
                 nothing to switch off",
            );
        }
        Ok(())
    }

    /// Read what the radio says about itself. Best-effort: a radio that will
    /// not name itself still receives, so none of this is fatal.
    fn identify(&mut self) {
        if let Ok(b) = self.io.control_in(Request::BoardIdRead, 0, 0, 1)
            && let Some(code) = b.first()
        {
            self.kind = BoardKind::from_code(*code);
        }
        if let Ok(b) = self.io.control_in(Request::VersionStringRead, 0, 0, 255) {
            self.version = parse_ascii(&b);
            // The line above will always be marked SHORT — the firmware returns
            // a NUL-terminated string far shorter than the buffer it is asked
            // for. Saying so here keeps the marker meaning something: everywhere
            // else in this trace a short reply really is a surprise, and a
            // reader who learns to skip it would skip the one that matters.
            self.io.trace().note("(the short version-string reply above is expected)");
        }
        if let Ok(b) = self.io.control_in(Request::BoardPartidSerialnoRead, 0, 0, 24) {
            self.part_serial = PartIdSerial::parse(&b);
        }
        let serial = self.part_serial.map(|p| p.serial_hex()).unwrap_or_default();
        self.io.trace().set_identity(format!(
            "{} — firmware {:?}, serial {}, API {:04x}",
            self.kind.name(),
            self.version,
            if serial.is_empty() { "unknown".into() } else { serial },
            self.io.api_version(),
        ));
    }

    // ---- state -----------------------------------------------------------

    pub fn kind(&self) -> BoardKind {
        self.kind
    }

    pub fn version(&self) -> &str {
        &self.version
    }

    pub fn serial_hex(&self) -> Option<String> {
        self.part_serial.map(|p| p.serial_hex())
    }

    pub fn rate_hz(&self) -> f64 {
        self.rate_hz
    }

    /// The rates this particular board will accept. Published rather than
    /// assumed so the message an operator sees names their radio's limits and
    /// not a HackRF One's.
    pub fn rate_range(&self) -> (f64, f64) {
        self.kind.rate_range()
    }

    /// Whether the baseband filter is the radio's choice rather than the
    /// operator's. See [`BoardKind::sets_own_filter`].
    pub fn sets_own_filter(&self) -> bool {
        self.kind.sets_own_filter()
    }

    pub fn center_hz(&self) -> f64 {
        self.center_hz
    }

    pub fn filter_bw(&self) -> u32 {
        self.filter_bw
    }

    pub fn mode(&self) -> TransceiverMode {
        self.mode
    }

    pub fn label(&self) -> String {
        match self.part_serial.map(|p| p.serial_hex()) {
            Some(s) if !s.is_empty() => {
                format!("{} ({})", self.kind.name(), &s[s.len().saturating_sub(8)..])
            }
            _ => self.kind.name().to_string(),
        }
    }

    /// The dB each stage is really at, after the hardware's truncation.
    pub fn gains(&self) -> (f64, f64, f64) {
        (self.lna_db, self.vga_db, self.txvga_db)
    }

    // ---- primitives ------------------------------------------------------

    fn set_mode(&mut self, mode: TransceiverMode) -> Result<()> {
        self.io.out(Request::SetTransceiverMode, mode as u16, 0)?;
        self.mode = mode;
        Ok(())
    }

    /// The frequency actually asked of the radio, with the operator's crystal
    /// correction folded in.
    ///
    /// A radio whose reference is high by `ppm` tunes high by the same
    /// fraction, so landing on `want` means asking for proportionally less.
    /// Dividing rather than multiplying by `(1 + ppm/1e6)` is what makes the
    /// correction exact rather than first-order.
    fn corrected(&self, want: f64) -> u64 {
        (want / (1.0 + self.ppm / 1.0e6)).round().max(0.0) as u64
    }

    fn retune(&mut self) -> Result<()> {
        let hz = self.corrected(self.center_hz);
        self.io.control_out(Request::SetFreq, 0, 0, &encode_set_freq(hz))
    }

    /// Program the sample rate and the baseband filter that belongs with it.
    ///
    /// The filter is not optional and not independent: the zero-IF LO offset is
    /// withdrawn when it would exceed 45 % of the analog bandwidth, so a rate
    /// change that did not move the filter would silently cost the DC-spike
    /// avoidance. They go out together, filter second, because the rate is what
    /// determines the automatic choice.
    fn apply_rate(&mut self) -> Result<()> {
        let (freq_hz, divider) = rate_params(self.rate_hz);
        self.io.control_out(Request::SampleRateSet, 0, 0, &encode_sample_rate(freq_hz, divider))?;
        self.apply_filter()
    }

    fn apply_filter(&mut self) -> Result<()> {
        // A board that derives its own filter is not sent the request — the
        // same rule as the bias tee two functions down, and for the same
        // reason. The firmware would accept it and discard it, and the driver
        // would then be holding a bandwidth the radio is not running. That
        // figure is not decoration: it is what `lo_offset_for` decides the
        // zero-IF offset from.
        if self.kind.sets_own_filter() {
            let bw = self.kind.auto_filter_bw(self.rate_hz);
            if self.filter_bw != bw {
                self.io.trace().note(format!(
                    "{} sets its own baseband filter; at {:.3} Msps that is {:.3} MHz",
                    self.kind.name(),
                    self.rate_hz / 1e6,
                    bw as f64 / 1e6,
                ));
            }
            self.filter_bw = bw;
            return Ok(());
        }
        let bw = if self.filter_pref_hz > 0.0 {
            compute_filter_bw(self.filter_pref_hz as u32)
        } else {
            auto_filter_bw(self.rate_hz)
        };
        self.io.out(
            Request::BasebandFilterBandwidthSet,
            (bw & 0xffff) as u16,
            (bw >> 16) as u16,
        )?;
        self.filter_bw = bw;
        Ok(())
    }

    /// Everything the receive path needs, applied as a block.
    ///
    /// Called at open and again after every return from transmit. See the
    /// module header: the whole front end is reprogrammed on a direction
    /// change rather than the parts believed to have been reset.
    fn apply_rx_front_end(&mut self) -> Result<()> {
        self.lna_db = self.io.set_gain(GainSetter::Lna, self.lna_db)? as f64;
        self.vga_db = self.io.set_gain(GainSetter::Vga, self.vga_db)? as f64;
        self.io.out(Request::AmpEnable, self.amp as u16, 0)?;
        self.apply_bias_tee()
    }

    /// Everything the transmit path needs, applied as a block — after the mode
    /// change, never before it.
    fn apply_tx_front_end(&mut self) -> Result<()> {
        self.txvga_db = self.io.set_gain(GainSetter::TxVga, self.txvga_db)? as f64;
        self.io.out(Request::AmpEnable, self.tx_amp as u16, 0)?;
        self.apply_bias_tee()
    }

    fn apply_bias_tee(&mut self) -> Result<()> {
        // A board without the circuit is not sent the request at all. Not
        // because the firmware would ignore it — it stalls (see
        // [`Self::safety_rail`]) — but because a control the radio cannot
        // honour is worse than no control at all.
        if !self.kind.has_bias_tee() {
            return Ok(());
        }
        // And a board that says it *is* a HackRF One and then refuses anyway is
        // still not a reason to take the radio off the air (issue #220): the
        // answer is that this one has no bias tee, and the antenna port is
        // unpowered either way. Cleared in the driver's own copy so the next
        // front-end pass does not ask again, and noted once in the trace.
        if self.bias_tee && self.io.out(Request::AntennaEnable, 1, 0).is_err() {
            self.io
                .trace()
                .note("the radio refused the bias tee; it has no antenna-port power to switch");
            self.bias_tee = false;
            return Ok(());
        }
        if !self.bias_tee {
            let _ = self.io.out(Request::AntennaEnable, 0, 0);
        }
        Ok(())
    }

    // ---- operations ------------------------------------------------------

    /// Move the receive dial.
    ///
    /// Remembered always, applied only while receiving. The engine can move the
    /// dial during an over — a second VFO, a click on the panadapter, a
    /// scrolling waterfall — and retuning the synthesiser then would drag the
    /// *transmitter* off frequency mid-transmission. [`Self::end_tx`] tunes back
    /// to whatever this last stored, so the move still lands, at the moment it
    /// becomes safe.
    pub fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center_hz = hz;
        if matches!(self.mode, TransceiverMode::Transmit) {
            return Ok(());
        }
        self.retune()
    }

    /// Change the sample rate, and re-tune afterwards.
    ///
    /// The retune is not redundant. Programming the rate reprograms the clock
    /// generator the synthesiser is derived from, so the frequency has to be
    /// restated or the radio is left near — but not on — the dial.
    pub fn set_rate(&mut self, hz: f64) -> Result<()> {
        self.rate_hz = self.snap_rate(hz);
        self.apply_rate()?;
        self.retune()
    }

    /// Bring a requested rate inside what *this* board takes, saying so in the
    /// trace when it moves.
    fn snap_rate(&self, hz: f64) -> f64 {
        let (min, max) = self.kind.rate_range();
        let want = hz.clamp(min, max);
        if want != hz {
            self.io.trace().note(format!(
                "rate {hz} is outside this {}'s {:.1}–{:.1} Msps; using {want}",
                self.kind.name(),
                min / 1e6,
                max / 1e6,
            ));
        }
        want
    }

    pub fn set_filter_pref(&mut self, hz: f64) -> Result<()> {
        self.filter_pref_hz = hz;
        self.apply_filter()
    }

    pub fn set_ppm(&mut self, ppm: f64) -> Result<()> {
        self.ppm = ppm;
        self.retune()
    }

    /// Set one gain stage, returning the dB the hardware really applied.
    pub fn set_gain(&mut self, stage: GainSetter, db: f64) -> Result<f64> {
        let applied = self.io.set_gain(stage, db)? as f64;
        match stage {
            GainSetter::Lna => self.lna_db = applied,
            GainSetter::Vga => self.vga_db = applied,
            GainSetter::TxVga => self.txvga_db = applied,
        }
        Ok(applied)
    }

    /// The 14 dB amplifier, for the direction the radio is currently in.
    ///
    /// Two settings, one switch. Which one a write lands on depends on where
    /// the radio is now, and both are remembered so the next direction change
    /// restores the right one — that is the whole difference from SoapyHackRF,
    /// which caches a single value and loses it on the first over.
    pub fn set_amp(&mut self, tx: bool, on: bool) -> Result<()> {
        if tx {
            self.tx_amp = on;
        } else {
            self.amp = on;
        }
        // Only touch the hardware if this is the direction in use; the other
        // setting is applied when the radio next enters that direction.
        let live = matches!(self.mode, TransceiverMode::Transmit) == tx;
        if live {
            self.io.out(Request::AmpEnable, on as u16, 0)?;
        }
        Ok(())
    }

    pub fn set_bias_tee(&mut self, on: bool) -> Result<()> {
        self.bias_tee = on;
        self.apply_bias_tee()
    }

    pub fn has_bias_tee(&self) -> bool {
        self.kind.has_bias_tee()
    }

    /// The control half of an `RX → TX` transition.
    ///
    /// The caller must already have closed the receive endpoint — not merely
    /// idled it. Deactivating and reactivating leaves the receive path
    /// returning repeated and aliased buffers until the device is reopened,
    /// which is `sdroxide-radio`'s hardest-won piece of knowledge about this
    /// radio and is not something this driver gets to rediscover.
    ///
    /// Note there is no transmit-side LO offset: the engine's upconverter
    /// centres the signal at DC and this tunes straight to it, so the radio's
    /// own LO leakage lands on the carrier. Inherent to the design, documented,
    /// and not fixable here.
    pub fn begin_tx(&mut self, tx_center_hz: f64) -> Result<()> {
        self.io.trace().direction("RX -> TX");
        self.tx_center_hz = tx_center_hz;
        self.set_mode(TransceiverMode::Off)?;
        let hz = self.corrected(tx_center_hz);
        self.io.control_out(Request::SetFreq, 0, 0, &encode_set_freq(hz))?;
        self.set_mode(TransceiverMode::Transmit)?;
        // After the mode change, never before: the direction change is what
        // drops the amplifier.
        self.apply_tx_front_end()
    }

    /// The control half of a `TX → RX` transition.
    ///
    /// Off the air first, then back to the receive frequency and mode, then the
    /// receive front end. The amplifier goes down in the same breath as the
    /// mode so there is no window in which the radio is keyed with the
    /// transmit amplifier still in circuit.
    pub fn end_tx(&mut self) -> Result<()> {
        self.io.trace().direction("TX -> RX");
        self.set_mode(TransceiverMode::Off)?;
        self.io.out(Request::AmpEnable, 0, 0)?;
        let hz = self.corrected(self.center_hz);
        self.io.control_out(Request::SetFreq, 0, 0, &encode_set_freq(hz))?;
        self.set_mode(TransceiverMode::Receive)?;
        self.apply_rx_front_end()
    }

    /// Retune while transmitting, for split and for a dial moved mid-over.
    pub fn set_tx_center_hz(&mut self, hz: f64) -> Result<()> {
        self.tx_center_hz = hz;
        if matches!(self.mode, TransceiverMode::Transmit) {
            let corrected = self.corrected(hz);
            self.io.control_out(Request::SetFreq, 0, 0, &encode_set_freq(corrected))?;
        }
        Ok(())
    }

    /// Put the radio away.
    ///
    /// Idempotent and best-effort: every step is attempted even if an earlier
    /// one failed, because this runs on the way out — including from a `Drop`
    /// after something has already gone wrong — and a radio left keyed because
    /// an earlier transfer errored is the one outcome that is not acceptable.
    pub fn shutdown(&mut self) {
        self.io.trace().direction("shutdown");
        let _ = self.io.out(Request::SetTransceiverMode, TransceiverMode::Off as u16, 0);
        self.mode = TransceiverMode::Off;
        let _ = self.io.out(Request::AmpEnable, 0, 0);
        let _ = self.io.out(Request::AntennaEnable, 0, 0);
    }

    /// Ask the radio what its own M0 core is doing.
    ///
    /// `None` on a firmware too old to have the request, which is the only
    /// reason this is not simply part of the trace at every key-down. It is
    /// read when the transmit lane reports that nothing is completing, because
    /// it is the one question the host cannot answer for itself: a transfer
    /// that has not come back looks identical whether the radio is ignoring it
    /// or has merely not reached it yet.
    pub fn m0_state(&self) -> Option<M0State> {
        self.io
            .optional_in(Request::GetM0State, 0, 0, M0_STATE_LEN)
            .and_then(|b| M0State::parse(&b))
    }

    /// Borrow the transport, for the streaming code's endpoint access.
    pub fn io(&self) -> &T {
        &self.io
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::fake::FakeTransport;

    fn cfg() -> HackRfConfig {
        HackRfConfig { sample_rate_hz: 2.0e6, lna_db: 16.0, vga_db: 16.0, ..Default::default() }
    }

    fn open() -> Device<FakeTransport> {
        Device::open(FakeTransport::new(), &cfg(), 100.0e6).expect("the fake accepts everything")
    }

    /// The transmitter is made safe before anything else touches the radio.
    /// However this open fails, and whatever a previous program left behind, it
    /// must not be left able to emit power.
    #[test]
    fn opening_silences_the_transmitter_before_anything_else() {
        let dev = open();
        let calls = dev.io().brief();

        let mode_off = calls
            .iter()
            .position(|c| c == "SetTransceiverMode(0)")
            .expect("the mode goes to Off first");
        let txvga = dev.io().first(Request::SetTxvgaGain).expect("TXVGA is zeroed");
        let amp = dev.io().first(Request::AmpEnable).expect("the amp is bypassed");

        assert_eq!(mode_off, 0, "mode Off is the very first thing sent:\n{calls:#?}");
        assert!(txvga < amp);
        assert_eq!(dev.io().calls()[txvga].index, 0, "TXVGA must be zeroed, not merely set");
        assert_eq!(dev.io().calls()[amp].value, 0, "the amp must be bypassed");

        // And all of that precedes any tuning or receive gain.
        let freq = dev.io().first(Request::SetFreq).expect("tuned at some point");
        assert!(amp < freq, "the safety rail comes before tuning:\n{calls:#?}");
    }

    /// The rate and the filter are one operation. A rate change that left the
    /// filter alone would silently withdraw the LO offset.
    #[test]
    fn a_rate_change_reprograms_the_filter_and_retunes_in_that_order() {
        let mut dev = open();
        dev.io().clear();
        dev.set_rate(20.0e6).expect("accepted");

        let rate = dev.io().first(Request::SampleRateSet).expect("rate sent");
        let filt = dev.io().first(Request::BasebandFilterBandwidthSet).expect("filter sent");
        let freq = dev.io().first(Request::SetFreq).expect("retuned");
        assert!(rate < filt, "the filter follows the rate it is derived from");
        assert!(
            filt < freq,
            "the retune is last: programming the rate moves the clock the \
             synthesiser hangs off, so the frequency has to be restated"
        );
        assert_eq!(dev.filter_bw(), 15_000_000, "20 Msps wants the 15 MHz filter");
    }

    /// The ordering that SoapyHackRF gets wrong, asserted directly: the
    /// amplifier is programmed *after* the mode change, in both directions.
    #[test]
    fn the_front_end_is_reprogrammed_after_every_mode_change() {
        let mut dev = open();

        dev.io().clear();
        dev.begin_tx(14.074e6).expect("keys up");
        let calls = dev.io().brief();
        let tx_mode =
            calls.iter().position(|c| c == "SetTransceiverMode(2)").expect("mode Transmit sent");
        let txvga = dev.io().first(Request::SetTxvgaGain).expect("TXVGA applied");
        let amp = dev.io().first(Request::AmpEnable).expect("amp applied");
        assert!(tx_mode < txvga, "the drive stage follows the mode change:\n{calls:#?}");
        assert!(tx_mode < amp, "the amplifier follows the mode change:\n{calls:#?}");
        // And the frequency was set while the radio was off, not while keyed.
        let off = calls.iter().position(|c| c == "SetTransceiverMode(0)").unwrap();
        let freq = dev.io().first(Request::SetFreq).unwrap();
        assert!(off < freq && freq < tx_mode, "retune happens with the radio off:\n{calls:#?}");

        dev.io().clear();
        dev.end_tx().expect("unkeys");
        let calls = dev.io().brief();
        let rx_mode =
            calls.iter().position(|c| c == "SetTransceiverMode(1)").expect("mode Receive sent");
        let lna = dev.io().first(Request::SetLnaGain).expect("LNA reapplied");
        let vga = dev.io().first(Request::SetVgaGain).expect("VGA reapplied");
        assert!(rx_mode < lna && rx_mode < vga, "receive gains follow the mode:\n{calls:#?}");
        // Off the air before anything else on the way back.
        assert_eq!(calls[0], "SetTransceiverMode(0)", "unkey first:\n{calls:#?}");
    }

    /// One switch, two settings. An operator who runs the preamp bypassed on
    /// receive and in circuit on transmit must get exactly that on every over —
    /// this is the thing SoapyHackRF cannot express at all.
    #[test]
    fn the_shared_amp_follows_the_direction() {
        let mut c = cfg();
        c.amp = false;
        c.tx_amp = true;
        let mut dev = Device::open(FakeTransport::new(), &c, 100.0e6).unwrap();

        dev.io().clear();
        dev.begin_tx(14.074e6).unwrap();
        let amp = dev.io().first(Request::AmpEnable).expect("amp written on the way in");
        assert_eq!(dev.io().calls()[amp].value, 1, "transmit wants the amp in circuit");

        dev.io().clear();
        dev.end_tx().unwrap();
        // Two writes: down with the mode, then the receive setting.
        let writes: Vec<u16> =
            dev.io().all(Request::AmpEnable).iter().map(|i| dev.io().calls()[*i].value).collect();
        assert_eq!(writes, vec![0, 0], "off the air, then bypassed for receive");

        // Repeat the cycle: the transmit setting must not have been lost.
        dev.io().clear();
        dev.begin_tx(14.074e6).unwrap();
        let amp = dev.io().first(Request::AmpEnable).unwrap();
        assert_eq!(
            dev.io().calls()[amp].value,
            1,
            "the transmit amp must work on the second over too — losing it here \
             is precisely the SoapyHackRF defect"
        );
    }

    /// A write to the direction that is not current must be remembered, not
    /// applied — and must take effect on the next transition.
    #[test]
    fn an_amp_write_for_the_other_direction_waits_its_turn() {
        let mut dev = open();
        dev.io().clear();
        // Receiving: a transmit-amp change must not touch the hardware.
        dev.set_amp(true, true).unwrap();
        assert!(dev.io().all(Request::AmpEnable).is_empty(), "not while receiving");

        // ...but it is applied when the radio next enters transmit.
        dev.io().clear();
        dev.begin_tx(14.074e6).unwrap();
        let amp = dev.io().first(Request::AmpEnable).unwrap();
        assert_eq!(dev.io().calls()[amp].value, 1);

        // And the reverse: a receive-amp change while keyed is deferred.
        dev.io().clear();
        dev.set_amp(false, true).unwrap();
        assert!(dev.io().all(Request::AmpEnable).is_empty(), "not while transmitting");
    }

    /// Shutdown must reach mode Off with the amplifier and bias tee down from
    /// every state, including mid-transmission, and must not give up partway
    /// because one transfer failed.
    #[test]
    fn shutdown_always_gets_off_the_air() {
        let mut dev = open();
        dev.begin_tx(14.074e6).unwrap();
        assert_eq!(dev.mode(), TransceiverMode::Transmit);

        dev.io().clear();
        dev.shutdown();
        let calls = dev.io().brief();
        assert_eq!(calls[0], "SetTransceiverMode(0)", "off the air first:\n{calls:#?}");
        assert!(calls.contains(&"AmpEnable(0)".to_string()), "{calls:#?}");
        assert!(calls.contains(&"AntennaEnable(0)".to_string()), "{calls:#?}");
        assert_eq!(dev.mode(), TransceiverMode::Off);

        // Idempotent: a second call is harmless.
        dev.io().clear();
        dev.shutdown();
        assert_eq!(dev.io().brief()[0], "SetTransceiverMode(0)");
    }

    /// Issue #220: a radio that refuses `SET_ANTENNA_ENABLE` still opens.
    ///
    /// The firmware stalls that request outright on any board that is not a
    /// HackRF One OG, a HackRF One r9 or a Pro
    /// (`hackrf_usb/usb_api_transceiver.c`), and the safety rail sends it
    /// before the board has been identified. Treating the stall as fatal took
    /// the radio off the air three requests into the open — while every other
    /// program on the same machine, which only sends it when the operator asks
    /// for the bias tee, worked fine.
    #[test]
    fn a_radio_that_refuses_the_bias_tee_still_opens() {
        let io = FakeTransport::new().stalling(Request::AntennaEnable);
        let dev = Device::open(io, &cfg(), 100.0e6).expect("the open must survive the refusal");
        // And it got all the way through: tuned, and receiving.
        assert_eq!(dev.mode(), TransceiverMode::Receive);
        assert!(dev.io().first(Request::SetFreq).is_some(), "the radio was never tuned");
        // The transmitter was still made safe, which is the whole point of the
        // rail the refusal happened in.
        let amp = dev.io().first(Request::AmpEnable).expect("the amp was bypassed");
        assert_eq!(dev.io().calls()[amp].value, 0);
    }

    /// ...and asking for the bias tee on a radio that then refuses it is a
    /// setting that does not take, not a radio that falls over.
    #[test]
    fn a_refused_bias_tee_is_a_setting_that_does_not_take() {
        let mut c = cfg();
        c.bias_tee = true;
        let io = FakeTransport::new().stalling(Request::AntennaEnable);
        let mut dev = Device::open(io, &c, 100.0e6).expect("opens");
        dev.io().clear();
        dev.set_bias_tee(true).expect("must not fail the radio");
        assert!(
            dev.io().all(Request::AntennaEnable).len() <= 1,
            "a refusal must not be retried on every front-end pass",
        );
    }

    /// A board with no bias-tee circuit must not be sent the request. The
    /// firmware would accept it and nothing would happen, which is a worse
    /// answer than never offering the control.
    #[test]
    fn a_board_without_a_bias_tee_is_never_sent_one() {
        // Board id 3 is a rad1o.
        let io = FakeTransport::new().reply(Request::BoardIdRead, &[3]);
        let mut c = cfg();
        c.bias_tee = true;
        let mut dev = Device::open(io, &c, 100.0e6).unwrap();
        assert_eq!(dev.kind(), BoardKind::Rad1o);
        assert!(!dev.has_bias_tee());

        dev.io().clear();
        dev.set_bias_tee(true).unwrap();
        assert!(dev.io().all(Request::AntennaEnable).is_empty(), "the rad1o has no bias tee");

        // A HackRF One does get it.
        let io = FakeTransport::new().reply(Request::BoardIdRead, &[2]);
        let mut dev = Device::open(io, &c, 100.0e6).unwrap();
        dev.io().clear();
        dev.set_bias_tee(true).unwrap();
        assert_eq!(dev.io().all(Request::AntennaEnable).len(), 1);
    }

    /// The crystal correction has to be exact rather than first-order, and it
    /// has to apply to the transmit frequency too — a rig that is 10 ppm off on
    /// receive is 10 ppm off on transmit.
    #[test]
    fn the_ppm_correction_applies_in_both_directions() {
        let mut c = cfg();
        c.ppm = 10.0;
        let mut dev = Device::open(FakeTransport::new(), &c, 100.0e6).unwrap();

        dev.io().clear();
        dev.set_center_hz(100.0e6).unwrap();
        let call = &dev.io().calls()[dev.io().first(Request::SetFreq).unwrap()];
        // 100 MHz / 1.00001 = 99_999_000.01
        assert_eq!(call.data, encode_set_freq(99_999_000));

        dev.io().clear();
        dev.begin_tx(100.0e6).unwrap();
        let call = &dev.io().calls()[dev.io().first(Request::SetFreq).unwrap()];
        assert_eq!(call.data, encode_set_freq(99_999_000), "transmit is corrected too");

        // Zero ppm is the identity, not an approximation of it.
        let mut dev = Device::open(FakeTransport::new(), &cfg(), 100.0e6).unwrap();
        dev.io().clear();
        dev.set_center_hz(100.0e6).unwrap();
        let call = &dev.io().calls()[dev.io().first(Request::SetFreq).unwrap()];
        assert_eq!(call.data, encode_set_freq(100_000_000));
    }

    /// Moving the receive dial during an over must not drag the transmitter off
    /// frequency — but it must not be lost either. It lands on unkey.
    #[test]
    fn a_dial_move_while_keyed_is_deferred_not_dropped() {
        let mut dev = open();
        dev.begin_tx(14.074e6).unwrap();

        dev.io().clear();
        dev.set_center_hz(21.0e6).unwrap();
        assert!(
            dev.io().all(Request::SetFreq).is_empty(),
            "retuning here would move the transmitter mid-over"
        );

        dev.io().clear();
        dev.end_tx().unwrap();
        let call = &dev.io().calls()[dev.io().first(Request::SetFreq).unwrap()];
        assert_eq!(call.data, encode_set_freq(21_000_000), "the move lands on unkey");
        assert_eq!(dev.center_hz(), 21.0e6);
    }

    /// The hardware truncates gains to its step; the value handed back is what
    /// it really did, because that is what the UI shows.
    #[test]
    fn a_gain_reports_what_the_hardware_actually_did() {
        let mut dev = open();
        assert_eq!(dev.set_gain(GainSetter::Lna, 15.0).unwrap(), 8.0);
        assert_eq!(dev.gains().0, 8.0);
        assert_eq!(dev.set_gain(GainSetter::Vga, 61.0).unwrap(), 60.0);
        assert_eq!(dev.set_gain(GainSetter::TxVga, 23.0).unwrap(), 23.0);
    }

    /// A rate the hardware cannot produce is clamped and said so, not refused —
    /// the same "snap and explain" contract the Airspy HF+ backend uses.
    #[test]
    fn an_impossible_rate_is_clamped_rather_than_refused() {
        let mut dev = open();
        dev.set_rate(50.0e6).expect("clamped, not refused");
        assert_eq!(dev.rate_hz(), 20.0e6);
        dev.set_rate(0.5e6).expect("clamped, not refused");
        assert_eq!(dev.rate_hz(), 2.0e6);
    }

    /// The floor belongs to the board, and the board is not known until
    /// `identify` has answered — so a rate the Pro can run must survive the
    /// open rather than being clamped to a HackRF One's 2 Msps on the way in.
    #[test]
    fn a_pro_keeps_the_narrow_rate_it_was_opened_with() {
        let io = FakeTransport::new().reply(Request::BoardIdRead, &[5]);
        let mut c = cfg();
        c.sample_rate_hz = 500.0e3;
        let mut dev = Device::open(io, &c, 14.1e6).unwrap();
        assert_eq!(dev.kind(), BoardKind::HackRfPro);
        assert_eq!(dev.rate_hz(), 500.0e3, "the Pro's floor is 200 ksps, not 2 Msps");
        assert_eq!(dev.rate_range(), (200.0e3, 20.0e6));

        // Still bounded at both ends, just at this board's bounds.
        dev.set_rate(100.0e3).unwrap();
        assert_eq!(dev.rate_hz(), 200.0e3);
        dev.set_rate(50.0e6).unwrap();
        assert_eq!(dev.rate_hz(), 20.0e6);

        // The same rate on a HackRF One is out of range and gets clamped, which
        // is the behaviour that must not change.
        let io = FakeTransport::new().reply(Request::BoardIdRead, &[2]);
        let dev = Device::open(io, &c, 14.1e6).unwrap();
        assert_eq!(dev.kind(), BoardKind::HackRfOne);
        assert_eq!(dev.rate_hz(), 2.0e6);
    }

    /// The Pro derives its own baseband filter and discards what the host asks
    /// for. Sending the request anyway would leave the driver reporting a
    /// bandwidth the radio is not running — and that figure is what the
    /// zero-IF LO offset is computed from.
    #[test]
    fn a_pro_is_never_sent_a_baseband_filter_request() {
        let io = FakeTransport::new().reply(Request::BoardIdRead, &[5]);
        let mut c = cfg();
        c.filter_bw_hz = 5.0e6;
        let mut dev = Device::open(io, &c, 14.1e6).unwrap();
        assert!(dev.sets_own_filter());
        assert!(
            dev.io().all(Request::BasebandFilterBandwidthSet).is_empty(),
            "the firmware would accept this and ignore it"
        );
        // What is reported is what the radio really settles on: 0.75 of the
        // rate, unquantised, rather than the MAX2837 table's nearest step.
        assert_eq!(dev.filter_bw(), 1_500_000, "0.75 × 2 Msps");

        // An override applied later is equally ignored, and the reported width
        // follows the rate instead.
        dev.io().clear();
        dev.set_filter_pref(28.0e6).unwrap();
        assert!(dev.io().all(Request::BasebandFilterBandwidthSet).is_empty());
        assert_eq!(dev.filter_bw(), 1_500_000);

        dev.set_rate(20.0e6).unwrap();
        assert_eq!(dev.filter_bw(), 15_000_000);

        // A HackRF One still honours the override, which is the behaviour this
        // must not have cost anybody.
        let io = FakeTransport::new().reply(Request::BoardIdRead, &[2]);
        let dev = Device::open(io, &c, 14.1e6).unwrap();
        assert!(!dev.sets_own_filter());
        assert_eq!(dev.io().all(Request::BasebandFilterBandwidthSet).len(), 1);
        assert_eq!(dev.filter_bw(), 5_000_000);
    }
}
