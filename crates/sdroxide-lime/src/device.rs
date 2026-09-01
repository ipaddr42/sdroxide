//! The one place an `lms_device_t*` is dereferenced.
//!
//! Everything that takes a device pointer goes through [`DevCtl`], including
//! the LimeRFE calls — which reach the board by bit-banging I²C on the
//! LimeSDR's GPIO pins and so touch the same device. Keeping them behind one
//! type is what makes it possible to say where the boundary is: the streaming
//! calls take an `lms_stream_t*` and touch only LimeSuite's own FIFO, so they
//! never come through here.

use std::ffi::c_char;
use std::sync::Arc;

use sdroxide_types::LimeConfig;

use crate::error::{Error, Result};
use crate::ffi;
use crate::trace::Trace;

/// What the board says it is.
#[derive(Debug, Clone, Default)]
pub struct DevInfo {
    pub name: String,
    pub firmware: String,
    pub hardware: String,
    pub gateware: String,
    pub serial: String,
}

pub struct DevCtl {
    api: Arc<ffi::Api>,
    dev: ffi::Device,
    channel: usize,
    /// The session's diagnostic trace. Every state-changing call here writes
    /// one line into it, which is what the settings panel's **Copy diagnostic
    /// report** button hands to an issue: this backend cannot be desk-checked
    /// against hardware, so what was asked for and what LimeSuite said has to
    /// survive the session that did it.
    trace: Trace,
}

// The pointer is only ever reachable through `&mut self`, and the type is not
// `Clone`, so there is exactly one owner at a time.
unsafe impl Send for DevCtl {}

impl DevCtl {
    pub(crate) fn new(
        api: Arc<ffi::Api>,
        dev: ffi::Device,
        channel: usize,
        trace: Trace,
    ) -> DevCtl {
        DevCtl { api, dev, channel, trace }
    }

    /// The session's trace, for the callers that record something this type
    /// did not do itself — the streams, and the LimeRFE's board link.
    pub(crate) fn trace(&self) -> &Trace {
        &self.trace
    }

    pub(crate) fn api(&self) -> &Arc<ffi::Api> {
        &self.api
    }

    pub(crate) fn raw(&self) -> ffi::Device {
        self.dev
    }

    pub fn channel(&self) -> usize {
        self.channel
    }

    /// Every call here funnels through this: LimeSuite reports failure as `-1`
    /// and puts the reason somewhere else entirely.
    ///
    /// A failure is traced whatever the call was, including the read-back ones
    /// — a refusal is always worth a line. A *success* is traced only by
    /// [`Self::checked`], because the periodic reads would otherwise fill the
    /// ring and push the interesting half of the session out of it.
    fn check(&self, call: &'static str, rc: std::ffi::c_int) -> Result<()> {
        if rc == ffi::OK {
            return Ok(());
        }
        let e = Error::api(call, self.api.err_text());
        self.trace.call(call, "", format!("FAILED: {e}"));
        Err(e)
    }

    /// The same, for a call that changes the state of the chip: traced whether
    /// it worked or not, with what it was asked for.
    ///
    /// "It answered every command and passed no signal" is the report this
    /// backend gets, and it is only answerable from a record of what the
    /// commands were.
    fn checked(
        &self,
        call: &'static str,
        detail: impl AsRef<str>,
        rc: std::ffi::c_int,
    ) -> Result<()> {
        if rc == ffi::OK {
            self.trace.call(call, detail, "ok");
            return Ok(());
        }
        let e = Error::api(call, self.api.err_text());
        self.trace.call(call, detail, format!("FAILED: {e}"));
        Err(e)
    }

    /// Put the chip into the state LimeSuite calls "ready for operation". Must
    /// come before anything else — the datasheet default is not it.
    pub fn init(&mut self) -> Result<()> {
        let rc = unsafe { (self.api.init)(self.dev) };
        self.checked("LMS_Init", "", rc)
    }

    pub fn num_channels(&self, tx: bool) -> usize {
        let n = unsafe { (self.api.get_num_channels)(self.dev, tx) };
        if n < 0 { 0 } else { n as usize }
    }

    pub fn enable_channel(&mut self, tx: bool, on: bool) -> Result<()> {
        self.enable_channel_on(tx, self.channel, on)
    }

    /// The same on a channel that is not this session's own — the board's
    /// second receive chain, which issue #98 puts a second aerial on. Every
    /// `_on` method here exists for that one caller; the plain forms are the
    /// same call against [`Self::channel`].
    pub fn enable_channel_on(&mut self, tx: bool, channel: usize, on: bool) -> Result<()> {
        let rc = unsafe { (self.api.enable_channel)(self.dev, tx, channel, on) };
        self.checked(
            "LMS_EnableChannel",
            format!("{} ch{} {}", dir(tx), channel + 1, if on { "on" } else { "off" }),
            rc,
        )
    }

    /// Set the host sample rate for every channel at once — LimeSuite has no
    /// per-channel form, and on this silicon the two directions share a clock
    /// tree anyway.
    pub fn set_sample_rate(&mut self, rate: f64, oversample: u8) -> Result<()> {
        let rc = unsafe { (self.api.set_sample_rate)(self.dev, rate, oversample as usize) };
        self.checked(
            "LMS_SetSampleRate",
            format!("{:.4} Msps, oversample {oversample}", rate / 1e6),
            rc,
        )
    }

    /// The rate actually in force, host side. Worth reading back rather than
    /// assuming: LimeSuite snaps to what the clock tree can synthesise.
    pub fn sample_rate(&self, tx: bool) -> Result<f64> {
        let mut host = 0.0f64;
        let mut rf = 0.0f64;
        let rc =
            unsafe { (self.api.get_sample_rate)(self.dev, tx, self.channel, &mut host, &mut rf) };
        self.check("LMS_GetSampleRate", rc)?;
        Ok(host)
    }

    pub fn rate_range(&self, tx: bool) -> Result<ffi::Range> {
        let mut r = ffi::Range::default();
        let rc = unsafe { (self.api.get_sample_rate_range)(self.dev, tx, &mut r) };
        self.check("LMS_GetSampleRateRange", rc)?;
        Ok(r)
    }

    pub fn set_lo(&mut self, tx: bool, hz: f64) -> Result<()> {
        let rc = unsafe { (self.api.set_lo_frequency)(self.dev, tx, self.channel, hz) };
        self.checked("LMS_SetLOFrequency", format!("{} {:.6} MHz", dir(tx), hz / 1e6), rc)
    }

    pub fn lo(&self, tx: bool) -> Result<f64> {
        let mut hz = 0.0f64;
        let rc = unsafe { (self.api.get_lo_frequency)(self.dev, tx, self.channel, &mut hz) };
        self.check("LMS_GetLOFrequency", rc)?;
        Ok(hz)
    }

    /// The synthesiser's reach.
    ///
    /// Read, never assumed, and the single most load-bearing thing this module
    /// reports: a LimeSDR asked for a frequency below the LMS7002M's range
    /// reconfigures its interface clock, fails half way, and then delivers
    /// nothing at all until the process restarts. The engine's retune guard is
    /// what stops the call being made, and it can only work from a published
    /// range.
    pub fn lo_range(&self, tx: bool) -> Result<ffi::Range> {
        let mut r = ffi::Range::default();
        let rc = unsafe { (self.api.get_lo_frequency_range)(self.dev, tx, &mut r) };
        self.check("LMS_GetLOFrequencyRange", rc)?;
        if !(r.min.is_finite() && r.max.is_finite()) || r.max <= r.min {
            return Err(Error::api(
                "LMS_GetLOFrequencyRange",
                format!("nonsensical range {}..{}", r.min, r.max),
            ));
        }
        Ok(r)
    }

    /// The port names this board offers, minus `NONE` — which is a real entry
    /// in LimeSuite's list and means "disconnected", not a choice anyone makes
    /// from a combo.
    pub fn antennas(&self, tx: bool) -> Vec<String> {
        self.antennas_on(tx, self.channel)
    }

    pub fn antennas_on(&self, tx: bool, channel: usize) -> Vec<String> {
        let n = unsafe { (self.api.get_antenna_list)(self.dev, tx, channel, std::ptr::null_mut()) };
        if n <= 0 {
            return Vec::new();
        }
        let mut buf = vec![[0 as c_char; ffi::NAME_LEN]; n as usize];
        let n = unsafe { (self.api.get_antenna_list)(self.dev, tx, channel, buf.as_mut_ptr()) };
        if n <= 0 {
            return Vec::new();
        }
        buf.iter()
            .take(n as usize)
            .map(|e| ffi::c_field(e))
            .filter(|s| !s.is_empty() && !s.eq_ignore_ascii_case("none"))
            .collect()
    }

    /// Select a port by name. The index is the position in LimeSuite's own
    /// list, `NONE` included, so the lookup is done against the unfiltered list
    /// rather than the one shown to the operator.
    pub fn set_antenna_named(&mut self, tx: bool, name: &str) -> Result<()> {
        self.set_antenna_named_on(tx, self.channel, name)
    }

    pub fn set_antenna_named_on(&mut self, tx: bool, channel: usize, name: &str) -> Result<()> {
        let n = unsafe { (self.api.get_antenna_list)(self.dev, tx, channel, std::ptr::null_mut()) };
        if n <= 0 {
            return Err(Error::api("LMS_GetAntennaList", self.api.err_text()));
        }
        let mut buf = vec![[0 as c_char; ffi::NAME_LEN]; n as usize];
        let n = unsafe { (self.api.get_antenna_list)(self.dev, tx, channel, buf.as_mut_ptr()) };
        let idx = buf
            .iter()
            .take(n.max(0) as usize)
            .position(|e| ffi::c_field(e).eq_ignore_ascii_case(name.trim()))
            .ok_or_else(|| {
                Error::api("LMS_SetAntenna", format!("this board has no port called {name:?}"))
            })?;
        let rc = unsafe { (self.api.set_antenna)(self.dev, tx, channel, idx) };
        self.checked(
            "LMS_SetAntenna",
            format!("{} ch{} {} (index {idx})", dir(tx), channel + 1, name.trim()),
            rc,
        )
    }

    pub fn antenna(&self, tx: bool) -> String {
        let idx = unsafe { (self.api.get_antenna)(self.dev, tx, self.channel) };
        if idx < 0 {
            return String::new();
        }
        let n = unsafe {
            (self.api.get_antenna_list)(self.dev, tx, self.channel, std::ptr::null_mut())
        };
        if n <= 0 {
            return String::new();
        }
        let mut buf = vec![[0 as c_char; ffi::NAME_LEN]; n as usize];
        let n =
            unsafe { (self.api.get_antenna_list)(self.dev, tx, self.channel, buf.as_mut_ptr()) };
        buf.get(idx as usize).filter(|_| n > 0).map(|e| ffi::c_field(e)).unwrap_or_default()
    }

    /// The combined gain. LimeSuite takes an integer, so this rounds and clamps
    /// — and [`Self::gain_db`] reads back what the chip actually got, which is
    /// what the settings panel shows.
    pub fn set_gain_db(&mut self, tx: bool, db: f64) -> Result<()> {
        self.set_gain_db_on(tx, self.channel, db)
    }

    pub fn set_gain_db_on(&mut self, tx: bool, channel: usize, db: f64) -> Result<()> {
        let g = db
            .round()
            .clamp(sdroxide_types::LimeConfig::GAIN_MIN_DB, sdroxide_types::LimeConfig::GAIN_MAX_DB)
            as u32;
        let rc = unsafe { (self.api.set_gain_db)(self.dev, tx, channel, g) };
        self.checked("LMS_SetGaindB", format!("{} ch{} {g} dB", dir(tx), channel + 1), rc)
    }

    pub fn gain_db(&self, tx: bool) -> Option<f64> {
        self.gain_db_on(tx, self.channel)
    }

    pub fn gain_db_on(&self, tx: bool, channel: usize) -> Option<f64> {
        let mut g = 0u32;
        let rc = unsafe { (self.api.get_gain_db)(self.dev, tx, channel, &mut g) };
        (rc == ffi::OK).then_some(f64::from(g))
    }

    pub fn set_lpf_bw(&mut self, tx: bool, hz: f64) -> Result<()> {
        self.set_lpf_bw_on(tx, self.channel, hz)
    }

    pub fn set_lpf_bw_on(&mut self, tx: bool, channel: usize, hz: f64) -> Result<()> {
        let rc = unsafe { (self.api.set_lpf_bw)(self.dev, tx, channel, hz) };
        self.checked(
            "LMS_SetLPFBW",
            format!("{} ch{} {:.3} MHz", dir(tx), channel + 1, hz / 1e6),
            rc,
        )
    }

    pub fn lpf_range(&self, tx: bool) -> Result<ffi::Range> {
        let mut r = ffi::Range::default();
        let rc = unsafe { (self.api.get_lpf_bw_range)(self.dev, tx, &mut r) };
        self.check("LMS_GetLPFBWRange", rc)?;
        Ok(r)
    }

    /// LimeSuite's own DC-offset and IQ-imbalance calibration. Hundreds of
    /// milliseconds, so never in a tuning path.
    pub fn calibrate(&mut self, tx: bool, bw_hz: f64) -> Result<()> {
        self.calibrate_on(tx, self.channel, bw_hz)
    }

    pub fn calibrate_on(&mut self, tx: bool, channel: usize, bw_hz: f64) -> Result<()> {
        let rc = unsafe { (self.api.calibrate)(self.dev, tx, channel, bw_hz, ffi::CAL_FLAGS_NONE) };
        self.checked(
            "LMS_Calibrate",
            format!("{} ch{} bw {:.3} MHz", dir(tx), channel + 1, bw_hz / 1e6),
            rc,
        )
    }

    pub fn chip_temp_c(&self) -> Option<f64> {
        let mut t = 0.0f64;
        let rc = unsafe { (self.api.get_chip_temperature)(self.dev, 0, &mut t) };
        (rc == ffi::OK).then_some(t)
    }

    pub fn info(&self) -> DevInfo {
        let p = unsafe { (self.api.get_device_info)(self.dev) };
        if p.is_null() {
            return DevInfo::default();
        }
        // Copied out while the device is open: LimeSuite frees this storage on
        // close, and the header says so.
        let i = unsafe { *p };
        DevInfo {
            name: ffi::c_field(&i.device_name),
            firmware: ffi::c_field(&i.firmware_version),
            hardware: ffi::c_field(&i.hardware_version),
            gateware: ffi::c_field(&i.gateware_version),
            serial: format!("{:016X}", i.board_serial_number),
        }
    }
}

impl DevCtl {
    /// Close the device now rather than waiting for the last holder to drop —
    /// the reopen path needs the board free *before* the replacement's
    /// `LMS_Open`. Idempotent, so `Drop` running afterwards is harmless.
    ///
    /// The caller answers for ordering: nothing else may be using the pointer
    /// when this runs — see `LimeHandle::close` for what that means for the
    /// LimeRFE's board link.
    pub(crate) fn close(&mut self) {
        if !self.dev.is_null() {
            unsafe { (self.api.close)(self.dev) };
            self.dev = std::ptr::null_mut();
        }
    }
}

impl Drop for DevCtl {
    fn drop(&mut self) {
        self.close();
    }
}

/// The word for a direction, so a traced line reads the way the operator
/// thinks rather than as `tx: true`.
fn dir(tx: bool) -> &'static str {
    if tx { "transmit" } else { "receive" }
}

/// The analog filter width to use for a given sample rate, when the operator
/// has not named one.
///
/// **Wide on purpose.** A filter narrower than a quarter of the span silently
/// withdraws the zero-IF LO offset rather than merely softening the band edges
/// — see `sdroxide_radio::lo_offset_for`, whose doc spells out the trap — so
/// this errs generous and lets the digital filters do the selectivity.
pub fn auto_lpf_bw(rate_hz: f64, range: ffi::Range) -> f64 {
    let want = rate_hz * 1.25;
    if range.max > range.min && range.min > 0.0 { want.clamp(range.min, range.max) } else { want }
}

/// Below this, LimeSuite parks the synthesiser *at* it and the TSP NCO makes
/// up the difference — the LMS7002M's SX simply stops at 30 MHz
/// (`LMS7_Device::SetFrequency`).
pub const NCO_LO_FLOOR_HZ: f64 = 30e6;

/// The analog filter width to actually program, given the width the operator
/// wants and the centre the synthesiser is about to be handed.
///
/// Above 30 MHz this is the wanted width unchanged. Below it, the NCO trick
/// above puts the wanted signal up to 30 MHz away from DC *inside the analog
/// chain* — LimeSuite retunes the data converters to span that offset but
/// leaves the analog low-pass wherever it was told (`LMS7_Device::SetLPF`
/// tunes around DC, NCO-blind). A filter chosen from the sample rate alone
/// then sits with its corner at a few MHz while the signal rides at 8–28 MHz,
/// which on transmit is the difference between full power and milliwatts —
/// issue #118's "TX very low compared to SDR-Console" was exactly this.
///
/// The floor is the worst case for the whole of HF rather than the current
/// offset, deliberately: retuning these filters costs LimeSuite's MCU a few
/// hundred milliseconds, so the only boundary an ordinary tune may cross is
/// 30 MHz itself, once — never band-to-band within HF.
pub fn effective_lpf_bw(want_hz: f64, center_hz: f64, rate_hz: f64, range: ffi::Range) -> f64 {
    let mut bw = want_hz;
    if center_hz < NCO_LO_FLOOR_HZ {
        bw = bw.max((2.0 * NCO_LO_FLOOR_HZ + rate_hz) * 1.25);
    }
    if range.max > range.min && range.min > 0.0 { bw.clamp(range.min, range.max) } else { bw }
}

/// The receive port to use when the operator has not named one.
///
/// LimeSuite has an "auto" value for this, but what it does is undocumented, so
/// the choice is made here where it can be read. LNAL is the low-band input and
/// LNAH the high one; LNAW spans both at the cost of a couple of dB.
///
/// **A LimeRFE settles it on its own, at every frequency.** The front end is
/// one coaxial cable into one socket, and retuning does not move it — so the
/// rule that serves a bare board, follow the dial from one input to another,
/// is the rule that leaves a board with a front end listening on a socket with
/// nothing plugged into it. LNAW is the wideband input and the one a LimeRFE is
/// cabled to, because it is the only one that spans everything the board's
/// filters present; an operator who wired theirs to LNAL or LNAH names it
/// instead and [`crate::LimeHandle::set_antenna`] then pins it.
pub fn auto_antenna_rx(hz: f64, available: &[String], has_rfe: bool) -> Option<String> {
    let want = if has_rfe {
        "LNAW"
    } else if hz < 1.5e9 {
        "LNAL"
    } else {
        "LNAH"
    };
    available
        .iter()
        .find(|a| a.eq_ignore_ascii_case(want))
        .or_else(|| available.iter().find(|a| a.eq_ignore_ascii_case("LNAW")))
        .or_else(|| available.first())
        .cloned()
}

/// The transmit port to use when the operator has not named one.
///
/// Follows the frequency, exactly as [`auto_antenna_rx`] does and for exactly
/// the same reason: the two sockets are different matching networks, not two
/// jacks onto the same one. See [`tx_port_band`] for which is which — BAND1
/// carries everything from 30 MHz to 1.9 GHz, so it is the answer on every band
/// this is normally pointed at, and BAND2 is for 13 cm.
pub fn auto_antenna_tx(hz: f64, available: &[String]) -> Option<String> {
    let want = if LimeConfig::tx_port_covers("BAND2", hz) { "BAND2" } else { "BAND1" };
    available
        .iter()
        .find(|a| a.eq_ignore_ascii_case(want))
        .or_else(|| available.iter().find(|a| a.eq_ignore_ascii_case("BAND1")))
        .or_else(|| available.first())
        .cloned()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The filter has to stay clear of the quarter-span the LO offset needs,
    /// or the offset is withdrawn and the LO leakage lands back on the VFO.
    #[test]
    fn the_automatic_filter_is_wider_than_the_lo_offset_needs() {
        let range = ffi::Range { min: 1.4e6, max: 130.0e6, step: 0.0 };
        for rate in [2.0e6, 5.0e6, 10.0e6, 20.0e6] {
            let bw = auto_lpf_bw(rate, range);
            assert!(
                bw > rate * 0.25 / 0.45,
                "at {rate} the filter {bw} is too narrow to keep the LO offset"
            );
        }
    }

    /// And it stays inside what the chip will accept.
    #[test]
    fn the_automatic_filter_is_clamped_to_the_chips_range() {
        let range = ffi::Range { min: 1.4e6, max: 130.0e6, step: 0.0 };
        assert_eq!(auto_lpf_bw(0.2e6, range), 1.4e6, "clamped up to the minimum");
        assert_eq!(auto_lpf_bw(200.0e6, range), 130.0e6, "clamped down to the maximum");
    }

    /// Below 30 MHz the synthesiser parks there and the NCO carries the rest,
    /// so the signal rides at the offset inside the analog chain — the filter
    /// must span it, or transmit comes out at milliwatts (issue #118).
    #[test]
    fn below_30_mhz_the_filter_opens_for_the_nco_offset() {
        let range = ffi::Range { min: 5.0e6, max: 130.0e6, step: 0.0 };
        let rate = 5.0e6;
        let want = auto_lpf_bw(rate, range);
        let hf = effective_lpf_bw(want, 14.1e6, rate, range);
        assert!(
            hf >= 2.0 * NCO_LO_FLOOR_HZ + rate,
            "{hf} does not span the worst NCO offset plus the span"
        );
        // One figure for the whole of HF: tuning band to band below 30 MHz
        // must never land the slow filter retune.
        assert_eq!(hf, effective_lpf_bw(want, 1.8e6, rate, range));
        assert_eq!(hf, effective_lpf_bw(want, 29.9e6, rate, range));
        // Above the boundary the wanted width passes through untouched.
        assert_eq!(effective_lpf_bw(want, 145.5e6, rate, range), want);
        // And the chip's ceiling holds where the floor would pass it.
        let fast = effective_lpf_bw(auto_lpf_bw(61.44e6, range), 14.1e6, 61.44e6, range);
        assert!(fast <= range.max);
    }

    /// A hand-set narrow filter gets the same floor: the operator's number is
    /// a width for the *signal*, not permission to park the passband 20 MHz
    /// away from where the NCO put it.
    #[test]
    fn the_nco_floor_applies_to_a_hand_set_width_too() {
        let range = ffi::Range { min: 5.0e6, max: 130.0e6, step: 0.0 };
        let hf = effective_lpf_bw(2.5e6, 14.1e6, 2.0e6, range);
        assert!(hf >= 2.0 * NCO_LO_FLOOR_HZ + 2.0e6);
        assert_eq!(effective_lpf_bw(8.0e6, 145.5e6, 2.0e6, range), 8.0e6);
    }

    #[test]
    fn the_automatic_port_follows_the_frequency_and_falls_back() {
        let all: Vec<String> = ["LNAH", "LNAL", "LNAW"].iter().map(|s| s.to_string()).collect();
        assert_eq!(auto_antenna_rx(14.2e6, &all, false).as_deref(), Some("LNAL"));
        assert_eq!(auto_antenna_rx(2.4e9, &all, false).as_deref(), Some("LNAH"));

        // A board that offers only the wideband input still gets an answer.
        let wide = vec!["LNAW".to_string()];
        assert_eq!(auto_antenna_rx(14.2e6, &wide, false).as_deref(), Some("LNAW"));
        // And one that offers nothing at all gets none, rather than a guess.
        assert_eq!(auto_antenna_rx(14.2e6, &[], false), None);
    }

    /// The two transmit sockets are two matching networks, and the whole of
    /// amateur radio below 23 cm lives in one of them. Ranges from LimeSuite's
    /// own `GetTxPathBand` — see [`LimeConfig::tx_port_band`].
    #[test]
    fn the_automatic_transmit_port_follows_the_band() {
        let all: Vec<String> = ["BAND1", "BAND2"].iter().map(|s| s.to_string()).collect();
        for hz in [3.573e6, 14.074e6, 145.5e6, 435.0e6, 1.296e9] {
            assert_eq!(auto_antenna_tx(hz, &all).as_deref(), Some("BAND1"), "{hz} Hz");
        }
        assert_eq!(auto_antenna_tx(2.4e9, &all).as_deref(), Some("BAND2"), "13 cm");
        // A board offering only the microwave port gets it whatever the dial
        // says — a transmitter with one socket has no choice to make.
        let only2 = vec!["BAND2".to_string()];
        assert_eq!(auto_antenna_tx(145.5e6, &only2).as_deref(), Some("BAND2"));
        assert_eq!(auto_antenna_tx(145.5e6, &[]), None);
    }

    /// The report behind issue #94: keyed on 2 m out of BAND2, which this board
    /// matches for 13 cm. Every call answers `ok` and no RF appears.
    #[test]
    fn the_microwave_transmit_port_does_not_cover_two_metres() {
        assert!(!LimeConfig::tx_port_covers("BAND2", 145.475e6));
        assert!(LimeConfig::tx_port_covers("BAND1", 145.475e6));
        assert!(LimeConfig::tx_port_covers("BAND2", 2.4e9));
        assert!(!LimeConfig::tx_port_covers("BAND1", 2.4e9));
        // A name this table does not describe is left alone rather than
        // second-guessed.
        assert!(LimeConfig::tx_port_covers("TXW", 145.475e6));
    }

    /// The field report this exists for: with a LimeRFE on the wideband socket
    /// and the port left automatic, the frequency rule picked LNAL and the
    /// radio listened to an empty connector. A front end is one cable into one
    /// socket, so the answer must not move when the dial does.
    #[test]
    fn a_limerfe_pins_the_port_to_the_wideband_socket() {
        let all: Vec<String> = ["LNAH", "LNAL", "LNAW"].iter().map(|s| s.to_string()).collect();
        for hz in [3.7e6, 14.2e6, 145.5e6, 432.1e6, 1296.0e6, 2400.0e6] {
            assert_eq!(
                auto_antenna_rx(hz, &all, true).as_deref(),
                Some("LNAW"),
                "a front end does not move between sockets at {hz:.0} Hz"
            );
        }
        // And without one the same frequencies still split across the two
        // narrow inputs, which is right for a bare board.
        assert_eq!(auto_antenna_rx(145.5e6, &all, false).as_deref(), Some("LNAL"));
    }
}
