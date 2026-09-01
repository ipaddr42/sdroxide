//! An open LimeSDR: the device, its two streams, and everything done to them.
//!
//! # No thread
//!
//! Unlike every native USB backend here this one has no background thread and
//! no ring buffer, because LimeSuite already has both. `LMS_RecvStream` takes a
//! timeout and reads out of LimeSuite's own FIFO, which is exactly the
//! `IqSource::read` contract — so the shape to copy is the SoapySDR source in
//! `sdroxide-radio`, which drives this same library through SoapyLMS7 from the
//! engine thread and has done since before this backend existed.
//!
//! Stacking a second FIFO on top of LimeSuite's would add latency and buy
//! nothing. What it *does* mean is that a slow call — `LMS_Calibrate` above
//! all — must never land in a tuning path; see [`LimeHandle::set_center_hz`].
//!
//! # Both streams are set up at open
//!
//! `LMS_SetupStream` stops LimeSuite's running data threads to make room, which
//! is recorded next door in `sdroxide-radio`'s SoapySDR device as the reason a
//! stream restart there is so disruptive. Creating both directions while the
//! device is idle and then only starting and stopping them means that path is
//! never taken mid-session.

use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use num_complex::Complex32;
use sdroxide_types::{LimeAuxRole, LimeConfig};

use crate::auxrx::AuxRx;
use crate::device::{self, DevCtl, DevInfo};
use crate::error::{Error, Result};
use crate::ffi;
use crate::trace::{self, Trace};

// The zero-copy receive below hands LimeSuite a `&mut [Complex32]` as its
// interleaved-f32 buffer. `num_complex::Complex<T>` is `#[repr(C)]`, so that is
// exactly what it is — and this is what makes the day that stops being true a
// compile error rather than a stream of transposed samples.
const _: () = assert!(std::mem::size_of::<Complex32>() == 8);
const _: () = assert!(std::mem::align_of::<Complex32>() == 4);

/// Timeout for a receive that is allowed to wait. Long enough to be worth
/// asking for, short enough that the engine's loop stays responsive. Matches
/// what the SoapySDR source next door uses for the same call.
pub const RX_TIMEOUT_MS: u32 = 200;

/// Timeout for a transmit write. LimeSuite blocks until its FIFO has room.
const TX_TIMEOUT_MS: u32 = 500;

/// How often to ask LimeSuite whether the stream is still running.
const STATUS_INTERVAL: Duration = Duration::from_secs(2);

pub struct LimeHandle {
    /// The device, shared because the LimeRFE's board link bit-bangs I²C on
    /// this same device's GPIO pins from its own thread. The boundary is
    /// exactly LimeSuite's own: a call taking an `lms_device_t*` goes through
    /// here, and a call taking an `lms_stream_t*` touches only LimeSuite's FIFO
    /// and does not — which is why the receive path never takes this lock.
    ctl: Arc<Mutex<DevCtl>>,
    /// The library, held directly so the streaming calls need no lock.
    api: Arc<ffi::Api>,
    rx: ffi::StreamT,
    tx: Option<ffi::StreamT>,
    /// The board's other receive chain, when it has been given a job — see
    /// [`crate::aux`]. `None` on a one-chain board, and on a two-chain board
    /// whose second chain is set to do nothing, which is the default.
    aux: Option<AuxRx>,
    rx_running: bool,
    tx_running: bool,

    info: DevInfo,
    label: String,
    rate: f64,
    center: f64,
    tx_center: f64,
    /// The receive filter actually in force — on HF this is wider than asked;
    /// see [`device::effective_lpf_bw`].
    analog_bw: f64,
    /// The receive filter width the operator (or the automatic choice) asked
    /// for, kept so a retune across 30 MHz can recompute what to program.
    lpf_rx_want: f64,
    lpf_tx_want: f64,
    /// The transmit filter actually in force, compared against on every
    /// key-down so the slow retune only happens when the answer changes.
    tx_lpf_applied: f64,
    /// The filter ranges, read once — `set_center_hz` is the panadapter's drag
    /// path and should not make even a cheap FFI call it does not need.
    lpf_range_rx: ffi::Range,
    lpf_range_tx: ffi::Range,

    antennas_rx: Vec<String>,
    antennas_tx: Vec<String>,
    antenna_rx: String,
    antenna_tx: String,
    rx_gain_db: f64,
    tx_gain_db: f64,

    cfg: LimeConfig,
    last_status: Instant,
    overruns: u64,
    underruns: u64,
    restarts: u64,
    /// Set when a stream was found stopped and put back. Reported once through
    /// `open_status` rather than every tick.
    note: Option<String>,
    /// Set when the chip's own DC/IQ calibration would not run. Worth its own
    /// field rather than folding into `note`: it is the standing explanation
    /// for a carrier in the middle of the span and an image across the band,
    /// and it is answered by a button in the settings panel.
    cal_note: Option<String>,
    /// Set once [`LimeHandle::close`] has run: the streams are destroyed and
    /// the device is closed, so nothing here may touch either again.
    closed: bool,
    /// This session's diagnostic trace, shared with [`DevCtl`]. Held here as
    /// well because the stream and transmit calls do not go through the device
    /// lock and so cannot reach it that way.
    trace: Trace,
    /// The last transmit path described in the log, so an FT8 station keying
    /// every fifteen seconds says it once rather than four times a minute. Any
    /// change — band, port, drive — says it again.
    last_tx_summary: String,
    /// What this over actually handed the transmitter: samples offered, samples
    /// LimeSuite took, and the loudest one of them. Reset at every key-down and
    /// reported at unkey — see [`LimeHandle::tx_end`] for why a transmitter
    /// that answers every command still needs to say this.
    tx_offered: u64,
    tx_sent: u64,
    tx_peak: f32,
    /// Set while the selected transmit socket does not cover the dial. Its own
    /// field rather than folded into `note`, for the same reason `cal_note`
    /// has one: it is a standing condition with a standing answer, and it is
    /// the whole explanation for a transmitter that reads correct everywhere
    /// and makes no power. See [`LimeHandle::refresh_tx_port_note`].
    tx_port_note: Option<String>,
    /// The frequency `tx_port_note` was written for, so a retune that stays on
    /// the wrong side of the boundary does not rebuild the sentence on every
    /// frame of a panadapter drag.
    tx_port_note_hz: f64,
}

impl LimeHandle {
    /// The device, locked. Poisoning is recovered from rather than propagated:
    /// a panic on another thread mid-transaction leaves the radio in an unknown
    /// state, but refusing to talk to it afterwards helps nobody.
    fn ctl(&self) -> MutexGuard<'_, DevCtl> {
        self.ctl.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// A handle on the device for the LimeRFE's board link, which drives its
    /// I²C through these same GPIO pins.
    pub fn shared_device(&self) -> Arc<Mutex<DevCtl>> {
        Arc::clone(&self.ctl)
    }

    /// Open, configure and start receiving.
    ///
    /// The order is not arbitrary: `LMS_Init` first because it overwrites
    /// everything, then the rate (which reprograms the clock tree every later
    /// setting depends on), then the analog filter, then the synthesiser, then
    /// gains and ports, then calibration, then the streams.
    pub fn open(cfg: &LimeConfig, center_hz: f64) -> Result<LimeHandle> {
        // Remembered before the first call, not after the last one: an open
        // that fails half way is precisely the session worth reporting, and it
        // never reaches the bottom of this function. The trace is shared, so
        // everything recorded from here on lands in what was remembered.
        let trace = Trace::new();
        trace::remember(&trace);
        let (api, dev, listed) = match crate::api::open(&cfg.device) {
            Ok(v) => v,
            Err(e) => {
                let want =
                    if cfg.device.trim().is_empty() { "(first found)" } else { cfg.device.trim() };
                trace.call("LMS_Open", want, format!("FAILED: {e}"));
                return Err(e);
            }
        };
        trace.set_identity(format!("LimeSuite {}, {}", api.version(), listed.label()));
        trace.call("LMS_Open", listed.info.as_str(), "ok");
        let channel = usize::from(cfg.channel);
        let mut ctl = DevCtl::new(Arc::clone(&api), dev, channel, trace.clone());

        let n_rx = ctl.num_channels(false);
        if channel >= n_rx {
            return Err(Error::NotFound(format!(
                "{} has {n_rx} receive chain(s) — {}; chain {} was asked for",
                listed.label(),
                (0..n_rx).map(|c| format!("RX{}_*", c + 1)).collect::<Vec<_>>().join(" and "),
                channel + 1
            )));
        }
        let want_tx = cfg.tx_enabled && ctl.num_channels(true) > channel;

        ctl.init()?;
        ctl.enable_channel(false, true)?;
        if want_tx {
            ctl.enable_channel(true, true)?;
        }

        // The rate reprograms the clock tree that the synthesiser and the
        // filters are both derived from, so it goes before either.
        ctl.set_sample_rate(cfg.sample_rate_hz, cfg.oversample)?;
        let rate = ctl.sample_rate(false).unwrap_or(cfg.sample_rate_hz);

        let lpf_range =
            ctl.lpf_range(false).unwrap_or(ffi::Range { min: 0.0, max: 0.0, step: 0.0 });
        let lpf_rx_want =
            if cfg.lpf_rx_hz > 0.0 { cfg.lpf_rx_hz } else { device::auto_lpf_bw(rate, lpf_range) };
        let analog_bw = device::effective_lpf_bw(lpf_rx_want, center_hz, rate, lpf_range);
        if analog_bw > lpf_rx_want {
            tracing::info!(
                "below 30 MHz the signal rides at the NCO offset inside the analog chain, so \
                 the receive filter opens to {:.1} MHz (instead of {:.1} MHz)",
                analog_bw / 1e6,
                lpf_rx_want / 1e6
            );
        }
        ctl.set_lpf_bw(false, analog_bw)?;

        ctl.set_lo(false, center_hz)?;

        let antennas_rx = ctl.antennas(false);
        let antennas_tx = if want_tx { ctl.antennas(true) } else { Vec::new() };
        let has_rfe = cfg.rfe.link != sdroxide_types::RfeLink::Off;
        let antenna_rx = if cfg.antenna_rx.trim().is_empty() {
            device::auto_antenna_rx(center_hz, &antennas_rx, has_rfe).unwrap_or_default()
        } else {
            cfg.antenna_rx.clone()
        };
        if !antenna_rx.is_empty() {
            ctl.set_antenna_named(false, &antenna_rx)?;
        }
        ctl.set_gain_db(false, cfg.rx_gain_db)?;

        let mut antenna_tx = String::new();
        let lpf_range_tx = ctl.lpf_range(true).unwrap_or(lpf_range);
        let lpf_tx_want = if cfg.lpf_tx_hz > 0.0 {
            cfg.lpf_tx_hz
        } else {
            device::auto_lpf_bw(rate, lpf_range_tx)
        };
        let mut tx_lpf_applied = 0.0;
        if want_tx {
            // Same 30 MHz rule as the receive filter above — this one is the
            // whole difference between full power and milliwatts on HF.
            let tx_bw = device::effective_lpf_bw(lpf_tx_want, center_hz, rate, lpf_range_tx);
            if ctl.set_lpf_bw(true, tx_bw).is_ok() {
                tx_lpf_applied = tx_bw;
            }
            antenna_tx = if cfg.antenna_tx.trim().is_empty() {
                device::auto_antenna_tx(center_hz, &antennas_tx).unwrap_or_default()
            } else {
                cfg.antenna_tx.clone()
            };
            if !antenna_tx.is_empty() {
                ctl.set_antenna_named(true, &antenna_tx)?;
            }
            ctl.set_gain_db(true, cfg.tx_gain_db)?;
            ctl.set_lo(true, center_hz)?;
        }

        let mut cal_note = None;
        if cfg.calibrate {
            // Best-effort: an uncalibrated radio still receives, and refusing
            // to open because the calibration would not converge would be a
            // poor trade. But it is *said*, on screen and not only in the log —
            // an uncorrected zero-IF front end puts a carrier in the middle of
            // the span and an image across the band, and an operator looking at
            // those deserves to be told which of the two possible causes it is
            // (issue #94).
            //
            // Calibrated for the *wanted* width, not the NCO-widened filter:
            // the span the operator uses is what the DC and image corrections
            // should be best over.
            if let Err(e) = ctl.calibrate(false, lpf_rx_want) {
                tracing::warn!("LimeSDR receive calibration failed, continuing: {e}");
                cal_note = Some(format!(
                    "the LimeSDR's own DC-offset and image calibration would not run ({e}), \
                     so the receiver is uncorrected — expect a carrier at the centre of \
                     the span and a mirror image of every signal. Try again with Calibrate \
                     now in Settings → Radio."
                ));
            }
            if want_tx && let Err(e) = ctl.calibrate(true, lpf_tx_want) {
                tracing::warn!("LimeSDR transmit calibration failed, continuing: {e}");
            }
            // LimeSuite's calibration drives the chip's own test tone through a
            // loopback and reprograms the receive chain to hear it. It restores
            // what it changed when it succeeds; a run that stopped half way is
            // exactly the case where that is least certain, so the three
            // settings that decide whether anything is heard at all are put
            // back by hand.
            if !antenna_rx.is_empty() {
                let _ = ctl.set_antenna_named(false, &antenna_rx);
            }
            let _ = ctl.set_gain_db(false, cfg.rx_gain_db);
            let _ = ctl.set_lpf_bw(false, analog_bw);
            // And the transmit chain, for exactly the same reason. It was
            // missing here: the calibration drives the chip's own test tone
            // through a loopback in *both* directions, and a transmit path
            // left on the wrong band or at the wrong gain by a run that
            // stopped half way is a radio that receives perfectly and puts
            // nothing on the air — with no error anywhere to say so.
            if want_tx {
                if !antenna_tx.is_empty() {
                    let _ = ctl.set_antenna_named(true, &antenna_tx);
                }
                let _ = ctl.set_gain_db(true, cfg.tx_gain_db);
                if tx_lpf_applied > 0.0 {
                    let _ = ctl.set_lpf_bw(true, tx_lpf_applied);
                }
                let _ = ctl.set_lo(true, center_hz);
            }
        }

        // Both streams while the device is idle — see the module doc.
        let mut rx = ffi::StreamT {
            handle: 0,
            is_tx: false,
            channel: cfg.channel as u32,
            fifo_size: cfg.fifo_ksamples.max(16) * 1024,
            throughput_vs_latency: cfg.throughput_vs_latency.clamp(0.0, 1.0),
            data_fmt: ffi::FMT_F32,
            // 12 bits per component over the link: three bytes a sample
            // instead of four, and nothing is lost because the converters are
            // 12-bit to begin with.
            link_fmt: ffi::LINK_FMT_I12,
        };
        let rc = unsafe { (api.setup_stream)(dev, &mut rx) };
        if rc != ffi::OK {
            let text = api.err_text();
            trace.call("LMS_SetupStream", "receive", format!("FAILED: {text}"));
            return Err(Error::api("LMS_SetupStream", text));
        }
        trace.call(
            "LMS_SetupStream",
            format!("receive ch{}, FIFO {}k", cfg.channel + 1, cfg.fifo_ksamples.max(16)),
            "ok",
        );
        // The second chain, if it has been given a job. Best-effort throughout:
        // an operator who came to listen must not lose their receiver because
        // the diversity aerial's chain would not start, so every failure here
        // is a note and a `None`.
        let mut aux_note = None;
        let aux = if cfg.aux.role == LimeAuxRole::Off {
            None
        } else {
            match Self::open_aux(
                &api,
                dev,
                &mut ctl,
                cfg,
                n_rx,
                center_hz,
                analog_bw,
                lpf_rx_want,
                &rx,
            ) {
                Ok(a) => Some(a),
                Err(e) => {
                    tracing::warn!("the LimeSDR's second receive chain was not opened: {e}");
                    aux_note = Some(format!(
                        "the second receive chain was not opened, so there is no diversity: {e}"
                    ));
                    None
                }
            }
        };
        let mut tx = None;
        if want_tx {
            let mut s = ffi::StreamT { is_tx: true, ..rx };
            s.handle = 0;
            let rc = unsafe { (api.setup_stream)(dev, &mut s) };
            if rc != ffi::OK {
                let text = api.err_text();
                trace.call("LMS_SetupStream", "transmit", format!("FAILED: {text}"));
                unsafe { (api.destroy_stream)(dev, &mut rx) };
                return Err(Error::api("LMS_SetupStream (tx)", text));
            }
            trace.call("LMS_SetupStream", format!("transmit ch{}", cfg.channel + 1), "ok");
            tx = Some(s);
        }

        let rc = unsafe { (api.start_stream)(&mut rx) };
        trace.call(
            "LMS_StartStream",
            "receive",
            if rc == ffi::OK { "ok".to_string() } else { format!("FAILED: {}", api.err_text()) },
        );
        if rc != ffi::OK {
            let text = api.err_text();
            unsafe { (api.destroy_stream)(dev, &mut rx) };
            if let Some(mut s) = tx {
                unsafe { (api.destroy_stream)(dev, &mut s) };
            }
            if let Some(mut a) = aux {
                unsafe { (api.destroy_stream)(dev, &mut a.stream) };
            }
            return Err(Error::api("LMS_StartStream", text));
        }
        let mut aux = aux;
        if let Some(a) = aux.as_mut() {
            let rc = unsafe { (api.start_stream)(&mut a.stream) };
            if rc == ffi::OK {
                a.running = true;
            } else {
                // Same rule as opening it: the main receiver stands.
                let text = api.err_text();
                tracing::warn!("the LimeSDR's second receive chain would not start: {text}");
                aux_note = Some(format!(
                    "the second receive chain would not start, so there is no \
                                  diversity: {text}"
                ));
                unsafe { (api.destroy_stream)(dev, &mut a.stream) };
                aux = None;
            }
        }

        let info = ctl.info();
        let label = if info.name.is_empty() { listed.label() } else { info.name.clone() };
        trace.set_identity(format!(
            "LimeSuite {}, {label} serial {} (firmware {}, hardware {}, gateware {})",
            api.version(),
            info.serial,
            info.firmware,
            info.hardware,
            info.gateware
        ));
        let rx_gain_db = ctl.gain_db(false).unwrap_or(cfg.rx_gain_db);
        let tx_gain_db = ctl.gain_db(true).unwrap_or(cfg.tx_gain_db);

        tracing::info!(
            "LimeSDR ready: {label} (firmware {}, gateware {}), {:.3} Msps, filter {:.2} MHz, \
             centre {center_hz:.0} Hz, gain {rx_gain_db} dB, receiving on {}{}",
            info.firmware,
            info.gateware,
            rate / 1e6,
            analog_bw / 1e6,
            // The socket, not the chip's port name: `LNAL` is the same word on
            // both chains and the operator has one aerial in one connector.
            LimeConfig::port_label(cfg.channel, &antenna_rx, false),
            if want_tx { ", transmitter armed" } else { "" }
        );

        let mut handle = LimeHandle {
            ctl: Arc::new(Mutex::new(ctl)),
            api: Arc::clone(&api),
            rx,
            tx,
            aux,
            rx_running: true,
            tx_running: false,
            info,
            label,
            rate,
            center: center_hz,
            tx_center: center_hz,
            analog_bw,
            lpf_rx_want,
            lpf_tx_want,
            tx_lpf_applied,
            lpf_range_rx: lpf_range,
            lpf_range_tx,
            antennas_rx,
            antennas_tx,
            antenna_rx,
            antenna_tx,
            rx_gain_db,
            tx_gain_db,
            cfg: cfg.clone(),
            last_status: Instant::now(),
            overruns: 0,
            underruns: 0,
            restarts: 0,
            note: aux_note,
            cal_note,
            closed: false,
            trace,
            last_tx_summary: String::new(),
            tx_offered: 0,
            tx_sent: 0,
            tx_peak: 0.0,
            tx_port_note: None,
            tx_port_note_hz: f64::NAN,
        };
        handle.refresh_tx_port_note(center_hz);
        Ok(handle)
    }

    /// Keep the standing "this over cannot leave the board" note in step with
    /// the dial.
    ///
    /// The two transmit sockets are two matching networks — see
    /// [`LimeConfig::tx_port_band`] — and a board on the wrong one transmits
    /// into a filter that passes almost none of the band. Nothing else in the
    /// session notices: the synthesiser is on frequency, the drive is whatever
    /// was asked for, the stream runs, every call answers `ok`, and the power
    /// meter reads zero. That is issue #94, and it took three releases to find
    /// because the only place the socket appeared was a log line that stated it
    /// without judging it.
    ///
    /// Only touched when the answer changes, because this runs on every retune
    /// — which on a panadapter drag is every frame.
    fn refresh_tx_port_note(&mut self, hz: f64) {
        let bad = self.tx.is_some() && !LimeConfig::tx_port_covers(&self.antenna_tx, hz);
        if !bad {
            self.tx_port_note = None;
            return;
        }
        if self.tx_port_note.is_some() && (hz - self.tx_port_note_hz).abs() < 1.0 {
            return;
        }
        self.tx_port_note_hz = hz;
        let (lo, hi) = LimeConfig::tx_port_band(&self.antenna_tx).unwrap_or_default();
        self.tx_port_note = Some(format!(
            "the transmit port is {}, which this board matches for {:.0}–{:.0} MHz — \
             {:.3} MHz is outside it, so an over here will read as nothing on a power meter \
             however much drive is set. Set Transmit port to Automatic in Settings → Radio, \
             or pick the socket that covers this band.",
            LimeConfig::port_label(self.cfg.channel, &self.antenna_tx, true),
            lo / 1e6,
            hi / 1e6,
            hz / 1e6,
        ));
    }

    /// Configure the board's other receive chain and create its stream.
    ///
    /// The chain is set up to be **as much like the first as possible** — same
    /// analog filter, same calibration width, and by default the same kind of
    /// port — because everything the two chains do differently is something the
    /// adaptive filter has to equalise before it can cancel anything. The gain
    /// is the exception, and is deliberately the operator's: the two aerials
    /// are not the same aerial, and matching the noise floors is the whole
    /// setup procedure.
    ///
    /// The stream mirrors the main one's FIFO depth and formats, so the two
    /// FIFOs drain at the same rate and neither runs away from the other.
    #[allow(clippy::too_many_arguments)]
    fn open_aux(
        api: &Arc<ffi::Api>,
        dev: ffi::Device,
        ctl: &mut DevCtl,
        cfg: &LimeConfig,
        n_rx: usize,
        center_hz: f64,
        analog_bw: f64,
        cal_bw: f64,
        main_rx: &ffi::StreamT,
    ) -> Result<AuxRx> {
        let ch = usize::from(cfg.aux_channel());
        if ch >= n_rx {
            return Err(Error::NotFound(format!(
                "this board has {n_rx} receive chain(s), so there is no second one to put an \
                 aerial on"
            )));
        }
        ctl.enable_channel_on(false, ch, true)?;
        // Same passband as the main chain: a different filter width is a
        // different phase response, and a phase response the adaptive filter
        // has to undo is taps spent on the radio rather than on the aerials.
        ctl.set_lpf_bw_on(false, ch, analog_bw)?;
        let ports = ctl.antennas_on(false, ch);
        let want = if cfg.aux.antenna.trim().is_empty() {
            // The same rule the main chain follows, which on a bare board
            // lands on the same kind of socket: `LNAL` on chain 0 is `RX1_L`
            // and on chain 1 is `RX2_L`, the pair issue #98 names.
            device::auto_antenna_rx(center_hz, &ports, false).unwrap_or_default()
        } else {
            cfg.aux.antenna.clone()
        };
        if !want.is_empty() {
            ctl.set_antenna_named_on(false, ch, &want)?;
        }
        ctl.set_gain_db_on(false, ch, cfg.aux.gain_db)?;
        if cfg.calibrate {
            // Best-effort, exactly as the main chain's is: an uncalibrated
            // second chain still cancels, it merely leaves a DC offset of its
            // own for the filter to account for.
            if let Err(e) = ctl.calibrate_on(false, ch, cal_bw) {
                tracing::warn!("the second receive chain would not calibrate, continuing: {e}");
            }
        }
        let mut stream = ffi::StreamT { channel: ch as u32, handle: 0, ..*main_rx };
        let rc = unsafe { (api.setup_stream)(dev, &mut stream) };
        if rc != ffi::OK {
            return Err(Error::api("LMS_SetupStream (aux)", api.err_text()));
        }
        tracing::info!(
            "the LimeSDR's second receive chain is on {}, gain {} dB",
            LimeConfig::port_label(cfg.aux_channel(), &want, false),
            cfg.aux.gain_db
        );
        Ok(AuxRx::new(stream, ch, want, cfg.aux.gain_db))
    }

    /// Refuse a control call on a handle [`Self::close`] has already been
    /// through. The engine keeps a released source callable while the
    /// replacement is opened, so this is an answer, not an assertion.
    fn ensure_open(&self) -> Result<()> {
        if self.closed { Err(Error::Closed) } else { Ok(()) }
    }

    pub fn label(&self) -> &str {
        &self.label
    }
    pub fn info(&self) -> &DevInfo {
        &self.info
    }
    pub fn sample_rate(&self) -> f64 {
        self.rate
    }
    pub fn analog_bw(&self) -> f64 {
        self.analog_bw
    }
    pub fn center_hz(&self) -> f64 {
        self.center
    }
    pub fn antennas_rx(&self) -> &[String] {
        &self.antennas_rx
    }
    pub fn antennas_tx(&self) -> &[String] {
        &self.antennas_tx
    }
    pub fn antenna_rx(&self) -> &str {
        &self.antenna_rx
    }
    /// Which of the board's receive chains this session is on, counted from
    /// zero. The chain decides the socket a port name reaches — see
    /// [`LimeConfig::port_label`].
    pub fn channel(&self) -> u8 {
        self.cfg.channel
    }
    /// The receive port with its board socket beside it, for a log line or a
    /// status note: `LNAL — RX2_L`.
    pub fn rx_socket_label(&self) -> String {
        LimeConfig::port_label(self.cfg.channel, &self.antenna_rx, false)
    }
    pub fn antenna_tx(&self) -> &str {
        &self.antenna_tx
    }
    pub fn rx_gain_db(&self) -> f64 {
        self.rx_gain_db
    }
    pub fn tx_gain_db(&self) -> f64 {
        self.tx_gain_db
    }
    pub fn can_tx(&self) -> bool {
        self.tx.is_some()
    }
    pub fn chip_temp_c(&self) -> Option<f64> {
        if self.closed {
            return None;
        }
        self.ctl().chip_temp_c()
    }
    pub fn lo_range(&self, tx: bool) -> Result<ffi::Range> {
        self.ensure_open()?;
        self.ctl().lo_range(tx)
    }
    pub fn rate_range(&self, tx: bool) -> Result<ffi::Range> {
        self.ensure_open()?;
        self.ctl().rate_range(tx)
    }

    /// Retune the receive synthesiser.
    ///
    /// Deliberately *not* recalibrating: `LMS_Calibrate` costs hundreds of
    /// milliseconds and this is called from the engine's loop every time the
    /// operator drags the panadapter past the edge of the span. The calibration
    /// from open remains good across a retune of ordinary size; a band change
    /// can be recalibrated explicitly from the settings panel.
    pub fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.ensure_open()?;
        self.ctl().set_lo(false, hz)?;
        self.center = hz;
        // A port chosen automatically follows the frequency, because LNAL and
        // LNAH are wired to different pins and the wrong one is deaf rather
        // than merely worse. With a LimeRFE in front the answer is the same at
        // every frequency and this never fires — see `device::auto_antenna_rx`.
        let has_rfe = self.cfg.rfe.link != sdroxide_types::RfeLink::Off;
        if self.cfg.antenna_rx.trim().is_empty()
            && let Some(want) = device::auto_antenna_rx(hz, &self.antennas_rx, has_rfe)
            && want != self.antenna_rx
        {
            self.ctl().set_antenna_named(false, &want)?;
            self.antenna_rx = want;
        }
        // The transmit port follows the dial for the same reason, and this is
        // where it has to happen: it used to be chosen once at open and then
        // left, so a session that started on 2 m and moved to 13 cm went on
        // transmitting into the low-band matching network — and one that
        // started on 13 cm kept the microwave port for every band below it.
        // The two sockets are two matching networks, not two jacks onto one;
        // see `device::tx_port_band`. Only while no port has been named, the
        // same rule the receive side follows.
        if self.tx.is_some()
            && self.cfg.antenna_tx.trim().is_empty()
            && let Some(want) = device::auto_antenna_tx(hz, &self.antennas_tx)
            && want != self.antenna_tx
        {
            // The guard is dropped before the field is written: the device
            // lock and a borrow of `self` must not overlap.
            let moved = self.ctl().set_antenna_named(true, &want);
            match moved {
                Ok(()) => {
                    tracing::info!("LimeSDR transmit port following the dial to {want}");
                    self.antenna_tx = want;
                }
                // Not fatal to a retune: the operator is tuning, and the next
                // key-down says loudly what socket the over is going out of.
                Err(e) => tracing::warn!("the LimeSDR transmit port did not follow the tune: {e}"),
            }
        }
        self.refresh_tx_port_note(hz);
        // Crossing 30 MHz changes which side of the NCO trick the filter has
        // to serve (see `device::effective_lpf_bw`). The answer is constant on
        // each side, so this slow call fires only on the crossing itself —
        // never while dragging around within a band. Best-effort: a tune that
        // succeeded is not refused because the filter would not follow.
        let bw = device::effective_lpf_bw(self.lpf_rx_want, hz, self.rate, self.lpf_range_rx);
        if (bw - self.analog_bw).abs() > 1.0 {
            let retuned = self.ctl().set_lpf_bw(false, bw);
            match retuned {
                Ok(()) => {
                    tracing::info!(
                        "receive filter retuned to {:.1} MHz for the 30 MHz crossing",
                        bw / 1e6
                    );
                    self.analog_bw = bw;
                    // LimeSuite's filter tuning moves the receive gain stages
                    // and does not put them back (its `SetLPF` preserves only
                    // the transmit IAMP).
                    let _ = self.ctl().set_gain_db(false, self.rx_gain_db);
                }
                Err(e) => tracing::warn!("receive filter did not follow the tune: {e}"),
            }
        }
        Ok(())
    }

    pub fn set_gain_db(&mut self, tx: bool, db: f64) -> Result<()> {
        self.ensure_open()?;
        self.ctl().set_gain_db(tx, db)?;
        // Read back rather than storing the request: LimeSuite takes an
        // integer, so what the chip got is not always what was asked for, and
        // the panel should show the truth.
        let applied = self.ctl().gain_db(tx).unwrap_or(db);
        if tx {
            self.tx_gain_db = applied;
        } else {
            self.rx_gain_db = applied;
        }
        Ok(())
    }

    /// Move to a named port, and stop choosing one automatically.
    ///
    /// The pinning is the point. `set_center_hz` re-runs the automatic choice
    /// only while no port has been named, so without this a socket the operator
    /// picked by hand — the LimeRFE's, most of the time — would be silently put
    /// back on the next retune, and the control would look like it did nothing.
    pub fn set_antenna(&mut self, tx: bool, name: &str) -> Result<()> {
        self.ensure_open()?;
        self.ctl().set_antenna_named(tx, name)?;
        if tx {
            self.antenna_tx = name.to_string();
            self.cfg.antenna_tx = name.to_string();
            // A port picked by hand is exactly where this goes wrong, so the
            // note is answered at the moment it is chosen rather than at the
            // next key-down.
            self.tx_port_note = None;
            let center = self.center;
            self.refresh_tx_port_note(center);
        } else {
            self.antenna_rx = name.to_string();
            self.cfg.antenna_rx = name.to_string();
        }
        Ok(())
    }

    /// Take whatever the second chain has, unpaired — see
    /// [`crate::auxrx::AuxRx::read_raw`]. Never blocks.
    pub fn read_aux_raw(&mut self, out: &mut [Complex32]) -> usize {
        if self.closed {
            return 0;
        }
        let api: &ffi::Api = &self.api;
        match self.aux.as_mut() {
            Some(a) => a.read_raw(api, out),
            None => 0,
        }
    }

    /// The transmit synthesiser's frequency, as `tx_begin` last set it. The
    /// predistortion loop needs it: the feedback arrives at the *receive*
    /// centre, so the difference is how far off the middle of the span it
    /// lands.
    pub fn tx_center_hz(&self) -> f64 {
        self.tx_center
    }

    /// Whether the second receive chain is running and pairing.
    pub fn aux_active(&self) -> bool {
        self.aux.as_ref().is_some_and(|a| a.running)
    }

    /// The second chain's port with its socket beside it, for a log line.
    pub fn aux_socket_label(&self) -> Option<String> {
        self.aux.as_ref().map(|a| LimeConfig::port_label(self.cfg.aux_channel(), &a.antenna, false))
    }

    /// How many times the pairing has had to be abandoned and restarted.
    /// Steadily climbing means the host is not keeping up with two chains at
    /// this sample rate.
    pub fn aux_slips(&self) -> u64 {
        self.aux.as_ref().map_or(0, |a| a.slips())
    }

    /// Whether the second chain went a whole block without a pairable sample.
    pub fn aux_stalled(&self) -> bool {
        self.aux.as_ref().is_some_and(|a| a.stalled)
    }

    /// Whether this LimeSuite stamps its receive blocks with the hardware
    /// sample counter. Where it does not, the two chains are paired by arrival
    /// order alone — which works, but has nothing to notice a dropped packet
    /// with, so it is worth saying once.
    pub fn aux_timestamped(&self) -> bool {
        self.aux.as_ref().is_none_or(|a| a.stamped())
    }

    pub fn aux_gain_db(&self) -> f64 {
        self.aux.as_ref().map_or(0.0, |a| a.gain_db)
    }

    /// Move the second chain's gain. The setting that matches the two noise
    /// floors, which is what both diversity modes are built on.
    pub fn set_aux_gain_db(&mut self, db: f64) -> Result<()> {
        self.ensure_open()?;
        let Some(ch) = self.aux.as_ref().map(|a| a.channel) else { return Ok(()) };
        self.ctl().set_gain_db_on(false, ch, db)?;
        // Read back from the chain that was set — LimeSuite takes whole
        // decibels, so what the chip got is not always what was asked for.
        let applied = self.ctl().gain_db_on(false, ch).unwrap_or(db);
        if let Some(a) = self.aux.as_mut() {
            a.gain_db = applied;
        }
        self.cfg.aux.gain_db = applied;
        Ok(())
    }

    /// Move the second chain to a named port, immediately — the same reasoning
    /// as the main chain's: which socket the aerial is in is exactly what an
    /// operator changes while listening.
    pub fn set_aux_antenna(&mut self, name: &str) -> Result<()> {
        self.ensure_open()?;
        let Some(ch) = self.aux.as_ref().map(|a| a.channel) else { return Ok(()) };
        self.ctl().set_antenna_named_on(false, ch, name)?;
        if let Some(a) = self.aux.as_mut() {
            a.antenna = name.to_string();
        }
        self.cfg.aux.antenna = name.to_string();
        Ok(())
    }

    pub fn set_lpf_bw(&mut self, tx: bool, hz: f64) -> Result<()> {
        self.ensure_open()?;
        let range = if tx { self.lpf_range_tx } else { self.lpf_range_rx };
        let want = if hz > 0.0 { hz } else { device::auto_lpf_bw(self.rate, range) };
        // The 30 MHz floor applies to the operator's number too: a hand-set
        // 2.5 MHz filter under a 14 MHz dial is a transmitter at milliwatts
        // and a half-deaf receiver, which nobody has ever meant.
        let center = if tx { self.tx_center } else { self.center };
        let bw = device::effective_lpf_bw(want, center, self.rate, range);
        if bw > want {
            tracing::info!(
                "the {} filter opens to {:.1} MHz (asked {:.1} MHz): below 30 MHz the signal \
                 rides at the NCO offset inside the analog chain",
                if tx { "transmit" } else { "receive" },
                bw / 1e6,
                want / 1e6
            );
        }
        self.ctl().set_lpf_bw(tx, bw)?;
        if tx {
            self.lpf_tx_want = want;
            self.tx_lpf_applied = bw;
        } else {
            self.lpf_rx_want = want;
            self.analog_bw = bw;
            // See `set_center_hz`: the filter tuning moves the gain stages.
            let _ = self.ctl().set_gain_db(false, self.rx_gain_db);
        }
        Ok(())
    }

    /// Run LimeSuite's calibration now. Stalls whatever thread calls it for the
    /// better part of a second — see `LimeSource::recalibrate_if_due` for when
    /// that is allowed to happen on its own.
    pub fn calibrate(&mut self) -> Result<()> {
        self.ensure_open()?;
        // The wanted widths, not the NCO-widened filters: the span the
        // operator uses is what the corrections should be best over.
        let bw = self.lpf_rx_want;
        let rx = self.ctl().calibrate(false, bw);
        let tx = if self.tx.is_some() {
            let tx_bw = self.lpf_tx_want;
            self.ctl().calibrate(true, tx_bw)
        } else {
            Ok(())
        };
        // Whatever happened above, put both chains back where they were — the
        // calibration reprograms them to hear the chip's own test tone through
        // a loopback, and a run that stopped part way is the case where its own
        // restore is least to be relied on. The transmit half matters just as
        // much as the receive one and is harder to notice: a port or a gain
        // left somewhere else is a radio that still receives and puts nothing
        // on the air.
        let (antenna, gain, analog) = (self.antenna_rx.clone(), self.rx_gain_db, self.analog_bw);
        if !antenna.is_empty() {
            let _ = self.ctl().set_antenna_named(false, &antenna);
        }
        let _ = self.ctl().set_gain_db(false, gain);
        let _ = self.ctl().set_lpf_bw(false, analog);
        if self.tx.is_some() {
            let (port, drive, filter, center) =
                (self.antenna_tx.clone(), self.tx_gain_db, self.tx_lpf_applied, self.tx_center);
            if !port.is_empty() {
                let _ = self.ctl().set_antenna_named(true, &port);
            }
            let _ = self.ctl().set_gain_db(true, drive);
            if filter > 0.0 {
                let _ = self.ctl().set_lpf_bw(true, filter);
            }
            let _ = self.ctl().set_lo(true, center);
        }
        rx?;
        tx?;
        // A calibration that ran is the answer to the note left at open.
        self.cal_note = None;
        Ok(())
    }

    /// Read what is there, waiting up to `timeout_ms` for it.
    ///
    /// `Ok(0)` on a timeout, which is the trait's contract: the caller retries.
    pub fn read_within(&mut self, buf: &mut [Complex32], timeout_ms: u32) -> Result<usize> {
        Ok(self.read_pair(buf, &mut [], timeout_ms)?.0)
    }

    /// Read the main chain, and the same samples of the second one beside it.
    ///
    /// Returns `(main, aux)`. `aux` is either `main` — the two blocks are the
    /// same span of time, sample for sample — or **zero**, which is not a
    /// failure: it means the pair could not be aligned this block (or there is
    /// no second chain), and combining is skipped rather than done against the
    /// wrong samples. See [`crate::aux`] for what the alignment is up against.
    ///
    /// `aux_out` may be empty, which asks for the main chain alone and costs
    /// nothing extra.
    pub fn read_pair(
        &mut self,
        buf: &mut [Complex32],
        aux_out: &mut [Complex32],
        timeout_ms: u32,
    ) -> Result<(usize, usize)> {
        if !self.rx_running || buf.is_empty() {
            return Ok((0, 0));
        }
        // The timestamp is only wanted when there is a second chain to line up
        // against it, so the ordinary single-chain read makes exactly the call
        // it always made.
        let mut meta = ffi::StreamMetaT::default();
        let want_aux = self.aux.is_some() && !aux_out.is_empty();
        let meta_ptr =
            if want_aux { &raw mut meta } else { std::ptr::null_mut::<ffi::StreamMetaT>() };
        // `Complex<f32>` is `#[repr(C)]`, so interleaved f32 I/Q *is* this
        // slice's memory — no conversion and no scratch buffer. Pinned by the
        // assert below.
        let n = unsafe {
            (self.api.recv_stream)(
                &mut self.rx,
                buf.as_mut_ptr().cast(),
                buf.len(),
                meta_ptr,
                timeout_ms,
            )
        };
        if n < 0 {
            return Err(Error::api("LMS_RecvStream", self.api.err_text()));
        }
        let n = n as usize;
        let mut got_aux = 0;
        if want_aux && n > 0 {
            // Disjoint fields: the library is read while the second chain is
            // written, which is why this is not one method call on `self`.
            let api: &ffi::Api = &self.api;
            if let Some(a) = self.aux.as_mut() {
                let take = n.min(aux_out.len());
                // The main read's own timeout says whether this caller may
                // block: zero is `read_available` during an over.
                got_aux = a.read_aligned(api, meta.timestamp, &mut aux_out[..take], timeout_ms > 0);
            }
        }
        self.poll_status();
        Ok((n, got_aux))
    }

    /// Ask LimeSuite how the stream is doing, occasionally.
    ///
    /// The reason this exists rather than being left to fail loudly: LimeSuite
    /// is recorded as stopping a running stream on its own when the chip is
    /// reconfigured — the SoapySDR path next door carries `reassert_gains` for
    /// the same behaviour. A stream that has quietly stopped delivers zeroes
    /// forever otherwise.
    fn poll_status(&mut self) {
        if self.last_status.elapsed() < STATUS_INTERVAL {
            return;
        }
        self.last_status = Instant::now();
        let mut st = ffi::StreamStatusT::default();
        let rc = unsafe { (self.api.get_stream_status)(&mut self.rx, &mut st) };
        if rc != ffi::OK {
            return;
        }
        self.overruns += u64::from(st.overrun);
        self.underruns += u64::from(st.underrun);
        if st.overrun > 0 {
            tracing::debug!("LimeSDR receive overrun ({} samples dropped)", st.overrun);
        }
        if !st.active {
            self.restarts += 1;
            tracing::warn!("LimeSDR receive stream had stopped; restarting it");
            let rc = unsafe { (self.api.start_stream)(&mut self.rx) };
            if rc == ffi::OK {
                // Whatever stopped it also reset the chip's settings, so put
                // them back rather than assuming they survived.
                let _ = self.ctl().set_gain_db(false, self.rx_gain_db);
                let _ = self.ctl().set_lo(false, self.center);
                self.note = Some(format!(
                    "the receive stream stopped and was restarted {} time(s) — if this keeps \
                     happening, try a lower sample rate",
                    self.restarts
                ));
            } else {
                self.rx_running = false;
                self.note = Some(format!(
                    "the receive stream stopped and could not be restarted: {}",
                    self.api.err_text()
                ));
            }
        }
    }

    /// Whether the session has failed badly enough to want reopening.
    pub fn needs_reopen(&self) -> bool {
        !self.rx_running
    }

    /// Stop both streams and close the device *now*, ahead of `Drop`, leaving
    /// the handle inert but callable: reads deliver nothing, controls answer
    /// [`Error::Closed`], and [`Self::needs_reopen`] says yes. Idempotent.
    ///
    /// This is `IqSource::release`'s half of a reopen. The engine builds this
    /// front end's replacement *before* the old source is dropped, and what a
    /// second `LMS_Open` does against a board still held here depends on the
    /// platform — both answers are wrong. On Linux, libusb refuses the second
    /// interface claim, so every Apply failed as "held by another program"
    /// (us). On Windows, CyAPI opens the device *shared*, so the open
    /// succeeded and the replacement's `LMS_Init` and stream setup landed on
    /// top of the running stream — both sessions came out of that dead, which
    /// is how changing the sample rate froze the waterfall until the program
    /// was restarted.
    ///
    /// Ordering contract: a LimeRFE reached through this board's GPIO (see
    /// [`Self::shared_device`]) must be dropped first — its handle keeps a
    /// pointer into the device this closes, and LimeSuite would dereference it.
    pub fn close(&mut self) {
        if self.closed {
            return;
        }
        self.closed = true;
        self.teardown_streams();
        self.ctl().close();
    }

    /// Stop and destroy both streams, in the order that leaves the radio
    /// quiet: stop transmitting, stop receiving, then let go of the streams.
    /// Shared by [`Self::close`] and `Drop`; runs at most once, which the
    /// `closed` flag guards.
    fn teardown_streams(&mut self) {
        let api = Arc::clone(&self.api);
        // The device pointer is read once, before anything borrows a stream:
        // holding the guard across those calls would overlap the two borrows,
        // and the pointer is stable for the life of the device anyway.
        let dev = self.ctl().raw();
        if let Some(tx) = self.tx.as_mut() {
            if self.tx_running {
                unsafe { (api.stop_stream)(tx) };
            }
            unsafe { (api.destroy_stream)(dev, tx) };
        }
        self.tx = None;
        self.tx_running = false;
        if let Some(a) = self.aux.as_mut() {
            if a.running {
                unsafe { (api.stop_stream)(&mut a.stream) };
            }
            unsafe { (api.destroy_stream)(dev, &mut a.stream) };
        }
        self.aux = None;
        if self.rx_running {
            unsafe { (api.stop_stream)(&mut self.rx) };
        }
        unsafe { (api.destroy_stream)(dev, &mut self.rx) };
        self.rx_running = false;
    }

    /// Standing conditions worth telling the operator about.
    pub fn status_note(&self) -> Option<String> {
        let lines: Vec<&str> = [&self.note, &self.cal_note, &self.tx_port_note]
            .into_iter()
            .flatten()
            .map(String::as_str)
            .collect();
        (!lines.is_empty()).then(|| lines.join("\n"))
    }

    /// Start transmitting on `center_hz`. Returns the transmit sample rate.
    pub fn tx_begin(&mut self, center_hz: f64) -> Result<f64> {
        self.ensure_open()?;
        if self.tx.is_none() {
            return Err(Error::api("LMS_StartStream", "the transmitter is not armed".into()));
        }
        // The transmit filter has to serve the right side of the 30 MHz NCO
        // boundary for the frequency this over is on (see
        // `device::effective_lpf_bw` — below it, a rate-derived filter
        // transmits milliwatts). The answer is constant on each side, so the
        // slow retune fires only when a band change actually crossed over;
        // an ordinary key-down compares two numbers and moves on.
        let bw =
            device::effective_lpf_bw(self.lpf_tx_want, center_hz, self.rate, self.lpf_range_tx);
        if (bw - self.tx_lpf_applied).abs() > 1.0 {
            let retuned = self.ctl().set_lpf_bw(true, bw);
            match retuned {
                Ok(()) => {
                    self.tx_lpf_applied = bw;
                    tracing::info!(
                        "transmit filter retuned to {:.1} MHz for the 30 MHz crossing",
                        bw / 1e6
                    );
                }
                // A filter already wider than needed still passes the signal,
                // so an over is not refused because the *narrowing* failed.
                // One too narrow would go out at milliwatts — that over is
                // refused with the reason in hand.
                Err(e) if self.tx_lpf_applied >= bw => {
                    tracing::warn!(
                        "transmit filter stayed at {:.1} MHz: {e}",
                        self.tx_lpf_applied / 1e6
                    );
                }
                Err(e) => return Err(e),
            }
        }
        // Retune before taking hold of the stream: the device lock and the
        // stream borrow must not overlap, and the LO is the device's.
        self.ctl().set_lo(true, center_hz)?;
        self.tx_center = center_hz;
        let Some(tx) = self.tx.as_mut() else { unreachable!("checked above") };
        if !self.tx_running {
            let rc = unsafe { (self.api.start_stream)(tx) };
            if rc != ffi::OK {
                let text = self.api.err_text();
                self.trace.call("LMS_StartStream", "transmit", format!("FAILED: {text}"));
                return Err(Error::api("LMS_StartStream", text));
            }
            // Traced when it *works*, not only when it fails. This is the one
            // call that decides whether the transmitter exists at all, and a
            // trace that says nothing about it leaves "no output on the meter"
            // with no way to tell a stream that never started from one that
            // started and was handed nothing (issue #94).
            self.trace.call("LMS_StartStream", "transmit", "ok");
            self.tx_running = true;
        }
        // A fresh set of books for this over.
        self.tx_offered = 0;
        self.tx_sent = 0;
        self.tx_peak = 0.0;
        self.announce_tx(center_hz);
        // The engine builds its upconverter from this number, so a nonsense
        // answer here is a transmitter that emits nothing with no error
        // anywhere: an interpolator built for 0 Hz produces no samples, and
        // every layer above would go on reporting a healthy over. LimeSuite
        // reads it back off the chip's own registers, which is exactly the
        // read that can come back as zero on a part that is mid-reconfigure —
        // so it is only believed when it is plausible, and the rate this
        // session opened at stands in when it is not.
        let asked = self.ctl().sample_rate(true).unwrap_or(self.rate);
        if !(asked.is_finite() && asked > 0.0) {
            tracing::warn!(
                "the LimeSDR reported a transmit sample rate of {asked}; transmitting at the \
                 session's {:.3} Msps instead",
                self.rate / 1e6
            );
            self.trace.call(
                "LMS_GetSampleRate",
                format!("transmit answered {asked}"),
                format!("using {:.3} Msps", self.rate / 1e6),
            );
            return Ok(self.rate);
        }
        Ok(asked)
    }

    /// Say what this over is actually going out through.
    ///
    /// "No output on the power meter" is the report a transmitter that answers
    /// every command gets, and the four things that decide whether any RF
    /// leaves the board — the frequency, the socket, the drive and the filter —
    /// are not visible anywhere else. Said in full the first time and again
    /// whenever any of them changes; an FT8 station keying every fifteen
    /// seconds is otherwise four lines a minute, so an unchanged path is
    /// repeated at debug only.
    ///
    /// The trace gets it every time regardless: a report about one over should
    /// not be missing the line for that over because an earlier one matched.
    fn announce_tx(&mut self, center_hz: f64) {
        let summary = format!(
            "{:.6} MHz out of {}, drive {} dB, filter {:.2} MHz",
            center_hz / 1e6,
            LimeConfig::port_label(self.cfg.channel, &self.antenna_tx, true),
            self.tx_gain_db,
            self.tx_lpf_applied / 1e6
        );
        self.trace.call("transmit", &summary, "keyed");
        // Ahead of the repeat-suppression below, and deliberately: an over that
        // cannot leave the board should say so *every* time, not once per
        // distinct transmit path. A station keying every fifteen seconds into
        // the wrong socket is putting nothing on the air every fifteen seconds.
        self.warn_if_port_is_wrong(center_hz);
        if summary == self.last_tx_summary {
            tracing::debug!("LimeSDR transmitting: {summary}");
            return;
        }
        self.last_tx_summary = summary.clone();
        tracing::info!("LimeSDR transmitting: {summary}");
        //
        // A LimeSDR's two transmit sockets are two matching networks — BAND1
        // is 30 MHz to 1.9 GHz, BAND2 is 2 to 2.6 GHz — and keying an amateur
        // band out of BAND2 puts the over into a microwave matching network
        // that passes essentially none of it. Nothing else in the session
        // notices: the synthesiser is on frequency, the drive is at maximum,
        // the stream is running, every call answers `ok`, and the power meter
        // reads zero. That is issue #94, and the line above had been printing
        // the socket in plain sight for three releases without anyone being
        // able to see that it was the wrong one.

        // The drive is a real setting with a real default, and the default is
        // the bottom of the range — deliberately, so an armed transmitter
        // cannot surprise anybody. What it must not do is stay there silently:
        // at 0 dB the LMS7002M puts out microwatts, a LimeRFE amplifies
        // microwatts, and every meter downstream reads zero.
        if self.tx_gain_db < LimeConfig::LOW_DRIVE_DB {
            tracing::warn!(
                "the LimeSDR's transmit gain is {} dB, at the bottom of its 0–{} dB range — \
                 that is a few microwatts out of the board, and will read as nothing on a \
                 power meter whatever is downstream of it. Raise Transmit gain in \
                 Settings → Radio.",
                self.tx_gain_db,
                LimeConfig::GAIN_MAX_DB
            );
        }
    }

    /// Say — loudly, and in the trace — when this over is going out of a socket
    /// the board does not match for this band.
    ///
    /// The four things that decide whether any RF leaves a LimeSDR are the
    /// frequency, the socket, the drive and the filter, and `announce_tx` above
    /// prints all four. Three of them are checked somewhere: the frequency
    /// against the synthesiser's range, the drive against the bottom of its
    /// own, the filter against the 30 MHz crossing. The socket was only ever
    /// *stated* — and it is the one that silently costs the whole over.
    fn warn_if_port_is_wrong(&mut self, center_hz: f64) {
        self.refresh_tx_port_note(center_hz);
        let Some(note) = self.tx_port_note.clone() else { return };
        self.trace.call("transmit", &note, "WRONG PORT");
        tracing::warn!("LimeSDR: {note}");
    }

    /// Write one block of modulated baseband.
    pub fn tx_write(&mut self, samples: &[Complex32]) -> Result<()> {
        let Some(tx) = self.tx.as_mut() else {
            return Err(Error::api("LMS_SendStream", "the transmitter is not armed".into()));
        };
        if !self.tx_running || samples.is_empty() {
            return Ok(());
        }
        let meta = ffi::StreamMetaT {
            timestamp: 0,
            // Send as it arrives rather than at a scheduled time: the engine
            // paces the over, not the hardware clock.
            wait_for_timestamp: false,
            flush_partial_packet: false,
        };
        // What is actually being handed to the transmitter, before any of it
        // reaches LimeSuite. A key-down that puts nothing on the air is either
        // the radio's fault or the signal's, and these two numbers are what
        // tells the two apart — see [`LimeHandle::tx_end`].
        self.tx_offered += samples.len() as u64;
        self.tx_peak = samples.iter().fold(self.tx_peak, |m, z| m.max(z.norm()));
        let mut sent = 0usize;
        while sent < samples.len() {
            let n = unsafe {
                (self.api.send_stream)(
                    tx,
                    samples[sent..].as_ptr().cast(),
                    samples.len() - sent,
                    &meta,
                    TX_TIMEOUT_MS,
                )
            };
            if n < 0 {
                return Err(Error::api("LMS_SendStream", self.api.err_text()));
            }
            if n == 0 {
                // The FIFO stayed full for the whole timeout. Dropping the rest
                // of the block is better than blocking the engine forever.
                tracing::debug!(
                    "LimeSDR transmit FIFO stalled, dropping {} samples",
                    samples.len() - sent
                );
                break;
            }
            sent += n as usize;
        }
        self.tx_sent += sent as u64;
        Ok(())
    }

    /// Push the last partial packet out.
    ///
    /// Without this the tail of a burst sits in LimeSuite's FIFO waiting for a
    /// packet that never comes — which on a mode with a hard timing boundary
    /// means the last symbols never reach the air.
    pub fn tx_drain(&mut self) {
        let Some(tx) = self.tx.as_mut() else { return };
        if !self.tx_running {
            return;
        }
        let meta = ffi::StreamMetaT {
            timestamp: 0,
            wait_for_timestamp: false,
            flush_partial_packet: true,
        };
        let silence = [Complex32::new(0.0, 0.0); 64];
        let _ = unsafe {
            (self.api.send_stream)(tx, silence.as_ptr().cast(), silence.len(), &meta, TX_TIMEOUT_MS)
        };
    }

    /// Stop transmitting, and say what the over actually put into the radio.
    ///
    /// # Why an over is accounted for at all
    ///
    /// "No output on the power meter" is the report a LimeSDR gives when every
    /// single call in this file has answered `ok` — the synthesiser is on
    /// frequency, the socket is selected, the drive is at maximum, the stream
    /// started — and it has been reported more than once against exactly that
    /// trace (issue #94). Everything the trace held was about *commands*, and
    /// commands were never the problem: what it could not say was whether any
    /// modulated sample ever reached the transmitter, which is the one link in
    /// the chain nothing else observes.
    ///
    /// So three numbers go in the trace at every unkey, and each one names a
    /// different culprit:
    ///
    /// * **nothing offered** — the radio was keyed and the engine never sent a
    ///   block. The transmitter is innocent; the modulator upstream of it made
    ///   no samples.
    /// * **offered, peak zero** — blocks arrived and every sample in them was
    ///   silence. That is drive or microphone gain at zero, or a digital mode
    ///   keying with nothing to say — again not the radio.
    /// * **offered but not taken** — LimeSuite's FIFO refused them, which is
    ///   the transmitter's own fault and the case where its stream status
    ///   matters.
    pub fn tx_end(&mut self) -> Result<()> {
        self.tx_drain();
        if let Some(tx) = self.tx.as_mut()
            && self.tx_running
        {
            // Read before the stream is stopped: a stopped stream has no
            // status worth having.
            let mut st = ffi::StreamStatusT::default();
            let status = (unsafe { (self.api.get_stream_status)(tx, &mut st) } == ffi::OK)
                .then(|| {
                    format!(
                        ", {} underrun(s), FIFO {}/{}",
                        st.underrun, st.fifo_filled_count, st.fifo_size
                    )
                })
                .unwrap_or_default();
            let rc = unsafe { (self.api.stop_stream)(tx) };
            self.tx_running = false;
            if rc != ffi::OK {
                let text = self.api.err_text();
                self.trace.call("LMS_StopStream", "transmit", format!("FAILED: {text}"));
                return Err(Error::api("LMS_StopStream", text));
            }
            let (offered, sent, peak) = (self.tx_offered, self.tx_sent, self.tx_peak);
            self.trace.call(
                "transmit",
                format!("{sent} of {offered} samples taken, peak {peak:.3} of full scale{status}"),
                "unkeyed",
            );
            self.report_over(offered, sent, peak);
        }
        Ok(())
    }

    /// The same three numbers as a sentence in the log, and only when one of
    /// them is wrong. A healthy over says nothing at info level — an FT8
    /// station keying every fifteen seconds would otherwise fill the log with
    /// its own good news.
    fn report_over(&self, offered: u64, sent: u64, peak: f32) {
        if offered == 0 {
            tracing::warn!(
                "the LimeSDR was keyed and no modulated samples were ever handed to it, so \
                 nothing went on the air. The transmitter is not the problem here: check that \
                 a microphone or digital mode is actually feeding this over."
            );
        } else if peak <= 0.0 {
            tracing::warn!(
                "the LimeSDR transmitted {offered} samples of pure silence, which is no RF \
                 whatever the drive on the board is set to. Raise Drive (and Mic gain for a \
                 voice over) — the LimeSDR's own Transmit gain cannot amplify a signal that \
                 is not there."
            );
        } else if sent < offered {
            let short = offered - sent;
            tracing::warn!(
                "the LimeSDR's transmit FIFO would not take {short} of {offered} samples this \
                 over ({:.0}% of it), so what went out was chopped. Try a lower sample rate.",
                100.0 * short as f64 / offered as f64
            );
        } else {
            tracing::debug!(
                "LimeSDR over finished: {sent} samples out, peak {peak:.3} of full scale"
            );
        }
    }

    pub fn tx_active(&self) -> bool {
        self.tx_running
    }
}

impl Drop for LimeHandle {
    fn drop(&mut self) {
        if self.closed {
            return; // `close` already ran, on the engine's release path.
        }
        self.teardown_streams();
        // Not `ctl().close()`: on this path a LimeRFE's board link may still
        // hold the shared device, so `DevCtl::drop` closes it only once the
        // last holder lets go.
    }
}
