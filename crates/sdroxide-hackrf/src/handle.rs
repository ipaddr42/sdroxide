//! The handle the rest of the program holds, and the accounting behind it.
//!
//! Same shape as [`sdroxide_rtlsdr::RtlSdrHandle`] and `sdroxide-airspyhf`: one
//! blocking thread owns the radio, control goes in over a crossbeam channel,
//! samples come back out through an `rtrb` ring. Three things differ, and all
//! three come from this being a half-duplex *transceiver* rather than a
//! receiver.
//!
//! **The rings carry bytes, not floats.** Every other backend here pushes
//! interleaved `f32` sized at half a second. At 20 Msps that formula wants
//! 20 M × 2 floats = 160 MB of ring, which is not a reasonable thing to
//! allocate. The wire format is one signed byte per component, so the ring
//! carries that and [`crate::convert`] runs on the consumer side — a quarter of
//! the memory, one fewer copy, and the conversion lands on the thread that was
//! going to touch the samples anyway.
//!
//! **A transmit transition cannot be coalesced.** [`Pending`] is last-value-wins,
//! which is right for a dial and catastrophic for a key-down: a `Begin` and an
//! `End` collapsed into one field would leave the transmitter keyed forever, or
//! never key at all. Direction changes therefore ride an ordered queue beside
//! the collapsible fields.
//!
//! **Silence is not death while transmitting.** There is no receive traffic
//! during an over, so [`HackRfHandle::silent_for`] reports zero while the
//! transmitter is keyed. Without that guard the engine's "the radio has gone
//! quiet, reopen it" timer fires three seconds into any long transmission and
//! tears the device down mid-over. No other backend here has this hazard,
//! because none of the others is both half duplex and on USB.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;
use rtrb::{Consumer, Producer, RingBuffer};
use sdroxide_types::HackRfConfig;

use crate::error::Result;
use crate::protocol::{BULK_PACKET, BYTES_PER_SAMPLE, BoardKind, GainSetter};
use crate::trace::Trace;

/// How often the stream thread emits a throughput line.
const STATS_INTERVAL: Duration = Duration::from_secs(2);

/// How much of the stream the RX ring holds, and the ceiling on that.
///
/// 120 ms rather than the half-second the receive-only backends use, and capped:
/// at 20 Msps half a second would be 20 MB even in bytes, and the cap is what
/// keeps a rate change from quietly tripling the process's memory. 120 ms is
/// still four times the engine's block period at every rate this radio runs.
const RX_RING_SECS: f64 = 0.120;
const RX_RING_MAX: usize = 8 << 20;

/// How much the TX ring holds.
///
/// Longer than the RX ring on purpose: the engine paces transmit blocks against
/// the wall clock and expects `tx_write` to block when the ring is full, so this
/// is the buffer that absorbs scheduling jitter between the modulator and the
/// USB pump. Too short and an over develops gaps under load; too long and PTT
/// release lags by the whole buffer.
const TX_RING_SECS: f64 = 0.200;
const TX_RING_MAX: usize = 8 << 20;

/// A change of direction. Ordered, never collapsed — see the module header.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum TxTransition {
    Begin {
        center_hz: f64,
    },
    End,
    /// Retune while keyed, for split and for a dial moved mid-over.
    Freq(f64),
}

/// A control message for the stream thread.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Ctrl {
    Center(f64),
    Rate(f64),
    FilterBw(f64),
    Ppm(f64),
    Gain(GainSetter, f64),
    Amp { tx: bool, on: bool },
    BiasTee(bool),
    Tx(TxTransition),
    Shutdown,
}

/// Control messages accumulated over one pass of the thread loop.
///
/// A retune is one control transfer and dragging the panadapter emits hundreds
/// of `Center` messages a second. Applying each in turn would put the thread
/// permanently behind the operator's hand *and* starve the completion drain, so
/// the collapsible fields are reduced to last-value-wins — which is exactly the
/// right semantics for a dial, and exactly the wrong one for [`TxTransition`],
/// which is why that keeps its own ordered queue.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct Pending {
    pub center: Option<f64>,
    pub rate: Option<f64>,
    pub filter_bw: Option<f64>,
    pub ppm: Option<f64>,
    pub lna: Option<f64>,
    pub vga: Option<f64>,
    pub txvga: Option<f64>,
    pub amp: Option<bool>,
    pub tx_amp: Option<bool>,
    pub bias_tee: Option<bool>,
    pub tx: VecDeque<TxTransition>,
    pub shutdown: bool,
}

impl Pending {
    pub(crate) fn absorb(&mut self, c: Ctrl) {
        match c {
            Ctrl::Center(v) => self.center = Some(v),
            Ctrl::Rate(v) => self.rate = Some(v),
            Ctrl::FilterBw(v) => self.filter_bw = Some(v),
            Ctrl::Ppm(v) => self.ppm = Some(v),
            Ctrl::Gain(GainSetter::Lna, v) => self.lna = Some(v),
            Ctrl::Gain(GainSetter::Vga, v) => self.vga = Some(v),
            Ctrl::Gain(GainSetter::TxVga, v) => self.txvga = Some(v),
            Ctrl::Amp { tx: false, on } => self.amp = Some(on),
            Ctrl::Amp { tx: true, on } => self.tx_amp = Some(on),
            Ctrl::BiasTee(v) => self.bias_tee = Some(v),
            // A run of retunes while keyed collapses onto itself — a drag is
            // still a drag during an over — but never across a Begin or an End,
            // which would reorder the key-down.
            Ctrl::Tx(TxTransition::Freq(hz)) => {
                if let Some(TxTransition::Freq(last)) = self.tx.back_mut() {
                    *last = hz;
                } else {
                    self.tx.push_back(TxTransition::Freq(hz));
                }
            }
            Ctrl::Tx(t) => self.tx.push_back(t),
            Ctrl::Shutdown => self.shutdown = true,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        *self == Pending::default()
    }
}

/// Throughput and health accounting, mirroring the RTL-SDR backend's.
pub(crate) struct RxStats {
    nominal_hz: f64,
    /// When the first sample arrived, and the sample count since then.
    ///
    /// Deliberately *not* the thread start: identifying and configuring the
    /// radio is a dozen control transfers before the first sample appears, and
    /// counting that dead time against the sample total biases the clock
    /// estimate low.
    first_iq: Option<Instant>,
    since: Instant,
    win_samples: u64,
    win_dropped: u64,
    /// Discarded while the engine was not reading this receiver because the
    /// station was transmitting. Counted apart from `win_dropped` because it
    /// is not a fault: see [`RxStats::on_dropped_keyed`].
    win_keyed: u64,
    win_errors: u64,
    total_samples: u64,
    total_dropped: u64,
    total_keyed: u64,
    total_errors: u64,
    stalls: u64,
}

impl RxStats {
    pub(crate) fn new(nominal_hz: f64) -> RxStats {
        RxStats {
            nominal_hz,
            first_iq: None,
            since: Instant::now(),
            win_samples: 0,
            win_dropped: 0,
            win_keyed: 0,
            win_errors: 0,
            total_samples: 0,
            total_dropped: 0,
            total_keyed: 0,
            total_errors: 0,
            stalls: 0,
        }
    }

    pub(crate) fn on_iq(&mut self, pairs: usize) {
        self.win_samples += pairs as u64;
        match self.first_iq {
            // Start the clock at the first block and do not count that block:
            // it spans an unknown interval reaching back into device setup.
            None => self.first_iq = Some(Instant::now()),
            Some(_) => self.total_samples += pairs as u64,
        }
    }

    pub(crate) fn on_dropped(&mut self, pairs: usize) {
        self.win_dropped += pairs as u64;
        self.total_dropped += pairs as u64;
    }

    /// Record `pairs` complex samples discarded because the ring was full while
    /// the station was transmitting.
    ///
    /// Not this radio's own over — a HackRF gives up its receive endpoint
    /// before it keys, so nothing arrives to discard. This is the case where it
    /// is somebody else's panadapter: the engine stops reading a half-duplex
    /// source for the length of an over and empties the ring on unkey, while
    /// this receiver, which is not the transmitter, carries on streaming
    /// throughout. The ring fills within its own depth of key-down and
    /// everything after that is discarded: expected, at exactly the sample
    /// rate, for as long as the operator transmits. Counting it as an overrun
    /// turns an ordinary over into a warning that blames the DSP thread and
    /// advises a lower sample rate. See `IqSource::set_rx_paused`.
    pub(crate) fn on_dropped_keyed(&mut self, pairs: usize) {
        self.win_keyed += pairs as u64;
        self.total_keyed += pairs as u64;
    }

    /// What this receiver threw away while the station was transmitting,
    /// phrased so it cannot be read as a fault. Empty when the operator did not
    /// key up, so it costs nothing on a receive-only session.
    fn keyed_note(&self) -> String {
        if self.win_keyed == 0 {
            return String::new();
        }
        format!(
            "; {} sample(s) discarded while keyed (expected — this receiver is not read \
             during an over); {} discarded while keyed in total",
            self.win_keyed, self.total_keyed,
        )
    }

    pub(crate) fn on_error(&mut self) {
        self.win_errors += 1;
        self.total_errors += 1;
    }

    pub(crate) fn on_stall(&mut self) {
        self.stalls += 1;
        self.on_error();
    }

    /// Reset the clock measurement.
    ///
    /// Called on every direction change, because an over is a gap in the
    /// receive stream and counting it as slow samples would report a wildly
    /// negative clock error. The RTL-SDR backend has no equivalent because it
    /// never stops receiving.
    pub(crate) fn restart_clock(&mut self) {
        self.first_iq = None;
        self.total_samples = 0;
    }

    /// The radio's sample clock measured against the host's, in ppm — the
    /// figure an operator can type into the settings tab's clock trim.
    fn clock_error(&self) -> String {
        let dt = self.first_iq.map(|t| t.elapsed().as_secs_f64()).unwrap_or(0.0);
        // Below ~20 s the host's own scheduling jitter dominates.
        if dt < 20.0 || self.total_samples == 0 || self.nominal_hz <= 0.0 {
            return "clock: measuring".to_string();
        }
        let measured = self.total_samples as f64 / dt;
        let ppm = (measured / self.nominal_hz - 1.0) * 1e6;
        format!("clock: {measured:.0} sps, {ppm:+.1} ppm")
    }

    pub(crate) fn summary(&self) -> String {
        format!(
            "{} samples, {} dropped, {} transfer errors, {} endpoint stalls; {}",
            self.total_samples,
            self.total_dropped,
            self.total_errors,
            self.stalls,
            self.clock_error()
        )
    }

    pub(crate) fn tick(&mut self, trace: &Trace) {
        let dt = self.since.elapsed();
        if dt < STATS_INTERVAL {
            return;
        }
        let ksps = self.win_samples as f64 / dt.as_secs_f64() / 1000.0;
        if self.win_dropped > 0 || self.win_errors > 0 {
            let line = format!(
                "HackRF RX: {} samples ({ksps:.1} ksps) over {:.2}s; \
                 {} sample(s) DROPPED (RX ring full — the DSP thread is not keeping up; \
                 try a lower sample rate), {} transfer error(s); \
                 totals {} dropped / {} errors{}",
                self.win_samples,
                dt.as_secs_f64(),
                self.win_dropped,
                self.win_errors,
                self.total_dropped,
                self.total_errors,
                self.keyed_note(),
            );
            tracing::warn!("{line}");
            trace.note(line);
        } else {
            tracing::debug!(
                "HackRF RX: {} samples ({ksps:.1} ksps) over {:.2}s; total {}; {}{}",
                self.win_samples,
                dt.as_secs_f64(),
                self.total_samples,
                self.clock_error(),
                self.keyed_note(),
            );
        }
        self.since = Instant::now();
        self.win_samples = 0;
        self.win_dropped = 0;
        self.win_keyed = 0;
        self.win_errors = 0;
    }
}

/// Push interleaved I/Q bytes into a ring, keeping I and Q paired.
///
/// All-or-nothing: if the ring cannot take the whole block it is dropped whole.
/// Pushing what fits would leave the ring one byte out of step, swapping I with
/// Q for the rest of the session — a mirrored, unusable spectrum that reads like
/// a driver bug rather than the overrun it is.
///
/// `paused` says whether the engine has stopped reading for an over, which
/// decides how a full ring is accounted for — a fault, or the normal cost of
/// transmitting. It is deliberately not a reason to skip the push: the samples
/// are still offered, and it is the reader's business whether it wants them.
pub(crate) fn push_iq(ring: &mut Producer<u8>, iq: &[u8], stats: &mut RxStats, paused: bool) {
    debug_assert_eq!(iq.len() % BYTES_PER_SAMPLE, 0, "a partial sample must never reach the ring");
    let Ok(mut chunk) = ring.write_chunk(iq.len()) else {
        if paused {
            stats.on_dropped_keyed(iq.len() / BYTES_PER_SAMPLE);
        } else {
            stats.on_dropped(iq.len() / BYTES_PER_SAMPLE);
        }
        return;
    };
    let (head, tail) = chunk.as_mut_slices();
    head.copy_from_slice(&iq[..head.len()]);
    tail.copy_from_slice(&iq[head.len()..]);
    chunk.commit_all();
}

/// Shared state the stream thread publishes and the handle reads.
pub(crate) struct Shared {
    pub alive: AtomicBool,
    /// Milliseconds since the thread started, at the last sample delivered.
    pub last_rx_ms: AtomicU64,
    /// Whether the transmitter is keyed. Read by [`HackRfHandle::silent_for`],
    /// which must not age out an over — see the module header.
    pub tx_active: AtomicBool,
    /// Components the modulator clipped, cumulative.
    pub tx_saturated: AtomicU64,
    /// What each gain stage is really set to, in milli-dB, after the hardware's
    /// truncation. The UI shows these back, so they have to be what the radio
    /// did rather than what it was asked for.
    pub lna_mdb: AtomicI64,
    pub vga_mdb: AtomicI64,
    pub txvga_mdb: AtomicI64,
    /// Set while the engine is transmitting and therefore not reading this
    /// receiver — see `IqSource::set_rx_paused`. Read by the stream thread on
    /// every block so a ring that fills during an over is accounted for as the
    /// cost of transmitting rather than as an overrun.
    pub rx_paused: AtomicBool,
    /// Set while a control change the radio would not take is still owed.
    ///
    /// The one fault on this backend that is otherwise completely silent: the
    /// dial, the panadapter labels and every attached screen come from the
    /// engine, which was told the tune succeeded the moment it was posted to
    /// this thread. If the radio then refuses it, the picture goes on saying
    /// one thing and the radio goes on doing another with nothing anywhere to
    /// say so (issue #352). Surfaced through `IqSource::open_status`, where the
    /// operator will see it beside the rest of this radio's standing notices.
    pub ctrl_stuck: AtomicBool,
}

impl Shared {
    pub(crate) fn new() -> Shared {
        Shared {
            alive: AtomicBool::new(true),
            last_rx_ms: AtomicU64::new(0),
            tx_active: AtomicBool::new(false),
            tx_saturated: AtomicU64::new(0),
            lna_mdb: AtomicI64::new(0),
            vga_mdb: AtomicI64::new(0),
            txvga_mdb: AtomicI64::new(0),
            rx_paused: AtomicBool::new(false),
            ctrl_stuck: AtomicBool::new(false),
        }
    }

    pub(crate) fn set_gain(&self, stage: GainSetter, db: f64) {
        let v = (db * 1000.0) as i64;
        match stage {
            GainSetter::Lna => self.lna_mdb.store(v, Ordering::Relaxed),
            GainSetter::Vga => self.vga_mdb.store(v, Ordering::Relaxed),
            GainSetter::TxVga => self.txvga_mdb.store(v, Ordering::Relaxed),
        }
    }
}

/// What the thread learned about the radio while opening it.
pub(crate) struct DeviceInfo {
    pub label: String,
    pub kind: BoardKind,
    pub firmware: String,
    pub serial: Option<String>,
    pub sample_rate_hz: f64,
    pub filter_bw_hz: u32,
    pub freq_range: (f64, f64),
    /// What this board takes, for the message an operator reads when their
    /// rate was moved. A HackRF Pro's floor is a decade below the rest.
    pub rate_range: (f64, f64),
    pub has_bias_tee: bool,
    /// Set on a board that derives its own baseband filter, so a stored
    /// override can be reported as having no effect rather than silently
    /// ignored. See `BoardKind::sets_own_filter`.
    pub filter_is_automatic: bool,
    /// Set when the configured rate was outside what the hardware takes.
    /// Surfaced through `IqSource::open_status` rather than logged and
    /// forgotten.
    pub snapped_from: Option<f64>,
    /// Set when the board has enumerated below the high-speed link every
    /// HackRF is built for, which no rate it offers will survive.
    pub link_warning: Option<String>,
}

/// An open HackRF.
pub struct HackRfHandle {
    rx: Consumer<u8>,
    tx: Producer<u8>,
    ctrl: Sender<Ctrl>,
    shared: Arc<Shared>,
    opened_at: Instant,
    join: Option<JoinHandle<()>>,
    trace: Trace,

    /// Description for logs and the UI, filled in by the thread at open time.
    pub label: String,
    pub kind: BoardKind,
    pub firmware: String,
    pub serial: Option<String>,
    pub sample_rate_hz: f64,
    pub filter_bw_hz: u32,
    pub freq_range: (f64, f64),
    pub rate_range: (f64, f64),
    pub has_bias_tee: bool,
    pub filter_is_automatic: bool,
    pub snapped_from: Option<f64>,
    pub link_warning: Option<String>,
}

impl HackRfHandle {
    /// Tell the stream thread that the engine has stopped reading for an over,
    /// and then that it has started again — see `IqSource::set_rx_paused`. The
    /// receiver itself is left running: the samples keep arriving and keep
    /// being offered, this only decides whether the ones that no longer fit are
    /// reported as a fault.
    pub fn set_rx_paused(&self, paused: bool) {
        self.shared.rx_paused.store(paused, Ordering::Relaxed);
    }

    /// Open a radio and start receiving.
    ///
    /// The device is opened and configured on the stream thread, not here, so
    /// that every control transfer in the process happens on one thread — see
    /// the invariant in [`crate::usb`]. This call blocks until that has either
    /// succeeded or failed.
    pub fn open(cfg: &HackRfConfig, center_hz: f64) -> Result<HackRfHandle> {
        crate::stream::spawn(cfg, center_hz)
    }

    /// Whether the stream thread is still running.
    pub fn is_alive(&self) -> bool {
        self.shared.alive.load(Ordering::Relaxed)
    }

    /// Whether the transmitter is keyed.
    pub fn is_transmitting(&self) -> bool {
        self.shared.tx_active.load(Ordering::Relaxed)
    }

    /// Whether a settings change the radio would not take is still outstanding
    /// — see [`Shared::ctrl_stuck`]. Cleared as soon as one lands.
    pub fn ctrl_stuck(&self) -> bool {
        self.shared.ctrl_stuck.load(Ordering::Relaxed)
    }

    /// How long the radio has gone without delivering samples, measured from
    /// the last block or — if none ever arrived — from when it was opened.
    ///
    /// **Zero while transmitting.** On half-duplex hardware there is no receive
    /// traffic during an over, so an unguarded timer here declares the radio
    /// dead three seconds into any long transmission and the engine tears it
    /// down mid-over. A stream that never starts still matters as much as one
    /// that stops, which is why the two ages are otherwise measured the same
    /// way.
    pub fn silent_for(&self) -> Duration {
        if self.is_transmitting() {
            return Duration::ZERO;
        }
        let since_open = self.opened_at.elapsed();
        let last = Duration::from_millis(self.shared.last_rx_ms.load(Ordering::Relaxed));
        since_open.saturating_sub(last)
    }

    /// How many components the modulator has clipped. Drive is applied
    /// digitally on this radio, so an operator who turns it past unity gets
    /// intermodulation rather than power — and nothing else on screen says so.
    pub fn tx_saturated(&self) -> u64 {
        self.shared.tx_saturated.load(Ordering::Relaxed)
    }

    /// What each stage is really set to, after the hardware's truncation.
    pub fn gains(&self) -> (f64, f64, f64) {
        let g = |a: &AtomicI64| a.load(Ordering::Relaxed) as f64 / 1000.0;
        (g(&self.shared.lna_mdb), g(&self.shared.vga_mdb), g(&self.shared.txvga_mdb))
    }

    pub fn trace(&self) -> &Trace {
        &self.trace
    }

    /// Drain interleaved I,Q bytes into `out`. Always returns an even count, so
    /// the stream can never come out of alignment. Zero means nothing is
    /// available yet.
    pub fn rx_read(&mut self, out: &mut [u8]) -> usize {
        let take = self.rx.slots().min(out.len()) & !1;
        if take == 0 {
            return 0;
        }
        let Ok(chunk) = self.rx.read_chunk(take) else {
            return 0;
        };
        let (head, tail) = chunk.as_slices();
        out[..head.len()].copy_from_slice(head);
        out[head.len()..take].copy_from_slice(&tail[..take - head.len()]);
        chunk.commit_all();
        take
    }

    /// Throw away everything the ring is holding.
    ///
    /// Called after an over: the samples buffered when the receiver stopped are
    /// from before the transmission and are stale by the length of it.
    pub fn discard_rx(&mut self) {
        let n = self.rx.slots();
        if n > 0
            && let Ok(chunk) = self.rx.read_chunk(n)
        {
            chunk.commit_all();
        }
    }

    /// Hand interleaved I,Q bytes to the transmit ring, blocking until they fit
    /// or `deadline` passes.
    ///
    /// Blocking is the contract: the engine paces transmit blocks against the
    /// wall clock and treats a slow `tx_write` as backpressure. Dropping
    /// instead would put a hole in the over that nothing downstream could see.
    /// Returns how many bytes were accepted — short only if the deadline passed
    /// or the thread died, both of which the caller reports rather than retries.
    pub fn tx_write(&mut self, iq: &[u8], deadline: Instant) -> usize {
        debug_assert_eq!(iq.len() % BYTES_PER_SAMPLE, 0);
        let mut done = 0;
        while done < iq.len() {
            if !self.is_alive() {
                break;
            }
            let want = iq.len() - done;
            let free = self.tx.slots() & !1;
            if free == 0 {
                if Instant::now() >= deadline {
                    break;
                }
                // Short enough that the pump is never left idle waiting for us,
                // long enough not to spin a core during a long over.
                std::thread::sleep(Duration::from_micros(500));
                continue;
            }
            let take = free.min(want);
            let Ok(mut chunk) = self.tx.write_chunk(take) else {
                break;
            };
            let (head, tail) = chunk.as_mut_slices();
            head.copy_from_slice(&iq[done..done + head.len()]);
            tail.copy_from_slice(&iq[done + head.len()..done + take]);
            chunk.commit_all();
            done += take;
        }
        done
    }

    /// Bytes still waiting to go out. The transmit teardown waits on this so an
    /// over is not cut short — the difference between an FT8 burst decoding and
    /// not.
    pub fn tx_pending(&self) -> usize {
        self.tx.buffer().capacity() - self.tx.slots()
    }

    /// Whether everything the pump can still move has left the ring.
    ///
    /// **Not `tx_pending() == 0`, and the difference is load-bearing.** The
    /// pump submits whole bulk packets only — a short transfer mid-over ends the
    /// radio's own fixed-size block early and puts the two out of step for the
    /// rest of the over — so it deliberately leaves a remainder of under one
    /// packet in the ring. That scrap goes out at key-up, where a short packet
    /// is the correct end-of-data marker and the drain waits for it. Waiting for
    /// zero here would therefore never be satisfied, and a caller would burn its
    /// whole timeout on every burst.
    pub fn tx_drained(&self) -> bool {
        self.tx_pending() < BULK_PACKET
    }

    fn send(&self, c: Ctrl) {
        // A closed channel means the thread has exited; `needs_reopen` will
        // pick that up from `is_alive`, so there is nothing useful to do here.
        let _ = self.ctrl.send(c);
    }

    pub fn set_center_hz(&self, hz: f64) {
        self.send(Ctrl::Center(hz));
    }

    pub fn set_rate_hz(&self, hz: f64) {
        self.send(Ctrl::Rate(hz));
    }

    pub fn set_filter_bw_hz(&self, hz: f64) {
        self.send(Ctrl::FilterBw(hz));
    }

    pub fn set_ppm(&self, ppm: f64) {
        self.send(Ctrl::Ppm(ppm));
    }

    pub fn set_gain(&self, stage: GainSetter, db: f64) {
        self.send(Ctrl::Gain(stage, db));
    }

    pub fn set_amp(&self, tx: bool, on: bool) {
        self.send(Ctrl::Amp { tx, on });
    }

    pub fn set_bias_tee(&self, on: bool) {
        self.send(Ctrl::BiasTee(on));
    }

    /// Key up on `center_hz`. Returns once the message is queued; the radio is
    /// keyed when [`Self::is_transmitting`] goes true.
    pub fn begin_tx(&self, center_hz: f64) {
        self.send(Ctrl::Tx(TxTransition::Begin { center_hz }));
    }

    pub fn set_tx_center_hz(&self, hz: f64) {
        self.send(Ctrl::Tx(TxTransition::Freq(hz)));
    }

    pub fn end_tx(&self) {
        self.send(Ctrl::Tx(TxTransition::End));
    }

    /// Stop the stream thread and let the radio go, without dropping the
    /// handle.
    ///
    /// The engine needs this before it can build a replacement front-end: the
    /// USB interface is claimed exclusively and a second claim is refused even
    /// from this same process, so a radio that has not let go is one that
    /// cannot be reopened. Blocks until the thread has closed the device — and
    /// the thread's exit path unkeys first, so a release in the middle of an
    /// over takes the radio off the air rather than leaving it keyed with
    /// nobody driving it.
    ///
    /// Afterwards the handle is inert rather than invalid: [`Self::rx_read`]
    /// drains what is left in the ring and then returns nothing, control
    /// messages go nowhere, and [`Self::is_alive`] is false — which is what
    /// makes `HackRfSource::needs_reopen` true. Idempotent.
    pub fn release(&mut self) {
        let _ = self.ctrl.send(Ctrl::Shutdown);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }

    pub(crate) fn from_parts(
        rx: Consumer<u8>,
        tx: Producer<u8>,
        ctrl: Sender<Ctrl>,
        shared: Arc<Shared>,
        join: JoinHandle<()>,
        trace: Trace,
        info: DeviceInfo,
    ) -> HackRfHandle {
        HackRfHandle {
            rx,
            tx,
            ctrl,
            shared,
            opened_at: Instant::now(),
            join: Some(join),
            trace,
            label: info.label,
            kind: info.kind,
            firmware: info.firmware,
            serial: info.serial,
            sample_rate_hz: info.sample_rate_hz,
            filter_bw_hz: info.filter_bw_hz,
            freq_range: info.freq_range,
            rate_range: info.rate_range,
            has_bias_tee: info.has_bias_tee,
            filter_is_automatic: info.filter_is_automatic,
            snapped_from: info.snapped_from,
            link_warning: info.link_warning,
        }
    }
}

impl Drop for HackRfHandle {
    fn drop(&mut self) {
        self.release();
    }
}

/// Size a ring in bytes for a sample rate, rounded up to a power of two and
/// capped. See [`RX_RING_SECS`] for why this is not the half-second the
/// receive-only backends use.
fn ring_bytes(rate_hz: f64, secs: f64, max: usize) -> usize {
    let want = (rate_hz * BYTES_PER_SAMPLE as f64 * secs) as usize;
    want.next_power_of_two().clamp(1 << 16, max)
}

pub(crate) fn rx_ring_for(rate_hz: f64) -> (Producer<u8>, Consumer<u8>) {
    RingBuffer::<u8>::new(ring_bytes(rate_hz, RX_RING_SECS, RX_RING_MAX))
}

pub(crate) fn tx_ring_for(rate_hz: f64) -> (Producer<u8>, Consumer<u8>) {
    RingBuffer::<u8>::new(ring_bytes(rate_hz, TX_RING_SECS, TX_RING_MAX))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Dragging the dial emits hundreds of messages a second and a retune costs
    /// a control transfer. Only the last value can matter.
    #[test]
    fn pending_keeps_only_the_last_value_of_each_field() {
        let mut p = Pending::default();
        assert!(p.is_empty());
        for hz in [7_000_000.0, 7_050_000.0, 7_074_000.0] {
            p.absorb(Ctrl::Center(hz));
        }
        p.absorb(Ctrl::Gain(GainSetter::Lna, 8.0));
        p.absorb(Ctrl::Gain(GainSetter::Lna, 24.0));
        p.absorb(Ctrl::Amp { tx: false, on: true });
        assert_eq!(p.center, Some(7_074_000.0));
        assert_eq!(p.lna, Some(24.0));
        assert_eq!(p.amp, Some(true));
        assert!(!p.is_empty());
        // Fields nobody set stay unset, so `apply` does not touch the hardware
        // for settings that did not change.
        assert_eq!(p.vga, None);
        assert_eq!(p.bias_tee, None);
        assert_eq!(p.tx_amp, None);
    }

    /// The two amp settings are separate fields even though they are one
    /// hardware switch — collapsing them would lose the operator's choice for
    /// whichever direction the radio is not in.
    #[test]
    fn the_two_amp_settings_do_not_collapse_into_each_other() {
        let mut p = Pending::default();
        p.absorb(Ctrl::Amp { tx: false, on: false });
        p.absorb(Ctrl::Amp { tx: true, on: true });
        assert_eq!(p.amp, Some(false));
        assert_eq!(p.tx_amp, Some(true));
    }

    /// The hazard the ordered queue exists for: a begin and an end coalesced
    /// into one field would leave the transmitter keyed forever, or never key
    /// at all.
    #[test]
    fn a_transmit_transition_is_never_coalesced_away() {
        let mut p = Pending::default();
        p.absorb(Ctrl::Tx(TxTransition::Begin { center_hz: 14.074e6 }));
        p.absorb(Ctrl::Center(7.0e6));
        p.absorb(Ctrl::Tx(TxTransition::End));
        assert_eq!(
            p.tx.iter().copied().collect::<Vec<_>>(),
            vec![TxTransition::Begin { center_hz: 14.074e6 }, TxTransition::End],
            "both transitions survive, in order"
        );
        // The dial still collapsed, which is the point of having both.
        assert_eq!(p.center, Some(7.0e6));
    }

    /// A drag during an over is still a drag: consecutive retunes collapse, but
    /// never across a key-down or a key-up.
    #[test]
    fn retunes_collapse_within_a_run_but_not_across_a_transition() {
        let mut p = Pending::default();
        p.absorb(Ctrl::Tx(TxTransition::Begin { center_hz: 14.074e6 }));
        for hz in [14.075e6, 14.076e6, 14.077e6] {
            p.absorb(Ctrl::Tx(TxTransition::Freq(hz)));
        }
        p.absorb(Ctrl::Tx(TxTransition::End));
        p.absorb(Ctrl::Tx(TxTransition::Freq(21.0e6)));
        assert_eq!(
            p.tx.iter().copied().collect::<Vec<_>>(),
            vec![
                TxTransition::Begin { center_hz: 14.074e6 },
                TxTransition::Freq(14.077e6),
                TxTransition::End,
                TxTransition::Freq(21.0e6),
            ],
            "three retunes became one, and the End still separates them"
        );
    }

    /// Shutdown must survive anything that arrives after it in the same batch,
    /// or a busy dial could keep the thread alive past a release.
    #[test]
    fn shutdown_is_sticky() {
        let mut p = Pending::default();
        p.absorb(Ctrl::Shutdown);
        p.absorb(Ctrl::Center(7_074_000.0));
        p.absorb(Ctrl::Tx(TxTransition::Begin { center_hz: 14.0e6 }));
        assert!(p.shutdown);
    }

    /// An odd ring capacity would eventually split an I/Q pair across the wrap,
    /// and an unbounded one would allocate 20 MB at the top rate.
    #[test]
    fn the_rings_are_even_bounded_and_long_enough() {
        for rate in [2.0e6, 8.0e6, 20.0e6] {
            let (p, _c) = rx_ring_for(rate);
            let cap = p.buffer().capacity();
            assert_eq!(cap % 2, 0, "{rate}");
            assert!(cap <= RX_RING_MAX, "{rate}: {cap} bytes exceeds the cap");
            // At least one engine block (~10 ms) even after the cap bites.
            assert!(cap as f64 >= rate * 2.0 * 0.010, "{rate}: {cap} bytes is under a block");
        }
        // The cap really does bite at the top rate: 20 Msps for 120 ms is
        // 4.8 MB, which rounds to 8 MB — at the ceiling, not past it.
        let (p, _c) = rx_ring_for(20.0e6);
        assert_eq!(p.buffer().capacity(), RX_RING_MAX);
        // And a low rate is not padded up to the ceiling.
        let (p, _c) = rx_ring_for(2.0e6);
        assert!(p.buffer().capacity() < RX_RING_MAX);
    }

    /// A partial push would leave the ring one byte out of step and swap I with
    /// Q for the rest of the session.
    #[test]
    fn push_iq_drops_whole_blocks_rather_than_splitting_a_pair() {
        let (mut prod, mut cons) = RingBuffer::<u8>::new(8);
        let mut stats = RxStats::new(2.0e6);
        push_iq(&mut prod, &[1, 2, 3, 4], &mut stats, false);
        assert_eq!(cons.slots(), 4);
        // Six more bytes into four free slots: nothing goes in.
        push_iq(&mut prod, &[0; 6], &mut stats, false);
        assert_eq!(cons.slots(), 4);
        assert_eq!(stats.total_dropped, 3);
        assert_eq!(cons.pop(), Ok(1));
    }

    /// An over is a gap in the receive stream; counting it as slow samples
    /// would report a wildly wrong clock error.
    #[test]
    fn the_clock_measurement_restarts_across_an_over() {
        let mut s = RxStats::new(2.0e6);
        s.on_iq(1000);
        s.on_iq(1000);
        assert!(s.total_samples > 0);
        s.restart_clock();
        assert_eq!(s.total_samples, 0);
        assert!(s.summary().contains("measuring"), "{}", s.summary());
    }

    /// A ring that fills because the engine stopped reading for an over is not
    /// the DSP thread falling behind. A HackRF gives up its receive endpoint
    /// before keying its own transmitter, so this is the case where it is
    /// somebody else's panadapter and the over belongs to another radio.
    #[test]
    fn a_full_ring_while_paused_is_not_an_overrun() {
        let (mut prod, mut cons) = RingBuffer::<u8>::new(4);
        let mut stats = RxStats::new(2.0e6);

        push_iq(&mut prod, &[1, 2, 3, 4], &mut stats, true);
        push_iq(&mut prod, &[5, 6], &mut stats, true);
        assert_eq!(stats.total_dropped, 0, "a paused receiver reports no overruns");
        assert_eq!(stats.total_keyed, 1, "the discarded sample is accounted for as keyed");

        push_iq(&mut prod, &[7, 8], &mut stats, false);
        assert_eq!(stats.total_dropped, 1);
        assert_eq!(stats.total_keyed, 1);

        while cons.pop().is_ok() {}
        push_iq(&mut prod, &[9, 10], &mut stats, false);
        assert_eq!(cons.slots() % BYTES_PER_SAMPLE, 0);
    }
}
