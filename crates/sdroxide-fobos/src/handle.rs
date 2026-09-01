//! The handle the rest of the program holds, and the accounting behind it.
//!
//! Same shape as `sdroxide-hydrasdr`/`sdroxide-airspy`: one thread owns the
//! receiver, control goes in over a crossbeam channel, samples come back out
//! through an `rtrb` ring of interleaved `f32`. What's genuinely simpler here
//! than either of those: `fobos_rx_read_sync` is one blocking call that
//! returns a whole block of already-interleaved `f32` I/Q, no USB transfer
//! queue, no unpacking, no framing to carry across a boundary — see
//! `stream.rs`.
//!
//! RF port, single-channel HF/direct-sampling, and dual-channel HF (see
//! `stream.rs`'s own module doc for the two `WbDdc`s that last one needs)
//! all stream through here. `Port::Hf1`/`Port::Hf2` pick one of the two real
//! ADC channels a direct-sampling read hands back; `Port::HfDual` keeps
//! both, delivered through [`FobosHandle::rx_read_pair`] rather than
//! [`FobosHandle::rx_read`] — the diversity *combining* itself is not here.
//! It lives in `src/fobos_source.rs`, the same layer `sdroxide-sdrplay`'s
//! own dual-tuner combining lives at, not inside the driver crate.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crossbeam_channel::Sender;
use rtrb::{Consumer, Producer, RingBuffer};

use crate::error::Result;

/// How often the stream thread emits a throughput line.
const STATS_INTERVAL: Duration = Duration::from_secs(2);

/// A control message for the stream thread.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Ctrl {
    Center(f64),
    /// A rate change is not a live setting — it stops the sync stream,
    /// reprograms the sample rate, and restarts it with a buffer sized for
    /// the new rate. Verified against real hardware in this exact
    /// stop/reconfigure/restart shape on `Port::Rf`; the HF ports build
    /// their downconverter at open time and log-and-ignore one instead.
    ///
    /// Part of this crate's own API rather than something sdroxide drives:
    /// `IqSource` has no live rate setter, so `src/fobos_source.rs` never
    /// sends this and an operator changing the rate in Settings gets a
    /// reopen. `examples/stream_probe.rs` is what exercises it — the same
    /// arrangement `sdroxide-hydrasdr`, `sdroxide-airspy` and
    /// `sdroxide-hackrf` all have for their own `Ctrl::Rate`.
    Rate(f64),
    /// 0..3, confirmed against `fobos.c`'s own register mask.
    LnaGain(u8),
    /// 0..31, likewise confirmed.
    VgaGain(u8),
    ClkExternal(bool),
    Shutdown,
}

/// Control messages accumulated over one pass of the thread loop.
///
/// A retune is one FFI call and dragging the panadapter emits hundreds of
/// `Center` messages a second. Applying each in turn would put the thread
/// permanently behind the operator's hand, so the whole channel is
/// collapsed into this and each field applied once, last value wins.
#[derive(Debug, Default, PartialEq)]
pub(crate) struct Pending {
    pub center: Option<f64>,
    pub rate: Option<f64>,
    pub lna_gain: Option<u8>,
    pub vga_gain: Option<u8>,
    pub clk_external: Option<bool>,
    pub shutdown: bool,
}

impl Pending {
    pub(crate) fn absorb(&mut self, c: Ctrl) {
        match c {
            Ctrl::Center(v) => self.center = Some(v),
            Ctrl::Rate(v) => self.rate = Some(v),
            Ctrl::LnaGain(v) => self.lna_gain = Some(v),
            Ctrl::VgaGain(v) => self.vga_gain = Some(v),
            Ctrl::ClkExternal(v) => self.clk_external = Some(v),
            Ctrl::Shutdown => self.shutdown = true,
        }
    }

    pub(crate) fn is_empty(&self) -> bool {
        *self == Pending::default()
    }
}

/// Throughput and health accounting. Trimmed from `sdroxide-hydrasdr`'s own
/// (no clock-drift/ppm estimate yet — not worth the complexity until this
/// backend has been run long enough against real hardware to say whether it
/// is even meaningful here).
pub(crate) struct RxStats {
    since: Instant,
    win_samples: u64,
    win_dropped: u64,
    /// Discarded while the engine was not reading this receiver because
    /// some other radio in the session was transmitting — see
    /// `IqSource::set_rx_paused`. Fobos itself never transmits, but the
    /// session-wide half-duplex convention still applies to it as a
    /// spectator/panadapter.
    win_keyed: u64,
    win_errors: u64,
    total_samples: u64,
    total_dropped: u64,
    total_keyed: u64,
    total_errors: u64,
    first_error: Option<String>,
}

impl RxStats {
    pub(crate) fn new() -> RxStats {
        RxStats {
            since: Instant::now(),
            win_samples: 0,
            win_dropped: 0,
            win_keyed: 0,
            win_errors: 0,
            total_samples: 0,
            total_dropped: 0,
            total_keyed: 0,
            total_errors: 0,
            first_error: None,
        }
    }

    pub(crate) fn on_iq(&mut self, pairs: usize) {
        self.win_samples += pairs as u64;
        self.total_samples += pairs as u64;
    }

    pub(crate) fn on_dropped(&mut self, pairs: usize) {
        self.win_dropped += pairs as u64;
        self.total_dropped += pairs as u64;
    }

    pub(crate) fn on_dropped_keyed(&mut self, pairs: usize) {
        self.win_keyed += pairs as u64;
        self.total_keyed += pairs as u64;
    }

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

    pub(crate) fn on_error(&mut self, what: &str) {
        self.win_errors += 1;
        self.total_errors += 1;
        if self.first_error.is_none() {
            self.first_error = Some(what.to_string());
        }
    }

    pub(crate) fn summary(&self) -> String {
        format!(
            "{} samples, {} dropped, {} errors{}",
            self.total_samples,
            self.total_dropped,
            self.total_errors,
            match &self.first_error {
                Some(e) => format!(" (first: {e})"),
                None => String::new(),
            },
        )
    }

    /// Report on the window just ended, but only when there is something
    /// worth reporting.
    pub(crate) fn tick(&mut self) {
        let dt = self.since.elapsed();
        if dt < STATS_INTERVAL {
            return;
        }
        let ksps = self.win_samples as f64 / dt.as_secs_f64() / 1000.0;
        let silent = self.win_samples == 0;
        if self.win_dropped > 0 || self.win_errors > 0 || silent {
            let mut line = format!(
                "Fobos RX: {} samples ({ksps:.1} ksps) over {:.2}s",
                self.win_samples,
                dt.as_secs_f64()
            );
            if silent {
                line.push_str(
                    "; NOTHING ARRIVED — the receiver is not sending, which is a \
                     different fault from not keeping up",
                );
            }
            if self.win_dropped > 0 {
                line.push_str(&format!(
                    "; {} sample(s) DROPPED (RX ring full — the DSP thread is not \
                     keeping up)",
                    self.win_dropped
                ));
            }
            if self.win_errors > 0 {
                line.push_str(&format!(
                    "; {} error(s){}",
                    self.win_errors,
                    match &self.first_error {
                        Some(e) => format!(", first was {e}"),
                        None => String::new(),
                    }
                ));
            }
            line.push_str(&self.keyed_note());
            tracing::warn!("{line}");
        } else {
            tracing::debug!(
                "Fobos RX: {} samples ({ksps:.1} ksps) over {:.2}s; total {}{}",
                self.win_samples,
                dt.as_secs_f64(),
                self.total_samples,
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

/// Push interleaved samples into the RX ring, keeping every sample's lanes
/// together. `lanes` is 2 for plain I,Q (every port but `Port::HfDual`) or 4
/// for `Port::HfDual`'s I,Q,I,Q main/aux pairs (see [`ring_for_pair`]) — it
/// only matters for how a drop is counted; the push itself is generic over
/// shape.
///
/// All-or-nothing, same reasoning as every other native backend here: a
/// partial push leaves the ring out of step with its own lane boundary,
/// which for `lanes: 2` swaps I with Q and for `lanes: 4` scrambles which
/// float belongs to which channel, for the rest of the session.
pub(crate) fn push_iq(
    rx: &mut Producer<f32>,
    iq: &[f32],
    lanes: usize,
    stats: &mut RxStats,
    paused: bool,
) {
    let Ok(mut chunk) = rx.write_chunk(iq.len()) else {
        if paused {
            stats.on_dropped_keyed(iq.len() / lanes);
        } else {
            stats.on_dropped(iq.len() / lanes);
        }
        return;
    };
    let (head, tail) = chunk.as_mut_slices();
    head.copy_from_slice(&iq[..head.len()]);
    tail.copy_from_slice(&iq[head.len()..]);
    chunk.commit_all();
}

/// Size the RX ring for a complex rate — half a second of interleaved
/// floats, rounded up to a power of two. Sized for the ADC's own top rate
/// (80 Msps on the unit this crate was verified against) rather than the
/// requested one, since a rate change does not reallocate the ring.
pub(crate) fn ring_for(rate_hz: f64) -> (Producer<f32>, Consumer<f32>) {
    let cap = ((rate_hz * 2.0 * 0.5) as usize).next_power_of_two().max(1 << 16);
    RingBuffer::<f32>::new(cap)
}

/// Same as [`ring_for`], sized for `Port::HfDual`'s ring — which carries
/// **four** floats per complex sample pair (main I, main Q, aux I, aux Q),
/// not two, so both channels can only ever be pushed and drained together
/// and can never drift apart the way two independent rings could if one
/// filled while the other had room. `rx_read_pair` is what unpacks this
/// back into two separate streams.
pub(crate) fn ring_for_pair(rate_hz: f64) -> (Producer<f32>, Consumer<f32>) {
    let cap = ((rate_hz * 4.0 * 0.5) as usize).next_power_of_two().max(1 << 16);
    RingBuffer::<f32>::new(cap)
}

/// Shared state the stream thread publishes and the handle reads.
pub(crate) struct Shared {
    pub alive: AtomicBool,
    /// Milliseconds since the thread started, at the last sample delivered.
    pub last_rx_ms: AtomicU64,
    /// The complex rate in use, in milli-hertz so a rate change is visible
    /// to the handle without a lock.
    pub rate_milli_hz: AtomicU64,
    /// Set while the engine is transmitting some other radio in the session
    /// and therefore not reading this one — see `IqSource::set_rx_paused`.
    pub rx_paused: AtomicBool,
}

impl Shared {
    pub(crate) fn new() -> Shared {
        Shared {
            alive: AtomicBool::new(true),
            last_rx_ms: AtomicU64::new(0),
            rate_milli_hz: AtomicU64::new(0),
            rx_paused: AtomicBool::new(false),
        }
    }
}

/// Which of the receiver's inputs to stream. `Rf` goes through the tuner
/// (mixer + LNA/VGA); `Hf1`/`Hf2` bypass it entirely for direct sampling of
/// one real ADC channel apiece — see `stream.rs`'s module doc for how that
/// becomes complex baseband. `HfDual` keeps both channels, coherent, read
/// together through [`FobosHandle::rx_read_pair`] — see that method and
/// `stream.rs`'s own module doc for why one call reading both is what keeps
/// them aligned, and why the diversity combining itself is not here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    Rf,
    Hf1,
    Hf2,
    HfDual,
}

/// What to open, and what to open it with. Converted from
/// `sdroxide_types::FobosConfig` at `src/fobos_source.rs`, the one place
/// that also owns the diversity combiner for `Port::HfDual` — this struct
/// stays deliberately unaware of it, matching `sdroxide-sdrplay`'s own
/// handle, which knows nothing about the `Diversity` filter its own second
/// tuner feeds.
#[derive(Debug, Clone)]
pub struct OpenParams {
    /// Exact serial match, or empty for the first device found.
    pub serial: String,
    pub port: Port,
    pub center_hz: f64,
    /// The ADC's own I/Q rate on `Port::Rf`; the *target* complex output
    /// rate of the software downconverter on `Port::Hf1`/`Port::Hf2` — the
    /// achieved rate for either is `FobosHandle::sample_rate_hz` once open,
    /// same as every other backend's snapped-rate convention.
    pub sample_rate_hz: f64,
    /// 0..3. Meaningless on `Port::Hf1`/`Port::Hf2` (per the reference
    /// driver's own UI, which hides these controls there) — the front end
    /// they gain is powered down while direct sampling is enabled. Applied
    /// only on `Port::Rf`.
    pub lna_gain: u8,
    /// 0..31. Same caveat as `lna_gain`.
    pub vga_gain: u8,
    pub clk_external: bool,
}

/// What the thread learned about the receiver while opening it.
pub(crate) struct DeviceInfo {
    pub label: String,
    pub board: crate::device::BoardInfo,
    pub rates_hz: Vec<f64>,
    pub sample_rate_hz: f64,
    /// The ADC's own real rate — on `Rf` the same as `sample_rate_hz`, on the
    /// HF ports the raw rate the downconverter decimates from. Exposed so a
    /// caller that needs to predict `WbDdc`'s own near-DC clamp (see
    /// `sdroxide_dsp::clamp_center_hz`) has both numbers the clamp actually
    /// needs, without a live readback from the thread that owns the DDC.
    pub adc_rate_hz: f64,
}

/// An open Fobos SDR, on whichever [`Port`] it was opened with.
pub struct FobosHandle {
    rx: Consumer<f32>,
    ctrl: Sender<Ctrl>,
    shared: Arc<Shared>,
    opened_at: Instant,
    join: Option<JoinHandle<()>>,

    /// Description for logs and the UI, filled in by the thread at open
    /// time.
    pub label: String,
    pub board: crate::device::BoardInfo,
    /// Every sample rate this receiver offers (including the defensive
    /// low-rate entries `device::samplerates` appends in case a unit's own
    /// firmware doesn't already report them).
    pub rates_hz: Vec<f64>,
    pub sample_rate_hz: f64,
    /// See [`DeviceInfo::adc_rate_hz`].
    pub adc_rate_hz: f64,
}

impl FobosHandle {
    /// Open a receiver and start streaming on the RF port. The device is
    /// opened and configured on the stream thread, not here, so that every
    /// FFI call happens on one thread — this blocks until that has either
    /// succeeded or failed.
    pub fn open(params: OpenParams) -> Result<FobosHandle> {
        crate::stream::spawn(params)
    }

    /// Tell the stream thread that the engine has stopped reading for an
    /// over elsewhere in the session, and then that it has started again —
    /// see `IqSource::set_rx_paused`. The receiver itself is left running.
    pub fn set_rx_paused(&self, paused: bool) {
        self.shared.rx_paused.store(paused, Ordering::Relaxed);
    }

    pub fn is_alive(&self) -> bool {
        self.shared.alive.load(Ordering::Relaxed)
    }

    /// The complex rate currently in use, which a rate change moves under
    /// the caller.
    pub fn current_rate_hz(&self) -> f64 {
        let m = self.shared.rate_milli_hz.load(Ordering::Relaxed);
        if m == 0 { self.sample_rate_hz } else { m as f64 / 1000.0 }
    }

    /// How long the receiver has gone without delivering samples, measured
    /// from the last block or — if none ever arrived — from when it was
    /// opened.
    pub fn silent_for(&self) -> Duration {
        let since_open = self.opened_at.elapsed();
        let last = Duration::from_millis(self.shared.last_rx_ms.load(Ordering::Relaxed));
        since_open.saturating_sub(last)
    }

    /// Drain interleaved I,Q floats into `out`, for every port except
    /// [`Port::HfDual`] — see [`Self::rx_read_pair`] for that one. Always
    /// returns an even count. Zero means nothing is available yet.
    pub fn rx_read(&mut self, out: &mut [f32]) -> usize {
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

    /// [`Port::HfDual`]'s own read: drains both channels' interleaved I,Q
    /// floats — HF1 into `main_out`, HF2 into `aux_out` — from the single
    /// four-lane ring [`ring_for_pair`] built. One call for both is what
    /// keeps them sample-aligned: the two can never desync the way two
    /// independently-drained rings could if one had room and the other
    /// didn't, since there is only the one ring and one take from it.
    /// Returns the number of complex pairs delivered to *each* buffer
    /// (always equal); zero means nothing is available yet. On any other
    /// port both buffers are left untouched and this returns 0.
    pub fn rx_read_pair(&mut self, main_out: &mut [f32], aux_out: &mut [f32]) -> usize {
        let want = (main_out.len() / 2).min(aux_out.len() / 2);
        let take = ((self.rx.slots() / 4).min(want)) * 4;
        if take == 0 {
            return 0;
        }
        let Ok(chunk) = self.rx.read_chunk(take) else {
            return 0;
        };
        let (head, tail) = chunk.as_slices();
        for (idx, &v) in head.iter().chain(tail.iter()).enumerate() {
            let (sample, lane) = (idx / 4, idx % 4);
            match lane {
                0 => main_out[2 * sample] = v,
                1 => main_out[2 * sample + 1] = v,
                2 => aux_out[2 * sample] = v,
                _ => aux_out[2 * sample + 1] = v,
            }
        }
        chunk.commit_all();
        take / 4
    }

    fn send(&self, c: Ctrl) {
        // A closed channel means the thread has exited; `is_alive` picks
        // that up, so there is nothing useful to do here.
        let _ = self.ctrl.send(c);
    }

    pub fn set_center_hz(&self, hz: f64) {
        self.send(Ctrl::Center(hz));
    }

    pub fn set_rate_hz(&self, hz: f64) {
        self.send(Ctrl::Rate(hz));
    }

    pub fn set_lna_gain(&self, value: u8) {
        self.send(Ctrl::LnaGain(value));
    }

    pub fn set_vga_gain(&self, value: u8) {
        self.send(Ctrl::VgaGain(value));
    }

    pub fn set_clk_external(&self, on: bool) {
        self.send(Ctrl::ClkExternal(on));
    }

    /// Stop the stream thread and let the receiver go, without dropping the
    /// handle. Blocks until the thread has closed the device. Afterwards
    /// the handle is inert: [`Self::rx_read`] drains what is left in the
    /// ring and then returns nothing, control messages go nowhere, and
    /// [`Self::is_alive`] is false. Idempotent.
    pub fn release(&mut self) {
        let _ = self.ctrl.send(Ctrl::Shutdown);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }

    pub(crate) fn from_parts(
        rx: Consumer<f32>,
        ctrl: Sender<Ctrl>,
        shared: Arc<Shared>,
        join: JoinHandle<()>,
        info: DeviceInfo,
    ) -> FobosHandle {
        FobosHandle {
            rx,
            ctrl,
            shared,
            opened_at: Instant::now(),
            join: Some(join),
            label: info.label,
            board: info.board,
            rates_hz: info.rates_hz,
            sample_rate_hz: info.sample_rate_hz,
            adc_rate_hz: info.adc_rate_hz,
        }
    }
}

impl Drop for FobosHandle {
    fn drop(&mut self) {
        self.release();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Dragging the dial emits hundreds of messages a second; only the last
    /// value of each field can matter.
    #[test]
    fn pending_keeps_only_the_last_value_of_each_field() {
        let mut p = Pending::default();
        assert!(p.is_empty());
        for hz in [7_100_000.0, 7_150_000.0, 7_200_000.0] {
            p.absorb(Ctrl::Center(hz));
        }
        p.absorb(Ctrl::LnaGain(1));
        p.absorb(Ctrl::LnaGain(3));
        p.absorb(Ctrl::VgaGain(20));
        assert_eq!(p.center, Some(7_200_000.0));
        assert_eq!(p.lna_gain, Some(3));
        assert_eq!(p.vga_gain, Some(20));
        assert!(!p.is_empty());
        // Fields nobody set stay unset.
        assert_eq!(p.rate, None);
        assert_eq!(p.clk_external, None);
    }

    /// Shutdown must survive anything that arrives after it in the same
    /// batch, or a busy dial could keep the thread alive past a release.
    #[test]
    fn shutdown_is_sticky() {
        let mut p = Pending::default();
        p.absorb(Ctrl::Shutdown);
        p.absorb(Ctrl::Center(7_100_000.0));
        p.absorb(Ctrl::Rate(10_000_000.0));
        assert!(p.shutdown);
    }

    /// An odd ring capacity would eventually split an I/Q pair across the
    /// wrap.
    #[test]
    fn the_ring_holds_at_least_half_a_second_and_an_even_number_of_floats() {
        for rate in [1_250_000.0, 2_500_000.0, 50_000_000.0, 80_000_000.0] {
            let (p, _c) = ring_for(rate);
            let cap = p.buffer().capacity();
            assert_eq!(cap % 2, 0, "{rate}");
            assert!(cap as f64 >= rate * 2.0 * 0.5, "{rate}: {cap} floats");
        }
    }

    /// `Port::HfDual`'s ring is the four-lane one, and must hold at least as
    /// much *per channel* as the single-channel ring does — twice the raw
    /// float capacity for the same complex rate, since each sample now
    /// carries two channels' worth of I,Q rather than one's.
    #[test]
    fn the_pair_ring_holds_twice_the_floats_of_the_single_ring_at_the_same_rate() {
        for rate in [1_250_000.0, 2_500_000.0, 50_000_000.0, 80_000_000.0] {
            let (single, _) = ring_for(rate);
            let (pair, _) = ring_for_pair(rate);
            let (cap1, cap4) = (single.buffer().capacity(), pair.buffer().capacity());
            assert_eq!(cap4 % 4, 0, "{rate}");
            assert!(cap4 as f64 >= rate * 4.0 * 0.5, "{rate}: {cap4} floats");
            assert!(cap4 >= cap1 * 2, "{rate}: pair ring {cap4} vs single ring {cap1}");
        }
    }

    /// A handle whose ring already has the given quad-interleaved floats in
    /// it, for exercising [`FobosHandle::rx_read_pair`] directly without a
    /// live stream thread. The join handle is an already-finished no-op
    /// thread and the control channel's receiver is dropped, both fine
    /// since this never sends a control message or calls `release`.
    fn dual_handle_with(quad_floats: &[f32]) -> FobosHandle {
        let (mut prod, cons) = RingBuffer::<f32>::new(64);
        let mut stats = RxStats::new();
        push_iq(&mut prod, quad_floats, 4, &mut stats, false);
        let (ctrl_tx, _ctrl_rx) = crossbeam_channel::unbounded();
        FobosHandle::from_parts(
            cons,
            ctrl_tx,
            Arc::new(Shared::new()),
            std::thread::spawn(|| {}),
            DeviceInfo {
                label: "test".into(),
                board: crate::device::BoardInfo {
                    hw_revision: String::new(),
                    fw_version: String::new(),
                    manufacturer: String::new(),
                    product: String::new(),
                    serial: String::new(),
                },
                rates_hz: Vec::new(),
                sample_rate_hz: 2_500_000.0,
                adc_rate_hz: 80_000_000.0,
            },
        )
    }

    /// `rx_read_pair`'s own de-interleaving: main's I,Q land at lanes 0,1
    /// and aux's at lanes 2,3, the same order `pump_hf_dual` writes them.
    #[test]
    fn quad_interleaved_samples_split_back_into_matching_main_and_aux_pairs() {
        let mut handle =
            dual_handle_with(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]);
        let mut main = [0.0f32; 6];
        let mut aux = [0.0f32; 6];
        let pairs = handle.rx_read_pair(&mut main, &mut aux);
        assert_eq!(pairs, 3);
        assert_eq!(main, [1.0, 2.0, 5.0, 6.0, 9.0, 10.0]);
        assert_eq!(aux, [3.0, 4.0, 7.0, 8.0, 11.0, 12.0]);
    }

    /// A caller with room for fewer pairs than are queued gets exactly that
    /// many from *both* channels, never more from one than the other.
    #[test]
    fn rx_read_pair_never_gives_more_of_one_channel_than_the_other() {
        let mut handle =
            dual_handle_with(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]);
        let mut main = [0.0f32; 2];
        let mut aux = [0.0f32; 6];
        let pairs = handle.rx_read_pair(&mut main, &mut aux);
        assert_eq!(pairs, 1, "bounded by the smaller of the two output buffers");
        assert_eq!(main, [1.0, 2.0]);
        assert_eq!(aux[..2], [3.0, 4.0]);
    }

    /// A partial push would leave the ring one float out of step and swap
    /// I with Q for the rest of the session.
    #[test]
    fn push_iq_drops_whole_blocks_rather_than_splitting_a_pair() {
        let (mut prod, mut cons) = RingBuffer::<f32>::new(8);
        let mut stats = RxStats::new();
        push_iq(&mut prod, &[1.0, 2.0, 3.0, 4.0], 2, &mut stats, false);
        assert_eq!(cons.slots(), 4);
        // Six more floats into four free slots: nothing goes in.
        push_iq(&mut prod, &[0.0; 6], 2, &mut stats, false);
        assert_eq!(cons.slots(), 4);
        assert_eq!(stats.total_dropped, 3);
        assert_eq!(cons.pop(), Ok(1.0));
    }

    /// A ring that fills because the engine stopped reading for an over
    /// elsewhere in the session is not the DSP thread falling behind.
    #[test]
    fn a_full_ring_while_paused_is_not_an_overrun() {
        let (mut prod, mut cons) = RingBuffer::<f32>::new(4);
        let mut stats = RxStats::new();

        push_iq(&mut prod, &[1.0, 2.0, 3.0, 4.0], 2, &mut stats, true);
        push_iq(&mut prod, &[5.0, 6.0], 2, &mut stats, true);
        assert_eq!(stats.total_dropped, 0, "a paused receiver reports no overruns");
        assert_eq!(stats.total_keyed, 1, "the discarded pair is accounted for as keyed");

        push_iq(&mut prod, &[7.0, 8.0], 2, &mut stats, false);
        assert_eq!(stats.total_dropped, 1);
        assert_eq!(stats.total_keyed, 1);

        while cons.pop().is_ok() {}
        push_iq(&mut prod, &[9.0, 10.0], 2, &mut stats, false);
        assert_eq!(cons.slots() % 2, 0);
    }
}
