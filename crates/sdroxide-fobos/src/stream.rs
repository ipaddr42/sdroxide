//! The one thread that owns the receiver, and the two pumps inside it — one
//! per [`crate::handle::Port`] family.
//!
//! Every call to the vendor library happens on this thread, matching every
//! other native backend here — `sdroxide-hydrasdr`'s own module doc gives
//! the reason: opening, configuring and streaming from one thread means
//! nothing here has to reason about concurrent access to a handle the
//! vendor library itself gives no thread-safety guarantee for.
//!
//! # RF: no USB-style transfer queue needed at all
//!
//! `fobos_rx_read_sync` is one blocking call that hands back a whole block
//! of already-interleaved `f32` I/Q — no completion to poll, no packed
//! format to unpack, no boundary to carry state across. The block period at
//! the buffer size used here is a couple of milliseconds at any rate this
//! receiver offers, which is plenty responsive for the control channel to
//! be drained once per iteration.
//!
//! # HF: two independent real ADC channels, not I/Q — settled from the
//! vendor's own source, not guessed
//!
//! With `fobos_rx_set_direct_sampling(dev, 1)`, the same interleaved buffer
//! carries two **independent real** channels — HF1 in `.re`, HF2 in `.im` —
//! rather than one channel's quadrature pair. This was genuinely ambiguous
//! from a live capture alone (two different real antennas on HF1/HF2 still
//! showed near-identical readings, which correlated local noise pickup
//! explains just as well as a shared-signal hypothesis would), so it was
//! settled by reading `libfobos`'s own C source instead of guessing further:
//! `fobos_rx_convert_samples` applies **independent DC removal** to `re` and
//! `im`, and the I/Q gain-ratio auto-calibration (`fobos_rx_calibrate`,
//! which only makes sense for two outputs of *one* quadrature mixer) is
//! explicitly skipped whenever `rx_direct_sampling` is set — a distinction
//! that has no reason to exist if the two channels were ever a quadrature
//! pair. `.re` was confirmed as HF1 by a controlled hardware test — antenna
//! on HF1 alone, 35 dB isolation between what the two channels reported.
//!
//! So a real channel needs turning into complex baseband before it is
//! useful, and this crate reuses `sdroxide_dsp::WbDdc` — the WOLA real-to-
//! complex channelizer `sdroxide-rx888` already built and verified for the
//! identical problem (direct-sampling HF, no on-board mixer). Tuning is
//! then software-only: [`sdroxide_dsp::WbDdc::set_center_hz`] *is* the
//! dial — there is no `fobos_rx_set_frequency` call in HF mode at all.
//!
//! # Real-hardware findings this is written around
//!
//! RF: open, configure, plain read loop, and a live rate change all verified
//! — a live rate change lands exactly on the requested rate above roughly
//! 8 Msps, but below that both open-time and live requests snap to
//! 8.000 Msps regardless of what's asked for (an ADC/firmware floor, not a
//! bug in this driver). HF: the raw-format finding above; that direct
//! sampling honors a requested ADC rate the same way (`open_hf`'s own
//! `hf_adc_rate_wanted`, and the same 8 Msps floor) rather than being stuck
//! at the top rate, which real-time throughput turned out to depend on —
//! at a fixed 80 Msps regardless of what the output actually needed, real
//! hardware measured roughly 85-100 audible discontinuities a second even
//! with DDC compute already on its own thread, gone once the ADC rate was
//! sized to the request instead; and that `WbDdc` wired into the pump loop
//! below produces a real spectrum from real HF1/HF2 antennas —
//! including the dual-channel path, where two identically-configured `WbDdc`
//! instances feed [`crate::handle::FobosHandle::rx_read_pair`] a genuinely
//! independent pair of channels (confirmed: the two never come back
//! byte-identical over a live run).

use std::ffi::c_int;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TrySendError};
use rtrb::Producer;
use sdroxide_dsp::{Complex32, WbDdc};

use crate::device;
use crate::error::{Error, Result};
use crate::ffi;
use crate::handle::{
    Ctrl, DeviceInfo, FobosHandle, OpenParams, Pending, Port, RxStats, Shared, push_iq, ring_for,
    ring_for_pair,
};

/// Complex (RF) or real (HF) samples per `fobos_rx_read_sync` block, at any
/// rate. This is not a latency/throughput tuning choice — it's a real
/// constraint on how large a sync-mode block this library tolerates before
/// its own async streaming path (unused here, broken on the units this was
/// checked against) would otherwise be needed. 400 blocks/sec regardless of
/// rate, so the block period is a few ms at any rate this receiver offers.
const BLOCKS_PER_SEC: f64 = 400.0;

fn buf_len_for(rate_hz: f64) -> u32 {
    (rate_hz / BLOCKS_PER_SEC).round().max(1.0) as u32
}

/// `WbDdc`'s input block size for the HF path. 8192 at this receiver's own
/// top ADC rate (80 Msps on the unit this crate was verified against) gives
/// a ~9.8 kHz coarse-tuning grid — the same order as `sdroxide-rx888`'s own
/// 7.9 kHz at 64.8 Msps/8192, which this figure is deliberately matched to
/// rather than picked independently.
const DDC_BLOCK: usize = 8192;

/// Depth of the raw-block handoff between the blocking hardware read and the
/// DDC compute on the HF ports — see [`pump_hf`]'s own doc comment for why
/// there is one at all. Smaller than `sdroxide-rx888`'s own equivalent (8):
/// each block here is a whole `ADC rate / BLOCKS_PER_SEC` read — hundreds of
/// thousands of samples — not one USB transfer's worth, so a handful is
/// already several read periods of headroom without holding megabytes of
/// raw samples in flight.
const HF_HANDOFF_DEPTH: usize = 4;

/// Pick the `WbDdc` bin count (a power of two, `DDC_BLOCK/2` at most) that
/// lands closest to `target_rate` out of `adc_rate` — the HF-path analogue
/// of `sdroxide-rx888`'s `sanitize_ddc_bins`, generalised from a fixed
/// discrete choice list to an arbitrary requested rate, since `OpenParams`
/// takes a target Hz on both ports rather than a bin-count index.
fn pick_bins(adc_rate: f64, target_rate: f64) -> usize {
    let raw = (target_rate / adc_rate * DDC_BLOCK as f64).round().max(1.0) as usize;
    raw.next_power_of_two().clamp(64, DDC_BLOCK / 2)
}

/// Open the receiver and start the stream thread.
///
/// The device is opened *on the thread* so that every FFI call happens on
/// one thread; this call blocks on a handshake until that has either
/// succeeded or failed, so the caller still gets a synchronous [`Result`].
pub(crate) fn spawn(params: OpenParams) -> Result<FobosHandle> {
    let (ctrl_tx, ctrl_rx) = crossbeam_channel::unbounded::<Ctrl>();
    // Rendezvous for the open result, so the caller learns about a missing
    // device or a configuration failure as a normal error rather than as a
    // stream that silently never starts.
    let (ready_tx, ready_rx) = crossbeam_channel::bounded::<Result<DeviceInfo>>(1);

    let shared = Arc::new(Shared::new());
    // Sized for the largest rate this receiver has been seen to offer (80
    // Msps on the unit this crate was verified against), not the one it
    // opens on: a rate change does not reallocate the ring,
    // and this same ring carries the HF path's *decimated* output, which is
    // always well under this. `HfDual`'s ring is the four-lane one — see
    // `ring_for_pair`'s own doc comment for why that, not two rings.
    let (rx_prod, rx_cons) = if params.port == Port::HfDual {
        ring_for_pair(80_000_000.0)
    } else {
        ring_for(80_000_000.0)
    };

    let thread_shared = Arc::clone(&shared);
    let join = std::thread::Builder::new()
        .name("sdroxide-fobos".into())
        .spawn(move || {
            run(params, ctrl_rx, rx_prod, Arc::clone(&thread_shared), ready_tx);
            thread_shared.alive.store(false, Ordering::Relaxed);
        })
        .map_err(|e| Error::NotFound(format!("could not start the Fobos thread: {e}")))?;

    match ready_rx.recv() {
        Ok(Ok(info)) => Ok(FobosHandle::from_parts(rx_cons, ctrl_tx, shared, join, info)),
        Ok(Err(e)) => {
            let _ = join.join();
            Err(e)
        }
        // The thread died before reporting; joining surfaces a panic message.
        Err(_) => {
            let _ = join.join();
            Err(Error::NotFound("the Fobos thread stopped before it opened the receiver".into()))
        }
    }
}

fn run(
    params: OpenParams,
    ctrl: Receiver<Ctrl>,
    mut rx: Producer<f32>,
    shared: Arc<Shared>,
    ready: crossbeam_channel::Sender<Result<DeviceInfo>>,
) {
    let (api, dev, chosen) = match device::open(&params.serial) {
        Ok(v) => v,
        Err(e) => {
            let _ = ready.send(Err(e));
            return;
        }
    };
    // SAFETY: `dev` was just opened above, belongs to `api`, and is not
    // touched again after `stop_sync`/`close` at the end of `pump_rf`/
    // `pump_hf`.
    let board = match unsafe { device::board_info(&api, dev) } {
        Ok(b) => b,
        Err(e) => {
            unsafe { device::close(&api, dev) };
            let _ = ready.send(Err(e));
            return;
        }
    };
    let rates_hz = match unsafe { device::samplerates(&api, dev) } {
        Ok(r) => r,
        Err(e) => {
            unsafe { device::close(&api, dev) };
            let _ = ready.send(Err(e));
            return;
        }
    };

    let rc = unsafe { (api.set_direct_sampling)(dev, c_int::from(params.port != Port::Rf)) };
    if rc != ffi::ERR_OK {
        tracing::warn!("fobos_rx_set_direct_sampling: {}", api.err_text(rc));
    }
    let rc = unsafe { (api.set_clk_source)(dev, c_int::from(params.clk_external)) };
    if rc != ffi::ERR_OK {
        tracing::warn!("fobos_rx_set_clk_source: {}", api.err_text(rc));
    }
    // LNA/VGA gain the tuner's front end provides — meaningless on the HF
    // ports, whose front end is powered down while direct sampling runs
    // (see the module doc).
    if params.port == Port::Rf {
        let rc = unsafe { (api.set_lna_gain)(dev, c_int::from(params.lna_gain)) };
        if rc != ffi::ERR_OK {
            tracing::warn!("fobos_rx_set_lna_gain: {}", api.err_text(rc));
        }
        let rc = unsafe { (api.set_vga_gain)(dev, c_int::from(params.vga_gain)) };
        if rc != ffi::ERR_OK {
            tracing::warn!("fobos_rx_set_vga_gain: {}", api.err_text(rc));
        }
    }

    let result = match params.port {
        Port::Rf => open_rf(&api, dev, &params, &rates_hz),
        Port::Hf1 | Port::Hf2 | Port::HfDual => open_hf(&api, dev, &params, &rates_hz),
    };
    let (adc_rate, out_rate) = match result {
        Ok(v) => v,
        Err(e) => {
            unsafe { device::close(&api, dev) };
            let _ = ready.send(Err(e));
            return;
        }
    };

    let label = format!(
        "Fobos SDR {} ({})",
        chosen.serial,
        match params.port {
            Port::Rf => "RF port",
            Port::Hf1 => "HF1",
            Port::Hf2 => "HF2",
            Port::HfDual => "HF1+HF2, diversity",
        }
    );
    shared.rate_milli_hz.store((out_rate * 1000.0) as u64, Ordering::Relaxed);
    let _ = ready.send(Ok(DeviceInfo {
        label: label.clone(),
        board: board.clone(),
        rates_hz: rates_hz.clone(),
        sample_rate_hz: out_rate,
        adc_rate_hz: adc_rate,
    }));

    let outcome = match params.port {
        Port::Rf => pump_rf(&api, dev, out_rate, &ctrl, &mut rx, &shared),
        // `rx` moves into a dedicated DDC thread for these two — see
        // `pump_hf`'s own doc comment for why.
        Port::Hf1 | Port::Hf2 => pump_hf(
            &api,
            dev,
            params.port,
            adc_rate,
            params.center_hz,
            out_rate,
            &ctrl,
            rx,
            &shared,
        ),
        Port::HfDual => {
            pump_hf_dual(&api, dev, adc_rate, params.center_hz, out_rate, &ctrl, rx, &shared)
        }
    };
    if let Err(e) = outcome {
        tracing::warn!("Fobos stream stopped: {e}");
    }
    let rc = unsafe { (api.stop_sync)(dev) };
    if rc != ffi::ERR_OK {
        tracing::debug!("fobos_rx_stop_sync: {}", api.err_text(rc));
    }
    unsafe { device::close(&api, dev) };
}

/// RF-port setup: the ADC rate the operator asked for, tuned by the mixer.
/// Returns `(adc_rate, out_rate)` — identical on this path.
fn open_rf(
    api: &ffi::Api,
    dev: ffi::Dev,
    params: &OpenParams,
    _rates_hz: &[f64],
) -> Result<(f64, f64)> {
    let mut actual_rate = 0.0f64;
    let rc = unsafe { (api.set_samplerate)(dev, params.sample_rate_hz, &mut actual_rate) };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_set_samplerate", api.err_text(rc)));
    }
    if (actual_rate - params.sample_rate_hz).abs() > 1.0 {
        tracing::info!(
            "Fobos: {:.3} Msps requested, {:.3} Msps actually set",
            params.sample_rate_hz / 1e6,
            actual_rate / 1e6,
        );
    }

    let mut actual_freq = 0.0f64;
    let rc = unsafe { (api.set_frequency)(dev, params.center_hz, &mut actual_freq) };
    if rc != ffi::ERR_OK {
        tracing::warn!("fobos_rx_set_frequency: {}", api.err_text(rc));
    }
    Ok((actual_rate, actual_rate))
}

/// HF-port setup, shared by `Port::Hf1`/`Port::Hf2`/`Port::HfDual` — the
/// ADC always runs at its own top rate (see the module doc — this is also
/// what sidesteps the low-rate snap-to-8-Msps finding, which was only ever
/// exercised on `Port::Rf`), and the requested rate instead picks `WbDdc`'s
/// bin count, the same bin count for both channels on `HfDual` so their two
/// `WbDdc`s stay in lock-step. Returns `(adc_rate, out_rate)`.
/// Real floor for the ADC's own rate while direct sampling — requests below
/// this snapped up to it on the unit this crate was verified against
/// (`fobos_rx_set_samplerate` returns success either way; only `actual`
/// says what really landed), so there is nothing to gain asking below it.
const HF_ADC_RATE_FLOOR: f64 = 8_000_000.0;

/// How much faster than the requested complex output the ADC runs while
/// direct sampling — headroom for the downconverter's own band selection
/// and edge taper. Not the ADC's own top rate: `open_hf` originally asked
/// for that unconditionally, on the assumption direct sampling required it,
/// which turned out to be untested rather than true — a direct check
/// (`fobos_rx_set_samplerate` with direct sampling already on) landed
/// exactly on 40/20/10 Msps when asked, snapping only below
/// [`HF_ADC_RATE_FLOOR`]. Always running the ADC at 80 Msps regardless of
/// what was actually wanted turned the raw hardware read into the
/// bottleneck: measured on real hardware, roughly 85-100 audible
/// discontinuities a second even after DDC compute moved to its own thread
/// ([`pump_hf`]'s own doc comment). A lower ADC rate shrinks both the raw
/// USB transfer and the downconverter's own forward-FFT work
/// proportionally; the coarse-tuning grid gets finer as a side effect
/// (`bin_hz` is `adc_rate / DDC_BLOCK`).
const HF_ADC_OVERSAMPLE: f64 = 8.0;

/// The one ADC rate confirmed broken for actually *streaming* direct
/// sampling on real hardware — not merely a `fobos_rx_set_samplerate` guess:
/// `fobos_rx_set_samplerate(40_000_000.0)` itself reports success and lands
/// exactly on it, but every attempt to then read from a stream opened at
/// that rate failed on the very first `fobos_rx_read_sync` call with
/// "Unsuppotred parameter or mode" (the vendor driver's own spelling) — 3
/// for 3, spread across two physical replugs, never once succeeding. That
/// is a different, more consistent failure than [`HF_STARTUP_READ_RETRIES`]
/// exists to paper over (which comes and goes at the *same* rate across
/// attempts); 40 Msps never once worked. It matters here specifically
/// because [`hf_adc_rate_wanted`]'s own 8x-oversample rule lands exactly on
/// it for a 5.0 Msps target — one of this receiver's own documented rates,
/// not an unusual request.
const HF_ADC_RATE_BROKEN: f64 = 40_000_000.0;

/// Where a request that would otherwise land on [`HF_ADC_RATE_BROKEN`] goes
/// instead — the only nearby rate with real, repeated evidence behind it on
/// *both* axes that matter (streams reliably, and sounds clean), found the
/// hard way by trying every plausible candidate live: 50 Msps streams fine
/// but produces audible distortion and static-like clicking on a real
/// listen (confirmed twice — once as the general HF floor, reverted, once
/// as this exact fallback); 25 Msps and 32 Msps both fail to stream at all,
/// even with [`HF_STARTUP_READ_RETRIES`]'s own retry exhausted, on repeated
/// isolated attempts; 20 Msps is the one rate confirmed clean and reliable
/// throughout this whole investigation — it's what a 2.5 Msps target has
/// always landed on, with no distortion ever reported at that setting. This
/// undershoots the oversampling a 5.0 Msps target's own 8x rule intended
/// (20 Msps of ADC for a 5.0 Msps output is 4x, not 8x) and re-caps the
/// receivable ceiling around 10 MHz the same way a narrow target always has
/// — a real cost, accepted deliberately: unreliable or audibly distorted
/// beats a narrower reachable band.
const HF_ADC_RATE_BROKEN_FALLBACK: f64 = 20_000_000.0;

/// The ADC rate `open_hf` asks for, given what the caller actually wants out
/// and the fastest rate this receiver offers — a pure function so
/// [`the_wanted_hf_adc_rate_stays_far_below_the_top_rate_for_a_modest_target`]
/// can check it without hardware.
fn hf_adc_rate_wanted(target_out_hz: f64, top_rate_hz: f64) -> f64 {
    // `f64::clamp` panics if its own min exceeds its max — real, not just
    // defensive: a `top_rate_hz` below `HF_ADC_RATE_FLOOR` (a genuinely
    // narrower future unit, or exactly the regression test this guards)
    // must not turn the floor into an impossible request.
    let floor = HF_ADC_RATE_FLOOR.min(top_rate_hz);
    let wanted = (target_out_hz.max(1.0) * HF_ADC_OVERSAMPLE).clamp(floor, top_rate_hz);
    if (wanted - HF_ADC_RATE_BROKEN).abs() < 1.0 {
        HF_ADC_RATE_BROKEN_FALLBACK.min(top_rate_hz)
    } else {
        wanted
    }
}

fn open_hf(
    api: &ffi::Api,
    dev: ffi::Dev,
    params: &OpenParams,
    rates_hz: &[f64],
) -> Result<(f64, f64)> {
    let top_rate = rates_hz.iter().copied().fold(0.0f64, f64::max);
    let top_rate = if top_rate > 0.0 { top_rate } else { 80_000_000.0 };
    let wanted = hf_adc_rate_wanted(params.sample_rate_hz, top_rate);
    let mut actual_rate = 0.0f64;
    let rc = unsafe { (api.set_samplerate)(dev, wanted, &mut actual_rate) };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_set_samplerate", api.err_text(rc)));
    }
    if (actual_rate - wanted).abs() > 1.0 {
        tracing::debug!(
            "Fobos: asked the ADC for {:.3} Msps, got {:.3} Msps instead",
            wanted / 1e6,
            actual_rate / 1e6,
        );
    }
    let bins = pick_bins(actual_rate, params.sample_rate_hz);
    let out_rate = actual_rate * bins as f64 / DDC_BLOCK as f64;
    Ok((actual_rate, out_rate))
}

fn pump_rf(
    api: &ffi::Api,
    dev: ffi::Dev,
    initial_rate_hz: f64,
    ctrl: &Receiver<Ctrl>,
    rx: &mut Producer<f32>,
    shared: &Arc<Shared>,
) -> Result<()> {
    let mut rate_hz = initial_rate_hz;
    let mut buf_len = buf_len_for(rate_hz);
    let rc = unsafe { (api.start_sync)(dev, buf_len) };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_start_sync", api.err_text(rc)));
    }
    tracing::info!(
        "Fobos streaming: {:.3} Msps, {buf_len} complex samples/block ({:.1} blocks/s)",
        rate_hz / 1e6,
        BLOCKS_PER_SEC,
    );

    let mut scratch: Vec<f32> = vec![0.0; buf_len as usize * 2];
    let mut stats = RxStats::new();
    let started = Instant::now();

    loop {
        // 1. Collapse the whole control channel, then apply each field
        //    once.
        let mut pending = Pending::default();
        while let Ok(c) = ctrl.try_recv() {
            pending.absorb(c);
        }
        if pending.shutdown {
            break;
        }
        if !pending.is_empty() {
            if let Some(hz) = pending.rate {
                // Confirmed working against real hardware above roughly
                // 8 Msps — see the module doc.
                let rc = unsafe { (api.stop_sync)(dev) };
                if rc != ffi::ERR_OK {
                    tracing::warn!("fobos_rx_stop_sync (rate change): {}", api.err_text(rc));
                }
                let mut actual = 0.0f64;
                let rc = unsafe { (api.set_samplerate)(dev, hz, &mut actual) };
                if rc != ffi::ERR_OK {
                    tracing::warn!("Fobos: rate change failed: {}", api.err_text(rc));
                    // The old rate is still what's programmed; carry on with
                    // it rather than leaving the stream stopped.
                    actual = rate_hz;
                }
                rate_hz = actual;
                buf_len = buf_len_for(rate_hz);
                scratch.resize(buf_len as usize * 2, 0.0);
                shared.rate_milli_hz.store((rate_hz * 1000.0) as u64, Ordering::Relaxed);
                let rc = unsafe { (api.start_sync)(dev, buf_len) };
                if rc != ffi::ERR_OK {
                    return Err(Error::api("fobos_rx_start_sync", api.err_text(rc)));
                }
                tracing::info!("Fobos: rate now {:.3} Msps", rate_hz / 1e6);
            }
            if let Some(hz) = pending.center {
                let mut actual = 0.0f64;
                let rc = unsafe { (api.set_frequency)(dev, hz, &mut actual) };
                if rc != ffi::ERR_OK {
                    tracing::warn!("fobos_rx_set_frequency: {}", api.err_text(rc));
                } else if (actual - hz).abs() > 1.0 {
                    tracing::debug!("Fobos: {hz:.0} Hz requested, {actual:.0} Hz actually set");
                }
            }
            if let Some(v) = pending.lna_gain {
                let rc = unsafe { (api.set_lna_gain)(dev, c_int::from(v)) };
                if rc != ffi::ERR_OK {
                    tracing::warn!("fobos_rx_set_lna_gain: {}", api.err_text(rc));
                }
            }
            if let Some(v) = pending.vga_gain {
                let rc = unsafe { (api.set_vga_gain)(dev, c_int::from(v)) };
                if rc != ffi::ERR_OK {
                    tracing::warn!("fobos_rx_set_vga_gain: {}", api.err_text(rc));
                }
            }
            if let Some(on) = pending.clk_external {
                let rc = unsafe { (api.set_clk_source)(dev, c_int::from(on)) };
                if rc != ffi::ERR_OK {
                    tracing::warn!("fobos_rx_set_clk_source: {}", api.err_text(rc));
                }
            }
        }

        // 2. One blocking read. At 400 blocks/sec this is at most a few ms,
        //    which is what makes draining control once per iteration (above)
        //    responsive enough without a separate timeout/select.
        let mut actual: u32 = 0;
        let rc = unsafe { (api.read_sync)(dev, scratch.as_mut_ptr(), &mut actual) };
        if rc != ffi::ERR_OK {
            // Treated as fatal to the stream rather than retried — nothing
            // in this API distinguishes a transient error from a dead
            // device.
            stats.on_error(&api.err_text(rc));
            tracing::warn!("Fobos: stream ended: {}", stats.summary());
            return Err(Error::api("fobos_rx_read_sync", api.err_text(rc)));
        }
        let pairs = (actual as usize).min(buf_len as usize);
        if pairs > 0 {
            stats.on_iq(pairs);
            push_iq(
                rx,
                &scratch[..pairs * 2],
                2,
                &mut stats,
                shared.rx_paused.load(Ordering::Relaxed),
            );
            shared.last_rx_ms.store(started.elapsed().as_millis() as u64, Ordering::Relaxed);
        }

        stats.tick();
    }

    tracing::info!("Fobos: stream ended: {}", stats.summary());
    Ok(())
}

/// The downconverter selects a band directly out of a real FFT's
/// non-negative bins (see `sdroxide_dsp::WbDdc`'s own module doc), so it can
/// never centre closer to DC than half its output rate — `set_center_hz`
/// clamps silently rather than erroring, since a live retune has nowhere to
/// report a `Result` to. Nothing upstream of this call knows that happened
/// unless something here says so, which is worth doing loudly: a clamped
/// tune keeps streaming, keeps naming itself after the frequency that was
/// asked for, and is actually receiving something else entirely.
fn warn_if_center_clamped(ddc: &WbDdc, requested_hz: f64, port: Port) {
    let achieved = ddc.center_hz();
    if (achieved - requested_hz).abs() > 1.0 {
        tracing::warn!(
            "Fobos {port:?}: {requested_hz:.0} Hz requested is below this port's reachable \
             floor at the current bandwidth ({:.0} Hz, out_rate {:.0} Hz) — actually tuned to \
             {achieved:.0} Hz instead. Lower the sample rate in Settings → Radio to reach lower \
             frequencies, or retune above the floor.",
            ddc.out_rate() / 2.0,
            ddc.out_rate(),
        );
    }
}

/// The half of [`pump_hf`]/[`pump_hf_dual`] that only ever blocks on
/// `fobos_rx_read_sync` and hands the raw block off — see [`pump_hf`]'s own
/// doc comment for why this is split from the DDC at all. Port-agnostic: a
/// raw block is the untouched `read_sync` buffer regardless of whether the
/// DDC side is about to read one lane out of it or both, so `Hf1`/`Hf2`/
/// `HfDual` all share this one loop.
/// How many extra tries [`hf_pump`] gives `fobos_rx_read_sync` before giving
/// up, but only before the stream has ever produced a real read — a real
/// finding from live hardware, not a defensive guess: the very *first* read
/// after `fobos_rx_start_sync` failed outright on this unit on several
/// separate opens — "No device specified, dev == NUL" — at ADC rates (20,
/// 32, 50 Msps) that streamed perfectly cleanly on other attempts, including
/// the first attempt right after a fresh replug. That shape — the same rate
/// working sometimes and not others, always on the very first read — is a
/// startup race between the driver actually being ready and this call,
/// not a rate the hardware cannot do. A failure once real samples have
/// already arrived is treated as before (fatal): that is a genuine
/// mid-stream fault, not the same startup window.
const HF_STARTUP_READ_RETRIES: u32 = 5;

/// Gap between [`HF_STARTUP_READ_RETRIES`] attempts. Short: the race this
/// covers resolved within tens of milliseconds every time it was seen to
/// resolve at all, so five tries at this spacing is real headroom without
/// turning a genuine fault into a slow one.
const HF_STARTUP_READ_RETRY_DELAY: Duration = Duration::from_millis(40);

fn hf_pump(
    api: &ffi::Api,
    dev: ffi::Dev,
    buf_len: u32,
    ctrl: &Receiver<Ctrl>,
    full: &Sender<Vec<f32>>,
    empty: &Receiver<Vec<f32>>,
    retune: &Sender<f64>,
) -> Result<()> {
    let mut dropped = 0u64;
    let mut streamed_yet = false;
    loop {
        let mut pending = Pending::default();
        while let Ok(c) = ctrl.try_recv() {
            pending.absorb(c);
        }
        if pending.shutdown {
            break;
        }
        if let Some(hz) = pending.rate {
            tracing::debug!(
                "Fobos: rate change to {:.3} Msps ignored on the HF path (not yet \
                 supported) — {hz:.0} Hz requested",
                hz / 1e6
            );
        }
        if let Some(hz) = pending.center {
            // The DDC(s) live on the converter thread; retuning is its job,
            // not this one's — see `warn_if_center_clamped`'s own call
            // sites over there.
            let _ = retune.send(hz);
        }
        if pending.lna_gain.is_some() || pending.vga_gain.is_some() {
            tracing::debug!("Fobos: LNA/VGA gain has no effect on the HF path — ignored");
        }
        if let Some(on) = pending.clk_external {
            let rc = unsafe { (api.set_clk_source)(dev, c_int::from(on)) };
            if rc != ffi::ERR_OK {
                tracing::warn!("fobos_rx_set_clk_source: {}", api.err_text(rc));
            }
        }

        // Reuse a buffer the converter has handed back rather than always
        // allocating fresh — same `empty`-queue shape `sdroxide-rx888`'s own
        // USB pump uses for the identical reason.
        let mut buf = empty.try_recv().unwrap_or_default();
        buf.resize(buf_len as usize * 2, 0.0);
        let mut actual: u32 = 0;
        let mut rc = unsafe { (api.read_sync)(dev, buf.as_mut_ptr(), &mut actual) };
        if rc != ffi::ERR_OK && !streamed_yet {
            // See `HF_STARTUP_READ_RETRIES`'s own doc comment: a real
            // startup race, seen on real hardware at rates that otherwise
            // stream cleanly, not a rate this receiver genuinely cannot do.
            for attempt in 1..=HF_STARTUP_READ_RETRIES {
                std::thread::sleep(HF_STARTUP_READ_RETRY_DELAY);
                rc = unsafe { (api.read_sync)(dev, buf.as_mut_ptr(), &mut actual) };
                if rc == ffi::ERR_OK {
                    tracing::debug!(
                        "Fobos: fobos_rx_read_sync recovered on startup retry {attempt}/{HF_STARTUP_READ_RETRIES}"
                    );
                    break;
                }
            }
        }
        if rc != ffi::ERR_OK {
            tracing::warn!("Fobos: fobos_rx_read_sync failed, stream ending: {}", api.err_text(rc));
            return Err(Error::api("fobos_rx_read_sync", api.err_text(rc)));
        }
        streamed_yet = true;
        let n = (actual as usize).min(buf_len as usize);
        if n == 0 {
            continue;
        }
        buf.truncate(n * 2);
        match full.try_send(buf) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) => {
                // The DDC is behind. Dropping the block here, rather than
                // blocking this thread until there's room, is what keeps
                // `read_sync` being called on the hardware's own pace
                // instead of falling further behind it every iteration —
                // the whole reason this is two threads and not one.
                dropped += 1;
                if dropped.is_power_of_two() {
                    tracing::warn!("Fobos: HF DDC thread behind, dropped {dropped} raw block(s)");
                }
            }
            Err(TrySendError::Disconnected(_)) => return Ok(()),
        }
    }
    Ok(())
}

/// The HF path: read two interleaved real channels, keep the selected one,
/// run it through [`WbDdc`] for complex baseband. No rate-change support —
/// the ADC rate is whatever [`hf_adc_rate_wanted`] settled on at open time
/// and stays there for the life of the stream, and changing the *output*
/// rate would mean rebuilding the FFT plans and the bin count with them.
/// That is not a gap: a rate change reaches this backend as a reopen, never
/// as a live control (see [`Ctrl::Rate`]'s own doc comment). And no
/// `fobos_rx_set_frequency` call at all: tuning is `WbDdc::set_center_hz`
/// alone.
///
/// The blocking hardware read ([`hf_pump`]) and the DDC's own FFT work
/// ([`hf_convert_loop`]) run on separate threads rather than back to back on
/// one, handed off through a small pool of reused buffers. Measured on real
/// hardware: DDC compute on a full read's worth of samples (dozens of
/// 8192-point FFTs) is not free, and running it between one
/// `fobos_rx_read_sync` call and the next delayed the next call by close to
/// as much again — the loop landed at not much over half the throughput
/// `BLOCKS_PER_SEC`/`out_rate` both assume, heard as choppy, gappy audio.
/// Splitting the two lets them overlap instead of serialising — the same
/// split `sdroxide-rx888` already uses, for the identical reason, in its own
/// `stream.rs`.
#[allow(clippy::too_many_arguments)]
fn pump_hf(
    api: &ffi::Api,
    dev: ffi::Dev,
    port: Port,
    adc_rate_hz: f64,
    initial_center_hz: f64,
    out_rate_hz: f64,
    ctrl: &Receiver<Ctrl>,
    rx: Producer<f32>,
    shared: &Arc<Shared>,
) -> Result<()> {
    let bins = pick_bins(adc_rate_hz, out_rate_hz);
    let mut ddc = WbDdc::new(adc_rate_hz, DDC_BLOCK, bins);
    ddc.set_center_hz(initial_center_hz);
    warn_if_center_clamped(&ddc, initial_center_hz, port);

    let buf_len = buf_len_for(adc_rate_hz);
    let rc = unsafe { (api.start_sync)(dev, buf_len) };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_start_sync", api.err_text(rc)));
    }
    tracing::info!(
        "Fobos streaming: {} direct sampling, {:.3} Msps real in -> {:.4} Msps complex out \
         ({buf_len} samples/block, {:.1} blocks/s)",
        if port == Port::Hf1 { "HF1" } else { "HF2" },
        adc_rate_hz / 1e6,
        ddc.out_rate() / 1e6,
        BLOCKS_PER_SEC,
    );

    let (full_tx, full_rx) = crossbeam_channel::bounded::<Vec<f32>>(HF_HANDOFF_DEPTH);
    let (empty_tx, empty_rx) = crossbeam_channel::bounded::<Vec<f32>>(HF_HANDOFF_DEPTH + 2);
    let (retune_tx, retune_rx) = crossbeam_channel::unbounded::<f64>();
    let conv_shared = Arc::clone(shared);
    let converter = std::thread::Builder::new()
        .name("sdroxide-fobos-hf-ddc".into())
        .spawn(move || hf_convert_loop(ddc, port, full_rx, empty_tx, rx, conv_shared, retune_rx));
    let converter = match converter {
        Ok(t) => t,
        Err(e) => {
            let rc = unsafe { (api.stop_sync)(dev) };
            if rc != ffi::ERR_OK {
                tracing::debug!("fobos_rx_stop_sync: {}", api.err_text(rc));
            }
            return Err(Error::NotFound(format!("could not start the Fobos HF DDC thread: {e}")));
        }
    };

    let outcome = hf_pump(api, dev, buf_len, ctrl, &full_tx, &empty_rx, &retune_tx);
    // Dropping the sender is what ends `hf_convert_loop`'s own `while let
    // Ok(buf) = full.recv()` — it has to happen before the join below or
    // shutdown deadlocks waiting on itself.
    drop(full_tx);
    let _ = converter.join();
    outcome
}

/// The converter half of [`pump_hf`]: owns the one [`WbDdc`], turns each raw
/// block [`hf_pump`] hands over into complex baseband, and pushes it into
/// the ring the rest of the program reads from.
fn hf_convert_loop(
    mut ddc: WbDdc,
    port: Port,
    full: Receiver<Vec<f32>>,
    empty: Sender<Vec<f32>>,
    mut rx: Producer<f32>,
    shared: Arc<Shared>,
    retune: Receiver<f64>,
) {
    let mut real: Vec<f32> = Vec::new();
    let mut cplx: Vec<Complex32> = Vec::new();
    let mut inter: Vec<f32> = Vec::new();
    let mut stats = RxStats::new();
    let started = Instant::now();

    while let Ok(buf) = full.recv() {
        // Last retune wins — collapsing the same way `hf_pump`'s own control
        // channel does, and for the same reason: only where the dial ends
        // up matters, not every point it passed through.
        let mut want = None;
        while let Ok(hz) = retune.try_recv() {
            want = Some(hz);
        }
        if let Some(hz) = want {
            ddc.set_center_hz(hz);
            warn_if_center_clamped(&ddc, hz, port);
        }

        // HF1 lives in `.re` (even offsets), HF2 in `.im` (odd) — see the
        // module doc. `as_chunks` rather than a strided iterator so the
        // extraction loop stays a straight-line stride the compiler can
        // vectorise, matching `sdroxide-rx888::convert`'s own reasoning for
        // the same shape of loop.
        real.clear();
        real.reserve(buf.len() / 2);
        let offset = if port == Port::Hf1 { 0 } else { 1 };
        for pair in buf.as_chunks::<2>().0 {
            real.push(pair[offset]);
        }

        cplx.clear();
        ddc.process(&real, &mut cplx);

        if !cplx.is_empty() {
            inter.clear();
            inter.reserve(cplx.len() * 2);
            for v in &cplx {
                inter.push(v.re);
                inter.push(v.im);
            }
            stats.on_iq(cplx.len());
            push_iq(&mut rx, &inter, 2, &mut stats, shared.rx_paused.load(Ordering::Relaxed));
            shared.last_rx_ms.store(started.elapsed().as_millis() as u64, Ordering::Relaxed);
        }

        // Give the buffer back — a full return queue just means `hf_pump`
        // has plenty, so a dropped `try_send` here is harmless.
        let _ = empty.try_send(buf);
        stats.tick();
    }

    tracing::info!("Fobos: stream ended: {}", stats.summary());
}

/// The dual-HF path: read both interleaved real channels every block, run
/// each through its own [`WbDdc`], and push both complex streams into the
/// same four-lane ring — see [`ring_for_pair`]'s own doc comment for why one
/// ring rather than two. The two `WbDdc`s are built with identical
/// parameters and fed the identical input length every call (both come from
/// the same raw block), so their outputs are the same length every block by
/// construction — [`WbDdc::process`] is a deterministic function of input
/// length and each instance's own prior state, and both start fresh and
/// stay in lock-step. Same caveats as [`pump_hf`]: no live rate change, no
/// `fobos_rx_set_frequency` call, tuning is both `WbDdc`s' `set_center_hz`
/// together — and the same read/DDC thread split, for the same reason, now
/// carrying twice the DDC compute per block.
///
/// No diversity combining here — that reads both channels back out through
/// [`FobosHandle::rx_read_pair`] and lives in `src/fobos_source.rs`, same
/// layer `sdroxide-sdrplay`'s own dual-tuner combining lives at.
#[allow(clippy::too_many_arguments)]
fn pump_hf_dual(
    api: &ffi::Api,
    dev: ffi::Dev,
    adc_rate_hz: f64,
    initial_center_hz: f64,
    out_rate_hz: f64,
    ctrl: &Receiver<Ctrl>,
    rx: Producer<f32>,
    shared: &Arc<Shared>,
) -> Result<()> {
    let bins = pick_bins(adc_rate_hz, out_rate_hz);
    let mut ddc_main = WbDdc::new(adc_rate_hz, DDC_BLOCK, bins);
    let mut ddc_aux = WbDdc::new(adc_rate_hz, DDC_BLOCK, bins);
    ddc_main.set_center_hz(initial_center_hz);
    ddc_aux.set_center_hz(initial_center_hz);
    // Both `WbDdc`s are built identically and tuned together, so one check
    // speaks for both.
    warn_if_center_clamped(&ddc_main, initial_center_hz, Port::HfDual);

    let buf_len = buf_len_for(adc_rate_hz);
    let rc = unsafe { (api.start_sync)(dev, buf_len) };
    if rc != ffi::ERR_OK {
        return Err(Error::api("fobos_rx_start_sync", api.err_text(rc)));
    }
    tracing::info!(
        "Fobos streaming: HF1+HF2 direct sampling, {:.3} Msps real in -> {:.4} Msps complex \
         out each ({buf_len} samples/block, {:.1} blocks/s)",
        adc_rate_hz / 1e6,
        ddc_main.out_rate() / 1e6,
        BLOCKS_PER_SEC,
    );

    let (full_tx, full_rx) = crossbeam_channel::bounded::<Vec<f32>>(HF_HANDOFF_DEPTH);
    let (empty_tx, empty_rx) = crossbeam_channel::bounded::<Vec<f32>>(HF_HANDOFF_DEPTH + 2);
    let (retune_tx, retune_rx) = crossbeam_channel::unbounded::<f64>();
    let conv_shared = Arc::clone(shared);
    let converter =
        std::thread::Builder::new().name("sdroxide-fobos-hf-ddc".into()).spawn(move || {
            hf_dual_convert_loop(ddc_main, ddc_aux, full_rx, empty_tx, rx, conv_shared, retune_rx)
        });
    let converter = match converter {
        Ok(t) => t,
        Err(e) => {
            let rc = unsafe { (api.stop_sync)(dev) };
            if rc != ffi::ERR_OK {
                tracing::debug!("fobos_rx_stop_sync: {}", api.err_text(rc));
            }
            return Err(Error::NotFound(format!("could not start the Fobos HF DDC thread: {e}")));
        }
    };

    let outcome = hf_pump(api, dev, buf_len, ctrl, &full_tx, &empty_rx, &retune_tx);
    drop(full_tx);
    let _ = converter.join();
    outcome
}

/// The converter half of [`pump_hf_dual`] — see [`hf_convert_loop`]'s own
/// doc comment for the shape this mirrors; the difference here is running
/// two `WbDdc`s per block and interleaving four floats per complex pair
/// (main.re, main.im, aux.re, aux.im) instead of two.
fn hf_dual_convert_loop(
    mut ddc_main: WbDdc,
    mut ddc_aux: WbDdc,
    full: Receiver<Vec<f32>>,
    empty: Sender<Vec<f32>>,
    mut rx: Producer<f32>,
    shared: Arc<Shared>,
    retune: Receiver<f64>,
) {
    let mut real_main: Vec<f32> = Vec::new();
    let mut real_aux: Vec<f32> = Vec::new();
    let mut cplx_main: Vec<Complex32> = Vec::new();
    let mut cplx_aux: Vec<Complex32> = Vec::new();
    let mut inter: Vec<f32> = Vec::new();
    let mut stats = RxStats::new();
    let started = Instant::now();

    while let Ok(buf) = full.recv() {
        let mut want = None;
        while let Ok(hz) = retune.try_recv() {
            want = Some(hz);
        }
        if let Some(hz) = want {
            // Both channels must follow the same tuning or the combining
            // weight is meaningless — they would be looking at different
            // signals.
            ddc_main.set_center_hz(hz);
            ddc_aux.set_center_hz(hz);
            warn_if_center_clamped(&ddc_main, hz, Port::HfDual);
        }

        let n = buf.len() / 2;
        real_main.clear();
        real_aux.clear();
        real_main.reserve(n);
        real_aux.reserve(n);
        for pair in buf.as_chunks::<2>().0 {
            real_main.push(pair[0]);
            real_aux.push(pair[1]);
        }

        cplx_main.clear();
        cplx_aux.clear();
        ddc_main.process(&real_main, &mut cplx_main);
        ddc_aux.process(&real_aux, &mut cplx_aux);
        debug_assert_eq!(
            cplx_main.len(),
            cplx_aux.len(),
            "two identically-configured WbDdcs fed equal-length input every call must \
             produce equal-length output — see this fn's own doc comment"
        );

        if !cplx_main.is_empty() && !cplx_aux.is_empty() {
            let pairs = cplx_main.len().min(cplx_aux.len());
            inter.clear();
            inter.reserve(pairs * 4);
            for i in 0..pairs {
                inter.push(cplx_main[i].re);
                inter.push(cplx_main[i].im);
                inter.push(cplx_aux[i].re);
                inter.push(cplx_aux[i].im);
            }
            stats.on_iq(pairs);
            push_iq(&mut rx, &inter, 4, &mut stats, shared.rx_paused.load(Ordering::Relaxed));
            shared.last_rx_ms.store(started.elapsed().as_millis() as u64, Ordering::Relaxed);
        }

        let _ = empty.try_send(buf);
        stats.tick();
    }

    tracing::info!("Fobos: stream ended: {}", stats.summary());
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buf_len_for_gives_400_blocks_a_second_at_any_rate() {
        for rate in [1_250_000.0, 8_000_000.0, 50_000_000.0, 80_000_000.0] {
            let len = buf_len_for(rate);
            assert_eq!(len, (rate / BLOCKS_PER_SEC) as u32, "{rate}");
            assert!(len > 0);
        }
    }

    /// Verified against the real unit this crate was built against: 80 Msps
    /// in, 2.0 Msps requested landed on 2.5 Msps out — `pick_bins` must
    /// reproduce that snap deterministically, not just "some" power of two.
    #[test]
    fn pick_bins_reproduces_the_real_hardware_run() {
        let bins = pick_bins(80_000_000.0, 2_000_000.0);
        let out_rate = 80_000_000.0 * bins as f64 / DDC_BLOCK as f64;
        assert_eq!(bins, 256);
        assert!((out_rate - 2_500_000.0).abs() < 1.0, "{out_rate}");
    }

    /// A power of two, never below the smallest usable channel or above
    /// half the block — the same bounds `sdroxide-rx888`'s own
    /// `sanitize_ddc_bins` enforces, since `WbDdc::new` panics outside them.
    #[test]
    fn pick_bins_always_stays_in_bounds() {
        for target in [1.0, 1_000.0, 100_000.0, 5_000_000.0, 40_000_000.0, 1e12] {
            let bins = pick_bins(80_000_000.0, target);
            assert!(bins.is_power_of_two(), "{target}: {bins}");
            assert!((64..=DDC_BLOCK / 2).contains(&bins), "{target}: {bins}");
        }
    }

    /// `WbDdc::set_center_hz` can never centre closer to DC than half its own
    /// output rate — real, tested, documented behaviour in `sdroxide-dsp`
    /// itself, not a bug to fix here. What *was* a real bug: this backend's
    /// old default target rate (2 Msps, `out_rate` 2.5 Msps) put that floor
    /// at 1.25 MHz, silently making the entire AM/mediumwave broadcast band
    /// (530 kHz–1.7 MHz) untunable on the HF ports — a dial set to, say,
    /// 820 kHz would clamp to 1.25 MHz without any indication, decode
    /// whatever was actually there, and label it 820 kHz regardless.
    ///
    /// `sdroxide_types::FobosConfig::default().sample_rate_hz` must stay at
    /// or below this value (625 kHz is duplicated here rather than pulled
    /// in, since this crate deliberately has no dependency on
    /// `sdroxide-types` — see `Cargo.toml`'s own comment on why).
    #[test]
    fn the_default_hf_rate_keeps_the_whole_am_broadcast_band_reachable() {
        let default_target_hz = 625_000.0;
        let bins = pick_bins(80_000_000.0, default_target_hz);
        let mut ddc = WbDdc::new(80_000_000.0, DDC_BLOCK, bins);
        for hz in [530_000.0, 820_000.0, 1_000_000.0, 1_700_000.0] {
            ddc.set_center_hz(hz);
            assert!(
                (ddc.center_hz() - hz).abs() < 1.0,
                "{hz} Hz should be exactly reachable at the default rate, landed on {} instead",
                ddc.center_hz()
            );
        }
    }

    /// The old default (2 Msps target, 2.5 Msps out) really did clamp 820
    /// kHz away from itself — pinned here so a future change to
    /// `pick_bins`/`WbDdc` that reintroduces the old floor by accident is
    /// caught by the AM-band test above rather than by another bug report.
    #[test]
    fn the_old_default_hf_rate_really_did_clamp_820_khz() {
        let old_target_hz = 2_000_000.0;
        let bins = pick_bins(80_000_000.0, old_target_hz);
        let mut ddc = WbDdc::new(80_000_000.0, DDC_BLOCK, bins);
        ddc.set_center_hz(820_000.0);
        assert!(
            (ddc.center_hz() - 820_000.0).abs() > 100_000.0,
            "expected the old default to clamp 820 kHz well away from itself, landed on {}",
            ddc.center_hz()
        );
    }

    /// Real, measured cost of `open_hf` always asking for the ADC's own top
    /// rate: on real hardware, ~85-100 audible discontinuities a second at
    /// the default HF bandwidth (625 kHz target — 80 Msps real in is 128x
    /// oversampled for that), even with DDC compute already split onto its
    /// own thread. `hf_adc_rate_wanted` must stay well under `top_rate_hz`
    /// for a modest target so the raw hardware read isn't forced to run ten
    /// times faster than anything downstream needs.
    #[test]
    fn the_wanted_hf_adc_rate_stays_far_below_the_top_rate_for_a_modest_target() {
        let wanted = hf_adc_rate_wanted(625_000.0, 80_000_000.0);
        assert!(wanted < 10_000_000.0, "625 kHz target asked for {wanted} Hz out of 80 Msps");
        assert_eq!(wanted, HF_ADC_RATE_FLOOR, "should land exactly on the floor at this target");
    }

    /// A wide target (a big panadapter view, or someone up near the top of
    /// the HF/6 m range) still gets proportionally more ADC rate — capped at
    /// what the receiver actually offers, never asked for more than that.
    #[test]
    fn the_wanted_hf_adc_rate_scales_up_for_a_wide_target_but_never_past_the_top_rate() {
        assert!(hf_adc_rate_wanted(4_000_000.0, 80_000_000.0) > HF_ADC_RATE_FLOOR);
        assert_eq!(hf_adc_rate_wanted(50_000_000.0, 80_000_000.0), 80_000_000.0);
    }

    /// The bug this exists to fix: 5.0 Msps is one of this receiver's own
    /// documented rates, and the 8x-oversample rule lands its ADC request
    /// exactly on the one rate real hardware never once streamed from — see
    /// `HF_ADC_RATE_BROKEN`'s own doc comment. The fix must steer clear of
    /// it, not merely avoid crashing on it.
    #[test]
    fn the_wanted_hf_adc_rate_steers_clear_of_the_confirmed_broken_rate() {
        let wanted = hf_adc_rate_wanted(5_000_000.0, 80_000_000.0);
        assert_ne!(wanted, HF_ADC_RATE_BROKEN, "5.0 Msps must not land on the broken 40 Msps");
        assert_eq!(wanted, HF_ADC_RATE_BROKEN_FALLBACK);
    }

    /// A defensive regression, not a behavioural claim about a real
    /// receiver: `top_rate_hz` below `HF_ADC_RATE_FLOOR` must not turn the
    /// floor into an impossible `min > max` clamp and panic — the
    /// receiver's own top rate always wins over the floor. (The fallback's
    /// own "what if it exceeds top_rate_hz" case, tested here before
    /// `HF_ADC_RATE_BROKEN_FALLBACK` dropped to 20 Msps, is no longer
    /// reachable: 40 Msps only survives unclamped into the broken-rate
    /// check at all when `top_rate_hz` is at least 40 Msps, which is
    /// already well above the fallback.)
    #[test]
    fn a_top_rate_below_the_floor_never_panics() {
        let wanted = hf_adc_rate_wanted(5_000_000.0, 5_000_000.0);
        assert_eq!(wanted, 5_000_000.0);
    }
}
