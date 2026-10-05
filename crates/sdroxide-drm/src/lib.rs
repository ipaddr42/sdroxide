//! Digital Radio Mondiale reception, over the **Dream** receiver.
//!
//! DRM is a shortwave broadcast system: OFDM, a few hundred carriers in 9 or
//! 10 kHz, carrying an AAC-coded programme plus a service label, a scrolling
//! text message and the broadcaster's clock. Decoding it end to end means
//! acquisition, channel estimation, MLC/QAM demapping, Viterbi decoding, the
//! FAC/SDC/MSC multiplex and finally the audio codec — which is why this wraps
//! Dream (<https://sourceforge.net/projects/drm/>) rather than reimplementing
//! any of it. The sources are vendored at `vendor/dream`; `vendor/PROVENANCE.md`
//! there lists the three changes made to them.
//!
//! The shape follows [`sdroxide_rade`]: a C++ shim reduces the upstream library
//! to a handful of C calls, `build.rs` compiles and binds it, and a worker
//! thread drives it so no decode ever runs on the audio path.
//!
//! Three layers live here:
//!
//! * [`Ring`] — the sample queues either side of the decoder, and the only
//!   thing that crosses threads.
//! * [`Decoder`] — one Dream receiver. Every method must be called on the
//!   thread that built it.
//! * [`DrmWorker`] — the two above driven on a dedicated thread, which is how
//!   [`DrmDemod`] uses them.
//!
//! Where openwebrx pipes 48 kHz zero-IF I/Q into a `dream` subprocess, this
//! feeds the same samples to the same code in-process — see [`DrmDemod`] for
//! the one thing that costs, which is the intermediate frequency Dream insists
//! on.

#[cfg(test)]
mod decoder_tests;
mod demod;
#[cfg(test)]
mod fftw_tests;
mod sys;
mod worker;

pub use demod::{DRM_IF_OFFSET_HZ, DrmDemod};
pub use worker::DrmWorker;

// Named so its faad2 archive is linked: Dream calls `NeAACDec*` from C, which
// Rust cannot see.
use sdroxide_faad2 as _;

use std::ffi::CStr;

use sdroxide_types::{
    DrmChannel, DrmCodec, DrmConstellation, DrmRobustness, DrmService, DrmStatus, DrmSync, DrmTime,
    spectrum_occupancy_khz,
};

/// The sample rate Dream is driven at. It accepts 24, 48, 96 and 192 kHz and
/// snaps anything else to the nearest, so the channel is resampled to land on
/// one exactly rather than left to be rounded.
pub const SIGNAL_RATE: f64 = 48_000.0;

/// The rate decoded audio comes back at.
pub const AUDIO_RATE: f64 = 48_000.0;

/// Most constellation points read back per update.
///
/// The MSC carries a couple of thousand cells a frame. Every one of them on a
/// plot a few hundred pixels across is ink on ink, and it is also several times
/// the wire cost — this is a scatter plot, where the shape of the cloud is the
/// information and the sample is as good as the population.
pub const CONSTELLATION_POINTS: usize = 512;

/// Errors crossing the C boundary.
#[derive(Debug, thiserror::Error)]
pub enum DrmError {
    /// `sdrx_drm_new` returned NULL: the receiver's constructor threw.
    #[error("the DRM receiver could not be created")]
    OpenFailed,
}

/// The sample queues between the host and the decoder.
///
/// Input holds interleaved I/Q as `i16` pairs; output holds interleaved stereo
/// audio the same way. Pushing never blocks — a decoder that has fallen behind
/// loses the oldest samples rather than stalling the receive chain — while the
/// decoder's own read blocks until it has a whole block, which is what Dream's
/// sound-card interface requires.
pub struct Ring(*mut sys::sdrx_drm_ring);

// The C side guards both queues with a mutex and a condition variable; that is
// the whole point of the type.
unsafe impl Send for Ring {}
unsafe impl Sync for Ring {}

impl Ring {
    /// Capacities in samples — two per interleaved frame.
    ///
    /// A failed allocation leaves a null handle rather than aborting; every
    /// call below tolerates one, and [`Decoder::new`] then refuses to open.
    pub fn new(in_capacity: usize, out_capacity: usize) -> Self {
        // SAFETY: allocation only, and the C side returns null rather than
        // letting `std::bad_alloc` unwind into this frame.
        Ring(unsafe { sys::sdrx_drm_ring_new(in_capacity, out_capacity) })
    }

    /// Queue interleaved I/Q. Returns how many samples had to be dropped to
    /// make room, which is non-zero only when the decoder is not keeping up.
    pub fn push(&self, interleaved: &[i16]) -> usize {
        // SAFETY: the pointer is valid for this Ring's lifetime and the slice
        // is read for `len` elements.
        unsafe { sys::sdrx_drm_ring_push(self.0, interleaved.as_ptr(), interleaved.len()) }
    }

    /// Take decoded audio, up to `out.len()` samples. Returns how many.
    pub fn pop(&self, out: &mut [i16]) -> usize {
        // SAFETY: as above, writing at most `len` elements.
        unsafe { sys::sdrx_drm_ring_pop(self.0, out.as_mut_ptr(), out.len()) }
    }

    /// Samples of decoded audio waiting.
    pub fn audio_available(&self) -> usize {
        // SAFETY: as above.
        unsafe { sys::sdrx_drm_ring_out_available(self.0) }
    }

    /// Release a decoder blocked waiting for input, so its thread can be joined.
    pub fn stop(&self) {
        // SAFETY: as above.
        unsafe { sys::sdrx_drm_ring_stop(self.0) }
    }
}

impl Drop for Ring {
    fn drop(&mut self) {
        // SAFETY: called once, and every Decoder using this Ring has already
        // been dropped — the worker joins its thread before releasing the Arc.
        unsafe { sys::sdrx_drm_ring_free(self.0) }
    }
}

/// One Dream receiver.
///
/// Deliberately neither [`Send`] nor [`Sync`]: Dream's receiver is not
/// internally synchronised, and the sound shims it allocates find their queues
/// through a thread-local set on the thread that built it.
pub struct Decoder {
    handle: *mut sys::sdrx_drm,
}

impl Decoder {
    /// Build a receiver reading from `ring`. Must be called on the thread that
    /// will drive it.
    pub fn new(ring: &Ring, iq_input: bool, flip_spectrum: bool) -> Result<Self, DrmError> {
        let cfg = sys::sdrx_drm_config {
            sig_sample_rate: SIGNAL_RATE as i32,
            aud_sample_rate: AUDIO_RATE as i32,
            iq_input: i32::from(iq_input),
            flip_spectrum: i32::from(flip_spectrum),
        };
        // SAFETY: `ring` outlives the decoder (the worker holds the Arc), and
        // the config is read-only for the duration of the call.
        let handle = unsafe { sys::sdrx_drm_new(ring.0, &cfg) };
        if handle.is_null() {
            return Err(DrmError::OpenFailed);
        }
        Ok(Decoder { handle })
    }

    /// One pass of the receive chain. Blocks until a block of input is queued,
    /// or until the ring is stopped. `false` means the chain threw and this
    /// decoder is finished — [`last_error`] then says what threw.
    pub fn process(&mut self) -> bool {
        // SAFETY: same-thread use of a handle this type owns.
        unsafe { sys::sdrx_drm_process(self.handle) == 0 }
    }

    /// Re-acquire from scratch, after a retune or a bandwidth change.
    pub fn restart(&mut self) {
        // SAFETY: as above.
        unsafe { sys::sdrx_drm_restart(self.handle) }
    }

    /// Choose which service of the multiplex to decode.
    pub fn select_service(&mut self, service: u8) {
        // SAFETY: as above; the C side range-checks the index.
        unsafe { sys::sdrx_drm_select_service(self.handle, i32::from(service)) }
    }

    /// The equalised symbols of one logical channel, at most
    /// [`CONSTELLATION_POINTS`] of them.
    pub fn constellation(&mut self, channel: DrmChannel) -> Option<DrmConstellation> {
        let mut points = vec![0.0f32; CONSTELLATION_POINTS * 2];
        let mut qam: i32 = 0;
        // SAFETY: same-thread use of a handle this type owns; the C side writes
        // at most `CONSTELLATION_POINTS` pairs into a buffer of that size.
        let n = unsafe {
            sys::sdrx_drm_constellation(
                self.handle,
                channel.as_raw(),
                points.as_mut_ptr(),
                CONSTELLATION_POINTS as i32,
                &mut qam,
            )
        };
        if n <= 0 {
            return None;
        }
        points.truncate(n as usize * 2);
        Some(DrmConstellation { channel, qam: qam.clamp(0, 255) as u8, points })
    }

    pub fn status(&mut self) -> DrmStatus {
        let mut raw = unsafe { std::mem::zeroed::<sys::sdrx_drm_status>() };
        // SAFETY: as above, writing one fully-owned struct.
        unsafe { sys::sdrx_drm_get_status(self.handle, &mut raw) };
        convert_status(&raw)
    }
}

impl Drop for Decoder {
    fn drop(&mut self) {
        // SAFETY: called once, on the thread that built the decoder.
        unsafe { sys::sdrx_drm_free(self.handle) }
    }
}

/// Why the last call into the decoder failed, on this thread.
///
/// The C shim converts every exception into a failure return, because Dream's
/// over-the-air parsers size buffers from lengths the broadcast supplies and an
/// unwind across the `extern "C"` boundary would take the whole radio down
/// rather than the decode. Empty when nothing has failed.
///
/// Must be called on the thread whose call failed.
pub fn last_error() -> String {
    // SAFETY: a pointer to a thread-local string that outlives this call and is
    // only invalidated by the next call into the shim on this thread.
    let s = unsafe { CStr::from_ptr(sys::sdrx_drm_last_error()) };
    s.to_string_lossy().into_owned()
}

/// Version string of the linked AAC decoder, for the log.
pub fn codec_version() -> String {
    // SAFETY: the C side returns a pointer to a static string that outlives
    // this call.
    let s = unsafe { CStr::from_ptr(sys::sdrx_drm_codec_version()) };
    s.to_string_lossy().into_owned()
}

/// A NUL-terminated C array of `i8`/`u8` to a Rust `String`, stopping at the
/// first NUL. Dream fills these from the broadcast, so they are not trusted to
/// be valid UTF-8.
fn c_string(buf: &[::std::os::raw::c_char]) -> String {
    let bytes: Vec<u8> = buf.iter().take_while(|&&c| c != 0).map(|&c| c as u8).collect();
    String::from_utf8_lossy(&bytes).into_owned()
}

fn convert_status(raw: &sys::sdrx_drm_status) -> DrmStatus {
    let service = DrmService {
        label: c_string(&raw.label),
        text: c_string(&raw.text_message),
        country: c_string(&raw.country_code),
        language: c_string(&raw.language_code),
        service_id: raw.service_id as u32,
        bitrate_kbps: raw.bitrate_kbps as f32,
        codec: (raw.has_signal != 0).then(|| DrmCodec::from_raw(raw.audio_codec)),
        codec_supported: raw.audio_codec_supported != 0,
        stereo: raw.is_stereo != 0,
    };
    // A multiplex with no clock signals all-zero rather than omitting the field.
    let time = (raw.year != 0 || raw.month != 0 || raw.day != 0).then_some(DrmTime {
        year: raw.year as u16,
        month: raw.month as u8,
        day: raw.day as u8,
        hour: raw.utc_hour as u8,
        minute: raw.utc_minute as u8,
    });
    DrmStatus {
        io: DrmSync::from_raw(raw.io_status),
        time_sync: DrmSync::from_raw(raw.time_sync_status),
        frame_sync: DrmSync::from_raw(raw.frame_sync_status),
        fac: DrmSync::from_raw(raw.fac_status),
        sdc: DrmSync::from_raw(raw.sdc_status),
        audio: DrmSync::from_raw(raw.audio_status),
        locked: raw.has_signal != 0,
        snr_db: raw.snr_db as f32,
        if_level_db: raw.if_level_db as f32,
        wmer_db: raw.wmer_db as f32,
        mer_db: raw.mer_db as f32,
        dc_offset_hz: raw.dc_frequency_hz as f32,
        sample_offset_hz: raw.sample_offset_hz as f32,
        // The C side reports "not estimated" as a negative spread.
        doppler_hz: (raw.doppler_hz >= 0.0).then_some(raw.doppler_hz as f32),
        delay_ms: raw.delay_ms as f32,
        robustness: DrmRobustness::from_raw(raw.robustness_mode),
        bandwidth_khz: spectrum_occupancy_khz(raw.spectrum_occupancy),
        interleaver_long: raw.interleaver_long != 0,
        protection_a: raw.prot_level_a.clamp(0, 255) as u8,
        protection_b: raw.prot_level_b.clamp(0, 255) as u8,
        audio_services: raw.num_audio_services.clamp(0, 255) as u8,
        data_services: raw.num_data_services.clamp(0, 255) as u8,
        current_service: raw.cur_service.clamp(0, 255) as u8,
        service,
        time,
        // Filled in by the worker only when somebody is looking at one; the
        // status conversion has no way to know that.
        constellation: None,
    }
    // The C side also fills sdc_scheme, msc_scheme, audio_mode,
    // audio_sample_rate and audio_sample_rate_out, none of which have anywhere
    // to go yet. They are there if a use turns up.
}
