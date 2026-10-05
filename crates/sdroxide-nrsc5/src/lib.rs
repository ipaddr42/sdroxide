//! HD Radio (NRSC-5) decoding, through the `libnrsc5` installed on the machine.
//!
//! The crate is the decoder for the HD Radio band: it takes raw I/Q samples
//! (via `HdReceiver::pipe_*`, at nrsc5's native sample rate — see the
//! `NRSC5_SAMPLE_RATE_*` definitions upstream) and turns them into decoded
//! audio and the SIS/ID3 metadata broadcasters send.
//!
//! nrsc5 is not built in: `src/ffi.rs` finds its shared library at run time,
//! and [`unavailable_reason`] is the sentence to show where there is none. That
//! is also what keeps its HDC audio codec — a patched faad2 the library carries
//! inside itself — out of sdroxide, and out of the way of the stock faad2 the
//! DRM receiver links (issue #488).
//!
//! [`HdReceiver`] is a thin binding on purpose. Opened on a pipe, the C library
//! runs no thread of its own: each `pipe_*` call does the decoding and fires
//! the callbacks before it returns, and they reach this side as `Event`s
//! through a plain channel. `HdReceiver` is therefore `Send` (move it to the
//! thread that will feed it) but not `Sync`: the pipe functions and the close
//! must not run concurrently. [`HdDemod`] is the receive-chain demodulator
//! built on it, and keeps that work off the chain's thread.
//!
//! Audio reaches the caller as `Event::Audio` in signed 16-bit interleaved
//! stereo PCM at 44.1 kHz, exactly as nrsc5 emits it. Metadata arrives as the
//! station / SIS
//! events; the finer data services (LOT files, HERE images, ID3 tags, the SIG
//! table) are decoded inside the library but not yet surfaced here.

#![deny(missing_docs)]

pub mod demod;
mod ffi;
mod worker;
pub use demod::HdDemod;
pub use ffi::{LIB_ENV, unavailable_reason};

use std::cell::RefCell;
use std::ffi::{c_char, c_float, c_int, c_uint, c_void};
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::mpsc::{self, Receiver};

use ffi::{Api, NrsCtx};

/// The analogue source the decoder expects to see.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Frequency-modulated hybrid HD Radio, native pipes at 744,187.5 S/s.
    Fm = 0,
    /// Amplitude-modulated HBO (HD on AM), native pipes at 46,511.71875 S/s.
    Am = 1,
}

impl Mode {
    /// The rate the decoder's input pipe runs at, in samples per second — the
    /// rate the channel is resampled to before it is fed in.
    pub fn native_rate_hz(self) -> f64 {
        match self {
            Mode::Fm => 744_187.5,
            Mode::Am => 46_511.71875,
        }
    }

    /// The narrowest channel this mode can be fed.
    ///
    /// The FM hybrid's digital sidebands reach 198.4 kHz either side of the
    /// carrier, so a channel under twice that cannot hold them. The AM-band
    /// variant's sidebands reach about 15 kHz, so it wants a far narrower
    /// stream — and would be drowned in medium-wave noise by the FM window
    /// (issue #489).
    pub fn min_channel_rate_hz(self) -> f64 {
        match self {
            Mode::Fm => 400_000.0,
            Mode::Am => 30_000.0,
        }
    }
}

/// A decoded event from the HD Radio receive path.
#[derive(Debug, Clone, PartialEq)]
pub enum Event {
    /// Lock has been acquired; `freq_offset` is the residual offset in Hz and
    /// `psmi` the Primary Service Mode Indicator.
    Sync {
        /// Residual carrier frequency offset, in Hz.
        freq_offset: f32,
        /// Primary Service Mode Indicator (1, 2, 3, 5, 6 or 11 in FM).
        psmi: i32,
    },
    /// The previously acquired lock was lost.
    LostSync,
    /// Modulation error ratio of the lower and upper sidebands, in dB.
    Mer {
        /// Lower sideband, in dB.
        lower: f32,
        /// Upper sideband, in dB.
        upper: f32,
    },
    /// Channel bit-error ratio.
    Ber {
        /// Channel bit-error ratio, 0.0 to 1.0.
        cber: f32,
    },
    /// Decoded PCM audio: signed 16-bit interleaved stereo, 44.1 kHz.
    Audio {
        /// The program the audio belongs to.
        program: u8,
        /// Signed 16-bit PCM, left and right interleaved, 44,100 frames per
        /// second.
        data: Vec<i16>,
        /// The frame is the silence the decoder fills in for a packet that was
        /// missing or failed to decode, not sound from the station.
        ///
        /// Read off the samples: every nrsc5 since 3.0 fills such a slot from
        /// one zeroed buffer, but only master says so, in a `flags` member that
        /// an older library leaves uninitialised. A station airing digital
        /// silence therefore reads as unavailable too, which is the one case
        /// the two ways of telling disagree on, and a harmless one.
        unavailable: bool,
    },
    /// A packet of coded HDC audio arrived, before decoding.
    ///
    /// Only its arrival is reported. A library built without its audio decoder
    /// (`USE_FAAD2=OFF`) sends these and never an [`Event::Audio`], not even
    /// filled-in silence, and that is how one is told apart.
    HdcPacket {
        /// The program the packet belongs to.
        program: u8,
    },
    /// An audio service is available on the wave (from SIS descriptors).
    AudioService {
        /// Program number, 0 to 7.
        program: u8,
        /// `NRSC5_ACCESS_PUBLIC` (0) or `NRSC5_ACCESS_RESTRICTED` (1).
        access: u8,
        /// Audio codec mode, per SY_IDD_1017s Table 5-2.
        codec_mode: u8,
    },
    /// The broadcaster's station name, e.g. "Q107".
    StationName(String),
    /// The station slogan, e.g. "You're Listening to Q".
    StationSlogan(String),
    /// A short text the broadcaster is currently airing.
    StationMessage(String),
}

/// Errors opening or driving an `HdReceiver`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NrsError {
    /// There is no `libnrsc5` to open — see [`unavailable_reason`].
    Unavailable,
    /// The library rejected the open or the mode switch.
    Open,
    /// A pipe call rejected its arguments (length not matching the format).
    Pipe,
}

/// A handle to an HD Radio receive session.
///
/// Opened on a pipe, the library starts no thread: every `pipe_*` call does the
/// decoding itself and fires the audio and metadata callbacks before it
/// returns. They are marshalled into a channel and drained with `poll` or
/// `drain`, which never block — there is no one else to send. Dropping the
/// receiver detaches the callback before the library is closed, so no callback
/// can outlive it.
pub struct HdReceiver {
    api: &'static Api,
    st: *mut NrsCtx,
    rx: Receiver<Event>,
    sink: Box<CbSink>,
    /// The `cs16` copy of a `cf32` block, for a library without
    /// `nrsc5_pipe_samples_cf32`.
    cs16: RefCell<Vec<i16>>,
}

// The C handle is only touched from the (single) calling thread after the
// worker has been joined, and the channel end is `Send`.
unsafe impl Send for HdReceiver {}

impl HdReceiver {
    /// Opens an HD Radio session in the given mode and starts its worker.
    pub fn open(mode: Mode) -> Result<Self, NrsError> {
        let api = ffi::api().map_err(|_| NrsError::Unavailable)?;
        let mut st: *mut NrsCtx = std::ptr::null_mut();
        if unsafe { (api.open_pipe)(&mut st) } != 0 || st.is_null() {
            return Err(NrsError::Open);
        }
        if unsafe { (api.set_mode)(st, mode as c_int) } != 0 {
            unsafe { (api.close)(st) };
            return Err(NrsError::Open);
        }
        let (tx, rx) = mpsc::channel();
        let sink = Box::new(CbSink { tx, audio_program: AtomicI32::new(-1) });
        let opaque = &*sink as *const CbSink as *mut c_void;
        unsafe { (api.set_callback)(st, Some(trampoline), opaque) };
        unsafe { (api.start)(st) };
        Ok(HdReceiver { api, st, rx, sink, cs16: RefCell::new(Vec::new()) })
    }

    /// Pipes raw 8-bit unsigned I/Q samples (2 bytes per complex sample).
    ///
    /// `samples.len()` is in bytes; a trailing odd pair is buffered by the
    /// library across calls, so the total count matters more than the chunk
    /// boundaries. Feed at `NRSC5_SAMPLE_RATE_CU8` (1,488,375 S/s).
    pub fn pipe_cu8(&self, samples: &[u8]) -> Result<(), NrsError> {
        let len = c_uint::try_from(samples.len()).map_err(|_| NrsError::Pipe)?;
        if unsafe { (self.api.pipe_cu8)(self.st, samples.as_ptr(), len) } != 0 {
            return Err(NrsError::Pipe);
        }
        Ok(())
    }

    /// Pipes signed 16-bit I/Q, interleaved: two values per complex sample.
    ///
    /// The length nrsc5 takes is a count of `int16_t` values — the slice's own
    /// length — and a trailing odd value is buffered by the library across
    /// calls. Handing it twice that, as a byte count, had the library read as
    /// far again past the end of the slice. Feed at the native FM (744,187.5 S/s)
    /// or AM (46,511.71875 S/s) rate, like [`Self::pipe_cf32`].
    ///
    /// The scale is the library's, and it moved: up to 3.2.0 the values go in
    /// as they are, where full scale off an 8-bit dongle is ±8192; later ones
    /// read ±32768 as full scale.
    pub fn pipe_cs16(&self, samples: &[i16]) -> Result<(), NrsError> {
        let len = c_uint::try_from(samples.len()).map_err(|_| NrsError::Pipe)?;
        if unsafe { (self.api.pipe_cs16)(self.st, samples.as_ptr(), len) } != 0 {
            return Err(NrsError::Pipe);
        }
        Ok(())
    }

    /// Pipes single-precision complex I/Q samples (2 floats per sample), where
    /// ±1.0 is the full scale of an 8-bit dongle.
    ///
    /// `samples.len()` is in floats and must be even. Feed at the native FM
    /// (744,187.5 S/s) or AM (46,511.71875 S/s) rate.
    ///
    /// A library without `nrsc5_pipe_samples_cf32` — every release up to 3.2.0
    /// — gets the block as `cs16` instead, scaled to the ±8192 its own 8-bit
    /// input path produces, so the two arrive at the same level.
    pub fn pipe_cf32(&self, samples: &[c_float]) -> Result<(), NrsError> {
        let len = c_uint::try_from(samples.len()).map_err(|_| NrsError::Pipe)?;
        if let Some(pipe_cf32) = self.api.pipe_cf32 {
            if unsafe { pipe_cf32(self.st, samples.as_ptr(), len) } != 0 {
                return Err(NrsError::Pipe);
            }
            return Ok(());
        }
        let mut cs16 = self.cs16.borrow_mut();
        cs16.clear();
        cs16.extend(samples.iter().map(|&x| cf32_to_legacy_cs16(x)));
        self.pipe_cs16(&cs16)
    }

    /// Copy out the audio of one programme only, or of every programme with
    /// `None` (the default).
    ///
    /// Each audio frame is copied as it is reported, and a multiplex decodes
    /// every programme it carries whichever one is being listened to. On a
    /// station with three, two of every three copies were thrown away unread.
    pub fn set_audio_program(&self, program: Option<u8>) {
        self.sink.audio_program.store(program.map_or(-1, i32::from), Ordering::Relaxed);
    }

    /// Returns the next queued event without waiting.
    pub fn poll(&self) -> Option<Event> {
        self.rx.try_recv().ok()
    }

    /// A non-blocking iterator over whatever is queued right now.
    pub fn drain(&self) -> Drain<'_> {
        Drain { rx: &self.rx }
    }
}

impl Drop for HdReceiver {
    fn drop(&mut self) {
        // Detach the callback first: the sink boxed on the receive side must
        // not be reachable once the worker is joined below.
        unsafe { (self.api.set_callback)(self.st, None, std::ptr::null_mut()) };
        unsafe { (self.api.stop)(self.st) };
        unsafe { (self.api.close)(self.st) };
    }
}

/// What full scale off an 8-bit dongle is in the `cs16` a library up to 3.2.0
/// reads: its own `U8_Q15` turns `0..=255` into `(x - 127) * 64`.
const LEGACY_CS16_FULL_SCALE: f32 = 8192.0;

/// One `cf32` value as that `cs16`, saturating rather than wrapping.
fn cf32_to_legacy_cs16(x: f32) -> i16 {
    (x * LEGACY_CS16_FULL_SCALE).round().clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

/// Iterator over the events queued at the moment `drain` was called.
pub struct Drain<'a> {
    rx: &'a Receiver<Event>,
}

impl Iterator for Drain<'_> {
    type Item = Event;
    fn next(&mut self) -> Option<Event> {
        self.rx.try_recv().ok()
    }
}

// Safety contract: `opaque` is the `Box<CbSink>` registered on `open`, and it
// outlives the worker because `drop` detaches the callback before joining.
struct CbSink {
    tx: mpsc::Sender<Event>,
    /// The programme whose audio is copied out, or -1 for all of them — see
    /// [`HdReceiver::set_audio_program`].
    audio_program: AtomicI32,
}

unsafe extern "C" fn trampoline(evt: *const NrsEvent, opaque: *mut c_void) {
    let sink = unsafe { &*(opaque as *const CbSink) };
    if !evt.is_null() && unsafe { (*evt).event } == NRS_EVENT_AUDIO {
        let wanted = sink.audio_program.load(Ordering::Relaxed);
        if wanted >= 0 && unsafe { (*evt).u.audio.program } != wanted as c_uint {
            return;
        }
    }
    if let Some(ev) = unsafe { translate(evt) } {
        let _ = sink.tx.send(ev);
    }
}

// NRSC5_EVENT_* values, in the order nrsc5.h declares them: LOST_DEVICE,
// IQ, SYNC, LOST_SYNC, MER, BER, HDC, AUDIO, ID3, SIG, LOT, SIS, STREAM,
// PACKET, AUDIO_SERVICE, STATION_ID, STATION_NAME, STATION_SLOGAN,
// STATION_MESSAGE, STATION_LOCATION, ...
//
// These numbers and the `#[repr(C)]` structs below are copied from the header
// by hand, and the library they are read against is whichever one the machine
// has. They were measured against the headers of v3.1.0, v3.2.0 and master
// (0225922): every number and every member read here is the same in all three,
// and in v3.0 bar the sync payload. The `layout` test pins the offsets that
// measurement found.
const NRS_EVENT_SYNC: c_uint = 2;
const NRS_EVENT_LOST_SYNC: c_uint = 3;
const NRS_EVENT_MER: c_uint = 4;
const NRS_EVENT_BER: c_uint = 5;
const NRS_EVENT_HDC: c_uint = 6;
const NRS_EVENT_AUDIO: c_uint = 7;
const NRS_EVENT_AUDIO_SERVICE: c_uint = 14;
const NRS_EVENT_STATION_NAME: c_uint = 16;
const NRS_EVENT_STATION_SLOGAN: c_uint = 17;
const NRS_EVENT_STATION_MESSAGE: c_uint = 18;

/// Translates the C event into a Rust one, copying every borrow out.
unsafe fn translate(evt: *const NrsEvent) -> Option<Event> {
    if evt.is_null() {
        return None;
    }
    let e = unsafe { &*evt };
    match e.event {
        NRS_EVENT_SYNC => {
            let (freq_offset, psmi) = unsafe { (e.u.sync.freq_offset, e.u.sync.psmi) };
            Some(Event::Sync { freq_offset: sane_offset(freq_offset), psmi: sane_psmi(psmi) })
        }
        NRS_EVENT_LOST_SYNC => Some(Event::LostSync),
        NRS_EVENT_MER => {
            Some(Event::Mer { lower: unsafe { e.u.mer.lower }, upper: unsafe { e.u.mer.upper } })
        }
        NRS_EVENT_BER => Some(Event::Ber { cber: unsafe { e.u.ber.cber } }),
        NRS_EVENT_HDC => Some(Event::HdcPacket { program: unsafe { e.u.hdc.program } as u8 }),
        NRS_EVENT_AUDIO => {
            let a = unsafe { e.u.audio };
            if a.data.is_null() || a.count == 0 {
                return None;
            }
            let data = unsafe { std::slice::from_raw_parts(a.data, a.count) }.to_vec();
            let unavailable = data.iter().all(|&s| s == 0);
            Some(Event::Audio { program: a.program as u8, data, unavailable })
        }
        NRS_EVENT_AUDIO_SERVICE => {
            let a = unsafe { e.u.audio_service };
            Some(Event::AudioService {
                program: a.program as u8,
                access: a.access as u8,
                codec_mode: a.codec_mode as u8,
            })
        }
        NRS_EVENT_STATION_NAME => unsafe { cstr(e.u.station_name.name) }.map(Event::StationName),
        NRS_EVENT_STATION_SLOGAN => {
            unsafe { cstr(e.u.station_slogan.name) }.map(Event::StationSlogan)
        }
        NRS_EVENT_STATION_MESSAGE => {
            unsafe { cstr(e.u.station_message.name) }.map(Event::StationMessage)
        }
        _ => None,
    }
}

/// The sync's carrier offset, or zero where it cannot be one.
///
/// nrsc5 3.0 sent the sync event with no payload, so on that library these
/// bytes are whatever the stack held. The search nrsc5 makes is a few
/// kilohertz either side; anything past that, or not a number at all, is not
/// an offset.
fn sane_offset(hz: f32) -> f32 {
    if hz.is_finite() && hz.abs() < 100_000.0 { hz } else { 0.0 }
}

/// The Primary Service Mode Indicator, or zero where it cannot be one: the
/// field is six bits on the air, and on nrsc5 3.0 it is not there at all.
fn sane_psmi(psmi: c_int) -> i32 {
    if (0..64).contains(&psmi) { psmi } else { 0 }
}

/// Copies a NUL-terminated C string.
unsafe fn cstr(p: *const c_char) -> Option<String> {
    if p.is_null() {
        return None;
    }
    let c = unsafe { std::ffi::CStr::from_ptr(p) };
    Some(c.to_string_lossy().into_owned())
}

/// `nrsc5_event_t`, as far as this crate reads it.
#[repr(C)]
pub(crate) struct NrsEvent {
    event: c_uint,
    u: NrsUnion,
}

// A union of the members this crate reads. Union members not declared here
// (id3, sig, lot, ...) are simply never read; all union arms share the offset
// of the enclosing struct, so the missing ones cost nothing.
#[repr(C)]
union NrsUnion {
    sync: NrsSync,
    ber: NrsBer,
    mer: NrsMer,
    hdc: NrsHdc,
    audio: NrsAudio,
    audio_service: NrsAudioService,
    station_name: NrsName,
    station_slogan: NrsName,
    station_message: NrsName,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct NrsSync {
    freq_offset: c_float,
    psmi: c_int,
    pli: c_int,
    hppi: c_int,
    aabi: c_int,
    rdbi: c_int,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct NrsBer {
    cber: c_float,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct NrsMer {
    lower: c_float,
    upper: c_float,
}

/// Only the programme is read: nothing here decodes the packet itself.
#[repr(C)]
#[derive(Clone, Copy)]
struct NrsHdc {
    program: c_uint,
}

/// Ends at `count`: master's `flags` after it is left uninitialised by any
/// older library — see [`Event::Audio`].
#[repr(C)]
#[derive(Clone, Copy)]
struct NrsAudio {
    program: c_uint,
    data: *const i16,
    count: usize,
}

#[repr(C)]
#[derive(Clone, Copy)]
#[allow(dead_code)]
struct NrsAudioService {
    program: c_uint,
    access: c_uint,
    type_: c_uint,
    codec_mode: c_uint,
    blend_control: c_uint,
    digital_audio_gain: c_int,
    common_delay: c_uint,
    latency: c_uint,
}

/// Mirrors `struct { const char *name; }`, `{ ... *slogan; }` and
/// `{ ... *message; }` — one pointer at offset 0 in each case.
#[repr(C)]
#[derive(Clone, Copy)]
struct NrsName {
    name: *const c_char,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The members read out of `nrsc5_event_t`, where the headers of v3.1.0,
    /// v3.2.0 and master (0225922) put them on a 64-bit target: offsets into
    /// the event, measured with `offsetof` against each header. A library
    /// loaded at run time is checked by nothing else, so a change to the
    /// structs above that moves one of these is caught here instead of as
    /// audio read from the middle of a pointer.
    #[cfg(target_pointer_width = "64")]
    #[test]
    fn layout() {
        use std::mem::offset_of;
        let u = offset_of!(NrsEvent, u);
        assert_eq!(u, 8, "the union follows the event number");
        assert_eq!(u + offset_of!(NrsSync, freq_offset), 8);
        assert_eq!(u + offset_of!(NrsSync, psmi), 12);
        assert_eq!(u + offset_of!(NrsMer, lower), 8);
        assert_eq!(u + offset_of!(NrsMer, upper), 12);
        assert_eq!(u + offset_of!(NrsBer, cber), 8);
        assert_eq!(u + offset_of!(NrsHdc, program), 8);
        assert_eq!(u + offset_of!(NrsAudio, program), 8);
        assert_eq!(u + offset_of!(NrsAudio, data), 16);
        assert_eq!(u + offset_of!(NrsAudio, count), 24);
        assert_eq!(u + offset_of!(NrsAudioService, program), 8);
        assert_eq!(u + offset_of!(NrsAudioService, access), 12);
        assert_eq!(u + offset_of!(NrsAudioService, codec_mode), 20);
        assert_eq!(u + offset_of!(NrsName, name), 8);
    }

    /// A `cf32` block handed to a library without `cf32` arrives at the level
    /// that library's own 8-bit path produces, and a peak past full scale
    /// saturates instead of wrapping to the opposite sign.
    #[test]
    fn cf32_reaches_an_old_library_at_its_own_scale() {
        assert_eq!(cf32_to_legacy_cs16(0.0), 0);
        // An 8-bit dongle's full swing, as `U8_F` and `U8_Q15` each see it.
        assert_eq!(cf32_to_legacy_cs16((255.0 - 127.0) / 128.0), (255 - 127) * 64);
        assert_eq!(cf32_to_legacy_cs16((0.0 - 127.0) / 128.0), (0 - 127) * 64);
        assert_eq!(cf32_to_legacy_cs16(10.0), i16::MAX);
        assert_eq!(cf32_to_legacy_cs16(-10.0), i16::MIN);
    }

    /// nrsc5 3.0 sends the sync with no payload, so what is read there is
    /// whatever was on its stack: nothing that cannot be an offset or a service
    /// mode is passed on as one.
    #[test]
    fn a_sync_without_a_payload_reads_as_nothing() {
        assert_eq!(sane_offset(-1234.5), -1234.5);
        assert_eq!(sane_offset(f32::NAN), 0.0);
        assert_eq!(sane_offset(3.0e30), 0.0);
        assert_eq!(sane_psmi(2), 2);
        assert_eq!(sane_psmi(-7), 0);
        assert_eq!(sane_psmi(0x7fff_0000), 0);
    }
}
