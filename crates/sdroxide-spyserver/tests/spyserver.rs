//! Integration tests against a fake SpyServer.
//!
//! The opcodes and setting numbers are spelled out here as literals rather
//! than imported from the crate. That is deliberate, and it is the same rule
//! the `rtl_tcp` tests keep: a test that shares its constants with the code
//! under test cannot catch a renumbering, which is exactly the mistake that
//! would make this client talk confidently to a server about the wrong thing.
//!
//! The fake writes in awkward chunk lengths on purpose. A TCP segment ends
//! wherever it ends, so a header split down the middle and a body arriving in
//! three pieces are the normal case rather than the rare one — and this
//! protocol has no marker to resynchronise on if that goes wrong.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_spyserver::SpyServerHandle;
use sdroxide_types::{SpyServerConfig, SpyServerFormat};

// --- the wire, restated ------------------------------------------------------

const CMD_HELLO: u32 = 0;
const CMD_SET_SETTING: u32 = 2;
const CMD_PING: u32 = 3;

const SETTING_STREAMING_MODE: u32 = 0;
const SETTING_STREAMING_ENABLED: u32 = 1;
const SETTING_GAIN: u32 = 2;
const SETTING_IQ_FORMAT: u32 = 100;
const SETTING_IQ_FREQUENCY: u32 = 101;
const SETTING_IQ_DECIMATION: u32 = 102;
const SETTING_IQ_DIGITAL_GAIN: u32 = 103;
const SETTING_FFT_FORMAT: u32 = 200;
const SETTING_FFT_FREQUENCY: u32 = 201;
const SETTING_FFT_DECIMATION: u32 = 202;
const SETTING_FFT_DB_OFFSET: u32 = 203;
const SETTING_FFT_DB_RANGE: u32 = 204;
const SETTING_FFT_DISPLAY_PIXELS: u32 = 205;

const STREAM_MODE_IQ_ONLY: u32 = 1;
const STREAM_MODE_FFT_IQ: u32 = 5;

const MSG_DEVICE_INFO: u16 = 0;
const MSG_CLIENT_SYNC: u16 = 1;
const MSG_PONG: u16 = 2;
const MSG_UINT8_IQ: u16 = 100;
const MSG_INT24_IQ: u16 = 102;
const MSG_DINT4_FFT: u16 = 300;
const MSG_UINT8_FFT: u16 = 301;

const PROTOCOL_VERSION: u32 = (2 << 24) | 1700;
const MSG_HEADER_LEN: usize = 20;
const MAX_MESSAGE_BODY_SIZE: u32 = 1 << 20;

/// One SET_SETTING the fake received.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Rec {
    setting: u32,
    value: u32,
}

/// What the fake claims to be.
#[derive(Debug, Clone, Copy)]
struct Spec {
    device_type: u32,
    max_rate: u32,
    max_bw: u32,
    stages: u32,
    min_decim: u32,
    max_gain: u32,
    min_freq: u32,
    max_freq: u32,
    resolution: u32,
    forced_format: u32,
    can_control: u32,
    device_center: u32,
    /// Send an 8-bit I/Q burst once the stream is enabled.
    stream_iq: bool,
    /// The byte both components of every 8-bit I/Q sample carry. `None` keeps
    /// the burst the framing tests want: mid-scale and both extremes. Either
    /// way the header states the digital gain the server was last set to, as
    /// a real one does.
    iq_fill: Option<u8>,
    /// Keep stating the first digital gain ever set, however many arrive
    /// after it: a link on which every sample the client judges was sent
    /// under a figure it has already moved on from.
    stale_iq_gain: bool,
    /// Send an 8-bit FFT frame once the stream is enabled.
    stream_fft: bool,
}

impl Default for Spec {
    fn default() -> Self {
        // An RTL-SDR behind a server, which is the commonest thing there is.
        Spec {
            device_type: 3,
            max_rate: 2_400_000,
            max_bw: 2_400_000,
            stages: 6,
            min_decim: 0,
            max_gain: 28,
            min_freq: 24_000_000,
            max_freq: 1_766_000_000,
            resolution: 8,
            forced_format: 0,
            can_control: 1,
            device_center: 100_000_000,
            stream_iq: false,
            iq_fill: None,
            stale_iq_gain: false,
            stream_fft: false,
        }
    }
}

fn msg(kind: u16, flags: u16, stream_type: u32, seq: u32, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(MSG_HEADER_LEN + body.len());
    out.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    out.extend_from_slice(&(u32::from(kind) | (u32::from(flags) << 16)).to_le_bytes());
    out.extend_from_slice(&stream_type.to_le_bytes());
    out.extend_from_slice(&seq.to_le_bytes());
    out.extend_from_slice(&(body.len() as u32).to_le_bytes());
    out.extend_from_slice(body);
    out
}

fn words(ws: &[u32]) -> Vec<u8> {
    ws.iter().flat_map(|w| w.to_le_bytes()).collect()
}

impl Spec {
    fn device_info(&self) -> Vec<u8> {
        words(&[
            self.device_type,
            0xDEAD_BEEF,
            self.max_rate,
            self.max_bw,
            self.stages,
            1,
            self.max_gain,
            self.min_freq,
            self.max_freq,
            self.resolution,
            self.min_decim,
            self.forced_format,
        ])
    }

    fn client_sync(&self, gain: u32) -> Vec<u8> {
        let c = self.device_center;
        words(&[self.can_control, gain, c, c, c, 0, 0, 0, 0])
    }
}

/// Write `bytes` in deliberately awkward pieces, so headers and bodies both
/// end up split across "segments".
fn write_chunked(sock: &mut TcpStream, bytes: &[u8]) -> std::io::Result<()> {
    let mut i = 0;
    for &n in [7usize, 13, 3, 4096].iter().cycle() {
        if i >= bytes.len() {
            break;
        }
        let end = (i + n).min(bytes.len());
        sock.write_all(&bytes[i..end])?;
        i = end;
    }
    Ok(())
}

struct Fake {
    addr: String,
    log: Arc<Mutex<Vec<Rec>>>,
    /// Raw bytes for the server thread to send, for the cases a well-behaved
    /// server would never produce.
    inject: crossbeam_channel::Sender<Vec<u8>>,
    stopped: Arc<AtomicBool>,
    _join: Option<std::thread::JoinHandle<()>>,
}

impl Fake {
    fn start(spec: Spec) -> Fake {
        let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("addr").to_string();
        let log = Arc::new(Mutex::new(Vec::new()));
        let stopped = Arc::new(AtomicBool::new(false));
        let (inject, injected) = crossbeam_channel::unbounded::<Vec<u8>>();

        let thread_log = Arc::clone(&log);
        let thread_stopped = Arc::clone(&stopped);
        let join = std::thread::spawn(move || {
            let Ok((mut sock, _)) = listener.accept() else { return };
            sock.set_read_timeout(Some(Duration::from_millis(10))).ok();
            let mut carry: Vec<u8> = Vec::new();
            let mut buf = [0u8; 4096];
            let mut greeted = false;
            let mut streaming = false;
            let mut seq = 0u32;
            let mut gain = 0u32;
            let mut digital_gain: Option<u16> = None;

            loop {
                if thread_stopped.load(Ordering::Relaxed) {
                    return;
                }
                while let Ok(bytes) = injected.try_recv() {
                    if write_chunked(&mut sock, &bytes).is_err() {
                        return;
                    }
                }
                match sock.read(&mut buf) {
                    Ok(0) => return,
                    Ok(n) => carry.extend_from_slice(&buf[..n]),
                    Err(e)
                        if matches!(
                            e.kind(),
                            std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                        ) => {}
                    Err(_) => return,
                }

                // Client commands: {u32 type, u32 body_size} then the body.
                while carry.len() >= 8 {
                    let cmd = u32::from_le_bytes(carry[0..4].try_into().unwrap());
                    let body = u32::from_le_bytes(carry[4..8].try_into().unwrap()) as usize;
                    if carry.len() < 8 + body {
                        break;
                    }
                    let payload: Vec<u8> = carry[8..8 + body].to_vec();
                    carry.drain(..8 + body);
                    match cmd {
                        CMD_HELLO if !greeted => {
                            greeted = true;
                            let mut out = msg(MSG_DEVICE_INFO, 0, 0, 0, &spec.device_info());
                            out.extend_from_slice(&msg(
                                MSG_CLIENT_SYNC,
                                0,
                                0,
                                0,
                                &spec.client_sync(gain),
                            ));
                            if write_chunked(&mut sock, &out).is_err() {
                                return;
                            }
                        }
                        CMD_SET_SETTING if body >= 8 => {
                            let setting = u32::from_le_bytes(payload[0..4].try_into().unwrap());
                            let value = u32::from_le_bytes(payload[4..8].try_into().unwrap());
                            thread_log.lock().unwrap().push(Rec { setting, value });
                            if setting == SETTING_GAIN {
                                gain = value;
                            }
                            // Echoed back in every I/Q header from here on,
                            // which is what a real server does and what lets a
                            // client tell a figure that has landed from one
                            // still in flight.
                            if setting == SETTING_IQ_DIGITAL_GAIN
                                && !(spec.stale_iq_gain && digital_gain.is_some())
                            {
                                digital_gain = Some(value as u16);
                            }
                            if setting == SETTING_STREAMING_ENABLED {
                                streaming = value != 0;
                            }
                        }
                        // A PONG has no body at all, which is the framer path
                        // a zero-length message exercises.
                        CMD_PING if sock.write_all(&msg(MSG_PONG, 0, 0, 0, &[])).is_err() => {
                            return;
                        }
                        _ => {}
                    }
                }

                if streaming {
                    let mut out = Vec::new();
                    if spec.stream_iq {
                        // Four complex samples: mid-scale, then the extremes —
                        // or whatever byte the test asked to be flooded with.
                        let body = match spec.iq_fill {
                            Some(byte) => vec![byte; 8],
                            None => vec![128, 128, 255, 0, 128, 128, 0, 255],
                        };
                        let flags = digital_gain.unwrap_or(0);
                        out.extend_from_slice(&msg(MSG_UINT8_IQ, flags, 1, seq, &body));
                    }
                    if spec.stream_fft {
                        let bins: Vec<u8> = (0..64u32).map(|i| (i * 4) as u8).collect();
                        out.extend_from_slice(&msg(MSG_UINT8_FFT, 0, 4, seq, &bins));
                    }
                    seq = seq.wrapping_add(1);
                    if !out.is_empty() && write_chunked(&mut sock, &out).is_err() {
                        return;
                    }
                }
                std::thread::sleep(Duration::from_millis(5));
            }
        });

        Fake { addr, log, inject, stopped, _join: Some(join) }
    }

    fn cfg(&self) -> SpyServerConfig {
        SpyServerConfig { address: self.addr.clone(), ..SpyServerConfig::default() }
    }

    fn settings(&self) -> Vec<Rec> {
        self.log.lock().unwrap().clone()
    }

    /// Index of the first record for `setting`, for order assertions.
    fn index_of(&self, setting: u32) -> Option<usize> {
        self.settings().iter().position(|r| r.setting == setting)
    }

    fn value_of(&self, setting: u32) -> Option<u32> {
        self.settings().iter().find(|r| r.setting == setting).map(|r| r.value)
    }

    fn values_of(&self, setting: u32) -> Vec<u32> {
        self.settings().iter().filter(|r| r.setting == setting).map(|r| r.value).collect()
    }
}

impl Drop for Fake {
    fn drop(&mut self) {
        self.stopped.store(true, Ordering::Relaxed);
    }
}

/// Wait for `f` to hold, up to `limit`. The pump is a separate thread and
/// everything here is asynchronous by nature; a fixed sleep would be either
/// slow or flaky, and usually both.
fn eventually(limit: Duration, mut f: impl FnMut() -> bool) -> bool {
    let deadline = Instant::now() + limit;
    while Instant::now() < deadline {
        if f() {
            return true;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    f()
}

// --- the handshake -----------------------------------------------------------

/// The opening sequence, in full and in order. Two things in it are not
/// negotiable: the streaming mode has to precede the gain, because a server may
/// reset its gain state when the mode changes; and the digital gain goes last,
/// because it is computed from the gain index just sent.
#[test]
fn the_handshake_states_the_settings_in_dependency_order() {
    let fake = Fake::start(Spec::default());
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let _h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");

    assert!(eventually(Duration::from_secs(2), || fake
        .index_of(SETTING_STREAMING_ENABLED)
        .is_some()));
    let got: Vec<u32> = fake.settings().iter().map(|r| r.setting).collect();
    assert_eq!(
        got,
        vec![
            SETTING_IQ_FORMAT,
            SETTING_IQ_DECIMATION,
            SETTING_IQ_FREQUENCY,
            SETTING_STREAMING_MODE,
            SETTING_GAIN,
            SETTING_IQ_DIGITAL_GAIN,
            SETTING_STREAMING_ENABLED,
        ],
    );
    assert_eq!(fake.value_of(SETTING_STREAMING_MODE), Some(STREAM_MODE_IQ_ONLY));
    assert_eq!(fake.value_of(SETTING_IQ_FORMAT), Some(1), "8-bit");
    // 2.4 Msps ladder, aiming at 1 Msps: 2.4M, 1.2M, 600k — stage 2.
    assert_eq!(fake.value_of(SETTING_IQ_DECIMATION), Some(2));
    assert_eq!(fake.value_of(SETTING_IQ_FREQUENCY), Some(100_000_000));
    // An RTL-SDR gets decimation gain only: 2 stages × 3.01 dB.
    assert_eq!(fake.value_of(SETTING_IQ_DIGITAL_GAIN), Some(6));
}

/// With the FFT lane on, its whole block lands between the I/Q lane's shape and
/// the streaming mode — and the mode becomes the combined one.
///
/// The I/Q *frequency* is the exception, and it is deliberately last of the
/// two windows: with this lane running the receiver follows the FFT window, and
/// `IQ_FREQUENCY` only places the I/Q window inside the band the receiver is
/// already on. Sending it first placed it against a band the receiver was about
/// to leave.
#[test]
fn the_fft_block_is_configured_before_the_streaming_mode() {
    let fake = Fake::start(Spec::default());
    let _h = SpyServerHandle::connect_vfo(&fake.cfg(), 100_000_000.0).expect("connect");

    assert!(eventually(Duration::from_secs(2), || fake
        .index_of(SETTING_STREAMING_ENABLED)
        .is_some()));
    let mode_at = fake.index_of(SETTING_STREAMING_MODE).expect("a streaming mode");
    let gain_at = fake.index_of(SETTING_GAIN).expect("a gain");
    let dgain_at = fake.index_of(SETTING_IQ_DIGITAL_GAIN).expect("a digital gain");
    let shape_at = fake.index_of(SETTING_IQ_DECIMATION).expect("an I/Q decimation");
    let freq_at = fake.index_of(SETTING_IQ_FREQUENCY).expect("an I/Q frequency");

    for s in [
        SETTING_FFT_FORMAT,
        SETTING_FFT_DECIMATION,
        SETTING_FFT_FREQUENCY,
        SETTING_FFT_DISPLAY_PIXELS,
        SETTING_FFT_DB_OFFSET,
        SETTING_FFT_DB_RANGE,
    ] {
        let at = fake.index_of(s).unwrap_or_else(|| panic!("setting {s} was never sent"));
        assert!(at > shape_at, "setting {s} must follow the I/Q lane's shape");
        assert!(at < mode_at, "setting {s} must precede the streaming mode");
    }
    let fft_freq_at = fake.index_of(SETTING_FFT_FREQUENCY).expect("an FFT frequency");
    assert!(
        fft_freq_at < freq_at,
        "the window the receiver follows has to be placed before the window that sits inside it",
    );
    assert!(mode_at < gain_at, "the streaming mode must precede the gain");
    assert!(gain_at < dgain_at, "the digital gain is computed from the gain index");

    assert_eq!(fake.value_of(SETTING_STREAMING_MODE), Some(STREAM_MODE_FFT_IQ));
    assert_eq!(fake.value_of(SETTING_FFT_FORMAT), Some(1), "8-bit, never the 4-bit coding");
    assert_eq!(fake.value_of(SETTING_FFT_DISPLAY_PIXELS), Some(2048));
    assert_eq!(fake.value_of(SETTING_FFT_DB_RANGE), Some(150));
    // The VFO ladder aims at 96 kHz and drops the bottom rung: 2.4M / 2^5 = 75k.
    assert_eq!(fake.value_of(SETTING_IQ_DECIMATION), Some(5));
}

// --- the streams -------------------------------------------------------------

#[test]
fn iq_arrives_through_a_stream_split_across_segments() {
    let fake = Fake::start(Spec { stream_iq: true, ..Spec::default() });
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let mut h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");

    let mut got = vec![0f32; 64];
    let mut n = 0;
    assert!(
        eventually(Duration::from_secs(3), || {
            n = h.rx_read(&mut got);
            n >= 8
        }),
        "no I/Q arrived",
    );
    assert_eq!(n % 2, 0, "I and Q must stay paired");
    // The fake sends mid-scale, then the extremes: 128 -> 0, 255 -> ~+1, 0 -> -1,
    // divided back down by the digital gain its header states — which at the
    // ladder's stage is not zero, and undoing it is the decoder's contract.
    let gain =
        10f32.powf(fake.value_of(SETTING_IQ_DIGITAL_GAIN).expect("a gain was set") as f32 / 20.0);
    assert_eq!(got[0], 0.0);
    assert_eq!(got[1], 0.0);
    assert!((got[2] - (127.0 / 128.0) / gain).abs() < 1e-6, "got {}", got[2]);
    assert!((got[3] + 1.0 / gain).abs() < 1e-6, "got {}", got[3]);
}

/// The FFT lane's contract in one test: bins mapped against the window that was
/// requested, labelled with the span the server actually delivers, and `None`
/// when nothing new has arrived.
#[test]
fn an_fft_frame_carries_the_window_and_the_span_it_was_drawn_at() {
    let fake = Fake::start(Spec { stream_fft: true, ..Spec::default() });
    let h = SpyServerHandle::connect_vfo(&fake.cfg(), 100_000_000.0).expect("connect");

    let mut frame = None;
    assert!(
        eventually(Duration::from_secs(3), || {
            frame = h.take_fft();
            frame.is_some()
        }),
        "no FFT frame arrived",
    );
    let f = frame.expect("a frame");
    assert_eq!(f.bins.len(), 64);
    // Stage 0 covers the receiver's whole analog bandwidth, not its rate label.
    assert_eq!(f.span_hz, 2_400_000.0);
    assert_eq!(f.center_hz, 100_000_000.0);
    // The default window is 0 dB down 150: byte 0 is the floor.
    assert!((f.bins[0] - -150.0).abs() < 1e-3, "{}", f.bins[0]);
    // Byte 4*63 = 252 is nearly the top.
    assert!(f.bins[63] > -5.0 && f.bins[63] <= 0.0, "{}", f.bins[63]);

    // Taken once, gone: the engine reads `None` as "nothing new", and a frame
    // handed over twice would be drawn as a fresh row of a band picture.
    assert!(h.take_fft().is_none() || h.take_fft().is_none());
}

/// Only the newest frame survives. A queue here would turn a slow consumer
/// into growing latency on a picture whose whole value is being current.
#[test]
fn only_the_newest_fft_frame_is_kept() {
    let fake = Fake::start(Spec { stream_fft: true, ..Spec::default() });
    let h = SpyServerHandle::connect_vfo(&fake.cfg(), 100_000_000.0).expect("connect");
    assert!(eventually(Duration::from_secs(3), || h.take_fft().is_some()));

    // Three frames with distinguishable first bins, injected back to back.
    for first in [10u8, 20, 30] {
        let mut bins = vec![0u8; 64];
        bins[0] = first;
        fake.inject.send(msg(MSG_UINT8_FFT, 0, 4, 9, &bins)).expect("inject");
    }
    let mut last = None;
    assert!(eventually(Duration::from_secs(2), || {
        if let Some(f) = h.take_fft() {
            last = Some(f);
        }
        // 30/255 of the way up a 150 dB window from -150.
        last.as_ref().is_some_and(|f| (f.bins[0] - (-150.0 + 150.0 * 30.0 / 255.0)).abs() < 0.5)
    }));
}

/// The 4-bit differential FFT coding is documented nowhere, so it is never
/// requested — and a server that sends one anyway must not take the receiver
/// down with it.
#[test]
fn a_dint4_fft_frame_is_ignored_and_the_iq_keeps_flowing() {
    let fake = Fake::start(Spec { stream_iq: true, ..Spec::default() });
    let mut h = SpyServerHandle::connect_vfo(&fake.cfg(), 100_000_000.0).expect("connect");
    fake.inject.send(msg(MSG_DINT4_FFT, 0, 4, 1, &[0xAB; 32])).expect("inject");

    let mut got = vec![0f32; 64];
    assert!(
        eventually(Duration::from_secs(3), || h.rx_read(&mut got) >= 8),
        "the I/Q stopped after an unreadable FFT frame",
    );
    assert!(h.is_alive(), "an undecodable band view is not a reason to drop a receiver");
    assert!(h.take_fft().is_none(), "nothing decodable came out of it");
}

/// 24-bit I/Q is not decoded by any open client and is not guessed at here.
#[test]
fn a_twenty_four_bit_stream_is_refused_rather_than_misread() {
    let fake = Fake::start(Spec::default());
    let h = SpyServerHandle::connect_wideband(&fake.cfg(), 100_000_000.0).expect("connect");
    fake.inject.send(msg(MSG_INT24_IQ, 0, 1, 1, &[1, 2, 3, 4, 5, 6])).expect("inject");
    assert!(
        eventually(Duration::from_secs(2), || !h.is_alive()),
        "a stream this client cannot read must stop rather than be decoded as something else",
    );
}

#[test]
fn a_server_that_forces_an_undecodable_format_fails_the_connect() {
    let fake = Fake::start(Spec { forced_format: 3, ..Spec::default() });
    let e = SpyServerHandle::connect_wideband(&fake.cfg(), 100_000_000.0)
        .expect_err("24-bit is forced");
    assert!(e.to_string().contains("24-bit"), "{e}");
}

/// A forced format this client *can* read is adopted, overriding what was
/// configured — the server's insistence wins, and silently sending 8-bit to a
/// server that will answer in 16 would be a waterfall full of noise.
#[test]
fn a_forced_format_overrides_the_configured_one() {
    let fake = Fake::start(Spec { forced_format: 2, ..Spec::default() });
    let cfg =
        SpyServerConfig { iq_format: SpyServerFormat::Uint8, fft_enabled: false, ..fake.cfg() };
    let _h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");
    assert!(eventually(Duration::from_secs(2), || fake.value_of(SETTING_IQ_FORMAT).is_some()));
    assert_eq!(fake.value_of(SETTING_IQ_FORMAT), Some(2), "16-bit, as the server insisted");
}

// --- tuning ------------------------------------------------------------------

/// A drag emits hundreds of centres a second and the far end takes about
/// eight. What matters as much as the limit is that the *last* one still
/// arrives: dropping it would leave the receiver wherever the limiter happened
/// to bite rather than where the operator's hand stopped.
#[test]
fn retunes_are_rate_limited_but_the_last_one_still_arrives() {
    let fake = Fake::start(Spec::default());
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");
    assert!(eventually(Duration::from_secs(2), || fake.value_of(SETTING_IQ_FREQUENCY).is_some()));
    let before = fake.values_of(SETTING_IQ_FREQUENCY).len();

    let start = Instant::now();
    let mut last = 0u32;
    while start.elapsed() < Duration::from_millis(500) {
        last = last.wrapping_add(1000).max(100_001_000);
        h.set_center_hz(f64::from(last));
        std::thread::sleep(Duration::from_millis(2));
    }
    let target = f64::from(last);
    h.set_center_hz(target);

    assert!(eventually(Duration::from_secs(2), || fake
        .values_of(SETTING_IQ_FREQUENCY)
        .last()
        .copied()
        == Some(last)));
    let sent = fake.values_of(SETTING_IQ_FREQUENCY).len() - before;
    // 500 ms at eight a second is four, plus the trailing flush and a little
    // slack for a loaded test machine.
    assert!(sent <= 10, "{sent} retunes in half a second is not rate limited");
    assert!(sent >= 1, "nothing was sent at all");
    assert_eq!(h.reachable_center(target), target);
}

/// The configured gain reaches the server, and the digital gain computed from
/// it goes with it.
///
/// The trap here is that a `CLIENT_SYNC` arrives *before* the opening settings
/// are sent, carrying whatever gain the server was already sitting at. Taking
/// that as the truth would overwrite the operator's setting with a stale one
/// and then send it back — quietly, and dragging the digital gain along, since
/// that is computed from the index.
#[test]
fn the_configured_gain_survives_the_servers_opening_sync() {
    // An Airspy R2, whose digital-gain formula depends on the gain index —
    // which is what makes a wrong index visible in a second setting.
    let fake = Fake::start(Spec {
        device_type: 1,
        max_rate: 10_000_000,
        max_bw: 10_000_000,
        stages: 8,
        min_decim: 2,
        max_gain: 21,
        ..Spec::default()
    });
    let cfg = SpyServerConfig { gain_index: 9, iq_decimation: 4, fft_enabled: false, ..fake.cfg() };
    let _h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");

    assert!(eventually(Duration::from_secs(2), || fake
        .value_of(SETTING_IQ_DIGITAL_GAIN)
        .is_some()));
    assert_eq!(fake.value_of(SETTING_GAIN), Some(9), "the server's opening sync said 0");
    // (21 - 9) + 4 stages × 3.01 dB = 24.04.
    assert_eq!(fake.value_of(SETTING_IQ_DIGITAL_GAIN), Some(24));
}

/// An Airspy HF+ gets the reference client's digital gain and nothing more:
/// decimation gain, and at stage 0 that is nothing at all. A constant boost
/// keyed on the stream being 8-bit was tried and reverted — the gain a band
/// needs is a property of the band, and 44 dB drove the quantiser twenty-odd
/// dB past full scale on every band with a signal on it, which is not a level
/// error but a stream of rails with no information left in it.
#[test]
fn an_hf_plus_is_asked_for_the_reference_digital_gain_in_either_format() {
    // As a real HF+ server describes itself: 768 ksps, 660 kHz of analog
    // bandwidth, no gain stages, and a 16-bit ADC.
    let hf_plus = Spec {
        device_type: 2,
        max_rate: 768_000,
        max_bw: 660_000,
        stages: 8,
        min_decim: 0,
        max_gain: 0,
        min_freq: 0,
        max_freq: 1_700_000_000,
        resolution: 16,
        ..Spec::default()
    };

    let fake = Fake::start(hf_plus);
    let cfg = SpyServerConfig {
        iq_format: SpyServerFormat::Uint8,
        iq_decimation: 0,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 1_081_400.0).expect("connect");
    assert!(eventually(Duration::from_secs(2), || fake
        .value_of(SETTING_IQ_DIGITAL_GAIN)
        .is_some()));
    assert_eq!(fake.value_of(SETTING_IQ_DIGITAL_GAIN), Some(0), "8-bit at stage 0");

    // The same receiver sent as 16-bit: the same figure, the format is not
    // an input.
    let fake = Fake::start(hf_plus);
    let cfg = SpyServerConfig {
        iq_format: SpyServerFormat::Int16,
        iq_decimation: 0,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 1_081_400.0).expect("connect");
    assert!(eventually(Duration::from_secs(2), || fake
        .value_of(SETTING_IQ_DIGITAL_GAIN)
        .is_some()));
    assert_eq!(fake.value_of(SETTING_IQ_DIGITAL_GAIN), Some(0), "16-bit at stage 0");
}

// --- the digital-gain loop ---------------------------------------------------

/// An HF+ as a real server describes one: 768 ksps, 660 kHz of analog
/// bandwidth, no gain stages, a 16-bit ADC — streaming whatever byte the test
/// asks for, under whatever digital gain it was last set to.
fn hf_plus_streaming(byte: u8) -> Spec {
    Spec {
        device_type: 2,
        max_rate: 768_000,
        max_bw: 660_000,
        stages: 8,
        min_decim: 0,
        max_gain: 0,
        min_freq: 0,
        max_freq: 1_700_000_000,
        resolution: 16,
        stream_iq: true,
        iq_fill: Some(byte),
        ..Spec::default()
    }
}

/// The digital gain is a closed loop on an 8-bit stream: a server sending
/// rails is a server being driven past full scale, and the client has to back
/// off rather than divide the wreckage by an ever larger number. Nothing the
/// loop does shows up in the level — the decoder divides by the gain the
/// header states, so the samples come out the same amplitude either side of a
/// change; what moves is how many of the eight bits the signal is using.
///
/// It comes down a step at a time rather than in one jump, because a rail
/// hides how far past full scale it is: a byte pinned at 255 reads the same
/// 6 dB over as 30 dB over. Each step is what would put a rail at the target,
/// and the next step sees an honest peak as soon as the rail lets go.
#[test]
fn an_eight_bit_stream_that_arrives_clipped_walks_the_digital_gain_down() {
    let fake = Fake::start(hf_plus_streaming(255));
    let cfg = SpyServerConfig {
        // Stage 4, so the reference formula opens at 12 dB and the loop has
        // somewhere to go. At stage 0 it opens at nothing and the test would
        // pass without a loop at all.
        iq_format: SpyServerFormat::Uint8,
        iq_decimation: 4,
        auto_digital_gain: true,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 1_081_400.0).expect("connect");

    assert!(
        eventually(Duration::from_secs(2), || fake.value_of(SETTING_IQ_DIGITAL_GAIN) == Some(12)),
        "it opens at the reference formula's figure"
    );
    assert!(
        eventually(Duration::from_secs(5), || fake
            .values_of(SETTING_IQ_DIGITAL_GAIN)
            .last()
            .copied()
            == Some(0)),
        "the rails never walked it down to the floor: {:?}",
        fake.values_of(SETTING_IQ_DIGITAL_GAIN)
    );
    // 6 dB a step: a rail at full scale, brought to the target of half.
    assert_eq!(fake.values_of(SETTING_IQ_DIGITAL_GAIN), vec![12, 6, 0]);

    // And at the floor it stops asking. Repeating a figure the server already
    // has costs one of the eight settings a second it will take.
    std::thread::sleep(Duration::from_millis(600));
    assert_eq!(fake.values_of(SETTING_IQ_DIGITAL_GAIN), vec![12, 6, 0], "still at the stop");
}

/// Every step is measured from the gain the samples' *own header* states, so a
/// link with something in flight cannot make the loop answer the same rail
/// twice. The fake here never stops claiming the figure it opened at — a
/// message always in flight, which is the case on exactly the slow uplinks an
/// 8-bit stream is chosen for — and the loop takes its one honest step and then
/// waits for samples that step actually reached, rather than reading its own
/// unanswered rails as an argument for walking the gain to the floor.
#[test]
fn a_rail_sent_under_a_superseded_gain_is_not_answered_twice() {
    let fake = Fake::start(Spec { stale_iq_gain: true, ..hf_plus_streaming(255) });
    let cfg = SpyServerConfig {
        // Stage 4, so the reference formula opens at 12 dB and there is room
        // below to walk into if the loop double-counts.
        iq_format: SpyServerFormat::Uint8,
        iq_decimation: 4,
        auto_digital_gain: true,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 1_081_400.0).expect("connect");
    assert!(
        eventually(Duration::from_secs(2), || fake
            .values_of(SETTING_IQ_DIGITAL_GAIN)
            .last()
            .copied()
            == Some(6)),
        "the rail was never answered at all: {:?}",
        fake.values_of(SETTING_IQ_DIGITAL_GAIN)
    );
    // Long enough for four more steps at `GAIN_MIN_INTERVAL`, had it taken any.
    std::thread::sleep(Duration::from_secs(1));
    assert_eq!(
        fake.values_of(SETTING_IQ_DIGITAL_GAIN),
        vec![12, 6],
        "a rail already answered walked the gain down again"
    );
}

/// The other half of the loop, and the reason it exists at all: on a quiet
/// band an HF+'s 8-bit I/Q sits below one LSB and arrives as mid-scale, every
/// sample, which is no information at all. That stream *has* arrived, and it
/// is the strongest possible argument for more gain — a loop that read an
/// all-zero peak as "nothing to judge by" would leave the band dead forever.
///
/// It goes up slowly: a step at a time, and only after the peak has been below
/// target for a whole two seconds, so a pause in the traffic does not pump the
/// gain up for the next syllable to clip on.
#[test]
fn an_eight_bit_stream_that_arrives_below_one_lsb_walks_the_digital_gain_up_slowly() {
    let fake = Fake::start(hf_plus_streaming(128));
    let cfg = SpyServerConfig {
        iq_format: SpyServerFormat::Uint8,
        iq_decimation: 0,
        auto_digital_gain: true,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 14_100_000.0).expect("connect");
    assert!(
        eventually(Duration::from_secs(2), || fake.value_of(SETTING_IQ_DIGITAL_GAIN) == Some(0))
    );

    // Not yet: room to spare has to be a settled fact before it is acted on.
    std::thread::sleep(Duration::from_millis(1000));
    assert_eq!(fake.values_of(SETTING_IQ_DIGITAL_GAIN), vec![0], "raised before the quiet time");

    assert!(
        eventually(Duration::from_secs(4), || fake
            .values_of(SETTING_IQ_DIGITAL_GAIN)
            .last()
            .copied()
            == Some(3)),
        "a stream of mid-scale never raised the gain: {:?}",
        fake.values_of(SETTING_IQ_DIGITAL_GAIN)
    );
    // One step, not a leap, however far below target the peak is.
    assert_eq!(fake.values_of(SETTING_IQ_DIGITAL_GAIN), vec![0, 3]);
}

/// The loop is for 8-bit streams and nothing else. A 16-bit one has 48 dB of
/// room it will never need, and moving its gain would spend the server's
/// settings budget for no gain in the samples.
///
/// The fake still sends 8-bit messages here, on purpose: the client reads the
/// body in the format it *asked* for, so what the bytes mean is beside the
/// point — what matters is that a stream is flowing and the loop leaves it be.
#[test]
fn a_sixteen_bit_stream_is_left_at_the_reference_figure() {
    let fake = Fake::start(hf_plus_streaming(255));
    let cfg = SpyServerConfig {
        iq_format: SpyServerFormat::Int16,
        iq_decimation: 4,
        auto_digital_gain: true,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 1_081_400.0).expect("connect");
    assert!(
        eventually(Duration::from_secs(2), || fake.value_of(SETTING_IQ_DIGITAL_GAIN) == Some(12))
    );
    std::thread::sleep(Duration::from_secs(1));
    assert_eq!(
        fake.values_of(SETTING_IQ_DIGITAL_GAIN),
        vec![12],
        "a 16-bit stream keeps the figure the formula gave it, and is asked once"
    );
}

/// The operator's own figure is never touched. Automatic off means a number
/// was typed, and the loop has no business second-guessing it — including the
/// right to clip a whole band flat.
#[test]
fn a_manual_digital_gain_is_sent_as_typed_and_left_alone() {
    let fake = Fake::start(hf_plus_streaming(255));
    let cfg = SpyServerConfig {
        iq_format: SpyServerFormat::Uint8,
        iq_decimation: 0,
        auto_digital_gain: false,
        digital_gain_db: 40.0,
        fft_enabled: false,
        ..fake.cfg()
    };
    let _h = SpyServerHandle::connect_wideband(&cfg, 1_081_400.0).expect("connect");
    assert!(
        eventually(Duration::from_secs(2), || fake.value_of(SETTING_IQ_DIGITAL_GAIN) == Some(40))
    );
    std::thread::sleep(Duration::from_secs(1));
    assert_eq!(fake.values_of(SETTING_IQ_DIGITAL_GAIN), vec![40], "the rails are the operator's");
}

/// The opposite case: where another client owns the receiver, the gain is
/// theirs and what they set is the only true answer.
#[test]
fn a_shared_servers_gain_is_taken_from_the_server() {
    let fake = Fake::start(Spec { can_control: 0, ..Spec::default() });
    let cfg = SpyServerConfig { gain_index: 9, fft_enabled: false, ..fake.cfg() };
    let h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");
    assert_eq!(h.gain_index(), 0, "the owner has it at 0, whatever this end configured");
}

/// The band view holds still while the dial roams inside it. This is the
/// property the whole VFO interface rests on: the narrow window chases the
/// operator, and the picture they are steering by does not move under them.
#[test]
fn the_band_view_holds_while_the_narrow_window_chases_the_dial() {
    // A wide receiver, so the FFT window has room to slide and the hysteresis
    // is actually exercised — at stage 0 it would be pinned and prove nothing.
    let fake = Fake::start(Spec {
        max_rate: 10_000_000,
        max_bw: 10_000_000,
        stages: 8,
        device_center: 100_000_000,
        ..Spec::default()
    });
    let cfg = SpyServerConfig { fft_decimation: 2, ..fake.cfg() };
    let h = SpyServerHandle::connect_vfo(&cfg, 100_000_000.0).expect("connect");
    assert!(eventually(Duration::from_secs(2), || fake
        .value_of(SETTING_STREAMING_ENABLED)
        .is_some()));
    let fft_before = fake.values_of(SETTING_FFT_FREQUENCY).len();

    // A slow walk of 600 kHz, well inside the 2.5 MHz FFT window's middle 70 %.
    for step in 1..=6 {
        h.set_center_hz(100_000_000.0 + f64::from(step) * 100_000.0);
        std::thread::sleep(Duration::from_millis(140));
    }
    assert!(eventually(Duration::from_secs(2), || fake
        .values_of(SETTING_IQ_FREQUENCY)
        .last()
        .copied()
        == Some(100_600_000)));
    assert_eq!(
        fake.values_of(SETTING_FFT_FREQUENCY).len(),
        fft_before,
        "the band view must not follow every retune",
    );

    // Out past the edge, and it re-centres on the dial — once.
    h.set_center_hz(101_500_000.0);
    assert!(
        eventually(Duration::from_secs(2), || fake.values_of(SETTING_FFT_FREQUENCY).len()
            > fft_before),
        "the band view never caught up with a dial that left it",
    );
    assert_eq!(fake.values_of(SETTING_FFT_FREQUENCY).last().copied(), Some(101_500_000));
}

/// A server whose receiver belongs to another client can only be tuned inside
/// the slice that client is already receiving — and it has to say so rather
/// than quietly clamp, or the dial ends up somewhere the samples are not.
#[test]
fn a_shared_server_reports_only_the_window_it_can_reach() {
    let fake = Fake::start(Spec { can_control: 0, ..Spec::default() });
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");

    assert!(!h.can_control(), "the fake said another client owns it");
    // 600 kHz of window inside 2.4 MHz of receiver leaves 900 kHz either side.
    let (lo, hi) = h.tuning_window();
    assert_eq!((lo, hi), (99_100_000.0, 100_900_000.0));

    // Inside the window is reachable exactly; outside it is not.
    assert_eq!(h.reachable_center(100_500_000.0), 100_500_000.0);
    assert_eq!(h.reachable_center(101_000_000.0), 100_900_000.0);
    assert_eq!(h.reachable_center(90_000_000.0), 99_100_000.0);
}

/// With control, the whole receiver is on offer — the server moves its own
/// centre to follow, and computing that move on this side would only be a
/// second opinion about something the server already decides.
#[test]
fn a_server_we_control_offers_its_whole_range() {
    let fake = Fake::start(Spec::default());
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");
    assert!(h.can_control());
    assert_eq!(h.tuning_window(), (24_000_000.0, 1_766_000_000.0));
    assert_eq!(h.reachable_center(430_000_000.0), 430_000_000.0);
    assert_eq!(h.reachable_center(10_000_000.0), 24_000_000.0, "below the receiver");
}

/// The owner of a shared server moving its receiver is news exactly once; a
/// restatement of the same position is not.
#[test]
fn a_device_move_bumps_the_sync_counter_and_an_unchanged_sync_does_not() {
    let spec = Spec { can_control: 0, ..Spec::default() };
    let fake = Fake::start(spec);
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");
    let before = h.sync_seq();

    // The same position again: nothing has happened.
    fake.inject.send(msg(MSG_CLIENT_SYNC, 0, 0, 0, &spec.client_sync(0))).expect("inject");
    std::thread::sleep(Duration::from_millis(150));
    assert_eq!(h.sync_seq(), before, "an unchanged sync is not a move");

    // Somewhere else: that is.
    let moved = Spec { device_center: 145_000_000, ..spec };
    fake.inject.send(msg(MSG_CLIENT_SYNC, 0, 0, 0, &moved.client_sync(0))).expect("inject");
    assert!(eventually(Duration::from_secs(2), || h.sync_seq() != before));
    assert_eq!(h.device_center_hz(), 145_000_000.0);
    // And the window it can reach moved with it.
    assert_eq!(h.tuning_window(), (144_100_000.0, 145_900_000.0));
}

// --- things that go wrong ----------------------------------------------------

#[test]
fn nothing_listening_names_what_to_check() {
    // Bind then drop, so the port is almost certainly free and nothing is on it.
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr").to_string();
    drop(listener);

    let cfg = SpyServerConfig { address: addr, ..SpyServerConfig::default() };
    let e = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect_err("nothing there");
    let s = e.to_string();
    assert!(s.contains("spyserver is running"), "{s}");
}

/// Something that answers but is not a SpyServer — a web server on the port,
/// which is the commonest way to get this wrong.
#[test]
fn a_server_that_is_not_a_spyserver_says_so() {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind");
    let addr = listener.local_addr().expect("addr").to_string();
    std::thread::spawn(move || {
        if let Ok((mut sock, _)) = listener.accept() {
            let _ = sock.write_all(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n");
            std::thread::sleep(Duration::from_millis(500));
        }
    });

    let cfg = SpyServerConfig { address: addr, ..SpyServerConfig::default() };
    let e = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect_err("not a SpyServer");
    let s = e.to_string();
    assert!(s.contains("protocol") || s.contains("device information"), "{s}");
}

/// A body size past the protocol's own ceiling is refused with the size named,
/// and — the point of the test — without trying to allocate it first.
#[test]
fn an_absurd_body_size_is_refused_without_allocating_it() {
    let fake = Fake::start(Spec::default());
    let cfg = SpyServerConfig { fft_enabled: false, ..fake.cfg() };
    let h = SpyServerHandle::connect_wideband(&cfg, 100_000_000.0).expect("connect");

    let mut bad = Vec::new();
    bad.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    bad.extend_from_slice(&u32::from(MSG_UINT8_IQ).to_le_bytes());
    bad.extend_from_slice(&1u32.to_le_bytes());
    bad.extend_from_slice(&0u32.to_le_bytes());
    bad.extend_from_slice(&(MAX_MESSAGE_BODY_SIZE + 1).to_le_bytes());
    fake.inject.send(bad).expect("inject");

    assert!(
        eventually(Duration::from_secs(2), || !h.is_alive()),
        "a stream out of step has nothing to resynchronise on and must stop",
    );
}

/// A server with nothing plugged into it is a different failure from a server
/// that is not there, and worth saying differently.
#[test]
fn a_server_with_no_receiver_is_refused_by_name() {
    let fake = Fake::start(Spec { device_type: 0, ..Spec::default() });
    let e = SpyServerHandle::connect_wideband(&fake.cfg(), 100_000_000.0).expect_err("no device");
    assert!(e.to_string().contains("no receiver"), "{e}");
}

// --- the address ------------------------------------------------------------

/// The port is supplied when it was left off, and a bracketed IPv6 literal —
/// which is all colons — is not mistaken for one that already has a port.
#[test]
fn the_default_port_is_supplied_including_for_ipv6() {
    let at = |a: &str| SpyServerConfig { address: a.into(), ..Default::default() }.endpoint();
    assert_eq!(at("example.org"), "example.org:5555");
    assert_eq!(at("example.org:1234"), "example.org:1234");
    assert_eq!(at("192.168.1.10"), "192.168.1.10:5555");
    assert_eq!(at("  pi.local  "), "pi.local:5555");
    assert_eq!(at("[::1]"), "[::1]:5555");
    assert_eq!(at("[2001:db8::1]:5555"), "[2001:db8::1]:5555");
}
