//! The SpyServer client: connect, configure, and keep two streams flowing.
//!
//! One blocking thread owns the socket. Control arrives over a crossbeam
//! channel and is coalesced ([`Pending`]); I/Q leaves through an `rtrb` ring;
//! full-band FFT frames leave through the newest-wins slot in [`Shared`].
//!
//! Three things run through everything below.
//!
//! *The connection is made before the thread starts.* Unlike the `rtl_tcp`
//! client next door, which must do everything on one thread because USB
//! register access demands it, here the handshake has to finish before the ring
//! can even be sized: the sample rate is `MaximumSampleRate / 2ⁿ` and nothing
//! on this side knows either number until the server has said so.
//!
//! *A framing error is fatal.* This protocol is message-framed and has no
//! resync marker of any kind, so one byte out of step makes every subsequent
//! header garbage. The thread stops, the handle goes dead, and the engine
//! reconnects — which is both correct and cheap.
//!
//! *The far end paces the retunes, not the operator.* A dial drag emits
//! hundreds of centres a second and the server will take about eight, so
//! retunes are deferred and coalesced rather than dropped: whatever the
//! operator's hand stopped on always arrives.

use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use rtrb::Producer;
use sdroxide_types::{SpyServerConfig, SpyServerFormat};

use crate::decode::{fft_to_db, iq_to_f32};
use crate::error::{Error, Result};
use crate::handle::{
    Ctrl, FftFrame, Pending, RxStats, Shared, SpyServerHandle, push_iq, resolve_stage, ring_for,
};
use crate::proto::{self, ClientSync, DeviceInfo, MessageHeader};

/// What this client calls itself in the handshake. Servers log it, and some
/// keep an allow-list of application names, so it is worth being recognisable.
pub const CLIENT_NAME: &str = "sdroxide";

/// How long to wait for the TCP connection itself. A server that is running
/// answers a LAN connect in milliseconds; this length is for a host that is up
/// with nothing listening on a filtered port, which is a timeout rather than a
/// refusal.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(5);

/// How long the server has to answer the handshake with its device
/// information. Generous: a server opens and configures its receiver first,
/// which on a Raspberry Pi with a cold USB stack is not instant.
const HANDSHAKE_TIMEOUT: Duration = Duration::from_secs(5);

/// How long a read may block before the loop goes back to serve control.
/// Bounds how long a dial drag waits when the stream has gone quiet; while
/// samples are flowing, reads return as soon as anything has arrived.
const READ_TIMEOUT: Duration = Duration::from_millis(20);

/// Socket read size. Only a ceiling — a TCP read returns whatever has arrived
/// — so it costs nothing at low rates and saves syscalls at high ones.
const READ_BYTES: usize = 64 * 1024;

/// The floor between retunes. The reference client uses the same eight a
/// second; past that a server spends more time re-tuning than streaming.
const RETUNE_MIN_INTERVAL: Duration = Duration::from_millis(125);

/// How often to ping. Eight bytes, and the reply exercises the write side and
/// the zero-body path in the framer. Not a substitute for the silence timer —
/// what actually catches a dead link is `SpyServerHandle::silent_for`.
const PING_INTERVAL: Duration = Duration::from_secs(3);

/// Where the digital-gain loop parks the peak of an 8-bit stream: half of
/// full scale, so 6 dB of headroom for whatever turns on next. See
/// [`maintain_digital_gain`].
const GAIN_TARGET_PEAK: f32 = 0.5;

/// A peak at or above this is a rail, and a rail means the quantiser is being
/// driven past full scale rather than merely close to it.
const GAIN_CLIP_PEAK: f32 = 0.98;

/// Corrections smaller than this are not worth a setting on the wire: the
/// gain goes out as whole dB, so a tighter deadband is a client that hunts.
const GAIN_DEADBAND_DB: f64 = 3.0;

/// The most one step may add. Coming down is not limited — a clipped stream
/// is already carrying nothing, and dawdling about it costs seconds of audio.
const GAIN_STEP_UP_DB: f64 = 3.0;

/// The floor between gain changes. The server takes about eight settings a
/// second in total and the retune lane is already spending some of them. It is
/// a budget and nothing more: what keeps a figure still in flight from being
/// counted twice is that every step is measured from the gain the samples'
/// own header states.
const GAIN_MIN_INTERVAL: Duration = Duration::from_millis(250);

/// How long the peak must stay below target before the gain is raised.
const GAIN_QUIET_BEFORE_RAISE: Duration = Duration::from_secs(2);

/// The ceiling on `IQ_DIGITAL_GAIN`, in dB.
const GAIN_MAX_DB: f64 = 60.0;

/// Bins to ask the server for.
///
/// Matched to the engine's `DISPLAY_BINS`, which is what the full-band strip
/// is pooled down to: asking for more spends link bandwidth on detail that is
/// pooled away, and asking for less spends it on a picture coarser than the
/// screen. Not a shared constant because `sdroxide-radio` is not — and should
/// not become — a dependency of a device driver.
const FFT_DISPLAY_PIXELS: u32 = 2048;

/// Connect, configure, and start the stream thread.
///
/// Blocks until the connection is up and the far end configured, or has
/// failed — so a wrong address or a server that is not running comes back as
/// an ordinary error rather than as a stream that never starts.
pub(crate) fn spawn(cfg: &SpyServerConfig, center_hz: f64, vfo: bool) -> Result<SpyServerHandle> {
    let mut client = Client::connect(cfg, center_hz, vfo)?;

    let shared = Arc::new(Shared {
        alive: AtomicBool::new(true),
        last_rx_ms: AtomicU64::new(0),
        fft: std::sync::Mutex::new(None),
        can_control: AtomicBool::new(client.can_control),
        device_center_milli_hz: AtomicI64::new((client.device_center * 1000.0) as i64),
        iq_center_milli_hz: AtomicI64::new((client.center * 1000.0) as i64),
        gain_index: AtomicU32::new(client.gain_index),
        sync_seq: AtomicU64::new(0),
        rx_paused: AtomicBool::new(false),
    });

    let (rx_prod, rx_cons) = ring_for(client.iq_rate);
    let (ctrl_tx, ctrl_rx) = crossbeam_channel::unbounded::<Ctrl>();

    let info = client.info;
    let label = client.describe();
    let iq_rate = client.iq_rate;
    let iq_stage = client.iq_stage;
    let fft_span = if client.fft_enabled { client.fft_span } else { 0.0 };

    let thread_shared = Arc::clone(&shared);
    let join = std::thread::Builder::new()
        .name("sdroxide-spyserver".into())
        .spawn(move || {
            let mut rx = rx_prod;
            if let Err(e) = pump(&mut client, &ctrl_rx, &mut rx, &thread_shared) {
                tracing::warn!("SpyServer stream stopped: {e}");
            }
            client.shutdown();
            thread_shared.alive.store(false, Ordering::Relaxed);
        })
        .map_err(|e| Error::Access(format!("could not start the SpyServer thread: {e}")))?;

    Ok(SpyServerHandle::from_parts(
        rx_cons, ctrl_tx, shared, join, info, label, iq_rate, iq_stage, fft_span,
    ))
}

/// Connect, complete the handshake, and disconnect without asking for a
/// stream.
///
/// Deliberately stops short of [`Client::configure`]: a test against a server
/// somebody else is using must not take its receiver, change its gain, or
/// start it streaming to a client that is about to hang up.
pub(crate) fn probe(cfg: &SpyServerConfig, timeout: Duration) -> Result<(DeviceInfo, bool)> {
    let endpoint = cfg.endpoint();
    let sock = dial(&endpoint)?;
    let _ = sock.set_nodelay(true);
    sock.set_read_timeout(Some(timeout))
        .map_err(|e| Error::Net(format!("cannot set a timeout on {endpoint}: {e}")))?;

    let mut client = Client {
        sock,
        endpoint,
        info: DeviceInfo::default(),
        vfo: false,
        framer: Framer::default(),
        iq_format: cfg.iq_format,
        iq_stage: 0,
        iq_rate: 0.0,
        center: 0.0,
        can_control: true,
        device_center: 0.0,
        gain_index: 0,
        auto_digital_gain: true,
        digital_gain_db: 0.0,
        servo_db: 0.0,
        peak: None,
        last_gain_move: Instant::now(),
        quiet_since: None,
        fft_enabled: false,
        fft_stage: 0,
        fft_span: 0.0,
        fft_center: 0.0,
        fft_db_offset: 0.0,
        fft_db_range: SpyServerConfig::FFT_DB_RANGE_MAX,
        deferred_center: None,
        last_retune: Instant::now(),
        last_fft_retune: Instant::now(),
        last_ping: Instant::now(),
        last_iq_seq: None,
    };
    client.handshake(timeout)?;
    let _ = client.sock.shutdown(std::net::Shutdown::Both);
    Ok((client.info, client.can_control))
}

/// Reassembles messages out of a byte stream.
///
/// The carry here is a *message under construction*, not the partial sample
/// pair the `rtl_tcp` client carries. A header split across two segments is
/// the normal case rather than a rare one, and so is a body — an I/Q body is
/// tens of kilobytes and a TCP segment is not.
#[derive(Default)]
struct Framer {
    head: [u8; proto::MSG_HEADER_LEN],
    head_len: usize,
    /// The header of the body currently being collected.
    hdr: Option<MessageHeader>,
    /// Kept allocated across messages; cleared, never dropped.
    body: Vec<u8>,
}

impl Framer {
    /// Feed one socket read in, dispatching every message it completes.
    ///
    /// Never re-scans: each byte is copied into either the header or the body
    /// exactly once.
    fn feed(
        &mut self,
        mut buf: &[u8],
        on_message: &mut dyn FnMut(&MessageHeader, &[u8]) -> Result<()>,
    ) -> Result<()> {
        loop {
            match self.hdr {
                None => {
                    if buf.is_empty() {
                        return Ok(());
                    }
                    let n = (proto::MSG_HEADER_LEN - self.head_len).min(buf.len());
                    self.head[self.head_len..self.head_len + n].copy_from_slice(&buf[..n]);
                    self.head_len += n;
                    buf = &buf[n..];
                    if self.head_len == proto::MSG_HEADER_LEN {
                        // The only place a malformed stream is caught, and the
                        // only place it can be.
                        let h = MessageHeader::parse(&self.head)?;
                        self.head_len = 0;
                        self.body.clear();
                        if h.body_size == 0 {
                            // A PONG has no body. Dispatching it here rather
                            // than waiting for a body that will never come is
                            // what keeps it from wedging the framer.
                            on_message(&h, &[])?;
                        } else {
                            self.body.reserve(h.body_size as usize);
                            self.hdr = Some(h);
                        }
                    }
                }
                Some(h) => {
                    let want = h.body_size as usize - self.body.len();
                    let n = want.min(buf.len());
                    if n == 0 {
                        return Ok(());
                    }
                    self.body.extend_from_slice(&buf[..n]);
                    buf = &buf[n..];
                    if self.body.len() == h.body_size as usize {
                        self.hdr = None;
                        on_message(&h, &self.body)?;
                    }
                }
            }
        }
    }
}

/// The loudest thing to arrive since the loop last looked, and the gain the
/// server said it had already applied to it.
///
/// The two travel together because a step is only meaningful relative to the
/// figure that produced the peak. See [`maintain_digital_gain`].
#[derive(Debug, Clone, Copy)]
struct Peak {
    /// Fraction of full scale: one is a rail.
    fraction: f32,
    /// The whole dB this message's own header claimed, which on a link with
    /// anything in flight is not always what has most recently been asked for.
    gain_db: f64,
}

/// The far end, and everything this end believes about it.
struct Client {
    sock: TcpStream,
    endpoint: String,
    info: DeviceInfo,
    /// Whether this is the narrow-I/Q interface. Decides the rate ladder and
    /// nothing else — the FFT lane is available to both.
    vfo: bool,
    framer: Framer,

    iq_format: SpyServerFormat,
    iq_stage: u32,
    iq_rate: f64,
    /// Where this client has asked its I/Q window to sit.
    center: f64,

    /// From the last `CLIENT_SYNC`: what the server says about itself.
    can_control: bool,
    device_center: f64,
    gain_index: u32,

    auto_digital_gain: bool,
    /// The operator's own figure, in dB, for when automatic is off.
    digital_gain_db: f64,
    /// The digital gain the loop has settled on, in dB. Only an 8-bit stream
    /// in automatic runs the loop — see [`Client::loop_runs`] — and this is
    /// where it starts from: the reference formula's figure.
    servo_db: f64,
    /// The loudest sample since the loop last looked — a whole
    /// [`GAIN_MIN_INTERVAL`] of them, not just the read that ended it — with
    /// the gain its own message was sent under. `None` is nothing having
    /// arrived, which is not the same thing as a `fraction` of 0.0 — that is a
    /// stream of mid-scale having arrived, and it is the strongest possible
    /// case for more gain.
    peak: Option<Peak>,
    /// When a digital gain last went out, whoever sent it. The loop waits on
    /// this so it spends no more of the server's settings budget than
    /// [`GAIN_MIN_INTERVAL`] allows.
    last_gain_move: Instant,
    /// Since when the peak has been continuously below target. Raising the
    /// gain waits on this so a pause in the traffic does not pump it up.
    quiet_since: Option<Instant>,

    fft_enabled: bool,
    fft_stage: u32,
    fft_span: f64,
    fft_center: f64,
    /// The window the FFT bytes are quantised into — as *sent*, already
    /// clamped, so the decoder and the encoder cannot disagree.
    fft_db_offset: f64,
    fft_db_range: f64,

    /// A centre asked for but not yet sent, and when the last one went.
    /// Deferred rather than dropped: the last position of a drag must always
    /// arrive, or the receiver stops wherever the rate limiter happened to
    /// bite.
    deferred_center: Option<f64>,
    last_retune: Instant,
    last_fft_retune: Instant,
    last_ping: Instant,
    /// The last I/Q sequence number seen, for spotting messages the server
    /// skipped. See [`Client::note_sequence`].
    last_iq_seq: Option<u32>,
}

impl Client {
    fn connect(cfg: &SpyServerConfig, center_hz: f64, vfo: bool) -> Result<Client> {
        let endpoint = cfg.endpoint();
        let sock = dial(&endpoint)?;
        // Commands are sixteen bytes and are almost all this end sends; letting
        // Nagle sit on them would add a round trip to every retune.
        let _ = sock.set_nodelay(true);
        sock.set_read_timeout(Some(HANDSHAKE_TIMEOUT))
            .map_err(|e| Error::Net(format!("cannot set a timeout on {endpoint}: {e}")))?;

        let mut client = Client {
            sock,
            endpoint,
            info: DeviceInfo::default(),
            vfo,
            framer: Framer::default(),
            iq_format: cfg.iq_format,
            iq_stage: 0,
            iq_rate: 0.0,
            center: center_hz,
            // Until a CLIENT_SYNC says otherwise, assume this client owns the
            // receiver. That is the common case, and the sync arrives moments
            // later to correct it if not — whereas assuming the opposite would
            // refuse the operator's first tune on every ordinary server.
            can_control: true,
            device_center: center_hz,
            gain_index: cfg.gain_index,
            auto_digital_gain: cfg.auto_digital_gain,
            digital_gain_db: cfg.digital_gain_db,
            // Overwritten in `plan`, once the stage is known.
            servo_db: 0.0,
            peak: None,
            last_gain_move: Instant::now(),
            quiet_since: None,
            fft_enabled: cfg.fft_enabled,
            fft_stage: cfg.fft_decimation,
            fft_span: 0.0,
            fft_center: center_hz,
            fft_db_offset: 0.0,
            fft_db_range: 0.0,
            deferred_center: None,
            last_retune: Instant::now(),
            last_fft_retune: Instant::now(),
            last_ping: Instant::now(),
            last_iq_seq: None,
        };
        (client.fft_db_offset, client.fft_db_range) =
            proto::clamp_fft_window(cfg.fft_db_offset, cfg.fft_db_range);

        client.handshake(HANDSHAKE_TIMEOUT)?;
        client.plan(cfg);
        client.configure()?;

        client
            .sock
            .set_read_timeout(Some(READ_TIMEOUT))
            .map_err(|e| Error::Net(format!("cannot set a timeout on {}: {e}", client.endpoint)))?;
        Ok(client)
    }

    /// Say hello and wait for the server to describe itself.
    fn handshake(&mut self, timeout: Duration) -> Result<()> {
        let hello = proto::frame_hello(CLIENT_NAME);
        self.sock.write_all(&hello).map_err(|e| {
            Error::Net(format!(
                "{} accepted the connection but refused the handshake: {e}",
                self.endpoint
            ))
        })?;

        let deadline = Instant::now() + timeout;
        let mut info: Option<DeviceInfo> = None;
        let mut sync: Option<ClientSync> = None;
        let mut buf = vec![0u8; READ_BYTES];

        // Both messages normally arrive together, but only the device
        // information is worth waiting the whole timeout for: without it there
        // is no rate ladder, no tuning range and nothing to configure. A sync
        // that has not turned up by then leaves the optimistic defaults above,
        // which the first one to arrive corrects.
        while Instant::now() < deadline && (info.is_none() || sync.is_none()) {
            let read = self.sock.read(&mut buf);
            let n = match read {
                Ok(0) => {
                    return Err(Error::Net(format!(
                        "{} closed the connection during the handshake — some servers do that \
                         when they are full, or when the client name is not one they accept",
                        self.endpoint
                    )));
                }
                Ok(n) => n,
                Err(e) if would_block(&e) => break,
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    return Err(Error::Net(format!(
                        "lost the connection to {} during the handshake: {e}",
                        self.endpoint
                    )));
                }
            };

            let mut framer = std::mem::take(&mut self.framer);
            let r = framer.feed(&buf[..n], &mut |h, body| {
                match h.kind {
                    proto::MSG_DEVICE_INFO => info = Some(DeviceInfo::parse(body)?),
                    proto::MSG_CLIENT_SYNC => sync = Some(ClientSync::parse(body)?),
                    // Anything else this early is a stream the server started
                    // before we asked it to; harmless, and dropped.
                    _ => {}
                }
                Ok(())
            });
            self.framer = framer;
            r?;
        }

        let Some(info) = info else {
            return Err(Error::Net(format!(
                "{} did not send device information within {:.0} s — it accepted the \
                 connection, so it is listening, but it is not answering as a SpyServer",
                self.endpoint,
                timeout.as_secs_f64(),
            )));
        };
        if info.kind() == proto::DeviceKind::Invalid {
            return Err(Error::Access(format!(
                "{} is running but has no receiver attached — it reports device type 0",
                self.endpoint
            )));
        }
        if info.maximum_sample_rate == 0 {
            return Err(Error::Proto(format!(
                "{} reports a maximum sample rate of zero, so there is no rate to ask it for",
                self.endpoint
            )));
        }
        self.info = info;

        if let Some(s) = sync {
            self.adopt_sync(&s);
        }
        // A server that forces a format overrides what was configured. Refusing
        // one this client cannot decode here — rather than requesting 8-bit and
        // reading whatever comes back as if it were 8-bit — is the difference
        // between an error message and a waterfall full of noise.
        if let Some(f) = proto::forced_format(&self.info)?
            && f != self.iq_format
        {
            tracing::info!(
                "SpyServer {}: the server requires {} I/Q, overriding the configured {}",
                self.endpoint,
                f.label(),
                self.iq_format.label(),
            );
            self.iq_format = f;
        }
        Ok(())
    }

    /// Work out the rates and spans from what the server said.
    fn plan(&mut self, cfg: &SpyServerConfig) {
        self.iq_stage = resolve_stage(cfg, &self.info, self.vfo);
        self.iq_rate =
            f64::from(self.info.maximum_sample_rate) / f64::from(1u32 << self.iq_stage.min(31));

        let stages = self.info.fft_stages();
        // A stage this server does not have falls back to its widest, which is
        // the useful end for a band view.
        let (stage, _, span) = stages
            .iter()
            .find(|&&(i, _, _)| i == cfg.fft_decimation)
            .copied()
            .or_else(|| stages.first().copied())
            .unwrap_or((0, 0.0, 0.0));
        self.fft_stage = stage;
        self.fft_span = span;
        if self.fft_span <= 0.0 {
            self.fft_enabled = false;
        }
        // The FFT starts where the dial is, clamped into whatever room the
        // window has beside the device centre.
        self.fft_center = self.clamped_fft_center(self.center);
        self.reset_servo();
    }

    /// Send the whole configuration, in the order the protocol wants it.
    fn configure(&mut self) -> Result<()> {
        tracing::info!(
            "SpyServer {}: {} — {} I/Q at {:.3} ksps (stage {}, {:.2} Mbit/s on the link){}",
            self.endpoint,
            self.info.describe(),
            self.iq_format.label(),
            self.iq_rate / 1e3,
            self.iq_stage,
            self.iq_rate * self.iq_format.bytes_per_sample() as f64 * 8.0 / 1e6,
            if self.fft_enabled {
                format!(", plus a {:.3} MHz FFT for the full-band strip", self.fft_span / 1e6)
            } else {
                String::new()
            },
        );

        self.set(proto::SETTING_IQ_FORMAT, proto::format_wire(self.iq_format))?;
        self.set(proto::SETTING_IQ_DECIMATION, self.iq_stage)?;
        // The FFT window before the I/Q position, because the receiver follows
        // the window and `IQ_FREQUENCY` is only a place inside the band the
        // receiver is on — see [`flush_retune`].
        if self.fft_enabled {
            self.send_fft_config()?;
        }
        self.set(proto::SETTING_IQ_FREQUENCY, proto::freq_wire(self.center))?;
        self.start_stream()
    }

    /// The FFT lane's own settings. Separate from [`Self::start_stream`]
    /// because they survive a stream restart and only need resending when one
    /// of them changes.
    fn send_fft_config(&mut self) -> Result<()> {
        // Always 8-bit. The protocol's other FFT format is a 4-bit
        // differential coding that is documented nowhere and implemented by no
        // open client, so asking for it would mean receiving frames this end
        // cannot read.
        self.set(proto::SETTING_FFT_FORMAT, proto::FORMAT_UINT8)?;
        self.set(proto::SETTING_FFT_DECIMATION, self.fft_stage)?;
        self.set(proto::SETTING_FFT_FREQUENCY, proto::freq_wire(self.fft_center))?;
        self.set(
            proto::SETTING_FFT_DISPLAY_PIXELS,
            FFT_DISPLAY_PIXELS
                .clamp(SpyServerConfig::DISPLAY_PIXELS_MIN, SpyServerConfig::DISPLAY_PIXELS_MAX),
        )?;
        self.set(proto::SETTING_FFT_DB_OFFSET, self.fft_db_offset as i32 as u32)?;
        self.set(proto::SETTING_FFT_DB_RANGE, self.fft_db_range as u32)?;
        Ok(())
    }

    /// Set the streaming mode and the gains that depend on it, then turn the
    /// stream on.
    ///
    /// The order is the protocol's and is not negotiable: the streaming mode
    /// has to precede the gain, because a server may reset its gain state when
    /// the mode changes, and the digital gain goes last because it is computed
    /// from the gain index that was just sent.
    fn start_stream(&mut self) -> Result<()> {
        let mode =
            if self.fft_enabled { proto::STREAM_MODE_FFT_IQ } else { proto::STREAM_MODE_IQ_ONLY };
        self.set(proto::SETTING_STREAMING_MODE, mode)?;
        self.set(proto::SETTING_GAIN, self.gain_index)?;
        self.send_digital_gain()?;
        self.set(proto::SETTING_STREAMING_ENABLED, 1)
    }

    /// The digital gain to ask for: what the loop has settled on for an 8-bit
    /// stream, the reference formula for anything wider, or whatever the
    /// operator pinned it to.
    fn digital_gain_wire(&self) -> u32 {
        let db = if !self.auto_digital_gain {
            self.digital_gain_db
        } else if self.loop_runs() {
            self.servo_db
        } else {
            self.info.digital_gain_db(self.gain_index, self.iq_stage)
        };
        db.round().clamp(0.0, GAIN_MAX_DB) as u32
    }

    /// Whether the closed loop is the authority on the digital gain: only an
    /// 8-bit stream in automatic. Everything wider has 48 dB of room it will
    /// never need, and moving its gain would spend the server's settings
    /// budget for nothing in the samples. The operator's manual figure is
    /// never touched — automatic off means a number was typed.
    fn loop_runs(&self) -> bool {
        self.auto_digital_gain && self.iq_format == SpyServerFormat::Uint8
    }

    /// Send the digital gain, and note when, so the loop does not judge
    /// samples the new figure has not reached yet.
    fn send_digital_gain(&mut self) -> Result<()> {
        self.last_gain_move = Instant::now();
        self.set(proto::SETTING_IQ_DIGITAL_GAIN, self.digital_gain_wire())
    }

    /// Put the loop back at its starting point and forget its evidence. For
    /// wherever its basis moves: a fresh connection, the gain index on an R2,
    /// automatic being switched on.
    fn reset_servo(&mut self) {
        self.servo_db = self.info.digital_gain_db(self.gain_index, self.iq_stage);
        self.peak = None;
        self.quiet_since = None;
    }

    fn set(&mut self, setting: u32, value: u32) -> Result<()> {
        self.sock.write_all(&proto::frame_setting(setting, value)).map_err(|e| {
            Error::Net(format!("lost the connection to {} while sending: {e}", self.endpoint))
        })
    }

    fn ping(&mut self) -> Result<()> {
        self.sock.write_all(&proto::frame_ping()).map_err(|e| {
            Error::Net(format!("lost the connection to {} while sending: {e}", self.endpoint))
        })
    }

    /// Take the server's statement of where things are.
    ///
    /// The gain is adopted **only on a server this client does not control**,
    /// and the asymmetry matters. A sync arrives before the opening settings
    /// are sent, carrying whatever gain the server happens to be sitting at;
    /// adopting it unconditionally would overwrite the operator's configured
    /// gain with a stale one and then send *that* — silently, and taking the
    /// computed digital gain with it, since that is a function of the index.
    /// Where another client owns the receiver the opposite is true: the gain
    /// is theirs, we never send one, and what they set is the only true
    /// answer.
    fn adopt_sync(&mut self, s: &ClientSync) {
        self.can_control = s.can_control != 0;
        self.device_center = f64::from(s.device_center_hz);
        if !self.can_control {
            self.gain_index = s.gain;
        }
    }

    /// Where the I/Q window may actually sit, given who owns the receiver.
    fn reachable_center(&self, want: f64) -> f64 {
        let (lo, hi) = if self.can_control {
            (f64::from(self.info.minimum_frequency), f64::from(self.info.maximum_frequency))
        } else {
            let slack = self.info.iq_slack_hz(self.iq_rate);
            (self.device_center - slack, self.device_center + slack)
        };
        want.clamp(lo, hi.max(lo))
    }

    /// Where the FFT window should be centred for the dial to be on `hz`.
    ///
    /// Where this client controls the receiver that is `hz` itself: with the
    /// FFT lane running the receiver follows this window, so the window goes
    /// where the dial goes. Clamping it into the slack around wherever the
    /// server happened to be sitting is what parked whole sessions on the
    /// server's own start-up frequency — at decimation stage 0 the window
    /// covers the entire receiver and has no slack at all, so the clamp
    /// collapsed to exactly that frequency and never let go.
    ///
    /// Where another client owns the receiver the device is theirs and does not
    /// move, and all this end may do is slide the window inside it.
    fn clamped_fft_center(&self, hz: f64) -> f64 {
        if self.can_control {
            return hz;
        }
        let slack = self.info.fft_slack_hz(self.fft_span);
        hz.clamp(self.device_center - slack, self.device_center + slack)
    }

    /// Where the FFT window should sit, or `None` to leave it where it is.
    /// The hysteresis itself lives in
    /// [`proto::DeviceInfo::fft_recenter`], where it can be tested without a
    /// socket.
    fn fft_target_center(&self) -> Option<f64> {
        if !self.fft_enabled {
            return None;
        }
        self.info.fft_recenter(
            self.center,
            self.fft_center,
            self.fft_span,
            self.device_center,
            self.can_control,
            self.info.iq_slack_hz(self.iq_rate),
        )
    }

    /// Note an I/Q message's sequence number, reporting messages the server
    /// skipped because this client fell behind.
    ///
    /// Only when the FFT lane is off. The protocol does not say whether the
    /// sequence counts per stream or across all of them, and if it is the
    /// latter then every FFT frame would read as a gap in the I/Q — a steady
    /// stream of warnings about a link that is working. With one stream
    /// running there is nothing to confound it.
    fn note_sequence(&mut self, seq: u32, stats: &mut RxStats) {
        if self.fft_enabled {
            return;
        }
        if let Some(prev) = self.last_iq_seq {
            let gap = seq.wrapping_sub(prev).wrapping_sub(1);
            // A large jump is a counter reset, not a million lost messages.
            if gap > 0 && gap < 1000 {
                stats.on_gap(u64::from(gap));
            }
        }
        self.last_iq_seq = Some(seq);
    }

    /// The one-line name the radio tab and the logs show.
    ///
    /// The link cost is in it on purpose: it is the number that decides
    /// whether a remote receiver works at all, it is not visible anywhere
    /// else, and an operator watching a stream restart every few seconds has
    /// no other way to see that the rate they picked does not fit down their
    /// uplink.
    fn describe(&self) -> String {
        format!(
            "SpyServer {} — {}, {:.3} ksps {}, {:.1} Mbit/s",
            self.endpoint,
            self.info.kind().name(),
            self.iq_rate / 1e3,
            self.iq_format.label(),
            self.iq_rate * self.iq_format.bytes_per_sample() as f64 * 8.0 / 1e6,
        )
    }

    /// Ask the server to stop before dropping the socket, so it can free the
    /// receiver for the next client rather than waiting for a timeout.
    fn shutdown(&mut self) {
        let _ = self.set(proto::SETTING_STREAMING_ENABLED, 0);
        let _ = self.sock.shutdown(std::net::Shutdown::Both);
    }
}

/// Resolve and connect, with a timeout on each candidate address.
fn dial(endpoint: &str) -> Result<TcpStream> {
    let addrs: Vec<_> = endpoint
        .to_socket_addrs()
        .map_err(|e| {
            Error::Net(format!(
                "cannot resolve {endpoint}: {e} — the address is host:port, and the port \
                 defaults to {}",
                SpyServerConfig::DEFAULT_PORT
            ))
        })?
        .collect();
    let mut last = None;
    for addr in &addrs {
        match TcpStream::connect_timeout(addr, CONNECT_TIMEOUT) {
            Ok(s) => return Ok(s),
            Err(e) => last = Some(e),
        }
    }
    Err(Error::Net(match last {
        Some(e) => format!(
            "cannot reach the SpyServer at {endpoint}: {e} — check that spyserver is running \
             there, that its config binds an address other clients can reach, and that nothing \
             is filtering the port"
        ),
        None => format!("{endpoint} resolved to no addresses"),
    }))
}

/// A read that hit [`READ_TIMEOUT`] with nothing to show for it.
///
/// Two kinds, and both have to be caught: a socket read timeout surfaces as
/// `WouldBlock` on Unix and as `TimedOut` on Windows. Missing one turns every
/// quiet moment on that platform into a dropped connection.
fn would_block(e: &std::io::Error) -> bool {
    matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut)
}

/// Apply coalesced control changes.
fn apply(client: &mut Client, p: &Pending) -> Result<()> {
    if let Some(hz) = p.center {
        // Deferred, not sent: the far end takes about eight of these a second
        // and a drag produces hundreds.
        client.deferred_center = Some(hz);
    }
    if let Some(v) = p.fft_db_offset {
        let (off, range) = proto::clamp_fft_window(v, client.fft_db_range);
        client.fft_db_offset = off;
        client.fft_db_range = range;
        if client.fft_enabled {
            client.set(proto::SETTING_FFT_DB_OFFSET, off as i32 as u32)?;
        }
    }
    if let Some(v) = p.fft_db_range {
        let (off, range) = proto::clamp_fft_window(client.fft_db_offset, v);
        client.fft_db_offset = off;
        client.fft_db_range = range;
        if client.fft_enabled {
            client.set(proto::SETTING_FFT_DB_RANGE, range as u32)?;
        }
    }
    if let Some(stage) = p.fft_decimation {
        let stages = client.info.fft_stages();
        if let Some(&(i, _, span)) = stages.iter().find(|&&(i, _, _)| i == stage) {
            client.fft_stage = i;
            client.fft_span = span;
            client.fft_center = client.clamped_fft_center(client.center);
            if client.fft_enabled {
                client.set(proto::SETTING_FFT_DECIMATION, i)?;
                client.set(proto::SETTING_FFT_FREQUENCY, proto::freq_wire(client.fft_center))?;
            }
        }
    }
    if let Some(on) = p.fft_enabled
        && on != client.fft_enabled
        && client.fft_span > 0.0
    {
        client.fft_enabled = on;
        // The streaming mode is changing, so the whole tail of the opening
        // sequence has to be restated: a server may reset its gain state when
        // the mode changes, and the gain must follow the mode rather than
        // precede it. Costs a short gap in the I/Q, which is what a checkbox
        // that changes what the server sends is worth.
        client.set(proto::SETTING_STREAMING_ENABLED, 0)?;
        if on {
            // Where the window is *now*, not where it was left when the lane
            // was last on: nothing has been maintaining it while it was off, so
            // sending the stale centre would drag the receiver back to whatever
            // band the operator was in then.
            client.fft_center = client.clamped_fft_center(client.center);
            client.send_fft_config()?;
            client.set(proto::SETTING_IQ_FREQUENCY, proto::freq_wire(client.center))?;
        }
        client.start_stream()?;
    }
    if p.gain.is_some() || p.auto_digital_gain.is_some() || p.digital_gain.is_some() {
        if let Some(g) = p.gain {
            client.gain_index = g.min(client.info.maximum_gain_index);
            client.set(proto::SETTING_GAIN, client.gain_index)?;
        }
        if let Some(v) = p.auto_digital_gain {
            client.auto_digital_gain = v;
        }
        if let Some(v) = p.digital_gain {
            client.digital_gain_db = v;
        }
        // The loop's starting point moves with the gain index on an R2, and a
        // loop that has just been switched on has no evidence yet.
        client.reset_servo();
        // Always resent alongside the gain index, never on its own. On an
        // Airspy R2 the computed digital gain is a function of the gain index,
        // so sending one without the other leaves the level wrong by as much
        // as the whole gain range.
        client.send_digital_gain()?;
    }
    Ok(())
}

/// Send a deferred retune, once the far end has had long enough since the last.
fn flush_retune(client: &mut Client, shared: &Arc<Shared>) -> Result<()> {
    let Some(want) = client.deferred_center else {
        return Ok(());
    };
    if client.last_retune.elapsed() < RETUNE_MIN_INTERVAL {
        return Ok(());
    }
    client.deferred_center = None;
    client.last_retune = Instant::now();

    let sent = client.reachable_center(want);
    client.center = sent;
    shared.iq_center_milli_hz.store((sent * 1000.0) as i64, Ordering::Relaxed);

    // The window moves first, and that ordering is load-bearing. On a server
    // this client controls, the receiver follows the FFT window; `IQ_FREQUENCY`
    // only places the I/Q window inside the band the receiver is already on, and
    // a position outside it is clamped — and *stays* clamped, because the server
    // holds it as an absolute frequency rather than as an offset. Placing the
    // I/Q window before the receiver has moved therefore leaves it on the band
    // being left behind.
    move_fft_window(client)?;
    client.set(proto::SETTING_IQ_FREQUENCY, proto::freq_wire(sent))
}

/// Move the FFT window if the dial has reached its edge, or if the device
/// moved out from under it.
///
/// Runs every pass rather than only after a retune: on a shared server the
/// owner can move the receiver with no dial move here at all, and the window's
/// slack is measured against where they put it.
fn maintain_fft(client: &mut Client) -> Result<()> {
    if client.last_fft_retune.elapsed() < RETUNE_MIN_INTERVAL {
        return Ok(());
    }
    move_fft_window(client)
}

/// The move itself, without the rate limit.
///
/// [`flush_retune`] calls this directly: it has already paid the interval for
/// the retune it is in the middle of, and on a server this client controls the
/// window *is* the retune — a window held back by its own limiter would be a
/// receiver held back with it.
fn move_fft_window(client: &mut Client) -> Result<()> {
    let Some(target) = client.fft_target_center() else {
        return Ok(());
    };
    if (target - client.fft_center).abs() < 1.0 {
        return Ok(());
    }
    client.fft_center = target;
    client.last_fft_retune = Instant::now();
    client.set(proto::SETTING_FFT_FREQUENCY, proto::freq_wire(target))
}

/// Walk the digital gain towards a peak that fits, on the evidence of what has
/// arrived since the last look.
///
/// Only an 8-bit stream in automatic — see [`Client::loop_runs`]. Nothing the
/// loop does is visible in the level: the decoder divides by the gain the
/// *header* states, so a stream whose gain changed mid-flight comes out at the
/// same amplitude either side of the change. What moves is how many of the
/// eight bits the signal is using, and that is the whole point. How much gain
/// a band needs is a property of the band — a dead 20 m and 80 m at night are
/// thirty decibels apart — so no fixed figure serves both, and the one that was
/// tried clipped every band that had a signal on it.
///
/// Asymmetric on purpose. Down is immediate and as far as the peak says: a
/// clipped stream is carrying nothing, and every message spent on it is lost
/// audio. A rail hides how far past full scale it is, so a clipped stream comes
/// down a step at a time — 6 dB, what would put a rail on target — and the next
/// step sees an honest peak the moment the rail lets go. Up is slow and small:
/// only after the peak has sat below target for [`GAIN_QUIET_BEFORE_RAISE`],
/// and by no more than [`GAIN_STEP_UP_DB`], so a pause in the traffic is not
/// mistaken for room and the next syllable does not land on a rail.
///
/// Every step is measured from the gain the samples' *own header* states, so
/// what the loop does is independent of how long a setting takes to reach the
/// stream. A rail that was sent under a figure already superseded resolves to
/// that same figure and moves nothing, and the loop simply waits for samples
/// the last change actually reached.
fn maintain_digital_gain(client: &mut Client) -> Result<()> {
    if !client.loop_runs() {
        return Ok(());
    }
    if client.last_gain_move.elapsed() < GAIN_MIN_INTERVAL {
        // Too soon to spend a setting, so do not spend the evidence either.
        // Left where it is, the peak goes on accumulating and the next look
        // judges the whole interval rather than whatever happened to arrive in
        // the last few milliseconds of it.
        return Ok(());
    }
    // Nothing arrived is not an argument for anything. A stream of mid-scale
    // is: that is `Some(0.0)`, and the strongest case there is for more gain.
    let Some(peak) = client.peak.take() else {
        return Ok(());
    };

    // The move that would put this peak on target. Positive is room to spare;
    // infinite when nothing in the stream cleared mid-scale, which the step
    // limit below makes finite.
    let want = 20.0 * f64::from(GAIN_TARGET_PEAK / peak.fraction).log10();
    if want > 0.0 {
        // Room to spare. Wait for it to be a settled fact rather than a gap in
        // the traffic, and then take it a step at a time.
        let quiet = *client.quiet_since.get_or_insert_with(Instant::now);
        if quiet.elapsed() < GAIN_QUIET_BEFORE_RAISE || want < GAIN_DEADBAND_DB {
            return Ok(());
        }
    } else {
        // A loud moment anywhere in the interval is in the accumulated peak,
        // so this still cancels a pending raise even though the loop now only
        // looks at the end of one.
        client.quiet_since = None;
        if peak.fraction < GAIN_CLIP_PEAK && -want < GAIN_DEADBAND_DB {
            return Ok(());
        }
    }
    let step = want.min(GAIN_STEP_UP_DB);
    // Measured from the figure the samples were *sent* under, not from the one
    // most recently asked for. On a link with a message or two in flight the
    // two differ for as long as the flight takes, and stepping from the latter
    // takes the same correction twice — down towards a floor the band never
    // needed, on exactly the slow links an 8-bit stream is chosen for.
    let held = client.servo_db.round();
    let ideal = (peak.gain_db + step).round().clamp(0.0, GAIN_MAX_DB);
    // A peak may never move the gain the way it did not argue for. A rail sent
    // under a figure already left behind resolves to that figure, which is an
    // argument for waiting rather than for climbing back onto the rail.
    let next = if want > 0.0 { ideal.max(held) } else { ideal.min(held) };
    if next == held {
        // Against a stop, or already acted on. Saying so again costs a setting.
        return Ok(());
    }
    tracing::debug!(
        "SpyServer {}: 8-bit peak at {:.0} % of full scale under {:.0} dB, \
         digital gain {:.0} -> {:.0} dB",
        client.endpoint,
        peak.fraction * 100.0,
        peak.gain_db,
        client.servo_db,
        next,
    );
    client.servo_db = next;
    client.quiet_since = None;
    client.send_digital_gain()
}

/// Everything one completed message does.
#[allow(clippy::too_many_arguments)]
fn on_message(
    client: &mut Client,
    h: &MessageHeader,
    body: &[u8],
    rx: &mut Producer<f32>,
    shared: &Arc<Shared>,
    stats: &mut RxStats,
    iq_scratch: &mut Vec<f32>,
    fft_scratch: &mut Vec<f32>,
    started: Instant,
) -> Result<()> {
    match h.kind {
        proto::MSG_UINT8_IQ | proto::MSG_INT16_IQ | proto::MSG_FLOAT_IQ => {
            let pairs = iq_to_f32(client.iq_format, body, h.digital_gain(), iq_scratch);
            if pairs > 0 {
                if client.loop_runs() {
                    // The peak as a fraction of full scale, which is what the
                    // loop needs and what the decoder has just divided out:
                    // the samples are `raw / (gain * full scale)`, so putting
                    // the gain back gives the raw fraction, in any format.
                    // The *header's* gain — what the server did — so a message
                    // from before the last change still reports where its own
                    // bytes sat, and carries the figure that put them there.
                    let fraction = iq_scratch.iter().fold(0f32, |m, v| m.max(v.abs()));
                    let seen =
                        Peak { fraction: fraction * h.digital_gain(), gain_db: f64::from(h.flags) };
                    // The loudest one wins, and brings its own gain with it:
                    // pairing a peak with a figure some other message was sent
                    // under is the whole mistake being avoided here.
                    client.peak = Some(match client.peak {
                        Some(p) if p.fraction >= seen.fraction => p,
                        _ => seen,
                    });
                }
                client.note_sequence(h.sequence, stats);
                stats.on_iq(pairs);
                push_iq(rx, iq_scratch, stats, shared.rx_paused.load(Ordering::Relaxed));
                shared.last_rx_ms.store(started.elapsed().as_millis() as u64, Ordering::Relaxed);
            }
        }
        proto::MSG_INT24_IQ => {
            return Err(Error::Unsupported(format!(
                "the SpyServer at {} is sending 24-bit I/Q, which this client does not decode",
                client.endpoint
            )));
        }
        proto::MSG_UINT8_FFT => {
            fft_to_db(body, client.fft_db_offset, client.fft_db_range, fft_scratch);
            if !fft_scratch.is_empty() {
                let frame = FftFrame {
                    center_hz: client.fft_center,
                    span_hz: client.fft_span,
                    bins: std::mem::take(fft_scratch),
                };
                // One swap under the lock, nothing decoded inside it. An unread
                // frame is overwritten on purpose — see `Shared::fft`.
                if let Ok(mut slot) = shared.fft.lock() {
                    *slot = Some(frame);
                }
            }
        }
        proto::MSG_DINT4_FFT => {
            // Never requested, so a server sending one is doing something this
            // client did not ask for. Ignored rather than fatal: the I/Q is
            // still good, and a band view is not worth dropping a receiver for.
            static SAID: std::sync::Once = std::sync::Once::new();
            SAID.call_once(|| {
                tracing::warn!(
                    "SpyServer: 4-bit FFT frames are arriving although 8-bit was requested; \
                     the encoding is undocumented, so the full-band strip stays empty"
                );
            });
        }
        proto::MSG_DEVICE_INFO => {
            // A server may restate this; nothing here changes mid-stream except
            // by reconnecting, so it is only worth noticing when it disagrees.
            if let Ok(info) = DeviceInfo::parse(body)
                && info.maximum_sample_rate != client.info.maximum_sample_rate
            {
                return Err(Error::Proto(format!(
                    "the SpyServer at {} changed receiver underneath us",
                    client.endpoint
                )));
            }
        }
        proto::MSG_CLIENT_SYNC => {
            let s = ClientSync::parse(body)?;
            let moved = (f64::from(s.device_center_hz) - client.device_center).abs() > 0.5;
            client.adopt_sync(&s);
            shared.can_control.store(client.can_control, Ordering::Relaxed);
            shared
                .device_center_milli_hz
                .store((client.device_center * 1000.0) as i64, Ordering::Relaxed);
            shared.gain_index.store(client.gain_index, Ordering::Relaxed);
            if moved {
                // Only bumped when the device actually moved. The counter is
                // what the source watches to decide whether to tell the engine
                // its span is somewhere else now, and an unchanged restatement
                // is not news.
                shared.sync_seq.fetch_add(1, Ordering::Relaxed);
            }
        }
        proto::MSG_PONG => {}
        _ => {}
    }
    Ok(())
}

fn pump(
    client: &mut Client,
    ctrl: &crossbeam_channel::Receiver<Ctrl>,
    rx: &mut Producer<f32>,
    shared: &Arc<Shared>,
) -> Result<()> {
    let mut stats = RxStats::network(client.iq_rate);
    let started = Instant::now();
    let mut buf = vec![0u8; READ_BYTES];
    let mut iq_scratch: Vec<f32> = Vec::with_capacity(READ_BYTES);
    let mut fft_scratch: Vec<f32> = Vec::with_capacity(FFT_DISPLAY_PIXELS as usize);
    let mut framer = std::mem::take(&mut client.framer);

    loop {
        let mut pending = Pending::default();
        while let Ok(c) = ctrl.try_recv() {
            pending.absorb(c);
        }
        if pending.shutdown {
            break;
        }
        if !pending.is_empty() {
            // A failed write means the link is gone, which is a reason to stop.
            apply(client, &pending)?;
        }
        flush_retune(client, shared)?;
        maintain_fft(client)?;
        maintain_digital_gain(client)?;
        if client.last_ping.elapsed() >= PING_INTERVAL {
            client.last_ping = Instant::now();
            client.ping()?;
        }

        let read = client.sock.read(&mut buf);
        match read {
            Ok(0) => {
                return Err(Error::Net(format!(
                    "the SpyServer at {} closed the connection — it does that when a client \
                     stops keeping up, and when its receiver is unplugged",
                    client.endpoint
                )));
            }
            Ok(n) => {
                framer.feed(&buf[..n], &mut |h, body| {
                    on_message(
                        client,
                        h,
                        body,
                        rx,
                        shared,
                        &mut stats,
                        &mut iq_scratch,
                        &mut fft_scratch,
                        started,
                    )
                })?;
                // `on_message` takes the scratch when it hands a frame over, so
                // give it somewhere to build the next one.
                if fft_scratch.capacity() == 0 {
                    fft_scratch = Vec::with_capacity(FFT_DISPLAY_PIXELS as usize);
                }
            }
            Err(e) if would_block(&e) => {}
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => {
                return Err(Error::Net(format!("lost the connection to {}: {e}", client.endpoint)));
            }
        }

        stats.tick();
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(kind: u16, flags: u16, body: usize) -> Vec<u8> {
        let mut b = vec![0u8; proto::MSG_HEADER_LEN];
        b[0..4].copy_from_slice(&proto::PROTOCOL_VERSION.to_le_bytes());
        b[4..8].copy_from_slice(&(u32::from(kind) | (u32::from(flags) << 16)).to_le_bytes());
        b[16..20].copy_from_slice(&(body as u32).to_le_bytes());
        b
    }

    /// Collect what a framer produces from a stream fed in arbitrary chunks.
    fn framed(chunks: &[&[u8]]) -> Vec<(u16, Vec<u8>)> {
        let mut f = Framer::default();
        let mut out = Vec::new();
        for c in chunks {
            f.feed(c, &mut |h, body| {
                out.push((h.kind, body.to_vec()));
                Ok(())
            })
            .expect("well-formed");
        }
        out
    }

    #[test]
    fn a_header_split_across_reads_is_reassembled() {
        let mut msg = header(proto::MSG_UINT8_IQ, 0, 4);
        msg.extend_from_slice(&[1, 2, 3, 4]);
        // Split inside the header, then inside the body.
        let got = framed(&[&msg[..7], &msg[7..13], &msg[13..22], &msg[22..]]);
        assert_eq!(got, vec![(proto::MSG_UINT8_IQ, vec![1, 2, 3, 4])]);
    }

    #[test]
    fn two_messages_in_one_read_are_both_delivered() {
        let mut stream = header(proto::MSG_UINT8_FFT, 0, 3);
        stream.extend_from_slice(&[10, 20, 30]);
        stream.extend_from_slice(&header(proto::MSG_UINT8_IQ, 0, 2));
        stream.extend_from_slice(&[7, 8]);
        let got = framed(&[&stream]);
        assert_eq!(
            got,
            vec![(proto::MSG_UINT8_FFT, vec![10, 20, 30]), (proto::MSG_UINT8_IQ, vec![7, 8])]
        );
    }

    /// A PONG has no body at all. If it were left waiting for one it would
    /// wedge the framer until the next message pushed it through.
    #[test]
    fn a_zero_body_message_is_dispatched_at_once() {
        let pong = header(proto::MSG_PONG, 0, 0);
        let got = framed(&[&pong]);
        assert_eq!(got, vec![(proto::MSG_PONG, vec![])]);

        // And it does not swallow what follows it in the same read.
        let mut stream = pong.clone();
        stream.extend_from_slice(&header(proto::MSG_UINT8_IQ, 0, 2));
        stream.extend_from_slice(&[1, 2]);
        assert_eq!(framed(&[&stream]).len(), 2);
    }

    #[test]
    fn an_absurd_body_size_is_refused_rather_than_allocated() {
        let mut f = Framer::default();
        let bad = header(proto::MSG_UINT8_IQ, 0, proto::MAX_MESSAGE_BODY_SIZE as usize + 1);
        let e = f.feed(&bad, &mut |_, _| Ok(())).expect_err("past the limit");
        assert!(e.to_string().contains("out of step"), "{e}");
    }

    /// Feeding nothing must not spin: an empty read happens on every timeout.
    #[test]
    fn an_empty_feed_returns_rather_than_looping() {
        let mut f = Framer::default();
        f.feed(&[], &mut |_, _| Ok(())).expect("no messages");
        // Half a header in, then nothing more.
        let h = header(proto::MSG_PONG, 0, 0);
        f.feed(&h[..5], &mut |_, _| panic!("not a whole header yet")).expect("partial");
        f.feed(&[], &mut |_, _| panic!("still nothing")).expect("empty");
    }

    // --- against a fake server -----------------------------------------------

    /// The `(setting, value)` pairs a fake server has been sent, in order.
    type SettingLog = std::sync::Arc<std::sync::Mutex<Vec<(u32, u32)>>>;

    /// A fake SpyServer that records every setting a client sends it.
    ///
    /// Shaped like the one this was found on: an Airspy HF+ whose analog
    /// bandwidth is its whole FFT stage-0 span — so the FFT window has no slack
    /// of its own — sitting on `device_center_hz`, and which never restates its
    /// `CLIENT_SYNC`. That last part is why nothing self-corrects: the client
    /// only ever hears where the receiver is once.
    fn recording_server(device_center_hz: u32, can_control: u32) -> (String, SettingLog) {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").expect("bind");
        let addr = listener.local_addr().expect("addr").to_string();
        let log = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let seen = std::sync::Arc::clone(&log);

        std::thread::spawn(move || {
            let Ok((mut sock, _)) = listener.accept() else { return };
            let msg = |kind: u16, body: &[u8]| {
                let mut out = Vec::new();
                out.extend_from_slice(&proto::PROTOCOL_VERSION.to_le_bytes());
                out.extend_from_slice(&u32::from(kind).to_le_bytes());
                out.extend_from_slice(&0u32.to_le_bytes());
                out.extend_from_slice(&0u32.to_le_bytes());
                out.extend_from_slice(&(body.len() as u32).to_le_bytes());
                out.extend_from_slice(body);
                out
            };
            let words =
                |ws: &[u32]| -> Vec<u8> { ws.iter().flat_map(|w| w.to_le_bytes()).collect() };

            // Airspy HF+: 768 ksps, 660 kHz of analog bandwidth, 8 stages, no
            // decimation floor, tunable across everything the server offers.
            let info =
                words(&[2, 0x3933_3038, 768_000, 660_000, 8, 0, 0, 0, 1_700_000_000, 16, 0, 0]);
            let sync = words(&[
                can_control,
                29,
                device_center_hz,
                device_center_hz,
                device_center_hz,
                0,
                0,
                0,
                0,
            ]);
            if sock.write_all(&msg(proto::MSG_DEVICE_INFO, &info)).is_err()
                || sock.write_all(&msg(proto::MSG_CLIENT_SYNC, &sync)).is_err()
            {
                return;
            }

            // Everything after the hello is commands; record the settings.
            let mut buf = [0u8; 4096];
            let mut carry: Vec<u8> = Vec::new();
            loop {
                let Ok(n) = sock.read(&mut buf) else { return };
                if n == 0 {
                    return;
                }
                carry.extend_from_slice(&buf[..n]);
                while carry.len() >= proto::CMD_HEADER_LEN {
                    let w = |i: usize| {
                        u32::from_le_bytes([carry[i], carry[i + 1], carry[i + 2], carry[i + 3]])
                    };
                    let (cmd, body_size) = (w(0), w(4) as usize);
                    if carry.len() < proto::CMD_HEADER_LEN + body_size {
                        break;
                    }
                    if cmd == proto::CMD_SET_SETTING && body_size == 8 {
                        seen.lock().expect("log").push((w(8), w(12)));
                    }
                    carry.drain(..proto::CMD_HEADER_LEN + body_size);
                }
            }
        });

        (addr, log)
    }

    /// Every `(setting, value)` recorded so far, waiting up to a second for at
    /// least `at_least` of the given setting to turn up.
    fn settings_sent(
        log: &std::sync::Mutex<Vec<(u32, u32)>>,
        setting: u32,
        at_least: usize,
    ) -> Vec<u32> {
        for _ in 0..200 {
            let values: Vec<u32> = log
                .lock()
                .expect("log")
                .iter()
                .filter(|&&(s, _)| s == setting)
                .map(|&(_, v)| v)
                .collect();
            if values.len() >= at_least {
                return values;
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        log.lock().expect("log").iter().filter(|&&(s, _)| s == setting).map(|&(_, v)| v).collect()
    }

    fn cfg(addr: &str) -> SpyServerConfig {
        SpyServerConfig { address: addr.into(), ..SpyServerConfig::default() }
    }

    /// The FFT window is where the receiver *is*: with the FFT lane running, a
    /// SpyServer tunes to `FFT_FREQUENCY` and treats `IQ_FREQUENCY` as a
    /// position inside the band it is already on. So opening the window on the
    /// frequency the server happened to be sitting on parks the whole session
    /// there, whatever the operator asked for — and nothing recovers, because
    /// this server never says where the receiver went.
    #[test]
    fn the_fft_window_opens_on_the_dial_not_on_the_servers_own_frequency() {
        let (addr, log) = recording_server(100_000_000, 1);
        let handle = SpyServerHandle::connect_wideband(&cfg(&addr), 7_100_000.0).expect("connect");
        let sent = settings_sent(&log, proto::SETTING_FFT_FREQUENCY, 1);
        drop(handle);
        assert_eq!(
            sent.first().copied(),
            Some(7_100_000),
            "the FFT window opened on the server's own frequency, which is where the \
             receiver then stayed: {sent:?}"
        );
    }

    /// ...and it follows the dial afterwards. At the top of the rate ladder the
    /// I/Q window has no slack to slide within, so the receiver has to move,
    /// and moving the receiver means moving this window.
    #[test]
    fn the_fft_window_follows_a_retune() {
        let (addr, log) = recording_server(100_000_000, 1);
        let handle = SpyServerHandle::connect_wideband(&cfg(&addr), 7_100_000.0).expect("connect");
        settings_sent(&log, proto::SETTING_FFT_FREQUENCY, 1);
        handle.set_center_hz(14_100_000.0);
        let sent = settings_sent(&log, proto::SETTING_FFT_FREQUENCY, 2);
        drop(handle);
        assert_eq!(
            sent.last().copied(),
            Some(14_100_000),
            "the receiver follows the FFT window, so a retune that leaves it behind is a \
             retune that never happens: {sent:?}"
        );
    }

    /// The opposite server: another client owns the receiver, so it does not
    /// move for us and the window may only slide inside what they are already
    /// receiving. At stage 0 that is nowhere at all — the window covers the
    /// whole receiver — so it stays on their centre.
    #[test]
    fn a_shared_servers_fft_window_stays_inside_the_owners_band() {
        let (addr, log) = recording_server(100_000_000, 0);
        let handle = SpyServerHandle::connect_wideband(&cfg(&addr), 7_100_000.0).expect("connect");
        let sent = settings_sent(&log, proto::SETTING_FFT_FREQUENCY, 1);
        drop(handle);
        assert_eq!(
            sent.first().copied(),
            Some(100_000_000),
            "the device belongs to another client; this end cannot tune it away: {sent:?}"
        );
    }

    /// The receiver follows the FFT window, and `IQ_FREQUENCY` is only a
    /// position inside the band it is on — a position the server clamps if it
    /// is outside, and then keeps. So the window has to move first, or the I/Q
    /// is placed against a band the receiver is about to leave.
    #[test]
    fn a_retune_moves_the_fft_window_before_it_places_the_iq_window() {
        let (addr, log) = recording_server(100_000_000, 1);
        let handle = SpyServerHandle::connect_wideband(&cfg(&addr), 7_100_000.0).expect("connect");
        settings_sent(&log, proto::SETTING_FFT_FREQUENCY, 1);
        let before = log.lock().expect("log").len();
        handle.set_center_hz(14_100_000.0);
        settings_sent(&log, proto::SETTING_FFT_FREQUENCY, 2);
        let after: Vec<(u32, u32)> = log.lock().expect("log")[before..].to_vec();
        drop(handle);

        let fft = after
            .iter()
            .position(|&(s, v)| s == proto::SETTING_FFT_FREQUENCY && v == 14_100_000)
            .expect("the FFT window moved");
        let iq = after
            .iter()
            .position(|&(s, v)| s == proto::SETTING_IQ_FREQUENCY && v == 14_100_000)
            .expect("the I/Q window moved");
        assert!(fft < iq, "the FFT window must move first: {after:?}");
    }
}
