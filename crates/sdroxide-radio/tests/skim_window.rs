//! Where the skimmers listen on a front end wider than their window.
//!
//! The window is 192 kHz of a stream that may be thirty megahertz wide, and it
//! used to be pinned to the hardware centre. On a receiver whose span is a band
//! that is the same place the operator is looking; on an RX-888 handing over the
//! whole of HF it is the middle of the sampled span and nothing else — so the
//! skimmers ran, cost their CPU, and decoded nothing at all, while the waterfall
//! plainly showed a band full of CW.
//!
//! So the window follows the waterfall. This walks the whole chain: a station is
//! keyed well away from the hardware centre, the client says what it is looking
//! at, and a spot has to come back with the callsign on it — which it can only
//! do if the window was placed on the view, mixed onto it, and left there.

use std::f64::consts::TAU;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sdroxide_radio::{AudioParams, Complex32, EngineConfig, IqSource, Result, rtrb, start_engine};
use sdroxide_types::{
    Command, CwEngine, DeviceCaps, RadioEvent, SkimmerKind, SkimmerSettings, Vfo,
};

/// Wide enough that the window is a slice of the span rather than all of it —
/// the skim chain decimates this by two — and narrow enough to synthesise in
/// real time in a debug build.
const RATE: f64 = 400_000.0;
/// Where the front end is parked. Nothing is transmitted here: it stands in for
/// the middle of an RX-888's half-spectrum, which is where the window used to
/// sit whatever the operator was doing.
const DEV_CENTER: f64 = 14_200_000.0;
/// The station, 140 kHz down the span — inside the 20 m CW segment in every
/// region, because a CW spot outside one is dropped on the way out.
const STATION: f64 = 14_060_000.0;

/// A receiver hearing one keyed station over a noise floor, paced in real time.
///
/// The pacing is not decoration: the skimmer worker takes IQ over a bounded
/// channel and drops what it cannot keep up with, so a source that ran as fast
/// as the engine could consume it would be decoding a clip full of holes.
struct Band {
    center: Arc<Mutex<f64>>,
    /// One pass of the message at [`RATE`], mixed onto its offset from
    /// [`DEV_CENTER`], played round and round.
    clip: Vec<Complex32>,
    pos: usize,
    started: Instant,
    sent: u64,
    seed: u64,
}

impl IqSource for Band {
    fn sample_rate(&self) -> f64 {
        RATE
    }
    fn center_hz(&self) -> f64 {
        *self.center.lock().unwrap()
    }
    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        *self.center.lock().unwrap() = hz;
        Ok(())
    }
    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        // Hand over only as much as the clock says has happened.
        let due = (self.started.elapsed().as_secs_f64() * RATE) as u64;
        if due <= self.sent {
            std::thread::sleep(Duration::from_millis(5));
            return Ok(0);
        }
        let n = buf.len().min((due - self.sent) as usize).min(4096);
        for z in buf[..n].iter_mut() {
            let mut r = || {
                self.seed ^= self.seed << 13;
                self.seed ^= self.seed >> 7;
                self.seed ^= self.seed << 17;
                ((self.seed >> 11) as f64 / (1u64 << 53) as f64) as f32 - 0.5
            };
            *z = self.clip[self.pos] + Complex32::new(r() * 0.04, r() * 0.04);
            self.pos = (self.pos + 1) % self.clip.len();
        }
        self.sent += n as u64;
        Ok(n)
    }
    fn describe(&self) -> String {
        "one keyed station".into()
    }
}

/// A keyed complex carrier `offset_hz` from the centre, carrying `text`.
///
/// The envelope is taken from a real sidetone and re-keyed onto a complex
/// carrier rather than generated at the output rate directly: it is the same
/// recipe the skimmer's own tests use, including the part that matters — the
/// envelope is sampled fast (a cycle of an 8 kHz tone) and interpolated between
/// samples, because holding each step would image the keying either side of the
/// carrier and plant a comb of stations that were never sent.
fn keyed_cw(text: &str, offset_hz: f64, wpm: f32, rate: f64) -> Vec<Complex32> {
    const AUDIO: f64 = 96_000.0;
    const TONE: f64 = 8_000.0;
    let mut tx = sdroxide_dsp::CwTx::new(AUDIO, TONE, wpm);
    tx.push_text(text);
    let mut tone: Vec<f32> = Vec::new();
    while !tx.drained() {
        let mut blk = [0.0f32; 512];
        tx.next_block(&mut blk);
        tone.extend_from_slice(&blk);
    }
    let per_cycle = (AUDIO / TONE) as usize;
    let env: Vec<f32> =
        tone.chunks(per_cycle).map(|c| c.iter().fold(0.0f32, |a, b| a.max(b.abs()))).collect();

    let n = (env.len() as f64 * rate / TONE) as usize;
    let step = TAU * offset_hz / rate;
    (0..n)
        .map(|i| {
            let x = i as f64 * TONE / rate;
            let k = x.floor() as usize;
            let f = (x - k as f64) as f32;
            let (a, b) =
                (env.get(k).copied().unwrap_or(0.0), env.get(k + 1).copied().unwrap_or(0.0));
            let amp = a + (b - a) * f;
            let ph = i as f64 * step;
            Complex32::new(amp * ph.cos() as f32, amp * ph.sin() as f32)
        })
        .collect()
}

fn caps() -> DeviceCaps {
    DeviceCaps {
        driver: "mock".into(),
        label: "mock".into(),
        rx_channels: 1,
        sample_rates: vec![RATE],
        freq_ranges_rx: vec![(1_000_000.0, 30_000_000.0)],
        ..DeviceCaps::default()
    }
}

/// The CW skimmer alone, on the decoder that does not need the model: this is
/// about which slice of band is being read, not about how well it is read.
fn cw_only() -> SkimmerSettings {
    let mut s = SkimmerSettings { enabled: [false; 3], ..SkimmerSettings::default() };
    s.enabled[SkimmerKind::Cw.index()] = true;
    s.cw_decoder = CwEngine::Timing;
    s
}

#[test]
fn a_station_on_the_visible_part_of_a_wide_span_is_skimmed() {
    let root = std::env::temp_dir().join(format!("sdroxide-skim-window-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(&root).unwrap();
    unsafe { std::env::set_var("SDROXIDE_CONFIG_DIR", &root) };

    // The skimmers are fed from the same pass as the main receiver, so an engine
    // with nowhere to play audio never reaches them: hence a ring nothing reads.
    let (producer, _consumer) = rtrb::RingBuffer::<f32>::new(48_000);
    let source = Band {
        center: Arc::new(Mutex::new(DEV_CENTER)),
        clip: keyed_cw("CQ CQ DE W1AW W1AW K ", STATION - DEV_CENTER, 20.0, RATE),
        pos: 0,
        started: Instant::now(),
        sent: 0,
        seed: 0x1234_5678_9ABC_DEF0,
    };
    let mut h = start_engine(
        Box::new(source),
        caps(),
        EngineConfig {
            remember_session: false,
            audio: Some(AudioParams { producer, out_rate: 48_000.0 }),
            ..Default::default()
        },
    );
    let thread = h.thread.take();
    let send = |c: Command| h.cmd_tx.send(c).unwrap();

    // The operator is listening to the station and looking at 40 kHz around it —
    // one tenth of what the front end is delivering, and nowhere near its centre.
    send(Command::SetVfo { vfo: Vfo::A, hz: STATION });
    send(Command::SetSkimmerView(Some((STATION - 20_000.0, STATION + 20_000.0))));
    send(Command::SetSkimmerConfig(cw_only()));

    let deadline = Instant::now() + Duration::from_secs(45);
    let mut seen: Vec<f64> = Vec::new();
    while Instant::now() < deadline {
        while let Ok(ev) = h.event_rx.try_recv() {
            if let RadioEvent::SkimmerSpots(spots) = ev {
                for s in &spots {
                    seen.push(s.freq_hz);
                    if (s.freq_hz - STATION).abs() < 300.0 && s.callsign.as_deref() == Some("W1AW")
                    {
                        drop(h.cmd_tx);
                        if let Some(t) = thread {
                            let _ = t.join();
                        }
                        unsafe { std::env::remove_var("SDROXIDE_CONFIG_DIR") };
                        return;
                    }
                }
            }
        }
        std::thread::sleep(Duration::from_millis(50));
    }
    panic!(
        "the station on {:.3} MHz was never spotted — the front end is centred on {:.3} MHz, so \
         the skim window has to have followed the view to reach it. Spots seen: {seen:?}",
        STATION / 1e6,
        DEV_CENTER / 1e6,
    );
}
