//! The worker thread the engine talks to.
//!
//! Same shape as the ADS-B and VDL2 controllers: a bounded channel for I/Q that
//! drops blocks rather than stalling the realtime thread, an unbounded one for
//! control that must never be dropped behind a backed-up queue, and a whole
//! snapshot of the vessel table twice a second.
//!
//! Inside, the window is split into one down-converter and one
//! [`crate::channel::ChannelRx`] per channel of the plan the window can reach.
//!
//! # Why the whole table goes out every time
//!
//! Forwarding each decoded report as it arrives is fewer bytes and worse in
//! every other way: a remote client that connected mid-session would see
//! nothing until the next transmission, a dropped report would leave a hole in
//! a vessel's trail forever, and the panel would have to fold the pieces of
//! twenty-seven message types into a table it was already keeping. The snapshot
//! does that folding once, on the worker, and has the property every snapshot
//! has — a dropped one costs nothing, because the next carries the same
//! information.

use std::thread::JoinHandle;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
use sdroxide_dsp::{Complex32, Ddc};
use sdroxide_types::{AisChannelStatus, AisSettings, AisStatus};
use tracing::info;

use crate::channel::{ChannelRx, Decoded};
use crate::plan;
use crate::track::Tracker;

/// How often a snapshot goes out.
///
/// Twice a second. Faster would re-send a table that had not moved — the
/// quickest AIS reporting interval is two seconds — and slower would make a
/// ferry visibly step across the map rather than track.
const EMIT_INTERVAL: Duration = Duration::from_millis(500);

/// What the engine drains each tick.
pub enum AisAction {
    Status(Box<AisStatus>),
}

/// Realtime data, dropped on backpressure.
struct Iq(Vec<Complex32>);

/// Control traffic, never dropped.
enum Ctl {
    Window { center_hz: f64, rate_hz: f64 },
    Config(AisSettings),
    Stop,
}

pub struct AisController {
    iq_tx: Sender<Iq>,
    ctl_tx: Sender<Ctl>,
    res_rx: Receiver<AisAction>,
    worker: Option<JoinHandle<()>>,
}

impl AisController {
    /// `window_rate_hz` is the rate of the I/Q the engine will feed, and
    /// `window_center_hz` the absolute RF frequency it is centred on.
    pub fn new(window_center_hz: f64, window_rate_hz: f64, cfg: AisSettings) -> AisController {
        let (iq_tx, iq_rx) = bounded::<Iq>(64);
        let (ctl_tx, ctl_rx) = unbounded::<Ctl>();
        let (res_tx, res_rx) = unbounded::<AisAction>();

        let worker = std::thread::Builder::new()
            .name("sdroxide-ais".into())
            .spawn(move || {
                let mut w = Worker::new(window_center_hz, window_rate_hz, cfg);
                let mut last_emit = Instant::now();
                loop {
                    select! {
                        recv(ctl_rx) -> msg => match msg {
                            Ok(Ctl::Window { center_hz, rate_hz }) => {
                                w.set_window(center_hz, rate_hz);
                            }
                            Ok(Ctl::Config(next)) => w.set_config(next),
                            Ok(Ctl::Stop) | Err(_) => break,
                        },
                        recv(iq_rx) -> msg => match msg {
                            Ok(Iq(iq)) => {
                                w.process(&iq);
                                if last_emit.elapsed() >= EMIT_INTERVAL {
                                    last_emit = Instant::now();
                                    if res_tx
                                        .send(AisAction::Status(Box::new(w.status())))
                                        .is_err()
                                    {
                                        break;
                                    }
                                }
                            }
                            Err(_) => break,
                        },
                    }
                }
            })
            .expect("spawn ais worker");

        AisController { iq_tx, ctl_tx, res_rx, worker: Some(worker) }
    }

    /// Realtime path: hand a block of window-rate I/Q to the worker.
    /// Non-blocking; drops the block if the worker is behind.
    pub fn on_rx_iq(&self, iq: &[Complex32]) {
        let _ = self.iq_tx.try_send(Iq(iq.to_vec()));
    }

    /// The window moved — the front end retuned, or changed rate.
    ///
    /// The vessel table survives: a receiver nudged a few kilohertz is still
    /// watching the same sea, and throwing away an hour of shipping for it
    /// would be a worse answer than the second of silence while the chains are
    /// rebuilt.
    pub fn set_window(&self, center_hz: f64, rate_hz: f64) {
        let _ = self.ctl_tx.send(Ctl::Window { center_hz, rate_hz });
    }

    pub fn set_config(&self, cfg: AisSettings) {
        let _ = self.ctl_tx.send(Ctl::Config(cfg));
    }

    /// Drain whatever the worker has produced since the last poll.
    /// Non-blocking.
    pub fn poll(&self) -> Vec<AisAction> {
        let mut out = Vec::new();
        while let Ok(a) = self.res_rx.try_recv() {
            out.push(a);
        }
        out
    }
}

impl Drop for AisController {
    fn drop(&mut self) {
        let _ = self.ctl_tx.send(Ctl::Stop);
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}

/// One channel of the plan that is actually being decoded.
struct Chan {
    /// Index into [`plan::CHANNELS`].
    index: usize,
    ddc: Ddc,
    rx: ChannelRx,
    buf: Vec<Complex32>,
}

struct Worker {
    window_center_hz: f64,
    window_rate_hz: f64,
    cfg: AisSettings,
    chans: Vec<Chan>,
    tracker: Tracker,
    decoded: Vec<Decoded>,
    /// Samples seen, which at a known rate is the stream clock — used to
    /// expire the table without asking the operating system the time on every
    /// block.
    samples: u64,
    last_expire: u64,
}

impl Worker {
    fn new(window_center_hz: f64, window_rate_hz: f64, cfg: AisSettings) -> Worker {
        let cfg = cfg.sane();
        let mut w = Worker {
            window_center_hz,
            window_rate_hz,
            cfg,
            chans: Vec::new(),
            tracker: Tracker::new(cfg),
            decoded: Vec::new(),
            samples: 0,
            last_expire: 0,
        };
        w.rebuild();
        w
    }

    /// Open a down-converter and a receiver on every channel the window reaches
    /// and the operator has left switched on.
    ///
    /// The two reasons a channel is dark are recorded separately, because
    /// "outside the receiver's window" and "you turned it off" produce the same
    /// empty column and want completely different answers.
    fn rebuild(&mut self) {
        self.chans.clear();
        let reachable = plan::channels_in_window(self.window_center_hz, self.window_rate_hz);
        // Whether *both* channels are in the window decides how far the channel
        // stream may be decimated — see `plan::channel_decimation`, and the
        // 50 kHz coincidence it exists for.
        let both = reachable.len() == plan::CHANNELS.len();
        let target = plan::channel_rate_for(self.window_rate_hz, both);
        for &i in &reachable {
            if !self.cfg.channel_enabled(i) {
                continue;
            }
            let ch = &plan::CHANNELS[i];
            let mut ddc = Ddc::new(self.window_rate_hz, target);
            ddc.set_offset_hz(ch.center_hz - self.window_center_hz);
            let rate = ddc.out_rate();
            let label = ch.label.chars().next().unwrap_or('A');
            let rx = ChannelRx::new(ch.center_hz, label, rate, f32::from(self.cfg.threshold_db));
            self.chans.push(Chan { index: i, ddc, rx, buf: Vec::new() });
        }
        info!(
            channels = self.chans.len(),
            center = self.window_center_hz,
            rate = self.window_rate_hz,
            channel_rate = target,
            "AIS channels opened"
        );
    }

    fn set_window(&mut self, center_hz: f64, rate_hz: f64) {
        if (center_hz - self.window_center_hz).abs() < 1.0
            && (rate_hz - self.window_rate_hz).abs() < 1.0
        {
            return;
        }
        self.window_center_hz = center_hz;
        self.window_rate_hz = rate_hz;
        self.rebuild();
    }

    fn set_config(&mut self, cfg: AisSettings) {
        let cfg = cfg.sane();
        let channels_changed = cfg.channels != self.cfg.channels;
        self.cfg = cfg;
        self.tracker.set_config(cfg);
        if channels_changed {
            self.rebuild();
            return;
        }
        // A threshold change goes straight through: rebuilding would throw away
        // each channel's learned noise floor, and the noise did not move
        // because the operator dragged a slider.
        for c in &mut self.chans {
            c.rx.set_threshold_db(f32::from(cfg.threshold_db));
        }
    }

    fn process(&mut self, iq: &[Complex32]) {
        self.samples += iq.len() as u64;
        let now = unix_now();
        let mut decoded = std::mem::take(&mut self.decoded);
        for c in &mut self.chans {
            c.buf.clear();
            c.ddc.process(iq, &mut c.buf);
            decoded.clear();
            c.rx.push(&c.buf, &mut decoded);
            for d in &decoded {
                self.tracker.absorb(d, now);
            }
        }
        decoded.clear();
        self.decoded = decoded;

        // Expiring walks the whole table, and its shortest window is five
        // minutes; once a second is plenty.
        if self.samples.saturating_sub(self.last_expire) as f64 >= self.window_rate_hz {
            self.last_expire = self.samples;
            self.tracker.expire(now);
        }
    }

    fn status(&self) -> AisStatus {
        let mut st = AisStatus {
            window_center_hz: self.window_center_hz,
            window_rate_hz: self.window_rate_hz,
            vessels: self.tracker.snapshot(),
            ..AisStatus::default()
        };
        for (i, ch) in plan::CHANNELS.iter().enumerate() {
            let live = self.chans.iter().find(|c| c.index == i);
            let reason = if live.is_some() {
                None
            } else if !self.cfg.channel_enabled(i) {
                Some("switched off".to_string())
            } else {
                Some("outside the receiver's window".to_string())
            };
            let (bursts, messages, floor) = match live {
                Some(c) => {
                    let n = c.rx.counters();
                    (n.bursts, n.messages, c.rx.floor_dbfs())
                }
                None => (0, 0, 0.0),
            };
            st.channels.push(AisChannelStatus {
                freq_hz: ch.center_hz,
                label: ch.label.to_string(),
                live: live.is_some(),
                reason,
                bursts,
                messages,
                floor_dbfs: floor,
            });
        }
        for c in &self.chans {
            let n = c.rx.counters();
            st.bursts += n.bursts;
            st.messages += n.messages;
            st.bad_fcs += n.bad_fcs;
            st.unsupported += n.unsupported;
        }
        st.samples_per_bit = self.chans.first().map_or(0.0, |c| c.rx.samples_per_bit() as f32);
        // The mean of whichever channels have measured one: they are looking at
        // the same oscillator, so a channel that has heard nothing yet should
        // not drag the figure toward zero.
        let offsets: Vec<f32> = self.chans.iter().filter_map(|c| c.rx.offset_hz()).collect();
        if !offsets.is_empty() {
            st.offset_hz = Some(offsets.iter().sum::<f32>() / offsets.len() as f32);
        }
        st
    }
}

fn unix_now() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs() as i64).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tx::{Noise, Payload, TxParams, modulate_bits};

    /// A transmission on one channel of the plan comes out of the controller —
    /// the whole crate, through the thread the engine talks to, with the window
    /// split into channels the way the engine feeds it.
    #[test]
    fn a_transmission_on_channel_b_reaches_the_snapshot() {
        let window_rate = 150_000.0;
        let window_center = plan::ideal_center_hz();
        let c = AisController::new(window_center, window_rate, AisSettings::default());

        let p = TxParams { sample_rate: window_rate, ..TxParams::default() };
        let bits = Payload::new(168)
            .put(0, 6, 1)
            .put(8, 30, 244_660_000)
            .put(50, 10, 154)
            .put_signed(61, 28, (4.8973 * 600_000.0) as i64)
            .put_signed(89, 27, (52.3791 * 600_000.0) as i64)
            .put(116, 12, 900)
            .bits();
        let mut sig = modulate_bits(&bits, &p);
        // Put it where channel B is, relative to the window's centre.
        crate::tx::shift(&mut sig, plan::CHANNELS[1].center_hz - window_center, window_rate);

        let mut n = Noise::new(0xA15);
        let mut quiet = vec![Complex32::default(); 40_000];
        n.add(&mut quiet, 0.004);
        n.add(&mut sig, 0.004);

        c.on_rx_iq(&quiet);
        c.on_rx_iq(&sig);
        c.on_rx_iq(&quiet);

        // The worker emits on its own clock; give it a moment and drain.
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut last: Option<AisStatus> = None;
        while Instant::now() < deadline {
            for a in c.poll() {
                let AisAction::Status(st) = a;
                if !st.vessels.is_empty() {
                    last = Some(*st);
                }
            }
            if last.is_some() {
                break;
            }
            c.on_rx_iq(&quiet);
            std::thread::sleep(Duration::from_millis(20));
        }
        let st = last.expect("the decoder never reported a vessel");
        assert_eq!(st.vessels.len(), 1);
        let v = &st.vessels[0];
        assert_eq!(v.mmsi, 244_660_000);
        assert_eq!(v.channel, 'B', "it was transmitted on channel B");
        assert_eq!(v.sog_kt, Some(15.4));
        assert_eq!(st.channels.len(), 2, "both channels are always reported");
        assert!(st.channels.iter().all(|c| c.live));
        assert!(st.samples_per_bit >= sdroxide_types::AIS_GOOD_SPS as f32);
    }
}
