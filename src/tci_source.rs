//! An [`IqSource`] for one receiver of a TCI (Transceiver Control Interface)
//! server. Receive is wideband IQ over the WebSocket → the engine's normal
//! DDC/demod path (`audio_mode = false`); transmit is raw 48 kHz audio
//! (`tx_write_audio`) which the rig modulates (`caps.tx_audio`). Control
//! (freq/mode/PTT) goes over the same WebSocket.
//!
//! The connection is shared: a rig with two receivers (SunSDR2DX) serves two
//! radio tabs from one WebSocket, each tab an independent `TciSource` on its
//! own receiver index. The [`crate::device_registry`] is what pairs a second
//! source with the first one's connection; the transmitter belongs to the
//! receiver-0 source alone.

use std::time::{Duration, Instant};

use sdroxide_radio::{Complex32, ControlUpdate, IqSource, Result};
use sdroxide_tci::{TciDevice, TciRx, TciUpdate};
use sdroxide_types::TxTelemetry;

use crate::device_registry::{DeviceKey, SharedDevice, registry};

impl SharedDevice for TciDevice {
    fn is_alive(&self) -> bool {
        TciDevice::is_alive(self)
    }
}

pub struct TciSource {
    /// The shared connection and this source's stream on it. `None` after
    /// [`IqSource::release`]: the stream is given back so a rebuilt source
    /// can claim the receiver again, while other radios' streams on the same
    /// connection run on undisturbed.
    dev: Option<std::sync::Arc<TciDevice>>,
    rx: Option<TciRx>,
    /// The connection's negotiated IQ rate, kept here so a released source
    /// still answers `IqSource::sample_rate` — see that method.
    rate: f64,
    center: f64,
    scratch: Vec<f32>,
    label: String,
    /// Last VFO-within-band offset pushed to the rig (dedup).
    if_offset: f64,
    /// Latest SWR the rig reported while keyed, latched for the engine's meter
    /// poll. Cleared on unkey.
    last_telem: Option<TxTelemetry>,
    /// How far behind its `dds:` commands this rig's IQ runs — see
    /// [`IqSource::stream_delay_s`]. Read once at open; the operator changes it
    /// by re-applying the radio settings, like every other TCI field.
    stream_delay_s: f64,
}

impl TciSource {
    /// Attach receiver `rx_index` of the rig at `address`, connecting only if
    /// no radio in this process already holds that connection. The IQ rate is
    /// a property of the connection, so whoever connects first sets it and a
    /// later attach runs at the established rate whatever its own config says.
    pub fn open(
        address: &str,
        iq_rate_hz: f64,
        center_hz: f64,
        rx_index: u32,
        stream_delay_ms: f64,
    ) -> anyhow::Result<Self> {
        let dev = registry()
            .get_or_open(DeviceKey::Tci(address.to_string()), || {
                TciDevice::connect(address, iq_rate_hz)
                    .map(std::sync::Arc::new)
                    .map_err(|e| e.to_string())
            })
            .map_err(anyhow::Error::msg)?;
        let rx = dev.rx(rx_index).map_err(|e| anyhow::Error::msg(e.to_string()))?;
        rx.set_center(center_hz);
        let label = if rx_index == 0 {
            format!("TCI {} @ {address} ({:.0} kHz IQ)", dev.device(), dev.sample_rate_hz() / 1e3)
        } else {
            format!(
                "TCI RX{} {} @ {address} ({:.0} kHz IQ)",
                rx_index + 1,
                dev.device(),
                dev.sample_rate_hz() / 1e3
            )
        };
        Ok(TciSource {
            rate: dev.sample_rate_hz(),
            center: center_hz,
            scratch: Vec::new(),
            label,
            dev: Some(dev),
            rx: Some(rx),
            if_offset: 0.0,
            last_telem: None,
            // Negative or absurd values are an edited config, not a request:
            // clamp rather than let the panadapter's axis run backwards.
            stream_delay_s: (stream_delay_ms / 1e3).clamp(0.0, 2.0),
        })
    }

    /// The connection's negotiated IQ rate, remembered rather than asked for:
    /// it is fixed at connect, and [`IqSource::release`] lets the connection go
    /// while the engine is still running on this source.
    pub fn sample_rate_hz(&self) -> f64 {
        self.rate
    }
}

impl IqSource for TciSource {
    fn sample_rate(&self) -> f64 {
        self.sample_rate_hz()
    }

    fn center_hz(&self) -> f64 {
        self.center
    }

    /// The rig is at the end of a socket and its own pipeline: a `dds:` command
    /// is acknowledged in well under a millisecond but the IQ takes about a
    /// tenth of a second to follow. See `TciConfig::stream_delay_ms`.
    fn stream_delay_s(&self) -> f64 {
        self.stream_delay_s
    }

    fn set_center_hz(&mut self, hz: f64) -> Result<()> {
        self.center = hz;
        if let Some(rx) = self.rx.as_ref() {
            rx.set_center(hz);
        }
        Ok(())
    }

    fn read(&mut self, buf: &mut [Complex32]) -> Result<usize> {
        let Some(rx) = self.rx.as_mut() else {
            // Released: nothing will ever arrive; nap so the engine loop
            // doesn't spin while the reopen it asked for is prepared.
            std::thread::sleep(Duration::from_millis(5));
            return Ok(0);
        };
        let need = buf.len() * 2;
        if self.scratch.len() < need {
            self.scratch.resize(need, 0.0);
        }
        let n = rx.rx_read(&mut self.scratch[..need]);
        let pairs = n / 2;
        if pairs == 0 {
            std::thread::sleep(Duration::from_millis(2));
            return Ok(0);
        }
        for p in 0..pairs {
            buf[p] = Complex32::new(self.scratch[2 * p], self.scratch[2 * p + 1]);
        }
        Ok(pairs)
    }

    fn describe(&self) -> String {
        self.label.clone()
    }

    fn set_control_mode(&mut self, mode: sdroxide_types::Mode) -> Result<()> {
        if let Some(rx) = self.rx.as_ref() {
            rx.set_mode(mode);
        }
        Ok(())
    }

    fn poll_control(&mut self) -> Vec<ControlUpdate> {
        let Some(rx) = self.rx.as_ref() else { return Vec::new() };
        rx.poll_updates()
            .into_iter()
            .map(|u| match u {
                TciUpdate::Freq(hz) => ControlUpdate::Freq(hz),
                TciUpdate::Mode(m) => ControlUpdate::Mode(m),
                TciUpdate::Drive(f) => ControlUpdate::TxDrive(f),
                TciUpdate::TuneDrive(f) => ControlUpdate::TuneDrive(f),
            })
            .collect()
    }

    fn tx_begin(&mut self, center_hz: f64, _rate: f64) -> Result<f64> {
        match self.rx.as_ref() {
            Some(rx) => Ok(rx.tx_begin(center_hz)),
            None => Ok(sdroxide_tci::TX_RATE_HZ as f64),
        }
    }

    fn tx_write_audio(&mut self, audio: &[f32]) -> Result<()> {
        if let Some(rx) = self.rx.as_mut() {
            rx.tx_write(audio);
        }
        Ok(())
    }

    /// Let the queued audio reach the rig before PTT drops. The engine hands us
    /// a burst faster than real time and the rig pulls it one buffer at a time,
    /// so unkeying immediately would cut the tail (FT8 needs every symbol).
    fn tx_drain(&mut self) {
        let Some(rx) = self.rx.as_ref() else { return };
        let deadline = Instant::now() + Duration::from_secs(2);
        while rx.tx_pending() > 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(2));
        }
        // The last packet handed over is still playing out in the rig.
        std::thread::sleep(Duration::from_millis(40));
    }

    fn tx_end(&mut self) -> Result<()> {
        if let Some(rx) = self.rx.as_ref() {
            rx.tx_end();
        }
        self.last_telem = None; // drop the stale SWR reading on unkey
        Ok(())
    }

    fn discard_pending_rx(&mut self) {
        if let Some(rx) = self.rx.as_mut() {
            rx.discard_pending_rx();
        }
    }

    fn tx_telemetry(&mut self) -> Option<TxTelemetry> {
        if let Some(t) = self.rx.as_ref().and_then(TciRx::poll_telemetry) {
            self.last_telem = Some(t);
        }
        self.last_telem
    }

    fn set_tx_drive(&mut self, frac: f64) {
        if let Some(rx) = self.rx.as_ref() {
            rx.set_drive(frac);
        }
    }

    fn set_tune_drive(&mut self, frac: f64) {
        if let Some(rx) = self.rx.as_ref() {
            rx.set_tune_drive(frac);
        }
    }

    /// The rig's `drive:`/`tune_drive:` are its real power control, so the TX
    /// audio we stream must stay full scale.
    fn commands_tx_power(&self) -> bool {
        true
    }

    /// The WebSocket thread stops when the rig closes the connection
    /// (ExpertSDR3 quit or restarted); the engine then reconnects on its own.
    /// A released source likewise: only a fresh open revives it.
    fn needs_reopen(&self) -> bool {
        self.rx.as_ref().is_none_or(|rx| !rx.is_alive())
    }

    /// Give this receiver's stream back ahead of a rebuild — **and the
    /// connection with it, unless another radio is still streaming its own
    /// receiver over it.**
    ///
    /// The WebSocket used to be kept deliberately, so that an Apply with the
    /// address unchanged re-attached to the live one rather than redialling
    /// the rig. But the IQ rate is negotiated once, at connect, and never
    /// again: a connection outliving its last radio made an Apply that changed
    /// it do nothing at all. Same rule as the Pluto and HPSDR backends next
    /// door (issue #135) and as the RSP.
    fn release(&mut self) {
        self.rx = None;
        self.dev = None;
    }

    fn set_if_offset(&mut self, hz: f64) {
        if (hz - self.if_offset).abs() > 0.5 {
            self.if_offset = hz;
            if let Some(rx) = self.rx.as_ref() {
                rx.set_if_offset(hz);
            }
        }
    }
}
