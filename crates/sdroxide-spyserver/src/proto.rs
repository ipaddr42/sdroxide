//! The SpyServer wire protocol: constants, headers, and the arithmetic that
//! turns what a server says about itself into rates, spans and gains.
//!
//! Nothing here touches a socket, so all of it is testable against byte
//! literals — which matters more than usual, because there is no specification
//! to check against. The protocol is defined by `spyserver_protocol.h` in
//! SDR#'s and SDR++'s sources and by what the servers actually do; the numbers
//! below are transcribed from the former and the behaviour from the latter.
//!
//! Everything is little-endian.

use sdroxide_types::{SpyServerConfig, SpyServerFormat};

use crate::error::{Error, Result};

/// A protocol version as the wire packs it: major, minor, build.
///
/// Spelled out as a function rather than folded into a literal so that the
/// three fields stay visible — a `ProtocolID` is read back apart again in
/// [`MessageHeader::parse`], and a constant that hid its shape would make the
/// two ends easy to get out of step.
pub const fn version(major: u32, minor: u32, build: u32) -> u32 {
    (major << 24) | (minor << 16) | build
}

/// The version this client announces. Servers check the major number and are
/// tolerant of the rest, which is how a 2017 protocol still talks to current
/// software.
pub const PROTOCOL_VERSION: u32 = version(2, 0, 1700);

/// Major version this client can read. A server announcing anything else in
/// its `ProtocolID` is speaking a protocol this code has not seen.
pub const PROTOCOL_MAJOR: u32 = 2;

/// `{u32 CommandType, u32 BodySize}`.
pub const CMD_HEADER_LEN: usize = 8;
/// `{u32 ProtocolID, u32 MessageType, u32 StreamType, u32 SequenceNumber, u32 BodySize}`.
pub const MSG_HEADER_LEN: usize = 20;

/// The protocol's own ceiling on a message body. Used as the sanity check on a
/// declared `BodySize`: past this the stream is out of step, and since there is
/// no resync marker anywhere in this protocol, that is not recoverable.
pub const MAX_MESSAGE_BODY_SIZE: u32 = 1 << 20;

/// How close to the edge of the FFT window the dial has to get before the
/// window moves. See [`DeviceInfo::fft_recenter`].
pub const FFT_EDGE_FRAC: f64 = 0.30;

/// Bytes in a `DeviceInfo` body: twelve `u32`.
pub const DEVICE_INFO_LEN: usize = 12 * 4;
/// Bytes in a `ClientSync` body: nine `u32`.
pub const CLIENT_SYNC_LEN: usize = 9 * 4;

// --- commands ---------------------------------------------------------------

pub const CMD_HELLO: u32 = 0;
pub const CMD_SET_SETTING: u32 = 2;
pub const CMD_PING: u32 = 3;

// --- settings ---------------------------------------------------------------

pub const SETTING_STREAMING_MODE: u32 = 0;
pub const SETTING_STREAMING_ENABLED: u32 = 1;
pub const SETTING_GAIN: u32 = 2;
pub const SETTING_IQ_FORMAT: u32 = 100;
pub const SETTING_IQ_FREQUENCY: u32 = 101;
pub const SETTING_IQ_DECIMATION: u32 = 102;
pub const SETTING_IQ_DIGITAL_GAIN: u32 = 103;
pub const SETTING_FFT_FORMAT: u32 = 200;
pub const SETTING_FFT_FREQUENCY: u32 = 201;
pub const SETTING_FFT_DECIMATION: u32 = 202;
pub const SETTING_FFT_DB_OFFSET: u32 = 203;
pub const SETTING_FFT_DB_RANGE: u32 = 204;
pub const SETTING_FFT_DISPLAY_PIXELS: u32 = 205;

// --- streaming modes --------------------------------------------------------

pub const STREAM_MODE_IQ_ONLY: u32 = 1;
pub const STREAM_MODE_FFT_IQ: u32 = 5;

// --- stream formats ---------------------------------------------------------

pub const FORMAT_UINT8: u32 = 1;
pub const FORMAT_INT16: u32 = 2;
pub const FORMAT_INT24: u32 = 3;
pub const FORMAT_FLOAT: u32 = 4;

// --- message types ----------------------------------------------------------

pub const MSG_DEVICE_INFO: u16 = 0;
pub const MSG_CLIENT_SYNC: u16 = 1;
pub const MSG_PONG: u16 = 2;
pub const MSG_UINT8_IQ: u16 = 100;
pub const MSG_INT16_IQ: u16 = 101;
pub const MSG_INT24_IQ: u16 = 102;
pub const MSG_FLOAT_IQ: u16 = 103;
/// 4-bit differential-coded FFT. The encoding is undocumented and no open
/// client implements it, so this end never asks for it — the constant exists
/// only so a frame that arrives anyway can be recognised and ignored rather
/// than decoded as something else.
///
/// The FFT message types are the two that run past 255, which is worth
/// noticing: they still fit the low half of the type word, where the kind
/// lives, and it is the *high* half that carries the gain flags.
pub const MSG_DINT4_FFT: u16 = 300;
pub const MSG_UINT8_FFT: u16 = 301;

/// What the far-end receiver is, as the server reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum DeviceKind {
    #[default]
    Invalid,
    /// Airspy R2 or Mini.
    AirspyOne,
    /// Airspy HF+ Dual, Discovery or Ranger.
    AirspyHf,
    RtlSdr,
    /// A server reporting a device this table does not have. Not an error —
    /// the ladder and the tuning range come from `DeviceInfo`, not from the
    /// device type, and only the digital-gain formula needs to know.
    Other(u32),
}

impl DeviceKind {
    pub fn from_wire(v: u32) -> DeviceKind {
        match v {
            0 => DeviceKind::Invalid,
            1 => DeviceKind::AirspyOne,
            2 => DeviceKind::AirspyHf,
            3 => DeviceKind::RtlSdr,
            other => DeviceKind::Other(other),
        }
    }

    pub fn name(self) -> String {
        match self {
            DeviceKind::Invalid => "no device".to_string(),
            DeviceKind::AirspyOne => "Airspy R2 / Mini".to_string(),
            DeviceKind::AirspyHf => "Airspy HF+".to_string(),
            DeviceKind::RtlSdr => "RTL-SDR".to_string(),
            DeviceKind::Other(v) => format!("device type {v}"),
        }
    }
}

/// A parsed message header.
///
/// `MessageType` on the wire is two fields in one word: the low half is the
/// message kind, the high half a flags word which — on a sample message — is
/// the digital gain the server applied, in whole dB.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MessageHeader {
    pub protocol_id: u32,
    pub kind: u16,
    pub flags: u16,
    pub stream_type: u32,
    pub sequence: u32,
    pub body_size: u32,
}

impl MessageHeader {
    pub fn parse(b: &[u8; MSG_HEADER_LEN]) -> Result<MessageHeader> {
        let w = |i: usize| u32::from_le_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
        let protocol_id = w(0);
        let msg_type = w(4);
        let body_size = w(16);

        // Both guards below are fatal, and deliberately so. Unlike `rtl_tcp`,
        // this protocol has no marker to resynchronise on: one byte out of step
        // and every subsequent "header" is read from the middle of a body, so
        // there is nothing to do but drop the connection and reconnect.
        if protocol_id >> 24 != PROTOCOL_MAJOR {
            return Err(Error::Proto(format!(
                "the server announced protocol {}.{}.{}, which this client cannot read — \
                 it expects major version {PROTOCOL_MAJOR}",
                protocol_id >> 24,
                (protocol_id >> 16) & 0xFF,
                protocol_id & 0xFFFF,
            )));
        }
        if body_size > MAX_MESSAGE_BODY_SIZE {
            return Err(Error::Proto(format!(
                "the server announced a {body_size}-byte message body, past the protocol's \
                 {MAX_MESSAGE_BODY_SIZE}-byte limit — the stream is out of step and this \
                 protocol has nothing to resynchronise on"
            )));
        }

        Ok(MessageHeader {
            protocol_id,
            kind: (msg_type & 0xFFFF) as u16,
            flags: ((msg_type >> 16) & 0xFFFF) as u16,
            stream_type: w(8),
            sequence: w(12),
            body_size,
        })
    }

    /// The linear factor the server multiplied the samples by before sending
    /// them, from the flags field's whole dB.
    ///
    /// This is what the server *did*, which is not the same thing as
    /// `IQ_DIGITAL_GAIN`, which is what it was *asked* to do. Only this may
    /// touch a sample: applying the requested figure as well would scale the
    /// stream twice, and the symptom — a level wrong by exactly the gain —
    /// looks like a gain-table bug rather than the double application it is.
    pub fn digital_gain(&self) -> f32 {
        10f32.powf(f32::from(self.flags) / 20.0)
    }
}

/// What the server said about the receiver behind it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DeviceInfo {
    pub device_type: u32,
    pub serial: u32,
    pub maximum_sample_rate: u32,
    pub maximum_bandwidth: u32,
    pub decimation_stage_count: u32,
    pub gain_stage_count: u32,
    pub maximum_gain_index: u32,
    pub minimum_frequency: u32,
    pub maximum_frequency: u32,
    /// ADC bits, as the server counts them — 16 or 18 for an Airspy HF+, 8 for
    /// an RTL-SDR. Shown in [`Self::describe`] and used for nothing else: it
    /// is the width of the ADC, not of the wire, and keying a wire decision on
    /// it is a mistake that has been made here once already.
    pub resolution: u32,
    pub minimum_iq_decimation: u32,
    /// A format the server insists on, or 0 for "take your pick".
    pub forced_iq_format: u32,
}

impl DeviceInfo {
    pub fn parse(b: &[u8]) -> Result<DeviceInfo> {
        if b.len() < DEVICE_INFO_LEN {
            return Err(Error::Proto(format!(
                "the server sent a {}-byte device-info message; it should be {DEVICE_INFO_LEN}",
                b.len()
            )));
        }
        let w = |i: usize| u32::from_le_bytes([b[i * 4], b[i * 4 + 1], b[i * 4 + 2], b[i * 4 + 3]]);
        Ok(DeviceInfo {
            device_type: w(0),
            serial: w(1),
            maximum_sample_rate: w(2),
            maximum_bandwidth: w(3),
            decimation_stage_count: w(4),
            gain_stage_count: w(5),
            maximum_gain_index: w(6),
            minimum_frequency: w(7),
            maximum_frequency: w(8),
            resolution: w(9),
            minimum_iq_decimation: w(10),
            forced_iq_format: w(11),
        })
    }

    pub fn kind(&self) -> DeviceKind {
        DeviceKind::from_wire(self.device_type)
    }

    /// The I/Q rates this server offers, as `(decimation stage, rate in Hz)`.
    ///
    /// The ladder is `MaximumSampleRate / 2ⁿ` from the server's own floor.
    /// `vfo` drops the last stage, matching the reference client: the very
    /// bottom of the ladder is a few hundred samples a second, which is below
    /// anything this program can demodulate and only clutters the list.
    pub fn iq_rates(&self, vfo: bool) -> Vec<(u32, f64)> {
        let top = if vfo {
            self.decimation_stage_count.saturating_sub(1)
        } else {
            self.decimation_stage_count
        };
        (self.minimum_iq_decimation..=top)
            .map(|i| (i, f64::from(self.maximum_sample_rate) / f64::from(1u32 << i.min(31))))
            .collect()
    }

    /// The FFT spans this server offers, as `(stage, labelled rate, delivered span)`.
    ///
    /// The two figures differ and both are needed. The reference clients label
    /// the dropdown with `MaximumSampleRate / 2ⁿ`, because those are the
    /// recognisable numbers, but what actually arrives covers
    /// `MaximumBandwidth / 2ⁿ` — the receiver's analog bandwidth, not its
    /// sample rate. The strip's axis has to use the second, or every signal on
    /// it is drawn at the wrong frequency.
    pub fn fft_stages(&self) -> Vec<(u32, f64, f64)> {
        (0..=self.decimation_stage_count)
            .map(|i| {
                let div = f64::from(1u32 << i.min(31));
                (
                    i,
                    f64::from(self.maximum_sample_rate) / div,
                    f64::from(self.maximum_bandwidth) / div,
                )
            })
            .collect()
    }

    /// The stage whose rate is nearest `target`, for
    /// [`SpyServerConfig::AUTO_DECIMATION`]. Prefers the stage at or below the
    /// target — going over costs link bandwidth the operator may not have.
    pub fn stage_for_rate(&self, target: f64, vfo: bool) -> u32 {
        let rates = self.iq_rates(vfo);
        rates
            .iter()
            .find(|&&(_, r)| r <= target)
            .or_else(|| rates.last())
            .map(|&(i, _)| i)
            .unwrap_or(self.minimum_iq_decimation)
    }

    /// The digital gain to open with, in dB, given the server's gain index and
    /// the I/Q decimation stage.
    ///
    /// This is the reference client's formula and deliberately nothing more:
    ///
    /// * Every decimation stage halves the bandwidth and so buys about 3 dB of
    ///   processing gain that would otherwise be thrown away when the samples
    ///   are quantised for the wire.
    /// * An Airspy R2's gain index counts *down* from its maximum, so a low
    ///   index means a quiet signal that needs the headroom back.
    ///
    /// The wire format is not an input, and the signature refuses it on
    /// purpose. An 8-bit stream from an Airspy HF+ used to get a flat 44 dB on
    /// top of this, because on a quiet band the HF+'s I/Q sits below one
    /// eighth-bit LSB and arrives as three distinct sample values. The boost is
    /// gone: how much gain a band needs is a property of the *band*, and 44 dB
    /// measured on a dead one drove the quantiser twenty-odd dB past full
    /// scale on every band with a signal on it. Every sample came back a rail
    /// and the stream carried nothing — the same figures on 80 m, 40 m and
    /// 20 m alike, which reads on screen as a receiver gone deaf.
    ///
    /// The quiet-band problem is real and is not solved here. It cannot be: a
    /// constant cannot be right for both a dead 20 m and a crowded 80 m at
    /// night, which are 30 dB apart. It belongs to the closed loop in the
    /// client that watches what actually arrives — `maintain_digital_gain` in
    /// `net.rs` — and this figure is only where that loop starts from.
    pub fn digital_gain_db(&self, gain_index: u32, decimation_stage: u32) -> f64 {
        let stage_gain = f64::from(decimation_stage) * 3.01;
        match self.kind() {
            DeviceKind::AirspyOne => {
                f64::from(self.maximum_gain_index.saturating_sub(gain_index)) + stage_gain
            }
            _ => stage_gain,
        }
    }

    /// The span the receiver actually hears, in Hz.
    ///
    /// `MaximumBandwidth` wherever a server states one, because that is the
    /// analog passband and everything between it and the sample rate is
    /// roll-off. On an Airspy HF+ the two are 660 kHz and 768 kHz, and the
    /// 54 kHz either side of the difference is where the noise floor climbs
    /// and nothing is received.
    ///
    /// Guarded rather than trusted. A server that leaves the field at zero
    /// would otherwise read as a receiver that hears nothing, which would pin
    /// every window to the device centre and refuse the operator's first tune
    /// — so a server with nothing to say for itself is taken at its sample
    /// rate, exactly as it was before this figure existed. A bandwidth wider
    /// than the sample rate is the same nonsense from the other end: nothing
    /// is heard that was never sampled.
    ///
    /// [`Self::fft_slack_hz`] deliberately does not come through here. The FFT
    /// spans are quoted in `MaximumBandwidth` to begin with, so its slack has
    /// to be measured in the same units, and a server that states no bandwidth
    /// has already had its FFT lane switched off for want of a span.
    pub fn usable_bandwidth_hz(&self) -> f64 {
        let rate = f64::from(self.maximum_sample_rate);
        match f64::from(self.maximum_bandwidth) {
            bw if bw > 0.0 => bw.min(rate),
            _ => rate,
        }
    }

    /// How far the I/Q window may slide either side of the device centre
    /// without the device itself having to move, in Hz.
    ///
    /// Measured against [`Self::usable_bandwidth_hz`] and not the sample rate,
    /// because room in the roll-off is not room. On an HF+ at 384 ksps the
    /// rate says 192 kHz of slack and the passband says 138 kHz, and the
    /// 54 kHz between the two answers is a dial that keeps moving onto signals
    /// getting quieter for a reason nothing on screen accounts for. Narrower is
    /// also the safe side of the server's own clamp: where a server holds
    /// `IQ_FREQUENCY` to its passband rather than its rate, asking past that is
    /// a position it pulls back without saying so — and per
    /// [`Self::fft_recenter`] the pull-back sticks, because the server keeps it
    /// as an absolute frequency.
    ///
    /// Computed from the geometry rather than read from
    /// `Min/MaximumIQCenterFrequency`, which several servers leave zeroed —
    /// trusting those would pin the window to the device centre on exactly the
    /// servers where sliding it matters most.
    pub fn iq_slack_hz(&self, iq_rate_hz: f64) -> f64 {
        (self.usable_bandwidth_hz() - iq_rate_hz).max(0.0) / 2.0
    }

    /// The same for the FFT window, against the analog bandwidth its spans are
    /// measured in. Zero at stage 0, where the window already covers
    /// everything there is.
    pub fn fft_slack_hz(&self, fft_span_hz: f64) -> f64 {
        (f64::from(self.maximum_bandwidth) - fft_span_hz).max(0.0) / 2.0
    }

    /// Where the FFT window should sit, or `None` to leave it where it is.
    ///
    /// The hysteresis is the whole point of the FFT lane being usable: a band
    /// view that re-centres on every retune is a band view nobody can read.
    /// The window holds while the dial is anywhere in the middle
    /// [`FFT_EDGE_FRAC`] of the span, and when it does move it puts the dial
    /// back in the middle — so the next move is most of a span away.
    ///
    /// `controls_device` is what all of that has to be qualified by, and the
    /// difference is not cosmetic. With the FFT lane running, a SpyServer tunes
    /// its *receiver* to the FFT window and treats `IQ_FREQUENCY` as a position
    /// inside the band the receiver is already on. So where this client controls
    /// the receiver, holding this window still holds the receiver still, and the
    /// dial can only stray as far as `iq_slack_hz` — the room the I/Q window has
    /// to slide on its own. At the top of the rate ladder there is none, and the
    /// window simply follows the dial.
    ///
    /// Where another client owns the receiver none of that applies: the device
    /// does not move for us at all, the window only slides inside what they are
    /// receiving, and the hysteresis is free to run its full width.
    pub fn fft_recenter(
        &self,
        iq_center: f64,
        fft_center: f64,
        fft_span: f64,
        device_center: f64,
        controls_device: bool,
        iq_slack_hz: f64,
    ) -> Option<f64> {
        let half = fft_span / 2.0;
        if half <= 0.0 {
            return None;
        }
        let mut hold = half * (1.0 - FFT_EDGE_FRAC);
        if controls_device {
            hold = hold.min(iq_slack_hz);
        }
        if (iq_center - fft_center).abs() <= hold {
            return None;
        }
        if controls_device {
            // The receiver goes wherever this window goes, so it goes to the
            // dial. Nothing to clamp against: the window is not sliding inside
            // a band somebody else chose, it is choosing the band.
            return Some(iq_center);
        }
        let slack = self.fft_slack_hz(fft_span);
        Some(iq_center.clamp(device_center - slack, device_center + slack))
    }

    /// One line for a log or a Test-connection reply.
    pub fn describe(&self) -> String {
        format!(
            "{} (serial {:08X}), {:.3} Msps max, {}-bit, gain 0..{}, {:.3}–{:.3} MHz",
            self.kind().name(),
            self.serial,
            f64::from(self.maximum_sample_rate) / 1e6,
            self.resolution,
            self.maximum_gain_index,
            f64::from(self.minimum_frequency) / 1e6,
            f64::from(self.maximum_frequency) / 1e6,
        )
    }
}

/// The server's view of where everything currently sits, resent whenever it
/// changes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ClientSync {
    /// Whether this client may move the device itself, as opposed to only its
    /// own window inside what the device is already receiving. Zero on a
    /// server whose owner is another client.
    pub can_control: u32,
    pub gain: u32,
    pub device_center_hz: u32,
    pub iq_center_hz: u32,
    pub fft_center_hz: u32,
    pub minimum_iq_center_hz: u32,
    pub maximum_iq_center_hz: u32,
    pub minimum_fft_center_hz: u32,
    pub maximum_fft_center_hz: u32,
}

impl ClientSync {
    pub fn parse(b: &[u8]) -> Result<ClientSync> {
        if b.len() < CLIENT_SYNC_LEN {
            return Err(Error::Proto(format!(
                "the server sent a {}-byte client-sync message; it should be {CLIENT_SYNC_LEN}",
                b.len()
            )));
        }
        let w = |i: usize| u32::from_le_bytes([b[i * 4], b[i * 4 + 1], b[i * 4 + 2], b[i * 4 + 3]]);
        Ok(ClientSync {
            can_control: w(0),
            gain: w(1),
            device_center_hz: w(2),
            iq_center_hz: w(3),
            fft_center_hz: w(4),
            minimum_iq_center_hz: w(5),
            maximum_iq_center_hz: w(6),
            minimum_fft_center_hz: w(7),
            maximum_fft_center_hz: w(8),
        })
    }
}

/// The opening command: our protocol version, then who we are.
///
/// The name has no terminator — its length is whatever is left of `BodySize`.
/// Servers log it and some allow-list on it, so it is worth sending something
/// recognisable.
pub fn frame_hello(client_name: &str) -> Vec<u8> {
    let name = client_name.as_bytes();
    let body_size = 4 + name.len();
    let mut out = Vec::with_capacity(CMD_HEADER_LEN + body_size);
    out.extend_from_slice(&CMD_HELLO.to_le_bytes());
    out.extend_from_slice(&(body_size as u32).to_le_bytes());
    out.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
    out.extend_from_slice(name);
    out
}

pub fn frame_setting(setting: u32, value: u32) -> [u8; CMD_HEADER_LEN + 8] {
    let mut out = [0u8; CMD_HEADER_LEN + 8];
    out[0..4].copy_from_slice(&CMD_SET_SETTING.to_le_bytes());
    out[4..8].copy_from_slice(&8u32.to_le_bytes());
    out[8..12].copy_from_slice(&setting.to_le_bytes());
    out[12..16].copy_from_slice(&value.to_le_bytes());
    out
}

pub fn frame_ping() -> [u8; CMD_HEADER_LEN] {
    let mut out = [0u8; CMD_HEADER_LEN];
    out[0..4].copy_from_slice(&CMD_PING.to_le_bytes());
    out
}

/// A frequency as the protocol carries it: an unsigned whole number of hertz.
/// Negative and out-of-range values are clamped rather than wrapped — a
/// wrapped `u32` would tune somewhere near 4 GHz.
pub fn freq_wire(hz: f64) -> u32 {
    hz.round().clamp(0.0, f64::from(u32::MAX)) as u32
}

/// The FFT window as it will be *sent*, clamped to what the protocol accepts.
///
/// Clamping here rather than letting the server do it is the point: a server
/// that silently clamps leaves this end decoding bytes against a window the
/// encoder did not use, which shifts the whole strip by the difference with
/// nothing to show for it. So the clamped values are what goes on the wire
/// *and* what the decoder is given.
pub fn clamp_fft_window(db_offset: f64, db_range: f64) -> (f64, f64) {
    (
        db_offset.clamp(SpyServerConfig::FFT_DB_OFFSET_MIN, SpyServerConfig::FFT_DB_OFFSET_MAX),
        db_range.clamp(SpyServerConfig::FFT_DB_RANGE_MIN, SpyServerConfig::FFT_DB_RANGE_MAX),
    )
}

/// The wire value for a format, refusing the ones this client cannot decode.
pub fn format_wire(f: SpyServerFormat) -> u32 {
    f.wire()
}

/// What a server's `ForcedIQFormat` means for us.
pub fn forced_format(info: &DeviceInfo) -> Result<Option<SpyServerFormat>> {
    match info.forced_iq_format {
        0 => Ok(None),
        FORMAT_INT24 => Err(Error::Unsupported(
            "this server forces 24-bit I/Q, which no open client decodes and this one does \
             not implement — the encoding is not documented anywhere"
                .into(),
        )),
        v => match SpyServerFormat::from_wire(v) {
            Some(f) => Ok(Some(f)),
            None => Err(Error::Unsupported(format!(
                "this server forces I/Q format {v}, which this client does not decode"
            ))),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A `DeviceInfo` as an Airspy HF+ Discovery's server reports it.
    ///
    /// The two widths differ and that is the point of using this one: a real
    /// HF+ samples 768 kHz and hears 660 kHz of it, so every figure derived
    /// from the wrong one of the two is wrong here by 54 kHz either side.
    fn hf_plus() -> DeviceInfo {
        DeviceInfo {
            device_type: 2,
            serial: 0x1234_5678,
            maximum_sample_rate: 768_000,
            maximum_bandwidth: 660_000,
            decimation_stage_count: 8,
            gain_stage_count: 1,
            maximum_gain_index: 8,
            minimum_frequency: 0,
            maximum_frequency: 260_000_000,
            resolution: 18,
            minimum_iq_decimation: 0,
            forced_iq_format: 0,
        }
    }

    /// An Airspy R2's: 10 Msps, and a server floor that hides the top stages.
    fn airspy_r2() -> DeviceInfo {
        DeviceInfo {
            device_type: 1,
            serial: 0xAABB_CCDD,
            maximum_sample_rate: 10_000_000,
            maximum_bandwidth: 10_000_000,
            decimation_stage_count: 8,
            gain_stage_count: 3,
            maximum_gain_index: 21,
            minimum_frequency: 24_000_000,
            maximum_frequency: 1_800_000_000,
            resolution: 12,
            minimum_iq_decimation: 2,
            forced_iq_format: 0,
        }
    }

    fn rtlsdr() -> DeviceInfo {
        DeviceInfo {
            device_type: 3,
            serial: 1,
            maximum_sample_rate: 2_400_000,
            maximum_bandwidth: 2_400_000,
            decimation_stage_count: 6,
            gain_stage_count: 1,
            maximum_gain_index: 28,
            minimum_frequency: 24_000_000,
            maximum_frequency: 1_766_000_000,
            resolution: 8,
            minimum_iq_decimation: 0,
            forced_iq_format: 0,
        }
    }

    #[test]
    fn hello_states_the_version_then_the_name() {
        let f = frame_hello("sdroxide");
        assert_eq!(&f[0..4], &0u32.to_le_bytes(), "command type HELLO");
        assert_eq!(&f[4..8], &12u32.to_le_bytes(), "4 bytes of version + 8 of name");
        assert_eq!(&f[8..12], &PROTOCOL_VERSION.to_le_bytes());
        assert_eq!(&f[12..], b"sdroxide", "the name has no terminator");
        assert_eq!(PROTOCOL_VERSION, 0x0200_06A4);
    }

    #[test]
    fn a_setting_is_two_words_after_an_eight_byte_body_size() {
        let f = frame_setting(SETTING_IQ_FREQUENCY, 14_070_000);
        assert_eq!(&f[0..4], &2u32.to_le_bytes(), "command type SET_SETTING");
        assert_eq!(&f[4..8], &8u32.to_le_bytes());
        assert_eq!(&f[8..12], &101u32.to_le_bytes());
        assert_eq!(&f[12..16], &14_070_000u32.to_le_bytes());
    }

    #[test]
    fn a_message_header_splits_the_type_word_into_kind_and_gain_flags() {
        let mut b = [0u8; MSG_HEADER_LEN];
        b[0..4].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        // Kind 100 (UINT8_IQ) with 20 dB of applied digital gain in the flags.
        b[4..8].copy_from_slice(&(100u32 | (20u32 << 16)).to_le_bytes());
        b[8..12].copy_from_slice(&1u32.to_le_bytes());
        b[12..16].copy_from_slice(&7u32.to_le_bytes());
        b[16..20].copy_from_slice(&256u32.to_le_bytes());

        let h = MessageHeader::parse(&b).expect("a well-formed header");
        assert_eq!(h.kind, MSG_UINT8_IQ);
        assert_eq!(h.flags, 20);
        assert_eq!(h.sequence, 7);
        assert_eq!(h.body_size, 256);
        assert!((h.digital_gain() - 10.0).abs() < 1e-4, "20 dB is a factor of ten");
    }

    #[test]
    fn a_body_size_past_the_protocol_limit_is_refused() {
        let mut b = [0u8; MSG_HEADER_LEN];
        b[0..4].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());
        b[16..20].copy_from_slice(&(MAX_MESSAGE_BODY_SIZE + 1).to_le_bytes());
        let e = MessageHeader::parse(&b).expect_err("past the limit");
        assert!(e.to_string().contains("out of step"), "{e}");
    }

    #[test]
    fn a_foreign_protocol_major_is_refused() {
        let mut b = [0u8; MSG_HEADER_LEN];
        b[0..4].copy_from_slice(&((3u32 << 24) | 1700).to_le_bytes());
        assert!(MessageHeader::parse(&b).is_err());
    }

    /// The ladder is the server's, and the VFO interface drops its bottom rung.
    #[test]
    fn the_rate_ladder_starts_at_the_servers_own_floor() {
        let r2 = airspy_r2();
        let wide = r2.iq_rates(false);
        assert_eq!(wide.first(), Some(&(2, 2_500_000.0)), "10 Msps / 2^2");
        assert_eq!(wide.last(), Some(&(8, 39_062.5)), "10 Msps / 2^8");

        let vfo = r2.iq_rates(true);
        assert_eq!(vfo.last(), Some(&(7, 78_125.0)), "the VFO list drops the last stage");
        assert_eq!(vfo.len(), wide.len() - 1);

        // A server with no floor starts at stage 0, i.e. its full rate.
        assert_eq!(hf_plus().iq_rates(false).first(), Some(&(0, 768_000.0)));
    }

    /// The label and the span differ whenever analog bandwidth is not the
    /// sample rate, and the strip's axis wants the span.
    #[test]
    fn fft_stages_report_the_delivered_span_separately_from_the_label() {
        let mut info = airspy_r2();
        info.maximum_bandwidth = 9_000_000;
        let stages = info.fft_stages();
        assert_eq!(stages[0], (0, 10_000_000.0, 9_000_000.0));
        assert_eq!(stages[1], (1, 5_000_000.0, 4_500_000.0));
        assert_eq!(stages.len(), 9, "stage 0 through the count itself");
    }

    #[test]
    fn automatic_decimation_picks_the_stage_at_or_below_the_target() {
        let r2 = airspy_r2();
        // 10 Msps ladder: 2.5M, 1.25M, 625k, 312.5k, 156.25k, 78.125k, 39.06k.
        assert_eq!(r2.stage_for_rate(1_000_000.0, false), 4, "625 kHz, not 1.25 M");
        assert_eq!(r2.stage_for_rate(96_000.0, true), 7, "78.125 kHz");
        // A target above everything on offer settles for the widest there is.
        assert_eq!(r2.stage_for_rate(50e6, false), 2);
        // A target below everything takes the narrowest.
        assert_eq!(r2.stage_for_rate(1.0, false), 8);
    }

    /// Each branch of the formula, which is the reference client's and no more.
    #[test]
    fn the_digital_gain_formula_matches_the_reference() {
        // Airspy R2: the index counts down from the maximum.
        let r2 = airspy_r2();
        assert!((r2.digital_gain_db(21, 0) - 0.0).abs() < 1e-9, "full gain needs no help");
        assert!((r2.digital_gain_db(0, 0) - 21.0).abs() < 1e-9);
        assert!((r2.digital_gain_db(21, 4) - 12.04).abs() < 1e-9, "four stages of 3.01 dB");

        // HF+: decimation only. The reference client gives it nothing else,
        // and neither does this.
        assert!((hf_plus().digital_gain_db(4, 3) - 9.03).abs() < 1e-9);

        // RTL-SDR: the same.
        assert!((rtlsdr().digital_gain_db(28, 2) - 6.02).abs() < 1e-9);
    }

    /// Nothing about the receiver's own description moves the figure: not the
    /// ADC width it reports, not the gain index on a receiver that has none. A
    /// constant boost for an HF+'s 8-bit stream was tried and reverted — it
    /// clipped the quantiser flat on every band that had a signal on it, and
    /// the symptom, identical levels on every band, reads as a dead receiver
    /// rather than an overdriven one. Whatever an HF+ needs on a quiet band is
    /// the closed loop's to find, not this table's to guess.
    #[test]
    fn an_hf_plus_opens_at_decimation_gain_alone_whatever_the_server_says_about_itself() {
        for resolution in [8, 16, 18] {
            let hf = DeviceInfo { resolution, ..hf_plus() };
            assert!(
                hf.digital_gain_db(0, 0).abs() < 1e-9,
                "an HF+ reporting {resolution}-bit opens at nothing at stage 0"
            );
            assert!(
                (hf.digital_gain_db(0, 4) - 12.04).abs() < 1e-9,
                "and at four stages of 3.01 dB at stage 4"
            );
        }
    }

    /// The slack is what lets the window sit off the device centre, and it is
    /// zero exactly when the window already covers everything.
    #[test]
    fn the_slack_is_the_room_left_over_after_the_window() {
        let r2 = airspy_r2();
        assert_eq!(r2.iq_slack_hz(10_000_000.0), 0.0, "a full-rate window cannot slide");
        assert_eq!(r2.iq_slack_hz(2_500_000.0), 3_750_000.0);
        assert_eq!(r2.fft_slack_hz(10_000_000.0), 0.0);
        // A window somehow wider than the device is not negative slack.
        assert_eq!(r2.iq_slack_hz(20_000_000.0), 0.0);
    }

    /// The room the I/Q window has is the analog passband's, not the sample
    /// rate's. On a receiver where the two differ that is the whole difference:
    /// a window sat as far out as the rate allows is a window half in the
    /// roll-off, receiving a band that is quiet for reasons the operator cannot
    /// see.
    #[test]
    fn the_iq_slack_stops_where_the_passband_does_not_where_the_sampling_does() {
        let hf = hf_plus();
        assert_eq!(hf.usable_bandwidth_hz(), 660_000.0);
        // Stage 1: 384 ksps inside 660 kHz of passband, not inside 768 kHz of
        // sample rate. The 54 kHz between the two answers is the roll-off.
        assert_eq!(hf.iq_slack_hz(384_000.0), 138_000.0);
        // At the top of the ladder the window is wider than the passband, and
        // wider than the receiver is not negative slack.
        assert_eq!(hf.iq_slack_hz(768_000.0), 0.0);
        // A receiver whose passband is its sample rate is untouched by any of
        // this, which is most of them.
        assert_eq!(airspy_r2().iq_slack_hz(2_500_000.0), 3_750_000.0);
    }

    /// A server that states no bandwidth must not read as a receiver that hears
    /// nothing: that would pin the window to the device centre and refuse the
    /// first tune, on exactly the servers with the least to say for themselves.
    /// One claiming more than it samples is the same nonsense reversed.
    #[test]
    fn a_server_that_states_no_bandwidth_is_taken_at_its_sample_rate() {
        let silent = DeviceInfo { maximum_bandwidth: 0, ..hf_plus() };
        assert_eq!(silent.usable_bandwidth_hz(), 768_000.0);
        assert_eq!(silent.iq_slack_hz(96_000.0), 336_000.0, "exactly as it was before");

        let boastful = DeviceInfo { maximum_bandwidth: 5_000_000, ..hf_plus() };
        assert_eq!(boastful.usable_bandwidth_hz(), 768_000.0, "nothing unsampled is heard");
    }

    /// The corner the two figures made, and the reason the narrower one has to
    /// be the one used. On a controlled HF+ at stage 1 the hysteresis is 231 kHz
    /// and the sample rate claimed the I/Q could follow the dial for 192 of
    /// them, so the strip held still while the dial walked out past the 138 kHz
    /// the passband actually reaches — the hold being the narrower of the two is
    /// only a safe rule while both figures are true.
    #[test]
    fn a_controlled_receivers_hold_stops_where_its_passband_does() {
        let info = hf_plus();
        let span = f64::from(info.maximum_bandwidth);
        let slack = info.iq_slack_hz(384_000.0);
        assert_eq!(slack, 138_000.0);
        let fft = 7_100_000.0;
        let hold = |iq: f64| info.fft_recenter(iq, fft, span, 100_000_000.0, true, slack);

        // Inside what the I/Q window can reach, the strip is free to hold.
        assert_eq!(hold(fft + 130_000.0), None, "the I/Q window can still be placed there");
        // Past it the window moves, and the receiver moves with it — rather
        // than the dial being left where the I/Q window cannot follow.
        assert_eq!(hold(fft + 150_000.0), Some(fft + 150_000.0));
        // 192 kHz is what the sample rate used to allow, and it sits inside the
        // hysteresis' own 231 kHz — so nothing else would have caught it.
        assert_eq!(hold(fft + 192_000.0), Some(fft + 192_000.0), "the gap this closes");
    }

    #[test]
    fn a_forced_int24_server_is_refused_by_name() {
        let mut info = hf_plus();
        info.forced_iq_format = FORMAT_INT24;
        let e = forced_format(&info).expect_err("24-bit is not decodable here");
        assert!(e.to_string().contains("24-bit"), "{e}");

        info.forced_iq_format = FORMAT_INT16;
        assert_eq!(forced_format(&info).unwrap(), Some(SpyServerFormat::Int16));

        info.forced_iq_format = 0;
        assert_eq!(forced_format(&info).unwrap(), None);
    }

    /// A negative or absurd frequency must clamp, not wrap: a wrapped `u32`
    /// would send the receiver to somewhere near 4 GHz.
    #[test]
    fn a_frequency_clamps_rather_than_wrapping() {
        assert_eq!(freq_wire(-1.0), 0);
        assert_eq!(freq_wire(14_070_000.4), 14_070_000);
        assert_eq!(freq_wire(1e12), u32::MAX);
    }

    /// On a *shared* server the band view holds while the dial roams inside it,
    /// and only jumps when the dial reaches the outer third — which is what
    /// makes it a picture to tune across rather than one that slides on every
    /// nudge. The receiver belongs to another client and does not move, so
    /// holding the window costs nothing.
    #[test]
    fn a_shared_servers_fft_window_holds_until_the_dial_reaches_its_edge() {
        let mut info = airspy_r2();
        info.maximum_bandwidth = 10_000_000;
        // A 2.5 MHz window (stage 2) inside a 10 MHz receiver: 3.75 MHz of
        // slack either side of the device centre.
        let span = 2_500_000.0;
        let dc = 100_000_000.0;
        let fft = 100_000_000.0;
        let held = |iq: f64, from: f64| info.fft_recenter(iq, from, span, dc, false, 0.0);

        // Anywhere in the middle 70 % — ±875 kHz — and it does not move.
        assert_eq!(held(dc, fft), None);
        assert_eq!(held(dc + 800_000.0, fft), None);
        assert_eq!(held(dc - 800_000.0, fft), None);

        // Past it, and the window re-centres on the dial.
        let moved = held(dc + 1_000_000.0, fft).expect("past the edge");
        assert_eq!(moved, dc + 1_000_000.0);

        // And having moved, it holds again around the new centre.
        assert_eq!(held(dc + 1_100_000.0, moved), None);

        // The slack is a hard limit: the window cannot leave the receiver.
        let far = held(dc + 9_000_000.0, fft).expect("past the edge");
        assert_eq!(far, dc + 3_750_000.0, "clamped to the device's own bandwidth");
    }

    /// A window already covering the whole of *another client's* receiver has
    /// nowhere to go, so the hysteresis never fires — it just stays on their
    /// centre.
    #[test]
    fn a_full_span_fft_window_is_pinned_to_a_shared_device_centre() {
        let info = airspy_r2();
        let span = f64::from(info.maximum_bandwidth);
        let dc = 100_000_000.0;
        assert_eq!(info.fft_slack_hz(span), 0.0);
        // Far off centre, so the hysteresis says "move" — and the clamp puts it
        // straight back where it was, which the caller reads as no change.
        assert_eq!(info.fft_recenter(dc + 4_000_000.0, dc, span, dc, false, 0.0), Some(dc));
        // A span of zero is the FFT lane switched off.
        assert_eq!(info.fft_recenter(dc, dc, 0.0, dc, false, 0.0), None);
    }

    /// On a server this client controls, the window *is* the receiver, and the
    /// dial may only stray as far as the I/Q window can follow it. At the top of
    /// the rate ladder that is nowhere, so the window tracks the dial exactly —
    /// and the device centre it is handed is stale by definition, so it must
    /// have no say in the answer.
    #[test]
    fn a_controlled_receivers_fft_window_follows_the_dial() {
        let info = hf_plus();
        // Stage 0: the window is the whole receiver, and the I/Q at full rate
        // has no room to slide inside it.
        let span = f64::from(info.maximum_bandwidth);
        let iq_slack = info.iq_slack_hz(f64::from(info.maximum_sample_rate));
        assert_eq!(iq_slack, 0.0);

        // The server was left on 100 MHz; the operator wants 7.1. The window
        // goes to the dial, not to where the server happened to be.
        let stale = 100_000_000.0;
        assert_eq!(
            info.fft_recenter(7_100_000.0, stale, span, stale, true, iq_slack),
            Some(7_100_000.0),
        );
        // Once there, it holds — an unchanged dial is not a reason to retune.
        assert_eq!(info.fft_recenter(7_100_000.0, 7_100_000.0, span, stale, true, iq_slack), None,);
        // And a dial move of any size takes it along, because the I/Q window
        // cannot reach a hertz on its own.
        assert_eq!(
            info.fft_recenter(7_101_000.0, 7_100_000.0, span, stale, true, iq_slack),
            Some(7_101_000.0),
        );
    }

    /// With room for the I/Q window to slide, the hysteresis comes back: the
    /// strip holds still while the dial roams whatever the I/Q can still reach.
    #[test]
    fn a_controlled_receiver_holds_the_window_while_the_iq_can_still_reach() {
        let info = hf_plus();
        let span = f64::from(info.maximum_bandwidth);
        // I/Q at stage 3 — 96 ksps inside 660 kHz of passband — leaves 282 kHz
        // either side.
        let iq_slack = info.iq_slack_hz(96_000.0);
        assert_eq!(iq_slack, 282_000.0);
        let fft = 7_100_000.0;
        let hold = |iq: f64| info.fft_recenter(iq, fft, span, 100_000_000.0, true, iq_slack);

        // The hysteresis is the narrower of the two here: 70 % of half the
        // 660 kHz span, so ±231 kHz against the I/Q window's 282 kHz.
        assert_eq!(hold(fft + 200_000.0), None, "still well inside the strip");
        assert_eq!(hold(fft + 300_000.0), Some(fft + 300_000.0), "past the strip's edge");

        // And when the I/Q slack is the narrower of the two, it is what bites.
        let tight = 100_000.0;
        assert_eq!(
            info.fft_recenter(fft + 150_000.0, fft, span, 100_000_000.0, true, tight),
            Some(fft + 150_000.0),
            "the I/Q window cannot reach 150 kHz with 100 kHz of slack",
        );
    }

    #[test]
    fn the_fft_window_is_clamped_before_it_is_sent() {
        let (off, range) = clamp_fft_window(500.0, 1000.0);
        assert_eq!(off, SpyServerConfig::FFT_DB_OFFSET_MAX);
        assert_eq!(range, SpyServerConfig::FFT_DB_RANGE_MAX);
        let (off, range) = clamp_fft_window(-500.0, 0.0);
        assert_eq!(off, SpyServerConfig::FFT_DB_OFFSET_MIN);
        assert_eq!(range, SpyServerConfig::FFT_DB_RANGE_MIN);
    }
}
