//! The station's external transmit/receive switch.
//!
//! # What this is
//!
//! An SDR sharing an antenna system with a transceiver has to be got out of the
//! way before the transceiver's RF appears. What hams use for that is a relay —
//! a coax relay that grounds the receiver's input, an outboard T/R switch, the
//! antenna port on an amplifier — and every one of them is driven the same way:
//! a contact closes while transmitting and opens while receiving. The same
//! contact, sequenced a few milliseconds apart, is what keys an amplifier.
//!
//! So this crate is not a driver for a board. It is the machinery that produces
//! that closure, on whatever the operator has: a USB relay board over a serial
//! port ([`serial`]), a handshake line, a USB HID relay, a sound-card GPIO pin,
//! a Raspberry Pi header, or a program they wrote.
//!
//! # Why it is its own crate
//!
//! The same argument `sdroxide-limerfe` makes. The switch is a *station*
//! accessory: it sits in front of whatever this program happens to be receiving
//! with, and belongs to the antenna rather than to any one front end. Keeping
//! it here — pure Rust, no system libraries — means it works in every build
//! variant on every target.
//!
//! # Shape
//!
//! [`frame`] holds the wire with no port in it, which is what makes the fiddly
//! half checkable against a datasheet with nothing plugged in.
//! [`driver::Sequencer`] decides which contacts should be closed and when, and
//! is pure and clock-injected for the same reason — the thing it is timing is a
//! receiver's front end against a kilowatt, and it should not take a
//! transmitter to test it. [`spawn`] puts a transport on a thread, because
//! every one of them blocks for longer than the engine's loop can spare.
//!
//! The configuration vocabulary is *not* here: it lives in `sdroxide-types`, so
//! the settings panel can edit it while compiled to wasm — a remote operator
//! has to be able to set up the switch on the machine the antenna is actually
//! attached to.
//!
//! # What it cannot do
//!
//! When a *separate* transceiver keys itself, this program learns about it by
//! asking over CAT, and that answer lands a few hundred milliseconds into the
//! over. The relay throws then. Wiring the rig's SEND line into a sense input
//! (`sdroxide_types::SenseConfig`) moves that to a few milliseconds, but
//! nothing driven from a computer makes it zero, and a genuinely valuable front
//! end wants an RF-sensed hardware switch regardless. This is said in the
//! settings panel and in the manual, and it is said here because it is the
//! first thing anyone reading this code will want to know.
//!
//! NATIVE ONLY — links `serialport` and opens device nodes; must never be a
//! dependency of any wasm-targeted crate.

pub mod cm108;
pub mod command;
pub mod dcttech;
pub mod driver;
pub mod error;
pub mod frame;
#[cfg(target_os = "linux")]
pub mod gpio;
pub mod hid;
pub mod serial;
pub mod serial_line;
pub mod trace;
pub mod transport;

pub use cm108::Cm108Transport;
pub use command::CommandTransport;
pub use dcttech::DcttechTransport;
pub use driver::{Ctrl, Presence, RelayHandle, Sequencer, Source, spawn};
pub use error::{Error, Result};
pub use frame::ChannelMask;
pub use serial::SerialTransport;
pub use serial_line::LineTransport;
pub use trace::diagnostics;
pub use transport::RelayTransport;

use sdroxide_types::{RelayConfig, RelayDevice, RelayLink};

/// Open the switch this configuration describes.
///
/// `Ok(None)` when none is configured — the ordinary case, and not a failure.
/// A configuration that is switched on but cannot be opened is an error,
/// because the operator asked for something and did not get it.
pub fn open(cfg: &RelayConfig) -> Result<Option<RelayHandle>> {
    if !cfg.enabled() {
        return Ok(None);
    }
    if let Some(why) = cfg.refusal() {
        return Err(Error::Config(why));
    }
    let managed = cfg.active_channels().fold(0, |m, c| m | frame::bit(c.index));
    let transport: Box<dyn RelayTransport> = match cfg.link {
        RelayLink::Off => return Ok(None),
        RelayLink::Serial => {
            Box::new(SerialTransport::open(&cfg.serial, cfg.family, managed, cfg.sense.line)?)
        }
        RelayLink::SerialLines => {
            Box::new(LineTransport::open(&cfg.serial, managed, cfg.sense.line)?)
        }
        RelayLink::Command => Box::new(CommandTransport::new(&cfg.tx_cmd, &cfg.rx_cmd)?),
        RelayLink::Hid => Box::new(DcttechTransport::open(cfg.device.trim(), managed)?),
        RelayLink::Cm108 => Box::new(Cm108Transport::open(cfg.device.trim(), cfg.cm108_pin)?),
        #[cfg(target_os = "linux")]
        RelayLink::Gpio => {
            Box::new(gpio::GpioTransport::open(cfg.device.trim(), &cfg.gpio_lines, managed)?)
        }
        #[cfg(not(target_os = "linux"))]
        RelayLink::Gpio => {
            return Err(Error::Unsupported(
                "GPIO lines are a Linux interface — on this platform use a relay board, an \
                 RTS/DTR line, or the external command hook"
                    .into(),
            ));
        }
    };
    Ok(Some(spawn(transport, cfg.clone())))
}

/// The switching devices this machine can see, for the settings panel's picker.
///
/// Answered on the engine's machine and sent to whichever screen asked, because
/// the antenna is not necessarily where the operator is sitting.
pub fn list() -> Vec<RelayDevice> {
    // Serial ports are not here: they are serial ports, and the settings panel
    // already asks the existing serial-port probe for those. What this answers
    // is the two kinds of device that need a bus walked to be found.
    let mut out = dcttech::list();
    out.extend(cm108::list());
    out
}
