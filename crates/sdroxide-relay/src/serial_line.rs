//! A contact closure on a serial port's own handshake lines, and the transmit
//! sense input on the same cable.
//!
//! The oldest interface in amateur radio and still the most useful one: RTS or
//! DTR through an opto-isolator or a small transistor, driving whatever wants a
//! PTT closure. A DigiRig, a homebrew interface, and every outboard sequencer
//! ever sold — W6PQL, DX Engineering, Array Solutions — take exactly this and
//! nothing else. sdroxide already drives these lines for CAT PTT; this is the
//! same trick pointed at the antenna instead of the radio.
//!
//! # Numbering
//!
//! Contact 1 is RTS and contact 2 is DTR. There is no contact 3: those are the
//! two output lines a USB-serial adapter has.
//!
//! # The sense input
//!
//! The same connector has *input* lines — CTS, DSR, DCD — and this is where
//! they earn their keep. Wire the transceiver's SEND line into one, through an
//! opto-isolator, and a transmitter keyed at its own microphone is seen in
//! milliseconds instead of the few hundred a CAT poll takes. See
//! `sdroxide_types::SenseConfig`; the polling is the driver's, and this only
//! has to read a level cheaply.

use std::time::Duration;

use sdroxide_types::{SenseLine, SerialConfig};

use crate::error::{Error, Result};
use crate::frame::{self, ChannelMask};
use crate::transport::RelayTransport;

/// Contact 1.
const RTS: ChannelMask = 1 << 0;
/// Contact 2.
const DTR: ChannelMask = 1 << 1;

pub struct LineTransport {
    port: Box<dyn serialport::SerialPort>,
    path: String,
    managed: ChannelMask,
    last: Option<ChannelMask>,
    sense: SenseLine,
}

impl LineTransport {
    pub fn open(
        cfg: &SerialConfig,
        managed: ChannelMask,
        sense: SenseLine,
    ) -> Result<LineTransport> {
        let path = cfg.path.trim().to_string();
        if path.is_empty() {
            return Err(Error::Config("no serial port chosen for the T/R switch".into()));
        }
        if managed & !(RTS | DTR) != 0 {
            return Err(Error::Config(
                "a serial RTS/DTR switch has two contacts: 1 is RTS and 2 is DTR".into(),
            ));
        }
        // The baud rate is irrelevant — nothing is ever written — but a port
        // has to be opened at one. The read timeout is what matters: `sense`
        // must never block the driver's tick.
        let port = serialport::new(&path, 9600)
            .timeout(Duration::from_millis(10))
            .open()
            .map_err(|e| Error::opening_serial(&path, e))?;

        // Deliberately *not* asserting a level here. Opening a port raises RTS
        // and DTR on many drivers, and the first `apply` — which the driver
        // makes unconditionally, because it has no agreement with the hardware
        // yet — is what puts both lines where the operator's polarity says they
        // belong. Setting them twice would throw a relay for no reason; setting
        // them here to a guess would throw it to the wrong place first.
        Ok(LineTransport { port, path, managed, last: None, sense })
    }
}

impl RelayTransport for LineTransport {
    fn apply(&mut self, want: ChannelMask) -> Result<()> {
        let want = want & self.managed;
        let changed = match self.last {
            Some(had) => (had ^ want) & self.managed,
            None => self.managed,
        };
        if changed == 0 {
            return Ok(());
        }
        let set = |on: bool, rts: bool, port: &mut dyn serialport::SerialPort| {
            if rts { port.write_request_to_send(on) } else { port.write_data_terminal_ready(on) }
        };
        for (bit, is_rts) in [(RTS, true), (DTR, false)] {
            if changed & bit == 0 {
                continue;
            }
            if let Err(e) = set(want & bit != 0, is_rts, &mut *self.port) {
                self.last = None;
                return Err(e.into());
            }
        }
        self.last = Some(want);
        Ok(())
    }

    fn sense(&mut self) -> Result<Option<bool>> {
        Ok(match self.sense {
            SenseLine::Off => None,
            SenseLine::Cts => Some(self.port.read_clear_to_send()?),
            SenseLine::Dsr => Some(self.port.read_data_set_ready()?),
            SenseLine::Dcd => Some(self.port.read_carrier_detect()?),
        })
    }

    fn round_trip(&self) -> Duration {
        // One `TIOCMSET` on a USB-serial bridge: a control transfer, not a
        // byte on a wire.
        Duration::from_millis(2)
    }

    fn describe(&self) -> String {
        let lines = match (self.managed & RTS != 0, self.managed & DTR != 0) {
            (true, true) => "RTS and DTR",
            (true, false) => "RTS",
            (false, true) => "DTR",
            (false, false) => "no line",
        };
        format!("{lines} on {}", self.path)
    }
}

/// The bit for whichever of the two lines a channel number names, for the
/// caller assembling the managed mask.
pub fn line_bit(channel: u8) -> ChannelMask {
    frame::bit(channel) & (RTS | DTR)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn there_are_two_lines_and_no_more() {
        assert_eq!(line_bit(1), RTS);
        assert_eq!(line_bit(2), DTR);
        assert_eq!(line_bit(3), 0, "a USB-serial adapter has no third output line");
    }
}
