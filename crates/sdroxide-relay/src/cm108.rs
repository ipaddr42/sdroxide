//! The GPIO pins of a C-Media CM108/CM119 sound card.
//!
//! By a wide margin the most common transmit-switching interface in amateur
//! radio, because every cheap USB "rig interface" is one: the DRA boards, the
//! RB-USB RIM, the AIOC, and a great many home-made ones with a wire tacked to
//! pin 13. If a station has anything that keys a radio from a computer, this is
//! most likely what it is.
//!
//! Using it for a T/R switch rather than for PTT is the same trick pointed at
//! the antenna, and it is worth saying why that is safe: the GPIO pins are on
//! the HID interface, and the audio the card is carrying for the rig is on a
//! different one. Driving a pin does not disturb the audio.
//!
//! # One pin, one contact
//!
//! The chip has eight GPIOs but the boards bring out one, and which one is
//! settled: pin 3, because it is at the end of the package and a wire can be
//! soldered to it by hand. The rest are left as *inputs* by the direction mask
//! on every write, so a card whose other pins are wired to something — a COS
//! line, a squelch input — is not disturbed either.

use std::time::Duration;

use sdroxide_types::{RelayDevice, RelayLink};

use crate::error::Result;
use crate::frame::{ChannelMask, cm108};
use crate::hid::{self, HidDev};
use crate::transport::RelayTransport;

pub struct Cm108Transport {
    dev: Box<dyn HidDev>,
    key: String,
    name: String,
    pin: u8,
    last: Option<bool>,
}

impl Cm108Transport {
    pub fn open(key: &str, pin: u8) -> Result<Cm108Transport> {
        let name = hid::enumerate(cm108::USB_IDS)
            .into_iter()
            .find(|e| e.key == key)
            .map(|e| e.name)
            .unwrap_or_default();
        let dev = hid::open(key)?;
        Ok(Cm108Transport {
            dev,
            key: key.to_string(),
            name,
            pin: if (1..=8).contains(&pin) { pin } else { cm108::DEFAULT_PIN },
            last: None,
        })
    }
}

impl RelayTransport for Cm108Transport {
    fn apply(&mut self, want: ChannelMask) -> Result<()> {
        // One pin, so one bit of state whatever the channel table says.
        let on = want != 0;
        if self.last == Some(on) {
            return Ok(());
        }
        let report = cm108::set_pin(self.pin, on);
        // The report the field-proven implementations write is the whole
        // five-byte buffer with the report id in byte 0 — so the body is the
        // four bytes after it.
        if let Err(e) = self.dev.write_output(report[0], &report[1..]) {
            self.last = None;
            return Err(e);
        }
        self.last = Some(on);
        Ok(())
    }

    fn round_trip(&self) -> Duration {
        // An interrupt-out transfer on a full-speed device: one bus frame.
        Duration::from_millis(2)
    }

    fn describe(&self) -> String {
        let what = if self.name.is_empty() { "sound-card GPIO" } else { self.name.as_str() };
        format!("{what} pin {} at {}", self.pin, self.key)
    }
}

/// The sound cards on this machine whose chips are known to have usable GPIO.
///
/// A list of *candidates*, not of working interfaces: several clones carry a
/// genuine C-Media id and ignore the GPIO report entirely, and nothing short of
/// listening for the click can tell them apart. The settings panel says so.
pub fn list() -> Vec<RelayDevice> {
    hid::enumerate(cm108::USB_IDS)
        .into_iter()
        .map(|e| RelayDevice {
            label: if e.name.is_empty() {
                format!("sound card {:04x}:{:04x} ({})", e.vendor, e.product, e.key)
            } else {
                format!("{} ({})", e.name, e.key)
            },
            key: e.key,
            link: RelayLink::Cm108,
            // One brought-out pin, on every board anyone has wired.
            channels: 1,
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// What the card was told: a report id and a body, in order.
    type Sent = Arc<Mutex<Vec<(u8, Vec<u8>)>>>;

    struct FakeHid {
        sent: Sent,
    }

    impl HidDev for FakeHid {
        fn set_feature(&mut self, _id: u8, _body: &[u8]) -> Result<()> {
            unreachable!("a sound card's GPIO is driven with an output report")
        }
        fn get_feature(&mut self, _id: u8, _body: &mut [u8]) -> Result<()> {
            unreachable!()
        }
        fn write_output(&mut self, id: u8, body: &[u8]) -> Result<()> {
            self.sent.lock().unwrap().push((id, body.to_vec()));
            Ok(())
        }
    }

    #[test]
    fn the_pin_is_driven_and_the_others_are_left_as_inputs() {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let mut t = Cm108Transport {
            dev: Box::new(FakeHid { sent: Arc::clone(&sent) }),
            key: "fake".into(),
            name: String::new(),
            pin: 3,
            last: None,
        };
        t.apply(1).unwrap();
        t.apply(0).unwrap();
        // Report id 0, then the four-byte report: pin 3 is bit 2 of both the
        // data and the direction mask, and every other pin stays an input.
        assert_eq!(
            *sent.lock().unwrap(),
            vec![(0, vec![0, 0x04, 0x04, 0]), (0, vec![0, 0x00, 0x04, 0])]
        );
    }

    #[test]
    fn an_impossible_pin_falls_back_rather_than_driving_nothing() {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let mut t = Cm108Transport {
            dev: Box::new(FakeHid { sent: Arc::clone(&sent) }),
            key: "fake".into(),
            name: String::new(),
            // What `open` does with a 0 or a 9 out of an old config file.
            pin: cm108::DEFAULT_PIN,
            last: None,
        };
        t.apply(1).unwrap();
        assert_eq!(sent.lock().unwrap()[0].1, vec![0, 0x04, 0x04, 0]);
    }
}
