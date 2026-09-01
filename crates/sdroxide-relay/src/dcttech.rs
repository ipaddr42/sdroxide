//! The dcttech USB HID relay boards — the "free-driver USB control switch"
//! family, sold under MagiDeal and a dozen other names, 1 to 8 channels.
//!
//! This is the board the feature request named. It is the cheapest way to get a
//! computer-controlled contact closure, it needs no driver on any platform, and
//! it is what a great many stations already have in a drawer.
//!
//! # The shared vendor id
//!
//! Every one of these carries `16c0:05df`, which is a *V-USB hobby id* — a
//! range Objective Development hands out to anyone using their software USB
//! stack. Other people's home-made keyboards, LED controllers and thermometers
//! share it. So the product string is checked as well, and it is the only thing
//! that makes the enumeration safe to run on a stranger's bus.

use std::time::Duration;

use sdroxide_types::{RelayDevice, RelayLink};

use crate::error::{Error, Result};
use crate::frame::{self, ChannelMask, dcttech};
use crate::hid::{self, HidDev};
use crate::transport::RelayTransport;

pub struct DcttechTransport {
    dev: Box<dyn HidDev>,
    key: String,
    name: String,
    managed: ChannelMask,
    last: Option<ChannelMask>,
}

impl DcttechTransport {
    pub fn open(key: &str, managed: ChannelMask) -> Result<DcttechTransport> {
        if managed & !0xFF != 0 {
            return Err(Error::Config("a dcttech relay board has at most eight contacts".into()));
        }
        let name = hid::enumerate(&[dcttech::USB_ID])
            .into_iter()
            .find(|e| e.key == key)
            .map(|e| e.name)
            .unwrap_or_default();
        let dev = hid::open(key)?;
        Ok(DcttechTransport { dev, key: key.to_string(), name, managed, last: None })
    }
}

impl RelayTransport for DcttechTransport {
    fn apply(&mut self, want: ChannelMask) -> Result<()> {
        let want = want & self.managed;
        let changed = match self.last {
            Some(had) => (had ^ want) & self.managed,
            None => self.managed,
        };
        if changed == 0 {
            return Ok(());
        }
        for ch in 1..=8u8 {
            let b = frame::bit(ch);
            if changed & b == 0 {
                continue;
            }
            if let Err(e) = self.dev.set_feature(0, &dcttech::set(ch, want & b != 0)) {
                self.last = None;
                return Err(e);
            }
        }
        self.last = Some(want);
        Ok(())
    }

    fn read_back(&mut self) -> Result<Option<ChannelMask>> {
        let mut body = [0u8; dcttech::REPORT_LEN];
        self.dev.get_feature(0, &mut body)?;
        // Believed only when it looks like an answer.
        //
        // The board puts its five-character serial in the first five bytes and
        // its contacts in byte 7, so a reply whose first five bytes are not
        // printable is one that landed at the wrong offset — which is exactly
        // the failure a platform that keeps the report id in byte 0 would
        // produce, and the one thing in the HID layer no test here can catch.
        // Reporting nothing is correct in that case: a read-back is never
        // load-bearing, and a wrong one would have the driver rewriting
        // contacts that were already right.
        let Some((serial, state)) = dcttech::decode(&body) else { return Ok(None) };
        if serial.len() != 5 || !serial.bytes().all(|b| b.is_ascii_graphic()) {
            tracing::debug!("the relay board's read-back did not look like one: {body:02x?}");
            return Ok(None);
        }
        Ok(Some(ChannelMask::from(state) & self.managed))
    }

    fn round_trip(&self) -> Duration {
        // A USB control transfer per contact, on a full-speed device. One
        // millisecond is the bus frame; the V-USB firmware is the rest.
        Duration::from_millis(5)
    }

    fn describe(&self) -> String {
        if self.name.is_empty() {
            format!("USB HID relay board at {}", self.key)
        } else {
            format!("{} at {}", self.name, self.key)
        }
    }
}

/// The relay boards on this machine.
///
/// Filtered on the product string as well as the ids, for the reason the module
/// docs give: the vendor id is shared with every other V-USB hobby device, and
/// offering somebody's home-made keyboard as an antenna relay would be a poor
/// joke.
pub fn list() -> Vec<RelayDevice> {
    hid::enumerate(&[dcttech::USB_ID])
        .into_iter()
        .filter(|e| e.name.contains(dcttech::PRODUCT_PREFIX))
        .map(|e| {
            // "USBRelay2" — the trailing digit is how many contacts it has, and
            // the only place the board says so.
            let channels = e
                .name
                .rsplit_once(dcttech::PRODUCT_PREFIX)
                .and_then(|(_, n)| n.trim().parse::<u8>().ok())
                .unwrap_or(0);
            RelayDevice {
                label: if e.name.is_empty() { e.key.clone() } else { e.name.clone() },
                key: e.key,
                link: RelayLink::Hid,
                channels,
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// A board that records what it was told and answers as the real one does.
    #[derive(Default)]
    struct FakeHid {
        sent: Arc<Mutex<Vec<Vec<u8>>>>,
        answer: Vec<u8>,
    }

    impl HidDev for FakeHid {
        fn set_feature(&mut self, id: u8, body: &[u8]) -> Result<()> {
            assert_eq!(id, 0, "these boards use report id 0");
            self.sent.lock().unwrap().push(body.to_vec());
            Ok(())
        }
        fn get_feature(&mut self, _id: u8, body: &mut [u8]) -> Result<()> {
            let n = body.len().min(self.answer.len());
            body[..n].copy_from_slice(&self.answer[..n]);
            Ok(())
        }
        fn write_output(&mut self, _id: u8, _body: &[u8]) -> Result<()> {
            unreachable!("a relay board is commanded with feature reports")
        }
    }

    fn transport(answer: Vec<u8>) -> (DcttechTransport, Arc<Mutex<Vec<Vec<u8>>>>) {
        let sent = Arc::new(Mutex::new(Vec::new()));
        let dev = FakeHid { sent: Arc::clone(&sent), answer };
        (
            DcttechTransport {
                dev: Box::new(dev),
                key: "fake".into(),
                name: "USBRelay2".into(),
                managed: 0b11,
                last: None,
            },
            sent,
        )
    }

    #[test]
    fn both_contacts_are_written_first_and_then_only_what_changed() {
        let (mut t, sent) = transport(Vec::new());
        t.apply(0).unwrap();
        assert_eq!(
            *sent.lock().unwrap(),
            vec![dcttech::set(1, false).to_vec(), dcttech::set(2, false).to_vec()]
        );
        sent.lock().unwrap().clear();

        t.apply(0b01).unwrap();
        assert_eq!(*sent.lock().unwrap(), vec![dcttech::set(1, true).to_vec()]);
        sent.lock().unwrap().clear();

        t.apply(0b01).unwrap();
        assert!(sent.lock().unwrap().is_empty(), "an unchanged state is not written");
    }

    #[test]
    fn a_read_back_yields_the_contacts_the_board_reports() {
        let (mut t, _) = transport(vec![b'A', b'B', b'C', b'D', b'E', 0, 0, 0b11]);
        assert_eq!(t.read_back().unwrap(), Some(0b11));
    }

    /// The defence against the one byte the HID layer cannot be tested on.
    #[test]
    fn a_read_back_at_the_wrong_offset_is_reported_as_nothing() {
        // What a platform that left the report id in byte 0 would hand back:
        // the serial shifted along by one.
        let (mut t, _) = transport(vec![0, b'A', b'B', b'C', b'D', b'E', 0, 0]);
        assert_eq!(
            t.read_back().unwrap(),
            None,
            "a reply that is not a serial must not be believed as a contact state"
        );
    }
}
