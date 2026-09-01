//! The two views the panel draws: what was said, and who is out there.
//!
//! A decoded AVLC frame becomes a [`Vdl2Message`] in a bounded log and updates a
//! [`Vdl2Station`] row keyed on the sender's 24-bit address. Both are re-sent
//! whole a couple of times a second, so a dropped snapshot costs nothing.
//!
//! # Why the payload is parsed here and not in the decoder
//!
//! [`crate::channel`] hands over an AVLC frame and stops. Whether the payload is
//! ACARS, an XID exchange or something this pass does not read is a question
//! about *content*, and it needs one thing the link layer does not have: which
//! way the frame is going. ACARS lays out its fields differently on an uplink
//! and a downlink, and nothing inside the block says which it is — the address
//! types do.

use std::collections::HashMap;

use sdroxide_types::{
    VDL2_MESSAGE_MAX, VDL2_STATION_MAX, Vdl2AddrKind, Vdl2Frame, Vdl2Message, Vdl2Payload,
    Vdl2Settings, Vdl2Station,
};

use crate::channel::Decoded;
use crate::{acars, avlc, xid};

/// The log and the station table.
pub struct Tracker {
    cfg: Vdl2Settings,
    stations: HashMap<u32, Vdl2Station>,
    messages: Vec<Vdl2Message>,
}

impl Tracker {
    pub fn new(cfg: Vdl2Settings) -> Tracker {
        Tracker { cfg: cfg.sane(), stations: HashMap::new(), messages: Vec::new() }
    }

    pub fn set_config(&mut self, cfg: Vdl2Settings) {
        self.cfg = cfg.sane();
        self.trim();
    }

    /// Fold one decoded frame into both views.
    pub fn absorb(&mut self, d: &Decoded, now: i64) {
        let msg = message(d, now);

        // The sender's row. The destination is deliberately *not* given one: a
        // frame addressed to an aircraft is no evidence that the aircraft is
        // within range, and a list that filled up with stations nobody here has
        // ever heard would be worse than no list.
        let kind = msg.src_kind;
        let entry =
            self.stations.entry(msg.src).or_insert_with(|| Vdl2Station::new(msg.src, kind, now));
        entry.kind = kind;
        entry.last_at = now;
        entry.messages = entry.messages.saturating_add(1);
        entry.last_freq_hz = msg.freq_hz;
        entry.last_snr_db = msg.snr_db;
        match &msg.payload {
            Vdl2Payload::Acars(a) => {
                // The registration and the flight in an ACARS block name the
                // *aircraft*, whichever end sent the block. On an uplink the
                // sender is the ground station, and copying them onto its row
                // would label a ground station with an aeroplane's markings —
                // which is what the synthetic recording caught this doing.
                if kind == Vdl2AddrKind::Aircraft {
                    if !a.registration.trim().is_empty() {
                        entry.registration = a.registration.trim().to_string();
                    }
                    if !a.flight.trim().is_empty() {
                        entry.flight = a.flight.trim().to_string();
                    }
                }
                entry.last_label = a.label.clone();
            }
            Vdl2Payload::Xid(x) => {
                if let (Some(lat), Some(lon)) = (x.lat, x.lon) {
                    entry.lat = Some(lat);
                    entry.lon = Some(lon);
                }
                entry.last_label = x.kind.clone();
            }
            _ => entry.last_label = msg.frame.label(),
        }

        self.messages.push(msg);
        self.trim();
    }

    /// Drop stations nothing has been heard from for a while.
    pub fn expire(&mut self, now: i64) {
        let window = i64::from(self.cfg.drop_list_s);
        self.stations.retain(|_, s| now - s.last_at <= window);
    }

    /// The station table, in no particular order — the panel sorts.
    pub fn stations(&self) -> Vec<Vdl2Station> {
        self.stations.values().cloned().collect()
    }

    /// The message log, oldest first.
    pub fn messages(&self) -> Vec<Vdl2Message> {
        self.messages.clone()
    }

    fn trim(&mut self) {
        let max = (self.cfg.max_messages as usize).min(VDL2_MESSAGE_MAX as usize);
        if self.messages.len() > max {
            let drop = self.messages.len() - max;
            self.messages.drain(..drop);
        }
        let max = (self.cfg.max_stations as usize).min(VDL2_STATION_MAX as usize);
        while self.stations.len() > max {
            // The longest silent goes first, which is the same rule the drop
            // age applies, only forced early.
            let Some(&oldest) = self.stations.iter().min_by_key(|(_, s)| s.last_at).map(|(k, _)| k)
            else {
                break;
            };
            self.stations.remove(&oldest);
        }
    }
}

/// Turn one decoded frame into a log entry, payload and all.
pub fn message(d: &Decoded, now: i64) -> Vdl2Message {
    let f = &d.frame;
    // ACARS lays out an uplink and a downlink differently and says nothing about
    // which it is; the address types do.
    let downlink = f.src.kind == Vdl2AddrKind::Aircraft;
    let pf = matches!(
        f.control,
        Vdl2Frame::Xid { pf: true } | Vdl2Frame::Ui { pf: true } | Vdl2Frame::I { p: true, .. }
    );
    let payload = if f.payload.is_empty() {
        Vdl2Payload::None
    } else if let Some(a) = acars::parse(&f.payload, downlink) {
        Vdl2Payload::Acars(a)
    } else if matches!(f.control, Vdl2Frame::Xid { .. }) {
        match xid::parse(&f.payload, f.dst.cr, pf) {
            Some(x) => Vdl2Payload::Xid(Box::new(x)),
            None => other(f.control, &f.payload),
        }
    } else {
        other(f.control, &f.payload)
    };

    Vdl2Message {
        at: now,
        freq_hz: d.center_hz,
        src: f.src.addr,
        src_kind: f.src.kind,
        dst: f.dst.addr,
        dst_kind: f.dst.kind,
        command: f.dst.cr,
        frame: f.control,
        payload,
        snr_db: d.snr_db,
        rssi_dbfs: d.rssi_dbfs,
        evm_deg: d.evm_deg,
        freq_err_hz: d.freq_err_hz,
        rs_corrected: d.rs_corrected.min(u16::MAX as usize) as u16,
        raw_hex: hex(&d.raw),
    }
}

fn other(control: Vdl2Frame, payload: &[u8]) -> Vdl2Payload {
    Vdl2Payload::Other { note: avlc::describe_payload(control, payload), hex: hex(payload) }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;
    use sdroxide_types::Vdl2Acars;

    fn decoded(frame: avlc::Frame, raw: Vec<u8>) -> Decoded {
        Decoded {
            frame,
            center_hz: 136_975_000.0,
            snr_db: 20.0,
            rssi_dbfs: -30.0,
            rs_corrected: 0,
            evm_deg: 4.0,
            freq_err_hz: 12.0,
            raw,
        }
    }

    fn frame(src: u32, src_kind: Vdl2AddrKind, control: Vdl2Frame, payload: Vec<u8>) -> Decoded {
        let f = avlc::Frame {
            dst: avlc::Address { addr: 0x10_00_01, kind: Vdl2AddrKind::GroundAdmin, cr: true },
            src: avlc::Address { addr: src, kind: src_kind, cr: false },
            control,
            payload,
        };
        decoded(f, vec![0xde, 0xad])
    }

    /// An ACARS downlink fills in the registration and the flight, and a
    /// station that has only ever sent link control does not pretend to have.
    #[test]
    fn acars_is_what_names_a_station() {
        let a = Vdl2Acars {
            mode: '2',
            registration: "OE-LWA".to_string(),
            label: "H1".to_string(),
            block_id: '1',
            msn: "M01A".to_string(),
            flight: "AUA123".to_string(),
            text: "hello".to_string(),
            ..Vdl2Acars::default()
        };
        let mut t = Tracker::new(Vdl2Settings::default());
        t.absorb(
            &frame(
                0x44_0F_31,
                Vdl2AddrKind::Aircraft,
                Vdl2Frame::Ui { pf: false },
                acars::build(&a, true),
            ),
            1000,
        );
        let s = t.stations();
        assert_eq!(s.len(), 1, "only the sender gets a row");
        assert_eq!(s[0].addr, 0x44_0F_31);
        assert_eq!(s[0].registration, "OE-LWA");
        assert_eq!(s[0].flight, "AUA123");
        assert_eq!(s[0].label(), "AUA123");
        assert_eq!(t.messages().len(), 1);
        assert!(matches!(t.messages()[0].payload, Vdl2Payload::Acars(_)));
    }

    /// A frame this pass cannot read is still a frame between two stations, and
    /// is named as far as its first octets allow.
    #[test]
    fn an_unread_payload_is_named_rather_than_dropped() {
        let mut t = Tracker::new(Vdl2Settings::default());
        t.absorb(
            &frame(
                0x44_0F_31,
                Vdl2AddrKind::Aircraft,
                Vdl2Frame::I { ns: 1, nr: 2, p: false },
                vec![0x81, 0x00, 0x11, 0x22],
            ),
            1000,
        );
        let m = &t.messages()[0];
        match &m.payload {
            Vdl2Payload::Other { note, hex } => {
                assert!(note.contains("CLNP"), "{note}");
                assert!(hex.contains("81"));
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(m.summary(), "CLNP, 4 octets");
    }

    /// A ground station is not labelled with the aeroplane it is talking to.
    ///
    /// An uplink's ACARS block carries the *aircraft's* registration, and the
    /// sender is the ground station: copying one onto the other put an airline
    /// registration on a ground station's row.
    #[test]
    fn an_uplink_does_not_name_the_ground_station_after_the_aircraft() {
        let a = Vdl2Acars {
            mode: '2',
            registration: "OE-LWA".to_string(),
            label: "_d".to_string(),
            block_id: '1',
            ..Vdl2Acars::default()
        };
        let mut t = Tracker::new(Vdl2Settings::default());
        t.absorb(
            &frame(
                0x10_20_31,
                Vdl2AddrKind::GroundAdmin,
                Vdl2Frame::I { ns: 0, nr: 1, p: false },
                acars::build(&a, false),
            ),
            1000,
        );
        let s = &t.stations()[0];
        assert!(s.registration.is_empty(), "the ground station took the aircraft's markings");
        assert_eq!(s.label(), "102031");
        assert_eq!(s.last_label, "_d", "the label is still worth showing");
    }

    /// A supervisory frame has no payload, and its row still reads as something.
    #[test]
    fn a_supervisory_frame_is_a_row_with_no_payload() {
        let mut t = Tracker::new(Vdl2Settings::default());
        t.absorb(
            &frame(
                0x10_00_02,
                Vdl2AddrKind::GroundAdmin,
                Vdl2Frame::Rr { nr: 3, pf: false },
                vec![],
            ),
            1000,
        );
        assert!(matches!(t.messages()[0].payload, Vdl2Payload::None));
        assert_eq!(t.stations()[0].last_label, "RR 3");
    }

    /// The log is bounded and the station table is bounded, and the oldest go
    /// first in both.
    #[test]
    fn both_views_are_bounded() {
        // Ten is the floor `Vdl2Settings::sane` allows, so asking for five gets
        // ten — which is itself worth pinning, since a hand-edited config that
        // asked for one would otherwise leave the panel with nothing to draw.
        let cfg = Vdl2Settings { max_messages: 10, max_stations: 5, ..Vdl2Settings::default() };
        let mut t = Tracker::new(cfg);
        for i in 0..50u32 {
            t.absorb(
                &frame(i, Vdl2AddrKind::Aircraft, Vdl2Frame::Ui { pf: false }, vec![]),
                1000 + i64::from(i),
            );
        }
        assert_eq!(t.messages().len(), 10);
        assert_eq!(t.stations().len(), 10);
        // The survivors are the most recent.
        let mut addrs: Vec<u32> = t.stations().iter().map(|s| s.addr).collect();
        addrs.sort_unstable();
        assert_eq!(addrs, (40..50).collect::<Vec<u32>>());
    }

    /// A station goes quiet and leaves the list; one still talking stays.
    #[test]
    fn a_silent_station_leaves_the_list() {
        let cfg = Vdl2Settings { drop_list_s: 60, ..Vdl2Settings::default() };
        let mut t = Tracker::new(cfg);
        t.absorb(&frame(1, Vdl2AddrKind::Aircraft, Vdl2Frame::Ui { pf: false }, vec![]), 1000);
        t.absorb(&frame(2, Vdl2AddrKind::Aircraft, Vdl2Frame::Ui { pf: false }, vec![]), 1100);
        t.expire(1120);
        let addrs: Vec<u32> = t.stations().iter().map(|s| s.addr).collect();
        assert_eq!(addrs, vec![2]);
    }
}
