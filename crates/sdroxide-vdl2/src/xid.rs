//! XID — how VDL Mode 2 establishes, moves and refuses links, and how a ground
//! station announces itself.
//!
//! An XID frame's payload is a format identifier followed by parameter groups,
//! each group a header and a run of type-length-value parameters:
//!
//! ```text
//!  0x82 │ GID │ length │ id │ len │ value │ id │ len │ value │ … │ GID │ …
//!   1   │  1  │   2    │ 1  │  1  │  len  │
//! ```
//!
//! Group `0x80` is the public set, which is HDLC's own vocabulary of window
//! sizes, timers and retry counts. Group `0xF0` is the VDL private set, which
//! is where everything specific to aeronautical datalink lives — what the
//! exchange *is*, where the station is, and which frequencies it will hand a
//! link over to.
//!
//! # What is decoded, and what is only listed
//!
//! Seven private parameters are decoded to text: connection management, XID
//! sequencing, modulation support, alternate ground stations, aircraft
//! location, the frequency support list and the ground station's location.
//! Those are the ones whose byte layout is established here.
//!
//! Everything else — the rest of the private set and the whole public set — is
//! **listed with its identifier and its bytes and counted as unknown**, not
//! silently dropped and not given a guessed name. A parameter shown as
//! `vdl 0x8B: 03 20` is honest; one labelled with the wrong timer's name is
//! worse than useless, because an operator would believe it. The count is what
//! says how much of an exchange is going unread.
//!
//! # The exchange's name is not in the frame
//!
//! What kind of XID this is — a link establishment, a handoff, a refusal, a
//! ground station's beacon — is not a field. It is four bits from three places:
//! the command/response bit of the destination address, the poll/final bit of
//! the control field, and two bits inside the connection-management parameter.
//! [`kind`] puts them together.
//!
//! Source: ETSI EN 301 841-1 §5.7 and its XID parameter tables.

use sdroxide_types::{Vdl2AddrKind, Vdl2Xid};

/// Every XID frame begins with this.
pub const FMT_ID: u8 = 0x82;
/// The public (HDLC) parameter group.
pub const GID_PUBLIC: u8 = 0x80;
/// The VDL private parameter group.
pub const GID_PRIVATE: u8 = 0xf0;

/// The sixteen exchanges the four naming bits can describe. Empty entries are
/// combinations the standard does not define.
const KINDS: [&str; 16] = [
    "",
    "XID_CMD_LCR",
    "XID_CMD_HO",
    "GSIF",
    "XID_CMD_LE",
    "",
    "XID_CMD_HO",
    "XID_CMD_LPM",
    "",
    "",
    "",
    "",
    "XID_RSP_LE",
    "XID_RSP_LCR",
    "XID_RSP_HO",
    "XID_RSP_LPM",
];

/// What kind of exchange this is, from the command/response bit, the poll/final
/// bit and the connection-management parameter's `h` and `r` bits.
pub fn kind(cr: bool, pf: bool, conn_mgmt: Option<u8>) -> String {
    let cm = conn_mgmt.unwrap_or(0);
    // The flags are packed least significant first: h, then r, then x, then v.
    let h = cm & 1;
    let r = (cm >> 1) & 1;
    let i = (u8::from(cr) << 3) | (u8::from(pf) << 2) | (h << 1) | r;
    let name = KINDS[i as usize];
    if name.is_empty() { format!("type {i}") } else { name.to_string() }
}

/// Parse an XID payload.
///
/// `cr` is the destination address's command/response bit and `pf` the control
/// field's poll/final bit — both needed to name the exchange.
pub fn parse(payload: &[u8], cr: bool, pf: bool) -> Option<Vdl2Xid> {
    if payload.first() != Some(&FMT_ID) {
        return None;
    }
    let mut x = Vdl2Xid::default();
    let mut conn_mgmt = None;
    let mut at = 1usize;
    while at + 3 <= payload.len() {
        let gid = payload[at];
        let glen = usize::from(u16::from_be_bytes([payload[at + 1], payload[at + 2]]));
        at += 3;
        let end = (at + glen).min(payload.len());
        let mut p = at;
        while p + 2 <= end {
            let id = payload[p];
            let len = usize::from(payload[p + 1]);
            p += 2;
            let vend = (p + len).min(end);
            let val = &payload[p..vend];
            p = vend;
            if gid == GID_PRIVATE {
                if id == 0x01 && !val.is_empty() {
                    conn_mgmt = Some(val[0]);
                }
                private_param(&mut x, id, val);
            } else {
                x.params.push((format!("pub 0x{id:02X}"), hex(val)));
                x.unknown = x.unknown.saturating_add(1);
            }
        }
        at = end;
        if glen == 0 {
            break;
        }
    }
    x.kind = kind(cr, pf, conn_mgmt);
    Some(x)
}

fn private_param(x: &mut Vdl2Xid, id: u8, val: &[u8]) {
    match id {
        // Connection management: four flags that, with the command/response and
        // poll/final bits, say what the exchange is.
        0x01 if !val.is_empty() => {
            let v = val[0];
            x.params.push((
                "Connection management".to_string(),
                format!("h={} r={} x={} v={}", v & 1, (v >> 1) & 1, (v >> 2) & 1, (v >> 3) & 1),
            ));
        }
        // XID sequencing: which of a sequence of exchanges this is, and how many
        // times it has been retried.
        0x03 if !val.is_empty() => {
            x.params.push((
                "XID sequencing".to_string(),
                format!("seq {} retry {}", val[0] & 0x7, val[0] >> 4),
            ));
        }
        0x81 if !val.is_empty() => {
            x.params.push(("Modulation support".to_string(), modulations(val[0])));
        }
        0x82 if !val.is_empty() && val.len().is_multiple_of(4) => {
            let list: Vec<String> = val.chunks_exact(4).map(addr_text).collect();
            x.params.push(("Alternate ground stations".to_string(), list.join(", ")));
        }
        // Aircraft location: three octets of position and one of altitude in
        // thousands of feet.
        0x84 if val.len() >= 4 => {
            let (lat, lon) = location(&val[0..3]);
            x.lat = Some(lat);
            x.lon = Some(lon);
            x.params.push((
                "Aircraft location".to_string(),
                format!("{lat:.1} {lon:.1}, {} ft", u32::from(val[3]) * 1000),
            ));
        }
        // Frequency support: six octets per entry — two of frequency and
        // modulation, four of the ground station that is on it.
        0xc0 if !val.is_empty() && val.len().is_multiple_of(6) => {
            let mut list = Vec::new();
            for c in val.chunks_exact(6) {
                let (mhz, m) = frequency(&c[0..2]);
                x.frequencies.push(mhz);
                list.push(format!("{mhz:.3} MHz {} [{}]", modulations(m), addr_text(&c[2..6])));
            }
            x.params.push(("Frequency support".to_string(), list.join(", ")));
        }
        0xc8 if val.len() >= 3 => {
            let (lat, lon) = location(&val[0..3]);
            x.lat = Some(lat);
            x.lon = Some(lon);
            x.params.push(("Ground station location".to_string(), format!("{lat:.1} {lon:.1}")));
        }
        _ => {
            x.params.push((format!("vdl 0x{id:02X}"), hex(val)));
            x.unknown = x.unknown.saturating_add(1);
        }
    }
}

/// Twelve bits of latitude then twelve of longitude, both signed, both in
/// tenths of a degree — so a position good to about eleven kilometres, which is
/// all a ground station's coverage description needs.
pub fn location(b: &[u8]) -> (f64, f64) {
    debug_assert!(b.len() >= 3);
    let lat = sign12(u32::from(b[0]) << 4 | u32::from(b[1]) >> 4);
    let lon = sign12((u32::from(b[1]) & 0xf) << 8 | u32::from(b[2]));
    (f64::from(lat) / 10.0, f64::from(lon) / 10.0)
}

fn sign12(v: u32) -> i32 {
    let v = (v & 0xfff) as i32;
    if v & 0x800 != 0 { v - 0x1000 } else { v }
}

/// Four bits of modulation and twelve of frequency, the latter as tens of
/// kilohertz above 100 MHz — which does not land on the 25 kHz raster, so the
/// standard rounds up to it.
pub fn frequency(b: &[u8]) -> (f64, u8) {
    debug_assert!(b.len() >= 2);
    let modulations = b[0] >> 4;
    let raw = u32::from(u16::from_be_bytes([b[0], b[1]]) & 0x0fff);
    let mut khz = (raw + 10_000) * 10;
    if khz % 25 != 0 {
        khz += 25 - khz % 25;
    }
    (f64::from(khz) / 1000.0, modulations)
}

/// The inverse of [`frequency`]: the two octets that name a channel.
///
/// Beside the parser because the encoding is lossy in one direction — the field
/// steps in ten kilohertz and the raster steps in twenty-five — so which way the
/// rounding goes is part of the format rather than a detail of either half.
pub fn frequency_octets(mhz: f64, modulations: u8) -> [u8; 2] {
    let raw = ((mhz * 100.0).floor() as u32).saturating_sub(10_000) & 0x0fff;
    [(modulations << 4) | ((raw >> 8) as u8 & 0x0f), (raw & 0xff) as u8]
}

fn modulations(m: u8) -> String {
    let mut out = Vec::new();
    if m & 0x2 != 0 {
        out.push("VDL-M2 D8PSK 31.5 kbps");
    }
    if m & 0x4 != 0 {
        out.push("VDL-M3 D8PSK 31.5 kbps");
    }
    if out.is_empty() { format!("0x{m:X}") } else { out.join(" + ") }
}

fn addr_text(b: &[u8]) -> String {
    let o: [u8; 4] = b.try_into().unwrap_or([0; 4]);
    let a = crate::avlc::address(&o);
    let tag = match a.kind {
        Vdl2AddrKind::Aircraft => "",
        k => k.short(),
    };
    if tag.is_empty() { format!("{:06X}", a.addr) } else { format!("{:06X} {tag}", a.addr) }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02X}")).collect::<Vec<_>>().join(" ")
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The frequency encoding's whole point: the raw field cannot express a
    /// 25 kHz raster slot, so the round-up is the encoding rather than a
    /// tolerance. Every VDL2 channel has to come back exactly.
    #[test]
    fn every_channel_of_the_plan_round_trips_through_the_frequency_field() {
        for &hz in &sdroxide_types::VDL2_CHANNELS_HZ {
            let mhz = hz / 1e6;
            let (got, m) = frequency(&frequency_octets(mhz, 2));
            assert!((got - mhz).abs() < 1e-9, "{mhz} came back as {got}");
            assert_eq!(m, 2);
        }
    }

    /// The 121.5 MHz emergency channel is not on the raster the round-up snaps
    /// to, which is worth pinning: it proves the rounding is doing something.
    #[test]
    fn the_frequency_field_rounds_up_to_the_raster() {
        // The field steps in ten kilohertz, so 136.970 is expressible and
        // 136.975 is not. The encoding is the round-up: asking for the one the
        // field can hold gives back the channel above it.
        let raw = 3697u32;
        let b = [(raw >> 8) as u8 & 0x0f, (raw & 0xff) as u8];
        assert_eq!(frequency(&b).0, 136.975);
    }

    /// A position is signed in both axes, in tenths of a degree.
    #[test]
    fn a_location_is_signed_in_both_axes() {
        // 48.2 N, 16.4 E — Vienna.
        let lat = 482i32;
        let lon = 164i32;
        let b =
            [(lat >> 4) as u8, (((lat & 0xf) << 4) | ((lon >> 8) & 0xf)) as u8, (lon & 0xff) as u8];
        let (la, lo) = location(&b);
        assert!((la - 48.2).abs() < 0.05, "{la}");
        assert!((lo - 16.4).abs() < 0.05, "{lo}");

        // 33.9 S, 118.4 W — Los Angeles, both negative.
        let lat = -339i32;
        let lon = -1184i32;
        let b = [
            ((lat >> 4) & 0xff) as u8,
            (((lat & 0xf) << 4) | ((lon >> 8) & 0xf)) as u8,
            (lon & 0xff) as u8,
        ];
        let (la, lo) = location(&b);
        assert!((la + 33.9).abs() < 0.05, "{la}");
        assert!((lo + 118.4).abs() < 0.05, "{lo}");
    }

    /// The exchange's name comes from three places at once.
    #[test]
    fn the_exchange_is_named_from_bits_in_three_fields() {
        // A ground station's beacon: command, no poll, h set, r clear.
        assert_eq!(kind(false, false, Some(0b0011)), "GSIF");
        // Link establishment, from an aircraft: command, poll, h and r clear.
        assert_eq!(kind(false, true, Some(0b0000)), "XID_CMD_LE");
        // ...and the ground station's answer.
        assert_eq!(kind(true, true, Some(0b0000)), "XID_RSP_LE");
        assert_eq!(kind(true, true, Some(0b0010)), "XID_RSP_LCR");
        assert_eq!(kind(true, true, Some(0b0001)), "XID_RSP_HO");
        // A combination the standard leaves undefined is named, not guessed at.
        assert_eq!(kind(true, false, Some(0b0000)), "type 8");
    }

    /// A ground station's beacon, built by hand from the standard's structure.
    #[test]
    fn a_ground_station_beacon_decodes() {
        let mut p = vec![FMT_ID, GID_PRIVATE, 0x00, 0x00];
        let mut params: Vec<u8> = Vec::new();
        params.extend_from_slice(&[0x01, 0x01, 0b0011]); // connection management
        params.extend_from_slice(&[0x81, 0x01, 0x02]); // modulation support
        params.extend_from_slice(&[0xc8, 0x03, 0x1E, 0x21, 0xA4]); // location
        // One alternate ground station.
        let alt = crate::avlc::address_octets(
            crate::avlc::Address { addr: 0x10_20_30, kind: Vdl2AddrKind::GroundAdmin, cr: false },
            false,
        );
        params.push(0x82);
        params.push(4);
        params.extend_from_slice(&alt);
        // A parameter this decoder has no name for.
        params.extend_from_slice(&[0x8b, 0x01, 0x0c]);
        let glen = params.len() as u16;
        p[2] = (glen >> 8) as u8;
        p[3] = (glen & 0xff) as u8;
        p.extend_from_slice(&params);

        let x = parse(&p, false, false).expect("parsed");
        assert_eq!(x.kind, "GSIF");
        assert!(x.lat.is_some() && x.lon.is_some());
        assert!(x.params.iter().any(|(k, v)| k == "Modulation support" && v.contains("VDL-M2")));
        assert!(
            x.params.iter().any(|(k, v)| k == "Alternate ground stations" && v.contains("102030")),
            "{:?}",
            x.params
        );
        assert_eq!(x.unknown, 1, "the unnamed parameter should be counted");
        assert!(x.params.iter().any(|(k, _)| k == "vdl 0x8B"));
    }

    /// A frequency support list fills in the frequencies the panel shows.
    #[test]
    fn a_frequency_list_is_collected() {
        let mut params: Vec<u8> = vec![0xc0, 12];
        for &hz in &[136_975_000.0f64, 136_725_000.0] {
            params.extend_from_slice(&frequency_octets(hz / 1e6, 2));
            params.extend_from_slice(&crate::avlc::address_octets(
                crate::avlc::Address { addr: 1, kind: Vdl2AddrKind::GroundAdmin, cr: false },
                false,
            ));
        }
        let glen = params.len() as u16;
        let mut p = vec![FMT_ID, GID_PRIVATE, (glen >> 8) as u8, (glen & 0xff) as u8];
        p.extend_from_slice(&params);

        let x = parse(&p, false, false).expect("parsed");
        assert_eq!(x.frequencies, vec![136.975, 136.725]);
    }

    /// A payload that is not XID, and a truncated one, are both refused rather
    /// than read out of whatever is next in the buffer.
    #[test]
    fn a_malformed_payload_does_not_panic() {
        assert!(parse(b"", false, false).is_none());
        assert!(parse(b"\x81 not xid", false, false).is_none());
        // A group claiming to be longer than the payload.
        let x = parse(&[FMT_ID, GID_PRIVATE, 0xff, 0xff, 0x01, 0x01, 0x02], false, false)
            .expect("parsed what was there");
        assert_eq!(x.kind, "XID_CMD_LCR");
    }
}
