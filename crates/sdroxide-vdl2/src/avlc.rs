//! AVLC — the link layer VDL Mode 2 carries everything in.
//!
//! One frame per burst, inside an ordinary HDLC wrapper: an opening `0x7E`
//! flag, the frame with a zero stuffed after every five ones, and a closing
//! flag. [`crate::hdlc`] takes that off; what is left is what this module
//! parses.
//!
//! The transmission header's length field says how long the *wrapper* is, in
//! bits — which is why it is in bits, since a stuffed frame is not a whole
//! number of octets. This module was written believing the field was the frame
//! itself and that the length made the flags unnecessary. It is not, and they
//! are there: every transmission in the issue #265 recording carries them, and
//! reading the field as a bare frame put every octet one out and failed every
//! check sequence.
//!
//! ```text
//!   4 octets     4 octets     1 octet    0..n octets   2 octets
//! ┌────────────┬────────────┬──────────┬─────────────┬──────────┐
//! │destination │  source    │ control  │   payload   │   FCS    │
//! └────────────┴────────────┴──────────┴─────────────┴──────────┘
//! ```
//!
//! # The address field
//!
//! Twenty-eight bits spread seven to an octet, because the low bit of every
//! octet belongs to HDLC's extended addressing: zero means another address
//! octet follows, one means this was the last. With eight octets of address the
//! only one set is the source's fourth.
//!
//! The twenty-eight bits arrive least-significant first, so they are reversed
//! and then split: twenty-four bits of address, three bits of address type, and
//! one bit that is the command/response flag on the destination address and
//! reserved on the source. For an aircraft the twenty-four bits are its ICAO
//! number — the same one its ADS-B squitters carry, which is the one place this
//! decoder's address arithmetic can be checked against something written by
//! somebody else.
//!
//! An extension bit set where the standard says it is clear is treated as a
//! malformed frame rather than ignored. The bit is not part of the address, so
//! nothing would visibly go wrong; but the Reed-Solomon layer has just declared
//! this frame repaired, and a bit that cannot be set being set says otherwise.
//!
//! # The frame check sequence
//!
//! CRC-16/X.25 — reflected `0x8408`, initialised `0xFFFF`, transmitted low byte
//! first, residue `0xF0B8`. Identical to AX.25's, and taken from
//! [`sdroxide_ax25::fcs`] rather than written again here: that module is tested
//! against catalogue values and a captured frame, and a second implementation
//! would round-trip perfectly against itself while being wrong.
//!
//! Source: ETSI EN 301 841-1 §5, the AVLC frame format.

use sdroxide_types::{Vdl2AddrKind, Vdl2Frame};

/// Destination and source addresses, a control field and a frame check
/// sequence: eleven octets before there is any payload at all.
pub const MIN_LEN: usize = 4 + 4 + 1 + 2;

/// One end of a conversation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Address {
    /// The 24-bit address. For an aircraft, its ICAO number.
    pub addr: u32,
    pub kind: Vdl2AddrKind,
    /// The command/response bit on a destination address; reserved, and
    /// normally clear, on a source address.
    pub cr: bool,
}

/// Read one four-octet address field.
///
/// The extension bits are masked out rather than shifted through, which is what
/// makes [`address_ext_ok`] a separate question: a stray one there cannot
/// corrupt the address here, and is reported instead of silently shifting every
/// bit above it.
pub fn address(b: &[u8; 4]) -> Address {
    let raw = u32::from(b[0] & 0xfe) >> 1
        | (u32::from(b[1] & 0xfe) << 6)
        | (u32::from(b[2] & 0xfe) << 13)
        | (u32::from(b[3] & 0xfe) << 20);
    let v = crate::header::reverse_bits(raw, 28);
    Address {
        addr: v & 0x00ff_ffff,
        kind: Vdl2AddrKind::from_bits(((v >> 24) & 0x7) as u8),
        cr: (v >> 27) & 1 == 1,
    }
}

/// Whether the address-extension bits are where the standard puts them: clear
/// on every octet but the last of the whole eight-octet field.
pub fn address_ext_ok(dst: &[u8; 4], src: &[u8; 4]) -> bool {
    dst.iter().all(|o| o & 1 == 0) && src[..3].iter().all(|o| o & 1 == 0) && src[3] & 1 == 1
}

/// Read the one-octet link control field.
///
/// AVLC is modulo-8 only, so the sequence numbers are three bits each and the
/// poll/final bit always sits at `0x10`.
pub fn control(b: u8) -> Vdl2Frame {
    let pf = b & 0x10 != 0;
    let nr = (b >> 5) & 0x7;
    if b & 1 == 0 {
        return Vdl2Frame::I { ns: (b >> 1) & 0x7, nr, p: pf };
    }
    if b & 0x3 == 0x1 {
        return match (b >> 2) & 0x3 {
            0 => Vdl2Frame::Rr { nr, pf },
            1 => Vdl2Frame::Rnr { nr, pf },
            2 => Vdl2Frame::Rej { nr, pf },
            _ => Vdl2Frame::Srej { nr, pf },
        };
    }
    // Unnumbered: the modifier bits are scattered either side of the poll/final
    // bit, so the codes are matched with that bit masked out rather than
    // reassembled.
    match b & !0x10 {
        0x03 => Vdl2Frame::Ui { pf },
        0x0f => Vdl2Frame::Dm { pf },
        0x43 => Vdl2Frame::Disc { pf },
        0x63 => Vdl2Frame::Ua { pf },
        0x87 => Vdl2Frame::Frmr { pf },
        0xaf => Vdl2Frame::Xid { pf },
        0xe3 => Vdl2Frame::Test { pf },
        _ => Vdl2Frame::Unknown(b),
    }
}

/// A frame that checked out.
#[derive(Debug, Clone, PartialEq)]
pub struct Frame {
    pub dst: Address,
    pub src: Address,
    pub control: Vdl2Frame,
    pub payload: Vec<u8>,
}

/// Why a frame was not believed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    /// Shorter than an empty frame can be.
    Short(usize),
    /// The frame check sequence did not come out.
    BadFcs,
    /// An address-extension bit is set where it cannot be.
    BadAddress,
}

/// Parse one AVLC frame, frame check sequence included.
pub fn parse(octets: &[u8]) -> Result<Frame, FrameError> {
    if octets.len() < MIN_LEN {
        return Err(FrameError::Short(octets.len()));
    }
    if !sdroxide_ax25::fcs::check(octets) {
        return Err(FrameError::BadFcs);
    }
    let dst_b: [u8; 4] = octets[0..4].try_into().expect("checked above");
    let src_b: [u8; 4] = octets[4..8].try_into().expect("checked above");
    if !address_ext_ok(&dst_b, &src_b) {
        return Err(FrameError::BadAddress);
    }
    Ok(Frame {
        dst: address(&dst_b),
        src: address(&src_b),
        control: control(octets[8]),
        payload: octets[9..octets.len() - 2].to_vec(),
    })
}

/// The transmitter's half: build a frame and append its check sequence.
///
/// Here rather than in a test so the two halves cannot drift apart about which
/// bit of an address octet is which.
pub fn build(dst: Address, src: Address, control: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(MIN_LEN + payload.len());
    out.extend_from_slice(&address_octets(dst, false));
    out.extend_from_slice(&address_octets(src, true));
    out.push(control);
    out.extend_from_slice(payload);
    let fcs = sdroxide_ax25::fcs::fcs(&out);
    out.extend_from_slice(&fcs);
    out
}

/// The inverse of [`address`]: four octets, with the extension bit set on the
/// last one only when this is the source address that ends the field.
pub fn address_octets(a: Address, last: bool) -> [u8; 4] {
    let v = (a.addr & 0x00ff_ffff) | (u32::from(kind_bits(a.kind)) << 24) | (u32::from(a.cr) << 27);
    let raw = crate::header::reverse_bits(v, 28);
    let mut b = [
        ((raw << 1) & 0xfe) as u8,
        ((raw >> 6) & 0xfe) as u8,
        ((raw >> 13) & 0xfe) as u8,
        ((raw >> 20) & 0xfe) as u8,
    ];
    if last {
        b[3] |= 1;
    }
    b
}

fn kind_bits(k: Vdl2AddrKind) -> u8 {
    match k {
        Vdl2AddrKind::Aircraft => 1,
        Vdl2AddrKind::GroundAdmin => 4,
        Vdl2AddrKind::GroundDelegated => 5,
        Vdl2AddrKind::AllStations => 7,
        Vdl2AddrKind::Reserved => 0,
    }
}

/// The control byte for a [`Vdl2Frame`] — the inverse of [`control`].
pub fn control_octet(f: Vdl2Frame) -> u8 {
    let pf = |b: bool| if b { 0x10u8 } else { 0 };
    match f {
        Vdl2Frame::I { ns, nr, p } => (nr << 5) | pf(p) | ((ns & 7) << 1),
        Vdl2Frame::Rr { nr, pf: f } => (nr << 5) | pf(f) | 0x01,
        Vdl2Frame::Rnr { nr, pf: f } => (nr << 5) | pf(f) | 0x05,
        Vdl2Frame::Rej { nr, pf: f } => (nr << 5) | pf(f) | 0x09,
        Vdl2Frame::Srej { nr, pf: f } => (nr << 5) | pf(f) | 0x0d,
        Vdl2Frame::Ui { pf: f } => 0x03 | pf(f),
        Vdl2Frame::Dm { pf: f } => 0x0f | pf(f),
        Vdl2Frame::Disc { pf: f } => 0x43 | pf(f),
        Vdl2Frame::Ua { pf: f } => 0x63 | pf(f),
        Vdl2Frame::Frmr { pf: f } => 0x87 | pf(f),
        Vdl2Frame::Xid { pf: f } => 0xaf | pf(f),
        Vdl2Frame::Test { pf: f } => 0xe3 | pf(f),
        Vdl2Frame::Unknown(b) => b,
    }
}

/// The three octets that mark ACARS carried over AVLC.
pub const ACARS_PREFIX: [u8; 3] = [0xff, 0xff, 0x01];

/// What a payload this decoder does not parse appears to be.
///
/// Named from the first octets, which is as far as the network-layer protocol
/// identifiers go. Being told "CLNP, 84 octets" between two addresses is worth
/// more than being told nothing, and it is honest about the difference between
/// "there was nothing there" and "there was something and it was not read".
pub fn describe_payload(control: Vdl2Frame, payload: &[u8]) -> String {
    if payload.is_empty() {
        return format!("{}, no payload", control.label());
    }
    let what = match payload[0] {
        0x81 => "CLNP",
        0x82 => "ES-IS",
        0x84 => "IDRP",
        0x8e => "IDRP",
        b if b & 0xf0 == 0x10 => "X.25",
        _ => "unrecognised",
    };
    format!("{what}, {} octets", payload.len())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addr(a: u32, k: Vdl2AddrKind, cr: bool) -> Address {
        Address { addr: a, kind: k, cr }
    }

    /// An address survives the trip to the wire and back, at every address type
    /// and with the command/response bit either way.
    #[test]
    fn an_address_round_trips() {
        for kind in [
            Vdl2AddrKind::Aircraft,
            Vdl2AddrKind::GroundAdmin,
            Vdl2AddrKind::GroundDelegated,
            Vdl2AddrKind::AllStations,
        ] {
            for &a in &[0u32, 1, 0x44_0F_31, 0x00_FF_FF, 0xFF_FF_FF] {
                for cr in [false, true] {
                    let want = addr(a, kind, cr);
                    let got = address(&address_octets(want, false));
                    assert_eq!(got, want, "{kind:?} {a:06X} cr={cr}");
                }
            }
        }
    }

    /// Seven bits of every octet are address; the eighth belongs to HDLC and
    /// never leaks into it.
    #[test]
    fn the_extension_bits_are_not_part_of_the_address() {
        let want = addr(0x44_0F_31, Vdl2AddrKind::Aircraft, false);
        let clean = address_octets(want, false);
        assert!(clean.iter().all(|o| o & 1 == 0));
        let mut dirty = clean;
        for o in dirty.iter_mut() {
            *o |= 1;
        }
        assert_eq!(address(&dirty), want, "an extension bit changed the address");
    }

    /// ...and an extension bit set where the standard says it is clear is a
    /// malformed frame, not something to shrug at.
    #[test]
    fn a_stray_extension_bit_is_a_malformed_frame() {
        let dst = address_octets(addr(0x100, Vdl2AddrKind::AllStations, false), false);
        let src = address_octets(addr(0x200, Vdl2AddrKind::GroundAdmin, false), true);
        assert!(address_ext_ok(&dst, &src));

        let mut bad = dst;
        bad[2] |= 1;
        assert!(!address_ext_ok(&bad, &src));

        // The source's last octet is the one that *must* be set.
        let mut bad = src;
        bad[3] &= 0xfe;
        assert!(!address_ext_ok(&dst, &bad));
    }

    /// Every control field this decoder names round-trips through its octet.
    #[test]
    fn every_control_field_round_trips() {
        let cases = [
            Vdl2Frame::I { ns: 0, nr: 0, p: false },
            Vdl2Frame::I { ns: 5, nr: 3, p: true },
            Vdl2Frame::Rr { nr: 7, pf: false },
            Vdl2Frame::Rnr { nr: 2, pf: true },
            Vdl2Frame::Rej { nr: 1, pf: false },
            Vdl2Frame::Srej { nr: 6, pf: true },
            Vdl2Frame::Ui { pf: false },
            Vdl2Frame::Dm { pf: true },
            Vdl2Frame::Disc { pf: false },
            Vdl2Frame::Ua { pf: true },
            Vdl2Frame::Frmr { pf: false },
            Vdl2Frame::Xid { pf: true },
            Vdl2Frame::Test { pf: false },
        ];
        for c in cases {
            assert_eq!(control(control_octet(c)), c, "{c:?}");
        }
    }

    /// The three families are told apart by the low bits, and every one of the
    /// 256 control octets decodes to something rather than panicking.
    #[test]
    fn the_control_octet_is_total() {
        for b in 0..=255u8 {
            let f = control(b);
            match b & 3 {
                0 | 2 => assert!(matches!(f, Vdl2Frame::I { .. }), "{b:02X}"),
                1 => assert!(
                    matches!(
                        f,
                        Vdl2Frame::Rr { .. }
                            | Vdl2Frame::Rnr { .. }
                            | Vdl2Frame::Rej { .. }
                            | Vdl2Frame::Srej { .. }
                    ),
                    "{b:02X}"
                ),
                _ => assert!(
                    !matches!(f, Vdl2Frame::I { .. }) && !matches!(f, Vdl2Frame::Rr { .. }),
                    "{b:02X}"
                ),
            }
        }
    }

    /// A whole frame round-trips, and its check sequence is the one AX.25 uses.
    #[test]
    fn a_frame_round_trips_with_a_valid_check_sequence() {
        let dst = addr(0x44_0F_31, Vdl2AddrKind::Aircraft, true);
        let src = addr(0x10_A1_B2, Vdl2AddrKind::GroundAdmin, false);
        let payload = b"\xff\xff\x01hello".to_vec();
        let wire = build(dst, src, control_octet(Vdl2Frame::Ui { pf: false }), &payload);
        assert!(wire.len() >= MIN_LEN);

        let f = parse(&wire).expect("valid frame");
        assert_eq!(f.dst, dst);
        assert_eq!(f.src, src);
        assert_eq!(f.control, Vdl2Frame::Ui { pf: false });
        assert_eq!(f.payload, payload);
    }

    /// One flipped bit anywhere in a frame is caught by the check sequence.
    #[test]
    fn a_flipped_bit_fails_the_check_sequence() {
        let wire = build(
            addr(1, Vdl2AddrKind::Aircraft, false),
            addr(2, Vdl2AddrKind::GroundAdmin, false),
            0x03,
            b"payload",
        );
        for i in 0..wire.len() {
            for bit in 0..8 {
                let mut bad = wire.clone();
                bad[i] ^= 1 << bit;
                if bad == wire {
                    continue;
                }
                match parse(&bad) {
                    Err(FrameError::BadFcs) | Err(FrameError::BadAddress) => {}
                    other => panic!("octet {i} bit {bit} accepted: {other:?}"),
                }
            }
        }
    }

    /// A frame too short to hold addresses is refused before anything is read
    /// out of it.
    #[test]
    fn a_short_frame_is_refused() {
        assert_eq!(parse(&[0u8; 10]), Err(FrameError::Short(10)));
        assert_eq!(parse(&[]), Err(FrameError::Short(0)));
    }
}
