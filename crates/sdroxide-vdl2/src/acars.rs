//! ACARS carried over AVLC — "AOA", the way nearly all VDL Mode 2 traffic an
//! operator can actually read arrives.
//!
//! An ACARS block over VHF normally begins with a SOH and a bit-sync preamble.
//! Over AVLC it does not: the link layer has already framed and checked it, so
//! the block starts at the mode character, prefixed only by the three octets
//! `FF FF 01` that say "what follows is ACARS".
//!
//! ```text
//!  FF FF 01 │ M │ registration │ A │ label │ B │ STX │ …text… │ ETX │ BCS │ DEL
//!           │ 1 │      7       │ 1 │   2   │ 1 │  1  │         │  1  │  2  │  1
//! ```
//!
//! A downlink — aircraft to ground — inserts a four-character message sequence
//! number and a six-character flight identification immediately after the STX.
//! Nothing in the block says which direction it is going; the AVLC address types
//! do, which is why [`parse`] is told rather than left to guess.
//!
//! # Parity, and why a failure here is not a rejection
//!
//! Every character is seven-bit ASCII with an odd parity bit in the eighth
//! position. By the time this runs, the AVLC frame check sequence has already
//! passed over the same octets, so a parity disagreement is not evidence about
//! the radio path — it is evidence about this decoder's understanding of where
//! the fields start. It is counted and reported, and nothing is thrown away for
//! it. The block check sequence at the end is reported on the same terms.
//!
//! # What is not verified
//!
//! The block check sequence is CRC-16/CCITT reflected (`0x8408`), seeded zero,
//! with no final inversion, computed over the parity-stripped characters from
//! the mode through the ETX and leaving zero when its own two octets are run
//! through as well. That is ARINC 618's, and it agrees with what `acarsdec`
//! does; it has **not** been checked against a real off-air message here. It is
//! reported and never used as a filter precisely because of that — if the
//! variant is wrong, the cost is a column that reads "no" rather than a message
//! that is thrown away.
//!
//! Source: ARINC 618 for the block structure; ETSI EN 301 841-1 §5.6 for the
//! `FF FF 01` protocol identifier.

use sdroxide_types::Vdl2Acars;

const STX: u8 = 0x02;
const ETX: u8 = 0x03;
const ETB: u8 = 0x17;
const DEL: u8 = 0x7f;
/// Mode, registration, technical acknowledgement, label and block id: the
/// header every block has, whether or not it carries text.
const HEAD_LEN: usize = 1 + 7 + 1 + 2 + 1;
/// The message sequence number and flight identification a downlink adds.
const DOWNLINK_LEN: usize = 4 + 6;

/// Parse an ACARS block from an AVLC payload, prefix included.
///
/// `downlink` is true when the frame's source address is an aircraft.
pub fn parse(payload: &[u8], downlink: bool) -> Option<Vdl2Acars> {
    let body = payload.strip_prefix(&crate::avlc::ACARS_PREFIX)?;
    if body.len() < HEAD_LEN {
        return None;
    }

    // Strip the parity bit off everything first: the terminator has to be found
    // before it is known where the characters stop and the check sequence
    // begins, and the search needs the stripped values.
    let chars: Vec<u8> = body.iter().map(|&c| c & 0x7f).collect();

    let mut a = Vdl2Acars {
        mode: chars[0] as char,
        registration: trim_pad(&chars[1..8]),
        ack: chars[8] as char,
        label: text(&chars[9..11]),
        block_id: chars[11] as char,
        ..Vdl2Acars::default()
    };

    // Everything past the header is optional: a handshake block ends here.
    let rest = &chars[HEAD_LEN..];
    let is_term = |c: u8| c == ETX || c == ETB;
    // The terminator is looked for from the end, because an ETX octet can occur
    // inside a text field and the last one is the one that ends the block. The
    // preferred answer is the last one with room for a check sequence behind
    // it; failing that, the last one at all, and no check sequence to report.
    let with_bcs = (0..rest.len()).rev().find(|&i| is_term(rest[i]) && i + 2 < rest.len());
    let term = with_bcs.or_else(|| rest.iter().rposition(|&c| is_term(c)));

    // Parity is a property of the *characters*, so it is counted over the block
    // up to and including the terminator. The check octets and the trailing DEL
    // carry none, and counting them would report an error on half of all blocks.
    let parity_end = term.map_or(body.len(), |i| HEAD_LEN + i + 1);
    a.parity_errors = body[..parity_end.min(body.len())]
        .iter()
        .filter(|c| c.count_ones() % 2 == 0)
        .count()
        .min(u16::MAX as usize) as u16;

    if rest.is_empty() {
        return Some(a);
    }
    a.more = term.map(|i| rest[i]) == Some(ETB);

    let mut at = 0usize;
    let mut text_end = term.unwrap_or(rest.len());
    if rest.first() == Some(&STX) {
        at = 1;
        if downlink && text_end >= at + DOWNLINK_LEN {
            a.msn = text(&rest[at..at + 4]);
            a.flight = trim_pad(&rest[at + 4..at + DOWNLINK_LEN]);
            at += DOWNLINK_LEN;
        }
    }
    if text_end < at {
        text_end = at;
    }
    a.text = text(&rest[at..text_end]);

    // The check sequence, over the parity-stripped block through the
    // terminator, with its own two octets folded in: a correct block leaves
    // zero. The trailing DEL is outside it.
    if let Some(i) = with_bcs {
        let bcs_at = HEAD_LEN + i + 1;
        let c = crc16_kermit(&chars[..bcs_at]);
        // The check octets are not characters and carry no parity, so they are
        // taken from the untouched body.
        a.crc_ok = crc16_kermit_from(c, &body[bcs_at..bcs_at + 2]) == 0;
    }
    Some(a)
}

/// The transmitter's half: an ACARS block as it rides over AVLC.
///
/// Beside [`parse`] so the two cannot disagree about where a field starts,
/// which is the one thing no round-trip test would catch if they were apart.
pub fn build(a: &Vdl2Acars, downlink: bool) -> Vec<u8> {
    let mut body: Vec<u8> = Vec::new();
    body.push(a.mode as u8);
    push_fixed(&mut body, &a.registration, 7);
    body.push(a.ack as u8);
    push_fixed(&mut body, &a.label, 2);
    body.push(a.block_id as u8);

    let has_text = !a.text.is_empty() || downlink;
    if has_text {
        body.push(STX);
        if downlink {
            push_fixed(&mut body, &a.msn, 4);
            push_fixed(&mut body, &a.flight, 6);
        }
        body.extend(a.text.bytes());
        body.push(if a.more { ETB } else { ETX });
        let c = crc16_kermit(&body);
        body.push((c & 0xff) as u8);
        body.push((c >> 8) as u8);
        body.push(DEL);
    }

    // Odd parity on the characters, not on the check sequence or the DEL.
    let parity_upto = if has_text { body.len() - 3 } else { body.len() };
    for c in body[..parity_upto].iter_mut() {
        if c.count_ones() % 2 == 0 {
            *c |= 0x80;
        }
    }

    let mut out = crate::avlc::ACARS_PREFIX.to_vec();
    out.extend_from_slice(&body);
    out
}

fn push_fixed(out: &mut Vec<u8>, s: &str, n: usize) {
    let b = s.as_bytes();
    for i in 0..n {
        out.push(if i < b.len() { b[i] } else { b' ' });
    }
}

/// CRC-16/CCITT, reflected `0x8408`, seeded zero, no final inversion.
pub fn crc16_kermit(data: &[u8]) -> u16 {
    crc16_kermit_from(0, data)
}

/// The same, continuing from an existing register — which is what makes
/// "run the check octets through and expect zero" expressible.
pub fn crc16_kermit_from(seed: u16, data: &[u8]) -> u16 {
    let mut c = seed;
    for &b in data {
        c ^= u16::from(b);
        for _ in 0..8 {
            c = if c & 1 != 0 { (c >> 1) ^ 0x8408 } else { c >> 1 };
        }
    }
    c
}

fn text(b: &[u8]) -> String {
    b.iter().map(|&c| printable(c)).collect()
}

fn trim_pad(b: &[u8]) -> String {
    text(b).trim_matches(|c: char| c == ' ' || c == '.' || c == '\0').to_string()
}

/// Keep the control characters an ACARS text really uses, and replace the rest
/// so a decode error cannot put a terminal escape sequence on somebody's screen.
fn printable(c: u8) -> char {
    match c {
        b'\r' | b'\n' | b'\t' => c as char,
        0x20..=0x7e => c as char,
        _ => '·',
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The catalogue check value for CRC-16/KERMIT, which is what pins the
    /// variant. A round trip against `build` would agree with any polynomial.
    #[test]
    fn the_check_sequence_is_the_kermit_variant() {
        assert_eq!(crc16_kermit(b"123456789"), 0x2189);
    }

    /// ...and running a block's own check octets through leaves zero, which is
    /// the form `parse` actually uses.
    #[test]
    fn a_block_plus_its_check_sequence_leaves_zero() {
        let msg = b"\x32.OE-LWA\x15H1B\x02hello\x03";
        let c = crc16_kermit(msg);
        let bcs = [(c & 0xff) as u8, (c >> 8) as u8];
        assert_eq!(crc16_kermit_from(crc16_kermit(msg), &bcs), 0);
    }

    /// An uplink round-trips: no sequence number, no flight identification.
    #[test]
    fn an_uplink_round_trips() {
        let want = Vdl2Acars {
            mode: '2',
            registration: "OE-LWA".to_string(),
            ack: '\x15',
            label: "H1".to_string(),
            block_id: 'B',
            text: "CLIMB FL350".to_string(),
            ..Vdl2Acars::default()
        };
        let wire = build(&want, false);
        let got = parse(&wire, false).expect("parsed");
        assert_eq!(got.mode, '2');
        assert_eq!(got.registration, "OE-LWA");
        assert_eq!(got.label, "H1");
        assert_eq!(got.block_id, 'B');
        assert_eq!(got.text, "CLIMB FL350");
        assert!(got.msn.is_empty());
        assert!(got.flight.is_empty());
        assert!(got.crc_ok, "check sequence did not come out");
        assert_eq!(got.parity_errors, 0);
    }

    /// A downlink carries the two extra fields, and they do not end up in the
    /// text — the failure a decoder that ignored the direction would make.
    #[test]
    fn a_downlink_keeps_its_sequence_number_and_flight_apart_from_the_text() {
        let want = Vdl2Acars {
            mode: '2',
            registration: "D-AIZP".to_string(),
            ack: '\x15',
            label: "44".to_string(),
            block_id: '1',
            msn: "M01A".to_string(),
            flight: "DLH456".to_string(),
            text: "/POS N48123W011456".to_string(),
            ..Vdl2Acars::default()
        };
        let wire = build(&want, true);
        let got = parse(&wire, true).expect("parsed");
        assert_eq!(got.msn, "M01A");
        assert_eq!(got.flight, "DLH456");
        assert_eq!(got.text, "/POS N48123W011456");
        assert!(got.crc_ok);
    }

    /// A block with no text at all — a handshake — is a real message, not a
    /// truncated one.
    #[test]
    fn a_handshake_block_has_no_text() {
        let want = Vdl2Acars {
            mode: '2',
            registration: "OE-LWA".to_string(),
            ack: 'A',
            label: "_d".to_string(),
            block_id: '1',
            ..Vdl2Acars::default()
        };
        let wire = build(&want, false);
        let got = parse(&wire, false).expect("parsed");
        assert_eq!(got.label, "_d");
        assert!(got.text.is_empty());
        assert!(!got.crc_ok, "a block with no terminator has no check sequence to pass");
    }

    /// An ETB says more blocks follow; an ETX says this was the last.
    #[test]
    fn etb_means_there_is_more_to_come() {
        let mut a = Vdl2Acars {
            mode: '2',
            label: "H1".to_string(),
            block_id: '1',
            text: "part one".to_string(),
            more: true,
            ..Vdl2Acars::default()
        };
        assert!(parse(&build(&a, false), false).expect("parsed").more);
        a.more = false;
        assert!(!parse(&build(&a, false), false).expect("parsed").more);
    }

    /// A parity bit knocked off is counted, and the message still arrives —
    /// the frame check sequence has already vouched for these octets.
    #[test]
    fn a_parity_error_is_counted_not_fatal() {
        let a = Vdl2Acars {
            mode: '2',
            registration: "OE-LWA".to_string(),
            label: "H1".to_string(),
            block_id: '1',
            text: "hello".to_string(),
            ..Vdl2Acars::default()
        };
        let mut wire = build(&a, false);
        wire[3] ^= 0x80; // the mode character's parity bit
        let got = parse(&wire, false).expect("parsed anyway");
        assert_eq!(got.parity_errors, 1);
        assert_eq!(got.text, "hello");
    }

    /// A payload that is not ACARS is not ACARS.
    #[test]
    fn a_payload_without_the_prefix_is_refused() {
        assert!(parse(b"\x81\x00\x00some clnp", false).is_none());
        assert!(parse(b"\xff\xff\x01", false).is_none(), "prefix alone is not a block");
    }

    /// Nothing a decode error can produce puts a control sequence on a screen.
    #[test]
    fn control_characters_do_not_reach_the_panel() {
        let mut wire = crate::avlc::ACARS_PREFIX.to_vec();
        wire.extend_from_slice(&[b'2', 0x1b, 0x1b, b'[', b'2', b'J', 0, 0, 0, 0, 0, 0]);
        let got = parse(&wire, false).expect("parsed");
        assert!(!got.registration.contains('\x1b'));
    }
}
