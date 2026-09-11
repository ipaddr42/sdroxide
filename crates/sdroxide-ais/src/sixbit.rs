//! Six-bit ASCII: the text alphabet AIS carries, and the armouring an
//! `!AIVDM` sentence wraps a message in.
//!
//! Two different six-bit encodings live here and they are not the same thing,
//! which is the one confusion worth heading off:
//!
//! * [`text`] reads the **payload's own** six-bit character set — the alphabet
//!   a ship's name, call sign and destination are stated in. Value 0 is `@`,
//!   1..26 are `A`..`Z`, 32 is a space; there are no lower-case letters,
//!   because there are none in the set.
//! * [`armour`] writes the **transport** encoding — the printable form a
//!   receiver hands to a chart plotter over NMEA 0183, where each six bits
//!   become one character by adding 48, and 8 more above 87 to skip the
//!   punctuation range. Nothing in a message is stated in it; it exists so the
//!   payload can travel down a line that carries text.
//!
//! The armouring is here because it is what makes this decoder *checkable*.
//! Every AIS tool in the world speaks `!AIVDM`, so a sentence in the panel is
//! something an operator can paste into one and compare against what sdroxide
//! made of it — which, for a decoder written from a standard and not from a
//! recording, is the check that matters.
//!
//! Sources: ITU-R M.1371-5 Table 47 for the payload alphabet; IEC 61162-1 for
//! the sentence, its six-bit transport encoding and its checksum.

/// The payload's six-bit character set, in value order.
///
/// Written out rather than computed. The arithmetic looks like "add 64 and wrap
/// at 32", and it is nearly that — but the block from 32 up is plain ASCII from
/// space, and a formula that got the join wrong would silently rename every
/// ship whose name contains a digit.
const SIXBIT: [u8; 64] = *b"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";

/// One six-bit value as its character.
pub fn char_of(v: u8) -> char {
    SIXBIT[usize::from(v & 0x3f)] as char
}

/// Read `n` six-bit characters from `bits` at `at`, as a trimmed string.
///
/// Trailing `@` is padding — the standard fills a fixed-width field with it —
/// and so is trailing space, because in practice transmitters use both.
/// Leading space goes too: a name that arrives as `"   MAERSK"` is the same
/// ship as one that arrives as `"MAERSK"`, and a list sorted on the raw field
/// would put them in different places.
pub fn text(bits: &[bool], at: usize, n: usize) -> String {
    let mut s = String::with_capacity(n);
    for k in 0..n {
        let start = at + k * 6;
        if start + 6 > bits.len() {
            break;
        }
        s.push(char_of(read(bits, start, 6) as u8));
    }
    s.trim_end_matches(['@', ' ']).trim_start().to_string()
}

/// Read `n` bits at `at` as an unsigned number, most significant bit first.
///
/// The payload's own order — see [`crate::hdlc`] for why it is not the check
/// sequence's. Reads past the end as zero rather than panicking: a message may
/// be truncated by a fade in its last slot, and the fields that did arrive are
/// still worth having.
pub fn read(bits: &[bool], at: usize, n: usize) -> u64 {
    let mut v = 0u64;
    for k in 0..n {
        v <<= 1;
        if bits.get(at + k).copied().unwrap_or(false) {
            v |= 1;
        }
    }
    v
}

/// Read `n` bits at `at` as a two's-complement signed number.
pub fn read_signed(bits: &[bool], at: usize, n: usize) -> i64 {
    let v = read(bits, at, n);
    if n == 0 || n >= 64 {
        return v as i64;
    }
    let sign = 1u64 << (n - 1);
    if v & sign != 0 { (v as i64) - (1i64 << n) } else { v as i64 }
}

/// The transport armouring: six bits to one printable character.
///
/// Add 48; if that lands above `'w'` (87), add 8 more to step over the
/// characters NMEA reserves. The inverse of [`unarmour_char`].
pub fn armour_char(v: u8) -> char {
    let mut c = (v & 0x3f) + 48;
    if c > 87 {
        c += 8;
    }
    c as char
}

/// One armoured character back to its six bits, or `None` if it is not one.
pub fn unarmour_char(c: char) -> Option<u8> {
    let mut n = u32::from(c);
    if !(48..=119).contains(&n) {
        return None;
    }
    if n > 87 {
        n -= 8;
    }
    u8::try_from(n.checked_sub(48)?).ok().filter(|v| *v < 64)
}

/// Longest payload one sentence may carry.
///
/// IEC 61162-1 caps a sentence at 82 characters including `!` and the closing
/// carriage return and line feed. `!AIVDM,n,n,s,C,` is fifteen and `,p*hh` is
/// five, which leaves sixty-two; sixty keeps a margin and is what every
/// transmitter in practice uses.
const MAX_PAYLOAD_CHARS: usize = 60;

/// The `!AIVDM` sentences carrying a message's data field.
///
/// One for a message that fits, several for one that does not — a
/// static-and-voyage report is 424 bits, which is 71 characters, and no
/// sentence may carry that many. `seq` is the sequential message identifier
/// that joins a multi-sentence group; it means nothing on its own and is only
/// there so two interleaved groups can be told apart.
///
/// The last sentence states how many *fill bits* were added to round the data
/// field up to a whole number of six-bit characters — without it a receiver
/// cannot tell a 168-bit message from a 174-bit one.
pub fn nmea(bits: &[bool], channel: char, seq: u8) -> Vec<String> {
    let chars: String =
        (0..bits.len().div_ceil(6)).map(|k| armour_char(read(bits, k * 6, 6) as u8)).collect();
    let fill = chars.len() * 6 - bits.len();
    let parts: Vec<&str> = chars
        .as_bytes()
        .chunks(MAX_PAYLOAD_CHARS)
        .map(|c| std::str::from_utf8(c).unwrap_or_default())
        .collect();
    let total = parts.len().max(1);
    parts
        .iter()
        .enumerate()
        .map(|(i, part)| {
            // The group identifier is left out of a single-sentence message,
            // which is what every AIS receiver emits and what every parser
            // expects to see.
            let id = if total == 1 { String::new() } else { (seq % 10).to_string() };
            let fill = if i + 1 == total { fill } else { 0 };
            let body = format!("AIVDM,{total},{},{id},{channel},{part},{fill}", i + 1);
            format!("!{body}*{:02X}", checksum(&body))
        })
        .collect()
}

/// The NMEA checksum: exclusive-or of everything between `!` and `*`.
fn checksum(body: &str) -> u8 {
    body.bytes().fold(0u8, |a, b| a ^ b)
}

/// The data field an armoured payload carries, and how many bits of it are
/// real. The inverse of [`nmea`]'s encoding, for tests and for anything that
/// wants to read a sentence back.
pub fn unarmour(payload: &str, fill: usize) -> Option<Vec<bool>> {
    let mut bits = Vec::with_capacity(payload.len() * 6);
    for c in payload.chars() {
        let v = unarmour_char(c)?;
        for k in (0..6).rev() {
            bits.push(v >> k & 1 != 0);
        }
    }
    let keep = bits.len().checked_sub(fill)?;
    bits.truncate(keep);
    Some(bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The payload alphabet's landmarks. A table that had slipped by one would
    /// rename every ship afloat, and the join at 32 is where a formula would
    /// have gone wrong.
    #[test]
    fn the_payload_alphabet_is_where_the_standard_puts_it() {
        assert_eq!(char_of(0), '@');
        assert_eq!(char_of(1), 'A');
        assert_eq!(char_of(26), 'Z');
        assert_eq!(char_of(32), ' ');
        assert_eq!(char_of(48), '0');
        assert_eq!(char_of(57), '9');
        assert_eq!(char_of(63), '?');
    }

    /// A fixed-width name field is padded with `@`, and comes back without it.
    #[test]
    fn a_name_loses_its_padding() {
        let mut bits = Vec::new();
        for v in [8u8, 1, 13, 2, 21, 18, 7, 0, 0, 0] {
            for k in (0..6).rev() {
                bits.push(v >> k & 1 != 0);
            }
        }
        assert_eq!(text(&bits, 0, 10), "HAMBURG");
    }

    /// The transport encoding, and the fact every AIS user knows it by: the
    /// first character of an armoured payload *is* the message type. A payload
    /// beginning `1` is a position report and one beginning `5` is a static
    /// report, and that is true because the type is the first six bits and the
    /// armouring adds 48 to them.
    ///
    /// It is the one end-to-end check on this file that does not come from this
    /// file: it exercises the armouring, the most-significant-bit-first read
    /// and their agreement, against a fact stated outside sdroxide.
    #[test]
    fn the_first_armoured_character_is_the_message_type() {
        for (ty, ch) in [(1u64, '1'), (3, '3'), (5, '5'), (18, 'B'), (21, 'E'), (24, 'H')] {
            let mut bits = vec![false; 168];
            for (k, b) in bits.iter_mut().take(6).enumerate() {
                *b = ty >> (5 - k) & 1 != 0;
            }
            let s = &nmea(&bits, 'A', 0)[0];
            let payload = s.split(',').nth(5).expect("a payload field");
            assert_eq!(payload.chars().next(), Some(ch), "type {ty} armoured as {s}");
            // ...and it reads back as the number it started as.
            assert_eq!(read(&bits, 0, 6), ty);
        }
    }

    /// A sentence is well formed: one fragment for a position report, two for a
    /// static report, the fill count on the last, and a checksum that verifies.
    #[test]
    fn a_message_becomes_sentences_a_chart_plotter_would_accept() {
        let one = nmea(&[false; 168], 'A', 0);
        assert_eq!(one.len(), 1);
        assert!(one[0].starts_with("!AIVDM,1,1,,A,"), "{}", one[0]);
        assert!(one[0].ends_with(",0*4C") || one[0].contains(",0*"), "{}", one[0]);
        assert!(one[0].len() <= 80, "{} is {} characters", one[0], one[0].len());

        // 424 bits is 71 armoured characters — over one sentence's limit.
        let many = nmea(&[false; 424], 'B', 3);
        assert_eq!(many.len(), 2);
        assert!(many[0].starts_with("!AIVDM,2,1,3,B,"), "{}", many[0]);
        assert!(many[1].starts_with("!AIVDM,2,2,3,B,"), "{}", many[1]);
        // 424 is not a multiple of six: two bits of fill on the last sentence.
        assert!(many[1].contains(",2*"), "{}", many[1]);

        for s in one.iter().chain(&many) {
            let (body, sum) = s[1..].split_once('*').expect("a checksum");
            assert_eq!(u8::from_str_radix(sum, 16).expect("hex"), checksum(body), "{s}");
        }
    }

    /// The armouring round trips, fill bits and all.
    #[test]
    fn the_armouring_round_trips() {
        let bits: Vec<bool> = (0..424).map(|i| (i * 7 + 3) % 5 < 2).collect();
        let sentences = nmea(&bits, 'A', 1);
        let mut joined = String::new();
        let mut fill = 0;
        for s in &sentences {
            let f: Vec<&str> = s.split(',').collect();
            joined.push_str(f[5]);
            fill = f[6].split('*').next().and_then(|v| v.parse().ok()).unwrap_or(0);
        }
        assert_eq!(unarmour(&joined, fill).expect("armoured payload"), bits);
    }

    /// Signed fields are two's complement, which is how a longitude west of
    /// Greenwich and a rate of turn to port arrive.
    #[test]
    fn signed_fields_are_twos_complement() {
        // -1 in 28 bits.
        let bits = vec![true; 28];
        assert_eq!(read_signed(&bits, 0, 28), -1);
        assert_eq!(read(&bits, 0, 28), (1 << 28) - 1);
        // The most negative value.
        let mut bits = vec![false; 28];
        bits[0] = true;
        assert_eq!(read_signed(&bits, 0, 28), -(1 << 27));
    }
}
