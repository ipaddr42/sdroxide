//! How a VDL2 data field is cut into Reed-Solomon blocks, and how those blocks
//! are spread across the transmission.
//!
//! The data field is split into blocks of at most 249 octets. Every full block
//! carries six parity octets; the final, partial block carries fewer, because
//! six parity octets on a twelve-octet block would be paying more for the
//! insurance than the goods are worth:
//!
//! | final block | parity |
//! | --- | --- |
//! | under 3 octets | none |
//! | 3 to 30 | 2 |
//! | 31 to 67 | 4 |
//! | 68 and over | 6 |
//!
//! The blocks are then *interleaved*: consecutive transmitted octets go to
//! different blocks, so a burst of interference that destroys a run of octets
//! spreads its damage across every block instead of exhausting one block's
//! correction on its own.
//!
//! # The one thing here that is a hypothesis
//!
//! Interleaving unequal-length blocks is not fully specified by "write down the
//! columns and read across the rows": the short block runs out first, and what
//! happens then has to be said. [`InterleaveOrder::RoundRobin`] is this
//! decoder's reading — the next octet goes to the next block that still has
//! room, skipping any that are full — and it is a hypothesis until a real
//! multi-block frame decodes with a good frame check sequence.
//!
//! Two things keep that contained. **A frame of 249 octets or fewer is one
//! block, and for one block every ordering is the identity** — and real VDL2
//! traffic is overwhelmingly twenty to two hundred and fifty octets, so the
//! great majority of frames never touch this. And the map is a bijection by
//! construction, checked for every length the standard allows, so a wrong
//! *ordering* can scramble a long frame but can never lose or duplicate an
//! octet. The decoder counts multi-block frames and multi-block successes
//! separately so that what is being left on the table is a number rather than a
//! mystery.
//!
//! Source: ETSI EN 301 841-1, the VDL Mode 2 data field.

use crate::rs::{self, RsError};

/// Data octets in a full Reed-Solomon block.
pub const RS_K: usize = rs::K;
/// Parity octets on a full block.
pub const RS_FULL_PARITY: usize = rs::N - rs::K;

/// How the octets of the transmission map onto the blocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum InterleaveOrder {
    /// Round-robin, skipping blocks that are already full. The reading this
    /// decoder takes.
    #[default]
    RoundRobin,
    /// No interleaving: each block's octets consecutive, blocks one after
    /// another. Kept so the alternative can be tried against a recording
    /// without rewriting the reasoning above.
    Sequential,
}

/// Parity octets for a final, partial block of `len` data octets.
pub fn fec_octets(len: usize) -> usize {
    if len < 3 {
        0
    } else if len < 31 {
        2
    } else if len < 68 {
        4
    } else {
        RS_FULL_PARITY
    }
}

/// How a data field of a given length is cut up.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Layout {
    /// `(data, parity)` octets per block, in block order.
    pub blocks: Vec<(u16, u16)>,
    pub data_octets: usize,
    pub parity_octets: usize,
    /// What actually goes over the air.
    pub total_octets: usize,
}

impl Layout {
    /// More than one block, so the interleaving above is in play.
    pub fn is_multiblock(&self) -> bool {
        self.blocks.len() > 1
    }
}

/// Cut a data field of `data_octets` into blocks.
pub fn layout(data_octets: usize) -> Layout {
    let mut blocks = Vec::new();
    let full = data_octets / RS_K;
    for _ in 0..full {
        blocks.push((RS_K as u16, RS_FULL_PARITY as u16));
    }
    let rem = data_octets % RS_K;
    if rem > 0 {
        blocks.push((rem as u16, fec_octets(rem) as u16));
    }
    let parity: usize = blocks.iter().map(|&(_, p)| p as usize).sum();
    Layout { blocks, data_octets, parity_octets: parity, total_octets: data_octets + parity }
}

/// Transmitted octet position to `(block, index within block)`.
///
/// A bijection onto the union of the blocks' positions, which is asserted for
/// every length the standard allows.
pub fn deinterleave_map(l: &Layout, order: InterleaveOrder) -> Vec<(u16, u16)> {
    let lens: Vec<usize> = l.blocks.iter().map(|&(d, p)| d as usize + p as usize).collect();
    let mut map = Vec::with_capacity(l.total_octets);
    match order {
        InterleaveOrder::Sequential => {
            for (b, &n) in lens.iter().enumerate() {
                for i in 0..n {
                    map.push((b as u16, i as u16));
                }
            }
        }
        InterleaveOrder::RoundRobin => {
            if lens.is_empty() {
                return map;
            }
            let mut pos = vec![0usize; lens.len()];
            let mut b = 0usize;
            for _ in 0..l.total_octets {
                while pos[b] == lens[b] {
                    b = (b + 1) % lens.len();
                }
                map.push((b as u16, pos[b] as u16));
                pos[b] += 1;
                b = (b + 1) % lens.len();
            }
        }
    }
    map
}

/// What came out of the forward error correction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FecResult {
    pub data: Vec<u8>,
    /// Symbols the Reed-Solomon layer had to change.
    pub corrected: usize,
}

/// Why a data field could not be recovered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockError {
    /// The field is not the length the header's transmission length implies.
    WrongLength { got: usize, want: usize },
    /// A block the parity could not repair.
    Rs(RsError),
}

/// Deinterleave a received data field and repair it.
pub fn decode(
    field: &[u8],
    data_octets: usize,
    order: InterleaveOrder,
) -> Result<FecResult, BlockError> {
    let l = layout(data_octets);
    if field.len() != l.total_octets {
        return Err(BlockError::WrongLength { got: field.len(), want: l.total_octets });
    }
    let map = deinterleave_map(&l, order);
    let mut blocks: Vec<Vec<u8>> =
        l.blocks.iter().map(|&(d, p)| vec![0u8; d as usize + p as usize]).collect();
    for (t, &(b, i)) in map.iter().enumerate() {
        blocks[b as usize][i as usize] = field[t];
    }

    let mut corrected = 0usize;
    for (b, block) in blocks.iter_mut().enumerate() {
        let nroots = l.blocks[b].1 as usize;
        corrected += rs::decode_block(block, nroots).map_err(BlockError::Rs)?;
    }

    let mut data = Vec::with_capacity(data_octets);
    for (b, block) in blocks.iter().enumerate() {
        data.extend_from_slice(&block[..l.blocks[b].0 as usize]);
    }
    Ok(FecResult { data, corrected })
}

/// The transmitter's half: cut, encode and interleave a data field.
pub fn encode(data: &[u8], order: InterleaveOrder) -> Vec<u8> {
    let l = layout(data.len());
    let mut blocks: Vec<Vec<u8>> = Vec::with_capacity(l.blocks.len());
    let mut at = 0usize;
    for &(d, p) in &l.blocks {
        let d = d as usize;
        blocks.push(rs::encode_block(&data[at..at + d], p as usize));
        at += d;
    }
    let map = deinterleave_map(&l, order);
    let mut out = vec![0u8; l.total_octets];
    for (t, &(b, i)) in map.iter().enumerate() {
        out[t] = blocks[b as usize][i as usize];
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The parity schedule, at every step of it.
    #[test]
    fn the_parity_steps_where_the_standard_says() {
        assert_eq!(fec_octets(0), 0);
        assert_eq!(fec_octets(2), 0);
        assert_eq!(fec_octets(3), 2);
        assert_eq!(fec_octets(30), 2);
        assert_eq!(fec_octets(31), 4);
        assert_eq!(fec_octets(67), 4);
        assert_eq!(fec_octets(68), 6);
        assert_eq!(fec_octets(248), 6);
    }

    /// The block cut at the lengths that matter, including the maximum frame.
    #[test]
    fn the_layout_at_the_lengths_that_matter() {
        assert_eq!(layout(11).blocks, vec![(11, 2)]);
        assert_eq!(layout(31).blocks, vec![(31, 4)]);
        assert_eq!(layout(100).blocks, vec![(100, 6)]);
        assert_eq!(layout(249).blocks, vec![(249, 6)]);
        assert_eq!(layout(250).blocks, vec![(249, 6), (1, 0)]);
        assert_eq!(layout(500).blocks, vec![(249, 6), (249, 6), (2, 0)]);

        // The longest transmission the header can describe: 0x3FFF bits.
        let max = layout(0x3FFF / 8 + 1);
        assert_eq!(max.data_octets, 2048);
        assert_eq!(max.blocks.len(), 9);
        assert_eq!(max.blocks[8], (56, 4));
        assert_eq!(max.total_octets, 2100);
    }

    /// The interleave map is a bijection for every length the standard allows.
    ///
    /// This is what makes the ordering a *hypothesis* rather than a hazard: get
    /// it wrong and a long frame is scrambled, which the frame check sequence
    /// catches. It cannot silently drop or duplicate an octet.
    #[test]
    fn the_interleave_map_is_a_bijection() {
        for order in [InterleaveOrder::RoundRobin, InterleaveOrder::Sequential] {
            for d in 0..=2048usize {
                let l = layout(d);
                let map = deinterleave_map(&l, order);
                assert_eq!(map.len(), l.total_octets, "{d} octets, {order:?}");
                let mut seen: Vec<Vec<bool>> =
                    l.blocks.iter().map(|&(dd, p)| vec![false; dd as usize + p as usize]).collect();
                for &(b, i) in &map {
                    let slot = &mut seen[b as usize][i as usize];
                    assert!(!*slot, "{d} octets, {order:?}: block {b} index {i} twice");
                    *slot = true;
                }
                for (b, block) in seen.iter().enumerate() {
                    assert!(
                        block.iter().all(|&s| s),
                        "{d} octets, {order:?}: block {b} has a hole"
                    );
                }
            }
        }
    }

    /// One block is one block: every ordering agrees, which is why the majority
    /// of real traffic is unaffected by the hypothesis above.
    #[test]
    fn a_single_block_is_the_identity_under_any_ordering() {
        for d in 1..=RS_K {
            let l = layout(d);
            assert!(!l.is_multiblock());
            let rr = deinterleave_map(&l, InterleaveOrder::RoundRobin);
            let seq = deinterleave_map(&l, InterleaveOrder::Sequential);
            assert_eq!(rr, seq, "{d} octets");
            for (t, &(b, i)) in rr.iter().enumerate() {
                assert_eq!((b, i as usize), (0, t));
            }
        }
    }

    /// The round-robin ordering drops a full block out of the rotation rather
    /// than stalling on it — the case the standard's picture does not draw.
    #[test]
    fn a_short_block_leaves_the_rotation_when_it_is_full() {
        let l = layout(500); // blocks of 255, 255 and 2
        let map = deinterleave_map(&l, InterleaveOrder::RoundRobin);
        assert_eq!(
            &map[..10],
            &[(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1), (0, 2), (1, 2), (0, 3), (1, 3)]
        );
    }

    /// A field round-trips through the whole chain, and a burst of damage
    /// spread across the blocks by the interleaver is repaired — which is what
    /// the interleaver is there for.
    #[test]
    fn an_interleaved_field_survives_a_burst() {
        let data: Vec<u8> = (0..600u16).map(|i| (i.wrapping_mul(37) >> 3) as u8).collect();
        let mut field = encode(&data, InterleaveOrder::RoundRobin);
        let l = layout(data.len());
        assert!(l.is_multiblock());
        // Three consecutive octets: one per block, so each block sees a single
        // symbol error and every block can repair it.
        for x in field.iter_mut().skip(120).take(3) {
            *x ^= 0x5a;
        }
        let out = decode(&field, data.len(), InterleaveOrder::RoundRobin).expect("repaired");
        assert_eq!(out.data, data);
        assert_eq!(out.corrected, 3);
    }

    /// A clean single-block frame — the shape almost all real traffic takes.
    #[test]
    fn a_single_block_frame_round_trips() {
        for d in [11usize, 30, 31, 67, 68, 100, 249] {
            let data: Vec<u8> = (0..d).map(|i| (i as u8).wrapping_mul(91)).collect();
            let field = encode(&data, InterleaveOrder::RoundRobin);
            assert_eq!(field.len(), d + fec_octets(d));
            let out = decode(&field, d, InterleaveOrder::RoundRobin).expect("clean");
            assert_eq!(out.data, data);
            assert_eq!(out.corrected, 0);
        }
    }

    /// A field of the wrong length is refused rather than decoded from
    /// whatever happens to be in the buffer.
    #[test]
    fn a_field_of_the_wrong_length_is_refused() {
        let field = encode(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11], InterleaveOrder::RoundRobin);
        assert!(matches!(
            decode(&field[..field.len() - 1], 11, InterleaveOrder::RoundRobin),
            Err(BlockError::WrongLength { .. })
        ));
    }
}
