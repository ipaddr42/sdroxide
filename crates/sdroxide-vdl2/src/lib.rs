//! VDL Mode 2 — the VHF datalink airliners and ground stations exchange ACARS
//! over, on seven 25 kHz channels around 136.8 MHz.
//!
//! Native-only, and driven by the engine like the ADS-B and ISM decoders: it
//! takes complex baseband from a wideband window and hands back a message log
//! and a station table.
//!
//! # Sources
//!
//! ETSI EN 301 841-1, published by ICAO as Annex 10 Volume III Part I Chapter 6
//! — the VDL Mode 2 SARPs — for the physical and link layers, and ARINC 618 for
//! the ACARS message structure carried over them. Each module cites the part it
//! implements.
//!
//! Cross-checked against `szpajder/dumpvdl2`'s behaviour; no code is taken from
//! it.

pub mod acars;
pub mod avlc;
pub mod block;
pub mod channel;
mod controller;
pub mod demod;
pub mod gate;
pub mod header;
pub mod plan;
pub mod rs;
pub mod scramble;
pub mod station;
pub mod sync;
pub mod tx;
pub mod xid;

pub use controller::{Vdl2Action, Vdl2Controller};

/// Whether a stream of this rate, centred here, reaches any VDL2 channel at all.
///
/// The engine's "can this receiver do it" predicate, the same shape
/// `sdroxide_adsb::window_covers` has.
pub fn window_covers(center_hz: f64, rate_hz: f64) -> bool {
    !plan::channels_in_window(center_hz, rate_hz).is_empty()
}
