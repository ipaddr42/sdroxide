//! The QO-100 narrowband beacon decoder: BPSK-400/AO-40-uncoded telemetry,
//! run off a dedicated downconversion of the raw IQ — the same shape
//! `sdroxide_skimmer` and `sdroxide_ism` use for their own narrowband work.
//! Native-only (runs in the engine); see [`bpsk`] for the protocol itself and
//! [`controller`] for how it is driven from the engine's audio-block loop.

pub mod bpsk;
mod controller;

pub use controller::Qo100Controller;
