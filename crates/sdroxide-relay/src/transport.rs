//! The seam every kind of switching hardware meets at.
//!
//! Semantic rather than byte-level, because most of what is behind it has no
//! bytes: a handshake line is a level, a GPIO line is a level, and the command
//! hook is a process. A trait in terms of buffers would fit the relay boards
//! and be a lie for everything else — the same argument `RfeTransport` makes
//! next door in `sdroxide-limerfe`.
//!
//! The mask a transport is handed is **physical**: the coils that should be
//! energised, with the operator's active-high/active-low choice already
//! resolved. Nothing below this line knows what a channel is *for*, which is
//! what keeps the polarity decision in exactly one place.
//!
//! [`RelayTransport::round_trip`] is load-bearing. A CDC board answers in a
//! couple of milliseconds and a 9600-baud one in ten, and the caller of a
//! key-down has to wait for whichever it actually has — so it is asked rather
//! than assumed.

use std::time::Duration;

use crate::error::Result;
use crate::frame::ChannelMask;

/// One piece of switching hardware.
///
/// Every method may block, which is why the whole trait lives behind a thread —
/// see [`crate::spawn`].
pub trait RelayTransport: Send {
    /// Put the contacts in this state. The transport is free to send only what
    /// changed, and must send everything it manages the first time and after
    /// any failure: a half-applied state is not a state.
    fn apply(&mut self, want: ChannelMask) -> Result<()>;

    /// What the hardware says its contacts are actually set to. `None` from
    /// anything that cannot be asked, which is most of it — and never
    /// load-bearing: a board that will not answer is not a board that is
    /// failing.
    fn read_back(&mut self) -> Result<Option<ChannelMask>> {
        Ok(None)
    }

    /// The raw level of the transmit-sense input, if one is wired. `true` is a
    /// high line; the operator's polarity is applied above this, so a transport
    /// never has to know which way round their opto-isolator is.
    ///
    /// This is polled far more often than anything else here — it is the whole
    /// point of the input, that it is seen in milliseconds — so it must be
    /// cheap and must not block.
    fn sense(&mut self) -> Result<Option<bool>> {
        Ok(None)
    }

    /// Roughly what one command costs on this link, wire time and all.
    /// Measured where it can be, estimated where it cannot; either way it is
    /// added to the operator's lead time before RF is allowed out.
    fn round_trip(&self) -> Duration;

    /// One line naming this link, for logs and the status area.
    fn describe(&self) -> String;
}
