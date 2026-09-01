//! Errors from a KiwiSDR link.
//!
//! Every variant carries a sentence rather than a code, for the same reason the
//! SpyServer client's do — and with one addition that matters more here than
//! anywhere else in this program. [`Error::Refused`] is a receiver saying *no*:
//! it is full, it wants a password, or it has reached this address's time
//! limit. That is not a fault to retry against, it is an answer, and a
//! reconnect loop aimed at somebody else's receiver because it just told us to
//! go away is the one thing this backend must never do.

use thiserror::Error;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, Error)]
pub enum Error {
    /// The link: could not resolve, could not connect, or lost the connection.
    #[error("{0}")]
    Net(String),
    /// Something arrived that this protocol does not allow.
    #[error("{0}")]
    Proto(String),
    /// The receiver answered, and the answer was no. Never retried
    /// automatically — see the module note above.
    #[error("{0}")]
    Refused(String),
}

impl Error {
    /// Whether reconnecting could plausibly help.
    ///
    /// False for [`Error::Refused`]: the receiver is working perfectly and has
    /// declined. Trying again immediately would be rude to its operator and
    /// would not work.
    pub fn is_retryable(&self) -> bool {
        !matches!(self, Error::Refused(_))
    }
}
