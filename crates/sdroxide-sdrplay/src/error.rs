//! Errors, written for the operator who has to act on them.
//!
//! Three different absences look identical from a distance — no library, no
//! service, no device — and each has a different fix, so they are three
//! different errors with the fix in the text.

use crate::ffi;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// The vendor library is not installed (or not findable).
    #[error("{0}")]
    LibMissing(String),

    /// The library is present but its background service is not answering.
    #[error(
        "the SDRplay API service is not responding ({0}) — start it \
         (on Linux: sudo systemctl enable --now sdrplay) and try again"
    )]
    ServiceDown(String),

    /// Another program (or another sdroxide) holds the device.
    #[error("{0} is in use by another SDRplay application")]
    InUse(String),

    /// No RSP matched what the config asked for.
    #[error("{0}")]
    NotFound(String),

    /// Any other API failure, with the call that failed named.
    #[error("sdrplay_api {call} failed: {text}")]
    Api { call: &'static str, text: String },
}

impl Error {
    /// Add what this machine looks like from outside the vendor's library, when
    /// there is anything to add — see [`crate::linux::hint`].
    ///
    /// Only the two errors that mean "the receiver is not there": a service
    /// that is not running and a device somebody else already holds are
    /// already as specific as they can be, and a second explanation on top of
    /// them would be a guess.
    pub(crate) fn with_host_hint(self) -> Error {
        let Some(hint) = crate::linux::hint() else { return self };
        match self {
            Error::Api { call, text } => Error::Api { call, text: format!("{text} — {hint}") },
            Error::NotFound(text) => Error::NotFound(format!("{text} {hint}")),
            other => other,
        }
    }

    /// Wrap an API status, routing the codes with a specific fix to the
    /// errors that state it.
    pub(crate) fn from_status(api: &ffi::Api, call: &'static str, err: ffi::ErrT) -> Error {
        match err {
            ffi::ERR_SERVICE_NOT_RESPONDING | ffi::ERR_INVALID_SERVICE_VERSION => {
                Error::ServiceDown(api.err_text(err))
            }
            _ => Error::Api { call, text: api.err_text(err) },
        }
    }
}
