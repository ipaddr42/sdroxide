//! Errors, written for the operator who has to act on them.
//!
//! "No library" and "no board" look identical from a distance and need
//! different fixes, so each is its own variant with the fix in the text —
//! the same reasoning `sdroxide-sdrplay`/`sdroxide-lime` apply to their own
//! errors.

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    /// `libfobos` is not installed, or not findable.
    #[error("{0}")]
    LibMissing(String),

    /// No board matched what the configuration asked for.
    #[error("{0}")]
    NotFound(String),

    /// `fobos_rx_open` failed on a device the enumeration just listed —
    /// almost always another program already has it open, since the API
    /// gives no distinct "busy" code to tell that apart from any other
    /// open failure.
    #[error("{0} may be held by another program — close it and try again")]
    InUse(String),

    /// Any other API failure: the call that failed, and its numeric code
    /// translated through `fobos_rx_error_name`.
    #[error("libfobos {call} failed: {text}")]
    Api { call: &'static str, text: String },
}

impl Error {
    pub(crate) fn api(call: &'static str, text: String) -> Error {
        Error::Api { call, text }
    }
}
