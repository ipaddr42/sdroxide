//! Errors, written for the operator who has to act on them.
//!
//! Every one of these ends up in front of somebody who is looking at a relay
//! that did not click, so each says what to check rather than what failed.

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("cannot open the T/R switch on {path}: {source}")]
    Open { path: String, source: std::io::Error },

    #[error("T/R switch I/O failed: {0}")]
    Io(#[from] std::io::Error),

    /// A handshake-line read or a port operation the serial layer answers in
    /// its own error type. Kept distinct from [`Error::Io`] because a modem
    /// line that cannot be read is a different fault from a write that failed.
    #[error("T/R switch serial port: {0}")]
    Serial(#[from] serialport::Error),

    /// The device was there when it was opened and is not answering now.
    #[error(
        "the T/R switch on {path} stopped answering — check the USB cable, and that nothing \
         else has the device open"
    )]
    NoAnswer { path: String },

    /// The board answered something that is not an answer to what was asked.
    #[error("the T/R switch replied with {got:?}, which is not an answer to {sent}")]
    BadReply { sent: String, got: String },

    /// A configuration that cannot be opened. Already a sentence.
    #[error("{0}")]
    Config(String),

    /// The device is not on this machine, or is not the one named.
    #[error(
        "no T/R switch found at {key} — it may have been unplugged, or enumerated under a \
         different device node since it was chosen"
    )]
    NotFound { key: String },

    /// The device node exists and cannot be opened, which on Linux is almost
    /// always the udev rule.
    #[error(
        "permission denied opening the T/R switch at {path} — install the packaged udev rule \
         (60-sdroxide-relay.rules) and replug the device, or add yourself to the group that \
         owns it"
    )]
    Permission { path: String },

    /// This build cannot reach this kind of device on this platform.
    #[error("{0}")]
    Unsupported(String),

    /// The device stopped answering and has been given up on for this session.
    #[error("the T/R switch stopped answering and has been left alone: {0}")]
    Absent(String),
}

impl Error {
    /// Classify an OS error from opening a device node, so the operator gets
    /// the udev sentence rather than "permission denied (os error 13)".
    pub fn opening(path: &str, e: std::io::Error) -> Error {
        match e.kind() {
            std::io::ErrorKind::PermissionDenied => Error::Permission { path: path.to_string() },
            std::io::ErrorKind::NotFound => Error::NotFound { key: path.to_string() },
            _ => Error::Open { path: path.to_string(), source: e },
        }
    }

    /// The same, for the serial layer, which reports in its own error type.
    ///
    /// Worth its own arm rather than an `io::Error::other`: that loses the
    /// kind, and the two kinds it loses are exactly the two an operator can act
    /// on — a port that is not there because the adapter was unplugged, and one
    /// that is there and not theirs to open. The second is the common case on
    /// Linux, and "permission denied (os error 13)" tells nobody to add
    /// themselves to `dialout`.
    pub fn opening_serial(path: &str, e: serialport::Error) -> Error {
        match e.kind() {
            serialport::ErrorKind::NoDevice => Error::NotFound { key: path.to_string() },
            serialport::ErrorKind::Io(k) => Error::opening(path, std::io::Error::new(k, e)),
            _ => Error::Open { path: path.to_string(), source: std::io::Error::other(e) },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_missing_port_and_an_unreadable_one_are_told_apart() {
        let gone = Error::opening_serial(
            "/dev/ttyUSB9",
            serialport::Error::new(serialport::ErrorKind::NoDevice, "no such device"),
        );
        assert!(matches!(gone, Error::NotFound { .. }), "{gone}");

        let denied = Error::opening_serial(
            "/dev/ttyUSB0",
            serialport::Error::new(
                serialport::ErrorKind::Io(std::io::ErrorKind::PermissionDenied),
                "denied",
            ),
        );
        // The one an operator can actually fix, and the sentence that says how.
        assert!(denied.to_string().contains("udev"), "{denied}");
    }
}
