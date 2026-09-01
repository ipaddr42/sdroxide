//! Run a program on key-down and another on key-up.
//!
//! The catch-all, and the reason this subsystem does not need a driver for
//! every board ever sold. Denkovi's FT245 bit-bang boards, microHAM's
//! interfaces, a network power switch, a GPIO expander on another machine over
//! ssh — all of them have a command-line tool, and none of them has a protocol
//! worth embedding here.
//!
//! # What it does not wait for
//!
//! The process is started and not waited on. Waiting would put a program's
//! whole run time — unbounded, and on a first run including a dynamic linker
//! and a Python interpreter — between the operator's thumb and their antenna
//! relay. So the transaction is *starting* it: an error here means the program
//! could not be launched at all, which is the fault an operator can actually
//! fix, and a program that starts and then fails is one the operator's own
//! testing has to find.
//!
//! That is also why the settings panel says to leave a wider lead for this link
//! than for a relay board. There is nothing honest to put in
//! [`RelayTransport::round_trip`] here.

use std::process::{Command, Stdio};
use std::time::Duration;

use crate::error::{Error, Result};
use crate::frame::ChannelMask;
use crate::transport::RelayTransport;

pub struct CommandTransport {
    tx: Vec<String>,
    rx: Vec<String>,
    last: Option<bool>,
}

impl CommandTransport {
    pub fn new(tx_cmd: &str, rx_cmd: &str) -> Result<CommandTransport> {
        let tx = split(tx_cmd);
        let rx = split(rx_cmd);
        if tx.is_empty() {
            return Err(Error::Config("no transmit command set for the T/R switch".into()));
        }
        // A receive command is not required — a script that toggles, or a tool
        // that takes the state as an argument the operator only wrote once,
        // is the operator's business. But it is almost always a mistake, so it
        // is worth a line in the log rather than silence.
        if rx.is_empty() {
            tracing::warn!(
                "the T/R switch has a transmit command and no receive command: nothing will put \
                 the contacts back"
            );
        }
        Ok(CommandTransport { tx, rx, last: None })
    }

    fn run(&self, argv: &[String]) -> Result<()> {
        if argv.is_empty() {
            return Ok(());
        }
        Command::new(&argv[0])
            .args(&argv[1..])
            // Nothing reads any of it, and a child inheriting this process's
            // terminal would print into the middle of the operator's log.
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .map_err(|e| Error::Open { path: argv[0].clone(), source: e })?;
        Ok(())
    }
}

/// Split a command line on whitespace. Not a shell: there is no quoting and no
/// expansion, because a T/R switch is not the place to discover that a path
/// with a space in it silently ran the wrong program.
fn split(s: &str) -> Vec<String> {
    s.split_whitespace().map(str::to_string).collect()
}

impl RelayTransport for CommandTransport {
    fn apply(&mut self, want: ChannelMask) -> Result<()> {
        // One bit of state, whatever the channel table says: a program is
        // either being told "transmit" or "receive".
        let on = want != 0;
        if self.last == Some(on) {
            return Ok(());
        }
        self.run(if on { &self.tx } else { &self.rx })?;
        self.last = Some(on);
        Ok(())
    }

    fn round_trip(&self) -> Duration {
        // Starting a process, and nothing more: the program's own run time is
        // not waited for and cannot be estimated. See the module docs.
        Duration::from_millis(5)
    }

    fn describe(&self) -> String {
        format!("external command \"{}\"", self.tx.join(" "))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_switch_with_no_transmit_command_is_refused_rather_than_silent() {
        assert!(CommandTransport::new("   ", "true").is_err());
    }

    #[test]
    fn the_command_line_is_split_and_not_interpreted() {
        assert_eq!(split("usbrelay 1_1=1"), vec!["usbrelay", "1_1=1"]);
        assert_eq!(split("  "), Vec::<String>::new());
        // No shell: a quote is a character in an argument, not syntax.
        assert_eq!(split("sh -c 'a b'"), vec!["sh", "-c", "'a", "b'"]);
    }

    /// Only on a change, so a per-tick reconcile does not fork a process two
    /// hundred times a second.
    #[test]
    fn nothing_is_run_twice_for_the_same_state() {
        let mut t = CommandTransport::new("/nonexistent-program-for-a-test", "").unwrap();
        assert!(t.apply(1).is_err(), "a program that cannot be started is an error");
        // The failure did not record a state, so it is tried again.
        assert!(t.apply(1).is_err());
        // Receive has no command at all: nothing runs and nothing fails.
        assert!(t.apply(0).is_ok());
        assert!(t.apply(0).is_ok());
    }
}
