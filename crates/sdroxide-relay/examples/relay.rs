//! A standalone prober for the station's T/R switch hardware.
//!
//! ```text
//! cargo run -p sdroxide-relay --example relay -- --list
//! cargo run -p sdroxide-relay --example relay -- --serial /dev/ttyUSB0 --board lcus --set 1 on
//! cargo run -p sdroxide-relay --example relay -- --hid /dev/hidraw0 --set 1 on
//! cargo run -p sdroxide-relay --example relay -- --cm108 /dev/hidraw2 --pin 3 --set 1 on
//! cargo run -p sdroxide-relay --example relay -- --serial /dev/ttyUSB0 --board lcus --sense cts
//! ```
//!
//! This exists for one reason above the obvious. The Windows and macOS halves
//! of the HID layer are written from documentation and have never run against
//! hardware — no machine here has either operating system. A compile break is
//! caught by the release builds; a wrong buffer offset is not. So this prints
//! what it did and what came back in enough detail that one operator pasting a
//! transcript settles the question, without asking them to reproduce anything
//! under a log filter or to trust the rest of the program while they do it.

use std::time::{Duration, Instant};

use sdroxide_relay::{RelayTransport, frame};
use sdroxide_types::{RelayFamily, SenseLine, SerialConfig};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let get = |name: &str| -> Option<String> {
        args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned()
    };
    let has = |name: &str| args.iter().any(|a| a == name);

    if args.is_empty() || has("--help") || has("-h") {
        eprintln!("{}", include_str!("relay_usage.txt"));
        return;
    }

    if has("--list") {
        list(has("--all"));
        return;
    }

    let sense = match get("--sense").as_deref() {
        Some("cts") => SenseLine::Cts,
        Some("dsr") => SenseLine::Dsr,
        Some("dcd") => SenseLine::Dcd,
        Some(other) => {
            eprintln!("unknown sense line {other:?}; try cts, dsr or dcd");
            return;
        }
        None => SenseLine::Off,
    };
    // Every contact 1..=8, so a probe drives whatever the operator asks for
    // without needing a channel table.
    let managed: u32 = 0xFF;

    let mut transport: Box<dyn RelayTransport> = if let Some(path) = get("--serial") {
        let family = match get("--board").as_deref() {
            None | Some("lcus") => RelayFamily::Lcus,
            Some("kmtronic") => RelayFamily::KMtronic,
            Some("numato") => RelayFamily::Numato,
            Some(other) => {
                eprintln!("unknown board {other:?}; try lcus, kmtronic or numato");
                return;
            }
        };
        let cfg = SerialConfig { path: path.clone(), ..SerialConfig::default() };
        println!("opening {} as a {} board…", path, family.label());
        match sdroxide_relay::SerialTransport::open(&cfg, family, managed, sense) {
            Ok(t) => Box::new(t),
            Err(e) => return eprintln!("failed: {e}"),
        }
    } else if let Some(path) = get("--lines") {
        let cfg = SerialConfig { path: path.clone(), ..SerialConfig::default() };
        println!("opening {path} for RTS/DTR…");
        match sdroxide_relay::LineTransport::open(&cfg, 0b11, sense) {
            Ok(t) => Box::new(t),
            Err(e) => return eprintln!("failed: {e}"),
        }
    } else if let Some(key) = get("--hid") {
        println!("opening {key} as a dcttech HID relay…");
        match sdroxide_relay::DcttechTransport::open(&key, managed) {
            Ok(t) => Box::new(t),
            Err(e) => return eprintln!("failed: {e}"),
        }
    } else if let Some(key) = get("--cm108") {
        let pin = get("--pin").and_then(|p| p.parse().ok()).unwrap_or(frame::cm108::DEFAULT_PIN);
        println!("opening {key} as a CM108-family sound card, pin {pin}…");
        match sdroxide_relay::Cm108Transport::open(&key, pin) {
            Ok(t) => Box::new(t),
            Err(e) => return eprintln!("failed: {e}"),
        }
    } else {
        eprintln!("{}", include_str!("relay_usage.txt"));
        return;
    };

    println!("open: {}", transport.describe());
    println!("one command costs about {:?} on this link", transport.round_trip());

    // Put everything in a known state first, and time it — the number an
    // operator setting their lead wants to see.
    let t0 = Instant::now();
    match transport.apply(0) {
        Ok(()) => println!("all contacts released, in {:?}", t0.elapsed()),
        Err(e) => println!("releasing them failed: {e}"),
    }

    if let Some(ch) = get("--set") {
        let ch: u8 = match ch.parse() {
            Ok(c) => c,
            Err(_) => return eprintln!("--set wants a contact number"),
        };
        let on = !has("off");
        let mask = if on { frame::bit(ch) } else { 0 };
        let t = Instant::now();
        match transport.apply(mask) {
            Ok(()) => println!(
                "contact {ch} {} in {:?} — listen for the relay",
                if on { "closed" } else { "opened" },
                t.elapsed()
            ),
            Err(e) => println!("failed: {e}"),
        }
        match transport.read_back() {
            Ok(Some(m)) => println!("the board reports contacts {m:#010b}"),
            Ok(None) => println!("the board does not report its contacts (or would not)"),
            Err(e) => println!("read-back failed: {e}"),
        }
        // Leave it as it was found: a probe that walks away with an antenna
        // relay energised is a probe that costs somebody a front end.
        std::thread::sleep(Duration::from_secs(1));
        let _ = transport.apply(0);
        println!("released again");
    }

    if sense != SenseLine::Off {
        println!("watching {} for ten seconds — key the rig", sense.label());
        let until = Instant::now() + Duration::from_secs(10);
        let mut last: Option<bool> = None;
        while Instant::now() < until {
            match transport.sense() {
                Ok(Some(level)) => {
                    if last != Some(level) {
                        last = Some(level);
                        println!(
                            "  {:>5.1}s  line is {}",
                            t0.elapsed().as_secs_f32(),
                            match level {
                                true => "HIGH",
                                false => "LOW",
                            }
                        );
                    }
                }
                Ok(None) => {}
                Err(e) => {
                    println!("  sense failed: {e}");
                    break;
                }
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        if last.is_none() {
            println!("  nothing read — is the line wired?");
        }
    }
}

fn list(all: bool) {
    if all {
        // Every HID device on the machine, matched or not. What an operator
        // whose board is not being recognised has to paste: `16c0:05df` is a
        // shared hobby id and the product string is what the filter uses, so
        // "it is there but not listed above" is a different fault from "the
        // udev rule is missing" and this is how they are told apart.
        println!("every HID device this machine can see:");
        for e in sdroxide_relay::hid::enumerate(&[]) {
            println!(
                "  {:04x}:{:04x}  {:<40} {}{}",
                e.vendor,
                e.product,
                e.name,
                e.key,
                if e.serial.is_empty() { String::new() } else { format!("  serial={}", e.serial) }
            );
        }
        println!();
    }
    let devices = sdroxide_relay::list();
    if devices.is_empty() {
        println!("no USB HID relay boards or GPIO-capable sound cards found.");
        println!(
            "on Linux that is often the udev rule: install \
             packaging/linux/60-sdroxide-relay.rules and replug the device."
        );
    }
    for d in &devices {
        println!(
            "{:<28} {:<10} {} contact(s)  key={}",
            d.label,
            d.link.label(),
            if d.channels == 0 { "?".to_string() } else { d.channels.to_string() },
            d.key
        );
    }
    println!();
    println!("serial relay boards are serial ports; list those with your usual tool.");
}
