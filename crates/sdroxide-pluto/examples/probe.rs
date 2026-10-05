//! Open a Pluto, print everything it says about itself, then stream for a
//! couple of seconds and report what came back.
//!
//! ```text
//! RUST_LOG=sdroxide_pluto=debug cargo run -p sdroxide-pluto --example probe -- 192.168.2.1
//! ```
//!
//! This backend has not been verified against hardware, so the output is
//! designed to be pasted into a bug report. Two parts of it matter most:
//!
//! - **The `first READBUF chunk` line in the trace.** The chunked framing —
//!   a length, then the channel mask, then the data — is the piece of the
//!   protocol most likely to be wrong in a from-scratch client, and it cannot
//!   be checked from this side of the wire.
//! - **The measured rate and RMS.** A plausible ksps figure with an RMS near
//!   zero is a working link and a wrong sample layout. An implausible ksps
//!   figure is a framing fault — or, when every sample is the same value, a
//!   receiver that is not running at all (issue #470).
//!
//! The stream ends because this program releases the radio after two seconds,
//! not because anything failed.

use std::time::{Duration, Instant};

use sdroxide_pluto::PlutoHandle;
use sdroxide_types::{PlutoAgc, PlutoConfig};

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_pluto=debug".into()),
        )
        .init();

    let mut args = std::env::args().skip(1);
    let address = args.next().unwrap_or_else(|| sdroxide_pluto::DEFAULT_ADDRESS.to_string());
    let center_hz: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(100_000_000.0);
    let rate_hz: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(2_000_000.0);

    println!("--- discovery ---");
    for dev in sdroxide_pluto::discover_default() {
        println!("  {}", dev.label());
    }

    println!("\n--- opening {address} at {:.6} MHz ---", center_hz / 1e6);
    let cfg = PlutoConfig {
        address: address.clone(),
        sample_rate_hz: rate_hz,
        agc: PlutoAgc::SlowAttack,
        ..PlutoConfig::default()
    };
    let mut handle = match PlutoHandle::open(&address, &cfg, center_hz) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("could not open {address}: {e}");
            if let Some(t) = sdroxide_pluto::diagnostics() {
                println!("\n{t}");
            }
            std::process::exit(1);
        }
    };

    println!("  {}", handle.label());
    println!("  firmware   {}", handle.firmware);
    println!("  serial     {}", handle.serial);
    println!(
        "  RX LO      {:.3} – {:.3} MHz",
        handle.limits.rx_lo_hz.0 / 1e6,
        handle.limits.rx_lo_hz.1 / 1e6
    );
    println!(
        "  TX LO      {:.3} – {:.3} MHz",
        handle.limits.tx_lo_hz.0 / 1e6,
        handle.limits.tx_lo_hz.1 / 1e6
    );
    println!(
        "  rate       {:.3} – {:.3} Msps (running at {:.3})",
        handle.limits.sample_rate_hz.0 / 1e6,
        handle.limits.sample_rate_hz.1 / 1e6,
        handle.sample_rate_hz / 1e6
    );
    println!("  filter     {:.3} MHz", handle.rf_bandwidth_hz / 1e6);
    println!(
        "  RX gain    {:.1} – {:.1} dB, step {:.2}",
        handle.limits.rx_gain_db.0, handle.limits.rx_gain_db.1, handle.limits.rx_gain_db.2
    );
    println!(
        "  TX gain    {:.2} – {:.2} dB, step {:.2}",
        handle.limits.tx_gain_db.0, handle.limits.tx_gain_db.1, handle.limits.tx_gain_db.2
    );
    println!("  RX ports   {}", join_or(&handle.limits.rx_ports, "(not published)"));
    println!("  TX ports   {}", join_or(&handle.limits.tx_ports, "(not published)"));
    println!("  AGC modes  {}", join_or(&handle.limits.agc_modes, "(not published)"));
    if !handle.limits.assumed.is_empty() {
        println!("  ASSUMED    {}", handle.limits.assumed.join(", "));
    }

    println!("\n--- streaming for 2 s ---");
    let mut buf = vec![0f32; 65536];
    let mut pairs = 0u64;
    let mut sum_sq = 0f64;
    let mut peak = 0f32;
    let mut first: Option<(f32, f32)> = None;
    let mut varied = false;
    let started = Instant::now();
    while started.elapsed() < Duration::from_secs(2) {
        let n = handle.rx_read(&mut buf);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(2));
            continue;
        }
        pairs += (n / 2) as u64;
        for &v in &buf[..n] {
            sum_sq += (v as f64) * (v as f64);
            peak = peak.max(v.abs());
        }
        for &[i, q] in buf[..n].as_chunks::<2>().0 {
            varied |= *first.get_or_insert((i, q)) != (i, q);
        }
    }
    let dt = started.elapsed().as_secs_f64();
    let rms = if pairs > 0 { (sum_sq / (pairs as f64 * 2.0)).sqrt() } else { 0.0 };
    let ksps = pairs as f64 / dt / 1000.0;
    println!("  {pairs} samples in {dt:.2} s = {ksps:.1} ksps");
    println!("  RMS {rms:.6} ({:.1} dBFS), peak {peak:.6}", 20.0 * rms.max(1e-12).log10());
    if pairs == 0 {
        println!("  NOTHING ARRIVED — the trace below is what to send in a bug report.");
    } else if rms < 1e-6 {
        println!("  The link works but every sample is zero: check the scan format above.");
    } else if !varied {
        // Issue #470: an AD9361 whose state machine is not receiving still
        // streams, and what it streams is one frozen word, over and over.
        println!(
            "  Every sample is the same value — the receiver is not running, however well the \
             link is carrying it. Look for `ensm_mode` in the trace below."
        );
    } else if ksps * 1e3 > handle.sample_rate_hz * 1.1 {
        println!(
            "  That is faster than the {:.1} ksps the radio is set to — no running receiver \
             produces that, so these are not samples from one.",
            handle.sample_rate_hz / 1e3
        );
    } else if peak >= 0.999 {
        println!("  Clipping. Lower the RX gain or switch the AGC out of manual.");
    }

    handle.release();
    if let Some(t) = sdroxide_pluto::diagnostics() {
        println!("\n{t}");
    }
}

fn join_or(items: &[String], empty: &str) -> String {
    if items.is_empty() { empty.to_string() } else { items.join(", ") }
}
