//! Open a public KiwiSDR, stream briefly, and print what came back.
//!
//! ```text
//! KIWI_HOST=sdr.example.org:8073 cargo run -p sdroxide-kiwisdr --example probe
//! KIWI_HOST=… KIWI_FREQ=9950000 cargo run -p sdroxide-kiwisdr --example probe
//! ```
//!
//! Two things this settles that no unit test can. The **measured** I/Q rate
//! against the one the receiver advertised — they differ by design, and it is
//! the advertised figure the resampler has to use. And the waterfall's bin
//! alignment: the peak bin is printed as a frequency, so pointing this at a
//! receiver with a known strong broadcast band says whether the band view is
//! drawn where the signal actually is.
//!
//! Be a guest about it: it takes one of the receiver's channels for the length
//! of the run, so pick one that is not full and do not leave it looping.

use std::time::{Duration, Instant};

use sdroxide_kiwisdr::KiwiHandle;
use sdroxide_types::KiwiConfig;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_kiwisdr=info".into()),
        )
        .init();

    let host = std::env::var("KIWI_HOST").unwrap_or_default();
    if host.is_empty() {
        eprintln!("set KIWI_HOST=host:port (pick one with a free channel from the listing)");
        std::process::exit(2);
    }
    let freq: f64 =
        std::env::var("KIWI_FREQ").ok().and_then(|s| s.parse().ok()).unwrap_or(9_950_000.0);
    let secs: u64 = std::env::var("KIWI_SECS").ok().and_then(|s| s.parse().ok()).unwrap_or(10);

    println!("--- {host} /status");
    match sdroxide_kiwisdr::test_connection(&host, Duration::from_secs(5)) {
        Ok(line) => println!("{line}"),
        Err(e) => {
            eprintln!("status: {e}");
            std::process::exit(1);
        }
    }

    let cfg = KiwiConfig { address: host.clone(), ..KiwiConfig::default() };
    println!("\n--- connecting, {:.3} kHz", freq / 1e3);
    let t0 = Instant::now();
    let mut h = match KiwiHandle::connect(&cfg, "sdroxide-probe", freq) {
        Ok(h) => h,
        Err(e) => {
            eprintln!("connect: {e} (retryable: {})", e.is_retryable());
            std::process::exit(1);
        }
    };
    println!("up in {:?}: {}", t0.elapsed(), h.info.describe());
    println!("  advertised I/Q rate {:.6} Hz", h.info.sample_rate_hz);
    println!(
        "  band {:.0}-{:.0} kHz, wf_cal {}",
        (h.info.center_hz - h.info.bandwidth_hz / 2.0) / 1e3,
        (h.info.center_hz + h.info.bandwidth_hz / 2.0) / 1e3,
        h.info.wf_cal
    );

    let mut buf = vec![0.0f32; 8192];
    let mut samples = 0u64;
    let mut peak = 0.0f32;
    let mut sumsq = 0.0f64;
    let mut frames = 0u32;
    let mut last_wf: Option<sdroxide_kiwisdr::WaterfallFrame> = None;
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs(secs) {
        let n = h.rx_read(&mut buf);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(5));
        } else {
            samples += n as u64 / 2;
            for v in &buf[..n] {
                peak = peak.max(v.abs());
                sumsq += f64::from(*v) * f64::from(*v);
            }
        }
        if let Some(f) = h.take_waterfall() {
            frames += 1;
            last_wf = Some(f);
        }
    }
    let el = start.elapsed().as_secs_f64();

    println!("\n--- I/Q");
    println!(
        "  {samples} pairs in {el:.2}s = {:.1} Hz measured (advertised {:.1})",
        samples as f64 / el,
        h.info.sample_rate_hz
    );
    let rms = (sumsq / (samples.max(1) * 2) as f64).sqrt();
    println!(
        "  peak {:.1} dBFS, rms {:.1} dBFS",
        20.0 * f64::from(peak).max(1e-9).log10(),
        20.0 * rms.max(1e-9).log10()
    );
    println!(
        "  receiver's own S-meter {:.1} dBm, ADC overflow {}",
        h.smeter_dbm(),
        h.adc_overflow()
    );

    println!("\n--- waterfall");
    match last_wf {
        Some(f) => {
            println!("  {frames} frames in {el:.2}s = {:.1} fps", f64::from(frames) / el);
            println!(
                "  {} bins over {:.3}-{:.3} MHz",
                f.bins.len(),
                (f.center_hz - f.span_hz / 2.0) / 1e6,
                (f.center_hz + f.span_hz / 2.0) / 1e6
            );
            let lo = f.center_hz - f.span_hz / 2.0;
            let step = f.span_hz / f.bins.len() as f64;
            let mut idx: Vec<usize> = (0..f.bins.len()).collect();
            idx.sort_by(|a, b| f.bins[*b].total_cmp(&f.bins[*a]));
            println!(
                "  floor {:.0} dBm, strongest bins:",
                f.bins.iter().copied().fold(f32::MAX, f32::min)
            );
            for i in idx.iter().take(5) {
                println!("    {:8.3} MHz  {:6.0} dBm", (lo + *i as f64 * step) / 1e6, f.bins[*i]);
            }
        }
        None => println!("  no frames (wide_lane off, or the socket was refused)"),
    }

    println!("\n--- releasing the channel");
    h.release();
    println!("done; alive={} refusal={:?}", h.is_alive(), h.refusal());
}
