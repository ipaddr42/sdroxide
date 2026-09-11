//! Pull I/Q from a SpyServer through the real client — framer, decoder, ring —
//! and report the level it arrives at, so one format or gain setting can be
//! held against another with nothing of the engine in the way.
//!
//! `cargo run --release --example spyserver_level -- host:port FORMAT GAIN [SECS] [HZ] [nofft]`
//!
//! `FORMAT` is `8`, `16` or `32`; `GAIN` is `auto` or a digital gain in dB.
//! `HZ` is where to tune, and it matters: how much headroom an 8-bit stream has
//! is a property of the band, not of the receiver, so a figure measured on one
//! band says nothing about another. The FFT lane is on unless `nofft` is given,
//! to match an ordinary session.

use std::time::{Duration, Instant};

use sdroxide_spyserver::SpyServerHandle;
use sdroxide_types::{SpyServerConfig, SpyServerFormat};

fn main() {
    // `RUST_LOG` if set, `info` otherwise. `sdroxide_spyserver::net=debug` is
    // the one that shows the digital-gain loop's steps.
    let filter = tracing_subscriber::EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"));
    tracing_subscriber::fmt().with_env_filter(filter).with_writer(std::io::stderr).init();

    let a: Vec<String> = std::env::args().collect();
    let address = a.get(1).cloned().unwrap_or_else(|| "127.0.0.1:5555".into());
    let format = match a.get(2).map(String::as_str) {
        Some("16") => SpyServerFormat::Int16,
        Some("32") => SpyServerFormat::Float32,
        _ => SpyServerFormat::Uint8,
    };
    let gain = a.get(3).cloned().unwrap_or_else(|| "auto".into());
    let seconds: f64 = a.get(4).and_then(|s| s.parse().ok()).unwrap_or(5.0);
    let center: f64 = a.get(5).and_then(|s| s.parse().ok()).unwrap_or(1_081_400.0);
    let fft = !a.iter().any(|s| s == "nofft");
    let (auto_digital_gain, digital_gain_db) =
        if gain == "auto" { (true, 0.0) } else { (false, gain.parse().expect("gain in dB")) };

    let cfg = SpyServerConfig {
        address,
        iq_format: format,
        iq_decimation: 0,
        auto_digital_gain,
        digital_gain_db,
        fft_enabled: fft,
        ..SpyServerConfig::default()
    };
    let mut h = SpyServerHandle::connect_wideband(&cfg, center).expect("connect");
    eprintln!("{}", h.label);

    // |v| buckets: <1e-4, <1e-3, <1e-2, <0.1, <0.5, <1, <10, >=10.
    let mut hist = [0usize; 8];
    let mut buf = vec![0f32; 1 << 16];
    let (mut total, mut max, mut sumsq, mut nonfinite) = (0usize, 0f32, 0f64, 0usize);
    let start = Instant::now();
    while start.elapsed() < Duration::from_secs_f64(seconds) {
        let n = h.rx_read(&mut buf);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(2));
            continue;
        }
        for &v in &buf[..n] {
            if !v.is_finite() {
                nonfinite += 1;
                continue;
            }
            let m = v.abs();
            max = max.max(m);
            sumsq += f64::from(v) * f64::from(v);
            let b =
                [1e-4, 1e-3, 1e-2, 0.1, 0.5, 1.0, 10.0].iter().position(|&t| m < t).unwrap_or(7);
            hist[b] += 1;
        }
        total += n;
    }
    let secs = start.elapsed().as_secs_f64();
    println!(
        "format={format:?} gain={gain} fft={fft} alive={} silent_for={:.2?}",
        h.is_alive(),
        h.silent_for()
    );
    println!(
        "pairs={} ({:.1} ksps over {secs:.2} s)  max|v|={max:.5}  rms={:.5}  non-finite={nonfinite}",
        total / 2,
        (total / 2) as f64 / secs / 1e3,
        (sumsq / total.max(1) as f64).sqrt(),
    );
    println!(
        "|v|: <1e-4 {}  <1e-3 {}  <1e-2 {}  <0.1 {}  <0.5 {}  <1 {}  <10 {}  >=10 {}",
        hist[0], hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], hist[7]
    );
    h.release();
}
