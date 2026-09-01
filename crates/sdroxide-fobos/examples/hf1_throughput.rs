//! Measure how close an HF port's real, sustained complex-sample throughput
//! comes to what `sample_rate_hz`/`out_rate` claim — the direct regression
//! check for the read/DDC thread split `pump_hf`/`pump_hf_dual` use. Before
//! that split, real-hardware runs at the AM broadcast band landed around
//! 45-52% of nominal (DDC compute serialised after each blocking read
//! delayed the next one by nearly as much again); after it, comfortably
//! above 85%, with zero drops.
//!
//! ```text
//! cargo run --release -p sdroxide-fobos --example hf1_throughput -- 820000 10
//! ```
//! Args: centre frequency in Hz (default 820 kHz), duration in seconds
//! (default 10). `--release` matters — DDC compute time is part of what
//! this measures.

use std::time::{Duration, Instant};

use sdroxide_fobos::{FobosHandle, OpenParams, Port};

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_fobos=info".into()),
        )
        .init();

    let hz: f64 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(820_000.0);
    let secs: u64 = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(10);

    let params = OpenParams {
        serial: String::new(),
        port: Port::Hf1,
        center_hz: hz,
        sample_rate_hz: 625_000.0,
        lna_gain: 0,
        vga_gain: 0,
        clk_external: false,
    };
    let mut handle = match FobosHandle::open(params) {
        Ok(h) => h,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    println!("tuned {:.3} kHz, nominal out rate {:.4} Msps", hz / 1e3, handle.sample_rate_hz / 1e6);

    let mut buf = vec![0.0f32; 1 << 15];
    let mut total = 0u64;
    let deadline = Instant::now() + Duration::from_secs(secs);
    let started = Instant::now();
    while Instant::now() < deadline {
        let n = handle.rx_read(&mut buf);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(1));
            continue;
        }
        total += (n / 2) as u64;
    }
    let elapsed = started.elapsed().as_secs_f64();
    let achieved = total as f64 / elapsed;
    println!(
        "{total} complex samples over {elapsed:.3}s = {achieved:.0} samples/sec of \
         {:.0} nominal ({:.1}%)",
        handle.sample_rate_hz,
        100.0 * achieved / handle.sample_rate_hz,
    );

    handle.release();
}
