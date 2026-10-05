//! Watch the dial read-back the stream thread publishes.
//!
//!     cargo run -p sdroxide-elad --example dial
//!
//! Opens the device, streams, and prints what `EladHandle::tuned_hz` says four
//! times a second. On an FDM-DUO that is the radio's own VFO, so turning the
//! knob shows up here; every other model answers nothing at all.
use std::time::Duration;

use sdroxide_elad::EladHandle;
use sdroxide_types::EladConfig;

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_elad=debug".into()),
        )
        .with_writer(std::io::stderr)
        .init();
    let cfg = EladConfig { sample_rate_hz: 192_000, ..EladConfig::default() };
    let mut handle = EladHandle::open(&cfg, 940_000.0).expect("open the ELAD");
    handle.follow_radio_dial(true);
    println!("opened {}", handle.label);
    let mut buf = vec![0f32; 8192];
    for i in 0..40 {
        // Keep reading samples, so this is the load the engine puts on it.
        let _ = handle.rx_read(&mut buf);
        println!("{:5.2}s  tuned_hz {:?}", i as f32 * 0.25, handle.tuned_hz());
        std::thread::sleep(Duration::from_millis(250));
    }
    handle.release();
}
