//! Open a port and stream for a few seconds, proving samples actually
//! arrive and are not all zero — the same real-hardware role
//! `examples/probe.rs` plays for open/enumerate alone.
//!
//! ```text
//! cargo run -p sdroxide-fobos --example stream_probe
//! ```

use std::time::{Duration, Instant};

use sdroxide_fobos::{FobosHandle, OpenParams, Port};

fn stream_for(handle: &mut FobosHandle, secs: u64) -> (u64, f32) {
    let mut buf = vec![0.0f32; 1 << 16];
    let mut total_pairs = 0u64;
    let mut peak = 0.0f32;
    let deadline = Instant::now() + Duration::from_secs(secs);
    while Instant::now() < deadline {
        let n = handle.rx_read(&mut buf);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(2));
            continue;
        }
        total_pairs += (n / 2) as u64;
        for chunk in buf[..n].as_chunks::<2>().0 {
            let mag = (chunk[0] * chunk[0] + chunk[1] * chunk[1]).sqrt();
            if mag > peak {
                peak = mag;
            }
        }
    }
    (total_pairs, peak)
}

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_fobos=debug".into()),
        )
        .init();

    // --- RF port ---
    let params = OpenParams {
        serial: String::new(),
        port: Port::Rf,
        center_hz: 100_000_000.0,
        sample_rate_hz: 2_000_000.0,
        lna_gain: 2,
        vga_gain: 10,
        clk_external: false,
    };
    let mut handle = match FobosHandle::open(params) {
        Ok(h) => h,
        Err(e) => {
            println!("RF open failed: {e}");
            return;
        }
    };
    println!("--- {} ---", handle.label);
    println!(
        "hw {}  fw {}  {} {}",
        handle.board.hw_revision,
        handle.board.fw_version,
        handle.board.manufacturer,
        handle.board.product
    );
    println!("rate {:.3} Msps (of {} offered)", handle.sample_rate_hz / 1e6, handle.rates_hz.len());

    let (total_pairs, peak) = stream_for(&mut handle, 3);
    println!("\nread {total_pairs} complex samples over 3s, peak {peak:.5} full scale");
    if peak == 0.0 {
        println!("  (all zero — the stream is running but nothing is arriving)");
    }
    println!("alive: {}, silent for {:?}", handle.is_alive(), handle.silent_for());

    // Exercise the live rate change (stream.rs's stop/reconfigure/restart
    // path — confirmed working above ~8 Msps).
    println!("\n--- live rate change: {:.3} -> 20.0 Msps ---", handle.sample_rate_hz / 1e6);
    handle.set_rate_hz(20_000_000.0);
    std::thread::sleep(Duration::from_millis(200));
    let (total_pairs2, peak2) = stream_for(&mut handle, 2);
    println!(
        "after rate change: now {:.3} Msps, read {total_pairs2} complex samples over 2s, peak {peak2:.5}",
        handle.current_rate_hz() / 1e6
    );
    println!("alive: {}", handle.is_alive());
    handle.release();
    println!("RF released cleanly\n");

    // --- HF1 port: the new DDC path ---
    let params = OpenParams {
        serial: String::new(),
        port: Port::Hf1,
        center_hz: 810_000.0,
        // Matches FobosConfig::default() — narrow enough that 810 kHz is
        // actually reachable rather than clamped (see WbDdc's own near-DC
        // floor, and FobosConfig::default()'s comment on why 625 kHz).
        sample_rate_hz: 625_000.0,
        lna_gain: 0,
        vga_gain: 0,
        clk_external: false,
    };
    let mut handle = match FobosHandle::open(params) {
        Ok(h) => h,
        Err(e) => {
            println!("HF1 open failed: {e}");
            return;
        }
    };
    println!("--- {} ---", handle.label);
    println!("rate {:.3} Msps (target 0.625 Msps complex, via WbDdc)", handle.sample_rate_hz / 1e6);
    let (total_pairs, peak) = stream_for(&mut handle, 3);
    println!("\nread {total_pairs} complex samples over 3s, peak {peak:.5} full scale");
    if peak == 0.0 {
        println!("  (all zero — the DDC path is running but nothing is arriving)");
    }
    println!("alive: {}, silent for {:?}", handle.is_alive(), handle.silent_for());

    println!("\n--- HF1 retune: 810 kHz -> 1010 kHz (WbDdc::set_center_hz only, no FFI call) ---");
    handle.set_center_hz(1_010_000.0);
    std::thread::sleep(Duration::from_millis(100));
    let (total_pairs2, peak2) = stream_for(&mut handle, 2);
    println!("after retune: read {total_pairs2} complex samples over 2s, peak {peak2:.5}");
    println!("alive: {}", handle.is_alive());

    handle.release();
    println!("HF1 released cleanly\n");

    // --- HF1+HF2 dual: rx_read_pair, real hardware ---
    let params = OpenParams {
        serial: String::new(),
        port: Port::HfDual,
        center_hz: 810_000.0,
        sample_rate_hz: 625_000.0,
        lna_gain: 0,
        vga_gain: 0,
        clk_external: false,
    };
    let mut handle = match FobosHandle::open(params) {
        Ok(h) => h,
        Err(e) => {
            println!("HfDual open failed: {e}");
            return;
        }
    };
    println!("--- {} ---", handle.label);
    println!(
        "rate {:.3} Msps per channel (target 0.625 Msps complex, via two WbDdcs)",
        handle.sample_rate_hz / 1e6
    );

    let mut main = vec![0.0f32; 1 << 16];
    let mut aux = vec![0.0f32; 1 << 16];
    let mut total_pairs = 0u64;
    let (mut peak_main, mut peak_aux) = (0.0f32, 0.0f32);
    let mut mismatches = 0u64;
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline {
        let n = handle.rx_read_pair(&mut main, &mut aux);
        if n == 0 {
            std::thread::sleep(Duration::from_millis(2));
            continue;
        }
        total_pairs += n as u64;
        for chunk in main[..n * 2].as_chunks::<2>().0 {
            let mag = (chunk[0] * chunk[0] + chunk[1] * chunk[1]).sqrt();
            if mag > peak_main {
                peak_main = mag;
            }
        }
        for chunk in aux[..n * 2].as_chunks::<2>().0 {
            let mag = (chunk[0] * chunk[0] + chunk[1] * chunk[1]).sqrt();
            if mag > peak_aux {
                peak_aux = mag;
            }
        }
        // rx_read (the single-channel method) must never be reachable here
        // by mistake — a real check that the two are actually separate
        // buffers, not the same data read twice.
        if main[..n * 2] == aux[..n * 2] {
            mismatches += 1;
        }
    }
    println!(
        "\nread {total_pairs} complex pairs/channel over 3s, peak main {peak_main:.5}, peak aux \
         {peak_aux:.5}"
    );
    if peak_main == 0.0 || peak_aux == 0.0 {
        println!("  (all zero on at least one channel — something's wrong)");
    }
    if mismatches > 0 {
        println!(
            "  WARNING: main and aux were byte-identical on {mismatches} block(s) — check the \
             lane de-interleave"
        );
    } else {
        println!("  main and aux never matched byte-for-byte — genuinely two channels");
    }
    println!("alive: {}, silent for {:?}", handle.is_alive(), handle.silent_for());

    handle.release();
    println!("HfDual released cleanly");
}
