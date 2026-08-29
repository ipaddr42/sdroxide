//! What a *real* TCI client's transmit cadence actually looks like.
//!
//! Stands the built-in TCI server up on its own, without a radio, and runs
//! `Engine::fill_tx_audio_fifo`'s pacing loop against whatever connects — so
//! WSJT-X can key it and every block of the over can be printed: what arrived,
//! what was queued, what was asked for, and how much of the over went out as
//! silence because nothing was there.
//!
//! Nothing transmits: there is no radio here at all. The point is the
//! conversation, which is the half of issue #202 that no synthetic client can
//! answer — a stand-in only ever proves what its author already believed.
//!
//! `cargo run -p sdroxide-tci --example tx_pacing_probe -- [port] [mode] [target_ms]`
//!
//! `mode` is `outstanding` (what the engine does today), `always` (ask for the
//! whole deficit on every block, what it did before) or `every:N` (ask on every
//! N-th block only).

use std::time::{Duration, Instant};

use sdroxide_tci::server::{TciServerController, TciStateSnapshot};
use sdroxide_types::{DeviceCaps, Mode, TciServerConfig};

/// One engine transmit block: 10 ms at 48 kHz.
const BLOCK: usize = 480;
const RATE: usize = 48_000;

fn main() {
    let port: u16 = std::env::args().nth(1).and_then(|s| s.parse().ok()).unwrap_or(50002);
    // Re-read at every key-down, so one client session can be measured under
    // several pacing rules without reconnecting it.
    let plan_path = std::env::args().nth(2).unwrap_or_else(|| "/tmp/tci_pace_plan".into());
    let read_plan = |path: &str| -> (String, usize, u32, bool) {
        let raw = std::fs::read_to_string(path).unwrap_or_else(|_| "outstanding 40".into());
        let mut it = raw.split_whitespace();
        let mode = it.next().unwrap_or("outstanding").to_string();
        let target_ms: usize = it.next().and_then(|s| s.parse().ok()).unwrap_or(40);
        let every: u32 = mode.strip_prefix("every:").and_then(|s| s.parse().ok()).unwrap_or(1);
        let grow = mode == "outstanding";
        (mode, target_ms, every, grow)
    };
    let (mut mode, mut target_ms, mut every, mut grow) = read_plan(&plan_path);
    let cfg = TciServerConfig { port, allow_tx: true, ..TciServerConfig::default() };
    let caps = DeviceCaps {
        rx_channels: 1,
        tx_channels: 1,
        freq_ranges_rx: vec![(100_000.0, 60_000_000.0)],
        freq_ranges_tx: vec![(100_000.0, 60_000_000.0)],
        ..DeviceCaps::default()
    };
    let snap = TciStateSnapshot {
        vfo_a_hz: 14_074_000.0,
        vfo_b_hz: 14_074_000.0,
        center_hz: 14_074_000.0,
        if_span_hz: 48_000.0,
        mode: Mode::Digu,
        drive_pct: 40,
        iq_rate: 0,
        vfo_lo_hz: 100_000.0,
        vfo_hi_hz: 60_000_000.0,
        can_tx: true,
        ..TciStateSnapshot::default()
    };
    let mut srv = TciServerController::start(&cfg, &caps, snap.clone()).expect("bind");
    println!("TCI server on 127.0.0.1:{port} — point a client at it and key up");

    let mut fifo: Vec<f32> = Vec::new();
    let mut block = [0.0f32; BLOCK];
    let mut keyed = false;
    let mut over_start = Instant::now();
    let mut over_got = 0usize;
    let mut over_played = 0usize;
    let mut over_padded = 0usize;
    let mut next = Instant::now();
    // The pacing state under test, in the engine's own terms.
    let mut outstanding = 0usize;
    let mut target = target_ms * RATE / 1000;
    let mut chronos = 0u32;
    let mut quiet = 0u32;
    let mut nblock = 0u32;
    loop {
        for r in srv.poll() {
            if let sdroxide_tci::server::ServerRequest::Key(k) = r {
                if k && !keyed {
                    println!("\n=== KEY DOWN ===");
                    over_start = Instant::now();
                    over_got = 0;
                    over_played = 0;
                    over_padded = 0;
                    outstanding = 0;
                    quiet = 0;
                    nblock = 0;
                    chronos = 0;
                    let p = read_plan(&plan_path);
                    mode = p.0;
                    target_ms = p.1;
                    every = p.2;
                    grow = p.3;
                    target = target_ms * RATE / 1000;
                    println!("plan: {mode} target={target_ms}ms");
                    fifo.clear();
                    srv.drain_tx_audio();
                } else if !k && keyed {
                    let secs = over_start.elapsed().as_secs_f64();
                    println!(
                        "=== KEY UP after {secs:.2}s: {over_got} frames from the client \
                         ({:.2}s of audio = {:.0}% of real time over {chronos} chronos), \
                         {over_played} played, {over_padded} silent, depth {target} ({} ms)",
                        over_got as f64 / RATE as f64,
                        over_got as f64 / RATE as f64 / secs * 100.0,
                        target * 1000 / RATE,
                    );
                }
                keyed = k;
            }
        }
        if !keyed {
            srv.broadcast_state(snap.clone());
            std::thread::sleep(Duration::from_millis(20));
            next = Instant::now();
            continue;
        }

        let mut got = 0usize;
        loop {
            let n = srv.read_tx_audio(&mut block);
            fifo.extend_from_slice(&block[..n]);
            got += n;
            if n < BLOCK {
                break;
            }
        }
        over_got += got;
        outstanding = outstanding.saturating_sub(got);
        quiet = if got > 0 { 0 } else { quiet + 1 };
        if quiet >= 50 {
            quiet = 0;
            outstanding = 0;
        }
        let held = if grow { fifo.len() + outstanding } else { fifo.len() };
        let deficit = target.saturating_sub(held);
        let due = nblock % every == 0;
        let asked = if deficit >= BLOCK && due {
            srv.request_chrono(deficit as u32);
            outstanding += deficit;
            chronos += 1;
            deficit
        } else {
            0
        };
        if fifo.len() > 24_000 {
            let cut = fifo.len() - 24_000;
            println!("  !! {cut} frames of the client's waveform thrown away (queue cap)");
            fifo.drain(..cut);
        }
        let take = fifo.len().min(BLOCK);
        over_played += take;
        fifo.drain(..take);
        if take < BLOCK {
            over_padded += BLOCK - take;
            if grow {
                target = (target + BLOCK).min(BLOCK * 25);
            }
        }
        if got > 0 || asked > 0 || nblock % 25 == 0 {
            println!(
                "  {:6.3}s got={got:<6} queued={:<6} asked={asked:<6} outstanding={outstanding:<6} \
                 target={target}",
                over_start.elapsed().as_secs_f64(),
                fifo.len(),
            );
        }
        nblock += 1;

        next += Duration::from_secs_f64(BLOCK as f64 / RATE as f64);
        if let Some(d) = next.checked_duration_since(Instant::now()) {
            std::thread::sleep(d);
        }
    }
}
