//! Two DDCs on one Protocol 2 connection, against a scripted board.
//!
//! The fake radio answers the discovery probe as a Saturn, parses the DDC
//! command's enable mask, streams distinct IQ for each enabled DDC from its
//! own source port (1035 + n), and reports the high-priority packet's per-DDC
//! NCO phases. The client must route every stream to its own ring, tune each
//! DDC independently, refuse what the board doesn't have, and detach one
//! stream without pausing the other.
//!
//! Uses the real Protocol 2 ports (1024..1027, 1035..) on 127.0.0.1 — they are
//! unprivileged — and skips rather than fails if any is taken (a real HPSDR
//! program running locally would hold them).

use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender};
use sdroxide_hpsdr::{CLOCK_HZ, HpsdrBoard, phase_word};

/// Exactly representable in the 24-bit fixed-point coding, so `expect_fill`
/// can compare for equality after the round trip.
const DDC0_FILL: f32 = 0.25;
const DDC1_FILL: f32 = 0.75;
const DDC0_AFTER_DETACH_FILL: f32 = 0.5;

/// What the fake radio tells the test as the client drives it.
enum Seen {
    /// A DDC command's enable mask.
    Mask(u8),
    /// A high-priority packet's (DDC0, DDC1) NCO phase words.
    Phases(u32, u32),
}

struct FakeRadio {
    general: UdpSocket,
    ddc_cmd: UdpSocket,
    duc_cmd: UdpSocket,
    high_prio: UdpSocket,
    iq: [UdpSocket; 2],
    seen: Sender<Seen>,
    stop: Arc<AtomicBool>,
}

fn bind(port: u16) -> Option<UdpSocket> {
    match UdpSocket::bind(("127.0.0.1", port)) {
        Ok(s) => {
            s.set_read_timeout(Some(Duration::from_millis(5))).ok();
            Some(s)
        }
        Err(e) => {
            eprintln!("skip: cannot bind 127.0.0.1:{port} ({e})");
            None
        }
    }
}

/// One DDC IQ datagram: 4-byte sequence, sample count at [14..16], then
/// 24-bit big-endian I/Q pairs — `fill` encoded exactly.
fn iq_datagram(seq: u32, fill: f32, pairs: usize) -> Vec<u8> {
    let mut b = vec![0u8; 16 + pairs * 6];
    b[0..4].copy_from_slice(&seq.to_be_bytes());
    b[14..16].copy_from_slice(&(pairs as u16).to_be_bytes());
    let v = (fill * 8_388_608.0) as i32;
    let s = [(v >> 16) as u8, (v >> 8) as u8, v as u8];
    for p in 0..pairs {
        let off = 16 + p * 6;
        b[off..off + 3].copy_from_slice(&s);
        b[off + 3..off + 6].copy_from_slice(&s);
    }
    b
}

impl FakeRadio {
    fn run(self) {
        let mut buf = [0u8; 2048];
        let mut last_mask = 0u8;
        let mut seqs = [0u32; 2];

        while !self.stop.load(Ordering::Relaxed) {
            // Discovery + general/run command.
            if let Ok((n, src)) = self.general.recv_from(&mut buf) {
                // Protocol 2 discovery request: command 0x02 at offset 4 (the
                // P1 request starts 0xEF 0xFE and is ignored).
                if n >= 60 && buf[0] != 0xEF && buf[4] == 0x02 {
                    let mut reply = [0u8; 60];
                    reply[4] = 0x02; // idle
                    reply[5..11].copy_from_slice(&[0x00, 0x1C, 0xC0, 0x00, 0x00, 0x01]);
                    reply[11] = 10; // Saturn
                    let _ = self.general.send_to(&reply, src);
                }
            }
            // DUC command: drained, nothing asserted.
            while self.duc_cmd.recv_from(&mut buf).is_ok() {}
            // High-priority: report the first two DDC NCO phases.
            while let Ok((n, _)) = self.high_prio.recv_from(&mut buf) {
                if n >= 17 {
                    let p0 = u32::from_be_bytes([buf[9], buf[10], buf[11], buf[12]]);
                    let p1 = u32::from_be_bytes([buf[13], buf[14], buf[15], buf[16]]);
                    let _ = self.seen.send(Seen::Phases(p0, p1));
                }
            }
            // DDC command: the enable mask decides what streams. Its source
            // is the client's main socket — the discovery probe uses a socket
            // of its own, so port 1024's sources are not the IQ target.
            while let Ok((n, src)) = self.ddc_cmd.recv_from(&mut buf) {
                if n < 8 {
                    continue;
                }
                let target = src;
                let mask = buf[7];
                let _ = self.seen.send(Seen::Mask(mask));
                // Stream a burst for each enabled DDC we model, from its own
                // source port — and when a DDC has just been switched off,
                // one more DDC0 frame proves the survivor keeps flowing.
                for ddc in 0..2u8 {
                    if mask & (1 << ddc) != 0 {
                        let fill = if ddc == 0 { DDC0_FILL } else { DDC1_FILL };
                        for _ in 0..2 {
                            let pkt = iq_datagram(seqs[ddc as usize], fill, 64);
                            seqs[ddc as usize] += 1;
                            let _ = self.iq[ddc as usize].send_to(&pkt, target);
                        }
                    }
                }
                if last_mask & 0x02 != 0 && mask & 0x02 == 0 && mask & 0x01 != 0 {
                    let pkt = iq_datagram(seqs[0], DDC0_AFTER_DETACH_FILL, 64);
                    seqs[0] += 1;
                    let _ = self.iq[0].send_to(&pkt, target);
                }
                last_mask = mask;
            }
        }
    }
}

/// Wait until the fake radio has seen a DDC enable mask of `wanted`.
fn expect_mask(seen: &Receiver<Seen>, wanted: u8) {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut masks = Vec::new();
    while Instant::now() < deadline {
        while let Ok(s) = seen.try_recv() {
            if let Seen::Mask(m) = s {
                if m == wanted {
                    return;
                }
                masks.push(m);
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("the board never saw enable mask {wanted:#04b}; it saw {masks:?}");
}

/// Wait until a high-priority packet carries `phase` for DDC `ddc` (0 or 1).
fn expect_phase(seen: &Receiver<Seen>, ddc: u8, phase: u32) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while Instant::now() < deadline {
        while let Ok(s) = seen.try_recv() {
            if let Seen::Phases(p0, p1) = s {
                let got = if ddc == 0 { p0 } else { p1 };
                if got == phase {
                    return;
                }
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("the board never saw DDC{ddc} phase {phase:#010X}");
}

/// Wait until `rx` yields a sample equal to `fill`.
fn expect_fill(rx: &mut sdroxide_hpsdr::HpsdrRx, fill: f32, what: &str) {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut buf = [0.0f32; 512];
    let mut seen = Vec::new();
    while Instant::now() < deadline {
        let n = rx.rx_read(&mut buf);
        for &v in &buf[..n] {
            if (v - fill).abs() < 1e-6 {
                return;
            }
            if seen.len() < 8 && !seen.contains(&v) {
                seen.push(v);
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    panic!("{what}: never saw fill {fill}; distinct values seen: {seen:?}");
}

/// Everything of the arrangement in one scripted session — the fake board is
/// a single conversation, and the real Protocol 2 ports can only be bound
/// once.
#[test]
fn two_ddcs_share_one_connection_and_detach_independently() {
    let Some(general) = bind(1024) else { return };
    let Some(ddc_cmd) = bind(1025) else { return };
    let Some(duc_cmd) = bind(1026) else { return };
    let Some(high_prio) = bind(1027) else { return };
    let Some(iq0) = bind(1035) else { return };
    let Some(iq1) = bind(1036) else { return };

    let (seen_tx, seen) = crossbeam_channel::unbounded();
    let stop = Arc::new(AtomicBool::new(false));
    let radio = FakeRadio {
        general,
        ddc_cmd,
        duc_cmd,
        high_prio,
        iq: [iq0, iq1],
        seen: seen_tx,
        stop: Arc::clone(&stop),
    };
    let server = std::thread::spawn(move || radio.run());

    let board = HpsdrBoard::open(
        "127.0.0.1".parse().unwrap(),
        384_000.0,
        0.0,
        sdroxide_types::HpsdrOcPlan::none(),
        false,
        true,
        sdroxide_types::HpsdrIoRxInput::Radio,
        // Automatic overload protection off: these tests drive the wire, not
        // the loop that rides on it.
        sdroxide_hpsdr::AutoGain::new(false, 1.0, 100, 10_000, -12.0, 48.0),
    )
    .expect("open");
    assert_eq!(board.protocol(), 2);
    assert_eq!(board.board(), "Saturn (ANAN-G2)");
    assert_eq!(board.ddc_count(), 8);

    // A DDC beyond the framing is refused with the count.
    let err = board.rx(8).expect_err("DDC 8 of 8 must be refused").to_string();
    assert!(err.contains("8 receiver"), "the refusal should carry the count: {err}");

    let mut r0 = board.rx(0).expect("attach DDC 0");
    let mut r1 = board.rx(1).expect("attach DDC 1");
    expect_mask(&seen, 0b0000_0011);

    // A DDC cannot be attached twice — two engines draining one ring would
    // each get half the samples.
    let err = board.rx(1).expect_err("duplicate DDC 1 must be refused").to_string();
    assert!(err.contains("already"), "{err}");

    // Each DDC's stream lands in its own ring, not the other's.
    expect_fill(&mut r0, DDC0_FILL, "DDC 0");
    expect_fill(&mut r1, DDC1_FILL, "DDC 1");

    // Tuning is per DDC: DDC 1's NCO moves in the frequency table while
    // DDC 0's holds its own value.
    r0.set_rx_freq(7_100_000.0);
    r1.set_rx_freq(14_074_000.0);
    expect_phase(&seen, 0, phase_word(7_100_000.0, CLOCK_HZ));
    expect_phase(&seen, 1, phase_word(14_074_000.0, CLOCK_HZ));

    // Detaching DDC 1 shrinks the enable mask to DDC 0 alone…
    drop(r1);
    expect_mask(&seen, 0b0000_0001);
    // …and DDC 0 streams on: the frame the board sent after the mask change
    // arrives, and the DDC can be attached again.
    expect_fill(&mut r0, DDC0_AFTER_DETACH_FILL, "DDC 0 after DDC 1 detached");
    assert!(r0.is_alive());
    let r1_again = board.rx(1).expect("re-attach DDC 1 after detach");
    drop(r1_again);

    drop(r0);
    drop(board);
    stop.store(true, Ordering::Relaxed);
    server.join().expect("fake radio thread");
}
