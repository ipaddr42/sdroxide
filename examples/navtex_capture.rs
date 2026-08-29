//! Write an I/Q capture carrying a synthetic NAVTEX broadcast.
//!
//! For checking the whole receive path — file source, receive chain, decoder,
//! panel — without a receiver on 518 kHz and without waiting four hours for a
//! slot. The signal is the one `sdroxide_dsp::navtex_test` builds, put on a
//! carrier 1700 Hz above the file's centre so a dial tuned to the centre in
//! USB lands the tones where the decoder looks for them.
//!
//! `cargo run --release --example navtex_capture -- /tmp/navtex.wav`

use sdroxide_dsp::navtex_test::{encode_bits, synth};
use sdroxide_radio::{Complex32, iq_wav::IqWavWriter};

const RATE: u32 = 48_000;
/// The dial a receiver would be on for the 518 kHz service.
const CENTER: f64 = 516_300.0;

fn main() {
    let path = std::env::args().nth(1).unwrap_or_else(|| "/tmp/navtex.wav".into());
    let msgs = [
        "ZCZC FA12\r\nGALE WARNING\r\nNORTH SEA\r\nSOUTHWEST 8 TO GALE 9 VEERING WEST 6 TO 7\r\nNNNN",
        "ZCZC OB07\r\nNAVIGATIONAL WARNING\r\nBUOY 5203N 00412E UNLIT\r\nNNNN",
    ];
    let mut w = IqWavWriter::create(std::path::Path::new(&path), RATE, CENTER).expect("create");
    for (i, m) in msgs.iter().enumerate() {
        // A quiet minute either side, so the decoder has to find the signal
        // rather than being handed it — which is the case that matters.
        let gap = vec![Complex32::new(0.0, 0.0); RATE as usize];
        w.write(&gap).expect("write");
        let audio = synth(&encode_bits(m), f64::from(RATE), 1700.0, 0.35);
        // Real audio onto a complex baseband: the decoder mixes it back down.
        let iq: Vec<Complex32> = audio.iter().map(|&a| Complex32::new(a, 0.0)).collect();
        w.write(&iq).expect("write");
        println!("message {} written", i + 1);
    }
    let p = w.finish().expect("finish");
    println!("wrote {}", p.display());
}
