//! Message bodies to numbers: I/Q to interleaved `f32`, FFT bytes to dB.
//!
//! Kept away from the socket so the arithmetic can be tested against literals.
//! It is worth testing: a sign, a divide where the reference multiplies, or a
//! forgotten offset all produce a stream that looks plausible and is wrong by
//! a constant, which is the hardest kind of bug to see on a waterfall.
//!
//! Bodies are self-contained — the framer has already reassembled a whole
//! message — so nothing here needs to carry bytes between calls. A body that
//! is not a whole number of samples is a protocol violation rather than a
//! segment boundary, and the remainder is dropped.

use sdroxide_types::SpyServerFormat;

/// Decode one I/Q body into interleaved `f32`, undoing the digital gain the
/// server applied.
///
/// `gain` is the *linear* factor from the message header's flags — what the
/// server did, never what it was asked to do. See
/// [`crate::proto::MessageHeader::digital_gain`].
///
/// Returns the number of complex samples written.
pub fn iq_to_f32(format: SpyServerFormat, body: &[u8], gain: f32, out: &mut Vec<f32>) -> usize {
    out.clear();
    // Bytes in one complete complex sample.
    let stride = format.bytes_per_sample();
    let whole = body.len() / stride * stride;
    out.reserve(whole / (stride / 2));

    match format {
        SpyServerFormat::Uint8 => {
            // Offset binary, mid-scale at 128. The scale folds the gain in so
            // the hot loop is one multiply.
            let scale = 1.0 / (gain * 128.0);
            for b in &body[..whole] {
                out.push((f32::from(*b) - 128.0) * scale);
            }
        }
        SpyServerFormat::Int16 => {
            let scale = 1.0 / (gain * 32768.0);
            for w in body[..whole].chunks_exact(2) {
                out.push(f32::from(i16::from_le_bytes([w[0], w[1]])) * scale);
            }
        }
        SpyServerFormat::Float32 => {
            // A divide, like the integer formats — and *not* what SDR++ does.
            // Its client multiplies floats by the header gain on the belief
            // that the server pre-attenuates them, and this branch used to copy
            // that. Measured against a real server it is the other way round:
            // a float stream asked for 20 dB arrives with header flags of 20
            // and its samples ten times larger — the same raw values as the
            // int16 stream at 20 dB, noise floor 20 dB up. So the multiply
            // applied the gain twice. Nobody notices at 0 dB, which is what an
            // undecimated stream opens at; with the gain set by hand, or by
            // the client's own loop, it is hard clipping.
            let scale = 1.0 / gain;
            for w in body[..whole].chunks_exact(4) {
                out.push(f32::from_le_bytes([w[0], w[1], w[2], w[3]]) * scale);
            }
        }
    }

    if whole != body.len() {
        tracing::warn!(
            "SpyServer: a {}-byte I/Q body is not a whole number of {}-byte samples; \
             dropped the {} trailing byte(s)",
            body.len(),
            stride,
            body.len() - whole,
        );
    }
    out.len() / 2
}

/// Decode one 8-bit FFT body into dBFS bins.
///
/// The server quantises its bins into the window this client asked for: byte
/// 255 is `db_offset`, byte 0 is `db_offset - db_range`. Both figures must be
/// the ones actually sent — see [`crate::proto::clamp_fft_window`] — or every
/// bin is out by the difference and the whole strip is shifted with nothing to
/// show for it.
pub fn fft_to_db(body: &[u8], db_offset: f64, db_range: f64, out: &mut Vec<f32>) {
    out.clear();
    out.reserve(body.len());
    let offset = db_offset as f32;
    let range = db_range as f32;
    for b in body {
        out.push(offset - range * (1.0 - f32::from(*b) / 255.0));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eight_bit_iq_is_offset_binary_around_128() {
        let mut out = Vec::new();
        let n = iq_to_f32(SpyServerFormat::Uint8, &[128, 128, 255, 0], 1.0, &mut out);
        assert_eq!(n, 2);
        assert_eq!(out[0], 0.0, "mid-scale is zero");
        assert_eq!(out[1], 0.0);
        assert!((out[2] - 127.0 / 128.0).abs() < 1e-6);
        assert_eq!(out[3], -1.0);
    }

    #[test]
    fn sixteen_bit_iq_is_signed_full_scale() {
        let mut out = Vec::new();
        let body: Vec<u8> = [0i16, -32768, 16384, 0].iter().flat_map(|v| v.to_le_bytes()).collect();
        let n = iq_to_f32(SpyServerFormat::Int16, &body, 1.0, &mut out);
        assert_eq!(n, 2);
        assert_eq!(out[0], 0.0);
        assert_eq!(out[1], -1.0);
        assert_eq!(out[2], 0.5);
    }

    /// The header's gain is undone the same way in every format. The float
    /// path used to multiply instead — copied from SDR++, on the belief that
    /// the server pre-attenuates floats — and a real server amplifies them
    /// exactly as it does the integers, so that applied the gain twice.
    /// Asserted against literals precisely so that a multiply slipping back
    /// into the float branch is caught.
    #[test]
    fn the_digital_gain_in_the_flags_is_undone_per_format() {
        // 20 dB is a linear factor of ten.
        let gain = 10.0f32;
        let mut out = Vec::new();

        iq_to_f32(SpyServerFormat::Uint8, &[255, 128], gain, &mut out);
        assert!((out[0] - (127.0 / 128.0) / 10.0).abs() < 1e-6, "8-bit divides by the gain");

        let body: Vec<u8> = [16384i16, 0].iter().flat_map(|v| v.to_le_bytes()).collect();
        iq_to_f32(SpyServerFormat::Int16, &body, gain, &mut out);
        assert!((out[0] - 0.05).abs() < 1e-6, "16-bit divides by the gain");

        // The same 0.5 full scale, as the server sends it after 20 dB of gain:
        // a float of 0.5, which has to come out as the 0.05 the int16 did.
        let body: Vec<u8> = [0.5f32, 0.0].iter().flat_map(|v| v.to_le_bytes()).collect();
        iq_to_f32(SpyServerFormat::Float32, &body, gain, &mut out);
        assert!((out[0] - 0.05).abs() < 1e-6, "float divides by the gain too");

        // A gain of unity is the identity in every format.
        iq_to_f32(SpyServerFormat::Uint8, &[255, 128], 1.0, &mut out);
        assert!((out[0] - 127.0 / 128.0).abs() < 1e-6);
    }

    /// A truncated body is a protocol violation, not a segment boundary — the
    /// framer only hands over whole messages — so the remainder is dropped
    /// rather than carried, and the pairs that did arrive stay aligned.
    #[test]
    fn a_body_that_is_not_whole_samples_keeps_i_with_q() {
        let mut out = Vec::new();
        // Five bytes at 8-bit: two complete pairs and a stray.
        let n = iq_to_f32(SpyServerFormat::Uint8, &[128, 128, 128, 128, 200], 1.0, &mut out);
        assert_eq!(n, 2);
        assert_eq!(out.len() % 2, 0);
    }

    #[test]
    fn fft_bytes_map_onto_the_window_that_was_requested() {
        let mut out = Vec::new();
        fft_to_db(&[0, 128, 255], 0.0, 150.0, &mut out);
        assert!((out[0] - -150.0).abs() < 1e-4, "byte 0 is the bottom of the range");
        assert!((out[2] - 0.0).abs() < 1e-4, "byte 255 is the offset");
        assert!((out[1] - -74.7).abs() < 0.1, "and it is linear in between");

        // A window that is not anchored at zero shifts with the offset.
        fft_to_db(&[0, 255], -20.0, 100.0, &mut out);
        assert!((out[0] - -120.0).abs() < 1e-4);
        assert!((out[1] - -20.0).abs() < 1e-4);
    }
}
