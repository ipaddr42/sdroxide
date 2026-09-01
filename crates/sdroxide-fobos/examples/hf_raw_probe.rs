//! Throwaway diagnostic: enable direct sampling and dump raw per-channel
//! statistics, to settle empirically whether the HF-port buffer carries
//! genuine I/Q (RF-port style) or the two separate real ADC channels
//! (HF1/HF2) interleaved — the open question that had to be settled before
//! any DDC got built on top of an assumption (see `stream.rs`'s own module
//! doc for how it was settled, against the vendor library's C source).
//!
//! ```text
//! cargo run -p sdroxide-fobos --example hf_raw_probe
//! ```

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_fobos=debug".into()),
        )
        .init();

    let (api, dev, chosen) = match sdroxide_fobos::open("") {
        Ok(v) => v,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    println!("opened {}", chosen.serial);

    let rc = unsafe { (api.set_direct_sampling)(dev, 1) };
    println!("set_direct_sampling(1): rc={rc}");

    let mut actual = 0.0f64;
    let rc = unsafe { (api.set_samplerate)(dev, 80_000_000.0, &mut actual) };
    println!("set_samplerate(80e6): rc={rc} actual={actual}");

    let rc = unsafe { (api.set_lna_gain)(dev, 2) };
    println!("set_lna_gain(2): rc={rc}");
    let rc = unsafe { (api.set_vga_gain)(dev, 15) };
    println!("set_vga_gain(15): rc={rc}");

    let buf_len: u32 = (actual / 400.0).round() as u32;
    let rc = unsafe { (api.start_sync)(dev, buf_len) };
    println!("start_sync({buf_len}): rc={rc}");

    let mut scratch = vec![0.0f32; buf_len as usize * 2];
    for block in 0..5 {
        let mut got: u32 = 0;
        let rc = unsafe { (api.read_sync)(dev, scratch.as_mut_ptr(), &mut got) };
        if rc != 0 {
            println!("read_sync: rc={rc} ({})", api.err_text(rc));
            continue;
        }
        let n = got as usize;
        let mut re_min = f32::MAX;
        let mut re_max = f32::MIN;
        let mut im_min = f32::MAX;
        let mut im_max = f32::MIN;
        let mut re_sum_abs = 0.0f64;
        let mut im_sum_abs = 0.0f64;
        // Correlation-ish: how similar are re and im, sample for sample?
        let mut cross = 0.0f64;
        for pair in scratch[..n * 2].as_chunks::<2>().0 {
            let (re, im) = (pair[0], pair[1]);
            re_min = re_min.min(re);
            re_max = re_max.max(re);
            im_min = im_min.min(im);
            im_max = im_max.max(im);
            re_sum_abs += re.abs() as f64;
            im_sum_abs += im.abs() as f64;
            cross += (re * im) as f64;
        }
        println!(
            "block {block}: n={n} re[{re_min:.5},{re_max:.5}] mean|re|={:.6} \
             im[{im_min:.5},{im_max:.5}] mean|im|={:.6} cross_sum={cross:.3}",
            re_sum_abs / n as f64,
            im_sum_abs / n as f64,
        );
        // First few raw pairs, so a genuinely-zeroed channel is visible
        // directly rather than just inferred from min/max.
        if block == 0 {
            print!("  first 8 pairs: ");
            for pair in scratch[..16.min(n * 2)].as_chunks::<2>().0 {
                print!("({:.5},{:.5}) ", pair[0], pair[1]);
            }
            println!();
        }
    }

    let _ = unsafe { (api.stop_sync)(dev) };
    unsafe { sdroxide_fobos::close(&api, dev) };
    println!("closed");
}
