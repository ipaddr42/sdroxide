//! Diagnostic: does `fobos_rx_set_samplerate` actually honor a rate below
//! the ADC's own top rate while direct sampling is enabled? `open_hf`
//! originally assumed not and always requested the top rate — real hardware
//! says otherwise: every rate down to 10 Msps landed exactly, and only
//! requests below `HF_ADC_RATE_FLOOR` (8 Msps on the unit this was checked
//! against) snapped up to it. `open_hf`'s own `hf_adc_rate_wanted` is built
//! on that finding.
//!
//! ```text
//! cargo run -p sdroxide-fobos --example hf_rate_probe
//! ```
//! Cycles the samplerate several times on one open session — give the
//! device a moment before opening it again afterward, same as any other
//! close/reopen (see `device.rs`'s own reopen-timing note).

use sdroxide_fobos::device;

fn main() {
    let (api, dev, info) = match device::open("") {
        Ok(v) => v,
        Err(e) => {
            println!("open failed: {e}");
            return;
        }
    };
    println!("opened {}", info.serial);

    let rates = unsafe { device::samplerates(&api, dev) }.unwrap_or_default();
    println!("rates_hz offered: {rates:?}");

    let rc = unsafe { (api.set_direct_sampling)(dev, 1) };
    println!("set_direct_sampling(1): rc={rc}");

    for want in [80_000_000.0, 40_000_000.0, 20_000_000.0, 10_000_000.0, 5_000_000.0, 2_500_000.0] {
        let mut actual = 0.0f64;
        let rc = unsafe { (api.set_samplerate)(dev, want, &mut actual) };
        println!("requested {:>10.3} Msps -> rc={rc}, actual={:.3} Msps", want / 1e6, actual / 1e6);
    }

    unsafe { device::close(&api, dev) };
    println!("closed cleanly");
}
