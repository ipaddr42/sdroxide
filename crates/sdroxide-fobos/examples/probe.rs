//! Open whatever Fobos SDR is attached and print everything the crate
//! skeleton can ask it for.
//!
//! The first thing to build and the first thing to run against real
//! hardware: it settles that the library loads, enumerates, and opens
//! before any of the streaming/DDC work built on top of it has to be
//! right. There is no streaming here, so this stops at board info and
//! the sample-rate list — see `examples/stream_probe.rs` for that.
//!
//! With no board attached it still exercises the useful half — that
//! `libfobos` loads and the symbols resolve.
//!
//! ```text
//! cargo run -p sdroxide-fobos --example probe
//! ```

fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "sdroxide_fobos=debug".into()),
        )
        .init();

    let found = match sdroxide_fobos::try_list() {
        Err(e) => {
            println!("libfobos unavailable: {e}");
            return;
        }
        Ok(found) => found,
    };
    println!("libfobos listed {} device(s)", found.len());
    for d in &found {
        println!("  dev# {}  {}", d.index, d.serial);
    }
    if found.is_empty() {
        println!("\nNothing to open. Is a Fobos SDR plugged in?");
        return;
    }

    let (api, dev, chosen) = match sdroxide_fobos::open("") {
        Ok(v) => v,
        Err(e) => {
            println!("\nopen failed: {e}");
            return;
        }
    };
    println!("\n--- opened dev# {} ({}) ---", chosen.index, chosen.serial);

    // SAFETY: `dev` is the still-open handle `open()` just returned, and
    // nothing below touches it again after `close()`.
    unsafe {
        match sdroxide_fobos::board_info(&api, dev) {
            Ok(info) => {
                println!("hw_revision:  {}", info.hw_revision);
                println!("fw_version:   {}", info.fw_version);
                println!("manufacturer: {}", info.manufacturer);
                println!("product:      {}", info.product);
                println!("serial:       {}", info.serial);
            }
            Err(e) => println!("board_info failed: {e}"),
        }

        match sdroxide_fobos::samplerates(&api, dev) {
            Ok(rates) => {
                print!("sample rates: ");
                let text: Vec<String> =
                    rates.iter().map(|r| format!("{:.3} Msps", r / 1e6)).collect();
                println!("{}", text.join(", "));
            }
            Err(e) => println!("samplerates failed: {e}"),
        }

        sdroxide_fobos::close(&api, dev);
    }
    println!("\nclosed cleanly");
}
