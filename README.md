# Rust bindings for libairspy

Requires libairspy at least 1.0.6 already built and installed on your system. 
Tests require an attached Airspy (otherwise there is not much to test!) and 
must be run with `RUST_TEST_THREADS=1` or the threads may stomp on each other.

Simple usage example:

```rust
extern crate airspy;
use airspy::{Airspy,IQ};
use std::sync::mpsc;

// Open first available Airspy and configure for floating IQ samples,
// 10Msps, 434MHz (can also set gains, AGC, RF bias etc).
let mut dev = Airspy::new().unwrap();
dev.set_sample_rate(0).unwrap();
dev.set_freq(434000000).unwrap();

// Samples are sent back to us from the C thread by an MPSC channel
let (tx, rx) = mpsc::channel();

// Begin receiving data. Need the type hint either here or when creating the
// channel, which must match with the sample type selected earlier.
dev.start_rx::<IQ<f32>>(tx).unwrap();

// Blocking receive samples, stop after 10M. When the `rx` object goes out of
// scope and is destroyed, the `tx` detects the hangup and tells libairspy to
// stop data reception.
let mut n_samples = 0usize;
while dev.is_streaming() {
    let samples = rx.recv().unwrap();
    n_samples += samples.len() / 2;
    println!("Got {} samples so far", n_samples);
    if n_samples > 10_000_000 {
        break;
    }
}
```

See `src/bin/rx.rs` for a complete example of usage.
