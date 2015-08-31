# Rust bindings for libairspy

## Prereqs

Requires libairspy at least 1.0.6 already built and installed on your system. 
Tests require an attached Airspy (otherwise there is not much to test!) and 
must be run with `RUST_TEST_THREADS=1` or the threads may stomp on each other.

## Outline

The basic gist is:

1. Make new Airspy object
2. Configure it using set_sample_rate, set_freq, etc
3. Create a MPSC channel for data transfer
4. Use `start_rx<T>` to begin transfer to your channel, specifying desired 
   sample type with `T` (see below).
5. Call `rx.recv()` on your side to read in the data buffers
6. When your half of the channel is destroyed, the Airspy is stopped

## Sample Types

Sample types are set by the `T` in `start_rx<T>`. It can be any of:

* `IQ<f32>`
* `IQ<i16>`
* `Real<f32>`
* `Real<i16>`
* `Real<u16>`

The type is passed to libairspy which does the conversion from the native 
`Real<u16>`. Note that `Real` types give you twice as many samples per time 
period than `IQ`. Also note that `Real<T>` is just an alias for `T` and `IQ<T>` 
for `Complex<T>`, so all the usual maths operations are available for both 
types.

## Example

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

See `src/bin/rx.rs` for a complete example of logging IQ data to file, or 
[asfmrs](https://github.com/adamgreig/asfmrs) for an example application 
(narrowband FM demodulation to audio output).
