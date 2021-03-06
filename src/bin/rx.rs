// Rust binding to libairspy
// Copyright 2015 Adam Greig <adam@adamgreig.com>
// Licensed under the MIT License.

extern crate airspy;

use std::fs::File;
use std::io::Write;
use std::sync::mpsc;
use airspy::{Airspy,IQ};

#[cfg_attr(test, allow(dead_code))]
fn open_airspy() -> Airspy {
    println!("\nOpening Airspy:");
    let mut dev = Airspy::new().unwrap();

    println!("    Got board type:   {}", dev.get_board_id().unwrap());
    println!("    Firmware version: {}", dev.get_version().unwrap());
    let part = dev.get_partid_serial().unwrap();
    println!("    Part ID:          0x{:08X} 0x{:08X}",
             part.part_id[0], part.part_id[1]);
    println!("    Serial Number:    0x{:08X}", part.serial_no);
    
    dev
}

#[cfg_attr(test, allow(dead_code))]
fn conf_airspy(dev: &mut Airspy) {
    println!("\nConfiguring Airspy:");

    println!("    Setting sample rate to 10Msps");
    dev.set_sample_rate(10_000_000).unwrap();

    println!("    Setting frequency to 434MHz");
    dev.set_freq(434000000).unwrap();

    println!("    Disabling LNA/mixer AGC");
    dev.set_lna_agc(false).unwrap();
    dev.set_mixer_agc(false).unwrap();

    println!("    Enabling output bias");
    dev.set_rf_bias(true).unwrap();

    println!("    Setting LNA/mixer/BB gains to 0/0/0dB");
    dev.set_lna_gain(0).unwrap();
    dev.set_mixer_gain(0).unwrap();
    dev.set_vga_gain(0).unwrap();
}

#[cfg_attr(test, allow(dead_code))]
fn main() {
    airspy::init().unwrap();
    let version = airspy::lib_version();
    println!("Using libairspy version: {}.{}.{}",
             version.major, version.minor, version.revision);

    // Open and configure the Airspy
    let mut dev = open_airspy();
    conf_airspy(&mut dev);

    // Open and configure file for writing
    let mut f = File::create("airspy.iq").unwrap();

    // Make a channel for talking to the callback
    let (tx_data, rx_data) = mpsc::channel();

    // Start streaming data
    println!("\nCollecting data...");
    dev.start_rx::<IQ<f32>>(tx_data).unwrap();

    // Receive data
    let mut n_samples = 0usize;
    while dev.is_streaming() {
        // Fetch buffer from callback
        let samples = rx_data.recv().unwrap();
        n_samples += samples.len();

        // Turn buffer into a u8 of the appropriate length
        let buf: &[u8] = unsafe {
            std::slice::from_raw_parts(
                std::mem::transmute(&samples[0].re as *const f32),
                samples.len() * std::mem::size_of::<IQ<f32>>())
        };

        // Write buffer to file
        f.write_all(buf).unwrap();

        // Quit after 50M samples
        if n_samples > 50_000_000 {
            println!("Collected {} samples, stopping.", n_samples);
            // When this thread finishes and rx_data goes out of scope, it
            // hangs up the channel, prompting the callback to return 'stop' to
            // libairspy, quitting sampling.
            break;
        }
    }

    println!("All done, exiting.");
}
