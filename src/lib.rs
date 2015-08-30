// Rust binding to libairspy
// Copyright 2015 Adam Greig <adam@adamgreig.com>
// Licensed under the MIT License.

use std::ffi::CStr;
use std::result;
use std::str;
use std::sync::mpsc::Sender;

mod ffi;

/// Store a reference to an opened Airspy device.
pub struct Airspy {
    ptr: *mut ffi::airspy_device
}

/// Ensure whenever the Airspy goes out of scope it is closed.
impl Drop for Airspy {
    #[inline(never)]
    fn drop(&mut self) {
        unsafe { ffi::airspy_close(self.ptr); }
    }
}

/// Hold major, minor and revision version numbers of libairspy C library.
pub struct LibVersion {
    pub major: u32,
    pub minor: u32,
    pub revision: u32
}

/// Hold an Airspy's part ID and serial number.
pub struct PartID {
    pub part_id: [u32; 2],
    pub serial_no: u64
}

/// Choice of sample type.
#[allow(non_camel_case_types)]
pub enum SampleType {
    f32IQ,
    f32Real,
    i16IQ,
    i16Real,
    u16Real
}

/// Error type for libairspy functions.
#[derive(Debug)]
pub struct FFIError {
    errno: ffi::airspy_error
}

impl FFIError {
    fn new(err: ffi::airspy_error) -> FFIError {
        FFIError { errno: err }
    }

    fn errstr(&self) -> &str {
        let cstr = unsafe {
            CStr::from_ptr(ffi::airspy_error_name(self.errno))
        };
        str::from_utf8(cstr.to_bytes()).unwrap()
    }
}

impl std::fmt::Display for FFIError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Airspy Error: {}", self.errstr())
    }
}

impl std::error::Error for FFIError {
    fn description(&self) -> &str {
        self.errstr()
    }
}

/// Result type for libairspy functions.
pub type Result<T> = result::Result<T, FFIError>;

/// Fetch the current version of libairspy C library.
pub fn lib_version() -> LibVersion {
    let mut v = ffi::airspy_lib_version_t {
        major_version: 0, minor_version: 0, revision: 0};
    unsafe { ffi::airspy_lib_version(&mut v); }

    LibVersion {
        major: v.major_version, minor: v.minor_version, revision: v.revision}
}

macro_rules! ffifn {
    ($f:expr, $r:expr) => (
        match unsafe { $f } {
            ffi::airspy_error::AIRSPY_SUCCESS => Ok($r),
            err => Err(FFIError::new(err))
        }
    );
    ($f:expr) => (
        ffifn!($f, ())
    );
}

/// Callback in Rust to send to the libairspy C library that sends buffers
/// through to a user channel, quitting streaming when the channel hangs up.
extern "C" fn rx_cb<T>(transfer: *mut ffi::airspy_transfer_t) -> ffi::c_int
    where T: Clone
{
    let transfer = unsafe { &*transfer };
    let sample_count = transfer.sample_count as usize;
    let iq_multiplier = match transfer.sample_type {
        ffi::airspy_sample_type::AIRSPY_SAMPLE_FLOAT32_IQ => 2,
        ffi::airspy_sample_type::AIRSPY_SAMPLE_FLOAT32_REAL => 1,
        ffi::airspy_sample_type::AIRSPY_SAMPLE_INT16_IQ => 2,
        ffi::airspy_sample_type::AIRSPY_SAMPLE_INT16_REAL => 1,
        ffi::airspy_sample_type::AIRSPY_SAMPLE_UINT16_REAL => 1,
        ffi::airspy_sample_type::AIRSPY_SAMPLE_END => unreachable!()
    };

    let buffer = unsafe {
        std::slice::from_raw_parts(transfer.samples as *const T,
                                   sample_count * iq_multiplier)
    }.to_vec();

    // Turn the ctx into a &Sender and send the buffer along it.
    // If it works, keep asking for more samples, but if not, we'll quit.
    let sender: &Sender<Vec<T>> = unsafe { &*(transfer.ctx as *const _)};
    match sender.send(buffer) {
        Ok(_) => 0,
        Err(_) => {
            // Drop the Sender to prevent leaks,
            // then tell libairspy to stop streaming.
            let boxed: Box<Sender<Vec<T>>> = unsafe {
                std::mem::transmute(transfer.ctx as *const _) };
            std::mem::drop(boxed);
            1
        }
    }
}


/// Initialise the Airspy library. Call once at application startup.
pub fn init() -> Result<()> {
    ffifn!(ffi::airspy_init())
}

/// Deinitialise the Airspy library. Call once at application end.
pub fn exit() -> Result<()> {
    ffifn!(ffi::airspy_exit())
}

impl Airspy {
    /// Try to open the next available Airspy device.
    pub fn new() -> Result<Airspy> {
        let mut device: Airspy = Airspy{ ptr: unsafe { std::mem::zeroed() }};
        ffifn!(ffi::airspy_open(&mut device.ptr), device)
    }

    /// Try to open a specific Airspy device by serial number.
    pub fn from_serial(serial_number: u64) -> Result<Airspy> {
        let mut device: Airspy = Airspy{ ptr: unsafe { std::mem::zeroed() }};
        ffifn!(ffi::airspy_open_sn(&mut device.ptr, serial_number), device)
    }

    /// Get available sample rates for this Airspy.
    pub fn get_sample_rates(&mut self) -> Result<Vec<u32>> {
        let mut len: u32 = 0;
        let lenp = &mut len as *mut u32;
        try!(ffifn!(ffi::airspy_get_samplerates(self.ptr, lenp, 0)));
        let mut rates: Vec<u32> = Vec::with_capacity(len as usize);
        let ratesp = rates.as_mut_ptr();
        try!(ffifn!(ffi::airspy_get_samplerates(self.ptr, ratesp, len)));
        unsafe { rates.set_len(len as usize); }
        Ok(rates)
    }

    /// Set this Airspy's sample rate by an index into available rates.
    ///
    /// `rate_idx` is an index from the Vector returned by `get_sample_rates`.
    pub fn set_sample_rate_by_idx(&mut self, rate_idx: u32) -> Result<()> {
        ffifn!(ffi::airspy_set_samplerate(self.ptr, rate_idx))
    }

    /// Set this Airspy's sample rate to a specific rate.
    ///
    /// This rate must be in the available rates or an error is returned.
    pub fn set_sample_rate(&mut self, target_rate: u32) -> Result<()> {
        let rates = try!(self.get_sample_rates());
        for (idx, rate) in rates.iter().enumerate() {
            if *rate == target_rate {
                return self.set_sample_rate_by_idx(idx as u32);
            }
        }
        Err(FFIError::new(ffi::airspy_error::AIRSPY_ERROR_INVALID_PARAM))
    }

    /// Start RX streaming from the Airspy.
    ///
    /// The given channel will be sent Vec<T> when Airspy callbacks occur. When
    /// the remote channel hangs up, libairspy is told to stop streaming and
    /// the Sender is dropped.
    ///
    /// T must match with whatever was set for SampleType, and must be one of
    /// f32, i16 or u16.
    pub fn start_rx<T>(&mut self, sender: Sender<Vec<T>>) -> Result<()>
        where T: Clone
    {
        // Box the Sender to move it onto the heap, then get a void* to it.
        let boxed_sender = Box::new(sender);
        let ctx = &*boxed_sender as *const _ as *mut ffi::c_void;
        // Forget the heap Sender so it is not immediately destroyed.
        std::mem::forget(boxed_sender);
        ffifn!(ffi::airspy_start_rx(self.ptr, rx_cb::<T>, ctx))
    }

    /// Stop the Airspy streaming.
    ///
    /// Causes the Sender from start_rx to leak. Consider destroying the
    /// Receiver half instead, which will also trigger libairspy to stop
    /// sending samples, but does not leak.
    pub fn stop_rx(&mut self) -> Result<()> {
        ffifn!(ffi::airspy_stop_rx(self.ptr))
    }

    /// Check if the Airspy is currently streaming.
    pub fn is_streaming(&mut self) -> bool {
        unsafe { ffi::airspy_is_streaming(self.ptr) == 1 }
    }

    /// Get the Airspy board type.
    pub fn get_board_id(&mut self) -> Result<&str> {
        let mut id: u8 = 0;
        try!(ffifn!(ffi::airspy_board_id_read(self.ptr, &mut id as *mut u8)));
        let cstr = unsafe {
            CStr::from_ptr(ffi::airspy_board_id_name(id))
        };
        Ok(str::from_utf8(cstr.to_bytes()).unwrap())
    }

    /// Get the Airspy firmware version number.
    pub fn get_version(&mut self) -> Result<String> {
        let mut buf: Vec<i8> = Vec::with_capacity(255);
        let bufp = buf.as_mut_ptr();
        try!(ffifn!(ffi::airspy_version_string_read(self.ptr, bufp, 255)));
        let cstr = unsafe { CStr::from_ptr(buf.as_ptr()) };
        Ok(String::from(str::from_utf8(cstr.to_bytes()).unwrap()))
    }

    /// Get the Airspy part ID and serial number.
    pub fn get_partid_serial(&mut self) -> Result<PartID> {
        let mut v = ffi::airspy_read_partid_serialno_t {
            part_id: [0u32; 2],
            serial_no: [0u32; 4]
        };
        try!(ffifn!(ffi::airspy_board_partid_serialno_read(self.ptr, &mut v)));
        Ok(PartID {
            part_id: v.part_id,
            serial_no: (v.serial_no[2] as u64) << 32 | v.serial_no[3] as u64
        })
    }

    /// Set sample type for data from this Airspy.
    pub fn set_sample_type(&mut self, stype: SampleType) -> Result<()> {
        let stype = match stype {
            SampleType::f32IQ => ffi::airspy_sample_type::AIRSPY_SAMPLE_FLOAT32_IQ,
            SampleType::f32Real => ffi::airspy_sample_type::AIRSPY_SAMPLE_FLOAT32_REAL,
            SampleType::i16IQ => ffi::airspy_sample_type::AIRSPY_SAMPLE_INT16_IQ,
            SampleType::i16Real => ffi::airspy_sample_type::AIRSPY_SAMPLE_INT16_REAL,
            SampleType::u16Real => ffi::airspy_sample_type::AIRSPY_SAMPLE_UINT16_REAL
        };
        ffifn!(ffi::airspy_set_sample_type(self.ptr, stype))
    }

    /// Set Airspy centre frequency, `freq` 24000000 to 1750000000 (in Hz)
    pub fn set_freq(&mut self, freq: u32) -> Result<()> {
        ffifn!(ffi::airspy_set_freq(self.ptr, freq))
    }

    /// Set LNA gain, 0 to 15dB
    pub fn set_lna_gain(&mut self, gain: u8) -> Result<()> {
        ffifn!(ffi::airspy_set_lna_gain(self.ptr, gain))
    }

    /// Set mixer gain, 0 to 15dB
    pub fn set_mixer_gain(&mut self, gain: u8) -> Result<()> {
        ffifn!(ffi::airspy_set_mixer_gain(self.ptr, gain))
    }

    /// Set VGA gain, 0 to 15dB
    pub fn set_vga_gain(&mut self, gain: u8) -> Result<()> {
        ffifn!(ffi::airspy_set_vga_gain(self.ptr, gain))
    }

    /// Enable/disable LNA AGC
    pub fn set_lna_agc(&mut self, enable: bool) -> Result<()> {
        ffifn!(ffi::airspy_set_lna_agc(self.ptr, enable as u8))
    }

    /// Enable/disable mixer AGC
    pub fn set_mixer_agc(&mut self, enable: bool) -> Result<()> {
        ffifn!(ffi::airspy_set_mixer_agc(self.ptr, enable as u8))
    }

    /// Enable/disable RF bias voltage
    pub fn set_rf_bias(&mut self, enable: bool) -> Result<()> {
        ffifn!(ffi::airspy_set_rf_bias(self.ptr, enable as u8))
    }

    /// Enable/disable USB sample packing
    /// NB: Not yet enabled as changes between shipping Ubuntu library and
    /// latest development library.
    pub fn set_packing(&mut self, _enable: bool) -> Result<()> {
        //ffifn!(ffi::airspy_set_packing(self.ptr, enable as u8))
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lib_version() {
        let v = lib_version();
        assert!(v.major == 1);
        assert!(v.minor == 0);
        assert!(v.revision >= 6);
    }

    #[test]
    fn test_init() {
        assert!(init().is_ok());
        let _ = exit();
    }

    #[test]
    fn test_exit() {
        let _ = init();
        assert!(exit().is_ok());
    }

    #[test]
    fn test_new() {
        let _ = init();
        let airspy = Airspy::new();
        assert!(airspy.is_ok());
        let _ = exit();
    }

    #[test]
    fn test_from_serial() {
        let _ = init();
        let mut serial: u64;
        {
            let mut airspy = Airspy::new().unwrap();
            let v = airspy.get_partid_serial().unwrap();
            serial = v.serial_no;
        }
        let airspy = Airspy::from_serial(serial);
        assert!(airspy.is_ok());
        let _ = exit();
    }

    #[test]
    fn test_get_sample_rates() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        let rates = airspy.get_sample_rates();
        assert!(rates.is_ok());
        let rates = rates.unwrap();
        assert!(rates.len() == 2);
        assert!(rates.contains(&2500000));
        assert!(rates.contains(&10000000));
        let _ = exit();
    }

    #[test]
    fn test_set_sample_rate_by_idx() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_sample_rate_by_idx(0).is_ok());
    }

    #[test]
    fn test_set_sample_rate() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_sample_rate(10_000_000).is_ok());
    }

    #[test]
    fn test_start_rx() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        let _ = airspy.set_sample_type(SampleType::f32IQ);
        let (tx, _) = ::std::sync::mpsc::channel();
        assert!(airspy.start_rx::<f32>(tx).is_ok());
    }

    #[test]
    fn test_is_streaming() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.is_streaming() == false);
    }

    #[test]
    fn test_get_board() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.get_board_id().unwrap() == "AIRSPY");
    }

    #[test]
    fn test_get_version() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.get_version().is_ok());
        // Skip this test as it is hardware-dependent.
        //let version = airspy.get_version().unwrap();
        //assert!(version == "AirSpy NOS v1.0.0-rc6-0-g035ff81 2015-07-14");
    }

    #[test]
    fn test_get_partid_serial() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.get_partid_serial().is_ok());
        // Skip these tests as they are hardware-dependent.
        //let v = airspy.get_partid_serial().unwrap();
        //assert!(v.serial_no == 0x440464c83833444f);
        //assert!(v.part_id[0] == 0x6906002B);
        //assert!(v.part_id[1] == 0x00000030);
    }

    #[test]
    fn test_set_sample_type() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_sample_type(SampleType::f32IQ).is_ok());
    }

    #[test]
    fn test_set_freq() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_freq(434000000).is_ok());
    }

    #[test]
    fn test_set_lna_gain() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_lna_gain(7).is_ok());
    }

    #[test]
    fn test_set_mixer_gain() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_mixer_gain(7).is_ok());
    }

    #[test]
    fn test_set_vga_gain() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_vga_gain(7).is_ok());
    }

    #[test]
    fn test_set_lna_agc() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_lna_agc(true).is_ok());
    }

    #[test]
    fn test_set_mixer_agc() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_mixer_agc(true).is_ok());
    }

    #[test]
    fn test_set_rf_bias() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_rf_bias(true).is_ok());
    }

    #[test]
    fn test_set_packing() {
        let _ = init();
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.set_packing(true).is_ok());
    }

}
