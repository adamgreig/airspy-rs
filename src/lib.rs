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

/// Choice of GPIO port
pub enum GPIOPort {
    Port0, Port1, Port2, Port3, Port4, Port5, Port6, Port7
}

/// Choice of GPIO pin
pub enum GPIOPin {
    Pin0,  Pin1,  Pin2,  Pin3,  Pin4,  Pin5,  Pin6,  Pin7,
    Pin8,  Pin9,  Pin10, Pin11, Pin12, Pin13, Pin14, Pin15,
    Pin16, Pin17, Pin18, Pin19, Pin20, Pin21, Pin22, Pin23,
    Pin24, Pin25, Pin26, Pin27, Pin28, Pin29, Pin30, Pin31
}

/// GPIO Direction
pub enum GPIODirection {
    Input, Output
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

    /// Set this Airspy's sample rate to a specific rate.
    ///
    /// This rate must be in the available rates or an error is returned.
    pub fn set_sample_rate(&mut self, target_rate: u32) -> Result<()> {
        let rates = try!(self.get_sample_rates());
        for (idx, rate) in rates.iter().enumerate() {
            if *rate == target_rate {
                return ffifn!(ffi::airspy_set_samplerate(
                    self.ptr, idx as u32));
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

    /// Check if the Airspy is currently streaming.
    pub fn is_streaming(&mut self) -> bool {
        unsafe { ffi::airspy_is_streaming(self.ptr) == 1 }
    }

    /// Write a register on the Si5351C
    pub fn si5351c_write(&mut self, register: u8, value: u8) -> Result<()> {
        ffifn!(ffi::airspy_si5351c_write(self.ptr, register, value))
    }

    /// Read a register on the Si5351C
    pub fn si5351c_read(&mut self, register: u8) -> Result<u8> {
        let mut val: u8 = 0;
        try!(ffifn!(ffi::airspy_si5351c_read(
            self.ptr, register, &mut val as *mut u8)));
        Ok(val)
    }

    /// Write a register on the R820t
    pub fn r820t_write(&mut self, register: u8, value: u8) -> Result<()> {
        ffifn!(ffi::airspy_r820t_write(self.ptr, register, value))
    }

    /// Read a register on the R820t
    pub fn r820t_read(&mut self, register: u8) -> Result<u8> {
        let mut val: u8 = 0;
        try!(ffifn!(ffi::airspy_r820t_read(
            self.ptr, register, &mut val as *mut u8)));
        Ok(val)
    }

    fn map_gpio_port_pin(port: GPIOPort, pin: GPIOPin)
            -> (ffi::airspy_gpio_port_t, ffi::airspy_gpio_pin_t) {
        (match port {
            GPIOPort::Port0 => ffi::airspy_gpio_port_t::GPIO_PORT0,
            GPIOPort::Port1 => ffi::airspy_gpio_port_t::GPIO_PORT1,
            GPIOPort::Port2 => ffi::airspy_gpio_port_t::GPIO_PORT2,
            GPIOPort::Port3 => ffi::airspy_gpio_port_t::GPIO_PORT3,
            GPIOPort::Port4 => ffi::airspy_gpio_port_t::GPIO_PORT4,
            GPIOPort::Port5 => ffi::airspy_gpio_port_t::GPIO_PORT5,
            GPIOPort::Port6 => ffi::airspy_gpio_port_t::GPIO_PORT6,
            GPIOPort::Port7 => ffi::airspy_gpio_port_t::GPIO_PORT7,
        }, match pin {
            GPIOPin::Pin0 => ffi::airspy_gpio_pin_t::GPIO_PIN0,
            GPIOPin::Pin1 => ffi::airspy_gpio_pin_t::GPIO_PIN1,
            GPIOPin::Pin2 => ffi::airspy_gpio_pin_t::GPIO_PIN2,
            GPIOPin::Pin3 => ffi::airspy_gpio_pin_t::GPIO_PIN3,
            GPIOPin::Pin4 => ffi::airspy_gpio_pin_t::GPIO_PIN4,
            GPIOPin::Pin5 => ffi::airspy_gpio_pin_t::GPIO_PIN5,
            GPIOPin::Pin6 => ffi::airspy_gpio_pin_t::GPIO_PIN6,
            GPIOPin::Pin7 => ffi::airspy_gpio_pin_t::GPIO_PIN7,
            GPIOPin::Pin8 => ffi::airspy_gpio_pin_t::GPIO_PIN8,
            GPIOPin::Pin9 => ffi::airspy_gpio_pin_t::GPIO_PIN9,
            GPIOPin::Pin10 => ffi::airspy_gpio_pin_t::GPIO_PIN10,
            GPIOPin::Pin11 => ffi::airspy_gpio_pin_t::GPIO_PIN11,
            GPIOPin::Pin12 => ffi::airspy_gpio_pin_t::GPIO_PIN12,
            GPIOPin::Pin13 => ffi::airspy_gpio_pin_t::GPIO_PIN13,
            GPIOPin::Pin14 => ffi::airspy_gpio_pin_t::GPIO_PIN14,
            GPIOPin::Pin15 => ffi::airspy_gpio_pin_t::GPIO_PIN15,
            GPIOPin::Pin16 => ffi::airspy_gpio_pin_t::GPIO_PIN16,
            GPIOPin::Pin17 => ffi::airspy_gpio_pin_t::GPIO_PIN17,
            GPIOPin::Pin18 => ffi::airspy_gpio_pin_t::GPIO_PIN18,
            GPIOPin::Pin19 => ffi::airspy_gpio_pin_t::GPIO_PIN19,
            GPIOPin::Pin20 => ffi::airspy_gpio_pin_t::GPIO_PIN20,
            GPIOPin::Pin21 => ffi::airspy_gpio_pin_t::GPIO_PIN21,
            GPIOPin::Pin22 => ffi::airspy_gpio_pin_t::GPIO_PIN22,
            GPIOPin::Pin23 => ffi::airspy_gpio_pin_t::GPIO_PIN23,
            GPIOPin::Pin24 => ffi::airspy_gpio_pin_t::GPIO_PIN24,
            GPIOPin::Pin25 => ffi::airspy_gpio_pin_t::GPIO_PIN25,
            GPIOPin::Pin26 => ffi::airspy_gpio_pin_t::GPIO_PIN26,
            GPIOPin::Pin27 => ffi::airspy_gpio_pin_t::GPIO_PIN27,
            GPIOPin::Pin28 => ffi::airspy_gpio_pin_t::GPIO_PIN28,
            GPIOPin::Pin29 => ffi::airspy_gpio_pin_t::GPIO_PIN29,
            GPIOPin::Pin30 => ffi::airspy_gpio_pin_t::GPIO_PIN30,
            GPIOPin::Pin31 => ffi::airspy_gpio_pin_t::GPIO_PIN31,
        })
    }

    /// Write a GPIO port:pin to `val`, false to clear or true to set.
    pub fn gpio_write(&mut self, port: GPIOPort, pin: GPIOPin, val: bool)
        -> Result<()>
    {
        let (port, pin) = Airspy::map_gpio_port_pin(port, pin);
        ffifn!(ffi::airspy_gpio_write(self.ptr, port, pin, val as u8))
    }

    /// Read a GPIO port:pin
    pub fn gpio_read(&mut self, port: GPIOPort, pin: GPIOPin) -> Result<bool> {
        let mut val: u8 = 0;
        let (port, pin) = Airspy::map_gpio_port_pin(port, pin);
        try!(ffifn!(ffi::airspy_gpio_read(
            self.ptr, port, pin, &mut val as *mut u8)));
        Ok(match val {
            0 => false,
            1 => true,
            _ => unreachable!()
        })
    }

    /// Set a GPIO port:pin direction.
    pub fn gpio_set_direction(&mut self, port: GPIOPort, pin: GPIOPin,
        dir: GPIODirection) -> Result<()>
    {
        let (port, pin) = Airspy::map_gpio_port_pin(port, pin);
        let dir: u8 = match dir {
            GPIODirection::Input => 0,
            GPIODirection::Output => 1
        };
        ffifn!(ffi::airspy_gpiodir_write(self.ptr, port, pin, dir))
    }

    /// Get a GPIO port:pin direction.
    pub fn gpio_get_direction(&mut self, port: GPIOPort, pin: GPIOPin)
        -> Result<GPIODirection>
    {
        let mut dir: u8 = 0;
        let (port, pin) = Airspy::map_gpio_port_pin(port, pin);
        try!(ffifn!(ffi::airspy_gpiodir_read(
            self.ptr, port, pin, &mut dir as *mut u8)));
        Ok(match dir {
            0 => GPIODirection::Input,
            1 => GPIODirection::Output,
            _ => unreachable!()
        })
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
    fn test_si5351c_read() {
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.si5351c_read(16).is_ok());
    }

    #[test]
    fn test_si5351c_write() {
        let mut airspy = Airspy::new().unwrap();
        let val = airspy.si5351c_read(16).unwrap();
        assert!(airspy.si5351c_write(16, val).is_ok());
    }

    #[test]
    fn test_r820t_read() {
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.r820t_read(0x0F).is_ok());
    }

    #[test]
    fn test_r820t_write() {
        let mut airspy = Airspy::new().unwrap();
        let val = airspy.r820t_read(0x0F).unwrap();
        assert!(airspy.r820t_write(0x0F, val).is_ok());
    }

    #[test]
    fn test_gpio_read() {
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.gpio_read(GPIOPort::Port0, GPIOPin::Pin0).is_ok());
    }

    #[test]
    fn test_gpio_write() {
        let mut airspy = Airspy::new().unwrap();
        let val = airspy.gpio_read(GPIOPort::Port0, GPIOPin::Pin0).unwrap();
        assert!(airspy.gpio_write(
            GPIOPort::Port0, GPIOPin::Pin0, val).is_ok());
    }

    #[test]
    fn test_gpio_get_dir() {
        let mut airspy = Airspy::new().unwrap();
        assert!(airspy.gpio_get_direction(
            GPIOPort::Port0, GPIOPin::Pin0).is_ok());
    }

    #[test]
    fn test_gpio_set_dir() {
        let mut airspy = Airspy::new().unwrap();
        let dir = airspy.gpio_get_direction(
            GPIOPort::Port0, GPIOPin::Pin0).unwrap();
        assert!(airspy.gpio_set_direction(
            GPIOPort::Port0, GPIOPin::Pin0, dir).is_ok());
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
}
