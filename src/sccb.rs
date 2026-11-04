//! Software (bit-banged) SCCB/I2C master over two open-drain GPIOs.
//!
//! This exists because the hardware I2C master on this pre-v3 silicon has been
//! observed to NACK on this board's shared `ESP_I2C` bus, while a software
//! master on the same two pins talks to the attached devices. It is used as a
//! fallback and as a diagnostic, not as the default transport.
//!
//! Timing is one `delay_us(4)` per half-period from the HAL delay driver, so
//! every figure here scales with what 4 us actually costs on this part. A read
//! clocks one bit per two half-periods (8 us, ~125 kHz); a write per three
//! (12 us, ~83 kHz), because its data-setup delay also lands while SCL is low.
//! The SCCB specification puts the minimum bit period tCYC at 10 us, i.e.
//! 100 kHz (v2.1 §2.2): the read path is 25 % over that limit, the write path is
//! inside it. The OV5647 datasheet states no SCCB clock rate of its own.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, Operation};
use esp_hal::delay::Delay;
use esp_hal::gpio::{DriveMode, Flex, InputConfig, OutputConfig, Pull};

/// Errors produced by the software bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpioBusError {
    /// Nothing pulled the ninth (Don't-Care) bit of a byte low.
    Nack,
    /// The operation is not supported by this transport.
    Unsupported,
}

impl embedded_hal::i2c::Error for GpioBusError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Nack => ErrorKind::NoAcknowledge(embedded_hal::i2c::NoAcknowledgeSource::Unknown),
            Self::Unsupported => ErrorKind::Other,
        }
    }
}

/// Software I2C master over two `Flex` pins wired as open-drain.
pub struct SccbGpio<'d> {
    sda: Flex<'d>,
    scl: Flex<'d>,
    delay: Delay,
}

impl<'d> SccbGpio<'d> {
    /// Builds the bus from the two pins (both are released high at idle).
    pub fn new(
        sda: impl esp_hal::gpio::Pin + 'd,
        scl: impl esp_hal::gpio::Pin + 'd,
        delay: Delay,
    ) -> Self {
        Self {
            sda: Self::open_drain(sda),
            scl: Self::open_drain(scl),
            delay,
        }
    }

    /// Input with pull-up, output open-drain with pull-up, driver released.
    ///
    /// The pull is set through `InputConfig`/`OutputConfig` and applied with
    /// `apply_input_config`/`apply_output_config`: `esp-hal-p4-pre-v3` 1.1.0 has
    /// no `Flex::set_pull`, and both apply methods document the pull direction as
    /// common to the input and output configuration, so both configs here say
    /// `Pull::Up`.
    fn open_drain(pin: impl esp_hal::gpio::Pin + 'd) -> Flex<'d> {
        let mut flex = Flex::new(pin);
        flex.apply_input_config(&InputConfig::default().with_pull(Pull::Up));
        flex.apply_output_config(
            &OutputConfig::default()
                .with_drive_mode(DriveMode::OpenDrain)
                .with_pull(Pull::Up),
        );
        flex.set_input_enable(true);
        flex.set_low();
        flex.set_output_enable(false);
        flex
    }

    fn half_period(&mut self) {
        self.delay.delay_us(4);
    }

    fn sda_low(&mut self) {
        self.sda.set_low();
        self.sda.set_output_enable(true);
    }

    fn sda_release(&mut self) {
        self.sda.set_output_enable(false);
    }

    fn scl_low(&mut self) {
        self.scl.set_low();
        self.scl.set_output_enable(true);
    }

    fn scl_release(&mut self) {
        self.scl.set_output_enable(false);
    }

    fn sda_is_high(&self) -> bool {
        self.sda.is_high()
    }

    fn scl_is_high(&self) -> bool {
        self.scl.is_high()
    }

    /// START: SDA falls while SCL is high.
    fn start(&mut self) {
        self.sda_release();
        self.scl_release();
        self.half_period();
        self.sda_low();
        self.half_period();
        self.scl_low();
        self.half_period();
    }

    /// STOP: SDA rises while SCL is high.
    fn stop(&mut self) {
        self.sda_low();
        self.half_period();
        self.scl_release();
        self.half_period();
        self.sda_release();
        self.half_period();
    }

    /// Writes one byte MSB-first, then reports whether anything pulled the ninth
    /// bit low.
    ///
    /// In SCCB that ninth bit is a Don't-Care bit, not an acknowledge (v2.1
    /// §3.2.1): the specification has the master mask SDA during it and keep
    /// clocking regardless of the response. This driver samples it instead and
    /// ends the transfer when it reads high, which is the I2C model. That is safe
    /// on this board because the OV5647 does pull it low — every mode-table write
    /// in `examples/` has to return `Ok` for a capture to happen at all.
    fn write_byte(&mut self, byte: u8) -> bool {
        for bit in (0..8).rev() {
            if byte & (1 << bit) != 0 {
                self.sda_release();
            } else {
                self.sda_low();
            }
            self.half_period();
            self.scl_release();
            self.half_period();
            self.scl_low();
            self.half_period();
        }
        // Ninth bit, which SCCB defines as Don't-Care for a write (not an I2C
        // ACK): release SDA and record whether a slave drove it low.
        self.sda_release();
        self.half_period();
        self.scl_release();
        self.half_period();
        let acked = !self.sda_is_high();
        self.scl_low();
        self.half_period();
        acked
    }

    /// Reads one byte MSB-first and NACKs it (single-byte reads are the norm).
    fn read_byte(&mut self) -> u8 {
        let mut byte = 0u8;
        self.sda_release();
        for _ in 0..8 {
            self.half_period();
            self.scl_release();
            self.half_period();
            byte = (byte << 1) | u8::from(self.sda_is_high());
            self.scl_low();
        }
        self.half_period();
        // NACK: leave SDA released (high) during the ninth clock.
        self.scl_release();
        self.half_period();
        self.scl_low();
        self.half_period();
        byte
    }

    /// Raw bus probe: START + address + W, then STOP. No register access.
    pub fn probe(&mut self, addr7: u8) -> bool {
        self.start();
        let acked = self.write_byte(addr7 << 1);
        self.stop();
        acked
    }

    /// True when both lines idle high (no stuck-low bus, no short).
    pub fn bus_idle(&mut self) -> (bool, bool) {
        self.sda_release();
        self.scl_release();
        self.half_period();
        (self.scl_is_high(), self.sda_is_high())
    }
}

impl ErrorType for SccbGpio<'_> {
    type Error = GpioBusError;
}

impl I2c for SccbGpio<'_> {
    fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.start();
        if !self.write_byte((address << 1) | 1) {
            self.stop();
            return Err(GpioBusError::Nack);
        }
        for (i, byte) in read.iter_mut().enumerate() {
            *byte = self.read_byte();
            let _ = i;
        }
        self.stop();
        Ok(())
    }

    fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.start();
        if !self.write_byte(address << 1) {
            self.stop();
            return Err(GpioBusError::Nack);
        }
        for byte in write {
            if !self.write_byte(*byte) {
                self.stop();
                return Err(GpioBusError::Nack);
            }
        }
        self.stop();
        Ok(())
    }

    fn transaction(
        &mut self,
        _address: u8,
        _operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        Err(GpioBusError::Unsupported)
    }
}
