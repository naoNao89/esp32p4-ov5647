//! OV5647 sensor driver: SCCB transport, identification, register tables.
//!
//! The transport is generic over any `embedded-hal` 1.0 I2C implementation so
//! that the same driver works over the ESP32-P4 I2C master or over a software
//! (bit-banged) SCCB implementation, whichever proves to work on this board.
//!
//! SCCB framing notes that the transport must respect:
//! * 16-bit register address, 8-bit value, MSB first.
//! * A register read is two independent transactions: `START addr W reg_hi
//!   reg_lo STOP` then `START addr R data STOP`. SCCB does not use a repeated
//!   START, so this driver never issues `write_read`.
//!
//! The module is deliberately free of HAL types — only `embedded-hal` traits are
//! used — so `crates/ov5647-core` can compile it for the host tests
//! (`cargo test -p ov5647-core --features std` from the workspace root) and cover
//! the table walker, the mode configuration and the stream sequences without
//! hardware.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{ErrorKind, I2c};

use crate::regs::{
    Reg, CHIP_ID, MIPI_CTRL00_NONCONT_LP11, MIPI_CTRL00_NONCONT_LP11_CLOCK_OFF, REG_DELAY, REG_END,
    REG_MIPI_CTRL0, REG_SW_STANDBY, SCCB_ADDR,
};

/// SCCB failure, normalised so callers do not depend on the HAL error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SccbError {
    /// The bus reported a NACK for the address or a data byte.
    Nack,
    /// Any other bus error, passed through by kind.
    Bus(ErrorKind),
}

/// Convenience: converts any `embedded_hal::i2c::Error` into `SccbError`.
pub fn classify<E: embedded_hal::i2c::Error>(e: E) -> SccbError {
    match e.kind() {
        ErrorKind::NoAcknowledge(_) => SccbError::Nack,
        other => SccbError::Bus(other),
    }
}

/// Blocking OV5647 driver.
pub struct Ov5647<I> {
    i2c: I,
    addr: u8,
}

impl<I> Ov5647<I>
where
    I: I2c,
{
    /// Wraps a bus. `addr` is the 7-bit SCCB address (`0x36` for OV5647).
    pub fn new(i2c: I, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Uses the default OV5647 address of `0x36`.
    pub fn with_default_addr(i2c: I) -> Self {
        Self::new(i2c, SCCB_ADDR)
    }

    /// Releases the underlying bus.
    pub fn release(self) -> I {
        self.i2c
    }

    /// Writes one 8-bit value to a 16-bit register address.
    pub fn write_reg(&mut self, reg: u16, value: u8) -> Result<(), SccbError> {
        let frame = [(reg >> 8) as u8, reg as u8, value];
        self.i2c.write(self.addr, &frame).map_err(classify)
    }

    /// Reads one 8-bit value from a 16-bit register address.
    pub fn read_reg(&mut self, reg: u16) -> Result<u8, SccbError> {
        let mut value = [0u8; 1];
        self.read_regs(reg, &mut value)?;
        Ok(value[0])
    }

    /// Reads `out.len()` consecutive registers starting at `reg`.
    ///
    /// # Caution
    ///
    /// The OV5647 does not auto-increment the register pointer on reads: only
    /// the first byte of a burst is meaningful, the rest come back as `0xFF`.
    /// Use [`Self::read_reg`] per register. Kept for the single-byte case,
    /// which is all this driver ever needs.
    pub fn read_regs(&mut self, reg: u16, out: &mut [u8]) -> Result<(), SccbError> {
        let addr = [(reg >> 8) as u8, reg as u8];
        // Phase 1: register address, terminated with STOP (SCCB has no
        // repeated START).
        if let Err(e) = self.i2c.write(self.addr, &addr) {
            return Err(classify(e));
        }
        // Phase 2: data read.
        self.i2c.read(self.addr, out).map_err(classify)
    }

    /// Reads the 16-bit chip ID from `0x300A`/`0x300B`.
    ///
    /// Each byte gets its own register-address phase. Measured on the attached
    /// module: a two-byte burst read from `0x300A` returns `0x56` for the high
    /// byte and then `0xFF`, i.e. the OV5647 does **not** auto-increment the
    /// register pointer across a burst read, so `read_regs` must not be used
    /// here. This is what made the first bring-up report `PID=0x56FF` while the
    /// part was perfectly healthy.
    pub fn chip_id(&mut self) -> Result<u16, SccbError> {
        let high = self.read_reg(crate::regs::REG_CHIP_ID_H)?;
        let low = self.read_reg(crate::regs::REG_CHIP_ID_L)?;
        Ok(u16::from_be_bytes([high, low]))
    }

    /// True if the sensor acknowledges an SCCB register read.
    pub fn is_present(&mut self) -> bool {
        self.read_reg(crate::regs::REG_CHIP_ID_H).is_ok()
    }

    /// Reads the chip ID and classifies the outcome, so "another sensor is on the
    /// bus" is not confused with "the bus did not answer".
    pub fn identify(&mut self) -> ChipIdOutcome {
        match self.chip_id() {
            Ok(id) if id == CHIP_ID => ChipIdOutcome::Match(id),
            Ok(id) => ChipIdOutcome::Mismatch(id),
            Err(e) => ChipIdOutcome::BusError(e),
        }
    }

    /// Writes one register, naming that register if the bus refuses it.
    fn write_entry(&mut self, entry: Reg) -> Result<(), (Reg, SccbError)> {
        self.write_reg(entry.addr, entry.value)
            .map_err(|e| (entry, e))
    }

    /// Applies a register table in order.
    ///
    /// Two addresses in a table are not registers, they are control codes (see
    /// [`crate::registers`]):
    ///
    /// * an entry with address `REG_END` ends the table: it and every entry after
    ///   it are skipped, and nothing is written;
    /// * an entry with address `REG_DELAY` is a delay of `value` milliseconds, not
    ///   a write.
    ///
    /// A failed write returns the [`Reg`] that failed together with the bus error,
    /// so a failure identifies a register rather than a table index.
    pub fn write_table<D: DelayNs>(
        &mut self,
        table: &[Reg],
        delay: &mut D,
    ) -> Result<(), (Reg, SccbError)> {
        for &entry in table {
            match entry.addr {
                REG_END => return Ok(()),
                REG_DELAY => delay.delay_ms(u32::from(entry.value)),
                _ => self.write_entry(entry)?,
            }
        }
        Ok(())
    }

    /// Programs the sensor for `mode` and leaves it in software standby.
    ///
    /// 1. `0x0103 = 0x01` (software reset) followed by [`RESET_DELAY_MS`] — the
    ///    reset step of the reference sequence (`registers::OV5647_RESET`,
    ///    `// S12`), which also stops the stream and forces the MIPI clock lane to
    ///    LP-11 first. The datasheet does not say whether the reset bit clears
    ///    itself, so it is not read back.
    /// 2. `mode.table`, in order, through [`Self::write_table`], so the
    ///    `REG_END`/`REG_DELAY` sentinels are honoured.
    ///
    /// `configure` never writes `0x0100` itself: streaming stays with the caller
    /// ([`Self::start_stream`] / [`Self::stop_stream`]). Every table in
    /// [`crate::registers`] ends at its last configuration write for that reason.
    pub fn configure<D: DelayNs>(
        &mut self,
        mode: &ModeSpec,
        delay: &mut D,
    ) -> Result<(), (Reg, SccbError)> {
        // Use the ported reference reset table rather than hand-writing the one
        // register: `OV5647_RESET` is the port of `ov5647_mipi_reset_regs[]` and
        // is `{0x0100 = 0x00, 0x0103 = 0x01, <10 ms>, 0x4800 = 0x01}`. The entry
        // this driver used to omit is `0x4800 = 0x01`, which disables the MIPI
        // clock lane and drives the link to LP-11 -- i.e. it is the one reset
        // entry that touches the MIPI block, and leaving it out was an
        // uncontrolled variable while investigating clock-lane state.
        self.write_table(crate::registers::OV5647_RESET, delay)?;
        self.write_table(mode.table, delay)
    }

    /// Starts streaming: `0x4800 = 0x24`, then `0x0100 = 0x01`.
    ///
    /// The register values are `registers::OV5647_STREAM_ON` (`// S10`); they
    /// are written here in the same order so that the sequence is explicit and a
    /// failure names its register.
    pub fn start_stream(&mut self) -> Result<(), (Reg, SccbError)> {
        self.write_entry(Reg::new(REG_MIPI_CTRL0, MIPI_CTRL00_NONCONT_LP11))?;
        self.write_entry(Reg::new(REG_SW_STANDBY, 0x01))
    }

    /// Stops streaming: `0x0100 = 0x00`, then `0x4800 = 0x25`.
    ///
    /// The values are esp-video's stream-off values — `0x25` is the clock lane
    /// gated, the bus idling in LP-11 and the clock lane forced into low power
    /// (`// S10`; the bits themselves are datasheet fields, see
    /// [`crate::regs::MIPI_CTRL00_CLOCK_LANE_DISABLE`] and family). The order is
    /// this driver's: the transmitter is stopped before its control register is
    /// changed. `registers::OV5647_STREAM_OFF` holds the same two writes in the
    /// opposite order (`// S10`).
    pub fn stop_stream(&mut self) -> Result<(), (Reg, SccbError)> {
        self.write_entry(Reg::new(REG_SW_STANDBY, 0x00))?;
        self.write_entry(Reg::new(REG_MIPI_CTRL0, MIPI_CTRL00_NONCONT_LP11_CLOCK_OFF))
    }
}

/// Delay after the software reset (`0x0103 = 0x01`), in milliseconds.
///
/// The esp-video reset table waits 10 ms at this point (`// S12`,
/// `{0x0103, 0x01}` then `{OV5647_REG_DELAY, 0x0a}`). esp-video's separate
/// `ov5647_soft_reset()` waits 5 ms (`// S10`), so 10 ms is the longer of the two
/// reference waits. The OV5647 datasheet states no post-reset delay at all (its
/// §2.6 gives only the >= 1 ms hardware RESETB pulse width; §2.5.1 gives the
/// power-up waits), so this constant is a reference-derived margin, not a
/// datasheet requirement.
pub const RESET_DELAY_MS: u32 = 10;

/// Outcome of identifying the sensor on the bus.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChipIdOutcome {
    /// A sensor answered with the OV5647 identifier; carries the value read.
    Match(u16),
    /// A sensor answered with some other identifier; carries the value read.
    Mismatch(u16),
    /// The SCCB transaction itself failed.
    BusError(SccbError),
}

/// A sensor mode: what the sensor will put on the CSI-2 link, and the register
/// table that programs it for that.
///
/// [`Self::table`] is what [`Ov5647::configure`] applies; the geometry and the link
/// rates are what the CSI/ISP side and the frame buffers must be told (the
/// receiver's `mipi_clk` is `mipi_bitrate_bps`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeSpec {
    /// Human-readable name, for logs.
    pub name: &'static str,
    /// Output width in pixels of the table's window.
    pub width: u32,
    /// Output height in pixels of the table's window.
    pub height: u32,
    /// Wire format the sensor is programmed for.
    pub format: ov5647_core::PixelFormat,
    /// MIPI CSI-2 data lanes the table enables.
    pub lanes: u8,
    /// XCLK the table's PLL programming assumes, in Hz.
    ///
    /// Every table here is programmed for a 25 MHz XCLK, which is what
    /// esp-video's mode metadata declares (`.xclk = 24000000` in `ov5647.c` is
    /// the metadata field, but its RAW10 pixel-clock constants only reproduce
    /// from 25 MHz: S1's multiplier 98 -> 81 666 700 Hz and S2's 106 ->
    /// 88 333 333 Hz are both `25 MHz x multiplier / 30`). This board's camera
    /// connector has no confirmed MCLK source, so the value the sensor actually
    /// receives is not measured here.
    pub xclk_hz: u32,
    /// Per-lane MIPI CSI-2 line rate in bit/s:
    /// `pclk * bits-per-pixel / lanes`, with `pclk` the clock the table's PLL
    /// programming produces (the clock lane runs at half of this). For a RAW10
    /// 2-lane mode that is `pclk * 5`; the value is checked against
    /// esp-video's constants where one exists (`// S12`).
    pub mipi_bitrate_bps: u32,
    /// Register table that programs this mode, applied by [`Ov5647::configure`].
    pub table: &'static [Reg],
}

impl ModeSpec {
    /// 1920x1080 RAW10 at 30 fps, 2 lanes, 25 MHz XCLK — the mode to bring up
    /// first.
    ///
    /// Its timings are the self-consistent ones: the table's HTS x VTS
    /// (2271 x 1199) at the pixel clock the PLL produces is 29.99 fps
    /// (`// S1`, `// S12`).
    ///
    /// * PLL: `0x3034 = 0x1A`, `0x3035 = 0x21`, `0x3036 = 0x62` (multiplier 98),
    ///   `0x3037 = 0x03` (reset value), `0x3106 = 0xF5` ->
    ///   VCO = 25 MHz x 2/6 x 98 = 816.67 MHz and
    ///   pclk = VCO x 2 / 2 / 1 / 5 / 2 = 81.6667 MHz, quoted by esp-video as
    ///   81 666 700 Hz (`// S12`)
    /// * per-lane line rate = pclk x 10 / 2 = 408 333 500 bit/s, clock lane
    ///   204 166 750 Hz (`// S12`)
    /// * the table ends with `0x4837 = 0x19` (PCLK_PERIOD for that pclk) and
    ///   `0x4800 = 0x24` (non-continuous clock, LP-11 idle, no line-sync packets),
    ///   and does not write `0x0100` (`// S1`)
    /// * Bayer order is an open item: esp-video declares GBRG for every mode
    ///   (`// S10`), and nothing in this repository measures the phase of this
    ///   table, so the host tool's `--bayer` value is a display choice.
    pub const fn mode_1920x1080_raw10_30fps() -> Self {
        Self {
            name: "1920x1080 RAW10 30fps 2-lane",
            width: 1920,
            height: 1080,
            format: ov5647_core::PixelFormat::Raw10,
            lanes: 2,
            xclk_hz: 25_000_000,
            mipi_bitrate_bps: 408_333_500,
            table: crate::registers::OV5647_MODE_1920X1080_RAW10_30FPS,
        }
    }

    /// 2592x1944 RAW10 at 15 fps, 2 lanes, 25 MHz XCLK — full field of view, no
    /// binning.
    ///
    /// Programs `registers::OV5647_MODE_2592X1944_RAW10`, which is derived here
    /// from the OV5647 datasheet and the Apache-2.0 esp-video reference (see that
    /// table's derivation record).
    ///
    /// * PLL: `0x3036 = 0x66` (multiplier 102) -> pclk = 25 MHz x 102 / 30 =
    ///   85 000 000 Hz
    /// * hts = 2885 (window + the 285 blanking columns the esp-video tables use and
    ///   the sensor is measured to need), vts = 1968 ->
    ///   85 000 000 / (2885 x 1968) = 14.97 fps
    /// * per-lane line rate = 85 000 000 x 10 / 2 = 425 000 000 bit/s, clock lane
    ///   212 500 000 Hz (`0x4837 = 0x18`, a 6.0 ns pclk2x period)
    /// * packed frame = 2592 x 1944 x 10 / 8 = 6 298 560 bytes
    /// * the table does not write `0x0100`: [`Ov5647::start_stream`] streams it
    pub const fn mode_2592x1944_raw10() -> Self {
        Self {
            name: "2592x1944 RAW10 15fps 2-lane",
            width: 2592,
            height: 1944,
            format: ov5647_core::PixelFormat::Raw10,
            lanes: 2,
            xclk_hz: 25_000_000,
            mipi_bitrate_bps: 425_000_000,
            table: crate::registers::OV5647_MODE_2592X1944_RAW10,
        }
    }

    /// 1280x960 RAW10 at 45 fps, 2 lanes, 25 MHz XCLK: 2x2 binned and subsampled.
    ///
    /// * hts = 1796, vts = 1093 (the encoded bytes; the source comment quotes older
    ///   values), pclk = 88 333 333 Hz, so 44.999 fps — `// S2`, `// S12`
    /// * per-lane line rate = 88 333 333 x 10 / 2 = 441 666 665 bit/s
    /// * the table does write `0x4800 = 0x24` itself but not `0x0100`;
    ///   [`Ov5647::start_stream`] writes the pair (`// S2`, `// S10`)
    pub const fn mode_1280x960_raw10_45fps() -> Self {
        Self {
            name: "1280x960 RAW10 45fps 2-lane",
            width: 1280,
            height: 960,
            format: ov5647_core::PixelFormat::Raw10,
            lanes: 2,
            xclk_hz: 25_000_000,
            mipi_bitrate_bps: 441_666_665,
            table: crate::registers::OV5647_MODE_1280X960_RAW10_45FPS,
        }
    }

    /// 800x640 RAW8 at 50 fps, 2 lanes, 25 MHz XCLK: 2x subsampled.
    ///
    /// * hts = 1896, vts = 984, nominal pclk 100 MHz (`// S3`); the declared
    ///   frame rate only holds if that nominal clock is real, and the RAW8 tables
    ///   are internally inconsistent about `0x4837` (S3 writes 0x28 = 40 where
    ///   S1/S2's rule for a RAW10 pclk of that size gives 20) — treat RAW8 as the
    ///   lower-confidence path
    /// * per-lane line rate = 100 000 000 x 8 / 2 = 400 000 000 bit/s (`// S12`)
    /// * the table does not write `0x4800` or `0x0100` (`// S3`, `// S10`)
    pub const fn mode_800x640_raw8_50fps() -> Self {
        Self {
            name: "800x640 RAW8 50fps 2-lane",
            width: 800,
            height: 640,
            format: ov5647_core::PixelFormat::Raw8,
            lanes: 2,
            xclk_hz: 25_000_000,
            mipi_bitrate_bps: 400_000_000,
            table: crate::registers::OV5647_MODE_800X640_RAW8_50FPS,
        }
    }

    /// 800x800 RAW8 at 50 fps, 2 lanes, 25 MHz XCLK: same window as
    /// [`Self::mode_800x640_raw8_50fps`], taller readout (`// S4`).
    pub const fn mode_800x800_raw8_50fps() -> Self {
        Self {
            name: "800x800 RAW8 50fps 2-lane",
            width: 800,
            height: 800,
            format: ov5647_core::PixelFormat::Raw8,
            lanes: 2,
            xclk_hz: 25_000_000,
            mipi_bitrate_bps: 400_000_000,
            table: crate::registers::OV5647_MODE_800X800_RAW8_50FPS,
        }
    }

    /// 800x1280 RAW8 at 50 fps, 2 lanes, 25 MHz XCLK: a window of its own — x
    /// 512..2621, y 248..1695 — not the 800x640/800x800 crop, with hts = 2394 and
    /// vts = 1732 (`// S5`). The 50 fps is the source's nominal label; 100 MHz
    /// over 2394 x 1732 is 24.1 fps.
    ///
    /// The output is 800 columns of that 2110-column window, so the table also sets
    /// the ISP output window offset (`0x3810`/`0x3811`) to centre the crop; see
    /// [`crate::registers::OV5647_MODE_800X1280_RAW8_50FPS`] for that and for the
    /// corrected AEC gain ceiling, both with the measurements behind them.
    pub const fn mode_800x1280_raw8_50fps() -> Self {
        Self {
            name: "800x1280 RAW8 50fps 2-lane",
            width: 800,
            height: 1280,
            format: ov5647_core::PixelFormat::Raw8,
            lanes: 2,
            xclk_hz: 25_000_000,
            mipi_bitrate_bps: 400_000_000,
            table: crate::registers::OV5647_MODE_800X1280_RAW8_50FPS,
        }
    }
}
