//! Board wiring for the ESP32-P4 rev 1.3 target board.
//!
//! # Verification status of every constant here
//!
//! The physical board is a Waveshare ESP32-P4 board carrying a 2-lane MIPI-DSI
//! panel and a GT911 touch controller (proved by the vendor factory demo that
//! still occupies the tail of the flash). Its exact SKU was never confirmed from
//! the silkscreen, and no schematic PDF for it has been obtained.
//!
//! * `SCCB_SDA_GPIO` / `SCCB_SCL_GPIO` — **CONFIRMED on hardware.** These are
//!   the board's shared `ESP_I2C` bus (2.2 kOhm pull-ups), the same bus the
//!   board routes to both the CSI and the DSI FPC connectors. A previous
//!   firmware for this exact unit drove a BH1750 on SDA=GPIO7 / SCL=GPIO8 and
//!   the camera sensor answers there too.
//! * `SCCB_FREQ_KHZ` — design choice: 100 kHz, which is what the SCCB
//!   specification allows at most (tCYC >= 10 us, v2.1 §2.2). The bit-banged
//!   transport in `sccb.rs` does not read this value; it times itself.
//! * Camera MCLK / power-down / reset — **UNVERIFIED.** No primary source for
//!   this SKU's camera connector was obtained, so no GPIO is assigned here.
//!   The CSI connector's differential lanes are dedicated MIPI D-PHY signals,
//!   not GPIO-matrix signals, so they need no pin assignment.

/// SDA of the board's shared `ESP_I2C` bus, which the CSI connector carries.
pub const SCCB_SDA_GPIO: u8 = 7;

/// SCL of the board's shared `ESP_I2C` bus, which the CSI connector carries.
pub const SCCB_SCL_GPIO: u8 = 8;

/// SCCB clock rate for a hardware I2C transport. SCCB allows at most 100 kHz
/// (minimum bit period tCYC = 10 us, spec v2.1 §2.2); the OV5647 datasheet
/// states no SCCB rate of its own. The bit-banged [`crate::sccb::SccbGpio`] does
/// not read this constant — it derives its own timing from the delay driver.
pub const SCCB_FREQ_KHZ: u32 = 100;

/// 7-bit SCCB device address of the OV5647.
pub const OV5647_SCCB_ADDR: u8 = 0x36;

/// Addresses probed by the bus scan, inclusive.
pub const SCAN_FIRST: u8 = 0x03;
pub const SCAN_LAST: u8 = 0x77;

/// Addresses the *hardware* I2C master is asked about.
///
/// Kept deliberately tiny: the hardware master can wedge mid-transaction on this
/// silicon, and each wedged attempt risks a watchdog reset. These are the only
/// devices that can plausibly answer on this board's shared `ESP_I2C` bus:
/// the OV5647 sensor, and the GT911 / FT5x06 touch controllers that the board's
/// vendor firmware talks to (`Touch 0x5d found`, `Touch 0x14 found`).
pub const HW_PROBE_ADDRESSES: &[u8] = &[OV5647_SCCB_ADDR, 0x5D, 0x14];
