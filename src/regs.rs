//! OV5647 register addresses and identifiers.
//!
//! Register values that configure the sensor (PLL, MIPI timing, mode tables)
//! live in [`crate::registers`]. Only identity, control and MIPI-transmitter
//! definitions are here — the addresses the driver itself names, the bit fields
//! of `MIPI CTRL 00`, and the two table-walker sentinels.
//!
//! Sources, named per constant below: the OV5647 datasheet (S8; table numbers
//! follow its chapter 6) for register names, addresses and bit fields, and the
//! Apache-2.0 esp-video driver (S10 for the MIPI composites, S12 for the
//! `MIPI CTRL 00` bit fields, S14 for the two sentinel values). The datasheet is
//! cited, not reproduced.

/// SCCB device address (7-bit) of the OV5647. Datasheet: `0x300C`
/// `SC_CMMN_SCCB_ID`, whose reset value `0x6C` is the 8-bit write address
/// (`0x6C >> 1 == 0x36`).
pub const SCCB_ADDR: u8 = 0x36;

/// `0x3008` `SC_CMMN_PAD_OUT0`, reset `0x00`: `io_y_o[11:8]` of the parallel
/// output pads (S8, table 6-1). Not a system-control register despite the name,
/// which is kept for API compatibility; this driver never writes it.
pub const REG_SYS_CTRL0: u16 = 0x3008;

/// `0x3009` `SC_CMMN_PAD_OUT1`, reset `0x00`: `io_y_o[7:0]` of the parallel
/// output pads (S8, table 6-1). Kept for API compatibility; never written by
/// this driver.
pub const REG_SYS_CTRL1: u16 = 0x3009;

/// Software reset, `0x0103` bit 0: writing `0x01` resets the sensor and returns
/// every register to its documented default (S8 §2.6). Whether the bit clears
/// itself is not stated by the datasheet and the reference implementations
/// disagree, so this driver does not read it back.
/// The reset sequence is `0x0100 = 0x00`, `0x0103 = 0x01`, 10 ms,
/// `0x4800 = 0x01` (clock lane forced to LP-11). `// S12`
pub const REG_SW_RESET: u16 = 0x0103;

/// Streaming gate, `0x0100` bit 0: `0` = software standby (stream off), `1` =
/// streaming (S8 §2.7). Written by esp-video's `ov5647_set_stream` (`// S10`),
/// which is the sequence [`crate::registers::OV5647_STREAM_ON`] transcribes.
pub const REG_SW_STANDBY: u16 = 0x0100;

/// Chip ID high byte, `0x300A` `SC_CMMN_CHIP_ID`, reset (read-only) value `0x56`.
pub const REG_CHIP_ID_H: u16 = 0x300A;

/// Chip ID low byte, `0x300B` `SC_CMMN_CHIP_ID`, read-only value `0x47`.
pub const REG_CHIP_ID_L: u16 = 0x300B;

/// Expected value of `{REG_CHIP_ID_H, REG_CHIP_ID_L}`.
pub const CHIP_ID: u16 = 0x5647;

/// `0x3034` `SC_CMMN_PLL_CTRL0` (S8 table 6-1): `[6:4]` = `pll_charge_pump`,
/// `[3:0]` = `mipi_bit_mode` (`0x0` = 8-bit, `0x1` = 10-bit); reset `0x1A`.
pub const REG_PLL_CTRL0: u16 = 0x3034;

/// `0x3035` `SC_CMMN_PLL_CTRL1` (S8 table 6-1): `[7:4]` = `system_clk_div`
/// (slows every clock), `[3:0]` = `scale_divider_mipi`; reset `0x11`.
pub const REG_PLL_CTRL1: u16 = 0x3035;

/// `0x3036` `SC_CMMN_PLL_MULTIPLIER` (S8 table 6-1): `[7:0]` = PLL multiplier,
/// 4..252 (integer 4..127, even only above 127); reset `0x69`.
pub const REG_PLL_CTRL2: u16 = 0x3036;

/// `0x3037` `SC_CMMN_PLL_CTR13` (S8 table 6-1): `[4]` = `pll_root_div`
/// (`0` bypass, `1` = /2), `[3:0]` = `pll_prediv` (1, 2, 3, 4, 6, 8); reset
/// `0x03`.
pub const REG_PLL_CTRL3: u16 = 0x3037;

/// `0x4800` `MIPI CTRL 00` (S8 table 6-9, sheet 1 of 8): the transmitter's
/// lane/clock-lane and sync controls whose bit fields are defined below.
pub const REG_MIPI_CTRL0: u16 = 0x4800;

/// `MIPI CTRL 00` (`0x4800`) bit fields. Source: OV5647 datasheet, table 6-9
/// (sheet 1 of 8). The same three fields, under these names, are defined in
/// esp-video's `private_include/ov5647_settings.h` (`// S12`; its
/// `private_include/ov5647_regs.h` holds only register addresses and the two
/// sentinels, no bit fields). Only the fields this crate uses are listed here;
/// the datasheet also defines `hs_only` (bit 7), `ck_mark1_en` (bit 6),
/// `lane_select` (bit 3) and `clock_lane_first_bits` (bit 1).
pub const MIPI_CTRL00_CLOCK_LANE_GATE: u8 = 1 << 5;
/// 1 = send a line start short packet for every line.
pub const MIPI_CTRL00_LINE_SYNC_ENABLE: u8 = 1 << 4;
/// 1 = the bus idles in LP-11 (no LP-00) when no packet is transmitted.
pub const MIPI_CTRL00_BUS_IDLE: u8 = 1 << 2;
/// 1 = force the clock lane into low power mode.
pub const MIPI_CTRL00_CLOCK_LANE_DISABLE: u8 = 1 << 0;

/// `0x4800 = 0x24`: clock lane gated (non-continuous clock) and the bus idling
/// in LP-11, without line-start short packets. The composite is what esp-video's
/// `ov5647_set_stream` writes (`// S10`), and every esp-video 2-lane mode table
/// ends with the same value (S1, S2).
pub const MIPI_CTRL00_NONCONT_LP11: u8 = MIPI_CTRL00_CLOCK_LANE_GATE | MIPI_CTRL00_BUS_IDLE;

/// `0x4800 = 0x34`: as above plus a line-start short packet per line. Kept
/// because it is the other legal composite of these fields; no table in this
/// crate uses it.
pub const MIPI_CTRL00_NONCONT_LP11_LINESYNC: u8 =
    MIPI_CTRL00_NONCONT_LP11 | MIPI_CTRL00_LINE_SYNC_ENABLE;

/// `0x4800 = 0x25`: as [`MIPI_CTRL00_NONCONT_LP11`] plus forcing the clock lane
/// into low power. This is `ov5647_set_stream(enable = false)`'s value
/// (`// S10`), and it is what [`crate::sensor::Ov5647::stop_stream`] writes.
pub const MIPI_CTRL00_NONCONT_LP11_CLOCK_OFF: u8 =
    MIPI_CTRL00_NONCONT_LP11 | MIPI_CTRL00_CLOCK_LANE_DISABLE;

/// `0x4801` `MIPI CTRL 01` (S8 table 6-9): long/short packet data-type manual
/// enables `[7:6]`, the short-packet word-counter select `[5]`, the three ECC
/// bit/byte-order selects `[4:2]` and `mark1_en1`/`mark1_en2` `[1:0]`. It holds
/// no lane count: the lane count is `0x3018[7:5]`.
pub const REG_MIPI_CTRL1: u16 = 0x4801;

/// `0x4802` `MIPI CTRL 02` (S8 table 6-9).
pub const REG_MIPI_CTRL2: u16 = 0x4802;

/// `0x4814` `MIPI CTRL14` (S8 table 6-9): `[7:6]` virtual channel, `[5:0]` the
/// manual data type.
pub const REG_MIPI_CTRL: u16 = 0x4814;

/// `0x4837` `PCLK_PERIOD` (S8 table 6-9): "period of pclk2x, pclk_div = 1", in
/// quarter-nanosecond units. It is a timing value, not a per-lane bit-rate
/// factor: 80 MHz pclk gives a 6.25 ns pclk2x period, i.e. `0x19`.
pub const REG_MIPI_BITRATE: u16 = 0x4837;

/// Vertical timing: total lines, high byte (`0x380E` `TIMING_VTS`, S8 table 6-4).
pub const REG_TIMING_VTS_H: u16 = 0x380E;

/// Vertical timing: total lines, low byte (`0x380F`).
pub const REG_TIMING_VTS_L: u16 = 0x380F;

/// Horizontal timing: total pixels, high byte (`0x380C` `TIMING_HTS`, S8 table 6-4).
pub const REG_TIMING_HTS_H: u16 = 0x380C;

/// Horizontal timing: total pixels, low byte (`0x380D`).
pub const REG_TIMING_HTS_L: u16 = 0x380D;

// ---------------------------------------------------------------------------
// Table-walker sentinels
// ---------------------------------------------------------------------------

// Two "addresses" in a table are not registers, they are control codes for the
// table walker (esp-video's `ov5647_regs.h`).
/// Ends a register table. An entry with this address is never written.
pub const REG_END: u16 = 0xFFFF;

/// Delay, not a write: the entry's value is a delay in milliseconds.
pub const REG_DELAY: u16 = 0xEEEE;

/// A single SCCB register write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reg {
    /// The SCCB register address (or a table-walker sentinel like
    /// [`REG_END`] / [`REG_DELAY`]).
    pub addr: u16,
    /// The value to write to `addr` (or a delay in milliseconds when `addr` is
    /// [`REG_DELAY`]).
    pub value: u8,
}

impl Reg {
    /// Const-fn constructor, so register tables can be written as `Reg::new(..)`.
    pub const fn new(addr: u16, value: u8) -> Self {
        Self { addr, value }
    }
}
