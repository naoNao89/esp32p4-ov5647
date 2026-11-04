//! Hardware-independent logic for the OV5647 → ESP32-P4 camera path.
//!
//! Everything here is pure: no register access, no HAL, no `unsafe`. That is
//! deliberate — the firmware crate only builds for `riscv32imafc-unknown-none-elf`
//! and needs real hardware to run, so the parts that *can* be checked on a
//! development host (chip-ID decoding, frame sizing, checksums and the host
//! transfer framing) live here and are covered by `cargo test`.
//!
//! This crate links `no_std` on the target and is host-tested with
//! `cargo test --features std`. It exists so the hardware contracts — geometry
//! (frame sizing, ISP word counts), DMA bounds (destination regions, block-transfer
//! field width, cache-line alignment) and frame integrity (ownership, sentinels,
//! row completeness, exposure statistics) — are testable without a board, on the
//! exact logic the firmware ships.
//!
//! The firmware's own HAL-free modules — its register addresses, its derived register
//! tables and its OV5647 driver — are compiled into the test build as well (see the
//! `#[path]` modules at the end of this file), so the tables and the driver's table
//! walker are covered by the same `cargo test --features std` run, on the exact
//! source the firmware ships.
//!
//! # Citations
//!
//! The note name used below (`ov5647-registers.md`) refers to a project research
//! note kept in the author's private reference archive, outside this repository.
//! It has no SPDX header and is **not shipped here**, so it is background rather
//! than a reference a reader can open. What backs a claim is the upstream source
//! named beside it (esp-video), or a constant quoted inline that a test pins. The
//! upstream attribution of the register tables themselves is the `S<n>` legend in
//! the firmware crate's `src/registers.rs`.

// `no_std` for the firmware build; the host test harness needs `std`.
#![cfg_attr(not(feature = "std"), no_std)]
// Every public item must be documented; `missing_docs` is a warning, not a deny.
#![warn(missing_docs)]

use core::fmt::Write as _;

pub mod dma_bounds;
pub mod frame_integrity;
pub mod frame_layout;
pub mod frame_stats;
#[cfg(feature = "std")]
pub mod interval_stats;
pub mod isp_geometry;
#[cfg(feature = "std")]
pub mod row_stats;

/// 7-bit SCCB address of the OV5647.
pub const OV5647_SCCB_ADDR: u8 = 0x36;

/// IC identifier returned by registers `0x300A`/`0x300B`.
pub const OV5647_CHIP_ID: u16 = 0x5647;

/// Combines the two chip-ID bytes (`0x300A` high, `0x300B` low), MSB first.
pub const fn chip_id_from_bytes(high: u8, low: u8) -> u16 {
    ((high as u16) << 8) | low as u16
}

/// True when `id` is the OV5647's identifier.
pub const fn chip_id_matches(id: u16) -> bool {
    id == OV5647_CHIP_ID
}

/// Sensor-side pixel formats this driver can configure and transfer.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PixelFormat {
    /// One byte per pixel, no packing.
    Raw8,
    /// Ten bits per pixel, packed 4 pixels into 5 bytes.
    Raw10,
    /// Two bytes per pixel, little endian.
    Rgb565,
    /// Three bytes per pixel.
    Rgb888,
}

impl PixelFormat {
    /// Token used by the text header on the wire.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Raw8 => "RAW8",
            Self::Raw10 => "RAW10",
            Self::Rgb565 => "RGB565",
            Self::Rgb888 => "RGB888",
        }
    }

    /// Parses the wire token; case sensitive, matching [`Self::name`].
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "RAW8" => Some(Self::Raw8),
            "RAW10" => Some(Self::Raw10),
            "RGB565" => Some(Self::Rgb565),
            "RGB888" => Some(Self::Rgb888),
            _ => None,
        }
    }

    /// Bytes a full frame of `width` x `height` occupies on the wire.
    ///
    /// RAW10 is the packed MIPI CSI-2 form (4 pixels per 5 bytes), which is what
    /// the receiver DMA writes and what the host has to unpack.
    pub const fn frame_len_bytes(self, width: u32, height: u32) -> u32 {
        let pixels = width as u64 * height as u64;
        let bits = pixels * bits_per_pixel(self) as u64;
        (bits / 8) as u32
    }
}

/// Bits one pixel of `format` occupies on the CSI-2 link.
pub const fn bits_per_pixel(format: PixelFormat) -> u32 {
    match format {
        PixelFormat::Raw8 => 8,
        PixelFormat::Raw10 => 10,
        PixelFormat::Rgb565 => 16,
        PixelFormat::Rgb888 => 24,
    }
}

/// Per-lane MIPI CSI-2 line rate of a sensor mode, in bit/s:
/// `pclk * bits-per-pixel / lanes`.
///
/// This is the rate the CSI/ISP receiver has to be told (esp-video feeds the same
/// number in as `mipi_info.mipi_clk`); the clock lane runs at half of it. Returns
/// `0` for `lanes == 0` so a caller cannot divide by zero.
///
/// See `ov5647-registers.md` §5.4.
pub const fn mipi_bitrate_bps(pclk_hz: u32, format: PixelFormat, lanes: u8) -> u32 {
    if lanes == 0 {
        return 0;
    }
    (pclk_hz as u64 * bits_per_pixel(format) as u64 / lanes as u64) as u32
}

/// VCO of the sensor PLL, as the vendor system-clock routine computes it:
/// `VCO = XCLK * 2 / pre_div * multiplier`, with the pre-divider 6 that
/// `0x3037 = 0x03` (its reset value) selects.
///
/// The product is formed in 64 bits and divided once, so the arithmetic comes out
/// exactly: XCLK 25 MHz with multiplier 105 is 875 000 000 Hz. That figure is this
/// formula evaluated at those register settings — a derived rate, not one a
/// published table states.
///
/// See `ov5647-registers.md` §5.2 and esp-video `ov5647.c`'s
/// `ov5647_get_sysclk()` (S10).
pub const fn vco_hz(xclk_hz: u32, pll_multiplier: u32) -> u32 {
    (xclk_hz as u64 * 2 * pll_multiplier as u64 / 6) as u32
}

/// Pixel clock a mode runs at, given its XCLK and the `0x3036` multiplier in its
/// table, for the 10-bit programming every RAW10 table uses: `0x3034 = 0x1A`
/// (`bit_div2x = 5`), `0x3035 = 0x21` (`system_clk_div = 2`), `0x3037 = 0x03`
/// (`pll_root_div = 1`) and `0x3106 = 0xF5` (`sclk_div = 2`), so
/// `pclk = VCO * 2 / 2 / 1 / 5 / 2 = VCO / 10`.
///
/// esp-video's metadata quotes pclk rounded (81 666 700 Hz for multiplier 98, where
/// this returns 81 666 666), so compare with a tolerance of a few hundred Hz. The
/// RAW8 tables use a different divider chain and are not covered by this function.
///
/// See `ov5647-registers.md` §5.2 and S12 for the quoted constant.
pub const fn pclk_hz_10bit(xclk_hz: u32, pll_multiplier: u32) -> u32 {
    vco_hz(xclk_hz, pll_multiplier) / 10
}

/// CRC-32/ISO-HDLC (the zlib/PNG polynomial `0xEDB88320`), reflected, init
/// `0xFFFF_FFFF`, final xor `0xFFFF_FFFF`.
///
/// Implemented here rather than pulled from a crate so the firmware, the host
/// script and the tests share one definition.
pub fn crc32_ieee(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

/// Header text that introduces a binary frame payload.
pub const FRAME_HEADER_PREFIX: &str = "<<<FRAME";
/// Trailer text that follows the payload.
pub const FRAME_TRAILER: &str = "<<<END>>>";

/// Metadata the firmware announces before each frame payload.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct FrameHeader {
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Pixel format of the payload, which determines its packed byte length.
    pub format: PixelFormat,
    /// Length in bytes of the binary payload that follows the header.
    pub payload_len: u32,
    /// CRC-32/ISO-HDLC of the payload, for the host to verify after transfer.
    pub crc32: u32,
}

impl FrameHeader {
    /// Renders the header as a single self-describing line, without newline.
    ///
    /// Format: `<<<FRAME w=1920 h=1080 fmt=RAW10 len=2592000 crc32=0x1A2B3C4D>>>`
    ///
    /// # Errors
    ///
    /// Returns `None` if `out` is too small to hold the rendered line. The caller
    /// should size the buffer from [`Self::write_into`]'s worst-case formatted
    /// length (a 96-byte buffer comfortably fits every format).
    pub fn write_into(self, out: &mut [u8]) -> Option<usize> {
        let mut sink = Sink { buf: out, len: 0 };
        write!(
            sink,
            "{} w={} h={} fmt={} len={} crc32=0x{:08X}>>>",
            FRAME_HEADER_PREFIX,
            self.width,
            self.height,
            self.format.name(),
            self.payload_len,
            self.crc32
        )
        .ok()?;
        Some(sink.len)
    }

    /// Parses a header line produced by [`Self::write_into`].
    ///
    /// Rejects unknown tokens and inconsistent fields so that the host cannot
    /// silently accept a truncated or mislabelled frame.
    ///
    /// # Errors
    ///
    /// Returns `None` for any malformed input: a missing/unknown token, a field
    /// that does not parse, or a `payload_len` inconsistent with the declared
    /// geometry (the payload length must equal
    /// `format.frame_len_bytes(width, height)`). This is deliberate — a header
    /// that fails its own consistency check is corrupt before the payload is read.
    pub fn parse(line: &str) -> Option<Self> {
        let line = line.trim();
        let body = line
            .strip_prefix(FRAME_HEADER_PREFIX)?
            .strip_suffix(">>>")?
            .trim();

        let mut width = None;
        let mut height = None;
        let mut format = None;
        let mut payload_len = None;
        let mut crc32 = None;

        for token in body.split_whitespace() {
            let (key, value) = token.split_once('=')?;
            match key {
                "w" => width = Some(value.parse::<u32>().ok()?),
                "h" => height = Some(value.parse::<u32>().ok()?),
                "fmt" => format = Some(PixelFormat::from_name(value)?),
                "len" => payload_len = Some(value.parse::<u32>().ok()?),
                "crc32" => crc32 = Some(u32::from_str_radix(value.strip_prefix("0x")?, 16).ok()?),
                _ => return None,
            }
        }

        let header = Self {
            width: width?,
            height: height?,
            format: format?,
            payload_len: payload_len?,
            crc32: crc32?,
        };

        // Self-consistency: a frame claiming a size that does not match its own
        // geometry is corrupt even before the payload is read.
        if header.payload_len != header.format.frame_len_bytes(header.width, header.height) {
            return None;
        }
        Some(header)
    }
}

/// Minimal `core::fmt::Write` sink over a caller-provided buffer.
struct Sink<'a> {
    buf: &'a mut [u8],
    len: usize,
}

impl core::fmt::Write for Sink<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        if self.len + bytes.len() > self.buf.len() {
            return Err(core::fmt::Error);
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chip_id_is_big_endian_over_the_two_registers() {
        assert_eq!(chip_id_from_bytes(0x56, 0x47), OV5647_CHIP_ID);
        assert!(chip_id_matches(chip_id_from_bytes(0x56, 0x47)));
        // A swapped read must not be mistaken for the real part.
        assert_eq!(chip_id_from_bytes(0x47, 0x56), 0x4756);
        assert!(!chip_id_matches(chip_id_from_bytes(0x47, 0x56)));
        assert!(!chip_id_matches(0x0000));
    }

    #[test]
    fn frame_sizes_match_the_wire_format() {
        // RAW8: one byte per pixel.
        assert_eq!(PixelFormat::Raw8.frame_len_bytes(800, 640), 512_000);
        // RAW10 is packed, 4 pixels per 5 bytes.
        assert_eq!(PixelFormat::Raw10.frame_len_bytes(1920, 1080), 2_592_000);
        assert_eq!(PixelFormat::Raw10.frame_len_bytes(4, 1), 5);
        assert_eq!(PixelFormat::Rgb565.frame_len_bytes(640, 480), 614_400);
        assert_eq!(PixelFormat::Rgb888.frame_len_bytes(640, 480), 921_600);
    }

    #[test]
    fn pixel_format_names_round_trip() {
        for fmt in [
            PixelFormat::Raw8,
            PixelFormat::Raw10,
            PixelFormat::Rgb565,
            PixelFormat::Rgb888,
        ] {
            assert_eq!(PixelFormat::from_name(fmt.name()), Some(fmt));
        }
        assert_eq!(PixelFormat::from_name("raw10"), None);
        assert_eq!(PixelFormat::from_name("YUV422"), None);
    }

    #[test]
    fn crc32_matches_the_standard_check_vector() {
        // "123456789" -> 0xCBF43926 is the canonical CRC-32/ISO-HDLC check.
        assert_eq!(crc32_ieee(b"123456789"), 0xCBF4_3926);
        assert_eq!(crc32_ieee(b""), 0x0000_0000);
    }

    #[test]
    fn header_round_trips() {
        let header = FrameHeader {
            width: 1920,
            height: 1080,
            format: PixelFormat::Raw10,
            payload_len: 2_592_000,
            crc32: 0x1A2B_3C4D,
        };
        let mut buf = [0u8; 96];
        let len = header.write_into(&mut buf).unwrap();
        let line = core::str::from_utf8(&buf[..len]).unwrap();
        assert_eq!(
            line,
            "<<<FRAME w=1920 h=1080 fmt=RAW10 len=2592000 crc32=0x1A2B3C4D>>>"
        );
        assert_eq!(FrameHeader::parse(line), Some(header));
    }

    #[test]
    fn header_parser_rejects_corrupt_headers() {
        // Payload length inconsistent with the geometry.
        assert_eq!(
            FrameHeader::parse("<<<FRAME w=1920 h=1080 fmt=RAW10 len=123 crc32=0x00000000>>>"),
            None
        );
        // Unknown format token.
        assert_eq!(
            FrameHeader::parse("<<<FRAME w=1 h=1 fmt=YUV len=2 crc32=0x00000000>>>"),
            None
        );
        // Missing field.
        assert_eq!(
            FrameHeader::parse("<<<FRAME w=1 h=1 fmt=RAW8 crc32=0x00000000>>>"),
            None
        );
        // Not a header at all.
        assert_eq!(FrameHeader::parse("garbage"), None);
    }

    #[test]
    fn mipi_bit_rate_matches_the_reference_modes() {
        // ov5647-registers.md §5.4 for the esp-video modes; 2592x1944
        // takes the pclk its own table's multiplier 102 produces (85 MHz, HTS 2885 —
        // the blanking width this board's bring-up needed).
        assert_eq!(
            mipi_bitrate_bps(81_666_700, PixelFormat::Raw10, 2),
            408_333_500,
            "1920x1080 RAW10"
        );
        assert_eq!(
            mipi_bitrate_bps(88_333_333, PixelFormat::Raw10, 2),
            441_666_665,
            "1280x960 RAW10"
        );
        assert_eq!(
            mipi_bitrate_bps(85_000_000, PixelFormat::Raw10, 2),
            425_000_000,
            "2592x1944 RAW10"
        );
        assert_eq!(
            mipi_bitrate_bps(100_000_000, PixelFormat::Raw8, 2),
            400_000_000,
            "800x* RAW8"
        );

        // It is the documented formula, not a fitted table.
        assert_eq!(
            mipi_bitrate_bps(81_666_700, PixelFormat::Raw10, 2),
            81_666_700 * 10 / 2
        );
        assert_eq!(
            mipi_bitrate_bps(100_000_000, PixelFormat::Raw8, 2),
            100_000_000 * 8 / 2
        );
        assert_eq!(
            mipi_bitrate_bps(81_666_700, PixelFormat::Raw10, 1),
            816_667_000
        );
        // No lanes is not a division by zero.
        assert_eq!(mipi_bitrate_bps(81_666_700, PixelFormat::Raw10, 0), 0);

        // And the per-pixel widths the frame sizes are built from.
        assert_eq!(bits_per_pixel(PixelFormat::Raw8), 8);
        assert_eq!(bits_per_pixel(PixelFormat::Raw10), 10);
        assert_eq!(bits_per_pixel(PixelFormat::Rgb565), 16);
        assert_eq!(bits_per_pixel(PixelFormat::Rgb888), 24);
    }

    #[test]
    fn pll_maths_reproduce_the_reference_pixel_clocks() {
        // §5.2: XCLK 25 MHz, 0x3037 = 0x03 -> pre-divisor 6, 0x3036 = 0x69 (105).
        // VCO 875 MHz and pclk 87.5 MHz are that formula evaluated at those values.
        assert_eq!(vco_hz(25_000_000, 105), 875_000_000);
        assert_eq!(pclk_hz_10bit(25_000_000, 105), 87_500_000);

        // 0x3036 = 0x62 (98) is the 1080p multiplier: pclk 81 666 666 Hz, which
        // esp-video's metadata quotes rounded to 81 666 700 Hz (§5.2, S12).
        assert_eq!(vco_hz(25_000_000, 98), 816_666_666);
        assert_eq!(pclk_hz_10bit(25_000_000, 98), 81_666_666);
        assert!(81_666_700 - pclk_hz_10bit(25_000_000, 98) < 100);

        // The product must be formed in 64 bits: 25 MHz * 2 * 255 does not fit u32.
        assert_eq!(
            vco_hz(25_000_000, 0xff),
            (25_000_000u64 * 2 * 255 / 6) as u32
        );
    }
}

// ---------------------------------------------------------------------------
// The firmware crate's HAL-free modules, compiled into the tests below
// ---------------------------------------------------------------------------
//
// These are the firmware's own source files: its register addresses, its derived
// register tables and its OV5647 driver. None of them names a HAL type (`ov5647.rs`
// uses `embedded-hal` traits only, which is a dev-dependency of this crate), so they
// compile unchanged on the host and the tests can walk the very same tables and run
// the very same driver code the firmware ships, instead of a copy that could drift.
//
// `ov5647.rs` refers to this crate as `ov5647_core::…`, the way the firmware does;
// `extern crate self` gives the test build that same spelling.

#[cfg(test)]
extern crate self as ov5647_core;

/// The firmware's register addresses, identifiers and walker sentinels.
///
/// This is the firmware's own `src/regs.rs`, compiled unchanged into the test
/// build so the tables and driver walk the exact addresses the firmware ships.
#[cfg(test)]
#[path = "../../../src/regs.rs"]
pub mod regs;

/// The firmware's register tables.
///
/// This is the firmware's own `src/registers.rs`, compiled unchanged into the
/// test build so the tests walk the exact tables the firmware ships. Every entry
/// carries the tag of the file it was taken from and that file's legend resolves the
/// tag to a URL; the tables' sources are the Espressif esp-video mode headers,
/// settings and register list (S1-S5, S10, S12, S14, Apache-2.0) and the OV5647
/// datasheet (S8).
#[cfg(test)]
#[path = "../../../src/registers.rs"]
pub mod registers;

/// The firmware's OV5647 driver: SCCB transport, mode configuration, streaming.
///
/// This is the firmware's own `src/sensor.rs`, compiled unchanged into the test
/// build so the tests run the exact driver code the firmware ships.
#[cfg(test)]
#[path = "../../../src/sensor.rs"]
pub mod sensor;

/// How the firmware's tables and driver behave, with a recording bus and clock.
#[cfg(test)]
mod firmware {
    use super::{bits_per_pixel, mipi_bitrate_bps, pclk_hz_10bit, vco_hz, PixelFormat};
    use crate::registers::{self, ALL_TABLES};
    use crate::regs::{
        Reg, CHIP_ID, MIPI_CTRL00_NONCONT_LP11, MIPI_CTRL00_NONCONT_LP11_CLOCK_OFF, REG_CHIP_ID_H,
        REG_CHIP_ID_L, REG_DELAY, REG_END, REG_MIPI_CTRL0, REG_PLL_CTRL2, REG_SW_RESET,
        REG_SW_STANDBY,
    };
    use crate::sensor::{ChipIdOutcome, ModeSpec, Ov5647, SccbError, RESET_DELAY_MS};

    use embedded_hal::delay::DelayNs;
    use embedded_hal::i2c::{
        ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
    };

    /// Stand-in SCCB bus: records every register write, answers the chip-ID read
    /// with `chip_id`, and can NACK a chosen register or every read.
    struct MockBus {
        chip_id: [u8; 2],
        nack_on: Option<u16>,
        nack_reads: bool,
        pending_reg: Option<u16>,
        writes: Vec<(u16, u8)>,
    }

    impl MockBus {
        fn new() -> Self {
            Self {
                chip_id: [0x56, 0x47],
                nack_on: None,
                nack_reads: false,
                pending_reg: None,
                writes: Vec::new(),
            }
        }

        fn with_chip_id(chip_id: [u8; 2]) -> Self {
            Self {
                chip_id,
                ..Self::new()
            }
        }

        /// NACKs every transaction that names `reg`, like a register this part does
        /// not implement.
        fn nack_on(reg: u16) -> Self {
            Self {
                nack_on: Some(reg),
                ..Self::new()
            }
        }

        /// NACKs the data phase, like a sensor that is not answering.
        fn nack_reads() -> Self {
            Self {
                nack_reads: true,
                ..Self::new()
            }
        }

        fn check(&self, reg: u16) -> Result<(), ErrorKind> {
            if self.nack_on == Some(reg) {
                return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Data));
            }
            Ok(())
        }
    }

    impl ErrorType for MockBus {
        type Error = ErrorKind;
    }

    impl I2c for MockBus {
        fn transaction(
            &mut self,
            _address: SevenBitAddress,
            operations: &mut [Operation<'_>],
        ) -> Result<(), ErrorKind> {
            for operation in operations {
                match operation {
                    // Three bytes are `reg_hi reg_lo value`; two bytes are the
                    // address phase of a read, which SCCB terminates with a STOP
                    // instead of a repeated START.
                    Operation::Write(bytes) => match bytes {
                        [high, low, value] => {
                            let reg = u16::from_be_bytes([*high, *low]);
                            self.check(reg)?;
                            self.writes.push((reg, *value));
                        }
                        [high, low] => {
                            let reg = u16::from_be_bytes([*high, *low]);
                            self.check(reg)?;
                            self.pending_reg = Some(reg);
                        }
                        _ => {}
                    },
                    Operation::Read(buf) => {
                        if self.nack_reads {
                            return Err(ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address));
                        }
                        // Model the attached part faithfully: one register per
                        // address phase, and no auto-increment. A burst read
                        // returns the first byte and zeros after it -- on real
                        // hardware the rest come back as 0xFF, which is what
                        // made the first bring-up report PID=0x56FF.
                        let first = match self.pending_reg {
                            Some(REG_CHIP_ID_H) => self.chip_id[0],
                            Some(REG_CHIP_ID_L) => self.chip_id[1],
                            _ => 0,
                        };
                        for (index, slot) in buf.iter_mut().enumerate() {
                            *slot = if index == 0 { first } else { 0 };
                        }
                    }
                }
            }
            Ok(())
        }
    }

    /// Records the delays the driver asks for, in milliseconds.
    #[derive(Default)]
    struct MockDelay {
        ms: Vec<u32>,
    }

    impl DelayNs for MockDelay {
        fn delay_ns(&mut self, ns: u32) {
            self.ms.push(ns / 1_000_000);
        }
    }

    /// Value last written to `addr` in `table`, if any.
    fn table_value(table: &[Reg], addr: u16) -> Option<u8> {
        table
            .iter()
            .filter(|r| r.addr == addr)
            .last()
            .map(|r| r.value)
    }

    /// Every entry that is a real register write, i.e. the table minus its delay and
    /// its terminator.
    fn real_writes(table: &[Reg]) -> Vec<(u16, u8)> {
        table
            .iter()
            .take_while(|r| r.addr != REG_END)
            .filter(|r| r.addr != REG_DELAY)
            .map(|r| (r.addr, r.value))
            .collect()
    }

    // --- the firmware's tables -----------------------------------------------

    #[test]
    fn every_exposed_table_is_non_empty_and_terminated() {
        assert_eq!(ALL_TABLES.len(), 9, "the firmware exposes nine tables");
        for (name, table) in ALL_TABLES {
            assert!(!table.is_empty(), "{name} is empty");
            assert_eq!(
                table.iter().filter(|r| r.addr == REG_END).count(),
                1,
                "{name} must contain exactly one REG_END"
            );
            assert_eq!(
                table.last().map(|r| r.addr),
                Some(REG_END),
                "{name}: REG_END only ends a table, so it must be the last entry"
            );
        }
    }

    #[test]
    fn the_reset_table_is_the_reference_reset_sequence() {
        // S12 (`ov5647_mipi_reset_regs[]`): stream off, software reset, 10 ms
        // delay, clock lane forced to LP-11.
        assert_eq!(
            registers::OV5647_RESET[..4],
            [
                Reg::new(0x0100, 0x00),
                Reg::new(0x0103, 0x01),
                Reg::new(REG_DELAY, 0x0a),
                Reg::new(0x4800, 0x01),
            ]
        );
    }

    #[test]
    fn the_1080p30_table_ends_with_the_expected_mipi_control_writes() {
        let table = registers::OV5647_MODE_1920X1080_RAW10_30FPS;
        let writes = real_writes(table);
        // The last two writes are the esp-video 1080p table's (S1): 0x4837 = 0x19
        // (PCLK_PERIOD for the 81.67 MHz pclk) and 0x4800 = 0x24, which S1 itself
        // labels "Non-continuous clock, no line sync short packets". The 0x24 bits
        // are the datasheet's MIPI_CTRL00 description (S8): bit 5 gates the clock
        // lane, and bit 2 is a status bit that reads 1 while the bus idles in LP-11.
        assert_eq!(
            writes[writes.len() - 2..],
            [(0x4837, 0x19), (REG_MIPI_CTRL0, MIPI_CTRL00_NONCONT_LP11)]
        );
        // The table programs the PLL multiplier the mode's pixel clock comes from,
        // and leaves 0x0100 to the driver.
        assert_eq!(table_value(table, REG_PLL_CTRL2), Some(0x62));
        assert_eq!(table_value(table, REG_SW_STANDBY), None);
    }

    #[test]
    fn mode_1080p30_declares_the_line_rate_its_pll_produces() {
        let mode = ModeSpec::mode_1920x1080_raw10_30fps();
        assert_eq!((mode.width, mode.height), (1920, 1080));
        assert_eq!(mode.format, PixelFormat::Raw10);
        assert_eq!(mode.lanes, 2);
        assert_eq!(mode.xclk_hz, 25_000_000);

        // The declared rate is pclk * 10 / 2 for RAW10 on 2 lanes, with the pclk
        // esp-video's metadata declares for this mode (§5.2, §5.4, S12).
        let pclk = 81_666_700;
        assert_eq!(mode.mipi_bitrate_bps, pclk * 10 / 2);
        assert_eq!(
            mode.mipi_bitrate_bps,
            mipi_bitrate_bps(pclk, mode.format, mode.lanes)
        );
        assert_eq!(mode.mipi_bitrate_bps, 408_333_500);
        // The clock lane runs at half the line rate.
        assert_eq!(mode.mipi_bitrate_bps / 2, 204_166_750);

        // The table's multiplier is what produces that pclk from the mode's XCLK:
        // VCO 816.67 MHz, pclk 81 666 666 Hz (quoted rounded as 81 666 700).
        let multiplier = u32::from(table_value(mode.table, REG_PLL_CTRL2).expect("0x3036"));
        assert_eq!(multiplier, 98);
        assert_eq!(vco_hz(mode.xclk_hz, multiplier), 816_666_666);
        let derived = pclk_hz_10bit(mode.xclk_hz, multiplier);
        assert_eq!(derived, 81_666_666);
        assert!(pclk - derived < 100, "{derived} is not the quoted {pclk}");
    }

    #[test]
    fn every_mode_declares_the_reference_line_rate() {
        // (mode, pclk). The five esp-video modes carry the pclk their own metadata
        // declares (§5.4, S12); 2592x1944's 85 MHz is what its PLL settings
        // produce — vco_hz(25 MHz, 102) / 10.
        for (mode, pclk) in [
            (ModeSpec::mode_1920x1080_raw10_30fps(), 81_666_700),
            (ModeSpec::mode_1280x960_raw10_45fps(), 88_333_333),
            (ModeSpec::mode_2592x1944_raw10(), 85_000_000),
            (ModeSpec::mode_800x640_raw8_50fps(), 100_000_000),
            (ModeSpec::mode_800x800_raw8_50fps(), 100_000_000),
            (ModeSpec::mode_800x1280_raw8_50fps(), 100_000_000),
        ] {
            let name = mode.name;
            assert!(mode.width > 0 && mode.height > 0, "{name}: empty geometry");
            assert_eq!(mode.lanes, 2, "{name}");
            assert_eq!(
                mode.xclk_hz, 25_000_000,
                "{name}: the published tables assume a 25 MHz XCLK"
            );
            assert_eq!(
                mode.mipi_bitrate_bps,
                mipi_bitrate_bps(pclk, mode.format, mode.lanes),
                "{name}: declared rate is not pclk x bits-per-pixel / lanes"
            );
            assert_eq!(
                mode.mipi_bitrate_bps,
                pclk * bits_per_pixel(mode.format) / u32::from(mode.lanes),
                "{name}"
            );
            assert_eq!(
                mode.table.last().map(|r| r.addr),
                Some(REG_END),
                "{name}: mode table is not terminated"
            );
        }
    }

    // --- the table walker ----------------------------------------------------

    #[test]
    fn the_walker_treats_the_sentinels_as_control_codes() {
        let mut ov = Ov5647::with_default_addr(MockBus::new());
        let mut delay = MockDelay::default();
        let table = [
            Reg::new(0x3000, 0x11),
            Reg::new(REG_DELAY, 0x1e), // 30 ms
            Reg::new(0x3001, 0x22),
            Reg::new(REG_END, 0x00),
            Reg::new(0x3002, 0x33), // after the terminator: never written
        ];
        ov.write_table(&table, &mut delay).unwrap();

        let bus = ov.release();
        assert_eq!(bus.writes, [(0x3000, 0x11), (0x3001, 0x22)]);
        assert_eq!(delay.ms, [30], "0xEEEE is a delay, not a write");
    }

    #[test]
    fn the_reset_table_walks_as_stream_off_reset_delay_lp11() {
        let mut ov = Ov5647::with_default_addr(MockBus::new());
        let mut delay = MockDelay::default();
        ov.write_table(registers::OV5647_RESET, &mut delay).unwrap();

        let bus = ov.release();
        assert_eq!(bus.writes, [(0x0100, 0x00), (0x0103, 0x01), (0x4800, 0x01)]);
        assert_eq!(delay.ms, [10]);
    }

    #[test]
    fn a_failed_write_names_its_register() {
        let mut ov = Ov5647::with_default_addr(MockBus::nack_on(0x3001));
        let table = [
            Reg::new(0x3000, 0x11),
            Reg::new(0x3001, 0x22),
            Reg::new(REG_END, 0x00),
        ];
        assert_eq!(
            ov.write_table(&table, &mut MockDelay::default()),
            Err((Reg::new(0x3001, 0x22), SccbError::Nack))
        );
    }

    // --- configure and streaming ---------------------------------------------

    #[test]
    fn configure_applies_the_reference_reset_then_the_mode_table() {
        let mode = ModeSpec::mode_1920x1080_raw10_30fps();

        let mut ov = Ov5647::with_default_addr(MockBus::new());
        let mut delay = MockDelay::default();
        ov.configure(&mode, &mut delay).unwrap();

        let bus = ov.release();
        let reset = super::registers::OV5647_RESET;

        // 1. The ported reference reset table runs first, verbatim and in order:
        //    standby, software reset, the reset delay, then the MIPI clock-lane
        //    disable that drives the link to LP-11.
        let reset_writes = real_writes(reset);
        assert_eq!(&bus.writes[..reset_writes.len()], &reset_writes[..]);
        assert_eq!(
            reset_writes,
            [
                (REG_SW_STANDBY, 0x00),
                (REG_SW_RESET, 0x01),
                (REG_MIPI_CTRL0, 0x01),
            ]
        );
        assert_eq!(delay.ms.first(), Some(&RESET_DELAY_MS));

        // 2. The mode table follows, also in order and with sentinels excluded.
        assert_eq!(
            &bus.writes[reset_writes.len()..],
            &real_writes(mode.table)[..]
        );

        // 3. The invariant that matters: `configure` leaves the streaming gate
        //    OFF. It now *writes* 0x0100 (the reset table's standby entry) rather
        //    than never touching it, so assert the state, not the absence.
        let last_gate = bus
            .writes
            .iter()
            .rev()
            .find(|(addr, _)| *addr == REG_SW_STANDBY);
        assert_eq!(
            last_gate,
            Some(&(REG_SW_STANDBY, 0x00)),
            "configure must leave streaming to start_stream/stop_stream"
        );

        // ...and it ends with the mode table's two MIPI-control writes.
        assert_eq!(
            bus.writes[bus.writes.len() - 2..],
            [(0x4837, 0x19), (REG_MIPI_CTRL0, MIPI_CTRL00_NONCONT_LP11)]
        );
    }

    #[test]
    fn streaming_sequences_write_the_documented_registers_in_order() {
        let mut ov = Ov5647::with_default_addr(MockBus::new());
        ov.start_stream().unwrap();
        ov.stop_stream().unwrap();

        let bus = ov.release();
        assert_eq!(
            bus.writes,
            [
                (REG_MIPI_CTRL0, MIPI_CTRL00_NONCONT_LP11), // 0x4800 = 0x24
                (REG_SW_STANDBY, 0x01),                     // 0x0100 = 0x01
                (REG_SW_STANDBY, 0x00),                     // 0x0100 = 0x00
                (REG_MIPI_CTRL0, MIPI_CTRL00_NONCONT_LP11_CLOCK_OFF), // 0x4800 = 0x25
            ]
        );
        // The composites are the values the research doc lists (§6).
        assert_eq!(MIPI_CTRL00_NONCONT_LP11, 0x24);
        assert_eq!(MIPI_CTRL00_NONCONT_LP11_CLOCK_OFF, 0x25);
    }

    #[test]
    fn a_stream_write_failure_names_its_register() {
        let mut ov = Ov5647::with_default_addr(MockBus::nack_on(REG_SW_STANDBY));
        assert_eq!(
            ov.start_stream(),
            Err((Reg::new(REG_SW_STANDBY, 0x01), SccbError::Nack))
        );
    }

    // --- identification ------------------------------------------------------

    #[test]
    fn identify_separates_a_wrong_part_from_a_dead_bus() {
        let mut ov = Ov5647::with_default_addr(MockBus::with_chip_id([0x56, 0x47]));
        assert_eq!(ov.identify(), ChipIdOutcome::Match(CHIP_ID));

        let mut ov = Ov5647::with_default_addr(MockBus::with_chip_id([0x56, 0x40]));
        assert_eq!(ov.identify(), ChipIdOutcome::Mismatch(0x5640));

        let mut ov = Ov5647::with_default_addr(MockBus::nack_reads());
        assert_eq!(ov.identify(), ChipIdOutcome::BusError(SccbError::Nack));
    }
}
