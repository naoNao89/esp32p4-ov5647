//! Camera modes: a typed selector over the sensor's register tables.
//!
//! A driver user should not have to edit a build-time constant to switch modes. [`CameraMode`]
//! is the whole public surface for "which mode"; every geometry, wire format and transfer size
//! the rest of the driver needs is derived from it, and the register table that programs the
//! sensor is reached through [`CameraMode::spec`].
//!
//! ```
//! use esp32p4_ov5647::CameraMode;
//!
//! let mode = CameraMode::Raw10_1920x1080;
//! assert_eq!(mode.width(), 1920);
//! assert_eq!(mode.height(), 1080);
//! assert_eq!(mode.frame_len(), 2_592_000);
//! assert_eq!(mode.transfer_words(), 324_000);
//! assert_eq!(mode.block_ts(), 323_999);
//! ```
//!
//! Every mode in this enum has been captured end to end on this board;
//! [`CameraMode::is_hardware_verified`] reports that, and stays as the place a mode added later
//! starts out unverified. The 2592x1944 table is derived here from the OV5647 datasheet and the
//! Apache-2.0 esp-video reference; the 800x1280 table is that reference's, with two entries
//! corrected from measurement (`registers::OV5647_MODE_800X1280_RAW8_50FPS` records both).

use crate::sensor::ModeSpec;
use ov5647_core::PixelFormat;

/// Which sensor mode to run.
///
/// The variants that matter in practice are [`Self::Raw8_800x640`],
/// [`Self::Raw8_800x1280`], [`Self::Raw10_1920x1080`] and [`Self::Raw10_2592x1944`]: those are
/// captured end to end, with the frame preserved in PSRAM, transferred to a host and CRC-checked.
/// See [`Self::is_hardware_verified`].
///
/// Mode selection is a value, not a build-time constant. Pass it to [`crate::camera::Camera`] (or
/// to [`crate::csi::CsiCapture`] directly) and the driver derives the geometry, wire format,
/// transfer word count and `BLOCK_TS` from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum CameraMode {
    /// 800x640 RAW8 at 50 fps nominal, 2 lanes, 25 MHz XCLK: 2x subsampled.
    ///
    /// 512,000 bytes packed, 64,000 64-bit words, `BLOCK_TS` 63,999.
    /// **Captured end to end on ESP32-P4 rev 1.3.**
    Raw8_800x640 = 0,
    /// 800x800 RAW8 at 50 fps nominal, 2 lanes: the same window as
    /// [`Self::Raw8_800x640`], taller readout.
    ///
    /// 640,000 bytes packed, 80,000 64-bit words, `BLOCK_TS` 79,999.
    /// **Captured end to end on ESP32-P4 rev 1.3**: the host read all 640,000 bytes and agreed
    /// with the device's CRC-32/ISO-HDLC. Payload mean 21.6, SD 8.4, scene structure present.
    Raw8_800x800 = 1,
    /// 800x1280 RAW8 at 50 fps nominal, 2 lanes: its own window (x 512..2621, y 248..1695),
    /// not the 800x640/800x800 crop.
    ///
    /// 1,024,000 bytes packed, 128,000 64-bit words, `BLOCK_TS` 127,999.
    /// **Captured end to end on ESP32-P4 rev 1.3**: the host read all 1,024,000 bytes and agreed
    /// with the device's CRC-32/ISO-HDLC.
    ///
    /// The table is the pinned esp-video reference's, with two entries corrected after
    /// measurement: the AEC gain ceiling (upstream `0x0022` = 2.125x pinned the AGC at its
    /// ceiling and left the frame flat - mean 6.0, max 0x0F, SD 1.1 - where the working RAW8
    /// tables use `0x00F8` = 15.5x) and the ISP output window offset, which the reference leaves
    /// at its reset default and which therefore read the window's extreme left edge.
    /// `registers::OV5647_MODE_800X1280_RAW8_50FPS` records both with their numbers, including
    /// what was measured and rejected. Its frames are dimmer than [`Self::Raw8_800x640`]'s, and
    /// interleaved pairs measure how much: four pairs captured seconds apart in one session, in
    /// both orders, read 22.06-22.62 mean over the payload for 800x640 against 15.28-15.37 here —
    /// 1.44-1.48x at every pair. Both ran with the AGC at the gain ceiling (`0x350A`/`0x350B` read
    /// back `0x00F8`, the ceiling in both), and their exposure settings were 984 lines here against
    /// 980 there — but those sit in different frames: this mode's VTS is 1732, so 984 lines is 57%
    /// of the frame period and a choice, while 800x640's VTS is 984, so its 980 lines are the frame
    /// limit. Their tables also read different windows with different subsampling (S3 2x2, S5
    /// none), so the gap is not separable into table and scene. Absolute levels are scene- and
    /// lighting-dependent: an earlier session measured 30.70/30.78 here against 30.44 there.
    Raw8_800x1280 = 2,
    /// 1280x960 RAW10 at 45 fps nominal, 2 lanes, 2x2 binned.
    ///
    /// 1,536,000 bytes packed, 192,000 64-bit words, `BLOCK_TS` 191,999.
    /// **Captured end to end on ESP32-P4 rev 1.3**: the host read all 1,536,000 bytes and agreed
    /// with the device's CRC-32/ISO-HDLC. The payload unpacks to a scene image (mean 240,
    /// SD 92.8) like the other RAW10 modes, not the flat data a wrong crop produces.
    Raw10_1280x960 = 3,
    /// 1920x1080 RAW10 at 30 fps nominal, 2 lanes, 25 MHz XCLK.
    ///
    /// 2,592,000 bytes packed, 324,000 64-bit words, `BLOCK_TS` 323,999.
    /// **Captured end to end on ESP32-P4 rev 1.3**, soaked for 1,001 frames.
    ///
    /// A 1,001-frame soak reaches the nominal 30 fps at minimal workload: 33.359 s in each of two
    /// runs — 30.01 fps, 33,325.9 us per frame, p50 cadence 33,323 us. Nine instrumentation
    /// checkpoints cost 35.747 s in each of two runs here (28.00 fps, 35,711.2 us per frame), and
    /// 35.692 s in an earlier run of the same configuration (28.04 fps), so the release log's 28.04
    /// is a magnitude match, not an identification — the logged run's workload is not recorded.
    /// The 2.388 s difference is measured, not attributed: 2.293 s is the checkpoints themselves
    /// (254.8 ms each: 153.9 ms histogram, 96.2 ms nonzero, 4.6 ms for six SCCB reads, 0.02 ms
    /// snapshot) and the remaining 94.8 ms is the frames that follow a checkpoint (~299.9 ms
    /// against the 33.32 ms median: 90.9 ms), 3.6 ms of re-arm and 0.3 ms of loop bookkeeping. That log labels 28.04 as minimal
    /// and 25.9 as instrumented, and neither label reproduces here. The interval's p50/p95/p99 are
    /// the same in both configurations. The sensor's own frame period has not been measured by an
    /// independent time source, so 30.0 is reported as the achieved rate, not as the sensor's
    /// period.
    Raw10_1920x1080 = 4,
    /// 2592x1944 RAW10 at 15 fps (14.97 by construction: HTS 2885 x VTS 1968 at an
    /// 85 MHz pclk), 2 lanes, 25 MHz XCLK: full field of view, no binning.
    ///
    /// 6,298,560 bytes packed, 787,320 64-bit words, `BLOCK_TS` 787,319.
    /// **Captured end to end on ESP32-P4 rev 1.3**: the host read all 6,298,560
    /// bytes and agreed with the device's CRC-32/ISO-HDLC.
    ///
    /// Its table is derived here from the OV5647 datasheet and the Apache-2.0 esp-video
    /// reference (see `registers::OV5647_MODE_2592X1944_RAW10` for the group-by-group
    /// derivation); it is not one of the esp-video mode tables.
    Raw10_2592x1944 = 5,
}

impl CameraMode {
    /// Every mode, in ascending enum order. Useful for probes and for tests that must cover all of
    /// them.
    pub const ALL: [CameraMode; 6] = [
        CameraMode::Raw8_800x640,
        CameraMode::Raw8_800x800,
        CameraMode::Raw8_800x1280,
        CameraMode::Raw10_1280x960,
        CameraMode::Raw10_1920x1080,
        CameraMode::Raw10_2592x1944,
    ];

    /// The register table and link parameters that program this mode.
    ///
    /// This is the value [`crate::sensor::Ov5647::configure`] applies, and the source of every
    /// geometry constant the CSI/ISP/DMA side is configured with.
    pub const fn spec(self) -> ModeSpec {
        match self {
            CameraMode::Raw8_800x640 => ModeSpec::mode_800x640_raw8_50fps(),
            CameraMode::Raw8_800x800 => ModeSpec::mode_800x800_raw8_50fps(),
            CameraMode::Raw8_800x1280 => ModeSpec::mode_800x1280_raw8_50fps(),
            CameraMode::Raw10_1280x960 => ModeSpec::mode_1280x960_raw10_45fps(),
            CameraMode::Raw10_1920x1080 => ModeSpec::mode_1920x1080_raw10_30fps(),
            CameraMode::Raw10_2592x1944 => ModeSpec::mode_2592x1944_raw10(),
        }
    }

    /// Human-readable name, for logs.
    pub const fn name(self) -> &'static str {
        self.spec().name
    }

    /// Output width in pixels.
    pub const fn width(self) -> u32 {
        self.spec().width
    }

    /// Output height in pixels.
    pub const fn height(self) -> u32 {
        self.spec().height
    }

    /// Wire format the sensor is programmed for, which is also the format the ISP's input module is
    /// told to expect.
    ///
    /// For every mode here the memory-side format equals the wire format, so the ISP is bypassed
    /// and no conversion happens. See [`crate::isp`] for why that matters: the ISP output formatter
    /// has no RAW10 setting, so a converting RAW10 path does not exist on this silicon.
    pub const fn format(self) -> PixelFormat {
        self.spec().format
    }

    /// MIPI CSI-2 data lanes the mode's table enables.
    pub const fn lanes(self) -> u8 {
        self.spec().lanes
    }

    /// Per-lane MIPI CSI-2 line rate in bit/s. The receiver's `mipi_clk` is this value.
    pub const fn mipi_bitrate_bps(self) -> u32 {
        self.spec().mipi_bitrate_bps
    }

    /// XCLK the mode's PLL programming assumes, in Hz.
    pub const fn xclk_hz(self) -> u32 {
        self.spec().xclk_hz
    }

    /// Bytes one frame occupies in memory: `width * height * bits_per_pixel / 8`.
    ///
    /// For RAW10 this is the *packed* size — 4 pixels per 5 bytes — not two bytes per pixel.
    pub const fn frame_len(self) -> usize {
        (self.width() as usize) * (self.height() as usize) * (self.bits_per_pixel() as usize) / 8
    }

    /// Bits per pixel on the wire and in memory.
    ///
    /// `8` for RAW8, `10` for RAW10, `16` for RGB565, `24` for RGB888. Note that this is *not*
    /// the stride of a single pixel in memory for RAW10 (which is packed 4 pixels into 5 bytes);
    /// see [`Self::frame_len`].
    pub const fn bits_per_pixel(self) -> u32 {
        match self.format() {
            PixelFormat::Raw8 => 8,
            PixelFormat::Raw10 => 10,
            PixelFormat::Rgb565 => 16,
            PixelFormat::Rgb888 => 24,
        }
    }

    /// Frame size in 64-bit words, which is the unit the DW-GDMA block transfer is expressed in.
    ///
    /// All six modes are a whole number of 64-bit words: 512,000 B = 64,000 words,
    /// 2,592,000 B = 324,000 words, 6,298,560 B = 787,320 words.
    pub const fn transfer_words(self) -> u32 {
        (self.frame_len() / 8) as u32
    }

    /// The value for the DW-GDMA `CH1_BLOCK_TS` register, which is the word count minus one.
    ///
    /// `BLOCK_TS` is a 22-bit field holding a *count minus one*, so the largest transferable block
    /// is 4,194,304 words. Programming a raw word count would be silently masked to 22 bits and the
    /// engine would move a shorter block while reporting success; that is why this is derived here
    /// and validated rather than computed at each write site.
    ///
    /// For the largest mode here, 6,298,560 bytes = 787,320 words, `BLOCK_TS` 787,319, which is
    /// well inside the field.
    pub const fn block_ts(self) -> u32 {
        self.transfer_words() - 1
    }

    /// The number of bytes a frame needs in the destination buffer, rounded up to a whole number of
    /// 64-byte cache lines.
    ///
    /// The cache sync that follows a DMA transfer has to cover the engine's writes exactly, so both
    /// the buffer base address and this length must be multiples of the cache line size. All six
    /// modes happen to need no rounding; this exists for tables that would.
    pub const fn cache_aligned_len(self) -> usize {
        let len = self.frame_len();
        (len + 63) & !63
    }

    /// Whether this mode has been captured end to end on ESP32-P4 revision 1.3 hardware.
    ///
    /// `true` means: sensor configured and read back, frame transferred into a caller-owned PSRAM
    /// buffer by the DW-GDMA, announced as whole, and checksummed by a host that agreed with the
    /// device's CRC. All six modes qualify.
    ///
    /// A mode added later starts here as `false`, and the variant's own documentation says so:
    /// treat an unverified mode as unproven — its table may be right, but no capture backs it.
    ///
    /// Note what this flag does *not* claim: that the frame is well exposed or well cropped. A
    /// table can be wrong about exposure or window and still produce a whole, CRC-correct frame —
    /// which is exactly what [`Self::Raw8_800x1280`] did before two of its entries were corrected
    /// from measurement.
    pub const fn is_hardware_verified(self) -> bool {
        matches!(
            self,
            CameraMode::Raw8_800x640
                | CameraMode::Raw8_800x800
                | CameraMode::Raw8_800x1280
                | CameraMode::Raw10_1280x960
                | CameraMode::Raw10_1920x1080
                | CameraMode::Raw10_2592x1944
        )
    }
}

// Some modes' sizes are contracts, not incidental arithmetic: they are the numbers the host tool,
// the release notes and the DMA bounds tests all quote. Pin them at compile time so a change to a
// table's window cannot silently change what "a frame" means.
const _: () = assert!(CameraMode::Raw8_800x640.frame_len() == 512_000);
const _: () = assert!(CameraMode::Raw8_800x640.transfer_words() == 64_000);
const _: () = assert!(CameraMode::Raw8_800x640.block_ts() == 63_999);
const _: () = assert!(CameraMode::Raw8_800x1280.frame_len() == 1_024_000);
const _: () = assert!(CameraMode::Raw8_800x1280.transfer_words() == 128_000);
const _: () = assert!(CameraMode::Raw8_800x1280.block_ts() == 127_999);
const _: () = assert!(CameraMode::Raw10_1920x1080.frame_len() == 2_592_000);
const _: () = assert!(CameraMode::Raw10_1920x1080.transfer_words() == 324_000);
const _: () = assert!(CameraMode::Raw10_1920x1080.block_ts() == 323_999);
const _: () = assert!(CameraMode::Raw10_2592x1944.frame_len() == 6_298_560);
const _: () = assert!(CameraMode::Raw10_2592x1944.transfer_words() == 787_320);
const _: () = assert!(CameraMode::Raw10_2592x1944.block_ts() == 787_319);

// Which modes are claimed as captured is a contract too — a variant that keeps the claim after its
// table changes, or a new mode that starts out claiming it, would publish a false statement. The
// crate's `#[cfg(test)]` module cannot run in CI here: the HAL only builds for the riscv target, so
// `cargo test` on the host cannot compile this crate. Pin the set where the build does see it.
const VERIFIED_MODE_COUNT: usize = {
    let mut i = 0;
    let mut n = 0;
    while i < CameraMode::ALL.len() {
        if CameraMode::ALL[i].is_hardware_verified() {
            n += 1;
        }
        i += 1;
    }
    n
};
const _: () = assert!(
    VERIFIED_MODE_COUNT == 6,
    "all six modes are captured on this board"
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_mode_is_in_all() {
        assert_eq!(CameraMode::ALL.len(), 6);
        for m in CameraMode::ALL {
            assert!(CameraMode::ALL.contains(&m));
        }
    }

    #[test]
    fn verified_modes_are_exactly_the_six_captured_on_hardware() {
        let verified: Vec<_> = CameraMode::ALL
            .into_iter()
            .filter(CameraMode::is_hardware_verified)
            .collect();
        assert_eq!(
            verified,
            vec![
                CameraMode::Raw8_800x640,
                CameraMode::Raw8_800x800,
                CameraMode::Raw8_800x1280,
                CameraMode::Raw10_1280x960,
                CameraMode::Raw10_1920x1080,
                CameraMode::Raw10_2592x1944
            ]
        );
    }

    #[test]
    fn frame_len_matches_the_wire_format() {
        // RAW8: one byte per pixel.
        assert_eq!(CameraMode::Raw8_800x640.frame_len(), 800 * 640);
        assert_eq!(CameraMode::Raw8_800x1280.frame_len(), 800 * 1280);
        // RAW10 packed: 4 pixels per 5 bytes, so width*height*10/8.
        assert_eq!(
            CameraMode::Raw10_1920x1080.frame_len(),
            1920 * 1080 * 10 / 8
        );
        // The packed size is not two bytes per pixel.
        assert_ne!(CameraMode::Raw10_1920x1080.frame_len(), 1920 * 1080 * 2);
    }

    #[test]
    fn block_ts_is_words_minus_one_and_fits_the_field() {
        for m in CameraMode::ALL {
            assert_eq!(m.block_ts(), m.transfer_words() - 1, "{m:?}");
            // The DW-GDMA block-transfer size field is 22 bits.
            assert!(m.block_ts() < (1 << 22), "{m:?} does not fit BLOCK_TS");
        }
    }

    #[test]
    fn every_mode_is_a_whole_number_of_words_and_cache_lines() {
        for m in CameraMode::ALL {
            assert_eq!(m.frame_len() % 8, 0, "{m:?} is not a whole word count");
            assert_eq!(m.frame_len() % 64, 0, "{m:?} is not a whole cache line");
            assert_eq!(m.cache_aligned_len(), m.frame_len(), "{m:?}");
        }
    }

    #[test]
    fn spec_geometry_agrees_with_the_selector() {
        for m in CameraMode::ALL {
            let s = m.spec();
            assert_eq!(
                (s.width, s.height, s.lanes),
                (m.width(), m.height(), m.lanes())
            );
            assert_eq!(s.format, m.format());
            assert_eq!(s.mipi_bitrate_bps, m.mipi_bitrate_bps());
            assert_eq!(s.table.len() > 0, true, "{m:?} has an empty table");
        }
    }
}
