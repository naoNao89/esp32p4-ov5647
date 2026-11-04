//! OV5647 register tables.
//!
//! Data only: `pub const` slices of [`crate::regs::Reg`] and no code, so the
//! driver can apply a table with `Ov5647::write_table` and the host tests in
//! `crates/ov5647-core` can walk the very same bytes.
//!
//! Every table was either transcribed from an Apache-2.0 source or derived here;
//! each entry states its basis at the end of its line (`// S<n>`, legend below).
//! Entries this repository changed after measuring them are marked `// corrected`
//! or `// added` instead of carrying a source tag, and the table's own
//! `# Measurement` section gives the numbers. Where a value was an arithmetic
//! expression upstream it is written here as the evaluated literal with the
//! original expression kept in the comment, e.g.
//! `// S3 expr:(100_000_000*8*4)/25_000_000`.
//!
//! Two conventions this file follows:
//!
//! * **Register addresses and identity constants are not repeated.** They already
//!   exist, with the same values and sources, in [`crate::regs`]:
//!   [`crate::regs::SCCB_ADDR`] is the SCCB address;
//!   [`crate::regs::CHIP_ID`], [`crate::regs::REG_CHIP_ID_H`] and
//!   [`crate::regs::REG_CHIP_ID_L`] are the chip identity;
//!   [`crate::regs::REG_SW_RESET`], [`crate::regs::REG_SW_STANDBY`] and
//!   [`crate::regs::REG_MIPI_CTRL0`] are the control registers the stream
//!   sequences use; and the `OV5647_MIPI_CTRL00_*` bit fields with their
//!   `0x24`/`0x25` composites are the [`crate::regs::MIPI_CTRL00_CLOCK_LANE_GATE`]
//!   family there.
//! * **Every table ends with the [`crate::regs::REG_END`] sentinel**, so the
//!   terminator is part of the data instead of an implicit length. The walker stops
//!   there; see [`crate::sensor::Ov5647::write_table`].
//!
//! # Sentinel values
//!
//! Neither `0xFFFF` nor `0xEEEE` is a register. They are control codes for the
//! table walker: [`crate::regs::REG_END`] ends a table, and
//! [`crate::regs::REG_DELAY`] delays its value in milliseconds.
//!
//! # Source legend
//!
//! `S1`–`S5`, `S10`, `S12` and `S14` are files from
//! <https://github.com/espressif/esp-video-components> (`esp_cam_sensor/sensors/ov5647/`),
//! all `SPDX-License-Identifier: Apache-2.0`; the blob ids pin the revision each
//! one was read at. `S8` and `S9` are documents of OmniVision Technologies that are
//! **cited, not reproduced**: no text, table or figure from them appears in this
//! repository.
//!
//! * `S1`  <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_mipi_2lane_24Minput_1920x1080_raw10_30fps.h>
//!         (blob df986da962c7fe2ca0d4006757b8bd3d0e68dfcb, fetched 2026-10-31)
//! * `S2`  <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_mipi_2lane_24Minput_1280x960_raw10_45fps.h>
//!         (blob 8e5f37a0efa62ee2b8bf4a560d1ab0fe074b08e4)
//! * `S3`  <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_mipi_2lane_24Minput_800x640_raw8_50fps.h>
//!         (blob 85af25b0ccecae13a476595acc8a92c86869d427)
//! * `S4`  <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_mipi_2lane_24Minput_800x800_raw8_50fps.h>
//!         (blob b407fe7eaed57a8690af4fa48d01b92d42e1872a)
//! * `S5`  <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_mipi_2lane_24Minput_800x1280_raw8_50fps.h>
//!         (blob f41ac5ca6985e0d0eff4ab7badccc1db475fe193)
//! * `S8`  <https://cdn.sparkfun.com/datasheets/Dev/RaspberryPi/ov5647_full.pdf>
//!         (OV5647 datasheet, preliminary v1.0; register descriptions, timing and
//!         PLL field lists — cited only)
//! * `S9`  <https://files.waveshare.com/upload/1/14/OmniVision_Technologies_Seril_Camera_Control_Bus(SCCB)_Specification.pdf>
//!         (OmniVision SCCB Functional Specification v2.1, 2003-02-26 — cited only)
//! * `S10` <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/ov5647.c>
//!         (blob 928aad33347b97a417f1c3f5acac2a0e9a4a5924)
//! * `S12` <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_settings.h>
//!         (blob aff082d189fb579c0ca53395c8b492cd1b97da48)
//! * `S14` <https://github.com/espressif/esp-video-components/blob/master/esp_cam_sensor/sensors/ov5647/private_include/ov5647_regs.h>
//!         (blob d35598306b4ed3ffe788d2344939156cbaccd57e)
//!

use crate::regs::{
    Reg, MIPI_CTRL00_BUS_IDLE, MIPI_CTRL00_CLOCK_LANE_DISABLE, MIPI_CTRL00_CLOCK_LANE_GATE,
    REG_DELAY, REG_END,
};

// ---------------------------------------------------------------------------
// Reset sequence (sensor reset / transport reset)
// ---------------------------------------------------------------------------

/// Reset sequence: stream off, software reset, 10 ms delay, then force the MIPI clock
/// lane to LP-11 while the stream is off. `// S12` (`ov5647_mipi_reset_regs[]`)
pub const OV5647_RESET: &[Reg] = &[
    Reg::new(0x0100, 0x00),    // S12 enable sleep (stream off)
    Reg::new(0x0103, 0x01),    // S12 software reset
    Reg::new(REG_DELAY, 0x0a), // S12 delay 10 ms
    Reg::new(0x4800, 0x01),    // S12 clock lane disable -> LP-11
    Reg::new(REG_END, 0x00),   // S14 end of table
];

// ---------------------------------------------------------------------------
// Mode: 2592x1944 @ 15 fps, RAW10, 2 lanes, 25 MHz XCLK (derived here)
// ---------------------------------------------------------------------------

/// Full-resolution 2592x1944 RAW10 at 15 fps on 2 lanes with a 25 MHz XCLK.
///
/// # Derivation record
///
/// This table was **independently derived** from the OV5647 datasheet (S8) and
/// the Apache-2.0 esp-video reference (S2 for the analog/ISP/AEC register block
/// and the MIPI values, S10/S12/S14 for the stream and sentinel conventions).
/// Register values are technical facts of the sensor and may coincide with any
/// other driver's; what this note establishes is the derivation path and the
/// attribution that goes with it.
///
/// Basis, group by group:
///
/// * **PLL** — the register block esp-video uses for its RAW10 modes
///   (`0x3034 = 0x1A` 10-bit MIPI, `0x3035 = 0x21`, `0x303C = 0x11`,
///   `0x3106 = 0xF5`, `0x3037` at its reset `0x03`, i.e. `pll_prediv = 6` and
///   `pll_root_div` bypass; field lists in S8 table 6-1, the same values in S2).
///   With that configuration the pixel clock is `XCLK * multiplier / 30`: it
///   reproduces both esp-video RAW10 modes exactly from a 25 MHz XCLK
///   (S1 multiplier 98 -> 81 666 700 Hz and S2 multiplier 106 -> 88 333 333 Hz,
///   both from `ov5647_settings.h`). The datasheet's format/frame-rate table
///   states 80 MHz for 5 Mpixel 2592x1944 at 15 fps; `0x3036 = 0x66`
///   (multiplier 102) gives 85 000 000 Hz, the pixel clock this readout needs to
///   reach 15 fps with the horizontal blanking below. Measured on this board:
///   85.0 MHz and 425 Mbit/s per lane capture the whole frame (see "Measurement").
/// * **Frame timing** — the line is 2600 pixels wide (the datasheet's window)
///   plus blanking, and the sensor needs that blanking. esp-video expresses the
///   line size as window + 285 where it writes the window as macros
///   (S5's `TOTAL_H_SIZE = HWIN + 285`), and on this board a line of `2592 + 118`
///   columns produces corrupt output whereas `2600 + 285` does not. So
///   `HTS = 2885` (S8's `TIMING_HTS` field layout; value from that convention
///   plus the measurement below). `VTS = 1968` is the 1956-row array plus 12
///   blanking rows and is the datasheet's own reset value for `0x380E`/`0x380F`.
///   That gives `85 000 000 / (2885 * 1968)` = 14.97 fps, the datasheet's 15 fps
///   within 0.2%.
/// * **Crop window and output size** — S8 §3.1 gives the array as 2624x1956 and
///   says the centre 2592x1944 is what should be output; its documented reset
///   values already describe that readout (x 12..2611, y 4..1955, ISP offsets
///   4/2, output 2592x1944). They are written out explicitly here rather than
///   relied on implicitly.
/// * **Subsampling, binning, flip, mirror** — `0x3814`/`0x3815` = `0x11`
///   (odd and even increment 1 = no subsampling) and `0x3820`/`0x3821` = `0x00`
///   (no vertical/horizontal binning, no flip, no mirror) per the field lists in
///   S8 tables 6-4/6-5. Written explicitly so that nothing about the full-array
///   readout depends on leftover state.
/// * **MIPI** — `0x3018 = 0x44` (two-lane MIPI with the PHY out of power-down:
///   S8 table 6-1 `mipi_lane_mode [7:5]` / `r_phy_pd_mipi [4]`; the same value in
///   S2) and `0x4800 = 0x24` (clock lane gated = non-continuous clock, bus idling
///   in LP-11, no line-start short packets: S8 table 6-9 bit fields, value as in
///   S2). `0x4837` is S8's `PCLK_PERIOD`, "period of pclk2x": pclk2x is 170 MHz,
///   a 5.88 ns period, and the register counts quarter-nanoseconds, so 24 = `0x18`
///   (rounded up; the same rule gives S1's `0x19` at 81.67 MHz and S2's `0x16` at
///   88.33 MHz).
/// * **Analog, ISP, AEC/AGC** — the register block of esp-video's 1280x960 RAW10
///   table (S2), which is the closest RAW10 mode that source publishes. Only the
///   mode-geometry registers of that block (PLL multiplier, HTS/VTS, window,
///   output size, subsampling) differ, and those are derived above.
/// * **Stream** — the table ends at the last configuration write. `0x0100` is
///   **not** written: [`crate::sensor::Ov5647::start_stream`] streams the sensor,
///   so a table that ended with `0x0100 = 0x01` would bring the link up while it
///   was still being configured.
///
/// # Measurement
///
/// Captured end to end on this board (ESP32-P4 rev 1.3 + OV5647, `tools/capture_frame.py`
/// `--port <usb port> --timeout 180 --payload-timeout 600 --out /tmp/cap2592`), one verified
/// run:
///
/// ```text
/// warm-up 0..7: words=787320 complete=true
/// frame complete: words=787320 expected=787320 complete=true
/// header:  <<<FRAME w=2592 h=1944 fmt=RAW10 len=6298560 crc32=0x535B6AF3>>>
/// frame:   2592x1944 RAW10 len=6298560 crc32=0x535B6AF3
/// verified: length OK, CRC-32/ISO-HDLC OK
/// payload: nonzero=6292726 distinct=256 min=0x00 max=0xFF
/// ```
///
/// So the DW-GDMA completed the whole 6 298 560-byte frame (`BLOCK_TS` 787 319, device
/// `words=787320`), the CSI host reported no error (`INT_ST_MAIN` = 0), the bridge reported
/// none and drained to 0, and the host's CRC-32/ISO-HDLC over the received payload matched the
/// device's. The payload is scene data, so the CRC is a property of that capture, not a
/// constant to expect from every run; the length and the word count are the invariants.
///
/// The blanking requirement above is measured, not assumed. With the same table
/// but `HTS = 2710` (118 blanking columns) the sensor still streams — the
/// receiver sees the clock lane in HS and the bridge's buffer fills — but the CSI
/// host reports `BNDRY_FRAME_FATAL | PLD_CRC_FATAL`, the bridge discards, and the
/// DMA moves zero words on every arm. `HTS` 2885, 2943 and 3200 all complete the
/// frame; 2885, the `window + 285` value, is kept as the smallest that does.
///
/// Geometry: output 2592x1944 RAW10, full field of view, no binning. Packed frame
/// `2592 * 1944 * 10 / 8` = 6 298 560 bytes = 787 320 64-bit words. Per-lane line
/// rate `85 000 000 * 10 / 2` = 425 000 000 bit/s (clock lane 212 500 000 Hz).
pub const OV5647_MODE_2592X1944_RAW10: &[Reg] = &[
    // PLL and clock (S2 configuration; multiplier from the frame-timing derivation).
    Reg::new(0x3034, 0x1a), // S2 10-bit MIPI mode
    Reg::new(0x3035, 0x21), // S2 system_clk_div 2, scale_divider_mipi 1
    Reg::new(0x3036, 0x66), // derived: multiplier 102 -> pclk 85.0 MHz at 25 MHz XCLK
    Reg::new(0x303c, 0x11), // S2 plls_cp 1, plls_sys_div 1
    Reg::new(0x3106, 0xf5), // S2 PLL clock divider
    Reg::new(0x3827, 0xec), // S2
    Reg::new(0x370c, 0x03), // S2
    Reg::new(0x3612, 0x59), // S2
    Reg::new(0x3618, 0x00), // S2
    // ISP / lens / gain (S2).
    Reg::new(0x5000, 0xff), // S2 LENC en
    Reg::new(0x583e, 0xf0), // S2 max gain
    Reg::new(0x583f, 0x40), // S2 min gain
    Reg::new(0x5003, 0x08), // S2
    Reg::new(0x5a00, 0x08), // S2
    Reg::new(0x3000, 0x00), // S2
    Reg::new(0x3001, 0x00), // S2
    Reg::new(0x3002, 0x00), // S2
    Reg::new(0x3016, 0x08), // S2
    Reg::new(0x3017, 0xe0), // S2
    Reg::new(0x3018, 0x44), // S2 two-lane MIPI, PHY enabled (S8 table 6-1 bits[7:5]/[4])
    Reg::new(0x301c, 0xf8), // S2
    Reg::new(0x301d, 0xf0), // S2
    Reg::new(0x3a18, 0x03), // S2 gain ceiling
    Reg::new(0x3a19, 0xff), // S2 gain ceiling
    Reg::new(0x3c00, 0x40), // S2
    Reg::new(0x3b07, 0x0c), // S2
    // Frame timing (derived: HTS = window 2600 + 285 per S5's convention and this
    // board's measurement; VTS = the datasheet's reset value).
    Reg::new(0x380c, 0x0b), // derived: HTS = 0x0B45 = 2885 = 2600 window + 285 blanking
    Reg::new(0x380d, 0x45), // derived
    Reg::new(0x380e, 0x07), // S8: VTS = 0x07B0 = 1968 = 1956 array + 12 blanking
    Reg::new(0x380f, 0xb0), // S8
    // Subsampling, binning, flip, mirror: all off, full-array readout (S8 field lists).
    Reg::new(0x3814, 0x11), // S8 h_odd_inc = h_even_inc = 1, no horizontal subsampling
    Reg::new(0x3815, 0x11), // S8 v_odd_inc = v_even_inc = 1, no vertical subsampling
    Reg::new(0x3820, 0x00), // S8 r_vflip_isp/r_vflip_snr/r_vbin all 0
    Reg::new(0x3821, 0x00), // S8 r_mirror_isp/r_mirror_snr/r_hbin all 0
    // Window and output size: the datasheet's documented reset values (S8).
    Reg::new(0x3808, 0x0a), // S8 output width = 0x0A20 = 2592
    Reg::new(0x3809, 0x20), // S8
    Reg::new(0x380a, 0x07), // S8 output height = 0x0798 = 1944
    Reg::new(0x380b, 0x98), // S8
    Reg::new(0x3800, 0x00), // S8 x start = 0x00C = 12
    Reg::new(0x3801, 0x0c), // S8
    Reg::new(0x3802, 0x00), // S8 y start = 0x004 = 4
    Reg::new(0x3803, 0x04), // S8
    Reg::new(0x3804, 0x0a), // S8 x end = 0xA33 = 2611
    Reg::new(0x3805, 0x33), // S8
    Reg::new(0x3806, 0x07), // S8 y end = 0x7A3 = 1955
    Reg::new(0x3807, 0xa3), // S8
    Reg::new(0x3810, 0x00), // S8 ISP horizontal offset high byte
    Reg::new(0x3811, 0x04), // S8 ISP horizontal offset = 4
    Reg::new(0x3812, 0x00), // S8 ISP vertical offset high byte
    Reg::new(0x3813, 0x02), // S8 ISP vertical offset = 2
    // Analog block (S2).
    Reg::new(0x3630, 0x2e), // S2
    Reg::new(0x3632, 0xe2), // S2
    Reg::new(0x3633, 0x23), // S2
    Reg::new(0x3634, 0x44), // S2
    Reg::new(0x3636, 0x06), // S2
    Reg::new(0x3620, 0x64), // S2
    Reg::new(0x3621, 0xe0), // S2
    Reg::new(0x3600, 0x37), // S2
    Reg::new(0x3704, 0xa0), // S2
    Reg::new(0x3703, 0x5a), // S2
    Reg::new(0x3715, 0x78), // S2
    Reg::new(0x3717, 0x01), // S2
    Reg::new(0x3731, 0x02), // S2
    Reg::new(0x370b, 0x60), // S2
    Reg::new(0x3705, 0x1a), // S2
    Reg::new(0x3f05, 0x02), // S2
    Reg::new(0x3f06, 0x10), // S2
    Reg::new(0x3f01, 0x0a), // S2
    // AEC/AGC (S2).
    Reg::new(0x3a08, 0x01), // S2
    Reg::new(0x3a09, 0x28), // S2
    Reg::new(0x3a0a, 0x00), // S2
    Reg::new(0x3a0b, 0xf6), // S2
    Reg::new(0x3a0d, 0x08), // S2
    Reg::new(0x3a0e, 0x06), // S2
    Reg::new(0x3a0f, 0x58), // S2
    Reg::new(0x3a10, 0x50), // S2
    Reg::new(0x3a1b, 0x58), // S2
    Reg::new(0x3a1e, 0x50), // S2
    Reg::new(0x3a11, 0x60), // S2
    Reg::new(0x3a1f, 0x28), // S2
    // Black level (S2).
    Reg::new(0x4001, 0x02), // S2
    Reg::new(0x4004, 0x04), // S2 blc line number
    Reg::new(0x4000, 0x09), // S2
    // MIPI (value from S2, bit fields from S8 table 6-9).
    Reg::new(0x4837, 0x18), // derived: PCLK_PERIOD 24 quarter-ns = 6.0 ns (pclk2x 170 MHz)
    Reg::new(0x4800, 0x24), // S2/S8 non-continuous clock, LP-11 idle, no line-sync packets
    Reg::new(0x3212, 0xa0), // S2 last entry of the esp-video RAW10 tables; semantics unverified
    Reg::new(REG_END, 0x00), // S14 end of table
];

// ---------------------------------------------------------------------------
// Mode: 1920x1080 @ 30 fps, RAW10, 2 lanes  (Espressif esp-video, RECOMMENDED FIRST)
// ---------------------------------------------------------------------------

/// `ov5647_mipi_2lane_24Minput_1920x1080_raw10_30fps[]` from esp-video-components:
/// complete self-contained sequence (no separate common-init needed).
///
/// * resolution: 1920 x 1080 (centre crop of the full array), RAW10, 2 lanes `// S1`
/// * Bayer: reported as GBRG by esp-video (`ESP_CAM_SENSOR_BAYER_GBRG`) `// S10`
/// * hts = 0x08DF = 2271, vts = 0x04AF = 1199 (as written by this table) `// S1`
/// * pclk claimed by esp-video for this mode: 81 666 700 Hz; MIPI CSI line rate
///   408 333 500 Hz = pclk * 5 (= bits-per-pixel / lanes for RAW10 on 2 lanes) `// S12`
/// * XCLK: 0x3036 = 0x62 (multiplier 98) with 0x3034 = 0x1A, 0x3035 = 0x21,
///   0x3037 at reset 0x03 (pre-div 6, root div bypass) reproduces exactly the
///   25 MHz-derived pclk above: with this PLL configuration
///   pclk = XCLK * multiplier / 30 = 25 MHz * 98 / 30 = 81.67 MHz (the same
///   relation gives S2's 106 -> 88.33 MHz)
/// * 2 lanes, non-continuous clock without line-sync short packets (0x4800 = 0x24)
/// * table does NOT write 0x0100 (streaming is done by the driver) `// S1`, `// S10`
pub const OV5647_MODE_1920X1080_RAW10_30FPS: &[Reg] = &[
    Reg::new(0x3034, 0x1a),  // S1 10-bit mode
    Reg::new(0x3035, 0x21),  // S1
    Reg::new(0x3036, 0x62),  // S1 PLL multiplier 98
    Reg::new(0x303c, 0x11),  // S1
    Reg::new(0x3106, 0xf5),  // S1 PLL clock divider = /2
    Reg::new(0x3821, 0x02), // S1 mirror on (comment in source says "vertical, bit1: flip" - it is 0x3821[1] r_mirror_snr, see S8)
    Reg::new(0x3820, 0x00), // S1 no vertical flip / no binning
    Reg::new(0x3827, 0xec), // S1
    Reg::new(0x370c, 0x03), // S1
    Reg::new(0x3612, 0x5b), // S1
    Reg::new(0x3618, 0x04), // S1
    Reg::new(0x5000, 0xff), // S1 ISP LENC enable
    Reg::new(0x583e, 0xf0), // S1 LSC max gain
    Reg::new(0x583f, 0x4f), // S1 LSC min gain
    Reg::new(0x5003, 0x08), // S1
    Reg::new(0x5a00, 0x08), // S1
    Reg::new(0x3000, 0x00), // S1
    Reg::new(0x3001, 0x00), // S1
    Reg::new(0x3002, 0x00), // S1
    Reg::new(0x3016, 0x08), // S1
    Reg::new(0x3017, 0xe0), // S1
    Reg::new(0x3018, 0x44), // S1 2-lane MIPI
    Reg::new(0x301c, 0xf8), // S1
    Reg::new(0x301d, 0xf0), // S1
    Reg::new(0x3a18, 0x03), // S1 gain ceiling
    Reg::new(0x3a19, 0xff), // S1 gain ceiling
    Reg::new(0x3c00, 0x40), // S1
    Reg::new(0x3b07, 0x0c), // S1
    Reg::new(0x380c, 0x08), // S1 HTS = 0x08DF = 2271
    Reg::new(0x380d, 0xdf), // S1
    Reg::new(0x380e, 0x04), // S1 VTS = 0x04AF = 1199
    Reg::new(0x380f, 0xaf), // S1
    Reg::new(0x3814, 0x11), // S1 no horizontal subsampling
    Reg::new(0x3815, 0x11), // S1 no vertical subsampling
    Reg::new(0x3708, 0x64), // S1
    Reg::new(0x3709, 0x12), // S1
    Reg::new(0x3808, 0x07), // S1 output width = 0x0780 = 1920
    Reg::new(0x3809, 0x80), // S1
    Reg::new(0x380a, 0x04), // S1 output height = 0x0438 = 1080
    Reg::new(0x380b, 0x38), // S1
    Reg::new(0x3800, 0x01), // S1 x start = 0x015C = 348
    Reg::new(0x3801, 0x5c), // S1
    Reg::new(0x3802, 0x01), // S1 y start = 0x01B2 = 434
    Reg::new(0x3803, 0xb2), // S1
    Reg::new(0x3804, 0x08), // S1 x end = 0x08E3 = 2275
    Reg::new(0x3805, 0xe3), // S1
    Reg::new(0x3806, 0x05), // S1 y end = 0x05F1 = 1521
    Reg::new(0x3807, 0xf1), // S1
    Reg::new(0x3811, 0x04), // S1 x offset 4
    Reg::new(0x3813, 0x02), // S1 y offset 2
    Reg::new(0x3630, 0x2e), // S1
    Reg::new(0x3632, 0xe2), // S1
    Reg::new(0x3633, 0x23), // S1
    Reg::new(0x3634, 0x44), // S1
    Reg::new(0x3636, 0x06), // S1
    Reg::new(0x3620, 0x64), // S1
    Reg::new(0x3621, 0xe0), // S1
    Reg::new(0x3600, 0x37), // S1
    Reg::new(0x3704, 0xa0), // S1
    Reg::new(0x3703, 0x5a), // S1
    Reg::new(0x3715, 0x78), // S1
    Reg::new(0x3717, 0x01), // S1
    Reg::new(0x3731, 0x02), // S1
    Reg::new(0x370b, 0x60), // S1
    Reg::new(0x3705, 0x1a), // S1
    Reg::new(0x3f05, 0x02), // S1
    Reg::new(0x3f06, 0x10), // S1
    Reg::new(0x3f01, 0x0a), // S1
    Reg::new(0x3a08, 0x01), // S1 50 Hz step
    Reg::new(0x3a09, 0x4b), // S1
    Reg::new(0x3a0a, 0x01), // S1 60 Hz step
    Reg::new(0x3a0b, 0x13), // S1
    Reg::new(0x3a0d, 0x04), // S1
    Reg::new(0x3a0e, 0x03), // S1
    Reg::new(0x3a0f, 0x58), // S1
    Reg::new(0x3a10, 0x50), // S1
    Reg::new(0x3a1b, 0x58), // S1
    Reg::new(0x3a1e, 0x50), // S1
    Reg::new(0x3a11, 0x60), // S1
    Reg::new(0x3a1f, 0x28), // S1
    Reg::new(0x4001, 0x02), // S1
    Reg::new(0x4004, 0x04), // S1 BLC lines
    Reg::new(0x4000, 0x09), // S1
    Reg::new(0x4837, 0x19), // S1 MIPI PCLK_PERIOD = 25
    Reg::new(0x4800, 0x24), // S1 non-continuous clock, LP-11 idle, no line sync
    Reg::new(REG_END, 0x00), // S14 end of table
];

// ---------------------------------------------------------------------------
// Mode: 1280x960 @ 45 fps, RAW10 (2x2 binned), 2 lanes  (Espressif)
// ---------------------------------------------------------------------------

/// `ov5647_mipi_2lane_24Minput_1280x960_raw10_45fps[]`: self-contained sequence.
///
/// * resolution: 1280 x 960, 2x2 binned + subsampled, RAW10, 2 lanes `// S2`
/// * hts = 0x0704 = 1796, vts = 0x0445 = 1093 (the source comment says 0x768/0x7B0,
///   the encoded bytes are the authoritative 1796/1093) `// S2`
/// * pclk claimed: 88 333 333 Hz; MIPI line rate 441 666 665 Hz = pclk * 5 `// S12`
/// * PLL: 0x3036 = 0x6A (multiplier 106) -> again the 25 MHz XCLK numbers `// S2`
/// * 2 lanes, non-continuous clock, no line sync (0x4800 = 0x24) `// S2`
pub const OV5647_MODE_1280X960_RAW10_45FPS: &[Reg] = &[
    Reg::new(0x3034, 0x1a),  // S2 10-bit mode
    Reg::new(0x3035, 0x21),  // S2
    Reg::new(0x3036, 0x6a),  // S2 PLL multiplier 106
    Reg::new(0x303c, 0x11),  // S2
    Reg::new(0x3106, 0xf5),  // S2
    Reg::new(0x3820, 0x01),  // S2; bit 0 = r_vbin (S8): vertical binning on
    Reg::new(0x3821, 0x03),  // S2; bits 1 and 0 = r_mirror_snr, r_hbin (S8): mirror + h-binning
    Reg::new(0x3827, 0xec),  // S2
    Reg::new(0x370c, 0x03),  // S2
    Reg::new(0x3612, 0x59),  // S2
    Reg::new(0x3618, 0x00),  // S2
    Reg::new(0x5000, 0xff),  // S2 LENC enable
    Reg::new(0x583e, 0xf0),  // S2 LSC max gain
    Reg::new(0x583f, 0x40),  // S2 LSC min gain
    Reg::new(0x5003, 0x08),  // S2
    Reg::new(0x5a00, 0x08),  // S2
    Reg::new(0x3000, 0x00),  // S2
    Reg::new(0x3001, 0x00),  // S2
    Reg::new(0x3002, 0x00),  // S2
    Reg::new(0x3016, 0x08),  // S2
    Reg::new(0x3017, 0xe0),  // S2
    Reg::new(0x3018, 0x44),  // S2 2-lane MIPI
    Reg::new(0x301c, 0xf8),  // S2
    Reg::new(0x301d, 0xf0),  // S2
    Reg::new(0x3a18, 0x03),  // S2 gain ceiling
    Reg::new(0x3a19, 0xff),  // S2 gain ceiling
    Reg::new(0x3c00, 0x40),  // S2
    Reg::new(0x3b07, 0x0c),  // S2
    Reg::new(0x380c, 0x07),  // S2 HTS = 0x0704 = 1796
    Reg::new(0x380d, 0x04),  // S2
    Reg::new(0x380e, 0x04),  // S2 VTS = 0x0445 = 1093
    Reg::new(0x380f, 0x45),  // S2
    Reg::new(0x3814, 0x31),  // S2 2x horizontal subsampling
    Reg::new(0x3815, 0x31),  // S2 2x vertical subsampling
    Reg::new(0x3808, 0x05),  // S2 output width = 0x0500 = 1280
    Reg::new(0x3809, 0x00),  // S2
    Reg::new(0x380a, 0x03),  // S2 output height = 0x03C0 = 960
    Reg::new(0x380b, 0xc0),  // S2
    Reg::new(0x3800, 0x00),  // S2 x start = 0x0018 = 24
    Reg::new(0x3801, 0x18),  // S2
    Reg::new(0x3802, 0x00),  // S2 y start = 0x000C = 12
    Reg::new(0x3803, 0x0c),  // S2
    Reg::new(0x3804, 0x0a),  // S2 x end = 0x0A27 = 2599
    Reg::new(0x3805, 0x27),  // S2
    Reg::new(0x3806, 0x07),  // S2 y end = 0x0797 = 1943
    Reg::new(0x3807, 0x97),  // S2
    Reg::new(0x3811, 0x04),  // S2 x offset 4
    Reg::new(0x3813, 0x02),  // S2 y offset 2
    Reg::new(0x3630, 0x2e),  // S2
    Reg::new(0x3632, 0xe2),  // S2
    Reg::new(0x3633, 0x23),  // S2
    Reg::new(0x3634, 0x44),  // S2
    Reg::new(0x3636, 0x06),  // S2
    Reg::new(0x3620, 0x64),  // S2
    Reg::new(0x3621, 0xe0),  // S2
    Reg::new(0x3600, 0x37),  // S2
    Reg::new(0x3704, 0xa0),  // S2
    Reg::new(0x3703, 0x5a),  // S2
    Reg::new(0x3715, 0x78),  // S2
    Reg::new(0x3717, 0x01),  // S2
    Reg::new(0x3731, 0x02),  // S2
    Reg::new(0x370b, 0x60),  // S2
    Reg::new(0x3705, 0x1a),  // S2
    Reg::new(0x3f05, 0x02),  // S2
    Reg::new(0x3f06, 0x10),  // S2
    Reg::new(0x3f01, 0x0a),  // S2
    Reg::new(0x3a08, 0x01),  // S2
    Reg::new(0x3a09, 0x28),  // S2
    Reg::new(0x3a0a, 0x00),  // S2
    Reg::new(0x3a0b, 0xf6),  // S2
    Reg::new(0x3a0d, 0x08),  // S2
    Reg::new(0x3a0e, 0x06),  // S2
    Reg::new(0x3a0f, 0x58),  // S2
    Reg::new(0x3a10, 0x50),  // S2
    Reg::new(0x3a1b, 0x58),  // S2
    Reg::new(0x3a1e, 0x50),  // S2
    Reg::new(0x3a11, 0x60),  // S2
    Reg::new(0x3a1f, 0x28),  // S2
    Reg::new(0x4001, 0x02),  // S2
    Reg::new(0x4004, 0x04),  // S2
    Reg::new(0x4000, 0x09),  // S2
    Reg::new(0x4837, 0x16),  // S2 MIPI PCLK_PERIOD = 22
    Reg::new(0x4800, 0x24),  // S2 non-continuous clock, LP-11 idle, no line sync
    Reg::new(0x3212, 0xa0), // S2 value; 0x3212 is not in the S8 register tables, so its meaning is unverified
    Reg::new(REG_END, 0x00), // S14 end of table
];

// ---------------------------------------------------------------------------
// Mode: 800x640 @ 50 fps, RAW8, 2 lanes  (Espressif) - 2x subsampled
// ---------------------------------------------------------------------------

/// `ov5647_mipi_2lane_24Minput_800x640_raw8_50fps[]`: self-contained RAW8 sequence.
/// Upstream builds several values from macros; the values below are the evaluated
/// literals (original expression kept in the comment).
///
/// * resolution: 800 x 640, RAW8, 2 lanes `// S3`
/// * hts = 1896 (0x0768), vts = 984 (0x03D8) `// S3`
/// * pclk claimed: 100 000 000 Hz; MIPI line rate 400 000 000 Hz = pclk * 4
///   (= bits-per-pixel / lanes for RAW8 on 2 lanes) `// S12`
/// * PLL: 0x3035 = 0x41 (system_clk_div = 4), 0x3036 = 0x80 (128) computed by the
///   source as `(100_000_000 * 8 * 4) / 25_000_000` - an explicit 25 MHz XCLK
/// * window: x start 500, y start 0, x end 2623, y end 1953 `// S3`
/// * does not write 0x4800 or 0x0100; the driver does that `// S3`, `// S10`
pub const OV5647_MODE_800X640_RAW8_50FPS: &[Reg] = &[
    Reg::new(0x3034, 0x18),  // S3 8-bit mode (mipi_bit_mode = 0x8)
    Reg::new(0x3035, 0x41),  // S3 system_clk_div = 4
    Reg::new(0x3036, 0x80),  // S3 expr:(100_000_000*8*4)/25_000_000 = 128
    Reg::new(0x303c, 0x11),  // S3
    Reg::new(0x3106, 0xf5),  // S3
    Reg::new(0x3821, 0x03),  // S3; bits 1 and 0 = r_mirror_snr, r_hbin (S8)
    Reg::new(0x3820, 0x41),  // S3; bit 0 = r_vbin (S8); bit 6 is not a field S8 names
    Reg::new(0x3827, 0xec),  // S3
    Reg::new(0x370c, 0x0f),  // S3
    Reg::new(0x3612, 0x59),  // S3
    Reg::new(0x3618, 0x00),  // S3
    Reg::new(0x5000, 0xff),  // S3 LENC enable
    Reg::new(0x583e, 0xf0),  // S3 LSC max gain
    Reg::new(0x583f, 0x20),  // S3 LSC min gain
    Reg::new(0x5002, 0x41),  // S3
    Reg::new(0x5003, 0x08),  // S3
    Reg::new(0x5a00, 0x08),  // S3
    Reg::new(0x3000, 0x00),  // S3
    Reg::new(0x3001, 0x00),  // S3
    Reg::new(0x3002, 0x00),  // S3
    Reg::new(0x3016, 0x08),  // S3
    Reg::new(0x3017, 0xe0),  // S3
    Reg::new(0x3018, 0x44),  // S3 2-lane MIPI
    Reg::new(0x301c, 0xf8),  // S3
    Reg::new(0x301d, 0xf0),  // S3
    Reg::new(0x3a18, 0x00),  // S3
    Reg::new(0x3a19, 0xf8),  // S3
    Reg::new(0x3c01, 0x80),  // S3
    Reg::new(0x3c00, 0x40),  // S3
    Reg::new(0x3b07, 0x0c),  // S3
    Reg::new(0x380c, 0x07),  // S3 expr:(1896>>8)&0x1F HTS = 1896
    Reg::new(0x380d, 0x68),  // S3 expr:1896&0xFF
    Reg::new(0x380e, 0x03),  // S3 expr:(984>>8)&0xFF VTS = 984
    Reg::new(0x380f, 0xd8),  // S3 expr:984&0xFF
    Reg::new(0x3814, 0x31),  // S3 2x horizontal subsampling
    Reg::new(0x3815, 0x31),  // S3 2x vertical subsampling
    Reg::new(0x3708, 0x64),  // S3
    Reg::new(0x3709, 0x52),  // S3
    Reg::new(0x3800, 0x01),  // S3 expr:(500>>8) X start = 500
    Reg::new(0x3801, 0xf4),  // S3 expr:500&0xFF
    Reg::new(0x3802, 0x00),  // S3 Y start = 0
    Reg::new(0x3803, 0x00),  // S3
    Reg::new(0x3804, 0x0a),  // S3 expr:((2624-1)>>8)&0x0F X end = 2623
    Reg::new(0x3805, 0x3f),  // S3 expr:(2624-1)&0xFF
    Reg::new(0x3806, 0x07),  // S3 expr:((1954-1)>>8)&0x07 Y end = 1953
    Reg::new(0x3807, 0xa1),  // S3 expr:(1954-1)&0xFF
    Reg::new(0x3808, 0x03),  // S3 expr:(800>>8)&0x0F output width 800
    Reg::new(0x3809, 0x20),  // S3 expr:800&0xFF
    Reg::new(0x380a, 0x02),  // S3 expr:(640>>8)&0x7F output height 640
    Reg::new(0x380b, 0x80),  // S3 expr:640&0xFF
    Reg::new(0x3810, 0x00),  // S3 expr:(8>>8)&0x0F horizontal offset 8
    Reg::new(0x3811, 0x08),  // S3 expr:8&0xFF
    Reg::new(0x3812, 0x00),  // S3 vertical offset 0
    Reg::new(0x3813, 0x00),  // S3
    Reg::new(0x3630, 0x2e),  // S3
    Reg::new(0x3632, 0xe2),  // S3
    Reg::new(0x3633, 0x23),  // S3
    Reg::new(0x3634, 0x44),  // S3
    Reg::new(0x3636, 0x06),  // S3
    Reg::new(0x3620, 0x64),  // S3
    Reg::new(0x3621, 0xe0),  // S3
    Reg::new(0x3600, 0x37),  // S3
    Reg::new(0x3704, 0xa0),  // S3
    Reg::new(0x3703, 0x5a),  // S3
    Reg::new(0x3715, 0x78),  // S3
    Reg::new(0x3717, 0x01),  // S3
    Reg::new(0x3731, 0x02),  // S3
    Reg::new(0x370b, 0x60),  // S3
    Reg::new(0x3705, 0x1a),  // S3
    Reg::new(0x3f05, 0x02),  // S3
    Reg::new(0x3f06, 0x10),  // S3
    Reg::new(0x3f01, 0x0a),  // S3
    Reg::new(0x3a08, 0x01),  // S3
    Reg::new(0x3a09, 0x27),  // S3
    Reg::new(0x3a0a, 0x00),  // S3
    Reg::new(0x3a0b, 0xf6),  // S3
    Reg::new(0x3a0d, 0x04),  // S3
    Reg::new(0x3a0e, 0x03),  // S3
    Reg::new(0x3a0f, 0x58),  // S3
    Reg::new(0x3a10, 0x50),  // S3
    Reg::new(0x3a1b, 0x58),  // S3
    Reg::new(0x3a1e, 0x50),  // S3
    Reg::new(0x3a11, 0x60),  // S3
    Reg::new(0x3a1f, 0x28),  // S3
    Reg::new(0x4001, 0x02),  // S3
    Reg::new(0x4004, 0x02),  // S3
    Reg::new(0x4000, 0x09),  // S3
    Reg::new(0x4837, 0x28),  // S3 expr:1000000000/(100_000_000/4) = 40
    Reg::new(0x4050, 0x6e),  // S3
    Reg::new(0x4051, 0x8f),  // S3
    Reg::new(REG_END, 0x00), // S14 end of table
];

// ---------------------------------------------------------------------------
// Mode: 800x800 @ 50 fps, RAW8, 2 lanes  (Espressif) - 2x subsampled
// ---------------------------------------------------------------------------

/// `ov5647_mipi_2lane_24Minput_800x800_raw8_50fps[]`: same geometry/window as the
/// 800x640 RAW8 mode, only the output height (0x380A/0x380B = 800) and 0x4837 differ.
///
/// * resolution: 800 x 800, RAW8, 2 lanes `// S4`
/// * hts = 1896 (0x0768), vts = 984 (0x03D8) `// S4`
/// * pclk claimed: 100 000 000 Hz; MIPI line rate 400 000 000 Hz = pclk * 4 `// S12`
/// * PLL: 0x3035 = 0x41, 0x3036 = 0x80 (128, `expr:(100_000_000*8*4)/25_000_000`) `// S4`
pub const OV5647_MODE_800X800_RAW8_50FPS: &[Reg] = &[
    Reg::new(0x3034, 0x18),  // S4 8-bit mode
    Reg::new(0x3035, 0x41),  // S4 system_clk_div = 4
    Reg::new(0x3036, 0x80),  // S4 expr:(100_000_000*8*4)/25_000_000 = 128
    Reg::new(0x303c, 0x11),  // S4
    Reg::new(0x3106, 0xf5),  // S4
    Reg::new(0x3821, 0x03),  // S4; bits 1 and 0 = r_mirror_snr, r_hbin (S8)
    Reg::new(0x3820, 0x41),  // S4; bit 0 = r_vbin (S8); bit 6 is not a field S8 names
    Reg::new(0x3827, 0xec),  // S4
    Reg::new(0x370c, 0x0f),  // S4
    Reg::new(0x3612, 0x59),  // S4
    Reg::new(0x3618, 0x00),  // S4
    Reg::new(0x5000, 0xff),  // S4 LENC enable
    Reg::new(0x583e, 0xf0),  // S4 LSC max gain
    Reg::new(0x583f, 0x20),  // S4 LSC min gain
    Reg::new(0x5002, 0x41),  // S4
    Reg::new(0x5003, 0x08),  // S4
    Reg::new(0x5a00, 0x08),  // S4
    Reg::new(0x3000, 0x00),  // S4
    Reg::new(0x3001, 0x00),  // S4
    Reg::new(0x3002, 0x00),  // S4
    Reg::new(0x3016, 0x08),  // S4
    Reg::new(0x3017, 0xe0),  // S4
    Reg::new(0x3018, 0x44),  // S4 2-lane MIPI
    Reg::new(0x301c, 0xf8),  // S4
    Reg::new(0x301d, 0xf0),  // S4
    Reg::new(0x3a18, 0x00),  // S4
    Reg::new(0x3a19, 0xf8),  // S4
    Reg::new(0x3c01, 0x80),  // S4
    Reg::new(0x3c00, 0x40),  // S4
    Reg::new(0x3b07, 0x0c),  // S4
    Reg::new(0x380c, 0x07),  // S4 expr:(1896>>8)&0x1F HTS = 1896
    Reg::new(0x380d, 0x68),  // S4 expr:1896&0xFF
    Reg::new(0x380e, 0x03),  // S4 expr:(984>>8)&0xFF VTS = 984
    Reg::new(0x380f, 0xd8),  // S4 expr:984&0xFF
    Reg::new(0x3814, 0x31),  // S4 2x horizontal subsampling
    Reg::new(0x3815, 0x31),  // S4 2x vertical subsampling
    Reg::new(0x3708, 0x64),  // S4
    Reg::new(0x3709, 0x52),  // S4
    Reg::new(0x3800, 0x01),  // S4 X start = 500
    Reg::new(0x3801, 0xf4),  // S4
    Reg::new(0x3802, 0x00),  // S4 Y start = 0
    Reg::new(0x3803, 0x00),  // S4
    Reg::new(0x3804, 0x0a),  // S4 X end = 2623
    Reg::new(0x3805, 0x3f),  // S4
    Reg::new(0x3806, 0x07),  // S4 Y end = 1953
    Reg::new(0x3807, 0xa1),  // S4
    Reg::new(0x3808, 0x03),  // S4 output width 800
    Reg::new(0x3809, 0x20),  // S4
    Reg::new(0x380a, 0x03),  // S4 output height 800
    Reg::new(0x380b, 0x20),  // S4
    Reg::new(0x3810, 0x00),  // S4 horizontal offset 8 (high byte)
    Reg::new(0x3811, 0x08),  // S4
    Reg::new(0x3812, 0x00),  // S4 vertical offset 0
    Reg::new(0x3813, 0x00),  // S4
    Reg::new(0x3630, 0x2e),  // S4
    Reg::new(0x3632, 0xe2),  // S4
    Reg::new(0x3633, 0x23),  // S4
    Reg::new(0x3634, 0x44),  // S4
    Reg::new(0x3636, 0x06),  // S4
    Reg::new(0x3620, 0x64),  // S4
    Reg::new(0x3621, 0xe0),  // S4
    Reg::new(0x3600, 0x37),  // S4
    Reg::new(0x3704, 0xa0),  // S4
    Reg::new(0x3703, 0x5a),  // S4
    Reg::new(0x3715, 0x78),  // S4
    Reg::new(0x3717, 0x01),  // S4
    Reg::new(0x3731, 0x02),  // S4
    Reg::new(0x370b, 0x60),  // S4
    Reg::new(0x3705, 0x1a),  // S4
    Reg::new(0x3f05, 0x02),  // S4
    Reg::new(0x3f06, 0x10),  // S4
    Reg::new(0x3f01, 0x0a),  // S4
    Reg::new(0x3a08, 0x01),  // S4
    Reg::new(0x3a09, 0x27),  // S4
    Reg::new(0x3a0a, 0x00),  // S4
    Reg::new(0x3a0b, 0xf6),  // S4
    Reg::new(0x3a0d, 0x04),  // S4
    Reg::new(0x3a0e, 0x03),  // S4
    Reg::new(0x3a0f, 0x58),  // S4
    Reg::new(0x3a10, 0x50),  // S4
    Reg::new(0x3a1b, 0x58),  // S4
    Reg::new(0x3a1e, 0x50),  // S4
    Reg::new(0x3a11, 0x60),  // S4
    Reg::new(0x3a1f, 0x28),  // S4
    Reg::new(0x4001, 0x02),  // S4
    Reg::new(0x4004, 0x02),  // S4
    Reg::new(0x4000, 0x09),  // S4
    Reg::new(0x4837, 0x14),  // S4 expr:(1000000000/100_000_000)*2 = 20
    Reg::new(0x4050, 0x6e),  // S4
    Reg::new(0x4051, 0x8f),  // S4
    Reg::new(REG_END, 0x00), // S14 end of table
];

// ---------------------------------------------------------------------------
// Mode: 800x1280 @ 50 fps, RAW8, 2 lanes  (Espressif) - its own window
// ---------------------------------------------------------------------------

/// `ov5647_mipi_2lane_24Minput_800x1280_raw8_50fps[]`: RAW8 with a taller readout.
/// Upstream expresses the window in HSTA/VSTA/HWIN/VWIN macros; evaluated here.
///
/// * resolution: 800 x 1280, RAW8, 2 lanes `// S5`
/// * window: HSTA = 512, VSTA = 248, HWIN = 2109, VWIN = 1447 -> x end 2621, y end 1695
/// * hts = HWIN + 285 = 2394 (0x095A), vts = VWIN + 285 = 1732 (0x06C4) `// S5`
/// * pclk claimed: 100 000 000 Hz; MIPI line rate 400 000 000 Hz = pclk * 4 `// S12`
/// * PLL: 0x3035 = 0x41, 0x3036 = 0x80 (128, `expr:(100_000_000*8*4)/25_000_000`) `// S5`
///
/// # Measurement
///
/// Two entries deviate from the upstream table; both are recorded here with the
/// numbers that justify them. Everything else is the upstream table unchanged
/// (`S5`, evaluated macros). Measured on ESP32-P4 rev 1.3 with the OV5647 on the
/// CSI connector, one session, same lighting, host-side statistics over the
/// transferred payload.
///
/// * **`0x3a18`/`0x3a19` = 0x00F8, not the upstream 0x0022.** `S8` table 6-1:
///   `0x3A18`/`0x3A19` = AEC gain ceiling `{gain_ceiling[9:8], gain_ceiling[7:0]}`
///   in 1/16 steps (reset `0x007C` = 7.75x); the working RAW8 tables S3/S4 use
///   `0x00F8` = 15.5x. Read-back after the table was applied: `0x350A`/`0x350B`
///   (AGC real gain) held exactly `0x0022` - the AGC was pinned at the ceiling -
///   and the frame was flat: mean 6.31, SD 1.11, max 0x0F, 14 distinct values,
///   block-mean SD 0.88 - no scene structure. With the ceiling at `0x00F8` the same
///   scene transfers as mean 10.6, SD 3.4, max 0x1D, block-mean SD 2.4, no clipping,
///   the AGC at its new ceiling and the AEC still exposure-limited by the frame (the
///   room is dark); combined with the crop entry below, mean 30.7, SD 10.4,
///   max 0x66-0x6F, block-mean SD 8.7.
/// * **`0x3810`/`0x3811` = 655 (added).** They are the ISP output window's
///   horizontal offset, so with an 800-column output inside a 2110-column window
///   they select the read-out 800. Upstream writes neither, leaving the reset
///   default 4 = the window's left edge. Applied by the table (written once, before
///   streaming) this is what takes the mode from the ceiling fix's mean 10.6 /
///   SD 3.4 / block-mean SD 2.4 to verified captures at mean 30.70 and 30.78
///   (SD 10.4, max 0x66-0x6F, block-mean SD 8.71) in the session it was fixed in.
///   Written *while streaming*
///   the same register moves the mean only 10 -> 15, so the sweep below understates
///   it: offset 4 -> mean 10 / block-mean SD 3.2; 300 -> 11; 655 (centred) -> 15 /
///   5.7; 1310 -> 22 / 18.5. The window itself is unchanged and stays centred on the
///   2624-column array; only the 800 columns taken out of it move.
///
///   A later session measured the mode against `OV5647_MODE_800X640_RAW8_50FPS`
///   directly, four interleaved pairs captured seconds apart in both orders, with
///   the host CRC agreeing on all eight: 800x640 read 22.62 / 22.39 / 22.30 / 22.06
///   mean and 800x1280 read 15.33 / 15.36 / 15.37 / 15.28 — 1.44-1.48x dimmer at
///   every pair. Both tables ran with the AGC at the gain ceiling (`0x350A`/`0x350B`
///   read back `0x00F8` in both modes), with exposure read back as `0x003D80` (984
///   lines) here against `0x003D40` (980 lines) for 800x640. Those sit in different
///   frames: this table's VTS is 1732, so 984 lines is 57% of the frame period,
///   while 800x640's VTS is 984, where 980 lines is the frame limit. The two windows
///   also differ (S3 2x2 subsampled at 500..2623 x 0..1953, S5 uncoupled 1:1 at
///   512..2621 x 248..1695), so the factor is not separable into table and scene,
///   and the absolute levels of the two sessions do not match each other either.
///
/// Excluded on the hardware, with numbers, so nobody repeats them:
///
/// * A window whose *read-out* width equals the 800-pixel output (an 800-wide
///   window at 1:1, or a 1600-wide window at HINC 0x31) does **not** recover the
///   lost light: the sensor then emits a stream in which 7.6%-15.2% of the bytes
///   are 0xFF, independent of exposure and gain (measured at gain 1.19x and 15.5x,
///   identical 0xFF counts), which inflates mean/SD without adding signal.
///   The upstream window/output ratio is therefore kept.
/// * The vertical window (1448 read-out rows for a 1280-row output) and the frame
///   timing are not the limit: raising VTS 1732 -> 3464 (2x the frame time) left
///   the payload at mean 15-16, and a 1280-row matching window stalls the bridge
///   (`wait_frame` timeout, `phy_stopstate` set).
/// * The AEC max-exposure entries (`0x3a14`/`0x3a15` = 0x0040, MAX EXPO 50 = 64
///   lines) are inert on this part: forcing 50 Hz banding (0x3C00/0x3C01) with the
///   entry as upstream writes it, and with the datasheet default `0x0E40`, both
///   give mean 10-11. Left as upstream.
/// * The S5-only `0x5025` = 0x01 (AEC averages the AWB-gain output) and the LENC
///   enable `0x5000` = 0xff change nothing measurable: 0x5025 = 0x00 held for 60
///   frames gives mean 51 / 0xFF count 155,588 against mean 51 / 155,584 for 0x01;
///   LENC off gives mean 48 / 155,593 against 51 / 155,590 for LENC on, and
///   programming the LSC gain limits (`0x583e`/`0x583f`, which S3/S4 set) changes
///   nothing either. Left as upstream.
pub const OV5647_MODE_800X1280_RAW8_50FPS: &[Reg] = &[
    Reg::new(0x3034, 0x18),  // S5 8-bit mode
    Reg::new(0x3035, 0x41),  // S5 system_clk_div = 4
    Reg::new(0x3036, 0x80),  // S5 expr:(100_000_000*8*4)/25_000_000 = 128
    Reg::new(0x303c, 0x11),  // S5
    Reg::new(0x3106, 0xf5),  // S5
    Reg::new(0x3820, 0x40),  // S5; bit 6 only, not a field S8 names (bits[2:0] flip/bin are 0)
    Reg::new(0x3821, 0x03),  // S5; bits 1 and 0 = r_mirror_snr, r_hbin (S8)
    Reg::new(0x3822, 0x11),  // S5
    Reg::new(0x3827, 0xec),  // S5
    Reg::new(0x370c, 0x0f),  // S5
    Reg::new(0x3612, 0x59),  // S5
    Reg::new(0x3618, 0x00),  // S5
    Reg::new(0x5025, 0x01),  // S5
    Reg::new(0x5003, 0x08),  // S5
    Reg::new(0x5a00, 0x08),  // S5
    Reg::new(0x3000, 0x00),  // S5
    Reg::new(0x3001, 0x00),  // S5
    Reg::new(0x3002, 0x00),  // S5
    Reg::new(0x3016, 0x08),  // S5
    Reg::new(0x3017, 0xe0),  // S5
    Reg::new(0x3018, 0x44),  // S5 2-lane MIPI
    Reg::new(0x301c, 0xf8),  // S5
    Reg::new(0x301d, 0xf0),  // S5
    Reg::new(0x3a18, 0x00),  // S5
    Reg::new(0x3a19, 0xf8),  // S5
    Reg::new(0x3c00, 0x40),  // S5
    Reg::new(0x3b07, 0x08),  // S5
    Reg::new(0x3800, 0x02),  // S5 expr:HSTA>>8 = 512
    Reg::new(0x3801, 0x00),  // S5 expr:HSTA&0xFF
    Reg::new(0x3802, 0x00),  // S5 expr:VSTA>>8 = 248
    Reg::new(0x3803, 0xf8),  // S5 expr:VSTA&0xFF
    Reg::new(0x3804, 0x0a),  // S5 expr:HEND>>8 = 2621
    Reg::new(0x3805, 0x3d),  // S5 expr:HEND&0xFF
    Reg::new(0x3806, 0x06),  // S5 expr:VEND>>8 = 1695
    Reg::new(0x3807, 0x9f),  // S5 expr:VEND&0xFF
    Reg::new(0x3808, 0x03),  // S5 expr:(800>>8)&0x0F output width 800
    Reg::new(0x3809, 0x20),  // S5 expr:800&0xFF
    Reg::new(0x380a, 0x05),  // S5 expr:(1280>>8)&0x7F output height 1280
    Reg::new(0x380b, 0x00),  // S5 expr:1280&0xFF
    Reg::new(0x380c, 0x09),  // S5 expr:TOTAL_H_SIZE>>8 = 2394
    Reg::new(0x380d, 0x5a),  // S5 expr:TOTAL_H_SIZE&0xFF
    Reg::new(0x380e, 0x06),  // S5 expr:TVS>>8 = 1732
    Reg::new(0x380f, 0xc4),  // S5 expr:TVS&0xFF
    Reg::new(0x3810, 0x02),  // added: ISP window offset 655 (see `# Measurement`)
    Reg::new(0x3811, 0x8f),  // added: expr:655 & 0xFF
    Reg::new(0x350c, 0x00),  // S5
    Reg::new(0x350d, 0xff),  // S5
    Reg::new(0x3814, 0x11),  // S5 no horizontal subsampling
    Reg::new(0x3815, 0x11),  // S5 no vertical subsampling
    Reg::new(0x3a1b, 0xa0),  // S5
    Reg::new(0x3a0f, 0x8e),  // S5
    Reg::new(0x3a10, 0x60),  // S5
    Reg::new(0x3a1e, 0x50),  // S5
    Reg::new(0x3a11, 0xf9),  // S5
    Reg::new(0x3a1f, 0xc0),  // S5
    Reg::new(0x3a14, 0x00),  // S5 gain ceiling
    Reg::new(0x3a15, 0x40),  // S5
    Reg::new(0x3a18, 0x00),  // corrected: AEC gain ceiling (see `# Measurement`)
    Reg::new(0x3a19, 0xf8),  // corrected: not upstream's 0x0022
    Reg::new(0x5000, 0xff),  // S5 LENC enable
    Reg::new(0x4001, 0x02),  // S5
    Reg::new(0x4004, 0x04),  // S5
    Reg::new(0x4007, 0x00),  // S5 expr:(0x0<<3)|0x0
    Reg::new(0x3630, 0x2e),  // S5
    Reg::new(0x3632, 0xe2),  // S5
    Reg::new(0x3633, 0x23),  // S5
    Reg::new(0x3634, 0x44),  // S5
    Reg::new(0x3636, 0x06),  // S5
    Reg::new(0x3620, 0x64),  // S5
    Reg::new(0x3621, 0xe0),  // S5
    Reg::new(0x3600, 0x37),  // S5
    Reg::new(0x3704, 0xa0),  // S5
    Reg::new(0x3703, 0x5a),  // S5
    Reg::new(0x3715, 0x78),  // S5
    Reg::new(0x3717, 0x01),  // S5
    Reg::new(0x3731, 0x02),  // S5
    Reg::new(0x370b, 0x60),  // S5
    Reg::new(0x3705, 0x1a),  // S5
    Reg::new(0x3f05, 0x02),  // S5
    Reg::new(0x3f06, 0x10),  // S5
    Reg::new(0x3f01, 0x0a),  // S5
    Reg::new(0x3a08, 0x01),  // S5
    Reg::new(0x3a09, 0x27),  // S5
    Reg::new(0x3a0a, 0x00),  // S5
    Reg::new(0x3a0b, 0xf6),  // S5
    Reg::new(0x3a0d, 0x04),  // S5
    Reg::new(0x3a0e, 0x03),  // S5
    Reg::new(0x3a0f, 0x58),  // S5
    Reg::new(0x3a10, 0x50),  // S5
    Reg::new(0x3a1b, 0x58),  // S5
    Reg::new(0x3a1e, 0x50),  // S5
    Reg::new(0x3a11, 0x60),  // S5
    Reg::new(0x3a1f, 0x28),  // S5
    Reg::new(0x4000, 0x09),  // S5
    Reg::new(0x4001, 0x02),  // S5
    Reg::new(0x4004, 0x04),  // S5
    Reg::new(0x4837, 0x28),  // S5 expr:1000000000/(100_000_000/4) = 40
    Reg::new(0x4050, 0x6e),  // S5
    Reg::new(0x4051, 0x8f),  // S5
    Reg::new(0x5903, 0xff),  // S5
    Reg::new(REG_END, 0x00), // S14 end of table
];

// ---------------------------------------------------------------------------
// Streaming control
// ---------------------------------------------------------------------------

/// Stream on: MIPI CTRL00 -> non-continuous clock + LP-11 idle (keep 0x4800 as the
/// mode table left it), then 0x0100 = 0x01. `// S10` (`ov5647_set_stream`)
pub const OV5647_STREAM_ON: &[Reg] = &[
    Reg::new(0x4800, MIPI_CTRL00_BUS_IDLE | MIPI_CTRL00_CLOCK_LANE_GATE), // S10 = 0x24
    Reg::new(0x0100, 0x01),                                               // S10 stream on
    Reg::new(REG_END, 0x00),                                              // S14 end of table
];

/// Stream off: gate the clock lane *and* force it into low power (LP-11), then
/// 0x0100 = 0x00. `// S10` (`ov5647_set_stream`)
pub const OV5647_STREAM_OFF: &[Reg] = &[
    Reg::new(
        0x4800,
        MIPI_CTRL00_BUS_IDLE | MIPI_CTRL00_CLOCK_LANE_GATE | MIPI_CTRL00_CLOCK_LANE_DISABLE,
    ), // S10 = 0x25
    Reg::new(0x0100, 0x00),  // S10 stream off
    Reg::new(REG_END, 0x00), // S14 end of table
];

/// Every table above, in declaration order.
///
/// Exists so a test can assert the invariants of *every* exposed table (non-empty,
/// terminated by [`REG_END`] and containing `0xFFFF` nowhere else) without keeping a
/// second, hand-maintained list that could drift from this file.
pub const ALL_TABLES: &[(&str, &[Reg])] = &[
    ("OV5647_RESET", OV5647_RESET),
    ("OV5647_MODE_2592X1944_RAW10", OV5647_MODE_2592X1944_RAW10),
    (
        "OV5647_MODE_1920X1080_RAW10_30FPS",
        OV5647_MODE_1920X1080_RAW10_30FPS,
    ),
    (
        "OV5647_MODE_1280X960_RAW10_45FPS",
        OV5647_MODE_1280X960_RAW10_45FPS,
    ),
    (
        "OV5647_MODE_800X640_RAW8_50FPS",
        OV5647_MODE_800X640_RAW8_50FPS,
    ),
    (
        "OV5647_MODE_800X800_RAW8_50FPS",
        OV5647_MODE_800X800_RAW8_50FPS,
    ),
    (
        "OV5647_MODE_800X1280_RAW8_50FPS",
        OV5647_MODE_800X1280_RAW8_50FPS,
    ),
    ("OV5647_STREAM_ON", OV5647_STREAM_ON),
    ("OV5647_STREAM_OFF", OV5647_STREAM_OFF),
];
