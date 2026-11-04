//! Pure ISP geometry arithmetic.
//!
//! In bypass mode the ISP's input module still runs even though no processing happens, and its input
//! H-size is expressed in **32-bit words** of the incoming pixel stream rather than in pixels. IDF:
//! `ISP_DIV_ROUND_UP(h_res * in_bits_per_pixel, 32)` in
//! `components/esp_driver_isp/src/isp_core.c`, minus the 1 that
//! `components/esp_hal_cam/esp32p4/include/hal/isp_ll.h` applies
//! (`frame_cfg.hadr_num = pixel_num - 1`).
//!
//! That formula was written here with `8` hardcoded, which is invisible while RAW8 is the only mode
//! and wrong for every other one: a 1920-wide RAW10 mode computes `1920*8/32 - 1 = 479` with the
//! literal, where the formula gives `(1920*10 + 31)/32 - 1 = 599`. The 479 is this board's own
//! bring-up reading — a measurement, not a value any published source states. Kept host-tested so
//! the divisor cannot go back to being a literal.

/// `ISP.FRAME_CFG.HADR_NUM` for the bypass path: the input width in 32-bit words, minus one.
///
/// `in_bits` is the *sensor's* bits per pixel (`8` for RAW8, `10` for RAW10), not the ISP output's.
///
/// # Examples
///
/// ```
/// use ov5647_core::isp_geometry::hadr_num_bypass;
///
/// // RAW8 800-wide: (800*8 + 31)/32 - 1 = 199.
/// assert_eq!(hadr_num_bypass(800, 8), 199);
/// // RAW10 1920-wide: (1920*10 + 31)/32 - 1 = 599 — not 479, which a
/// // hardcoded divisor of 8 would give.
/// assert_eq!(hadr_num_bypass(1920, 10), 599);
/// ```
pub const fn hadr_num_bypass(width: u32, in_bits: u32) -> u16 {
    let words = (width as u64 * in_bits as u64 + 31) / 32;
    (words.saturating_sub(1) & 0xFFFF) as u16
}

/// `ISP.FRAME_CFG.HADR_NUM` for the converting path: the width in pixels, minus one.
pub const fn hadr_num_converting(width: u32) -> u16 {
    (width.saturating_sub(1) & 0xFFFF) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw8_800_matches_the_measured_reference() {
        // The working reference's RAW8 bypass reads HADR_NUM = 199 for an 800-wide frame.
        assert_eq!(hadr_num_bypass(800, 8), 199);
    }

    #[test]
    fn raw10_1920_is_599_not_479() {
        // The bug this module exists for: with 8 hardcoded the value is 479.
        assert_eq!(hadr_num_bypass(1920, 10), 599);
        assert_ne!(hadr_num_bypass(1920, 10), hadr_num_bypass(1920, 8));
        assert_eq!(hadr_num_bypass(1920, 8), 479);
    }

    #[test]
    fn the_divisor_is_the_sensor_bits_not_a_literal() {
        // Same width, different wire format: the two must differ.
        for w in [640u32, 800, 1280, 1920, 2592] {
            assert_ne!(hadr_num_bypass(w, 8), hadr_num_bypass(w, 10));
        }
    }

    #[test]
    fn converting_path_is_pixels_minus_one() {
        assert_eq!(hadr_num_converting(1920), 1919);
        assert_eq!(hadr_num_converting(1), 0);
    }

    #[test]
    fn width_zero_does_not_underflow() {
        assert_eq!(hadr_num_bypass(0, 10), 0);
        assert_eq!(hadr_num_converting(0), 0);
    }
}
