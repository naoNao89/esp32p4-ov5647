//! Per-row completeness statistics for the row-count check.
//!
//! The hardware's `VADR_NUM` is a *configured* geometry and `VADR_NUM_CHECK` is
//! off by default, so "no VADR_NUM_GT/LT was seen" does not prove the payload has
//! 640 rows. The payload itself has to be shown to contain 640 full rows. That is
//! what this module measures: for each of `height` rows of `width` bytes, it counts
//! non-zero bytes and computes a per-row checksum, so the caller can see whether
//! every row is present and distinct rather than a suffix being repeated or zero.
//!
//! This is deliberately conservative: a row of all zeros is legitimate for a dark
//! scene, so `row_nonzero > 0` is *not* a pass condition. The signal of a missing
//! or duplicated row is structure in the row checksums, not their count.

/// Per-row statistics over a `width` x `height` payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RowStats {
    /// Row width in bytes expected of the payload.
    pub width: usize,
    /// Row count (height) expected of the payload.
    pub height: usize,
    /// Count of rows that are entirely zero.
    pub all_zero_rows: usize,
    /// Number of distinct per-row CRC-32 values across all rows.
    pub distinct_row_crc: usize,
    /// Non-zero count of the first and last rows (for a scene-change sanity check).
    pub first_row_nonzero: usize,
    /// Non-zero count of the last row.
    pub last_row_nonzero: usize,
    /// Row-wise mean of the per-row non-zero counts (as a fraction of width, in
    /// thousandths).
    pub mean_row_fill_milli: u32,
    /// Whether the payload has exactly `height` rows of `width` bytes.
    pub length_ok: bool,
}

/// Computes [`RowStats`] for a payload that is `width * height` bytes.
///
/// Caller must have checked the length matches; a mismatched length yields
/// `length_ok == false` and all-zero stats.
///
/// # Return
///
/// If `payload.len() != width * height`, or `width` or `height` is zero, the
/// result has `length_ok == false` and all other fields zero — the caller can
/// treat this as "the frame is not the declared geometry" without this function
/// panicking on out-of-bounds row slicing.
///
/// # Examples
///
/// ```
/// use ov5647_core::row_stats::row_stats;
///
/// let s = row_stats(&[7u8; 640 * 800], 800, 640);
/// assert!(s.length_ok);
/// assert_eq!(s.distinct_row_crc, 1);
/// assert_eq!(s.mean_row_fill_milli, 1000);
/// ```
pub fn row_stats(payload: &[u8], width: usize, height: usize) -> RowStats {
    if payload.len() != width * height || width == 0 || height == 0 {
        return RowStats {
            width,
            height,
            all_zero_rows: 0,
            distinct_row_crc: 0,
            first_row_nonzero: 0,
            last_row_nonzero: 0,
            mean_row_fill_milli: 0,
            length_ok: false,
        };
    }

    let mut crcs = Vec::new();
    let mut all_zero_rows = 0usize;
    let mut row_fill_sum: u64 = 0;
    let mut first_row_nonzero = 0usize;
    let mut last_row_nonzero = 0usize;

    for r in 0..height {
        let row = &payload[r * width..(r + 1) * width];
        let mut nz = 0usize;
        let mut crc = 0xFFFF_FFFFu32;
        for &b in row {
            if b != 0 {
                nz += 1;
            }
            crc ^= b as u32;
            for _ in 0..8 {
                let mask = (crc & 1).wrapping_neg();
                crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
        crc = !crc;
        crcs.push(crc);
        row_fill_sum += nz as u64;
        if nz == 0 {
            all_zero_rows += 1;
        }
        if r == 0 {
            first_row_nonzero = nz;
        }
        if r == height - 1 {
            last_row_nonzero = nz;
        }
    }

    crcs.sort_unstable();
    crcs.dedup();

    RowStats {
        width,
        height,
        all_zero_rows,
        distinct_row_crc: crcs.len(),
        first_row_nonzero,
        last_row_nonzero,
        mean_row_fill_milli: (row_fill_sum * 1000 / height as u64 / width as u64) as u32,
        length_ok: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mismatched_length_is_not_ok() {
        let s = row_stats(&[0u8; 100], 10, 11);
        assert!(!s.length_ok);
    }

    #[test]
    fn all_constant_rows_have_one_distinct_crc() {
        // 640 rows of 800 identical bytes: one distinct per-row CRC.
        let v = vec![7u8; 640 * 800];
        let s = row_stats(&v, 800, 640);
        assert!(s.length_ok);
        assert_eq!(s.distinct_row_crc, 1);
        assert_eq!(s.all_zero_rows, 0);
        assert_eq!(s.mean_row_fill_milli, 1000);
    }

    #[test]
    fn distinct_rows_yield_many_crcs() {
        // Each row gets a unique pattern: the row index low byte, then a byte that
        // differs per row, so the per-row CRC is distinct across all 640 rows.
        let mut v = Vec::with_capacity(640 * 800);
        for r in 0..640usize {
            // A 16-bit row seed, split into two bytes and repeated: rows alias only
            // if they have the same 16-bit seed, and 0..640 has 640 distinct seeds,
            // so all 640 rows have a distinct per-row checksum.
            let lo = ((r & 0xFF) as u8).wrapping_add(1);
            let hi = (((r >> 8) & 0xFF) as u8).wrapping_add(1);
            for c in 0..800usize {
                v.push(if c & 1 == 0 { lo } else { hi });
            }
        }
        let s = row_stats(&v, 800, 640);
        assert!(s.length_ok);
        assert_eq!(s.distinct_row_crc, 640);
        assert_eq!(s.first_row_nonzero, 800);
        assert_eq!(s.last_row_nonzero, 800);
    }

    #[test]
    fn all_zero_scene_is_legitimate_and_reported() {
        let s = row_stats(&[0u8; 640 * 800], 800, 640);
        assert!(s.length_ok);
        assert_eq!(s.all_zero_rows, 640);
        assert_eq!(s.distinct_row_crc, 1);
        assert_eq!(s.mean_row_fill_milli, 0);
    }

    #[test]
    fn half_dark_half_full_is_detected_as_half() {
        // Rows 0..320 fill, rows 320..640 zero: a genuine "top lit, bottom dark"
        // scene must not be confused with a truncated frame.
        let mut v = vec![0u8; 640 * 800];
        for r in 0..320usize {
            for c in 0..800usize {
                v[r * 800 + c] = 200;
            }
        }
        let s = row_stats(&v, 800, 640);
        assert!(s.length_ok);
        assert_eq!(s.all_zero_rows, 320);
        assert!(s.distinct_row_crc >= 2);
        assert_eq!(s.first_row_nonzero, 800);
        assert_eq!(s.last_row_nonzero, 0);
        assert_eq!(s.mean_row_fill_milli, 500);
    }
}
