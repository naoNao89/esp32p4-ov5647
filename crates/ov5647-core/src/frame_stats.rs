//! Pure pixel statistics for exposure/AEC tuning.
//!
//! `mean`, `p95`, `p99` and `max` are what decide whether the sensor's auto-exposure
//! has converged and whether the image is clipping. They are computed from a
//! 256-bin histogram, so the pass over the payload is one read per byte and the
//! percentiles need no sort and no allocation.
//!
//! Host-tested because the numbers drive a tuning decision: getting `p99` wrong
//! would send the exposure search in the wrong direction.

/// Statistics over one frame's payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameStats {
    /// Payload bytes considered.
    pub len: usize,
    /// Arithmetic mean, in thousandths (`(sum * 1000) / len`), so an integer field
    /// still shows `5.053` as `5053`.
    pub mean_milli: u32,
    /// 95th percentile of the byte values.
    pub p95: u8,
    /// 99th percentile.
    pub p99: u8,
    /// Largest byte value seen.
    pub max: u8,
    /// Bytes at or above [`CLIP_HIGH`] — saturated highlights.
    pub clip_high: usize,
    /// Bytes at or below [`CLIP_LOW`] — crushed shadows.
    pub clip_low: usize,
}

/// A byte at or above this is treated as a saturated highlight. RAW8 has no
/// "white point" the driver can know, so this is a tuning threshold, not a
/// property of the format; 250 of 255 is the conventional near-full-scale cut.
pub const CLIP_HIGH: u8 = 250;

/// A byte at or below this is treated as crushed shadow.
pub const CLIP_LOW: u8 = 0;

/// Computes [`FrameStats`] for `payload` in one pass.
///
/// An empty payload yields all-zero statistics rather than a division by zero.
///
/// # Examples
///
/// ```
/// use ov5647_core::frame_stats::frame_stats;
///
/// let s = frame_stats(&[10u8; 1000]);
/// assert_eq!(s.mean_milli, 10_000);
/// assert_eq!(s.p95, 10);
/// assert_eq!(s.max, 10);
/// ```
pub fn frame_stats(payload: &[u8]) -> FrameStats {
    let mut hist = [0u32; 256];
    let mut sum: u64 = 0;
    for &b in payload {
        hist[b as usize] += 1;
        sum += b as u64;
    }
    let len = payload.len();
    if len == 0 {
        return FrameStats {
            len: 0,
            mean_milli: 0,
            p95: 0,
            p99: 0,
            max: 0,
            clip_high: 0,
            clip_low: 0,
        };
    }
    let mean_milli = (sum * 1000 / len as u64) as u32;

    // Percentiles: smallest value whose cumulative count reaches the target.
    // `p` is a hundredth, so the target is `ceil(len * p / 100)`.
    let target = |p: u64| -> u64 { (len as u64 * p).div_ceil(100) };
    let p95_target = target(95);
    let p99_target = target(99);
    let mut cum: u64 = 0;
    let mut p95 = 0u8;
    let mut p99 = 0u8;
    let mut max = 0u8;
    let mut p95_set = false;
    let mut p99_set = false;
    for v in 0..256usize {
        cum += hist[v] as u64;
        if !p95_set && cum >= p95_target {
            p95 = v as u8;
            p95_set = true;
        }
        if !p99_set && cum >= p99_target {
            p99 = v as u8;
            p99_set = true;
        }
        if hist[v] != 0 {
            max = v as u8;
        }
    }

    let mut clip_high = 0usize;
    for v in CLIP_HIGH as usize..256 {
        clip_high += hist[v] as usize;
    }
    let clip_low = hist[CLIP_LOW as usize] as usize;

    FrameStats {
        len,
        mean_milli,
        p95,
        p99,
        max,
        clip_high,
        clip_low,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_payload_is_all_zero() {
        let s = frame_stats(&[]);
        assert_eq!(s.len, 0);
        assert_eq!(s.mean_milli, 0);
        assert_eq!(s.max, 0);
        assert_eq!(s.clip_high, 0);
    }

    #[test]
    fn constant_payload_has_mean_equal_to_the_value() {
        let s = frame_stats(&[10u8; 1000]);
        assert_eq!(s.mean_milli, 10_000);
        assert_eq!(s.p95, 10);
        assert_eq!(s.max, 10);
        assert_eq!(s.clip_high, 0);
    }

    #[test]
    fn mean_keeps_thousandths() {
        // 5.053 average, as the real first frame had.
        let mut v = vec![5u8; 947];
        v.extend(core::iter::repeat(6u8).take(53));
        let s = frame_stats(&v);
        assert_eq!(s.mean_milli, 5053);
    }

    #[test]
    fn percentiles_track_the_upper_tail() {
        // 980 zeros, 20 at 200. The 95th percentile target is 950, reached at v=0,
        // so p95 is 0; the 99th target is 990, not reached by the zero block, so
        // p99 is 200. This is the boundary that matters: 990 zeros would put the
        // 99th percentile *at* the top of the zero block, and p99 = 0 would be
        // correct there, not a bug.
        let mut v = vec![0u8; 980];
        v.extend(core::iter::repeat(200u8).take(20));
        let s = frame_stats(&v);
        assert_eq!(s.p95, 0);
        assert_eq!(s.p99, 200);
        assert_eq!(s.max, 200);
    }

    #[test]
    fn percentile_at_the_exact_boundary() {
        // 95 of 100 at value 7: the 95th percentile is 7, not the value below it.
        let mut v = vec![1u8; 5];
        v.extend(core::iter::repeat(7u8).take(95));
        let s = frame_stats(&v);
        assert_eq!(s.p95, 7);
    }

    #[test]
    fn clipping_is_counted_at_both_ends() {
        let mut v = vec![0u8; 100];
        v.extend(core::iter::repeat(255u8).take(7)); // 255 >= CLIP_HIGH
        v.extend(core::iter::repeat(250u8).take(3)); // boundary, counted
        v.extend(core::iter::repeat(249u8).take(5)); // just below, not counted
        let s = frame_stats(&v);
        assert_eq!(s.clip_high, 10);
        assert_eq!(s.clip_low, 100);
    }

    #[test]
    fn the_mission_frame_is_dark_and_the_tuned_frame_is_not() {
        // Values from the two real captures, expressed as their measured summaries.
        // This is a documentation test: it asserts the *shape* a converged exposure
        // produces, so a future change that makes `mean` or `max` collapse is caught.
        let dark = {
            let mut v = vec![0u8; 510_905];
            v.extend(core::iter::repeat(18u8).take(1_095));
            frame_stats(&v)
        };
        let bright = {
            let mut v = vec![0u8; 9];
            v.extend(core::iter::repeat(120u8).take(504_000));
            v.extend(core::iter::repeat(200u8).take(7_991));
            frame_stats(&v)
        };
        assert!(bright.mean_milli > 4 * dark.mean_milli);
        assert!(bright.max > dark.max);
        assert_eq!(dark.clip_high, 0);
    }
}
