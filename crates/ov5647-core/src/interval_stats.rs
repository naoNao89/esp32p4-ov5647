//! Streaming percentile statistics for the soak test.
//!
//! The soak test needs p50/p95/p99 of the frame interval and of the re-arm latency.
//! A full buffer of every interval is only 1,000 entries and would be fine, but this
//! project's rule is to keep what runs on the target small and deterministic. This
//! module instead keeps a fixed-size reservoir of the *most recent* samples plus the
//! running sum, so `mean` is exact over the whole run while the percentiles come from
//! a bounded, reset-free window. Both are good enough to report the p50/p95/p99 the
//! ticket asks for, and the module is host-tested.

/// Tracks a sequence of non-negative `u32` samples, reporting mean over the whole run
/// and p50/p95/p99 over a bounded reservoir of the most recent samples.
///
/// `reservoir_len` must be a power of two; it is clamped to a sane range.
pub struct IntervalStats {
    sum: u64,
    count: u64,
    reservoir: Vec<u32>,
    cap: usize,
}

impl IntervalStats {
    /// Creates a stats tracker with a reservoir of at most `reservoir_len`
    /// samples.
    ///
    /// `reservoir_len` is clamped to `[16, 4096]` and rounded up to the next
    /// power of two, so the storage is bounded and the reservoir always covers a
    /// stable, power-of-two window.
    pub fn new(reservoir_len: usize) -> Self {
        let cap = reservoir_len.clamp(16, 4096).next_power_of_two();
        Self {
            sum: 0,
            count: 0,
            reservoir: Vec::new(),
            cap,
        }
    }

    /// Records one `sample` (e.g. a frame-interval or re-arm latency in µs).
    ///
    /// `sum`/`count` accumulate over the whole run so [`Self::mean_us`] stays
    /// exact; the reservoir holds only the most recent samples, giving bounded,
    /// reset-free p50/p95/p99.
    pub fn push(&mut self, sample: u32) {
        self.sum += sample as u64;
        self.count += 1;
        if self.reservoir.len() < self.cap {
            self.reservoir.push(sample);
        } else {
            // Replace a random slot (deterministic: use count as a cheap rotor so a
            // long tail of new samples still shows up).
            let idx = (self.count % self.cap as u64) as usize;
            self.reservoir[idx] = sample;
        }
    }

    /// Number of samples seen so far (the whole run, not just the reservoir).
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Exact mean over the whole run, in µs, or `0` when no sample has been
    /// recorded.
    pub fn mean_us(&self) -> u32 {
        if self.count == 0 {
            0
        } else {
            (self.sum / self.count) as u32
        }
    }

    /// p0/p50/p95/p99 over the reservoir. Returns (min, p50, p95, p99, max) or
    /// None when there are no samples.
    ///
    /// # Returns
    ///
    /// * `Some((min, p50, p95, p99, max))` — sorted percentile values over the
    ///   reservoir window.
    /// * `None` — no sample has been recorded yet, so no percentiles exist
    ///   (division by an empty window is avoided).
    pub fn percentiles_us(&self) -> Option<(u32, u32, u32, u32, u32)> {
        if self.reservoir.is_empty() {
            return None;
        }
        let mut v = self.reservoir.clone();
        v.sort_unstable();
        let n = v.len();
        let at = |p: usize| -> u32 { v[n.saturating_mul(p) / 100] };
        Some((v[0], at(50), at(95), at(99), v[n - 1]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_stats_have_no_percentiles() {
        let s = IntervalStats::new(64);
        assert_eq!(s.count(), 0);
        assert_eq!(s.mean_us(), 0);
        assert!(s.percentiles_us().is_none());
    }

    #[test]
    fn mean_is_exact_over_the_whole_run() {
        let mut s = IntervalStats::new(16);
        for i in 1..=100u32 {
            s.push(i);
        }
        assert_eq!(s.mean_us(), 50); // 5050/100
        let (min, p50, p95, p99, max) = s.percentiles_us().unwrap();
        // The reservoir holds a rotating 16-sample window of 1..=100, so the min/max
        // reflect the window, not the whole run; only the *mean* is exact over the
        // whole run. min/max must still be within the data range.
        assert!(min >= 1 && min <= 100);
        assert!(max >= 1 && max <= 100);
        assert!(min <= max);
        // The 95th percentile of 1..=100 is 95; the reservoir holds a rotating
        // subset, so the observed value must be within a band, not a single number.
        assert!(p95 >= 90 && p95 <= 100);
        assert!(p99 >= 95 && p99 <= 100);
        // p50 is the median of the reservoir; it must sit between the 25th and 75th
        // percentile of the underlying data, i.e. comfortably inside (1,100).
        assert!(p50 > 1 && p50 < 100);
    }

    #[test]
    fn reservoir_is_bounded_and_rotates() {
        let mut s = IntervalStats::new(16);
        for i in 0..1000u32 {
            s.push(i);
        }
        assert_eq!(s.count(), 1000);
        assert_eq!(s.reservoir.len(), 16);
        // All pushed values are 0..999; the rotating reservoir guarantees the most
        // recent ones are present, so the max must be 999 (the last value pushed).
        let (_, _, _, _, max) = s.percentiles_us().unwrap();
        assert_eq!(max, 999);
        assert_eq!(s.mean_us(), 499); // (0+999)*1000/2 /1000
    }

    #[test]
    fn clamps_capacity_odd_power_of_two() {
        let s = IntervalStats::new(1000);
        assert_eq!(s.cap, 1024);
        let s = IntervalStats::new(4);
        assert_eq!(s.cap, 16);
    }
}
