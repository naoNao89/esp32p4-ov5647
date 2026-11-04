//! Pure layout arithmetic for a PSRAM frame destination with sentinel granules.
//!
//! The bound test this project relies on is "did the engine write past the frame?".
//! It is answered by two sentinels: one at the frame end (first bytes past the
//! frame) and one in the escape granule (one granule past the last granule the
//! frame touches). If the offsets are wrong the test is meaningless — a sentinel
//! inside the frame would be overwritten by legitimate data, and one outside the
//! allocation would not be written at all. Keeping the arithmetic here means the
//! invariants are checked on the host, where a mistake is cheap.
//!
//! Copied bit-for-bit from `PsramFrame::layout` in the firmware; the firmware
//! delegates to this so there is one source of truth.

/// Page/granule size: the APM's address granule, and the unit sentinels are
/// allocated in.
pub const GRANULE: usize = 4096;

/// Bytes of sentinel written at each end. One cache line is the smallest unit a
/// stale line can hide in, so 64 is the smallest useful length.
pub const SENTINEL_LEN: usize = 64;

/// Value written into the frame-end sentinel.
///
/// A change here means the engine wrote past `frame_len` by less than a granule.
/// It is *not* by itself a bound failure: a page-exact bound with an inclusive
/// `REGIONn_HIGH` permits that granule, which is exactly why the sentinel is
/// there — its verdict is what settles the inclusive/exclusive rule.
pub const SENTINEL_FRAME_END: u8 = 0xA5;

/// Value written into the escape sentinel. No reading of the bound permits this
/// address, so a change here is a definite failure of the length bound.
pub const SENTINEL_ESCAPE: u8 = 0xD3;

/// Offsets and sizes of one PSRAM frame destination.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameLayout {
    /// Bytes of frame.
    pub frame_len: usize,
    /// `span + GRANULE`: the whole slice handed to the DMA destination.
    pub capacity: usize,
    /// Offset of the frame-end sentinel; equals `frame_len` and is inside the
    /// allocation by construction.
    pub frame_end_off: usize,
    /// Offset of the escape sentinel: one granule past the last granule the frame
    /// touches.
    pub escape_off: usize,
}

impl FrameLayout {
    /// Computes the layout for a frame of `frame_len` bytes.
    ///
    /// The frame occupies `ceil(frame_len / GRANULE)` granules; the escape sentinel
    /// sits one full granule past the last frame granule, so it is never shared
    /// with frame data, and `capacity` adds another full granule so both sentinels
    /// fit inside the allocation. `capacity` is therefore granule-aligned (and, as
    /// a multiple of 4096, also satisfies the DMA's 64-byte cache-line rule).
    ///
    /// # Examples
    ///
    /// ```
    /// use ov5647_core::frame_layout::{FrameLayout, GRANULE};
    ///
    /// let l = FrameLayout::new(512_000);
    /// assert_eq!(l.frame_end_off, l.frame_len);
    /// assert_eq!(l.escape_off, 126 * GRANULE); // 125 frame granules + 1 escape
    /// assert_eq!(l.capacity, 127 * GRANULE);
    /// ```
    pub const fn new(frame_len: usize) -> Self {
        let frame_pages = (frame_len + GRANULE - 1) / GRANULE;
        let span = (frame_pages + 1) * GRANULE;
        Self {
            frame_len,
            capacity: span + GRANULE,
            frame_end_off: frame_len,
            escape_off: span,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sentinels_sit_outside_the_frame_and_inside_the_allocation() {
        let l = FrameLayout::new(512_000);
        // Frame-end sentinel is exactly at the frame end, not before it.
        assert_eq!(l.frame_end_off, l.frame_len);
        // Both sentinels must have room for SENTINEL_LEN bytes inside capacity.
        assert!(l.frame_end_off + SENTINEL_LEN <= l.capacity);
        assert!(l.escape_off + SENTINEL_LEN <= l.capacity);
        // The escape sentinel is strictly past the frame-end sentinel: otherwise
        // the escape check would read frame bytes.
        assert!(l.escape_off >= l.frame_end_off + SENTINEL_LEN);
    }

    #[test]
    fn escape_is_one_granule_past_the_last_frame_granule() {
        let l = FrameLayout::new(512_000);
        // 512,000 bytes occupies ceil(512000/4096)=125 granules; escape is 126*4096.
        assert_eq!(l.frame_len.div_ceil(GRANULE), 125);
        assert_eq!(l.escape_off, 126 * GRANULE);
        assert_eq!(l.capacity, 127 * GRANULE);
    }

    #[test]
    fn capacity_is_granule_aligned_for_the_dma_64_byte_rule() {
        for len in [4096usize, 512_000, 1, 4095, 4097, 1_000_000] {
            let l = FrameLayout::new(len);
            assert_eq!(l.capacity % GRANULE, 0);
            // Capacity is also a multiple of the 64-byte cache line.
            assert_eq!(l.capacity % 64, 0);
        }
    }

    #[test]
    fn frame_that_is_itself_granule_aligned_still_gets_a_full_escape_granule() {
        // Exactly 3 granules: span must be 4 granules and capacity 5, so the
        // escape granule is never shared with the frame.
        let l = FrameLayout::new(3 * GRANULE);
        assert_eq!(l.escape_off, 4 * GRANULE);
        assert_eq!(l.capacity, 5 * GRANULE);
    }
}
