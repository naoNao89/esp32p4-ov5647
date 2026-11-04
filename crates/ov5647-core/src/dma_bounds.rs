//! Hardware-independent validation of a DMA destination buffer.
//!
//! This is the part of `DmaFrameBuffer::new` that is pure arithmetic: it does
//! not touch registers, but it encodes the rules that, when violated, historically
//! walked `DAR0` past the end of RAM or silently mis-transferred. Keeping it here
//! means the same checks run on the host under `cargo test` and in the firmware,
//! from one source of truth.
//!
//! Every error here corresponds to a `CsiError` variant in the firmware; the
//! firmware maps between them.

/// Cache line size in bytes. Both the DMA's transfer length and the destination
/// base must be multiples of this for the cache-sync to cover the DMA's writes
/// exactly.
pub const CACHE_LINE: usize = 64;

/// Address ranges this chip's DW-GDMA can reach as a destination.
///
/// From `src/csi.rs`:
///   * `0x4800_0000 .. 0x4A00_0000` — external PSRAM
///   * `0x4FF0_0000 .. 0x4FF2_CBD0` — internal L2MEM (low window)
///   * `0x4FF4_0000 .. 0x4FFA_0000` — internal L2MEM (high window; excludes the
///     reserved top band where the stack must not live)
pub const DMA_DEST_REGIONS: [(usize, usize); 3] = [
    (0x4800_0000, 0x4A00_0000),
    (0x4FF0_0000, 0x4FF2_CBD0),
    (0x4FF4_0000, 0x4FFA_0000),
];

/// Width of `CH1_BLOCK_TS` in bits, and therefore the largest block it can describe.
///
/// From `dw_gdma_reg.h`: `DMAC_CH1_BLOCK_TS_V = 0x003FFFFF` — a **22-bit** field. The PAC's writer is
/// the same width, so a larger value would be *silently masked*: the engine would move a shorter
/// block than the caller asked for and report success. That is exactly the failure this function
/// exists to prevent, and the 1080p RAW10 frame (324,000 words) is the first mode here large enough
/// for the question to be worth asking.
pub const BLOCK_TS_BITS: u32 = 22;
/// Largest value the field can hold: `count - 1` for a 4,194,304-word block.
pub const BLOCK_TS_MAX: u32 = (1 << BLOCK_TS_BITS) - 1;

/// Errors from sizing a block transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockTsError {
    /// `words` was zero: a transfer of nothing has no `count - 1`.
    ZeroWords,
    /// `words - 1` does not fit the hardware field.
    TooManyWords {
        /// The count that was requested.
        words: u32,
        /// The largest value the hardware field can hold (`count - 1`).
        field_max: u32,
    },
}

/// Converts a 64-bit-word count into the value to program into `CH1_BLOCK_TS`.
///
/// The field holds `count - 1`, so a whole frame of `n` words is programmed as `n - 1`. A count whose
/// `n - 1` exceeds [`BLOCK_TS_MAX`] is **rejected** rather than masked, because the masked result is a
/// successful-looking transfer of the wrong length.
///
/// # Errors
///
/// * [`BlockTsError::ZeroWords`] if `words` is zero — a transfer of nothing has no `count - 1`.
/// * [`BlockTsError::TooManyWords`] if `words - 1` does not fit the hardware field. The 1080p RAW10
///   frame (324,000 words) is comfortably inside; refusing here keeps a larger future mode from being
///   silently masked to a shorter transfer.
///
/// # Examples
///
/// ```
/// use ov5647_core::dma_bounds::{block_ts_from_words, BLOCK_TS_MAX};
///
/// assert_eq!(block_ts_from_words(324_000), Ok(323_999));
/// assert!(block_ts_from_words(BLOCK_TS_MAX + 1).is_ok());
/// assert!(block_ts_from_words(BLOCK_TS_MAX + 2).is_err());
/// ```
pub fn block_ts_from_words(words: u32) -> Result<u32, BlockTsError> {
    if words == 0 {
        return Err(BlockTsError::ZeroWords);
    }
    let value = words - 1;
    if value > BLOCK_TS_MAX {
        return Err(BlockTsError::TooManyWords {
            words,
            field_max: BLOCK_TS_MAX,
        });
    }
    Ok(value)
}

/// The register range occupied by the DW-GDMA link-list item (`LINK_ITEM`). This
/// is `(base, base + len)` in firmware; the firmware calls
/// [`descriptor_range`] with the item's address and size, and tests here use the
/// L2MEM placement directly.
pub const fn descriptor_range(item_addr: usize, item_len: usize) -> (usize, usize) {
    (item_addr, item_addr.saturating_add(item_len))
}

/// Errors from validating a DMA destination buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaBufferError {
    /// `buffer` is empty or `frame_len` is zero.
    Empty,
    /// `frame_len > buffer.len()`.
    TooSmall {
        /// Bytes the transfer would write.
        needed: usize,
        /// Bytes available at the destination.
        capacity: usize,
    },
    /// `base`, `buffer.len()` or `frame_len` is not a multiple of [`CACHE_LINE`].
    NotAligned {
        /// The misaligned address.
        addr: usize,
        /// The misaligned length.
        len: usize,
    },
    /// `base + frame_len` overflowed `usize`.
    AddressOverflow {
        /// The destination base address.
        addr: usize,
        /// The frame length.
        len: usize,
    },
    /// `[base, base + frame_len)` is not wholly inside a [`DMA_DEST_REGIONS`] range.
    OutOfRange {
        /// Start of the frame range.
        start: usize,
        /// Exclusive end of the frame range.
        end: usize,
    },
    /// `[base, base + frame_len)` overlaps the link-list descriptor.
    OverlapsDescriptor {
        /// Start of the frame range.
        start: usize,
        /// Exclusive end of the frame range.
        end: usize,
    },
}

/// Checks that `[base, base + frame_len)` is a legal DMA destination.
///
/// Order matters: the cheapest and most generic checks run first, and each check
/// is the one that historically produced a *named* failure, so the returned error
/// identifies the exact rule being violated.
///
/// # Arguments
///
/// * `base` — destination base address.
/// * `capacity` — bytes available at `base` (the allocation size).
/// * `frame_len` — bytes the transfer will write.
/// * `desc_start`/`desc_end` — exclusive address range of the link-list descriptor,
///   or `(0, 0)` when no descriptor is being used.
///
/// # Errors
///
/// * [`DmaBufferError::Empty`] if `capacity` or `frame_len` is zero.
/// * [`DmaBufferError::TooSmall`] if the frame does not fit the allocation.
/// * [`DmaBufferError::NotAligned`] if `base`, `capacity` or `frame_len` is not a
///   multiple of [`CACHE_LINE`] (the cache-sync would miss part of the DMA writes).
/// * [`DmaBufferError::AddressOverflow`] if `base + frame_len` overflows `usize`.
/// * [`DmaBufferError::OutOfRange`] if the frame is not wholly inside a
///   [`DMA_DEST_REGIONS`] region (a truncated range is what walked `DAR0` past RAM).
/// * [`DmaBufferError::OverlapsDescriptor`] if the frame overlaps the link-list
///   descriptor (the DMA would clobber its own descriptor mid-transfer).
///
/// Returns `Ok(())` only when every rule passes.
///
/// # Examples
///
/// ```
/// use ov5647_core::dma_bounds::{validate_dma_buffer, DmaBufferError};
///
/// let base = 0x4800_0000usize; // PSRAM
/// assert_eq!(validate_dma_buffer(base, 520_192, 512_000, 0, 0), Ok(()));
/// assert_eq!(
///     validate_dma_buffer(base, 520_192, 512_000, base, base + 64),
///     Err(DmaBufferError::OverlapsDescriptor { start: base, end: base + 512_000 })
/// );
/// ```
pub fn validate_dma_buffer(
    base: usize,
    capacity: usize,
    frame_len: usize,
    desc_start: usize,
    desc_end: usize,
) -> Result<(), DmaBufferError> {
    if capacity == 0 || frame_len == 0 {
        return Err(DmaBufferError::Empty);
    }
    if frame_len > capacity {
        return Err(DmaBufferError::TooSmall {
            needed: frame_len,
            capacity,
        });
    }
    if base % CACHE_LINE != 0 || capacity % CACHE_LINE != 0 || frame_len % CACHE_LINE != 0 {
        return Err(DmaBufferError::NotAligned {
            addr: base,
            len: frame_len,
        });
    }
    let end = base
        .checked_add(frame_len)
        .ok_or(DmaBufferError::AddressOverflow {
            addr: base,
            len: frame_len,
        })?;
    if !dma_reachable(base, end) {
        return Err(DmaBufferError::OutOfRange { start: base, end });
    }
    if base < desc_end && desc_start < end {
        return Err(DmaBufferError::OverlapsDescriptor { start: base, end });
    }
    Ok(())
}

/// True when `[start, end)` lies wholly inside one DMA-reachable region.
///
/// Whole-range containment, not `start`-only: a buffer that starts in a legal
/// window but ends outside it is exactly the case that walked `DAR0` past the end
/// of RAM, so it is refused.
pub fn dma_reachable(start: usize, end: usize) -> bool {
    DMA_DEST_REGIONS
        .iter()
        .any(|&(lo, hi)| start >= lo && end <= hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The 800x640 RAW8 frame, in PSRAM (the real capture's destination).
    fn psram_frame() -> (usize, usize, usize) {
        (0x4800_0000, 520_192, 512_000)
    }

    #[test]
    fn valid_psram_frame_passes() {
        let (base, cap, len) = psram_frame();
        assert_eq!(validate_dma_buffer(base, cap, len, 0, 0), Ok(()));
    }

    #[test]
    fn undersized_buffer_is_rejected() {
        // The frame is 512,000 bytes and the buffer capacity is 520,192; a request
        // for 524,288 (multiple of 64) exceeds the capacity and must be refused before
        // any DMA register is written. (A non-aligned short length is rejected earlier,
        // as NotAligned, which is also correct but tests a different rule.)
        let (base, cap, _) = psram_frame();
        assert_eq!(
            validate_dma_buffer(base, cap, 524_288, 0, 0),
            Err(DmaBufferError::TooSmall {
                needed: 524_288,
                capacity: cap,
            })
        );
    }

    #[test]
    fn zero_capacity_is_rejected() {
        assert_eq!(
            validate_dma_buffer(0x4800_0000, 0, 512_000, 0, 0),
            Err(DmaBufferError::Empty)
        );
        assert_eq!(
            validate_dma_buffer(0x4800_0000, 520_192, 0, 0, 0),
            Err(DmaBufferError::Empty)
        );
    }

    #[test]
    fn misaligned_base_is_rejected() {
        // 64-byte alignment is required; 0x...0040 + 1 is not aligned.
        let (_, cap, len) = psram_frame();
        assert_eq!(
            validate_dma_buffer(0x4800_0041, cap, len, 0, 0),
            Err(DmaBufferError::NotAligned {
                addr: 0x4800_0041,
                len,
            })
        );
        assert_eq!(
            validate_dma_buffer(0x4800_0000, cap, len + 1, 0, 0),
            Err(DmaBufferError::NotAligned {
                addr: 0x4800_0000,
                len: len + 1,
            })
        );
    }

    #[test]
    fn address_overflow_is_rejected() {
        // base + len wraps usize: must be caught, never wrap into a region.
        let base = usize::MAX - 63;
        assert_eq!(
            validate_dma_buffer(base, 64, 64, 0, 0),
            Err(DmaBufferError::AddressOverflow {
                addr: base,
                len: 64
            })
        );
    }

    #[test]
    fn out_of_range_is_rejected() {
        // A frame that starts in L2MEM but ends in the reserved top band.
        let base = 0x4FF9_0000;
        let len = 512_000;
        assert_eq!(
            validate_dma_buffer(base, len, len, 0, 0),
            Err(DmaBufferError::OutOfRange {
                start: base,
                end: base + len
            })
        );
    }

    #[test]
    fn overlaps_descriptor_is_rejected() {
        let (base, cap, len) = psram_frame();
        // Descriptor at base..base+64 would be clobbered by the frame.
        let (d0, d1) = (base, base + 64);
        assert_eq!(
            validate_dma_buffer(base, cap, len, d0, d1),
            Err(DmaBufferError::OverlapsDescriptor {
                start: base,
                end: base + len
            })
        );
    }

    #[test]
    fn the_1080p_raw10_frame_fits_the_block_ts_field() {
        // 1920x1080 RAW10 is packed 4 pixels per 5 bytes: 2,592,000 bytes = 324,000 64-bit words.
        assert_eq!(1920 * 1080 * 10 / 8, 2_592_000);
        assert_eq!(2_592_000 / 8, 324_000);
        let ts = block_ts_from_words(324_000).unwrap();
        assert_eq!(ts, 323_999);
        assert!(ts <= BLOCK_TS_MAX);
        // The field is 22 bits, so the 1080p frame is far inside it; the check is here so the next,
        // larger mode fails loudly rather than being masked.
        assert_eq!(BLOCK_TS_MAX, 4_194_303);
    }

    #[test]
    fn a_count_larger_than_the_field_is_rejected_not_masked() {
        let too_big = BLOCK_TS_MAX + 2; // words - 1 = BLOCK_TS_MAX + 1, one past the field
        assert_eq!(
            block_ts_from_words(too_big),
            Err(BlockTsError::TooManyWords {
                words: too_big,
                field_max: BLOCK_TS_MAX,
            })
        );
        // The boundary itself is fine.
        assert_eq!(block_ts_from_words(BLOCK_TS_MAX + 1), Ok(BLOCK_TS_MAX));
    }

    #[test]
    fn zero_words_is_rejected() {
        assert_eq!(block_ts_from_words(0), Err(BlockTsError::ZeroWords));
    }

    #[test]
    fn dma_reachable_requires_whole_range() {
        // Starts in PSRAM, ends outside it => refused even though start is legal.
        assert!(!dma_reachable(0x49FF_FF00, 0x4A00_0100));
        assert!(dma_reachable(0x4800_0000, 0x4807_D000));
        // A stack-top in the reserved band is not reachable by DMA either.
        assert!(!dma_reachable(0x4FFA_0080, 0x4FFA_0100));
    }
}
