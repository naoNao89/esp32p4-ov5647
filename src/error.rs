//! Errors for the OV5647 CSI capture path.
//!
//! The error type is defined here in its own module so that every module
//! in this crate can refer to `crate::error::CsiError` without a dependency
//! cycle; `csi` re-exports it as `crate::csi::CsiError`.

use crate::csi::StatusSnapshot;
use esp_hal::time::Duration;
use ov5647_core::PixelFormat;

/// Everything this driver can refuse or fail on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CsiError {
    /// `width`/`height` is zero or does not fit the 12-bit geometry fields.
    BadGeometry {
        /// Requested frame width in pixels.
        width: u32,
        /// Requested frame height in pixels.
        height: u32,
    },
    /// Lane count is outside the CSI host's range: the host has two data lanes
    /// (`MIPI_CSI_HOST_LL_LANE_NUM_MAX` = 2 in `mipi_csi_host_ll.h`), so this
    /// driver accepts 1..=2.
    BadLaneCount(u8),
    /// The requested per-lane bitrate (Mbps) is outside the D-PHY PLL table
    /// (90..=1500 Mbps, `soc_mipi_csi_phy_pll_ranges[]` in `mipi_csi_periph.c`).
    /// ESP-IDF leaves `hs_freq_sel` at its zero initialiser when no range
    /// matches, i.e. silently selects the lowest range; this driver refuses
    /// instead.
    BadBitrate {
        /// Requested per-lane bitrate in Mbps.
        mbps: u32,
    },
    /// The MIPI D-PHY LDO is not at the verified set-point when capture was about
    /// to be armed.
    ///
    /// This is not a theoretical check. `Psram::new` whole-writes
    /// `EXT_LDO_P0_0P2A`/`..._ANA` and, measured on hardware, **clears `xpd`** --
    /// it switches the D-PHY rail off, leaving `dref=10, mul=0` (~1108 mV) where
    /// the working configuration is `dref=9, mul=6` (~2510 mV). Calling
    /// `enable_phy_ldo()` per cycle only helps if nothing re-initialises PSRAM
    /// afterwards; a lazy PSRAM init on first allocation would land *after* it and
    /// silently undo the fix, reproducing the original "no HS clock" fault in a
    /// much harder-to-see form. So the rail state is asserted immediately before
    /// arming, and capture refuses rather than running on a dead PHY.
    PhyRailNotConfigured {
        /// Whether the D-PHY rail is powered (`xpd` = 1 when on).
        xpd: bool,
        /// `dref` field of the LDO set-point.
        dref: u8,
        /// `mul` field of the LDO set-point.
        mul: u8,
    },
    /// The ISP cannot produce this format. `isp_ll_set_output_data_color_format()`
    /// accepts only RAW8 for a RAW colour space and returns false for the rest
    /// (`isp_ll.h`), so a RAW10 output would need the bypass path instead.
    UnsupportedFormat(PixelFormat),
    /// The ISP output frame is not a whole number of 64-bit words (the
    /// block-transfer size is counted in 64-bit words) or not a whole number of
    /// cache lines (the cache sync has to cover the buffer exactly, which it can
    /// only do if the length is a whole number of lines).
    /// `bits = width * height * out_bpp`.
    Not64BitAligned {
        /// Frame size in bits.
        bits: u64,
    },
    /// A bring-up block-length cap ([`crate::CsiCapture::new_with_buffer_capped`]) was
    /// zero, larger than the frame's word count, or not a whole number of cache
    /// lines. A cap that is silently clamped would make an experiment report on a
    /// transfer it did not perform.
    BadBlockWords {
        /// The cap value that was requested.
        requested: u32,
        /// 64-bit words the frame actually needs.
        available: u32,
    },
    /// The 64-bit-word count does not fit `CH1_BLOCK_TS`, which is 22 bits wide
    /// (`[21:0]`, `dw_gdma_reg.h`). Programming the raw value would be silently
    /// masked by the field, so the engine would move a shorter block and report
    /// success. Refused instead.
    BlockTsOutOfRange {
        /// The 64-bit-word count the block transfer would need.
        words: u32,
        /// The largest value `CH1_BLOCK_TS` can encode (22 bits).
        field_max: u32,
    },
    /// The ISP output frame does not fit the destination: for
    /// [`crate::CsiCapture::new`] that is the `.bss` static sized
    /// [`crate::framebuffer::FRAME_BUFFER_LIMIT`], for [`crate::CsiCapture::new_with_buffer`] it is the
    /// caller's [`crate::framebuffer::DmaFrameBuffer`] capacity.
    BufferTooSmall {
        /// Bytes the frame needs.
        needed: usize,
        /// Bytes the destination can hold.
        capacity: usize,
    },
    /// A [`crate::framebuffer::DmaFrameBuffer`] was constructed from an empty slice (or for a
    /// zero-length frame). There is nothing to write, so there is nothing to
    /// validate.
    EmptyBuffer,
    /// A [`crate::framebuffer::DmaFrameBuffer`] is not 64-byte aligned, or a length is not a whole
    /// number of 64-byte cache lines. `cache_sync` must cover the DMA's writes
    /// exactly, so both the base address and the length have to be multiples of
    /// [`crate::framebuffer::CACHE_LINE`]. `addr` is the slice base, `len` the frame length.
    BufferNotAligned {
        /// Slice base address.
        addr: usize,
        /// Frame length.
        len: usize,
    },
    /// `addr + len` overflowed `usize`. Checked, never a bare `+`: a wrapped end
    /// address would compare as *inside* a region rather than outside it.
    BufferAddressOverflow {
        /// Slice base address.
        addr: usize,
        /// Frame length.
        len: usize,
    },
    /// The destination range is not wholly inside a region this chip's DMA can
    /// reach. The accepted ranges are [`ov5647_core::dma_bounds::DMA_DEST_REGIONS`]; see that constant for
    /// the citation of each and for what the rejected ranges contain.
    BufferOutOfRange {
        /// First byte of the destination range.
        start: usize,
        /// One past the last byte of the destination range.
        end: usize,
    },
    /// The destination range overlaps the DW-GDMA link-list item
    /// (`LINK_ITEM`) — the request/response descriptor the engine fetches and
    /// writes status back into. A transfer into it would destroy its own
    /// descriptor.
    BufferOverlapsDescriptor {
        /// First byte of the destination range.
        start: usize,
        /// One past the last byte of the destination range.
        end: usize,
    },
    /// [`crate::CsiCapture::start`] has not been called (or `stop()` has).
    NotStarted,
    /// [`crate::CsiCapture::start`] was called while a transfer is already armed. `wait_frame` (or
    /// `stop()`) has to finish the previous transaction first: `start()` reprograms the channel's
    /// registers, which is only safe between transactions.
    AlreadyArmed,
    /// The DW-GDMA controller never released its software reset.
    DmaResetTimeout,
    /// No frame arrived within the timeout. The snapshot is what the pipeline
    /// reported at that moment, which is the useful part of a bring-up failure.
    Timeout {
        /// How long `wait_frame` waited before giving up.
        waited: Duration,
        /// What the pipeline reported at the moment of the timeout.
        status: StatusSnapshot,
    },
    /// The DW-GDMA reported a transfer error (channel event bits 5..=8 and
    /// 9..=13 in `dw_gdma_ll.h`'s channel event bitmap).
    DmaError(StatusSnapshot),
    /// The cache sync engine did not complete in the spin budget.
    CacheSyncFailed,
}
