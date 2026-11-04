//! Frame buffers and cache maintenance.
//!
//! The DW-GDMA destination is either a caller-owned slice ([`DmaFrameBuffer`])
//! or the internal static buffer. Buffer addresses must be aligned to the
//! 64-byte L1 D-cache cache line and lengths must be whole multiples of it so
//! that `cache_sync` covers the DMA's writes exactly; the destination must also
//! lie inside a DMA-reachable region of this part.
//!
//! ESP-IDF and esp-hal files are cited by basename. The identifiers taken from the
//! ESP-IDF ones (`cache_ll.h`, the generated `cache_reg.h`/`cache_struct.h`) were
//! checked at tag `v5.5` and at the commit this repo's notes pin,
//! `d445e947156f276dfe0db5804e4edf6ae90fe6fb`; the esp-hal ones are the vendored
//! `esp-hal-p4-pre-v3 1.1.0-p4v13.2` dependency, and the ROM entry-point addresses
//! come from `esp-rom-sys-p4-pre-v3 0.1.4-p4v13.1`'s
//! `ld/esp32p4/rom/esp32p4.rom.ld`.
//!
//! # Citations
//!
//! The `§N` section numbers and the note names used below
//! (`pre-v3-memory-map.md`, `psram-frame-bound.md`) refer to the author's private
//! bring-up notes, kept outside this repository: they are **not shipped here**, so
//! they are background rather than a reference a reader can open. What backs a
//! claim is either the upstream source named beside it (the ESP-IDF and esp-hal
//! files cited above), or a constant that the host tests in `ov5647-core` pin.

use core::cell::UnsafeCell;

use crate::error::CsiError;
use crate::gdma::descriptor_range;
use ov5647_core::dma_bounds;

/// L1 D-cache line size on ESP32-P4, and therefore the alignment/size quantum of
/// an internal-RAM DMA buffer (§7.4: `cache_ll_l1_dcache_get_line_size()` = 64).
pub const CACHE_LINE: usize = 64;

/// Size of the static frame buffer.
///
/// The buffer lives in internal RAM (`.bss`), is 64-byte aligned and its length
/// is a multiple of 64 so that the cache sync covers it whole. 320 KiB is what
/// this build's RAM budget allows with headroom: the vendored HAL's `memory.x`
/// places the application in `0x4FF4_0000..0x4FFA_E000` (440 KiB, with the default
/// 256 KiB L2-cache split), and everything else the build keeps in internal RAM
/// is ≈10.5 KiB — `_bss_end = 0x4FF9_2A31` in a rev103 build of
/// `examples/capture_raw10_1920x1080`, minus the RAM origin and minus the
/// 327,680 B this constant sizes. Requests whose ISP output frame is larger are
/// rejected by [`crate::CsiCapture::new`] with [`CsiError::BufferTooSmall`].
/// Frame buffer capacity for the **internal static** buffer.
///
/// Deliberately NOT raised to fit a real mode.
///
/// On pre-v3 silicon internal RAM is two disjoint windows,
/// `0x4FF00000..0x4FF2CBD0` (183,248 B) and `0x4FF40000..0x4FFA0000` (393,216 B),
/// separated by a bootloader/ROM gap, and the L2 cache occupies
/// `0x4FFA0000..0x4FFC0000` (see `pre-v3-memory-map.md`; the
/// low-window end is ESP-IDF `v5.5`'s `SRAM_LOW_END`, and the pinned commit
/// `d445e947` puts it 4 KiB lower, at `0x4FF2BBD0`). A static
/// cannot span the gap, and the largest contiguous window is 393,216 B, so
/// **no legal placement exists for a buffer holding a real OV5647 frame** -- the
/// smallest mode, 800x640 RAW8, needs 512,000 B.
///
/// This value is therefore kept low enough that [`crate::CsiCapture::new`] refuses every
/// real geometry, because a guard that compares against an internal static's size
/// **cannot be made correct** on this chip: raising the constant would make `new()`
/// accept a geometry whose buffer has nowhere legal to live, and the check would
/// pass while the placement was invalid.
///
/// The API change that fixes this has since been made, additively: [`DmaFrameBuffer`]
/// plus [`crate::CsiCapture::new_with_buffer`] take a **caller-provided destination** (a
/// PSRAM slice, verified at `0x4800_0000` for 32 MiB) and validate that range
/// instead of sizing a `.bss` static. This constant is deliberately unchanged: it
/// sizes the static that [`crate::CsiCapture::new`] still uses, and it must keep that
/// constructor refusing every real geometry.
pub const FRAME_BUFFER_LIMIT: usize = 320 * 1024;

#[repr(C, align(64))]
pub(crate) struct FrameBuffer([u8; FRAME_BUFFER_LIMIT]);

/// Wrapper that makes the static buffer shareable. There is exactly one
/// [`CsiCapture`] by contract (it owns the CSI/ISP/DW-GDMA blocks), and the
/// buffer is only written by the DMA while `start()`..`wait_frame()` is in
/// flight, which the type's state machine enforces.
pub(crate) struct StaticFrameBuffer(pub(crate) UnsafeCell<FrameBuffer>);

// SAFETY: see `StaticFrameBuffer`. Access is serialised by the CsiCapture state
// machine; no safe API hands out a `&mut` to the buffer.
unsafe impl Sync for StaticFrameBuffer {}

pub(crate) static FRAME_BUFFER: StaticFrameBuffer =
    StaticFrameBuffer(UnsafeCell::new(FrameBuffer([0u8; FRAME_BUFFER_LIMIT])));

/// A frame destination the **caller** owns.
///
/// This is the pre-v3 form of the §7.6 ownership contract, and the only one that
/// can hold a real frame: internal RAM on this chip is two disjoint windows whose
/// larger half is 393,216 B (`pre-v3-memory-map.md` section 2.5),
/// so the smallest OV5647 mode (800×640 RAW8 = 512,000 B) has no legal `.bss`
/// placement at all. A caller-owned PSRAM slice does.
///
/// # Ownership contract
///
/// The buffer is held as `&'a mut [u8]`, so while a `DmaFrameBuffer` exists the
/// CPU cannot alias it through any safe path: no other reference to those bytes
/// can be alive. That is the whole contract — mutual exclusion is enforced by the
/// borrow checker, not by a flag, and it must be held for the **entire transfer**:
/// the DMA writes the range between [`crate::CsiCapture::start`] and the end of
/// [`crate::CsiCapture::wait_frame`], and the caller may only look at the bytes after
/// `wait_frame` has returned and unwound the channel (§5.6, §8.8).
///
/// # Not a bounding mechanism
///
/// A canary in (or after) the buffer is **not** an acceptable bound on what the
/// DMA writes. It is detected only after the damage, it does not stop the engine
/// from advancing `DAR0` past the end of RAM, and it cannot make an in-range
/// check correct. The bound is the range validation in [`DmaFrameBuffer::new`]
/// plus refusing any frame that does not fit (see [`CsiError::BufferTooSmall`]);
/// the same reasoning that keeps [`FRAME_BUFFER_LIMIT`] unraised. No further
/// runtime bounding mechanism is added here — that is a separate task.
pub struct DmaFrameBuffer<'a> {
    buffer: &'a mut [u8],
    dma_addr: usize,
    capacity: usize,
}

impl<'a> DmaFrameBuffer<'a> {
    /// Takes ownership of `buffer` as the destination for a `frame_len`-byte
    /// frame, after checking every property the DMA depends on.
    ///
    /// Validated, in this order:
    ///
    /// 1. non-empty, and `frame_len` non-zero;
    /// 2. `frame_len <= buffer.len()`;
    /// 3. the base address, the slice length (`buffer.len()`) and `frame_len` are
    ///    all multiples of [`CACHE_LINE`] — the cache sync has to cover the DMA's
    ///    writes exactly, and a slice whose length is not a line multiple would
    ///    put the write range and the sync range out of step (§7.4);
    /// 4. `base + frame_len` does not overflow (`checked_add`, never a bare `+`);
    /// 5. `[base, base + frame_len)` lies wholly inside one of
    ///    [`ov5647_core::dma_bounds::DMA_DEST_REGIONS`] — only the frame range has to be reachable, since
    ///    the engine is programmed never to write past `frame_len`;
    /// 6. that range does not overlap the DW-GDMA link-list descriptor
    ///    (`descriptor_range`).
    pub fn new(buffer: &'a mut [u8], frame_len: usize) -> Result<Self, CsiError> {
        let base = buffer.as_mut_ptr() as usize;
        let capacity = buffer.len();
        let (desc_start, desc_end) = descriptor_range();
        // The validation rules live in `ov5647_core::dma_bounds` — one source of
        // truth, exercised by the host `cargo test` and used here by the firmware.
        dma_bounds::validate_dma_buffer(base, capacity, frame_len, desc_start, desc_end).map_err(
            |e| match e {
                dma_bounds::DmaBufferError::Empty => CsiError::EmptyBuffer,
                dma_bounds::DmaBufferError::TooSmall { needed, capacity } => {
                    CsiError::BufferTooSmall { needed, capacity }
                }
                dma_bounds::DmaBufferError::NotAligned { addr, len } => {
                    CsiError::BufferNotAligned { addr, len }
                }
                dma_bounds::DmaBufferError::AddressOverflow { addr, len } => {
                    CsiError::BufferAddressOverflow { addr, len }
                }
                dma_bounds::DmaBufferError::OutOfRange { start, end } => {
                    CsiError::BufferOutOfRange { start, end }
                }
                dma_bounds::DmaBufferError::OverlapsDescriptor { start, end } => {
                    CsiError::BufferOverlapsDescriptor { start, end }
                }
            },
        )?;
        Ok(Self {
            buffer,
            dma_addr: base,
            capacity,
        })
    }

    /// The address the DW-GDMA is programmed with: the slice's own base address,
    /// captured once so it cannot change under the transfer.
    pub fn dma_addr(&self) -> usize {
        self.dma_addr
    }

    /// Bytes the caller handed over (`buffer.len()`), which is the most a frame
    /// may occupy.
    pub fn capacity(&self) -> usize {
        self.capacity
    }
}

/// Where the DW-GDMA writes a frame: the `.bss` static of [`crate::CsiCapture::new`],
/// or the caller-owned slice of [`crate::CsiCapture::new_with_buffer`].
pub(crate) enum Destination<'a> {
    /// The internal static. [`crate::CsiCapture::new`] keeps its own bounds check
    /// against this allocation.
    Static,
    /// A caller-owned slice, already validated by [`DmaFrameBuffer::new`].
    Caller(DmaFrameBuffer<'a>),
}

impl<'a> Destination<'a> {
    pub(crate) fn buffer_ptr(&self) -> *mut u8 {
        match self {
            Destination::Static => FRAME_BUFFER.0.get() as *mut u8,
            Destination::Caller(dest) => dest.buffer.as_ptr() as *mut u8,
        }
    }

    /// Bytes behind [`Self::buffer_ptr`]: the **whole** allocation, which for the
    /// caller-owned case is `DmaFrameBuffer::capacity()` and therefore includes
    /// the guard/sentinel granules that the frame itself does not occupy.
    ///
    /// Both cache operations at arm and at completion cover this length rather
    /// than `frame_len`, so a sentinel written by the CPU before arming reaches
    /// memory, and a sentinel the DMA overwrote is invalidated before it is read
    /// (`psram-frame-bound.md` §6.1).
    pub(crate) fn capacity(&self) -> usize {
        match self {
            Destination::Static => FRAME_BUFFER_LIMIT,
            Destination::Caller(dest) => dest.capacity,
        }
    }

    pub(crate) fn dma_addr(&self) -> usize {
        match self {
            Destination::Static => FRAME_BUFFER.0.get() as usize,
            Destination::Caller(dest) => dest.dma_addr,
        }
    }
}

/// Cache sync direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum CacheDir {
    /// CPU → memory, before handing a buffer to the DMA (writeback).
    C2M,
    /// Memory → CPU, after the DMA wrote a buffer (invalidate).
    M2C,
}

/// Runs one cache-maintenance operation over `[addr, addr+len)`.
///
/// §7.5 requires explicit `msync` on this SoC, and it is performed through the
/// ROM's cache-maintenance entry points, resolved by the ROM linker script
/// `esp-rom-sys` contributes (`ld/esp32p4/rom/esp32p4.rom.ld`:
/// `Cache_WriteBack_Addr = 0x4fc003f4` at line 190, `Cache_Invalidate_Addr =
/// 0x4fc003e4` at line 186) — the same two functions esp-hal's own
/// `cache_writeback_addr` / `cache_invalidate_addr` call.
///
/// The CACHE peripheral's own sync engine is **not** used. Its registers are
/// documented field by field in the generated register header — `SYNC_ADDR`
/// (start virtual address), `SYNC_SIZE` (size of the operation), `SYNC_MAP`
/// (which caches take part: `[4]` L1-DCache, `[5]` L2-Cache) and `SYNC_CTRL`
/// (`INVALIDATE_ENA` / `WRITEBACK_ENA` trigger the operation, `SYNC_DONE` reports
/// it finished, and `SYNC_DONE_INT_RAW` / `SYNC_DONE_INT_CLR` in
/// `SYNC_L1_CACHE_PRELOAD_INT_RAW` / `_CLR` carry the same event) — but its
/// `SYNC_DONE` never asserted on this board; see the note in the body.
///
/// The caller must pass a 64-byte aligned address and a 64-byte multiple length
/// (both are guaranteed for the frame buffer and the link item by their types).
pub(crate) unsafe fn cache_sync(addr: usize, len: usize, dir: CacheDir) -> bool {
    debug_assert!(addr % CACHE_LINE == 0 && len % CACHE_LINE == 0 && len > 0);

    // The CACHE sync-engine registers (SYNC_ADDR / SYNC_SIZE / SYNC_MAP /
    // SYNC_CTRL, then poll SYNC_DONE) never complete on this part: the poll timed
    // out on the very first hardware run and surfaced as `CacheSyncFailed` from
    // `start()`. Use the ROM cache-maintenance entry points instead -- the same
    // ones esp-hal's own `cache_writeback_addr` / `cache_invalidate_addr` call.
    const CACHE_MAP: u32 = (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5); // L1 I0, L1 I1, L1 D, L2

    unsafe extern "C" {
        fn Cache_WriteBack_Addr(bus: u32, addr: u32, size: u32);
        fn Cache_Invalidate_Addr(bus: u32, addr: u32, size: u32);
    }

    unsafe {
        match dir {
            CacheDir::C2M => Cache_WriteBack_Addr(CACHE_MAP, addr as u32, len as u32),
            CacheDir::M2C => Cache_Invalidate_Addr(CACHE_MAP, addr as u32, len as u32),
        }
    }
    true
}
