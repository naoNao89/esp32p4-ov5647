//! DW-GDMA channel programming and the link-list item.
//!
//! The DW-GDMA (at `0x5008_1000`) moves a frame from the CSI bridge to the
//! destination buffer. The channel is programmed completely here: flow control,
//! handshake, SAR/DAR, the 22-bit `BLOCK_TS` field (count of 64-bit words), and
//! the CTL0/CTL1 transfer-control words.
//!
//! ESP-IDF headers are cited by basename, and the identifiers taken from them were
//! checked at tag `v5.5` and at the commit this repo's notes pin,
//! `d445e947156f276dfe0db5804e4edf6ae90fe6fb`; where the two disagree the comment
//! says which one it follows. The `v5.5` locations
//! `components/hal/esp32p4/include/hal/` and
//! `components/hal/include/hal/dw_gdma_types.h` do not exist at the pinned commit —
//! the per-chip HAL tree was split into `components/esp_hal_dma/`,
//! `components/esp_hal_cam/` and `components/esp_hal_cache/`, so those paths return
//! 404 there and the same header has to be fetched from the new one
//! (`components/hal/include/hal/color_types.h` is the one that stayed put).
//!
//! # Citations
//!
//! The `§N` section numbers and the note names used below (`bringup-log.md`,
//! `psram.md`, `pre-v3-memory-map.md`) refer to the author's private bring-up
//! notes, kept outside this repository: they are **not shipped here**, so they are
//! background rather than a reference a reader can open. What backs a claim is
//! either the upstream source named beside it (ESP-IDF at the tag and commit this
//! file pins), or an address range and size that `ov5647_core::dma_bounds` and
//! its tests pin.

use core::cell::UnsafeCell;

use crate::error::CsiError;
use esp32p4::{DMA, HP_SYS_CLKRST};

/// Address window through which the CSI bridge exposes its internal buffer, and
/// therefore the DW-GDMA source address (`MIPI_CSI_BRG_MEM_BASE`, §1.1/§5.5).
const MIPI_CSI_BRG_MEM_BASE: u32 = 0x5010_4000;

/// Master port of the DW-GDMA that can reach the CSI bridge
/// (`DW_GDMA_LL_MASTER_PORT_MIPI_CSI = 0`, `dw_gdma_ll.h`).
const DW_GDMA_PORT_MIPI_CSI: u8 = 0;

/// Master port that reaches L2MEM/PSRAM/flash (`DW_GDMA_LL_MASTER_PORT_MEMORY = 1`).
const DW_GDMA_PORT_MEMORY: u8 = 1;

/// Spin budget for hardware handshakes that must not hang the caller. Each
/// iteration is a volatile register read.
const SPIN_LIMIT: u32 = 1_000_000;

/// The 64-byte DW-GDMA link-list item (§7.3), placed in internal RAM so the DMA
/// can read it. One item is enough for the single-buffer capture; the item's
/// `llp` is 0, i.e. end of list.
///
/// The layout is fixed by `dw_gdma_ll.h`'s `dw_gdma_link_list_item_t`: 16 words
/// in the same order, `DW_GDMA_LL_LINK_LIST_ALIGNMENT` = 64 bytes and 64-byte
/// aligned, which the header static-asserts. The field names below differ from the
/// header's in two places — `block_ts` is `block_ts_lo` there, and `ctl_lo`/`ctl_hi`
/// are `ctrl_lo`/`ctrl_hi`.
#[repr(C, align(64))]
struct LinkListItem {
    sar_lo: u32,
    sar_hi: u32,
    dar_lo: u32,
    dar_hi: u32,
    block_ts: u32,
    reserved_14: u32,
    llp_lo: u32,
    llp_hi: u32,
    ctl_lo: u32,
    ctl_hi: u32,
    sstat: u32,
    dstat: u32,
    status_lo: u32,
    status_hi: u32,
    reserved_38: u32,
    reserved_3c: u32,
}

struct StaticLinkItem(UnsafeCell<LinkListItem>);

// SAFETY: as `StaticFrameBuffer`; the item is written before the DMA is armed.
unsafe impl Sync for StaticLinkItem {}

static LINK_ITEM: StaticLinkItem = StaticLinkItem(UnsafeCell::new(LinkListItem {
    sar_lo: 0,
    sar_hi: 0,
    dar_lo: 0,
    dar_hi: 0,
    block_ts: 0,
    reserved_14: 0,
    llp_lo: 0,
    llp_hi: 0,
    ctl_lo: 0,
    ctl_hi: 0,
    sstat: 0,
    dstat: 0,
    status_lo: 0,
    status_hi: 0,
    reserved_38: 0,
    reserved_3c: 0,
}));

/// Address ranges a DW-GDMA frame destination may live in on this part, as
/// `(start, end)` with an exclusive `end`. Anything outside them is refused with
/// [`CsiError::BufferOutOfRange`].
///
/// * `0x4800_0000 .. 0x4A00_0000` — external PSRAM as this firmware actually
///   maps and verifies it: 32 MiB at `0x4800_0000`, MMU-mapped and round-tripped
///   byte-exact with zero mismatches (`bringup-log.md` section 31,
///   "HARDWARE: PSRAM works"; window provenance in `psram.md`).
///   The TRM lists the DMA-reachable external-RAM window as
///   `0x4800_0000..0x4BFF_FFFF`, but only 32 MiB is fitted on this board, so the
///   upper half is rejected rather than assumed present. The destination reaches
///   PSRAM through DW-GDMA master port 1 ([`DW_GDMA_PORT_MEMORY`]), which the
///   link item's `DMS` selects (`ctl0_word`).
/// * `0x4FF0_0000 .. 0x4FF2_CBD0` and `0x4FF4_0000 .. 0x4FFA_0000` — the two
///   application-visible internal-RAM windows of pre-v3 silicon
///   (`pre-v3-memory-map.md` section 2.5 and
///   `bringup-log.md` section 31's companion section 33; the endpoints are
///   `SRAM_LOW_START/END` and `SRAM_HIGH_START/SIZE` in ESP-IDF's
///   `esp_system/ld/esp32p4/memory.ld.in`, with the measured 128 KiB L2 split).
///   `SRAM_LOW_END` is the one endpoint the two revisions disagree about:
///   `0x4FF2_CBD0` at tag `v5.5` (line 19 of that file) against `0x4FF2_BBD0` at
///   the pinned commit `d445e947` (line 32) — 4 KiB lower. The region table in
///   `ov5647_core::dma_bounds` uses the `v5.5` value.
///
/// The two internal windows are **disjoint**, and everything between/above them
/// is deliberately not accepted: `0x4FF2_CBD0..0x4FF4_0000` — whose last 4 KiB
/// already belongs to the bootloader under the pinned commit's layout — holds the
/// bootloader image, the ROM's stack and the ROM's `.bss`/`.data`, and
/// `0x4FFA_0000..0x4FFC_0000` is the L2 cache on this revision. A range that
/// straddles the gap, lands in the ROM's memory, or sits in the cache is
/// therefore rejected instead of being handed to an engine that would happily
/// write over it.
///
/// The DMA-reachable region table and the reachability check now live in
/// `ov5647_core::dma_bounds`, the single source of truth for DMA-buffer validation (host-tested
/// under `cargo test --features std`). The table (`DMA_DEST_REGIONS`) and `dma_reachable` are
/// documented there.
/// Address range of the DW-GDMA link-list item: the request half (`sar`, `dar`,
/// `block_ts`, `ctl`) the engine fetches, and the response half (`sstat`,
/// `dstat`, `status`) it writes back. A frame destination must not overlap it, or
/// the transfer would overwrite its own descriptor.
pub(crate) fn descriptor_range() -> (usize, usize) {
    let start = LINK_ITEM.0.get() as usize;
    let len = core::mem::size_of::<LinkListItem>();
    (start, start.saturating_add(len))
}

/// §7.1: channel flow control/handshake/priority/outstanding limits and the
/// setup that must happen once, then the §7.3 64-byte link-list item.
pub(crate) fn configure_dma_channel(dma: &DMA, block_ts: u32, dest_addr: usize) {
    let ch = dma.ch1();

    // Transfer flow: peripheral (CSI bridge) -> memory, controlled by the
    // source peripheral, hardware handshake, CSI as the handshaking
    // peripheral. `_MAKE_GDMA_FLOW_CTRL_CODE(1, 0, DW_GDMA_FLOW_CTRL_SRC)`
    // -> DW_GDMA_LL_FLOW_P2M_SRC = 4 (dw_gdma_ll.h's tt_fc enum).
    ch.cfg1().modify(|_, w| unsafe {
        w.ch1_tt_fc()
            .bits(DW_GDMA_FLOW_P2M_SRC)
            .ch1_hs_sel_src()
            .clear_bit()
            .ch1_hs_sel_dst()
            .clear_bit()
            .ch1_src_per()
            .bits(DW_GDMA_PERIPH_CSI)
    });
    // Both sides are contiguous-block transfers. The reference configures the CSI channel this
    // way (`dw_channel_config` leaves `src_multblk_type`/`dst_multblk_type` at
    // `DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS` and the measured `LLP0` is 0), and it measures
    // `CTL1 = 0x000F87C0` with no block markers.
    //
    // This was `DW_GDMA_BLOCK_TRANSFER_LIST` with a 64-byte link item and `LLP0` pointing at
    // it. The engine then fetched the item and loaded SAR/DAR/BLOCK_TS/CTL0/CTL1 from it, which
    // is why the channel registers read back populated either way — but a list transfer also
    // requires the shadow/list block markers in CTL1, and those are what put `0x44000000` into
    // a register the reference has at `0x000F87C0`.
    ch.cfg0().modify(|_, w| unsafe {
        w.ch1_src_multblk_type()
            .bits(DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS)
            .ch1_dst_multblk_type()
            .bits(DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS)
    });
    // Channel priority 1 and 5 outstanding requests per side
    // (`num_outstanding_requests = 5` -> `reg = 5 - 1 = 4`).
    ch.cfg1().modify(|_, w| unsafe {
        w.ch1_ch_prior()
            .bits(DMA_CHANNEL_PRIORITY)
            .ch1_src_osr_lmt()
            .bits(DMA_OUTSTANDING_REQUESTS - 1)
            .ch1_dst_osr_lmt()
            .bits(DMA_OUTSTANDING_REQUESTS - 1)
    });
    // `status_fetch_addr = MIPI_CSI_BRG_MEM_BASE` for the source; the
    // destination has none. Status write-back itself stays disabled, exactly
    // as in the CSI driver's allocation config (`en_status_write_back` unset).
    ch.sstatar0()
        .write(|w| unsafe { w.ch1_sstatar0().bits(MIPI_CSI_BRG_MEM_BASE) });
    ch.dstatar0().write(|w| unsafe { w.ch1_dstatar0().bits(0) });
    // Latch every channel event for polling (`INTSTATUS_ENABLE0 = all`, as
    // `dw_gdma_ll_channel_enable_intr_generation(dev, chan, UINT32_MAX, true)`
    // does); propagation to the CPU stays off because `cfg0.int_en` is 0.
    ch.intstatus_enable0()
        .write(|w| unsafe { w.bits(u32::MAX) });
    ch.intclear0().write(|w| unsafe { w.bits(u32::MAX) });

    // The transfer's own registers, written directly because the channel is
    // contiguous: no link item, no LLI fetch, `LLP0` stays 0. The reference's
    // channel holds exactly these (`sar = 0x50104000`, `block_ts = 0x0000f9ff`,
    // `ctl0 = 0x00221b14`, `ctl1 = 0x000f87c0`, `llp0 = 0`).
    //
    // `DAR` and `BLOCK_TS` are re-written by `start()` for each transfer (the
    // buffer and the count change); `SAR` and the two control words do not.
    ch.sar0()
        .write(|w| unsafe { w.ch1_sar0().bits(MIPI_CSI_BRG_MEM_BASE) });
    ch.sar1().write(|w| unsafe { w.ch1_sar1().bits(0) });
    ch.dar0()
        .write(|w| unsafe { w.ch1_dar0().bits(dest_addr as u32) });
    ch.dar1().write(|w| unsafe { w.ch1_dar1().bits(0) });
    ch.block_ts0()
        .write(|w| unsafe { w.ch1_block_ts().bits(block_ts) });
    ch.ctl0().write(|w| unsafe { w.bits(ctl0_word()) });
    ch.ctl1().write(|w| unsafe { w.bits(ctl1_word()) });
    ch.llp0().write(|w| unsafe { w.bits(0) });
}

/// §2.3 item 6: DW-GDMA bus clock and reset, then the controller's software
/// reset handshake and `dmac_en`. Global interrupt generation stays off: this
/// driver polls.
pub(crate) fn reset_dma_controller(clkrst: &HP_SYS_CLKRST, dma: &DMA) -> Result<(), CsiError> {
    critical_section::with(|_| {
        clkrst
            .soc_clk_ctrl0()
            .modify(|_, w| w.gdma_cpu_clk_en().set_bit());
        clkrst
            .soc_clk_ctrl1()
            .modify(|_, w| w.gdma_sys_clk_en().set_bit());
        clkrst.hp_rst_en0().modify(|_, w| w.rst_en_gdma().set_bit());
        clkrst
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_gdma().clear_bit());
    });

    // `dw_gdma_ll_reset()`: raise the reset and wait for it to self-clear.
    dma.reset0().modify(|_, w| w.dmac_rst().set_bit());
    for _ in 0..SPIN_LIMIT {
        if dma.reset0().read().dmac_rst().bit_is_clear() {
            dma.cfg0()
                .modify(|_, w| w.dmac_en().set_bit().int_en().clear_bit());
            return Ok(());
        }
    }
    Err(CsiError::DmaResetTimeout)
}

/// `DW_GDMA_FLOW_CTRL_SRC` with a peripheral source and a memory destination
/// (`dw_gdma_ll.h`: `DW_GDMA_LL_FLOW_P2M_SRC = 4`).
const DW_GDMA_FLOW_P2M_SRC: u8 = 4;

/// `DW_GDMA_ROLE_PERIPH_CSI` (`dw_gdma_types.h`) as `CFG1.SRC_PER`.
/// Handshake peripheral number written to `CFG1.SRC_PER`.
///
/// This is **not** the same enum as `DW_GDMA_ROLE_PERIPH_CSI` in
/// `dw_gdma_types.h`. IDF translates role -> handshake number in
/// `dw_gdma_ll_channel_set_src_handshake_periph()` (`dw_gdma_ll.h`):
///
/// ```c
/// case DW_GDMA_ROLE_PERIPH_CSI:
///     dev->ch[channel].cfg1.src_per = DW_GDMA_LL_HW_HANDSHAKE_PERIPH_CSI;
/// ```
///
/// and on ESP32-P4 that enum is
/// `{ PERIPH_DSI = 0, PERIPH_CSI = 1, PERIPH_ISP = 2 }`.
///
/// This driver previously wrote `2` here -- the **ISP** handshake. The channel
/// was therefore armed and waiting for a request the CSI bridge never issues:
/// `dma_words_moved = 0`, no error, bridge buffer filling and eventually
/// discarding an incomplete frame. Exactly what was measured.
const DW_GDMA_PERIPH_CSI: u8 = 1;

/// `DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS` (`dw_gdma_types.h`, first member of the
/// block-transfer enum, = 0).
const DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS: u8 = 0;

/// `DW_GDMA_BLOCK_TRANSFER_LIST` (`dw_gdma_types.h`). The CSI channel here is
/// contiguous, so the list mode is unused.
#[allow(dead_code)]
const DW_GDMA_BLOCK_TRANSFER_LIST: u8 = 3;

/// `chan_priority = 1` from the CSI driver's channel allocation config (§7.1).
const DMA_CHANNEL_PRIORITY: u8 = 1;

/// `num_outstanding_requests = 5` (§7.1); the register holds `limit - 1`.
const DMA_OUTSTANDING_REQUESTS: u8 = 5;

/// CTL0 word of the link item — the same fields ESP-IDF programs through
/// `dw_gdma_ll_channel_set_*` (`dw_gdma_ll.h`) and
/// `dw_gdma_lli_config_transfer()`, at the SVD bit offsets:
///
/// | field | bits | value |
/// |---|---|---|
/// | `SMS` source master port | 0 | 0 = `DW_GDMA_LL_MASTER_PORT_MIPI_CSI` |
/// | `DMS` dest master port | 2 | 1 = `DW_GDMA_LL_MASTER_PORT_MEMORY` |
/// | `SINC` source address increment | 4 | 1 = fixed (`DW_GDMA_ADDR_INC_MODE_FIXED`) |
/// | `DINC` dest address increment | 6 | 0 = increment |
/// | `SRC_TR_WIDTH` / `DST_TR_WIDTH` | 8 / 11 | 3 = 64-bit |
/// | `SRC_MSIZE` / `DST_MSIZE` | 14 / 18 | 8 = 512-item burst |
const fn ctl0_word() -> u32 {
    (DW_GDMA_PORT_MIPI_CSI as u32) << 0
        | (DW_GDMA_PORT_MEMORY as u32) << 2
        | 1 << 4 // SINC: FIXED
        | 0 << 6 // DINC: INCREMENT
        | 3 << 8 // SRC_TR_WIDTH: 64-bit
        | 3 << 11 // DST_TR_WIDTH: 64-bit
        | 8 << 14 // SRC_MSIZE: 512
        | 8 << 18 // DST_MSIZE: 512
}

/// CTL1 word of the channel: AXI burst lengths 16 per side (`axi_burst_len = 16` in the CSI
/// driver's transfer config) and **nothing else**.
///
/// The reference measures `dma_ch1_ctl1 = 0x000f87c0`, which is exactly `ARLEN_EN`, `ARLEN = 15`,
/// `AWLEN_EN`, `AWLEN = 15` — the two burst-length fields and no marker bits. ESP-IDF's own channel
/// agrees: `dw_gdma_channel_config_transfer()` sets only the burst lengths and
/// **never calls `dw_gdma_ll_channel_set_block_markers()`** — the same
/// burst-lengths-only distinction in `ctl1_word()`.
///
/// This driver used to set bit 26 (`IOC_BLKTFR`), bit 30 (`SHADOWREG_OR_LLI_LAST`) and bit 31
/// (`SHADOWREG_OR_LLI_VALID`) as well, because it armed the channel through a 64-byte link item.
/// Those markers are defined for **shadow/list** transfers; the reference runs the CSI channel
/// contiguous (`LLP0 = 0`), and measured `0x440F87C0` against the reference's `0x000F87C0` — the
/// difference is bit 26 plus bit 30 and nothing else, and bit 31 read back 0 regardless.
///
/// `15` in both length fields is the value the working reference reads back
/// (`dma_ch1_ctl1 = 0x000f87c0` has `0b01111` in each) and the value every verified
/// capture here ran with. How the field *encodes* a burst is not settled: the
/// `dw_gdma_ll.h` burst-length writers pass the length through verbatim
/// (`HAL_FORCE_MODIFY_U32_REG_FIELD(dev->ch[channel].ctl1, arlen, len)`), the
/// generated register headers annotate `ARLEN`/`AWLEN` as "NA", and the CSI
/// driver's `axi_burst_len = 16` is therefore not evidence of a `beats - 1`
/// reading. `15` is measured, not decoded.
const fn ctl1_word() -> u32 {
    1 << 6 // ARLEN_EN
        | 15 << 7 // ARLEN = 15
        | 1 << 15 // AWLEN_EN
        | 15 << 16 // AWLEN = 15
}
