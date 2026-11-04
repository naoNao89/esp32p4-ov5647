//! ESP32-P4 MIPI CSI-2 receiver, D-PHY, ISP and frame DMA driver.
//!
//! Register-level bring-up of the whole camera front-end on **pre-v3 silicon**
//! (chip revision 1.3, `CHIP_SUPPORT_MIN_REV = 103`), assembled section by
//! section from the author's bring-up notes (see *Citations* below). The HAL has
//! no driver for any of this (`esp-hal-p4-pre-v3` 1.1.0 has no `mipi_csi`, `isp`,
//! `ldo` or `dw_gdma` module), so everything here is raw PAC work on:
//!
//! * `PMU` — the MIPI PHY rail (LDO channel 3 / VO3), §3.1
//! * `HP_SYS_CLKRST` — clocks and resets for CSI host, D-PHY, CSI bridge, ISP, DW-GDMA, §2
//! * `MIPI_CSI_HOST` — the CSI-2 RX controller and the D-PHY control port, §3.2/§3.3/§4
//! * `MIPI_CSI_BRIDGE` — packet filter, frame geometry, buffer window, §5
//! * `ISP` — RAW10 Bayer (GBRG) → RGB conversion, §6
//! * `DMA` (DW-GDMA at `0x5008_1000`, *not* `AXI_DMA`) — frame DMA, §7
//! * `CACHE` — the L1-D-cache sync engine used for `msync`, §7.5
//!
//! # Citations
//!
//! The `§N` section numbers and the note names used below refer to the author's
//! private bring-up notes (`p4-csi-isp.md`, `pre-v3-memory-map.md`,
//! `psram-frame-bound.md` and the hydra-x series). They are **not shipped with
//! this repository**, so they are background here rather than a reference a
//! reader can open. What backs a claim is either the upstream source named
//! beside it (ESP-IDF at the commit in *Values this module refuses to invent*,
//! or esp-video), or a register value quoted inline, which the accessors on
//! [`CsiCapture`] (`isp_registers`, `phy_registers`, `status`) read back on a
//! rev-1.3 part.
//!
//! # Data path
//!
//! ```text
//! OV5647 --2 lanes--> MIPI_CSI_HOST --> MIPI_CSI_BRIDGE --(buffer window 0x5010_4000)-->
//!                          ISP (reads and writes that same window) -->
//!                     DW-GDMA channel 1 (src = 0x5010_4000 fixed, dst = frame buffer)
//! ```
//!
//! The ISP is mandatory: the bridge routes data into its buffer window only
//! once a user has claimed it, and on revision 1.3 the bridge's own colour
//! conversion block does not exist, so `set_color_mode_bypass()` is a no-op
//! (§8.1/§8.4).
//!
//! # Order honoured
//!
//! 1. LDO channel 3, then the clock/reset enables of §2.3;
//! 2. D-PHY reset + PLL-range selection (§3.3) and the host-side lane/VC/scrambler config;
//! 3. bridge routing/geometry, then the **ISP fully initialised and enabled** (§6.3/§6.5);
//! 4. only then the CSI host is started and the DW-GDMA armed (`start()`);
//! 5. the sensor must be streamed by the caller **after** `start()` returns (§8.11).
//!
//! `new()` performs steps 1–4's setup but arms nothing; `start()` arms the DMA and
//! enables the bridge in the §5.6 order (DMA first, bridge second); `stop()` unwinds
//! them in the reverse order.
//!
//! # Buffer ownership contract (§7.6, §8.8)
//!
//! One frame buffer, a `static` in internal RAM, 64-byte aligned (the L1 D-cache
//! line size, §7.4) and a multiple of the line size in length. The DMA is armed
//! **only** by [`CsiCapture::start`] and unwound again by the end of
//! [`CsiCapture::wait_frame`] (bridge off, then channel off — §5.6), so the
//! buffer can only be written while a capture is in flight, and never while the
//! caller is reading the frame it got back. That is the single-buffer form of the
//! §7.6 ownership contract, and it is why two buffers are not needed to avoid the
//! tearing described in §8.8.
//!
//! `wait_frame()` therefore means "exactly one frame": to capture continuously the
//! caller loops `start(); wait_frame(..); consume(buffer());`.
//!
//! [`CsiCapture::new_with_buffer`] is the same contract with a buffer the **caller**
//! owns: a [`DmaFrameBuffer`], whose `&'a mut [u8]` makes the CPU-side exclusion a
//! borrow-checker guarantee, and which is validated against the DMA-reachable
//! address ranges of this chip before any register is written. That is the form
//! that can actually hold a frame, because internal RAM on pre-v3 is two disjoint
//! windows (the larger 384 KiB) and no 512,000-byte `.bss` static has a legal
//! home there (`pre-v3-memory-map.md`; this crate's `ld/stack-top.x` records the
//! `.bss`/stack placement that follows from it).
//!
//! # Values this module refuses to invent
//!
//! Everything below is *not* fixed by the bring-up notes, so it is derived from
//! data the part itself provides, left unprogrammed (hardware reset default
//! applies), or rejected with an error instead of guessed. [`UNRESOLVED`] is the
//! machine-readable copy of this list.
//!
//! 1. **LDO set-point.** `PMU.EXT_LDO_P0_0P2A_ANA.dref/mul` for 2500 mV are
//!    eFuse-calibrated; IDF *computes* them by exhaustive search over
//!    `ldo_vo3_k/vos/c` (§3.1.1). This part's calibration (K=40, VOS=37, C=5)
//!    gives `dref=9`/`mul=6` -> 2509.9 mV, which is what
//!    [`LDO_DREF_2500MV`]/[`LDO_MUL_2500MV`] hold and what `enable_phy_ldo()`
//!    writes. [`CsiCapture::start`] then refuses to arm unless the pair reads
//!    back. At the reset default (`dref=10`, `mul=0` = 1108.5 mV) the rail
//!    starved the D-PHY: high-speed reception was dead.
//! 2. **D-PHY LP/HS timing.** ESP-IDF writes exactly one D-PHY register (0x44)
//!    and no settle/lane timing registers; the TRM values are unknown here
//!    (§3.3, open item 2). Nothing else is programmed.
//! 3. **Per-lane bitrate.** The reference code uses 200/221 Mbps while the sensor
//!    driver declares 400/408 Mbps for the same modes (§3.4, open item 5). The
//!    bitrate is therefore a **parameter** of `new()`; a value outside the
//!    documented 90–1500 Mbps table is rejected rather than silently mapped to
//!    the lowest PLL range the way ESP-IDF does.
//! 4. **Bridge `HADR_NUM`/`VADR_NUM`: count or count−1.** IDF's bridge code writes
//!    the count while the ISP writes count−1 (§5.3, open item 6). This module
//!    writes the count to match IDF and expects to be checked against the
//!    `VADR_NUM_GT`/`VADR_NUM_LT` bridge status bits during bring-up.
//! 5. **`ENDIAN_MODE.BIT_ENDIAN_ORDER`.** The SVD says "N/A" (open item 7); left
//!    unprogrammed. `BYTE_ENDIAN_ORDER` is programmed to no-swap.
//! 6. **L2 cache line size.** Only the L1 D-cache line size (64 B) is verified
//!    (§7.4, open item 10). The buffer is 64-B aligned and its length is a
//!    multiple of 64; if the L2 line is larger than that on real silicon, the
//!    sync range must be widened.
//! 7. **The module/board 1.8 V rail.** Unverified everywhere (§3.1, open item 11);
//!    no register outside LDO channel 3 is touched for PHY power.
//! 8. **ISP shadow-register mode.** `isp_ll_shadow_set_mode()` is compiled out
//!    below chip rev 3.0 in ESP-IDF, so `SHADOW_REG_CTRL.SHADOW_UPDATE_SEL` is
//!    left unprogrammed here too (§6.3 step 19 vs. `isp_ll.h`'s
//!    `#if CHIP_SUPPORT_MIN_REV >= 300`).
//! 9. **Bridge data-type window.** §5.3 (the quoted driver code, standard path)
//!    says `DATA_TYPE_MIN/MAX = 0x12/0x2f`, §8.12's checklist says "0x2B for
//!    RAW10". This module writes 0x12/0x2f (RAW10 = 0x2B lies inside that window).
//! 10. **Sensor-side `0x4800` bit 4** (§8.11, open item 9) is the caller's
//!    problem: this module never touches SCCB.
//!
//! Open items that *are* resolved by re-reading the pinned ESP-IDF commit
//! `d445e947156f276dfe0db5804e4edf6ae90fe6fb` (the source the note tags `[S1]`: a
//! `master` merge commit of 2026-10-28, not a release, so a v5.5.0 checkout has
//! some of the paths below in different directories and some files not at all):
//!
//! * D-PHY config clock source default = `MIPI_CSI_PHY_CLK_SRC_PLL_F20M` = 0
//!   (`soc/esp32p4/include/soc/clk_tree_defs.h`, open item 4), with 0 =
//!   `PLL_F20M` per `mipi_csi_ll.h: mipi_csi_ll_set_phy_clock_source()`.
//! * ISP clock source default = `ISP_CLK_SRC_PLL240` (`clk_tree_defs.h`), encoded
//!   as 1 by `isp_ll.h: isp_ll_select_clk_source()`; IDF's example asks for
//!   80 MHz, so `peri_clk_ctrl26.reg_isp_clk_div_num = 240/80 − 1 = 2`
//!   (`isp_core.c` + `isp_ll.h: isp_ll_set_clock_div()`, "integer − 1").
//! * DW-GDMA bus clock/reset ownership (open item 8): `dw_gdma_ll.h:
//!   dw_gdma_ll_enable_bus_clock()` sets `soc_clk_ctrl0.gdma_cpu_clk_en` **and**
//!   `soc_clk_ctrl1.gdma_sys_clk_en`, and `dw_gdma_ll_reset_register()` pulses
//!   `hp_rst_en0.rst_en_gdma`.
//! * DW-GDMA field encodings (the note only lists register offsets): field values
//!   come from `hal/dw_gdma_ll.h` + `hal/dw_gdma_types.h` (master ports, transfer
//!   width, burst size, address-increment mode, flow control, link-list layout)
//!   and from the SVD bit offsets for every field written here.

/// A single point-in-time DMA/Bridge/DMA-channel snapshot, stored so the lead can read the state
/// `wait_frame` sees without USB printing during the measurement.
#[derive(Clone, Copy)]
pub struct TraceSample {
    /// Microseconds since the DMA arm, per `Instant::elapsed()`.
    pub us: u32,
    /// `DMA.CH1_INTSTATUS0` sampled at this point.
    pub intstatus0: u32,
    /// `CH1_STATUS0.CMPLTD_BLK_TFR_SIZE` (64-bit words moved) at this point.
    pub words: u32,
    /// `CH1_CFG0` (raw) at this point.
    pub cfg0: u32,
    /// `MIPI_CSI_BRIDGE.INT_RAW` at this point.
    pub bridge_int_raw: u32,
    /// `MIPI_CSI_BRIDGE.BUF_FLOW_CTL` (raw) at this point.
    pub bridge_buf_flow: u32,
    /// `MIPI_CSI_BRIDGE.DMA_REQ_CFG` (raw) at this point.
    pub bridge_dma_req: u32,
    /// `DMA.CHEN0.CH1_EN` at this point.
    pub ch_en: bool,
}

const TRACE_CAP: usize = 32;
#[derive(Clone, Copy)]
/// A fixed-capacity ring of [`TraceSample`]s, filled by the bounded sampler
/// during a mid-transfer probe.
pub struct TraceBuf {
    /// Up to `TRACE_CAP` samples taken during one bounded sampling run.
    pub samples: [TraceSample; TRACE_CAP],
    /// How many of `samples` are valid (filled by the sampler).
    pub len: usize,
}

impl TraceBuf {
    /// Const-fn constructor yielding an all-zero, empty trace buffer.
    pub const fn new() -> Self {
        Self {
            samples: [TraceSample {
                us: 0,
                intstatus0: 0,
                words: 0,
                cfg0: 0,
                bridge_int_raw: 0,
                bridge_buf_flow: 0,
                bridge_dma_req: 0,
                ch_en: false,
            }; TRACE_CAP],
            len: 0,
        }
    }
}

use core::cell::Cell;

use esp32p4::{DMA, HP_SYS_CLKRST, ISP, MIPI_CSI_BRIDGE, MIPI_CSI_HOST, PMU};
use esp_hal::delay::Delay;
use esp_hal::time::{Duration, Instant};
use ov5647_core::PixelFormat;

pub use crate::error::CsiError;
use crate::framebuffer::cache_sync;
use crate::framebuffer::CacheDir;
use crate::framebuffer::Destination;
use crate::framebuffer::DmaFrameBuffer;
use crate::framebuffer::CACHE_LINE;
use crate::framebuffer::FRAME_BUFFER;
use crate::framebuffer::FRAME_BUFFER_LIMIT;
use crate::isp::isp_data_type_for;
use crate::isp::isp_output_config;
use crate::isp::IspRegisters;
use crate::isp::DEMOSAIC_PADDING_SRND_DATA;
use crate::isp::ISP_BAYER_MODE_GBRG;
use crate::isp::ISP_CLK_DIV_NUM;
use crate::isp::ISP_CLK_SRC_PLL240;
use crate::isp::ISP_LL_EVENT_ALL_MASK;
use crate::isp::ISP_LL_EVENT_ERROR_MASK;
use crate::isp::PHY_CLK_SRC_PLL_F20M;

// ---------------------------------------------------------------------------
// Addresses and fixed values
// ---------------------------------------------------------------------------

/// The MIPI D-PHY LDO set-point for 2500 mV on this part: with the eFuse
/// calibration K=40/VOS=37/C=5, the exhaustive search over `dref`/`mul` picks
/// 9/6 -> 2509.9 mV (the runner-up, 10/5, gives 2487.2 mV).
/// `enable_phy_ldo()` writes these; [`CsiCapture::start`] refuses to arm unless
/// they read back.
pub const LDO_DREF_2500MV: u8 = 9;
/// The `mul` field of the 2500 mV D-PHY LDO set-point (§ `LDO_DREF_2500MV`).
pub const LDO_MUL_2500MV: u8 = 6;

/// The items the bring-up notes leave open, one entry per numbered item in the
/// module documentation above, each restated as what this module does instead:
/// derive it, take it as a parameter, or leave it unprogrammed.
pub const UNRESOLVED: [&str; 9] = [
    "LDO set-point: PMU EXT_LDO_P0_0P2A_ANA.dref/mul for 2500 mV derived from the eFuse calibration (doc 3.1.1); 9/6 written",
    "D-PHY LP/HS timing registers: only PHY reg 0x44 is written (doc 3.3)",
    "Per-lane bitrate: 200 vs 221 vs 400/408 Mbps unresolved (doc 3.4); taken as a parameter",
    "Bridge FRAME_CFG.HADR_NUM/VADR_NUM: count vs count-1 unresolved (doc 5.3); count written",
    "Bridge ENDIAN_MODE.BIT_ENDIAN_ORDER: SVD says N/A (doc open item 7); not programmed",
    "L2 cache line size unknown (doc open item 10); buffer aligned/sized to the 64 B L1 line",
    "OV5647 module/board 1.8 V rail sourcing unverified (doc 3.1); only LDO channel 3 touched",
    "ISP SHADOW_REG_CTRL.SHADOW_UPDATE_SEL: ESP-IDF no-op below rev 3.0 (doc 6.3 step 19); not programmed",
    "Bridge data-type window: doc 5.3 says 0x12..0x2f, doc 8.12 says 0x2B (RAW10); 0x12..0x2f written",
];

/// Delay after enabling the MIPI PHY LDO, before the D-PHY is touched (§8.10:
/// the rev-1.3 projects wait 10 ms; IDF exposes the same as
/// `voltage_stable_delay_us`).
const LDO_STABLE_DELAY: Duration = Duration::from_millis(10);

/// Hard iteration bound of one [`CsiCapture::probe_mid_transfer`] run.
///
/// The sampler is a diagnostic that runs while the channel is enabled, so it
/// must never be able to sit there: both this count and
/// [`PROBE_MAX_DURATION`] bound it, and whichever is reached first ends the run.
pub const PROBE_MAX_SAMPLES: u32 = 128;

/// Hard wall-clock bound of one [`CsiCapture::probe_mid_transfer`] run.
pub const PROBE_MAX_DURATION: Duration = Duration::from_millis(1);

/// Spacing between mid-transfer samples. 5 µs × [`PROBE_MAX_SAMPLES`] fits
/// inside [`PROBE_MAX_DURATION`], so the sample count is what normally ends a
/// run.
const PROBE_INTERVAL_US: u32 = 5;

// ---------------------------------------------------------------------------
// Errors, frame description, diagnostics
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A snapshot of the raw MIPI CSI host D-PHY registers, taken when capture
/// refuses to arm, so the failing rail state can be reported.
pub struct PhyRegisters {
    /// `CSI2_RESETN`: 1 = receiver out of reset.
    pub csi2_resetn: u32,
    /// `DPHY_RSTZ`: 1 = D-PHY out of reset.
    pub dphy_rstz: u32,
    /// `PHY_SHUTDOWNZ`: 1 = PHY not shut down.
    pub phy_shutdownz: u32,
    /// `N_LANES`: active data lanes minus one.
    pub n_lanes: u32,
    /// `PHY_CAL`: PHY calibration control.
    pub phy_cal: u32,
    /// `PHY_RX`: bit 17 = clock lane active in HS, bit 16 = clock lane not in
    /// ULPS, bits 0/1 = data lanes 0/1 in ULPS escape.
    pub phy_rx: u32,
    /// `PHY_STOPSTATE`: bit 0 = data lane 0 LP-11, bit 1 = data lane 1 LP-11,
    /// bit 16 = clock lane LP-11.
    pub phy_stopstate: u32,
}

/// One sample of the DW-GDMA channel registers, taken while the channel is
/// enabled.
///
/// Every field is a raw register value. Nothing is written and no interrupt
/// flag is cleared by producing one (the channel's clear path is
/// `CH?_INTCLEAR0`, which the sampler never touches).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MidTransferSample {
    /// `CH1_DAR0` — destination address the channel is using.
    pub dar0: u32,
    /// `CH1_SAR0` — source address the channel is using.
    pub sar0: u32,
    /// `CH1_STATUS0` (raw; `CMPLTD_BLK_TFR_SIZE` lives in the low bits).
    pub status0: u32,
    /// `CH1_INTSTATUS0` (raw channel event bits).
    pub intstatus0: u32,
    /// `DMA_CHEN0` (raw; bit 1 = `CH1_EN`).
    pub chen0: u32,
}

/// Result of one bounded mid-transfer sampling run, from
/// [`CsiCapture::probe_mid_transfer`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MidTransferProbe {
    /// `CH1_DAR0` as read before the first sample: what the channel held when
    /// sampling began.
    pub initial_dar0: u32,
    /// The first `DAR0` value observed that differed from `initial_dar0`, if
    /// the register ever changed during the run.
    pub first_changed_dar0: Option<u32>,
    /// 0-based index of the sample that produced [`Self::first_changed_dar0`].
    pub changed_at_sample: Option<u32>,
    /// Number of samples actually taken.
    pub samples: u32,
    /// The most recent sample (the pre-loop read when `samples == 0`).
    pub last: MidTransferSample,
    /// Time spent sampling.
    pub elapsed: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// A point-in-time sample of the whole capture pipeline's status registers,
/// taken by `CsiCapture::status()` for diagnostics.
pub struct StatusSnapshot {
    /// `DMA.INTSTATUS0` (bit 0 = channel 1, bit 16 = common).
    pub dma_int_status: u32,
    /// `DMA.CH1_INTSTATUS0` (channel event bits, `dw_gdma_ll.h`).
    pub dma_channel_int: u32,
    /// `DMA.CH1_STATUS0` (`CMPLTD_BLK_TFR_SIZE`, 64-bit words transferred).
    pub dma_channel_status: u32,
    /// `DMA.CHEN0.CH1_EN` — false once the channel has finished or been stopped.
    pub dma_channel_enabled: bool,
    /// `MIPI_CSI_BRIDGE.INT_ST`.
    pub bridge_int_st: u32,
    /// `MIPI_CSI_HOST.INT_ST_MAIN` (frame/lane error summary; **clear on read**,
    /// as the per-category `INT_ST_*` registers are, §4.2 — so a raw read of the
    /// register consumes the value and is blind to everything that asserted
    /// before it).
    ///
    /// This field is therefore **not** a raw sample: it is the running OR of
    /// every sample taken since [`CsiCapture::start`], i.e. "did any host error
    /// assert during this transfer". `0` means none did.
    pub host_int_st_main: u32,
    /// `ISP.INT_RAW` (unmasked ISP events).
    pub isp_int_raw: u32,
    /// `MIPI_CSI_HOST.PHY_STOPSTATE` (lane LP-11 detection).
    pub phy_stopstate: u32,
    /// 64-bit words the DW-GDMA channel has actually completed
    /// (`CH1_STATUS0.CH1_CMPLTD_BLK_TFR_SIZE`). Hardware's own answer to "did the
    /// DMA move anything": zero on a timeout means it moved nothing, non-zero
    /// means it is moving data but never reached the programmed count.
    pub dma_words_moved: u32,
    /// `MIPI_CSI_BRIDGE.DMA_REQ_CFG`, read-only. Bit 17 has **no field on rev
    /// 1.x**: the rev-1.x register description
    /// (`soc/esp32p4/register/hw_ver1/soc/mipi_csi_bridge_struct.h`) reserves
    /// bits 17–31. The `CSI_DMA_FLOW_CONTROLLER` name the `esp32p4` PAC offers
    /// at bit 17 is the rev-3.x field (default 1), so the word is reported raw
    /// and nothing is decoded from it.
    pub bridge_dma_req_cfg: u32,
    /// `MIPI_CSI_BRIDGE.BUF_FLOW_CTL.CSI_BUF_DEPTH`: how many pixels the bridge is
    /// currently holding. Non-zero while streaming means the bridge *is* receiving
    /// data from the CSI host, which separates "no data at the bridge" from "data
    /// arrives but is never drained".
    pub bridge_buf_depth: u32,
    /// `MIPI_CSI_BRIDGE.BUF_FLOW_CTL` (raw): `CSI_BUF_AFULL_THRD` `[0:15]` and
    /// `CSI_BUF_DEPTH` `[16:31]`. The threshold is the point at which the bridge
    /// stops being drained politely, so it belongs next to the depth.
    pub bridge_buf_flow_ctl: u32,
    /// `MIPI_CSI_BRIDGE.DATA_TYPE_CFG`: `DATA_TYPE_MIN` `[0:5]`, `DATA_TYPE_MAX` `[8:13]`.
    /// A min/max window, not a single data type.
    pub bridge_data_type_cfg: u32,
    /// `MIPI_CSI_BRIDGE.DMABLK_SIZE`.
    pub bridge_dmablk_size: u32,
    /// `MIPI_CSI_BRIDGE.HOST_CTRL`.
    pub bridge_host_ctrl: u32,
    /// `MIPI_CSI_BRIDGE.CSI_EN.CSI_BRIG_EN` as a number, so the enable state is visible
    /// in the same line as the routing fields it enables.
    pub bridge_csi_en: u32,
    /// `MIPI_CSI_BRIDGE.FRAME_CFG.VADR_NUM_CHECK` (bit 25). When 0 the bridge does not
    /// compare the received row count to `VADR_NUM`, so the absence of `VADR_NUM_GT/LT`
    /// proves nothing about row completeness -- the payload check is the only evidence.
    pub bridge_vadr_num_check: bool,
    /// `MIPI_CSI_BRIDGE.FRAME_CFG.HAS_HSYNC_E` (bit 24).
    pub bridge_has_hsync_e: bool,
    /// `MIPI_CSI_BRIDGE.INT_RAW`: `vadr_num_gt`, `vadr_num_lt`, `discard`,
    /// `csi_buf_overrun`, `csi_async_fifo_ovf`, `dma_cfg_has_updated`.
    pub bridge_int_raw: u32,
    /// `CH1_CTL0.SMS`: source master. 0 = MIPI CSI, 1 = memory.
    pub dma_sms: u32,
    /// `CH1_CTL0.DMS`: destination master. 0 = MIPI CSI, 1 = memory.
    /// Writing the frame buffer through the CSI master would raise
    /// `DST_DEC_ERR`, so this is read back rather than assumed.
    pub dma_dms: u32,
    /// `CH1_CTL0.SINC` / `CH1_CTL0.DINC` (0 = fixed, 2 = increment).
    pub dma_sinc: u32,
    /// `CH1_CTL0.DINC` (0 = fixed, 2 = increment).
    pub dma_dinc: u32,
    /// `CH1_SAR0` (source), `CH1_DAR0` (destination), `CH1_BLOCK_TS0` (count - 1).
    pub dma_sar0: u32,
    /// `CH1_DAR0` — the address the DMA is writing to.
    pub dma_dar0: u32,
    /// `CH1_BLOCK_TS0` — 64-bit words to move, minus one.
    pub dma_block_ts: u32,
    /// `CH1_CFG0` (raw): `DMAC_EN`, `INT_EN` and the channel's flow-control /
    /// handshake encoding.
    pub dma_cfg0: u32,
    /// `CH1_LLP0` (raw): bit 0 is `CH1_LMS` (locality/master-select), bits 5..=37
    /// `CH1_LOC0`. Zero means no link-list pointer, which is what a one-item
    /// block transfer should show.
    pub dma_llp0: u32,
    /// `CH1_LLP1` (raw): bits 0..=31 of `CH1_LOC1`. Zero for the same reason.
    pub dma_llp1: u32,
    /// `CH1_CTL0` (raw): source/destination master, increment modes, transfer
    /// widths and `MSIZE`.
    pub dma_ctl0: u32,
    /// `CH1_CTL1` (raw): AXI burst-length/protection, status write-back and the
    /// link-list `is_last`/`is_valid` markers.
    pub dma_ctl1: u32,
    /// `DMA.COMMONREG_INTSTATUS0` (raw): controller-wide interrupt status
    /// (`slvif_commonreg_dec_err_intstat`, the `MXIF*_ECCPROT_*` bits, ...).
    /// Distinct from [`Self::dma_int_status`], which is the per-channel
    /// aggregate.
    pub dma_common_int_status: u32,
    /// Samples taken while the channel was still enabled, on the `wait_frame`
    /// failure paths. `None` when no failure occurred, so the success path
    /// costs nothing and reports nothing new.
    pub mid_transfer: Option<MidTransferProbe>,
}

impl StatusSnapshot {
    /// True when no CSI host error of any category has asserted **since the last
    /// [`CsiCapture::start`]** — the field is a latched OR, not a sample, so this
    /// is "the host was clean for the whole transfer" rather than "the last poll
    /// happened to see zero".
    pub fn host_clean(&self) -> bool {
        self.host_int_st_main == 0
    }

    /// True when the DW-GDMA channel latched an error event (bits 5..=8 are the
    /// source/destination decode and slave errors, 9..=13 the link-list errors).
    pub fn dma_error(&self) -> bool {
        self.dma_channel_int & 0x0000_3FE0 != 0
    }

    /// `DAR0` observed by the mid-transfer sampler, if one ran.
    ///
    /// Prefers the first value that differed from the pre-sample read — the
    /// value the channel switched to. Falls back to the last sample when `DAR0`
    /// never changed, so "the engine kept the address it was given" and "the
    /// engine replaced it mid-transfer" cannot be confused.
    pub fn sampled_dar0(&self) -> Option<u32> {
        self.mid_transfer
            .map(|p| p.first_changed_dar0.unwrap_or(p.last.dar0))
    }
}

/// What a completed frame transfer produced.
///
/// `len` is derived from the ISP configuration (§4.3/§8.3: never from a
/// hardware "received size"), and `words` is what the DW-GDMA actually reports
/// in `CH1_STATUS0` — the two are compared by the caller if it wants to prove
/// the transfer completed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameInfo {
    /// Frame width in pixels, as configured.
    pub width: u32,
    /// Frame height in rows, as configured.
    pub height: u32,
    /// Format of the bytes behind [`CsiCapture::buffer`].
    pub format: PixelFormat,
    /// Valid bytes in the frame buffer: `width * height * out_bpp / 8`.
    pub len: usize,
    /// 64-bit words the channel reports as transferred (`CH1_STATUS0`).
    pub words: u32,
    /// 64-bit words the block transfer was programmed for.
    pub expected_words: u32,
    /// Time from arming the channel to completion.
    pub elapsed: Duration,
    /// When the bridge's buffer depth first became non-zero, measured from the arm. `None` means it
    /// never did before the transfer completed, i.e. the engine finished before the bridge had
    /// anything to give.
    pub bridge_first_depth: Option<Duration>,
    /// When the bridge's `DISCARD` first asserted, measured from the arm. `None` means it never did.
    /// Comparing this with [`Self::elapsed`] is what separates "the bridge began dropping data while
    /// the DMA was still draining" from "the DMA had already stopped and the bridge kept feeding".
    pub bridge_first_discard: Option<Duration>,
    /// When `wait_frame` began polling, measured from the arm. The delta to [`Self::elapsed`] is the
    /// detection lag: if it is large, the completion and the DISCARD may both have happened before
    /// polling ever started, and the other timestamps are first-observations, not event times.
    pub wait_started_from_arm: Duration,
}

impl FrameInfo {
    /// True when the hardware moved exactly the programmed number of words.
    pub fn complete(&self) -> bool {
        self.words == self.expected_words
    }
}

// ---------------------------------------------------------------------------
// Whole-frame verdict
// ---------------------------------------------------------------------------

/// `MIPI_CSI_BRIDGE.INT_RAW` bit 0 — `VADR_NUM_GT`: the bridge saw **more** rows
/// than `FRAME_CFG.VADR_NUM` (`p4-csi-isp.md` §5.1).
pub const BRIDGE_ERR_VADR_NUM_GT: u32 = 1 << 0;
/// `MIPI_CSI_BRIDGE.INT_RAW` bit 1 — `VADR_NUM_LT`: the bridge saw **fewer** rows
/// than `FRAME_CFG.VADR_NUM`. Either of these is the bridge's own verdict that
/// the frame it forwarded was not the geometry that was configured.
pub const BRIDGE_ERR_VADR_NUM_LT: u32 = 1 << 1;
/// `MIPI_CSI_BRIDGE.INT_RAW` bit 2 — `DISCARD`: a packet was dropped by the
/// data-type filter.
pub const BRIDGE_ERR_DISCARD: u32 = 1 << 2;
/// `MIPI_CSI_BRIDGE.INT_RAW` bit 3 — `CSI_BUF_OVERRUN`: the bridge's pixel FIFO
/// overflowed, i.e. the DMA did not drain it fast enough. A frame that arrives
/// with this bit set has lost pixels regardless of what the word count says.
pub const BRIDGE_ERR_CSI_BUF_OVERRUN: u32 = 1 << 3;
/// `MIPI_CSI_BRIDGE.INT_RAW` bit 4 — `CSI_ASYNC_FIFO_OVF`.
pub const BRIDGE_ERR_CSI_ASYNC_FIFO_OVF: u32 = 1 << 4;
/// Every `INT_RAW` bit of the bridge that means "this frame is not trustworthy".
/// Bit 5 (`DMA_CFG_HAS_UPDATED`) is a normal event, not an error, and is
/// deliberately excluded (`p4-csi-isp.md` §5.1).
pub const BRIDGE_ERR_MASK: u32 = BRIDGE_ERR_VADR_NUM_GT
    | BRIDGE_ERR_VADR_NUM_LT
    | BRIDGE_ERR_DISCARD
    | BRIDGE_ERR_CSI_BUF_OVERRUN
    | BRIDGE_ERR_CSI_ASYNC_FIFO_OVF;

/// Everything needed to say whether a **whole** frame arrived, as distinct from
/// "the DMA stopped".
///
/// This is the answer to "how do we know a whole frame arrived, not a partial
/// one", and it is deliberately more than the DW-GDMA's own completion bit. The
/// channel is armed with `tt_fc = P2M_SRC`
/// (`DW_GDMA_FLOW_P2M_SRC`, `configure_dma_channel`), i.e. the CSI bridge — not
/// the engine — owns the flow control, so *"the block finished"* and *"the
/// programmed number of words moved"* are two different claims and only the
/// second one is about length. The cases this separates:
///
/// * `words < expected_words`: the bridge stopped feeding early (a partial
///   frame). `CH1_STATUS0.CH1_CMPLTD_BLK_TFR_SIZE` is the hardware's own count of
///   64-bit words; it is not derived from configuration anywhere in this driver.
/// * `words > expected_words`: the engine wrote past the block. On a PSRAM
///   destination that is silent corruption, which is why the sentinel granules
///   exist.
/// * `words == expected_words` with a sticky error bit: the right number of
///   bytes moved, but the bridge had already dropped or miscounted rows
///   (`VADR_NUM_GT`/`LT`, `CSI_BUF_OVERRUN`) or the CSI host raised a frame
///   error (`INT_ST_MAIN`, latched) — the length is right and the content is not.
///
/// None of the fields is computed from configuration: each is read out of the
/// block that produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameCompleteness {
    /// `CH1_STATUS0.CH1_CMPLTD_BLK_TFR_SIZE` — 64-bit words the channel reports
    /// as completed. Hardware's count, not a software constant (§4.3/§8.3).
    pub words: u32,
    /// `frame_len / 8`, the value programmed into `BLOCK_TS` (`+1`).
    pub expected_words: u32,
    /// `CH1_INTSTATUS0` bit 1 — the block-transfer-done event latched.
    pub block_done: bool,
    /// `CH1_INTSTATUS0` bit 2 — the DMA-transfer-done event latched.
    pub tfr_done: bool,
    /// `CH1_INTSTATUS0` bits 5..=13 — a decode/slave/link-list error
    /// (`StatusSnapshot::dma_error`).
    pub dma_error: bool,
    /// Latched OR of `MIPI_CSI_HOST.INT_ST_MAIN` over the whole transfer.
    pub host_error_or: u32,
    /// `MIPI_CSI_BRIDGE.INT_RAW & BRIDGE_ERR_MASK`.
    pub bridge_errors: u32,
    /// Raw `MIPI_CSI_BRIDGE.INT_RAW`, so the excluded bit 5 stays visible.
    pub bridge_int_raw: u32,
    /// `ISP.INT_RAW & ISP_LL_EVENT_ERROR_MASK`.
    pub isp_errors: u32,
    /// Raw `ISP.INT_RAW`.
    pub isp_int_raw: u32,
}

impl FrameCompleteness {
    /// True only when a whole, uncorrupted frame arrived: the hardware moved
    /// exactly the programmed number of words, the channel latched no error of
    /// its own, and **not one** sticky error bit is set in the CSI host, the CSI
    /// bridge or the ISP.
    ///
    /// The caller is expected to add the two memory-side checks this type cannot
    /// see — the sentinel granules inside the destination allocation, and the
    /// content verdict on the frame bytes — before it treats the buffer as an
    /// image.
    pub fn whole_frame(&self) -> bool {
        self.words == self.expected_words
            // The transfer completed, signalled by **whichever completion bit this arm latches**.
            // Which one that is depends on `CTL1.IOC_BLKTFR` (bit 26): with it set, the engine
            // latches `INTSTATUS0` bit 0 (`BLOCK_TFR_DONE`); without it, completion arrives as
            // bit 1 (`DMA_TFR_DONE`) — the same coupling as the block-marker bit.
            //
            // This used to require `block_done` alone, which made a perfect frame report
            // `block_done=false` -> `PARTIAL/ERRORED` -> "NOT announcing" once the CSI channel was
            // moved to the reference's contiguous, marker-free `CTL1 = 0x000F87C0`. The reference
            // does not set `IOC_BLKTFR` either, so requiring only that bit would have failed for
            // the reference's own configuration too.
            && (self.block_done || self.tfr_done)
            && !self.dma_error
            && self.host_error_or == 0
            && self.bridge_errors == 0
            && self.isp_errors == 0
    }

    /// One line naming every gate that failed, for a bring-up log. Reads "ok"
    /// only when [`Self::whole_frame`] is true.
    pub fn report(&self) -> FrameCompletenessReport {
        FrameCompletenessReport(*self)
    }
}

/// A `core::fmt` view of [`FrameCompleteness`]; see [`FrameCompleteness::report`].
pub struct FrameCompletenessReport(FrameCompleteness);

impl core::fmt::Display for FrameCompletenessReport {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let c = &self.0;
        write!(
            f,
            "words={}/{} block_done={} tfr_done={} dma_error={} host_int_or={:#06X} \
             bridge_int_raw={:#07X} bridge_err={:#06X} isp_int_raw={:#010X} isp_err={:#06X}",
            c.words,
            c.expected_words,
            c.block_done,
            c.tfr_done,
            c.dma_error,
            c.host_error_or,
            c.bridge_int_raw,
            c.bridge_errors,
            c.isp_int_raw,
            c.isp_errors
        )
    }
}

// ---------------------------------------------------------------------------
// §3.3 per-lane bitrate -> D-PHY PLL range selection
// ---------------------------------------------------------------------------

/// `hs_freq_sel` for a per-lane bitrate, from `soc_mipi_csi_phy_pll_ranges[]`
/// (`mipi_csi_periph.c`), as tabulated in §3.3 of the research note.
///
/// Ranges are inclusive of both ends, in whole Mbps; the table is indexed by
/// integer Mbps exactly as the value passed to ESP-IDF's `lane_bit_rate_mbps`.
/// ESP-IDF initialises `hs_freq_sel` to 0 and leaves it there for anything
/// outside 90–1500 Mbps; this driver returns `None` so the caller gets an error.
pub const fn hs_freq_sel_for_mbps(mbps: u32) -> Option<u8> {
    Some(match mbps {
        90..=99 => 0x00,
        100..=109 => 0x10,
        110..=129 => 0x20,
        130..=139 => 0x01,
        140..=149 => 0x11,
        150..=169 => 0x21,
        170..=179 => 0x02,
        180..=199 => 0x12,
        200..=219 => 0x22,
        220..=239 => 0x03,
        240..=249 => 0x13,
        250..=269 => 0x23,
        270..=299 => 0x04,
        300..=329 => 0x14,
        330..=359 => 0x05,
        360..=399 => 0x15,
        400..=449 => 0x25,
        450..=499 => 0x06,
        500..=549 => 0x16,
        550..=599 => 0x07,
        600..=649 => 0x17,
        650..=699 => 0x08,
        700..=749 => 0x18,
        750..=799 => 0x09,
        800..=849 => 0x19,
        850..=899 => 0x29,
        900..=949 => 0x39,
        950..=999 => 0x0A,
        1000..=1049 => 0x1A,
        1050..=1099 => 0x2A,
        1100..=1149 => 0x3A,
        1150..=1199 => 0x0B,
        1200..=1249 => 0x1B,
        1250..=1299 => 0x2B,
        1300..=1349 => 0x3B,
        1350..=1399 => 0x0C,
        1400..=1449 => 0x1C,
        1450..=1500 => 0x2C,
        _ => return None,
    })
}

/// Bits per pixel the **sensor** puts on the wire for this format.
///
/// Distinct from [`out_bits_per_pixel`] even where the numbers coincide: this one describes the CSI-2
/// input the ISP's input module has to be sized for, which is what `HADR_NUM` is expressed in.
const fn in_bits_per_pixel(format: PixelFormat) -> u32 {
    match format {
        PixelFormat::Raw8 => 8,
        PixelFormat::Raw10 => 10,
        // Unreachable: `isp_data_type_for` rejects these as sensor formats before a `Plan` exists.
        // Mapped through the output table rather than returning 0, because a 0 here would silently
        // size the input module for nothing.
        PixelFormat::Rgb565 => 16,
        PixelFormat::Rgb888 => 24,
    }
}

/// Bits per pixel of the ISP output format (memory side).
const fn out_bits_per_pixel(format: PixelFormat) -> u32 {
    match format {
        PixelFormat::Raw8 => 8,
        PixelFormat::Raw10 => 10,
        PixelFormat::Rgb565 => 16,
        PixelFormat::Rgb888 => 24,
    }
}

// ---------------------------------------------------------------------------
// The frame buffer
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// The caller-owned destination
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Cache maintenance (§7.5)
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// The driver
// ---------------------------------------------------------------------------

/// CSI-2 → ISP → memory capture path.
///
/// `new()` powers and configures everything against the internal static buffer;
/// `new_with_buffer()` does the same against a caller-owned [`DmaFrameBuffer`].
/// `start()`/`stop()` bracket a capture and `wait_frame()` returns when the DMA
/// has delivered a frame.
pub struct CsiCapture<'a> {
    host: MIPI_CSI_HOST,
    bridge: MIPI_CSI_BRIDGE,
    isp: ISP,
    dma: DMA,
    width: u32,
    height: u32,
    format: PixelFormat,
    /// Bytes of ISP output in one frame (`width * height * out_bpp / 8`).
    frame_len: usize,
    /// 64-bit words the block transfer moves (`frame_len / 8`).
    transfer_words: u32,
    /// D-PHY `hs_freq_sel << 1`, the one D-PHY register ESP-IDF programs.
    phy_pll_value: u8,
    lanes: u8,
    /// Set between `start()` and `stop()`: the DMA may write the buffer.
    armed: bool,
    armed_at: Instant,
    trace: TraceBuf,
    /// Largest gap in microseconds between consecutive `wait_frame` polls in the
    /// last capture, for performance bring-up.
    pub last_max_poll_gap_us: u32,
    /// Microseconds the last capture's cache sync spent flushing the frame buffer.
    pub last_cache_sync_us: u32,
    /// OR of every `MIPI_CSI_HOST.INT_ST_MAIN` sample taken since the last
    /// `start()`.
    ///
    /// The CSI host block has **no `INT_RAW`**: its summary register is a
    /// destructive read (`p4-csi-isp.md` §4.2 — "reading `INT_ST_*`
    /// also clears the corresponding bit of `INT_ST_MAIN`") and the per-category
    /// `INT_ST_*` registers are cleared the same way. `wait_frame` polls
    /// `status()` every iteration, so a plain read of the register reports only
    /// the *last* sample and silently discards every error that asserted before
    /// it — a frame could be declared clean while the PHY raised `ERR_SOTHS` /
    /// `ERRESC` or the packet layer raised `ERR_ECC_DOUBLE` during the transfer.
    ///
    /// This latch is the fix, and it is correct whether or not `INT_ST_MAIN`
    /// itself is destructive: an OR over samples is what "did any error ever
    /// assert during this transfer" means either way. Cleared by `start()`.
    host_err_or: Cell<u32>,
    /// Where the DMA writes: the internal static, or the caller's slice.
    dest: Destination<'a>,
}

/// Everything a constructor derives from `(width, height, format, lanes,
/// bitrate)` before it touches hardware.
///
/// Split out so [`CsiCapture::new`] and [`CsiCapture::new_with_buffer`] share one
/// validation path and one bring-up path, instead of two copies that can drift
/// apart on the geometry or ISP arithmetic.
struct Plan {
    width: u32,
    height: u32,
    format: PixelFormat,
    frame_len: usize,
    transfer_words: u32,
    /// The validated value for `CH1_BLOCK_TS`, which is `transfer_words - 1`. Validated once here so
    /// both write sites use a value that is known to fit the 22-bit field.
    block_ts: u32,
    hs_freq_sel: u8,
    lanes: u8,
    isp_data_type: u8,
    /// The sensor's bits per pixel, which is the width the ISP input module is sized in.
    isp_in_bits: u32,
    /// The ISP is bypassed when the sensor's wire format already equals the memory-side
    /// output format. That is the first branch of esp-video's `is_isp_bypass_required()`
    /// (`esp_video/src/device/esp_video_csi_format.c:196`, "ESPRESSIF MIT"): `if
    /// (sensor_v4l2_fmt == requested_fmt) return true;` — a RAW format going to the same
    /// RAW format needs no processing. Measured on the working reference, whose capture
    /// runs bypassed: `ISP.CNTL = 0x00025001` for the whole frame, i.e. `ISP_EN` clear
    /// with `MIPI_DATA_EN` set.
    isp_bypass: bool,
    isp_out_type: u8,
    demosaic: bool,
    rgb2yuv: bool,
    yuv2rgb: bool,
}

impl Plan {
    fn new(
        width: u32,
        height: u32,
        format: PixelFormat,
        sensor_format: PixelFormat,
        lanes: u8,
        bitrate_bps: u32,
        block_words: Option<u32>,
    ) -> Result<Self, CsiError> {
        if width == 0 || height == 0 || width > 4095 || height > 4095 {
            return Err(CsiError::BadGeometry { width, height });
        }
        if lanes == 0 || lanes > 2 {
            return Err(CsiError::BadLaneCount(lanes));
        }
        let mbps = bitrate_bps / 1_000_000;
        let hs_freq_sel = hs_freq_sel_for_mbps(mbps).ok_or(CsiError::BadBitrate { mbps })?;

        // RAW in, same RAW out => bypass, exactly as the working reference does.
        let isp_bypass = sensor_format == format;

        // The ISP's output *formatter* has no RAW10 setting: `isp_ll_set_output_data_color_format()`
        // accepts only RAW8 among the RAW formats (0 = RAW8, 1 = YUV422, 2 = RGB888, 3 = YUV420,
        // 4 = RGB565). That is a statement about the formatter, and it is why `isp_output_config`
        // returns None for RAW10 -- correctly, for the *converting* path.
        //
        // In bypass there is no conversion to ask for. `ISP_EN` is clear, so the pipeline does not
        // run and the bytes the ISP's input module passes through are the sensor's own. The output
        // type is therefore the identity, and `0` (the only RAW code) is what the *measured* RAW8
        // bypass uses: the reference reads `ISP.CNTL = 0x00025001`, whose `ISP_OUT_TYPE [31:29]` is 0
        // with RAW8 on the wire.
        //
        // For RAW10 the same 0 is chosen **by analogy, not by measurement**: the RAW8 case shows the
        // field is written to the RAW code, and with `ISP_EN = 0` it is not driving a conversion
        // either way. The read-back at capture time is what tests it, and if the ISP turns out to
        // inspect this field while bypassed, the symptom would be a wrong byte count or a
        // `DATA_TYPE_SETTING_ERR`, not a silent truncation.
        let (isp_out_type, demosaic, rgb2yuv, yuv2rgb) = if isp_bypass {
            (0u8, false, false, false)
        } else {
            isp_output_config(format).ok_or(CsiError::UnsupportedFormat(format))?
        };

        // The ISP input declaration must match what the sensor actually puts on the wire.
        let isp_data_type =
            isp_data_type_for(sensor_format).ok_or(CsiError::UnsupportedFormat(sensor_format))?;

        // §7.1: the block transfer size is expressed in 64-bit words, and the ISP
        // path requires the frame to be a whole number of them. The size is
        // derived from the ISP *output* format — the bytes the ISP actually
        // writes into the bridge window — rather than from ESP-IDF's
        // `h_res * v_res * in_bpp / 64` echo of the controller configuration,
        // which is what makes its `received_size` disagree with the buffer
        // (§4.3/§8.3).
        // Checked arithmetic: a multiplication that overflows here would produce a
        // small `frame_len` for a huge geometry and quietly pass the capacity guard
        // in the constructors.
        let frame_bits = (width as u64)
            .checked_mul(height as u64)
            .and_then(|px| px.checked_mul(out_bits_per_pixel(format) as u64))
            .ok_or(CsiError::BadGeometry { width, height })?;
        let frame_len = (frame_bits / 8) as usize;
        if frame_bits % 64 != 0 || frame_len % CACHE_LINE != 0 {
            return Err(CsiError::Not64BitAligned { bits: frame_bits });
        }
        // A bring-up block-length cap: program `BLOCK_TS` for fewer words than the
        // frame while leaving the bridge and ISP geometry at the full frame, so
        // the source keeps feeding after the block length has been reached. That
        // is the only configuration in which "does the engine stop at the
        // programmed count?" is a question with an answer; see the hydra-x
        // notes' block-length-cap scenario.
        let (frame_len, transfer_words) = match block_words {
            None => (frame_len, (frame_len / 8) as u32),
            Some(requested) => {
                let available = (frame_len / 8) as u32;
                if requested == 0 || requested > available {
                    return Err(CsiError::BadBlockWords {
                        requested,
                        available,
                    });
                }
                let len = requested as usize * 8;
                if len % CACHE_LINE != 0 {
                    return Err(CsiError::BadBlockWords {
                        requested,
                        available,
                    });
                }
                (len, requested)
            }
        };
        // The CSI-side geometry fields and the ISP's 12-bit HADR/VADR fields.
        debug_assert!(width <= 4095 && height <= 4095);

        Ok(Self {
            width,
            height,
            format,
            frame_len,
            transfer_words,
            block_ts: ov5647_core::dma_bounds::block_ts_from_words(transfer_words).map_err(
                |e| match e {
                    ov5647_core::dma_bounds::BlockTsError::ZeroWords => CsiError::BadBlockWords {
                        requested: 0,
                        available: transfer_words,
                    },
                    ov5647_core::dma_bounds::BlockTsError::TooManyWords { words, field_max } => {
                        CsiError::BlockTsOutOfRange { words, field_max }
                    }
                },
            )?,
            hs_freq_sel,
            lanes,
            isp_data_type,
            isp_in_bits: in_bits_per_pixel(sensor_format),
            isp_bypass,
            isp_out_type,
            demosaic,
            rgb2yuv,
            yuv2rgb,
        })
    }
}

impl CsiCapture<'static> {
    /// Brings up LDO, clocks, D-PHY, the CSI host, the bridge, the ISP and the
    /// DW-GDMA channel for a `width × height` frame in `format`, with the frame
    /// written into the internal `.bss` static bounded by [`FRAME_BUFFER_LIMIT`].
    ///
    /// `format` is the format the ISP writes to memory (RAW8, RGB565 or RGB888);
    /// the ISP input is RAW10, which is the OV5647's GBRG MIPI mode (§6.5). The
    /// sensor must be configured to stream RAW10 before the caller streams it.
    ///
    /// `lanes` is the MIPI data-lane count (1 or 2; the OV5647 module uses 2) and
    /// `bitrate_bps` the **per-lane** line rate in bit/s, at least 90 Mbps and at
    /// most 1500 Mbps (§3.3/§3.4).
    ///
    /// This keeps the static destination for the existing bring-up path, and it
    /// still refuses every real geometry — a static large enough for one has
    /// nowhere legal to live on pre-v3 silicon (see [`FRAME_BUFFER_LIMIT`]). For
    /// a destination that can actually hold a frame, use
    /// [`CsiCapture::new_with_buffer`].
    pub fn new(
        width: u32,
        height: u32,
        format: PixelFormat,
        // The sensor's wire format, which is what `ISP.CNTL.ISP_DATA_TYPE` must declare.
        sensor_format: PixelFormat,
        lanes: u8,
        bitrate_bps: u32,
    ) -> Result<Self, CsiError> {
        let plan = Plan::new(
            width,
            height,
            format,
            sensor_format,
            lanes,
            bitrate_bps,
            None,
        )?;

        // Refuse rather than truncate.
        //
        // This used to clamp the transfer to the buffer capacity, as a
        // "partial frame, enough to prove the path" shortcut. That was wrong and
        // it is exactly the state the hardware punished: the engine has no reason
        // to treat the buffer end as a limit, so the destination advanced past
        // it -- `DAR0` reached 0x4FFC0200, which is past the end of physical L2MEM
        // (0x4FF00000..0x4FFC0000) -- and the transfer died with DST_DEC_ERR.
        //
        // The reference driver checks `buflen >= fb_size_in_bytes` before it will
        // receive a frame. Until a verified partial-capture mechanism exists, the
        // only safe answer for a frame that does not fit is to refuse to arm.
        // No OV5647 mode in the verified tables fits today: the smallest,
        // 800x640 RAW8, needs 512_000 B against a ~450 KiB RAM region.
        // Guard against the *actual allocation*, not a separate constant: if the
        // frame buffer's size ever changes, this follows it automatically instead
        // of silently disagreeing with it.
        let capacity = unsafe { core::mem::size_of_val(&*FRAME_BUFFER.0.get()) };
        if plan.frame_len > capacity {
            return Err(CsiError::BufferTooSmall {
                needed: plan.frame_len,
                capacity,
            });
        }
        // Every byte the channel is programmed to write must lie inside that
        // allocation. This is the invariant whose absence let `DAR0` walk to
        // 0x4FFC0200, past the end of physical L2MEM. Both ends are built with
        // `checked_add`; a wrapped end would compare as inside the allocation.
        let base = FRAME_BUFFER.0.get() as usize;
        let fits = base
            .checked_add(plan.frame_len)
            .zip(base.checked_add(capacity))
            .map_or(false, |(end, allocation_end)| end <= allocation_end);
        if !fits {
            return Err(CsiError::BufferTooSmall {
                needed: plan.frame_len,
                capacity,
            });
        }

        let (host, bridge, isp, dma) = Self::bring_up(&plan, base)?;
        Ok(Self::assemble(
            plan,
            host,
            bridge,
            isp,
            dma,
            Destination::Static,
        ))
    }

    /// [`Self::new`] with an explicit cap on the transfer length, in 64-bit
    /// words — the same bring-up instrument as [`Self::new_with_buffer_capped`],
    /// but over the internal `.bss` static so it needs no caller-owned buffer.
    ///
    /// The bridge's `FRAME_CFG` and the ISP's `HADR_NUM`/`VADR_NUM` stay at the
    /// **full frame**; only `BLOCK_TS` shrinks. That is what makes it usable as a
    /// configuration audit for a geometry that cannot fit the static at all:
    /// 800x640 RAW8 needs 512,000 B against a 327,680 B static, so `new` refuses
    /// before any register is written and the ISP could never be read back.
    ///
    /// It is still an *armed* transfer if `start()` is called, so it does not by
    /// itself make capture safe — the caller must not arm it.
    pub fn new_static_capped(
        width: u32,
        height: u32,
        format: PixelFormat,
        sensor_format: PixelFormat,
        lanes: u8,
        bitrate_bps: u32,
        block_words: u32,
    ) -> Result<Self, CsiError> {
        let plan = Plan::new(
            width,
            height,
            format,
            sensor_format,
            lanes,
            bitrate_bps,
            Some(block_words),
        )?;
        let capacity = FRAME_BUFFER_LIMIT;
        if plan.frame_len > capacity {
            return Err(CsiError::BufferTooSmall {
                needed: plan.frame_len,
                capacity,
            });
        }
        let base = FRAME_BUFFER.0.get() as usize;
        let (host, bridge, isp, dma) = Self::bring_up(&plan, base)?;
        Ok(Self::assemble(
            plan,
            host,
            bridge,
            isp,
            dma,
            Destination::Static,
        ))
    }
}

impl<'a> CsiCapture<'a> {
    /// [`CsiCapture::new`] with a destination the **caller** owns.
    ///
    /// Same validation (`width`/`height`/`format`/`lanes`/`bitrate_bps`, the
    /// 64-bit-word and cache-line alignment of the frame, and the §7.1 transfer
    /// size) and the same §2/§3/§4/§5/§6/§7 bring-up — only the destination
    /// differs, so the two cannot configure the pipeline differently.
    ///
    /// `dest` has already been validated by [`DmaFrameBuffer::new`] (alignment,
    /// DMA-reachable region, no overlap with the descriptor). Here the frame is
    /// checked against its capacity: `frame_len <= dest.capacity()`, refused with
    /// [`CsiError::BufferTooSmall`] otherwise, exactly as the reference driver
    /// checks `buflen >= fb_size_in_bytes` before receiving.
    ///
    /// `dest` is held for the whole transfer, so the CPU cannot alias it: see the
    /// ownership contract on [`DmaFrameBuffer`].
    pub fn new_with_buffer(
        width: u32,
        height: u32,
        format: PixelFormat,
        // The sensor's wire format, which is what `ISP.CNTL.ISP_DATA_TYPE` must declare.
        sensor_format: PixelFormat,
        lanes: u8,
        bitrate_bps: u32,
        dest: DmaFrameBuffer<'a>,
    ) -> Result<Self, CsiError> {
        Self::new_with_buffer_capped(
            width,
            height,
            format,
            sensor_format,
            lanes,
            bitrate_bps,
            dest,
            None,
        )
    }

    /// [`Self::new_with_buffer`] with an explicit cap on the transfer length, in
    /// 64-bit words — a **bring-up instrument**, not a feature.
    ///
    /// With `block_words = Some(w)` the DW-GDMA's `BLOCK_TS` is programmed for `w`
    /// words while the CSI bridge's `FRAME_CFG`, the ISP's `HADR_NUM`/`VADR_NUM`
    /// and the sensor mode all stay at the full frame. The source therefore keeps
    /// feeding *after* the programmed count has been reached, which is the only
    /// configuration in which "does this engine stop at `BLOCK_TS`?" is a
    /// question with an answer: with a full-length block the two candidate
    /// mechanisms (the count, and the source's own end-of-frame) are identical and
    /// the experiment is void.
    ///
    /// [`Self::frame_len`] and [`Self::buffer`] then describe the **capped**
    /// prefix, so the frame-completeness verdict, the cache sync and the caller's
    /// content analysis all refer to the bytes actually programmed — and the
    /// caller's sentinels, which it placed from its own layout, are the evidence
    /// about anything written past that prefix.
    ///
    /// A `w` that is zero, greater than the frame's word count, or not a whole
    /// number of cache lines is refused with [`CsiError::BadBlockWords`]; a silent
    /// clamp would produce a green report for a transfer that never ran.
    pub fn new_with_buffer_capped(
        width: u32,
        height: u32,
        format: PixelFormat,
        // The sensor's wire format, which is what `ISP.CNTL.ISP_DATA_TYPE` must declare.
        sensor_format: PixelFormat,
        lanes: u8,
        bitrate_bps: u32,
        dest: DmaFrameBuffer<'a>,
        block_words: Option<u32>,
    ) -> Result<Self, CsiError> {
        let plan = Plan::new(
            width,
            height,
            format,
            sensor_format,
            lanes,
            bitrate_bps,
            block_words,
        )?;
        if plan.frame_len > dest.capacity() {
            return Err(CsiError::BufferTooSmall {
                needed: plan.frame_len,
                capacity: dest.capacity(),
            });
        }

        let (host, bridge, isp, dma) = Self::bring_up(&plan, dest.dma_addr())?;
        Ok(Self::assemble(
            plan,
            host,
            bridge,
            isp,
            dma,
            Destination::Caller(dest),
        ))
    }

    /// Steps 1–10 of the module documentation: everything the constructors do
    /// after the destination has been validated.
    ///
    /// `dest_addr` is the address written into the link item's `dar` (`start()`
    /// rewrites it before every arm; `configure_dma_channel` writes it here so the
    /// item is never left pointing at a different buffer).
    fn bring_up(
        plan: &Plan,
        dest_addr: usize,
    ) -> Result<(MIPI_CSI_HOST, MIPI_CSI_BRIDGE, ISP, DMA), CsiError> {
        let width = plan.width;
        let height = plan.height;

        let clkrst = unsafe { HP_SYS_CLKRST::steal() };
        let host = unsafe { MIPI_CSI_HOST::steal() };
        let bridge = unsafe { MIPI_CSI_BRIDGE::steal() };
        let isp = unsafe { ISP::steal() };
        let dma = unsafe { DMA::steal() };
        let delay = Delay::new();

        // 1. MIPI PHY rail: LDO channel 3 (VO3 = LDO unit 2 = EXT_LDO_P0_0P2A) at
        //    2500 mV. This comment used to say the dref/mul set-point was "NOT
        //    programmed" -- that was the bug, not the design: the rail sat at
        //    ~1108 mV and the D-PHY received nothing. `enable_phy_ldo()` now
        //    programs dref=9/mul=6, and `start()` refuses to arm unless they
        //    still read back, because PSRAM init clears the rail.
        unsafe { Self::enable_phy_ldo() };
        delay.delay(LDO_STABLE_DELAY);

        // 2..5. Clocks and resets, in the §2.3 order.
        critical_section::with(|_| {
            Self::enable_csi_host_clock(&clkrst);
            Self::enable_dphy_clock(&clkrst);
            Self::enable_bridge_clock(&clkrst, &bridge);
            Self::enable_isp_clock(&clkrst);
        });

        // 6. DW-GDMA bus clock/reset, then the controller reset handshake.
        crate::gdma::reset_dma_controller(&clkrst, &dma)?;

        // 7. D-PHY init + host-side lane/VC/scrambler config (§3.3/§4.2).
        Self::init_dphy(&host, plan.lanes, plan.hs_freq_sel);

        // 8. Bridge routing and frame geometry (§5.3).
        Self::configure_bridge(&bridge, width, height);

        // 9. ISP, in the §6.3 order, with §6.5's mandatory blocks and §8.2's
        //    explicit demosaic enable. The ISP must be up before the CSI
        //    controller starts (§8.1).
        Self::configure_isp(
            &isp,
            width,
            height,
            plan.isp_data_type,
            plan.isp_in_bits,
            plan.isp_bypass,
            plan.isp_out_type,
            plan.demosaic,
            plan.rgb2yuv,
            plan.yuv2rgb,
        );

        // 10. DW-GDMA channel: flow control, then the single 64-byte link item.
        crate::gdma::configure_dma_channel(&dma, plan.block_ts, dest_addr);

        Ok((host, bridge, isp, dma))
    }

    /// Packs the validated plan and the brought-up register blocks into a
    /// `CsiCapture` for `dest`.
    fn assemble(
        plan: Plan,
        host: MIPI_CSI_HOST,
        bridge: MIPI_CSI_BRIDGE,
        isp: ISP,
        dma: DMA,
        dest: Destination<'a>,
    ) -> Self {
        Self {
            host,
            bridge,
            isp,
            dma,
            width: plan.width,
            height: plan.height,
            format: plan.format,
            frame_len: plan.frame_len,
            transfer_words: plan.transfer_words,
            phy_pll_value: plan.hs_freq_sel << 1,
            lanes: plan.lanes,
            armed: false,
            armed_at: Instant::now(),
            trace: TraceBuf::new(),
            last_max_poll_gap_us: 0,
            last_cache_sync_us: 0,
            host_err_or: Cell::new(0),
            dest,
        }
    }

    /// Appends one [`TraceSample`] if the trace still has room. The trace is written while the
    /// bridge streams and read only after `stop()`; printing inside the sweep moves the timing.
    pub fn record_trace(&mut self) {
        if self.trace.len >= TRACE_CAP {
            return;
        }
        let ch = self.dma.ch1();
        let us = self.armed_at.elapsed().as_micros() as u32;
        self.trace.samples[self.trace.len] = TraceSample {
            us,
            intstatus0: ch.intstatus0().read().bits(),
            words: ch.status0().read().ch1_cmpltd_blk_tfr_size().bits(),
            cfg0: ch.cfg0().read().bits(),
            bridge_int_raw: self.bridge.int_raw().read().bits(),
            bridge_buf_flow: self.bridge.buf_flow_ctl().read().bits(),
            bridge_dma_req: self.bridge.dma_req_cfg().read().bits(),
            ch_en: self.dma.chen0().read().ch1_en().bit_is_set(),
        };
        self.trace.len += 1;
    }

    /// Read-only access to the trace so the caller can print it after `stop()`.
    pub fn trace(&self) -> &TraceBuf {
        &self.trace
    }

    /// Arms the DW-GDMA and enables the CSI bridge, in that order (§5.6).
    ///
    /// After this returns the caller may stream the sensor (§8.11). Calling it
    /// while a capture is already armed re-arms the channel, which is allowed —
    /// the DMA is finished or stopped by then in every supported flow.
    pub fn start(&mut self) -> Result<(), CsiError> {
        // Refuse to arm while a transfer is already armed.
        //
        // `start()` rewrites SAR/DAR/BLOCK_TS and sets `CHEN`; doing that to a channel that is mid
        // transfer would reprogram the registers under a running engine. The state machine this
        // driver implements — and the one the reference implements in
        // `csi_dma_trans_done_callback` — is arm once per transaction, only after the previous one
        // has completed, which `wait_frame` signals by calling `disarm()` (and thus clearing
        // `armed`). A caller that wants a second frame must let `wait_frame` return first. This was
        // found as a real gap: `start()` had no such guard, so a second call while armed would have
        // silently corrupted the in-flight transaction.
        if self.armed {
            return Err(CsiError::AlreadyArmed);
        }

        // Refuse to arm on a dead or mis-set D-PHY rail (see the error variant's
        // docs: PSRAM init clears `xpd` and resets dref/mul).
        {
            let pmu = unsafe { PMU::steal() };
            let ana = pmu.ext_ldo_p0_0p2a_ana().read();
            let ldo = pmu.ext_ldo_p0_0p2a().read();
            let xpd = ldo._0p2a_xpd_0().bit_is_set();
            let dref = ana.ana_0p2a_dref_0().bits();
            let mul = ana.ana_0p2a_mul_0().bits();
            if !xpd || dref != LDO_DREF_2500MV || mul != LDO_MUL_2500MV {
                return Err(CsiError::PhyRailNotConfigured { xpd, dref, mul });
            }
        }

        let dma = &self.dma;
        let ch = dma.ch1();
        let buffer = self.buffer_ptr() as u32;
        let words = self.transfer_words;
        // The whole allocation, not just the frame: a sentinel in the guard
        // granule is written by the caller before arming, so it has to reach
        // memory or the post-transfer read would report a "bound failure" that
        // is really a lost cache line.
        let sync_len = self.dest.capacity();

        // A new transfer, a new error history: any host error bit still latched
        // is from the previous one.
        self.host_err_or.set(0);

        // Point the channel at the frame buffer for this transfer. The channel is
        // contiguous, so these are the transfer's own registers and not a link
        // item's fields: `SAR` and the control words were set in
        // `configure_dma_channel`, and only the destination and the count change
        // here (`block_ts` is a count, so it is size - 1).
        ch.dar0().write(|w| unsafe { w.ch1_dar0().bits(buffer) });
        ch.dar1().write(|w| unsafe { w.ch1_dar1().bits(0) });
        ch.block_ts0()
            .write(|w| unsafe { w.ch1_block_ts().bits(words - 1) });

        // §7.5: everything the CPU wrote into the frame buffer must reach memory
        // before the DMA writes it.
        unsafe {
            if !cache_sync(buffer as usize, sync_len, CacheDir::C2M) {
                return Err(CsiError::CacheSyncFailed);
            }
        }

        // Clear any latched channel status from the previous transfer so the
        // completion poll in `wait_frame()` cannot see a stale bit.
        ch.intclear0().write(|w| unsafe { w.bits(u32::MAX) });

        // And the bridge's latched events, for the same reason but more so: `INT_RAW` is a
        // latch-and-`INT_CLR` pair, so a `DISCARD` left set by an earlier cycle would be read as
        // this cycle's. Without this the flag cannot say *when* it arose, which is exactly the
        // question it has to answer.
        self.bridge.int_clr().write(|w| unsafe { w.bits(u32::MAX) });

        // Arm the channel (the controller itself was enabled in `new()`). The arm moment is the
        // `CHEN` write, so time is taken **before** it, not after; the poll's event times are
        // measured from this instant.
        self.armed_at = Instant::now();
        dma.chen0()
            .modify(|_, w| w.ch1_en().set_bit().ch1_en_we().set_bit());
        self.armed = true;

        // Then the bridge, which starts feeding the buffer window.
        // (CSIBRG.CSI_EN.CSI_BRIG_EN; on rev < 3.0 the bridge's colour block is
        // absent, so nothing else in the bridge has to be enabled — §8.1/§8.4.)
        self.bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_en().set_bit());

        // NOTE: `ISP.DMA_CNTL.DMA_EN` / `ISP.DMA_RAW_DATA` are deliberately NOT
        // written here. Arming them was tried and made the DW-GDMA channel fail
        // with `dma_channel_int = 0x40000048` and self-disable, while
        // `ISP_BUF_FULL` persisted. The bring-up notes describe that path as the
        // ISP **DMA-input** path (`isp_dma_configure_input()`, §6.3 step 20,
        // "DMA-input path only"), and this design feeds the ISP from the CSI
        // bridge, so arming it perturbs the wrong block. That function lives in
        // `components/esp_driver_isp/src/isp_dma.c`, which exists at ESP-IDF
        // `d445e947156f276dfe0db5804e4edf6ae90fe6fb` but **not** in v5.5.0, so the
        // citation only resolves against that pinned commit. What is established
        // about the ISP output path is only that the ISP fills its buffer and
        // raises `ISP_BUF_FULL`; which block is supposed to drain it is still open.
        Ok(())
    }

    /// Disables the bridge and then the DMA channel (§5.6 stop order).
    ///
    /// A capture that [`Self::wait_frame`] has already unwound is left as it is;
    /// calling `stop()` twice is harmless.
    pub fn stop(&mut self) {
        self.disarm();
    }

    /// §5.6's stop order, used both by [`Self::stop`] and at the end of every
    /// `wait_frame()`: the bridge stops feeding the buffer and the channel is
    /// disabled, so nothing can write the frame buffer while the caller reads it
    /// (§8.8). `start()` re-enables both.
    fn disarm(&mut self) {
        self.bridge
            .csi_en()
            .modify(|_, w| w.csi_brig_en().clear_bit());
        self.dma
            .chen0()
            .modify(|_, w| w.ch1_en().clear_bit().ch1_en_we().set_bit());
        self.armed = false;
    }

    /// Waits for the armed channel to deliver one frame, up to `timeout`.
    ///
    /// On success the frame buffer is invalidated from the cache (§7.5), the
    /// bridge and channel are unwound (§5.6) so the buffer cannot be overwritten
    /// while it is read, and the returned [`FrameInfo`] describes what is behind
    /// [`Self::buffer`].
    ///
    /// The sensor keeps streaming while the pipeline is unwound, so a `start()`
    /// issued long after the previous frame may catch the bridge mid-frame; the
    /// first frame of a fresh capture is expected to be suspect (§8.12 item 9).
    pub fn wait_frame(&mut self, timeout: Duration) -> Result<FrameInfo, CsiError> {
        if !self.armed {
            return Err(CsiError::NotStarted);
        }
        // The wait window is measured from *this call*, not from `start()`.
        // Measuring from `armed_at` made the timeout check true on the first
        // iteration whenever the caller did any work between `start()` and here
        // (logging, a status dump, a lane poll), so `wait_frame` returned
        // `Timeout` instantly without ever waiting -- and the caller saw
        // "no frame in 0 ms" beside a `waited` of several seconds.
        let started = Instant::now();
        let delay = Delay::new();
        let mut last_poll_at = started;
        self.last_max_poll_gap_us = 0;

        // Event times, relative to the arm. The reference does not just configure differently -- it
        // re-arms the channel after every completed transaction while the bridge keeps running, so
        // the question "when did the data arrive versus when did the DMA take it" has to be answered
        // before any re-arm work means anything. These six are the ones that discriminate:
        //
        //   T3 bridge depth first non-zero  -> the bridge began holding data
        //   T4 first DISCARD                -> it began dropping it
        //   T5 DMA tfr_done                 -> the engine finished (returned as `FrameInfo::elapsed`)
        //
        // T0/T1/T2 (channel armed, bridge enabled, sensor stream-on) happen in `start()` and the
        // caller's `start_stream()`; the deltas below are all measured from T0.
        let mut first_depth: Option<Duration> = None;
        let mut first_discard: Option<Duration> = None;

        loop {
            // Re-fetched each iteration instead of held in a local: the failure
            // paths below take `&mut self` to sample and unwind, which a live
            // borrow of `self.dma` would forbid.
            let st = self.dma.ch1().intstatus0().read();
            let status = self.status();
            // Measured from **`started`**, i.e. from entering this wait, not from `self.armed_at`.
            // The arm is a long way back: `start()` is followed by the sensor's stream-on, the
            // AEC/AGC settle and several `step!` dumps, so an arm-relative figure reads in whole
            // seconds and says nothing about pacing. The three events are also recorded on the poll's
            // own cadence, which is one `status()` plus a 50 us delay -- sub-100 us, against a 20 ms
            // frame.
            // Measured from the arm (`self.armed_at`), which is now the instant of the `CHEN` write.
            // The wait may start later (stream-on, optional settle), so these are the *first
            // observations* of each state, and `wait_started` below is printed beside them so the
            // detection lag is visible rather than hidden.
            if first_depth.is_none() && status.bridge_buf_depth != 0 {
                first_depth = Some(self.armed_at.elapsed());
            }
            if first_discard.is_none() && status.bridge_int_raw & BRIDGE_ERR_DISCARD != 0 {
                first_discard = Some(self.armed_at.elapsed());
            }

            if status.dma_error() {
                // Sample before unwinding: `disarm()` is what resets the
                // registers the probe is after.
                return Err(CsiError::DmaError(self.disarm_with_probe(status)));
            }
            if st.ch1_block_tfr_done_intstat().bit_is_set()
                || st.ch1_dma_tfr_done_intstat().bit_is_set()
            {
                break;
            }
            if started.elapsed() >= timeout {
                let waited = started.elapsed();
                return Err(CsiError::Timeout {
                    waited,
                    status: self.disarm_with_probe(status),
                });
            }
            let now = Instant::now();
            let gap = now - last_poll_at;
            if gap > Duration::from_micros(self.last_max_poll_gap_us as u64) {
                self.last_max_poll_gap_us = gap.as_micros() as u32;
            }
            last_poll_at = now;
            delay.delay_micros(50);
        }

        // `CH1_STATUS0.CH1_CMPLTD_BLK_TFR_SIZE` is in the **`BLOCK_TS` convention**: it reports the
        // value that was programmed, and `BLOCK_TS` holds `count - 1`. Measured twice on this
        // board: with `BLOCK_TS = 511` the engine moved 512 words (the hydra-x PSRAM-reachability
        // scenario's 4,096-byte destination body compare passed), and with `BLOCK_TS = 63,999` for
        // the 512,000-byte RAW8 frame it read back 63,999. The reference's own register for that
        // same frame is `0x0000f9ff = 63,999`, i.e. `512000/8 - 1`. So `+ 1` here converts to a
        // true word count, which is what `expected_words` (`transfer_words`) is in.
        //
        // Without this the comparison read `63999/64000`, a whole-frame transfer was reported as
        // `PARTIAL/ERRORED`, and `FrameInfo::complete()` was false for a transfer whose sentinels
        // both said it had written exactly the frame.
        let hw_words = self
            .dma
            .ch1()
            .status0()
            .read()
            .ch1_cmpltd_blk_tfr_size()
            .bits();
        let words = hw_words.wrapping_add(1);
        self.disarm();

        // §7.5: the DMA wrote the buffer; make the CPU see it. The whole
        // allocation, not just the frame, so the guard/escape sentinels past
        // `frame_len` are invalidated before the caller reads them
        // (`psram-frame-bound.md` §6.1).
        let sync_started = Instant::now();
        let sync_ok = unsafe {
            cache_sync(
                self.buffer_ptr() as usize,
                self.dest.capacity(),
                CacheDir::M2C,
            )
        };
        self.last_cache_sync_us = sync_started.elapsed().as_micros() as u32;
        if !sync_ok {
            return Err(CsiError::CacheSyncFailed);
        }

        Ok(FrameInfo {
            width: self.width,
            height: self.height,
            format: self.format,
            len: self.frame_len,
            words,
            expected_words: self.transfer_words,
            elapsed: self.armed_at.elapsed(),
            bridge_first_depth: first_depth,
            bridge_first_discard: first_discard,
            wait_started_from_arm: started.elapsed(),
        })
    }

    /// The frame buffer. Valid from the moment `wait_frame()` returned until the
    /// next [`Self::start`] (see the ownership contract in the module docs).
    ///
    /// Works for both constructors: under [`Self::new_with_buffer`] these are the
    /// caller's own bytes, still owned by this `CsiCapture` for as long as `'a`.
    /// Samples the DMA/bridge at fixed offsets from the arm, then waits for the frame. This captures
    /// the first ~100 ms after the arm without relying on `wait_frame`'s poll, which starts after
    /// stream-on and may miss an early completion. Samples are plain register reads; nothing prints
    /// during the sweep.
    pub fn wait_and_trace(&mut self, timeout: Duration) -> Result<FrameInfo, CsiError> {
        for &at_us in &[0u32, 100, 500, 1000, 5000, 10000, 20000, 40000, 100000] {
            let target = at_us as u64;
            while (self.armed_at.elapsed().as_micros() as u64) < target {
                // busy-wait; deliberately no delay call so the pacing is exact
            }
            self.record_trace();
        }
        self.wait_frame(timeout)
    }

    /// The whole destination allocation for writing, **only while no transfer is armed**.
    ///
    /// `DmaFrameBuffer`'s contract is that the destination is held for the whole transfer so the CPU
    /// cannot alias DMA-owned memory, which is why [`Self::buffer`] is a shared slice. Between
    /// transactions that contract does not apply: `wait_frame` calls `disarm()` before it returns and
    /// `disarm()` clears `armed`, and `start()` is what arms again. So this returns `Some` exactly in
    /// that window and `None` at any other time, which is the state a caller needs to prime sentinels
    /// and a poison pattern for the *next* transaction without creating a second `&mut` over memory
    /// the engine might be writing.
    pub fn destination_mut(&mut self) -> Option<&mut [u8]> {
        if self.armed {
            return None;
        }
        let len = self.dest.capacity();
        // SAFETY: `armed` is false, so the engine is disabled (both `disarm()` and `stop()` clear
        // `CHEN` and the bridge's enable before clearing `armed`), and the caller holds `&mut self`,
        // so no other reference to the destination exists.
        Some(unsafe { core::slice::from_raw_parts_mut(self.buffer_ptr(), len) })
    }

    /// The frame that `wait_frame` has delivered: exactly `frame_len` bytes of
    /// ISP output, valid for reads once `armed` is false.
    pub fn buffer(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.buffer_ptr(), self.frame_len) }
    }

    /// The **whole** destination allocation, frame plus the guard/sentinel
    /// granules beyond it (`DmaFrameBuffer::capacity()`, or the internal static's
    /// size for [`Self::new`]).
    ///
    /// Valid under the same ownership rule as [`Self::buffer`]. This is what a
    /// caller reads its sentinels from: `[frame_len, capacity)` is exactly the
    /// region a bound failure would damage, and it is outside the frame the
    /// header describes, so no announced payload ever contains it.
    pub fn capacity_slice(&self) -> &[u8] {
        let len = self.dest.capacity();
        unsafe { core::slice::from_raw_parts(self.buffer_ptr(), len) }
    }

    /// Bytes of the destination allocation, i.e. `capacity_slice().len()`. Equal
    /// to [`Self::frame_len`] plus the guard/sentinel space.
    pub fn capacity(&self) -> usize {
        self.dest.capacity()
    }

    /// Base address the DW-GDMA writes to: the internal static, or the
    /// caller-owned [`DmaFrameBuffer`]'s address. Exposed for bring-up logs
    /// (`DAR0` is compared against it) and to let a caller confirm the slice it
    /// handed over is the one being written.
    pub fn dma_addr(&self) -> usize {
        self.dest.dma_addr()
    }

    /// Bytes of one frame in the buffer.
    pub fn frame_len(&self) -> usize {
        self.frame_len
    }

    /// Raw status of every block in the path.
    pub fn status(&self) -> StatusSnapshot {
        let ch = self.dma.ch1();
        StatusSnapshot {
            dma_int_status: self.dma.intstatus0().read().bits(),
            dma_channel_int: ch.intstatus0().read().bits(),
            dma_channel_status: ch.status0().read().bits(),
            dma_channel_enabled: self.dma.chen0().read().ch1_en().bit_is_set(),
            bridge_int_st: self.bridge.int_st().read().bits(),
            host_int_st_main: {
                // Destructive read (see `CsiCapture::host_err_or`): accumulate
                // instead of reporting only this sample.
                let latched = self.host_err_or.get() | self.host.int_st_main().read().bits();
                self.host_err_or.set(latched);
                latched
            },
            isp_int_raw: self.isp.int_raw().read().bits(),
            phy_stopstate: self.host.phy_stopstate().read().bits(),
            // Hardware's own answer to "did the DMA move anything": the count of
            // 64-bit words the channel has completed. Zero on a timeout means the
            // channel genuinely moved nothing; non-zero means it is moving data
            // but never reached the programmed count.
            dma_words_moved: self
                .dma
                .ch1()
                .status0()
                .read()
                .ch1_cmpltd_blk_tfr_size()
                .bits(),
            // Read-only: the bridge's DMA request configuration. Bit 17 has no
            // field on rev 1.x -- `hw_ver1/soc/mipi_csi_bridge_struct.h` reserves
            // bits 17-31; the `CSI_DMA_FLOW_CONTROLLER` name the PAC offers at
            // bit 17 is the rev-3.x field, default 1 -- so the raw word is kept
            // and nothing is decoded from it.
            bridge_dma_req_cfg: self.bridge.dma_req_cfg().read().bits(),
            bridge_buf_depth: u32::from(self.bridge.buf_flow_ctl().read().csi_buf_depth().bits()),
            bridge_buf_flow_ctl: self.bridge.buf_flow_ctl().read().bits(),
            bridge_data_type_cfg: self.bridge.data_type_cfg().read().bits(),
            bridge_dmablk_size: self.bridge.dmablk_size().read().bits(),
            bridge_host_ctrl: self.bridge.host_ctrl().read().bits(),
            bridge_csi_en: self.bridge.csi_en().read().csi_brig_en().bit_is_set() as u32,
            bridge_vadr_num_check: self.bridge.frame_cfg().read().vadr_num_check().bit_is_set(),
            bridge_has_hsync_e: self.bridge.frame_cfg().read().has_hsync_e().bit_is_set(),
            dma_sms: self.dma.ch1().ctl0().read().ch1_sms().bit_is_set() as u32,
            dma_dms: self.dma.ch1().ctl0().read().ch1_dms().bit_is_set() as u32,
            dma_sinc: self.dma.ch1().ctl0().read().ch1_sinc().bit_is_set() as u32,
            dma_dinc: self.dma.ch1().ctl0().read().ch1_dinc().bit_is_set() as u32,
            dma_sar0: self.dma.ch1().sar0().read().ch1_sar0().bits(),
            dma_dar0: self.dma.ch1().dar0().read().ch1_dar0().bits(),
            dma_block_ts: self.dma.ch1().block_ts0().read().ch1_block_ts().bits(),
            dma_cfg0: ch.cfg0().read().bits(),
            dma_llp0: ch.llp0().read().bits(),
            dma_llp1: ch.llp1().read().bits(),
            dma_ctl0: ch.ctl0().read().bits(),
            dma_ctl1: ch.ctl1().read().bits(),
            dma_common_int_status: self.dma.commonreg_intstatus0().read().bits(),
            // Only `wait_frame`'s failure paths fill this in; a plain snapshot
            // never samples, so it stays as cheap as it was.
            mid_transfer: None,
            bridge_int_raw: self.bridge.int_raw().read().bits(),
        }
    }

    /// The whole-frame verdict: did a **complete, uncorrupted** frame arrive, as
    /// opposed to "the DMA stopped"?
    ///
    /// Reads only sticky registers — `CH1_STATUS0`/`CH1_INTSTATUS0`, the latched
    /// `Self::host_err_or`, `MIPI_CSI_BRIDGE.INT_RAW` and `ISP.INT_RAW` — so it
    /// may be called after [`Self::wait_frame`] has unwound the channel and gives
    /// the same answer then as it would have given at the moment of completion.
    /// The bridge's `INT_RAW` and the ISP's `INT_RAW` are cleared only by
    /// `INT_CLR`, which this driver never writes, so both accumulate across the
    /// whole transfer.
    ///
    /// It does **not** look at memory. Two checks belong to the caller, because
    /// only the caller knows its layout: the sentinel granules inside
    /// [`Self::capacity_slice`], and the content verdict on [`Self::buffer`].
    /// See [`FrameCompleteness`].
    pub fn frame_completeness(&self) -> FrameCompleteness {
        let st = self.dma.ch1().intstatus0().read();
        let ch_int = st.bits();
        let bridge_int_raw = self.bridge.int_raw().read().bits();
        let isp_int_raw = self.isp.int_raw().read().bits();
        FrameCompleteness {
            // Same `BLOCK_TS`-convention conversion as in `wait_frame`; see the note there.
            words: self
                .dma
                .ch1()
                .status0()
                .read()
                .ch1_cmpltd_blk_tfr_size()
                .bits()
                .wrapping_add(1),
            expected_words: self.transfer_words,
            block_done: st.ch1_block_tfr_done_intstat().bit_is_set(),
            tfr_done: st.ch1_dma_tfr_done_intstat().bit_is_set(),
            dma_error: ch_int & 0x0000_3FE0 != 0,
            host_error_or: self.host_err_or.get(),
            bridge_errors: bridge_int_raw & BRIDGE_ERR_MASK,
            bridge_int_raw,
            isp_errors: isp_int_raw & ISP_LL_EVENT_ERROR_MASK,
            isp_int_raw,
        }
    }

    /// Samples `CH1_DAR0`, `CH1_SAR0`, `CH1_STATUS0`, `CH1_INTSTATUS0` and
    /// `DMA_CHEN0` while the channel is enabled.
    ///
    /// Pure reads. Nothing is written, the channel is neither disabled nor
    /// reconfigured, its interrupt flags are not cleared, and no transfer
    /// parameter is touched — so it is safe to run in the middle of a capture,
    /// and in particular inside [`Self::wait_frame`]'s polling loop.
    ///
    /// It exists to answer one question with hardware rather than inference:
    /// the destination-decode fault this project chased leaves `DAR0` reading
    /// `0x4FFC_0200` *after* the fault even though the frame buffer address was
    /// written. Reading it only after the fact cannot distinguish
    ///
    /// * the engine never loaded the written `DAR` (no sample ever shows the
    ///   frame buffer address, `first_changed_dar0` = `Some(0x4FFC0200)` and
    ///   a `0x4FFC0200` first sample), from
    /// * the engine loaded it and something replaced it mid-transfer (the
    ///   first sample is the frame buffer address, a later one is the anomaly).
    ///
    /// Both bounds ([`PROBE_MAX_SAMPLES`], [`PROBE_MAX_DURATION`]) are enforced
    /// independently, so the call cannot hang on any silicon revision.
    pub fn probe_mid_transfer(&self) -> MidTransferProbe {
        let ch = self.dma.ch1();
        let started = Instant::now();
        let delay = Delay::new();
        let initial_dar0 = ch.dar0().read().ch1_dar0().bits();
        let mut last = MidTransferSample {
            dar0: initial_dar0,
            sar0: ch.sar0().read().ch1_sar0().bits(),
            status0: ch.status0().read().bits(),
            intstatus0: ch.intstatus0().read().bits(),
            chen0: self.dma.chen0().read().bits(),
        };
        let mut first_changed_dar0 = None;
        let mut changed_at_sample = None;
        let mut samples = 0u32;

        while samples < PROBE_MAX_SAMPLES && started.elapsed() < PROBE_MAX_DURATION {
            last = MidTransferSample {
                dar0: ch.dar0().read().ch1_dar0().bits(),
                sar0: ch.sar0().read().ch1_sar0().bits(),
                status0: ch.status0().read().bits(),
                intstatus0: ch.intstatus0().read().bits(),
                chen0: self.dma.chen0().read().bits(),
            };
            if first_changed_dar0.is_none() && last.dar0 != initial_dar0 {
                first_changed_dar0 = Some(last.dar0);
                changed_at_sample = Some(samples);
            }
            samples += 1;
            delay.delay_micros(PROBE_INTERVAL_US);
        }

        MidTransferProbe {
            initial_dar0,
            first_changed_dar0,
            changed_at_sample,
            samples,
            last,
            elapsed: started.elapsed(),
        }
    }

    /// Failure-path unwind: samples the still-enabled channel, attaches the
    /// samples to `status`, then performs §5.6's stop order.
    ///
    /// The sample has to happen **before** [`Self::disarm`], because disabling
    /// the channel is exactly the event that resets the registers under study.
    /// Only [`Self::wait_frame`]'s error paths use this; the success path keeps
    /// calling [`Self::disarm`] directly and is unchanged.
    fn disarm_with_probe(&mut self, mut status: StatusSnapshot) -> StatusSnapshot {
        status.mid_transfer = Some(self.probe_mid_transfer());
        self.disarm();
        status
    }

    /// Samples `PHY_RX` and `PHY_STOPSTATE` once, cheaply enough to poll.
    ///
    /// Two MMIO reads, so a caller can take thousands of samples in a second.
    /// That matters because `CLKACTIVEHS` is not a latch: with the OV5647's
    /// non-continuous clock the high-speed clock only runs in short bursts, so a
    /// single spot read cannot establish that the lane never entered HS.
    pub fn sample_phy(&self) -> (u32, u32) {
        (
            self.host.phy_rx().read().bits(),
            self.host.phy_stopstate().read().bits(),
        )
    }

    /// Raw `MIPI_CSI_HOST` PHY control/status registers.
    ///
    /// These are what separate "the receiver is still held in reset" from "the
    /// receiver is up but the line is silent". `PHY_STOPSTATE` on its own cannot
    /// distinguish them, and reading it as if bit 0 were the clock lane is a trap
    /// (bit 0 is data lane 0; the clock lane is bit 16).
    pub fn phy_registers(&self) -> PhyRegisters {
        let host = &self.host;
        PhyRegisters {
            csi2_resetn: host.csi2_resetn().read().bits(),
            dphy_rstz: host.dphy_rstz().read().bits(),
            phy_shutdownz: host.phy_shutdownz().read().bits(),
            n_lanes: host.n_lanes().read().bits(),
            phy_cal: host.phy_cal().read().bits(),
            phy_rx: host.phy_rx().read().bits(),
            phy_stopstate: host.phy_stopstate().read().bits(),
        }
    }

    /// `ISP.CNTL` and `ISP.FRAME_CFG`, read raw so a bring-up log can decode them
    /// against the field layout in `isp_reg.h` (bit 0 `MIPI_DATA_EN`, bit 1
    /// `ISP_EN`; `VADR_NUM [11:0]`, `HADR_NUM [23:12]`, `BAYER_MODE [28:27]`).
    ///
    /// Reading offsets from notes instead of from that header is what produced a
    /// wrong "`MIPI_DATA_EN = 0`" reading earlier in this port, so the raw words
    /// are returned rather than any pre-decoded flags.
    pub fn isp_registers(&self) -> IspRegisters {
        let isp = &self.isp;
        IspRegisters {
            cntl: isp.cntl().read().bits(),
            frame_cfg: isp.frame_cfg().read().bits(),
        }
    }

    /// True when the D-PHY clock lane reports LP-11 (`PHY_STOPSTATECLK`) and no
    /// data lane is stuck out of stop state — the §4.2 bring-up check that the
    /// PHY and the sensor are talking before a frame is expected.
    pub fn phy_ready(&self) -> bool {
        let stop = self.host.phy_stopstate().read();
        let clock_lp11 = stop.phy_stopstateclk().bit_is_set();
        let data_ok = stop.phy_stopstatedata_0().bit_is_set()
            && (self.lanes < 2 || stop.phy_stopstatedata_1().bit_is_set());
        clock_lp11 && data_ok
    }

    /// The `hs_freq_sel` this plan selected from the requested bitrate, i.e. the
    /// D-PHY PLL range index. Exposed so a log can show which range the mode
    /// landed in without decoding `phy_pll_value`.
    pub fn hs_freq_sel(&self) -> u8 {
        self.phy_pll_value >> 1
    }

    /// The transfer length in 64-bit words this plan will program into
    /// `BLOCK_TS` when armed — the full frame (`frame_len / 8`) unless a
    /// `block_words` cap was requested.
    ///
    /// Read from the plan rather than from the register, because `BLOCK_TS` is
    /// only written by [`CsiCapture::start`]: a configuration audit that
    /// deliberately does not arm sees 0 there and would otherwise report a
    /// mismatch that is really just "not armed yet".
    pub fn transfer_words(&self) -> u32 {
        self.transfer_words
    }

    /// The value this driver writes into D-PHY register 0x44 for the requested
    /// bitrate: `hs_freq_sel << 1` (§3.3). Exposed so a bring-up log can show
    /// which PLL range was selected.
    pub fn phy_pll_value(&self) -> u8 {
        self.phy_pll_value
    }

    // -- internals ---------------------------------------------------------

    /// Pointer to the destination buffer: the internal static under
    /// [`CsiCapture::new`], the caller's slice under
    /// [`CsiCapture::new_with_buffer`].
    fn buffer_ptr(&self) -> *mut u8 {
        self.dest.buffer_ptr()
    }

    /// §3.1: LDO channel 3 (VO3) at 2500 mV. The sequence mirrors
    /// `esp_ldo_acquire_channel()` (§3.1.1); the set-point is derived the same
    /// way IDF derives it — exhaustive search over this part's eFuse
    /// calibration — and lands on [`LDO_DREF_2500MV`] / [`LDO_MUL_2500MV`],
    /// 2509.9 mV.
    ///
    /// Channel 3 maps to LDO unit 2, whose register pair is `ext_ldo[1]` =
    /// `EXT_LDO_P0_0P2A` / `EXT_LDO_P0_0P2A_ANA` (`ldo_ll.h`'s
    /// `index_array[4] = {0, 3, 1, 4}`, §3.1.2).
    unsafe fn enable_phy_ldo() {
        let pmu = unsafe { PMU::steal() };
        let ldo = pmu.ext_ldo_p0_0p2a();
        let ana = pmu.ext_ldo_p0_0p2a_ana();

        // Order mirrors ESP-IDF `esp_ldo_acquire_channel()`: current limit on,
        // set the voltage, take ownership, ripple suppression, enable, then
        // release the current limit.
        //
        // The voltage step was missing before, and that mattered. The rail was
        // left at the reset/eFuse default `dref=10, mul=0`, which Espressif's own
        // conversion (ldo_ll.h, release/v5.5) turns into **1108.5 mV** -- not the
        // 2500 mV the MIPI D-PHY needs. A D-PHY starved to ~1.1 V is exactly what
        // the measurements showed: high-speed reception dead while low-power
        // squelch still flickered.
        //
        // `dref`/`mul` are the exhaustive-search optimum for a 2500 mV request
        // under this part's eFuse calibration (K=40, VOS=37, C=5), computed with
        // the same algorithm:
        //   vref_20 = 20 + (dref - 9) * 2
        //   vout    = (vref_20 * K_1000 + 20 * Vos_1000) * (4000 + mul * C_1000) / 80000
        // with K_1000 = 1015, Vos_1000 = -8, C_1000 = 995 -> 2509.9 mV (error 9.9 mV).
        // The runner-up, dref=10 mul=5, gives 2487.2 mV.
        const DREF_2500MV: u8 = LDO_DREF_2500MV;
        const MUL_2500MV: u8 = LDO_MUL_2500MV;

        ana.modify(|_, w| w.ana_0p2a_en_cur_lim_0().set_bit());

        // Drop the regulator before re-trimming. `enable_phy_ldo()` runs on every
        // cycle against an LDO that may already be enabled, and a trim applied to
        // a live regulator is not guaranteed to latch -- which would leave
        // `dref`/`mul` reading back as the new values while the rail stayed put,
        // indistinguishable from "still broken". IDF programs the trim with the
        // regulator down and enables it afterwards.
        ldo.modify(|_, w| w._0p2a_xpd_0().clear_bit());

        // Voltage set-point, and the regulated output path (not the 3.3 V rail
        // bypass: `tieh = 0`, matching `use_rail_voltage == false`).
        ana.modify(|_, w| {
            w.ana_0p2a_dref_0()
                .bits(DREF_2500MV)
                .ana_0p2a_mul_0()
                .bits(MUL_2500MV)
        });
        ldo.modify(|_, w| {
            w._0p2a_force_tieh_sel_0()
                .set_bit()
                ._0p2a_tieh_sel_0()
                .bits(0)
                ._0p2a_tieh_0()
                .clear_bit()
        });

        ana.modify(|_, w| w.ana_0p2a_en_vdet_0().set_bit());
        ldo.modify(|_, w| w._0p2a_xpd_0().set_bit());
        ana.modify(|_, w| w.ana_0p2a_en_cur_lim_0().clear_bit());
    }

    /// §2.2a: CSI host bus clock, glitch-free (0 then 1), then reset pulse.
    fn enable_csi_host_clock(clkrst: &HP_SYS_CLKRST) {
        clkrst
            .soc_clk_ctrl1()
            .modify(|_, w| w.csi_host_sys_clk_en().clear_bit());
        clkrst
            .soc_clk_ctrl1()
            .modify(|_, w| w.csi_host_sys_clk_en().set_bit());
        clkrst
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_csi_host().set_bit());
        clkrst
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_csi_host().clear_bit());
    }

    /// §2.2b: D-PHY config clock source and enable (0 then 1).
    fn enable_dphy_clock(clkrst: &HP_SYS_CLKRST) {
        clkrst
            .peri_clk_ctrl03()
            .modify(|_, w| unsafe { w.mipi_csi_dphy_clk_src_sel().bits(PHY_CLK_SRC_PLL_F20M) });
        clkrst
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_csi_dphy_cfg_clk_en().clear_bit());
        clkrst
            .peri_clk_ctrl03()
            .modify(|_, w| w.mipi_csi_dphy_cfg_clk_en().set_bit());
    }

    /// §2.2c: bridge module clock, reset pulse, then `HOST_CTRL.CSI_ENABLECLK`.
    fn enable_bridge_clock(clkrst: &HP_SYS_CLKRST, bridge: &MIPI_CSI_BRIDGE) {
        clkrst
            .soc_clk_ctrl1()
            .modify(|_, w| w.csi_brg_sys_clk_en().set_bit());
        clkrst
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_csi_brg().set_bit());
        clkrst
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_csi_brg().clear_bit());
        bridge
            .host_ctrl()
            .modify(|_, w| w.csi_enableclk().set_bit());
    }

    /// §2.2d: ISP module clock, reset pulse, clock source and integer divider.
    fn enable_isp_clock(clkrst: &HP_SYS_CLKRST) {
        clkrst
            .peri_clk_ctrl25()
            .modify(|_, w| w.isp_clk_en().set_bit());
        clkrst.hp_rst_en0().modify(|_, w| w.rst_en_isp().set_bit());
        clkrst
            .hp_rst_en0()
            .modify(|_, w| w.rst_en_isp().clear_bit());
        clkrst
            .peri_clk_ctrl25()
            .modify(|_, w| unsafe { w.isp_clk_src_sel().bits(ISP_CLK_SRC_PLL240) });
        clkrst
            .peri_clk_ctrl26()
            .modify(|_, w| unsafe { w.isp_clk_div_num().bits(ISP_CLK_DIV_NUM) });
    }

    /// §3.3: the whole D-PHY init sequence, in the exact order ESP-IDF performs
    /// it, followed by the §4.2 host-side lane/VC/scrambler configuration.
    fn init_dphy(host: &MIPI_CSI_HOST, lanes: u8, hs_freq_sel: u8) {
        // 1. hold the PHY and the host side in reset / shutdown.
        host.phy_shutdownz()
            .modify(|_, w| w.phy_shutdownz().clear_bit());
        host.dphy_rstz().modify(|_, w| w.dphy_rstz().clear_bit());
        host.csi2_resetn()
            .modify(|_, w| w.csi2_resetn().clear_bit());
        // 2. clear the PHY test port: register address 0x0, then a TESTCLR pulse
        //    with the test clock low and released (mipi_csi_hal_init's step 1).
        Self::phy_write_reg_addr(host, 0x00);
        Self::phy_write_clock(host, 0, true);
        Self::phy_write_clock(host, 0, false);
        // 3. select the PLL range for the requested per-lane bitrate: the only
        //    D-PHY internal register ESP-IDF writes is 0x44 = hs_freq_sel << 1.
        Self::phy_write_reg(host, 0x44, hs_freq_sel << 1);
        // 4. release: shutdown input, D-PHY reset output, host reset output.
        host.phy_shutdownz()
            .modify(|_, w| w.phy_shutdownz().set_bit());
        host.dphy_rstz().modify(|_, w| w.dphy_rstz().set_bit());
        host.csi2_resetn().modify(|_, w| w.csi2_resetn().set_bit());
        // 5. host-side configuration: lane count, virtual-channel extension off
        //    (VCX = 1 means disabled), descrambling off.
        host.n_lanes()
            .modify(|_, w| unsafe { w.n_lanes().bits(lanes - 1) });
        host.vc_extension().modify(|_, w| w.vcx().set_bit());
        host.scrambling()
            .modify(|_, w| w.scramble_enable().clear_bit());
    }

    /// `mipi_csi_phy_ll_write_clock()`: `TEST_CTRL0 = clock_level << 1 | clear`.
    fn phy_write_clock(host: &MIPI_CSI_HOST, clock_level: u32, clear: bool) {
        host.phy_test_ctrl0().write(|w| {
            if clock_level != 0 {
                w.phy_testclk().set_bit()
            } else {
                w.phy_testclk().clear_bit()
            };
            if clear {
                w.phy_testclr().set_bit()
            } else {
                w.phy_testclr().clear_bit()
            }
        });
    }

    /// `mipi_csi_phy_ll_write_reg_addr()`: `TEST_CTRL1 = 1 << 16 | reg_addr`.
    fn phy_write_reg_addr(host: &MIPI_CSI_HOST, reg_addr: u8) {
        host.phy_test_ctrl1().write(|w| unsafe {
            w.phy_testen().set_bit();
            w.phy_testdin().bits(reg_addr)
        });
    }

    /// `mipi_csi_phy_ll_write_reg_val()`: `TEST_CTRL1 = reg_val`.
    fn phy_write_reg_val(host: &MIPI_CSI_HOST, reg_val: u8) {
        host.phy_test_ctrl1().write(|w| unsafe {
            w.phy_testen().clear_bit();
            w.phy_testdin().bits(reg_val)
        });
    }

    /// `s_mipi_csi_hal_phy_write_register()`: address, clock pulse, value, clock
    /// pulse. `mipi_csi_hal_init()`'s first write uses the raw primitives instead
    /// (address + a `TESTCLR` pulse), see `init_dphy()`.
    fn phy_write_reg(host: &MIPI_CSI_HOST, reg_addr: u8, reg_val: u8) {
        Self::phy_write_clock(host, 0, false);
        Self::phy_write_reg_addr(host, reg_addr);
        Self::phy_write_clock(host, 1, false);
        Self::phy_write_clock(host, 0, false);
        Self::phy_write_reg_val(host, reg_val);
        Self::phy_write_clock(host, 1, false);
        Self::phy_write_clock(host, 0, false);
    }

    /// §5.3: bridge burst length, buffer almost-full threshold, frame geometry,
    /// data-type filter window and output byte order.
    ///
    /// `HOST_CM_CTRL` (the bridge's colour converter) is deliberately **not**
    /// touched: on rev < 3.0 it does not exist and every helper that writes it is
    /// a no-op (§5.4/§8.4).
    fn configure_bridge(bridge: &MIPI_CSI_BRIDGE, width: u32, height: u32) {
        // `mipi_csi_brg_ll_set_burst_len(bridge, 512)` — 64-bit words per burst.
        bridge
            .dma_req_cfg()
            .modify(|_, w| unsafe { w.dma_burst_len().bits(BRIDGE_BURST_LEN) });
        // `mipi_csi_brg_ll_set_flow_ctl_buf_afull_thrd(bridge, 960)`.
        bridge
            .buf_flow_ctl()
            .modify(|_, w| unsafe { w.csi_buf_afull_thrd().bits(BRIDGE_BUF_AFULL_THRD) });
        // The bridge's HADR/VADR are loaded with the *count* (§5.3; the count vs
        // count-1 question is UNRESOLVED[3] and is what the VADR_NUM_* status
        // bits are for).
        bridge.frame_cfg().modify(|_, w| unsafe {
            w.hadr_num()
                .bits(width as u16)
                .vadr_num()
                .bits(height as u16)
        });
        // Standard-path data-type window: embedded data (0x12) .. RAW12 (0x2f),
        // which contains RAW10 = 0x2B.
        bridge.data_type_cfg().modify(|_, w| unsafe {
            w.data_type_min()
                .bits(DATA_TYPE_MIN)
                .data_type_max()
                .bits(DATA_TYPE_MAX)
        });
        // `byte_swap_en == false` for this path.
        bridge
            .endian_mode()
            .modify(|_, w| w.byte_endian_order().clear_bit());
    }

    /// §6.3 in order, §6.4's mandatory output-dependent gates, §6.5's RAW10 Bayer
    /// GBRG recipe and §8.2's explicit demosaic enable.
    fn configure_isp(
        isp: &ISP,
        width: u32,
        height: u32,
        data_type: u8,
        in_bits: u32,
        bypass: bool,
        out_type: u8,
        demosaic: bool,
        rgb2yuv: bool,
        yuv2rgb: bool,
    ) {
        // `isp_ll_init()`: clear CNTL, clear all event bits, mask all events.
        isp.cntl().write(|w| unsafe { w.bits(0) });
        isp.int_clr()
            .write(|w| unsafe { w.bits(ISP_LL_EVENT_ALL_MASK) });
        isp.int_ena()
            .write(|w| unsafe { w.bits(!ISP_LL_EVENT_ALL_MASK) });

        // §6.3 step 8 + §6.4: input data type (RAW10) and output type, including
        // the DEMOSAIC/RGB2YUV/YUV2RGB gates the output format forces.
        isp.cntl().modify(|_, w| unsafe {
            w.isp_data_type()
                .bits(data_type)
                .isp_out_type()
                .bits(out_type);
            if demosaic {
                w.demosaic_en().set_bit()
            } else {
                w.demosaic_en().clear_bit()
            };
            if rgb2yuv {
                w.rgb2yuv_en().set_bit()
            } else {
                w.rgb2yuv_en().clear_bit()
            };
            if yuv2rgb {
                w.yuv2rgb_en().set_bit()
            } else {
                w.yuv2rgb_en().clear_bit()
            }
        });

        // §6.3 step 9: ISP register clock force-on.
        isp.clk_en().modify(|_, w| w.clk_en().set_bit());

        // §6.3 step 10: error events enabled for status only (no handler is
        // installed and the CPU interrupt is never unmasked).
        isp.int_ena()
            .modify(|_, w| unsafe { w.bits(ISP_LL_EVENT_ERROR_MASK) });

        // §6.3 step 11: input source = CSI host, MIPI data path enabled.
        //
        // `MIPI_DATA_EN` is enabled in BOTH modes. Measured on the working reference, whose
        // ISP runs bypassed: `ISP.CNTL = 0x00025001`, i.e. bit 0 (`MIPI_DATA_EN`) is SET and
        // bit 1 (`ISP_EN`) is CLEAR. An earlier version of this branch cleared bit 0 when
        // bypassing, which the measurement contradicts -- the data path is enabled while the
        // pipeline is bypassed, not disabled.
        isp.cntl()
            .modify(|_, w| unsafe { w.isp_in_src().bits(ISP_IN_SRC_CSI).mipi_data_en().set_bit() });

        // The three sub-block enables the reference leaves SET while the pipeline is bypassed.
        // Measured: `ISP.CNTL = 0x00025001`, i.e. bit 0 `MIPI_DATA_EN`, bit 12 `COLOR_EN`, bit 14
        // `AE_EN`, bit 17 `HIST_EN` -- and bit 1 `ISP_EN` clear. Decoded against `isp_reg.h`; the
        // earlier claim that "all pipeline gates are 0" came from reading the wrong offset
        // (`ISP_BASE + 0x00` is `ISP_VER_DATE`, not `ISP_CNTL`).
        //
        // These are colour and statistics blocks, all under a disabled `ISP_EN`, and the reference
        // produces correct frames with them set, so matching it is the conservative choice rather
        // than a behaviour change. They do not touch `ISP_EN`.
        isp.cntl()
            .modify(|_, w| w.color_en().set_bit().ae_en().set_bit().hist_en().set_bit());

        // §6.3 step 12: the OV5647 sends no line start/end packets.
        isp.frame_cfg().modify(|_, w| {
            w.hsync_start_exist()
                .clear_bit()
                .hsync_end_exist()
                .clear_bit()
        });

        // §6.3 steps 13/14: geometry. In bypass mode the input H-size is re-expressed in
        // 32-bit words -- IDF: `ISP_DIV_ROUND_UP(h_res * in_bits_per_pixel, 32) - 1` -- because
        // the input module still runs even though no processing happens.
        // The `8` here used to be a literal with "RAW8 in, IDI32 out" beside it: the input width was
        // re-expressed in 32-bit words using **8** bits per pixel whatever the sensor was sending.
        // That is invisible while RAW8 is the only mode and wrong for every other one -- measured on
        // hardware, a 1920-wide RAW10 bypass programmed `HADR_NUM = 479` (1920*8/32 - 1) where the
        // formula gives `(1920*10 + 31)/32 - 1 = 599`. The divisor is the *sensor's* bits per pixel,
        // so it is now derived from the format rather than assumed.
        let hadr = if bypass {
            ov5647_core::isp_geometry::hadr_num_bypass(width, in_bits)
        } else {
            ov5647_core::isp_geometry::hadr_num_converting(width)
        };
        isp.frame_cfg()
            .modify(|_, w| unsafe { w.hadr_num().bits(hadr).vadr_num().bits((height - 1) as u16) });

        // §6.3 step 15 / §6.5 item 4: GBRG.
        isp.frame_cfg()
            .modify(|_, w| unsafe { w.bayer_mode().bits(ISP_BAYER_MODE_GBRG) });

        // §6.3 step 17 (DIG-474 colour default) applies to DVP/DW-GDMA inputs
        // only; our input is the CSI host. Step 18 (byte swap) is off.
        // Step 19 (shadow mode) is a no-op below rev 3.0 — not programmed
        // (UNRESOLVED[7]).

        // §6.3 step 21: ISP_EN last, after everything above -- BUT NOT IN BYPASS MODE.
        // Measured on the working reference: `ISP.CNTL.ISP_EN = 0` for the whole capture, and
        // IDF's `esp_isp_enable()` (the only writer of this bit) is called only when the ISP is
        // not bypassed.
        if !bypass {
            isp.cntl().modify(|_, w| w.isp_en().set_bit());
        }

        // §8.2: `esp_isp_enable()` does not turn the demosaic block on for this
        // revision, so configure and enable it explicitly, after ISP_EN. The
        // settings are the ones the working rev-1.3 projects use: gradient ratio
        // 0, surrounding-data edge padding with zero padding data and zero tail
        // pixel pulses.
        if demosaic {
            isp.demosaic_grad_ratio()
                .write(|w| unsafe { w.demosaic_grad_ratio().bits(0) });
            isp.demosaic_matrix_ctrl().modify(|_, w| unsafe {
                w.demosaic_padding_data()
                    .bits(0)
                    .demosaic_tail_pixen_pulse_tl()
                    .bits(0)
                    .demosaic_tail_pixen_pulse_th()
                    .bits(0);
                // `ISP_DEMOSAIC_EDGE_PADDING_MODE_SRND_DATA` = 0.
                if DEMOSAIC_PADDING_SRND_DATA != 0 {
                    w.demosaic_padding_mode().set_bit()
                } else {
                    w.demosaic_padding_mode().clear_bit()
                }
            });
            isp.cntl().modify(|_, w| w.demosaic_en().set_bit());
        }
    }
}

// ---------------------------------------------------------------------------
// DW-GDMA field encodings (constants)
// ---------------------------------------------------------------------------

/// `mipi_csi_brg_ll_set_burst_len(bridge, 512)`.
const BRIDGE_BURST_LEN: u16 = 512;
/// `mipi_csi_brg_ll_set_flow_ctl_buf_afull_thrd(bridge, 960)`.
const BRIDGE_BUF_AFULL_THRD: u16 = 960;
/// Bridge pixel-filter window, standard path (§5.3).
const DATA_TYPE_MIN: u8 = 0x12;
/// Bridge pixel-filter window, standard path (§5.3); 0x2B (RAW10) is inside it.
const DATA_TYPE_MAX: u8 = 0x2F;

/// `isp_ll_set_input_data_source(ISP_INPUT_DATA_SOURCE_CSI)` -> `ISP_IN_SRC = 0`.
const ISP_IN_SRC_CSI: u8 = 0;
