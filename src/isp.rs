//! ISP bypass and geometry configuration for the OV5647 CSI path.
//!
//! This module owns the ISP register block: bypass mode, input/output data
//! type selection, Bayer geometry, and the clock/burst constants. The ISP is
//! bypassed when the sensor's wire format already matches the memory-side
//! output format; in that case the output type is written as 0 because the
//! pipeline does not run and the raw sensor bytes pass through.
//!
//! ESP-IDF headers are cited by basename (`isp_ll.h`, `isp_types.h`,
//! `color_types.h`, the generated `isp_reg.h`/`isp_struct.h`), and the identifiers
//! taken from them were checked at tag `v5.5` and at the commit this repo's notes
//! pin, `d445e947156f276dfe0db5804e4edf6ae90fe6fb`. The clock-source encoding,
//! noted on [`ISP_CLK_SRC_PLL240`], is the one place here where the two revisions
//! disagree.

use ov5647_core::PixelFormat;

/// `ISP_CLK_SRC_PLL240` — value 1 in `peri_clk_ctrl25.isp_clk_src_sel`
/// (`isp_ll.h`, at the pinned commit `d445e947`; tag `v5.5` numbers the same source
/// `2` and puts `PLL160` at 1, so the encoding is revision-dependent).
pub(crate) const ISP_CLK_SRC_PLL240: u8 = 1;

/// `PERI_CLK_CTRL26.isp_clk_div_num` for 80 MHz out of the 240 MHz source selected
/// above: `div = 240/80 = 3`, register value `div - 1 = 2`
/// (`isp_ll_set_clock_div()` writes `integer - 1`). ESP-IDF's ISP test app
/// (`test_isp_driver.c:19`) and its multi-pipeline example (`isp_dsi_main.c:280`)
/// request `clk_hz = 80 MHz`; `isp_core.c` takes that value from its caller and
/// only checks it against the source frequency — it does not choose it.
pub(crate) const ISP_CLK_DIV_NUM: u8 = 2;

/// `ISP.FRAME_CFG.BAYER_MODE` = 0b01 = GB/RG (`COLOR_RAW_ELEMENT_ORDER_GBRG = 1`
/// in `color_types.h`; the vendor OV5647 driver declares GBRG for every mode).
/// §8.9: re-check this whenever the sensor window changes.
pub(crate) const ISP_BAYER_MODE_GBRG: u8 = 0b01;

/// `ISP.DEMOSAIC_MATRIX_CTRL.DEMOSAIC_PADDING_MODE` = 0 =
/// `ISP_DEMOSAIC_EDGE_PADDING_MODE_SRND_DATA` (`isp_types.h`).
pub(crate) const DEMOSAIC_PADDING_SRND_DATA: u8 = 0;

/// `ISP_LL_EVENT_ALL_MASK` for chip revision < 3.0 (`isp_ll.h`).
pub(crate) const ISP_LL_EVENT_ALL_MASK: u32 = 0x1FFF_FFFF;

/// `ISP_LL_EVENT_ERROR_MASK` (`isp_ll.h`): DATA_TYPE_ERR | ASYNC_FIFO_OVF |
/// BUF_FULL | HVNUM_SETTING_ERR | DATA_TYPE_SETTING_ERR | MIPI_HNUM_UNMATCH |
/// CROP_ERR. Enabled for status only — no interrupt handler is installed and the
/// global/CPU interrupt is never unmasked, so these bits are read from `INT_RAW`.
pub(crate) const ISP_LL_EVENT_ERROR_MASK: u32 =
    (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 31);

/// `MIPI_CSI_PHY_CLK_SRC_PLL_F20M` — value 0 in
/// `peri_clk_ctrl03.mipi_csi_dphy_clk_src_sel` (`mipi_csi_ll.h`), and the
/// documented default source (`clk_tree_defs.h`: `MIPI_CSI_PHY_CLK_SRC_DEFAULT`).
pub(crate) const PHY_CLK_SRC_PLL_F20M: u8 = 0;

/// `ISP.CNTL.ISP_DATA_TYPE` -- the format of the data the ISP *receives*, which is the
/// sensor's wire format, not the memory-side output format. SVD:
/// `0 = RAW8, 1 = RAW10, 2 = RAW12`. This used to be the fixed constant
/// `ISP_DATA_TYPE_RAW10 = 1`, which silently mis-declared the input whenever the mode was
/// RAW8 -- and the ISP raises `INT_RAW.ISP_DATA_TYPE_SETTING_ERR` when the declaration does
/// not match the stream.
pub(crate) const fn isp_data_type_for(sensor_format: PixelFormat) -> Option<u8> {
    match sensor_format {
        PixelFormat::Raw8 => Some(0),
        PixelFormat::Raw10 => Some(1),
        // The ISP can accept RAW12, but no OV5647 table here produces it, and the
        // memory-side output formats below are never a sensor wire format.
        PixelFormat::Rgb888 | PixelFormat::Rgb565 => None,
    }
}

/// `ISP.CNTL.ISP_OUT_TYPE` and the output-format-dependent gate bits, from
/// `isp_ll_set_output_data_color_format()` (§6.4).
///
/// Returns `None` for formats the ISP cannot produce: RAW10/RAW12 are not
/// accepted as an ISP output at all (the bypass path would be needed, §6.4), so
/// a RAW10 capture request is refused instead of silently producing something.
pub(crate) const fn isp_output_config(format: PixelFormat) -> Option<(u8, bool, bool, bool)> {
    // (ISP_OUT_TYPE, DEMOSAIC_EN, RGB2YUV_EN, YUV2RGB_EN)
    match format {
        PixelFormat::Raw8 => Some((0, false, false, false)),
        PixelFormat::Raw10 => None,
        PixelFormat::Rgb888 => Some((2, true, true, true)),
        PixelFormat::Rgb565 => Some((4, true, true, true)),
    }
}

/// Raw status of the whole path, for bring-up diagnostics.
///
/// All fields are register values read back from the hardware; nothing here is
/// derived from configuration (which is exactly the mistake §4.3/§8.3 documents
/// for ESP-IDF's `received_size`).
/// Snapshot of the D-PHY / CSI host control and status registers.
///
/// This is what separates "the receiver is still held in reset" from "the
/// receiver is up but the line is silent" -- a distinction `PHY_STOPSTATE` alone
/// cannot make. Note the bit map: bit 0 is data lane 0 and the clock lane is
/// bit 16, not bit 0.
/// `ISP.CNTL` and `ISP.FRAME_CFG` as raw words, for decoding by the caller.
///
/// The field layout is in `isp_reg.h` and is **not** the one an offset taken from
/// notes would suggest: `ISP_CNTL` is at `+0x08` and `ISP_FRAME_CFG` at `+0x10`
/// (`+0x00` is `ISP_VER_DATE`, a constant). Decoding is left to the reader so the
/// raw words can be checked against the header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IspRegisters {
    /// `ISP.CNTL[0]` = `MIPI_DATA_EN`, `ISP.CNTL[1]` = `ISP_EN`.
    pub cntl: u32,
    /// `VADR_NUM [11:0]`, `HADR_NUM [23:12]`, `BAYER_MODE [28:27]`,
    /// `HSYNC_START_EXIST [29]`, `HSYNC_END_EXIST [30]`.
    pub frame_cfg: u32,
}
