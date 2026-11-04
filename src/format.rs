//! Pixel formats and frame-transfer sizing.
//!
//! This module gathers the pixel-format / transfer-sizing surface that the
//! sensor, ISP and frame-transfer paths share, so the mapping from a
//! [`PixelFormat`] to the number of wire bits and memory bytes it occupies
//! lives in one place.
//!
//! # RAW8 vs packed RAW10
//!
//! **RAW8** sends one byte per pixel: the ISP's input and the DMA's memory
//! footprint both scale linearly at 8 bits/pixel, so a `width × height` frame
//! is exactly `width × height` bytes.
//!
//! **Packed RAW10** sends 10 bits per pixel on the CSI-2 wire and stores them
//! packed — four 10-bit pixels occupy five bytes (40 bits per four pixels), so
//! the on-memory footprint is `width × height × 10 / 8` bytes, i.e. 1.25
//! bytes per pixel. That non-integer ratio is the reason sizing is expressed
//! in bits first ([`out_bits_per_pixel`]) and rounded down to whole bytes only
//! when the buffer length is needed.
//!
//! # 64-bit-word / cache-line constraints
//!
//! The ISP's block-transfer size is expressed in 64-bit words, so a frame must
//! be a whole number of them — and the DMA landing zone is cache-line aligned,
//! so the buffer length must be a multiple of the 64-byte data-cache line for a
//! whole-buffer flush (see `frame::DMA_ALIGN`). Callers that size a buffer must
//! therefore check both the 64-bit-word and the 64-byte alignment after
//! computing [`frame_len_bytes`]; that helper does not round or check for them.

pub use ov5647_core::PixelFormat;

/// Bits per pixel the **sensor** puts on the wire for this format.
///
/// Distinct from [`out_bits_per_pixel`] even where the numbers coincide: this
/// one describes the CSI-2 input the ISP's input module has to be sized for,
/// which is what `HADR_NUM` is expressed in.
pub const fn in_bits_per_pixel(f: PixelFormat) -> u32 {
    match f {
        PixelFormat::Raw8 => 8,
        PixelFormat::Raw10 => 10,
        // Unreachable: these are ISP output formats, not sensor formats. They
        // are mapped through the output table rather than returning 0, because a
        // 0 here would silently size the input module for nothing.
        PixelFormat::Rgb565 => 16,
        PixelFormat::Rgb888 => 24,
    }
}

/// Bits per pixel of the ISP output format (memory side).
pub const fn out_bits_per_pixel(f: PixelFormat) -> u32 {
    match f {
        PixelFormat::Raw8 => 8,
        PixelFormat::Raw10 => 10,
        PixelFormat::Rgb565 => 16,
        PixelFormat::Rgb888 => 24,
    }
}

/// Byte length of a `width × height` frame in `f`, or `None` on overflow.
///
/// Thin convenience wrapper over the shared sizing rule `width*height*bits/8`
/// using `checked_mul`, returning `None` if the geometry overflows `usize`.
/// It is a public convenience only — it does not enforce the 64-bit-word /
/// cache-line alignment discussed in the module docs; callers that need a
/// buffer should round up with `frame::aligned_len` and check alignment
/// themselves.
pub const fn frame_len_bytes(width: u32, height: u32, f: PixelFormat) -> Option<usize> {
    let px = match (width as usize).checked_mul(height as usize) {
        Some(px) => px,
        None => return None,
    };
    match px.checked_mul(out_bits_per_pixel(f) as usize) {
        Some(bits) => Some(bits / 8),
        None => None,
    }
}
