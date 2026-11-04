//! Frame buffers, frame validation and the host transfer framing.
//!
//! Three jobs live on the receiver side of the MIPI CSI-2 path:
//!
//! * [`FrameBuffer`] — the DMA landing zone: a `static`-plausible, 64-byte
//!   aligned byte array whose size is a const parameter, so the caller sizes it
//!   for the sensor mode it actually configured.
//! * [`analyze`] — turns "the DMA says it wrote N bytes" into an explicit
//!   [`FrameVerdict`], so the firmware never announces an image that is in fact
//!   a truncated, all-zero or stuck-constant buffer.
//! * [`transfer`] — writes `<<<FRAME ...>>>`, the payload and `<<<END>>>` to a
//!   [`FrameSink`]: on this board the USB Serial/JTAG console, but equally a
//!   UART or a test double that records what would have gone out. The write is
//!   bounded, so a sink that stops accepting bytes aborts the frame instead of
//!   parking the firmware.
//!
//! Alignment: the receive buffer must be aligned to the larger of the DW-GDMA
//! alignment (4 B) and the data cache line — 64 B on this part, which is
//! [`DMA_ALIGN`] — and its length must be a multiple of that same size, so one
//! cache maintenance call covers the buffer exactly. [`aligned_len`] does that
//! rounding.
//!
//! Nothing here fabricates picture content: every payload byte comes from the
//! DMA, and every number announced to the host is measured from that buffer.
//!
//! For the wire framing itself (header rendering/parsing and CRC-32) this module
//! defers to `ov5647_core`, so firmware and host script share one definition.

use core::fmt;

use ov5647_core::{crc32_ieee, FrameHeader, PixelFormat};

/// DMA / data-cache-line alignment for the frame buffer; see the module docs.
pub const DMA_ALIGN: usize = 64;

/// Rounds `len` up to a whole number of [`DMA_ALIGN`] units.
///
/// `const` so that a `static` buffer can be sized from a sensor mode at compile
/// time, e.g.
///
/// ```text
/// const FRAME_LEN: usize = aligned_len(PixelFormat::Raw10.frame_len_bytes(1920, 1080) as usize);
/// static FRAME: FrameBuffer<FRAME_LEN> = FrameBuffer::new();
/// ```
pub const fn aligned_len(len: usize) -> usize {
    (len + DMA_ALIGN - 1) & !(DMA_ALIGN - 1)
}

/// Landing zone for the CSI receive DMA.
///
/// `repr(align)` only accepts a literal, so the attribute is tied back to
/// [`DMA_ALIGN`] by the `const` assertion below rather than by the attribute
/// itself. The buffer itself is zero-initialised; a capture overwrites the first
/// `written` bytes and leaves the tail as-is, so consumers must use the count
/// the DMA reports (see [`FrameBuffer::captured`]) and never the whole capacity.
#[repr(align(64))]
pub struct FrameBuffer<const N: usize> {
    data: [u8; N],
}

// Keeps the literal in the attribute above honest.
const _: () = assert!(core::mem::align_of::<FrameBuffer<1>>() == DMA_ALIGN);

impl<const N: usize> FrameBuffer<N> {
    /// A zeroed buffer, usable in a `static`.
    pub const fn new() -> Self {
        Self { data: [0u8; N] }
    }

    /// Bytes this buffer can hold — the DMA transfer size must not exceed it.
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Whole buffer, as background access for diagnostics.
    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }

    /// Whole buffer, for pre-capture poisoning ([`Self::fill`]).
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data
    }

    /// Address to program into the GDMA destination-address register.
    ///
    /// The caller is responsible for the cache maintenance that a DMA write into
    /// a cached buffer needs; this function only hands out the address.
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.data.as_mut_ptr()
    }

    /// Fills the buffer with `value`.
    ///
    /// The CSI path does not naturally land on a distinctive "no data" pattern,
    /// so poisoning the buffer before a capture makes an inert buffer (filled
    /// with this value) trivially distinguishable from one the DMA wrote.
    pub fn fill(&mut self, value: u8) {
        self.data.fill(value);
    }

    /// The first `written` bytes, clamped to the buffer: exactly what the DMA
    /// reports having produced.
    pub fn captured(&self, written: usize) -> &[u8] {
        &self.data[..written.min(N)]
    }

    /// Analyses the first `written` bytes against `header`; see [`analyze`].
    pub fn analyze(&self, header: &FrameHeader, written: usize, elapsed_ms: u64) -> FrameAnalysis {
        analyze_header(self.captured(written), header, elapsed_ms)
    }
}

impl<const N: usize> Default for FrameBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// What a received payload turned out to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameVerdict {
    /// The receiver produced fewer bytes than the header claims: the frame is
    /// incomplete and must not be treated as an image.
    Truncated {
        /// Bytes the header claims the frame contains.
        claimed: usize,
        /// Bytes the receiver actually produced.
        actual: usize,
    },
    /// Every payload byte is zero: no signal reached the buffer (sensor still
    /// parked, or the DMA never ran).
    AllZero,
    /// Every payload byte holds the same non-zero value: a stuck data path.
    Constant {
        /// The repeated byte value.
        value: u8,
    },
    /// At least two distinct byte values: the payload carries real variance.
    Plausible {
        /// Number of distinct byte values observed (2..=256).
        distinct_bytes: u16,
    },
}

impl FrameVerdict {
    /// True only for [`FrameVerdict::Plausible`] — the sole verdict that
    /// justifies announcing a picture to the host.
    pub fn is_plausible_image(self) -> bool {
        matches!(self, Self::Plausible { .. })
    }
}

impl fmt::Display for FrameVerdict {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::Truncated { claimed, actual } => {
                write!(f, "truncated (header claims {claimed} bytes, got {actual})")
            }
            Self::AllZero => f.write_str("all zero (no signal)"),
            Self::Constant { value } => write!(f, "constant byte 0x{value:02X} (stuck data path)"),
            Self::Plausible { distinct_bytes } => {
                write!(f, "plausible image ({distinct_bytes} distinct byte values)")
            }
        }
    }
}

/// Measurements taken over one received payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameSummary {
    /// Frame width in pixels.
    pub width: u32,
    /// Frame height in pixels.
    pub height: u32,
    /// Pixel format of the frame.
    pub format: PixelFormat,
    /// Payload bytes considered: the smaller of what the receiver produced and
    /// what the header claims.
    pub len: usize,
    /// CRC-32/ISO-HDLC over those bytes — the value a header must carry for the
    /// host to accept the frame.
    pub crc32: u32,
    /// Payload bytes that are not zero.
    pub nonzero_bytes: usize,
    /// Number of distinct byte values in the payload (`0..=256`).
    pub distinct_bytes: u16,
    /// Wall time from capture start to this analysis, in milliseconds.
    pub elapsed_ms: u64,
}

impl FrameSummary {
    /// The header that describes this payload exactly: same geometry, format,
    /// byte count and CRC, i.e. what [`transfer`] must announce.
    pub fn header(&self) -> FrameHeader {
        FrameHeader {
            width: self.width,
            height: self.height,
            format: self.format,
            payload_len: self.len.min(u32::MAX as usize) as u32,
            crc32: self.crc32,
        }
    }
}

/// A [`FrameSummary`] together with the verdict drawn from it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FrameAnalysis {
    /// Measurements taken over the payload.
    pub summary: FrameSummary,
    /// The verdict drawn from those measurements.
    pub verdict: FrameVerdict,
}

/// Analyses a payload against the header that claims to describe it.
///
/// Convenience wrapper over [`analyze`] using the header's own geometry, format
/// and claimed length — the same fields the host will re-check.
pub fn analyze_header(bytes: &[u8], header: &FrameHeader, elapsed_ms: u64) -> FrameAnalysis {
    analyze(
        bytes,
        header.width,
        header.height,
        header.format,
        header.payload_len as usize,
        elapsed_ms,
    )
}

/// Measures `bytes` and judges whether it looks like an image.
///
/// `claimed_len` is the payload length the header announces; only the first
/// `min(bytes.len(), claimed_len)` bytes are considered, so a buffer that is
/// larger than the frame does not drag stale tail bytes into the CRC the host
/// will compute. Fewer bytes than claimed yields [`FrameVerdict::Truncated`],
/// which takes precedence over the pixel verdicts because the payload is
/// incomplete either way. An empty payload (nothing written *and* nothing
/// claimed) is reported as [`FrameVerdict::AllZero`].
pub fn analyze(
    bytes: &[u8],
    width: u32,
    height: u32,
    format: PixelFormat,
    claimed_len: usize,
    elapsed_ms: u64,
) -> FrameAnalysis {
    let payload = &bytes[..bytes.len().min(claimed_len)];

    let mut nonzero_bytes = 0usize;
    // 256-bit set of the byte values seen: a full histogram would cost 1 KiB of
    // stack for a count we can popcount out of four words.
    let mut seen = [0u64; 4];
    let mut min = u8::MAX;
    let mut max = u8::MIN;
    for &byte in payload {
        if byte != 0 {
            nonzero_bytes += 1;
        }
        seen[(byte >> 6) as usize] |= 1u64 << (byte & 63);
        min = min.min(byte);
        max = max.max(byte);
    }
    let distinct_bytes: u16 = seen.iter().map(|word| word.count_ones() as u16).sum();

    let verdict = if bytes.len() < claimed_len {
        FrameVerdict::Truncated {
            claimed: claimed_len,
            actual: bytes.len(),
        }
    } else if nonzero_bytes == 0 {
        FrameVerdict::AllZero
    } else if distinct_bytes == 1 {
        // `distinct_bytes == 1` implies a non-empty payload, so `min` is a real
        // observation and equals `max`.
        FrameVerdict::Constant { value: min }
    } else {
        FrameVerdict::Plausible { distinct_bytes }
    };

    FrameAnalysis {
        summary: FrameSummary {
            width,
            height,
            format,
            len: payload.len(),
            crc32: crc32_ieee(payload),
            nonzero_bytes,
            distinct_bytes,
            elapsed_ms,
        },
        verdict,
    }
}

/// Largest byte count any header line can occupy, excluding the line ending:
/// prefix, five `key=value` fields with `u32` extremes, and the `>>>` suffix.
pub const HEADER_TEXT_MAX: usize = 96;

/// Payload bytes handed to the sink per [`FrameSink::write_bytes`] call.
///
/// Bounds how long one critical section is held and how far past the failure
/// point a stalled transfer can run before it notices.
const TRANSFER_CHUNK: usize = 1024;

/// Why a frame could not be put on the wire at all.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferError {
    /// The header's `len` field disagrees with the payload handed in — the host
    /// would reject the frame as mislabelled, so nothing is written.
    LengthMismatch {
        /// Length the header declares.
        header_len: u32,
        /// Length of the payload actually handed in.
        payload_len: usize,
    },
    /// The header did not render into [`HEADER_TEXT_MAX`] bytes.
    HeaderUnrenderable,
}

impl fmt::Display for TransferError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::LengthMismatch {
                header_len,
                payload_len,
            } => write!(
                f,
                "header claims {header_len} payload bytes, transfer has {payload_len}"
            ),
            Self::HeaderUnrenderable => f.write_str("frame header does not fit the text buffer"),
        }
    }
}

/// What a transfer attempt actually managed to emit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransferReport {
    /// Payload bytes the peripheral accepted before the transfer stopped.
    pub payload_bytes_written: usize,
    /// Payload length the header announced (and the transfer intended to send).
    pub payload_len: usize,
    /// Whether the `<<<END>>>` trailer was emitted.
    ///
    /// The text path ([`FrameSink::write_text`]) is bounded but cannot report a
    /// partially written line, so this records that the trailer was written
    /// *after* every payload byte had been accepted, not that each of its bytes
    /// drained.
    pub trailer_sent: bool,
}

impl TransferReport {
    /// Every payload byte reached the peripheral and the frame was closed.
    pub fn complete(&self) -> bool {
        self.payload_bytes_written == self.payload_len && self.trailer_sent
    }

    /// The sink stopped accepting bytes mid-frame.
    pub fn aborted(&self) -> bool {
        !self.complete()
    }
}

/// Where a finished frame is put on the wire.
///
/// [`transfer`] does not know what it is writing to: it renders the header line,
/// pushes the payload and closes the frame through this trait, so the same
/// transfer path can drive the USB Serial/JTAG console, a UART, or a test double
/// that records the bytes instead of sending them.
///
/// Both methods are expected to be **bounded**: an implementation must give up
/// rather than block forever on a receiver that has stopped reading, because
/// [`transfer`] uses their return values to abort a frame instead of parking the
/// firmware. Implementations are also free to be lossy — the text methods report
/// only whether a whole line went out, not whether every byte drained.
pub trait FrameSink {
    /// Payload bytes. Returns how many were accepted; may be short.
    fn write_bytes(&mut self, bytes: &[u8]) -> usize;
    /// A text line (header or trailer). Returns true if fully written.
    fn write_text(&mut self, text: &str) -> bool;
}

/// Emits one frame: the header line, the payload, then [`ov5647_core::FRAME_TRAILER`].
///
/// The header line and the trailer are handed to `sink` as text, the payload
/// bytes in `TRANSFER_CHUNK`-sized pieces; each `"\r\n"` line ending is its own
/// [`FrameSink::write_text`] call. The payload path is accept-counted, so this
/// function cannot park the firmware on a sink that has stopped accepting: as
/// soon as a chunk is only partially accepted it stops, skips the trailer (so the
/// host sees a short frame rather than a closed one) and returns the count that
/// actually went out. Text is best-effort, exactly as before: `write_text` is
/// called and its verdict not used, because [`TransferReport`] reports nothing
/// about the header line.
///
/// The length guard runs first, so a header that misdescribes its payload never
/// reaches the wire. Note that on an abort the host is left waiting for payload
/// bytes: whatever text is logged afterwards is indistinguishable from payload
/// on the wire, which is why the host must treat a short or CRC-failing frame as
/// a communication failure, and why the abort itself is reported here instead.
pub fn transfer<S: FrameSink>(
    sink: &mut S,
    header: &FrameHeader,
    payload: &[u8],
) -> Result<TransferReport, TransferError> {
    if header.payload_len as usize != payload.len() {
        return Err(TransferError::LengthMismatch {
            header_len: header.payload_len,
            payload_len: payload.len(),
        });
    }

    let mut text = [0u8; HEADER_TEXT_MAX];
    let line_len = header
        .write_into(&mut text)
        .ok_or(TransferError::HeaderUnrenderable)?;
    // The header renders keys, digits and ASCII tokens only.
    let line =
        core::str::from_utf8(&text[..line_len]).map_err(|_| TransferError::HeaderUnrenderable)?;
    sink.write_text(line);
    sink.write_text("\r\n");

    let mut payload_bytes_written = 0usize;
    for chunk in payload.chunks(TRANSFER_CHUNK) {
        let accepted = sink.write_bytes(chunk);
        payload_bytes_written += accepted;
        if accepted < chunk.len() {
            return Ok(TransferReport {
                payload_bytes_written,
                payload_len: payload.len(),
                trailer_sent: false,
            });
        }
    }

    sink.write_text(ov5647_core::FRAME_TRAILER);
    sink.write_text("\r\n");
    Ok(TransferReport {
        payload_bytes_written,
        payload_len: payload.len(),
        trailer_sent: true,
    })
}
