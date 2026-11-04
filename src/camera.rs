//! The camera lifecycle object that ties the sensor to the capture pipeline.
//!
//! A [`Camera`] owns one [`Ov5647`] driver and one [`CsiCapture`] whose destination is a
//! caller-owned frame buffer in PSRAM. It is the high-level entry point for board code: configure
//! the sensor, arm a capture, wait for a whole frame, and read it back. The low-level parts stay
//! reachable — [`crate::csi::CsiCapture`] and [`crate::sensor::Ov5647`] are public — so a board
//! that needs the bring-up instruments can use them directly.
//!
//! The destination buffer is caller-owned and lives in the board example, because only the board
//! knows where PSRAM is and how it was configured. See [`crate::framebuffer::DmaFrameBuffer`] for
//! the alignment and reachability contract the caller must meet.

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;

use crate::csi::CsiCapture;
use crate::framebuffer::DmaFrameBuffer;
use crate::sensor::{ModeSpec, Ov5647};
use crate::CameraMode;

/// Error type for the high-level camera lifecycle.
///
/// This wraps the two lower-level error types this abstraction crosses — sensor (SCCB) and capture
/// (CSI) — so a caller handles one error type instead of two.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CameraError {
    /// A SCCB transaction failed.
    Sccb(crate::sensor::SccbError),
    /// A register-table write failed; names the register.
    RegEntry(crate::regs::Reg, crate::sensor::SccbError),
    /// A capture-pipeline configuration or run failed.
    Csi(crate::CsiError),
}

impl From<crate::sensor::SccbError> for CameraError {
    fn from(e: crate::sensor::SccbError) -> Self {
        Self::Sccb(e)
    }
}

impl From<(crate::regs::Reg, crate::sensor::SccbError)> for CameraError {
    fn from((reg, e): (crate::regs::Reg, crate::sensor::SccbError)) -> Self {
        Self::RegEntry(reg, e)
    }
}

impl From<crate::CsiError> for CameraError {
    fn from(e: crate::CsiError) -> Self {
        Self::Csi(e)
    }
}

/// A camera bound to a sensor bus and a caller-owned frame buffer.
///
/// # Type parameters
///
/// * `I` — the SCCB/I2C bus driver (an `embedded-hal` 1.0 `I2c`), typically
///   [`crate::sccb::SccbGpio`] or an ESP32-P4 hardware I2C master.
/// * `D` — a `DelayNs` implementation for the sensor's timing requirements.
///
/// # Safety contract (caller's)
///
/// `buffer` must satisfy the [`DmaFrameBuffer`] contract: 64-byte cache-line alignment, within a
/// DMA-reachable region, and not aliased by anything else while a capture is armed. `Camera` does
/// not own the buffer; it borrows it mutably for the capture.
pub struct Camera<'a, I> {
    sensor: Ov5647<I>,
    mode: CameraMode,
    capture: Option<CsiCapture<'a>>,
}

impl<'a, I> Camera<'a, I>
where
    I: I2c,
{
    /// Create a camera on a sensor bus with a mode and a caller-owned frame buffer.
    ///
    /// This identifies the sensor, configures it for `mode`, then builds the capture pipeline for
    /// the same mode with `buffer` as the destination. If the sensor does not answer or the buffer
    /// fails validation (alignment, capacity, reachability), the construction fails before any
    /// DMA is armed.
    ///
    /// # Errors
    ///
    /// * [`CameraError::Sccb`] if the sensor's SCCB bus does not answer during identification.
    /// * [`CameraError::RegEntry`] if the mode's register table fails to write.
    /// * [`CameraError::Csi`] if the capture pipeline cannot be brought up for this mode and
    ///   buffer (see [`CsiCapture::new_with_buffer`]).
    ///
    /// # Panics
    ///
    /// None; all failure is returned.
    pub fn new<D: DelayNs>(
        i2c: I,
        mut delay: D,
        mode: CameraMode,
        buffer: DmaFrameBuffer<'a>,
    ) -> Result<Self, CameraError> {
        let mut sensor = Ov5647::new(i2c, crate::regs::SCCB_ADDR);
        let spec: ModeSpec = mode.spec();

        match sensor.identify() {
            crate::sensor::ChipIdOutcome::Match(_) => {}
            crate::sensor::ChipIdOutcome::Mismatch(_) => {
                // The sensor answered but is not an OV5647. Refuse rather than configure a part
                // we do not recognise.
                return Err(CameraError::Sccb(crate::sensor::SccbError::Nack));
            }
            crate::sensor::ChipIdOutcome::BusError(e) => return Err(CameraError::Sccb(e)),
        }

        sensor.configure(&spec, &mut delay)?;
        sensor.stop_stream()?;

        let capture = CsiCapture::new_with_buffer(
            spec.width,
            spec.height,
            spec.format,
            spec.format,
            spec.lanes,
            spec.mipi_bitrate_bps,
            buffer,
        )?;

        Ok(Self {
            sensor,
            mode,
            capture: Some(capture),
        })
    }

    /// The mode this camera was created for.
    pub const fn mode(&self) -> CameraMode {
        self.mode
    }

    /// Access the underlying sensor driver (for example, to read AEC registers).
    pub fn sensor(&mut self) -> &mut Ov5647<I> {
        &mut self.sensor
    }

    /// Start streaming and arm the first capture.
    ///
    /// The sensor is put in streaming mode, then the capture pipeline is started. If the capture is
    /// already armed (a previous frame was not consumed), this returns [`CameraError::Csi`]
    /// wrapping [`crate::CsiError::AlreadyArmed`].
    ///
    /// # Errors
    ///
    /// * [`CameraError::RegEntry`] if the sensor cannot enter streaming mode.
    /// * [`CameraError::Csi`] if the capture cannot be armed.
    pub fn start(&mut self) -> Result<(), CameraError> {
        self.sensor.start_stream()?;
        if let Some(cap) = self.capture.as_mut() {
            cap.start()?;
        }
        Ok(())
    }

    /// Wait for the next whole frame.
    ///
    /// Returns the [`crate::csi::FrameInfo`] for the completed frame. The frame bytes are in the
    /// buffer passed to [`Self::new`]; call [`Self::frame_bytes`] to read them.
    ///
    /// # Errors
    ///
    /// * [`CameraError::Csi`] if the frame does not complete within the timeout (see
    ///   [`crate::CsiError::Timeout`]).
    pub fn wait_frame(
        &mut self,
        timeout: esp_hal::time::Duration,
    ) -> Result<crate::csi::FrameInfo, CameraError> {
        let cap = self.capture.as_mut().ok_or(crate::CsiError::NotStarted)?;
        Ok(cap.wait_frame(timeout)?)
    }

    /// Borrow the last completed frame's bytes.
    ///
    /// The slice is the buffer's contents up to the frame length. While this borrow is live, the
    /// capture cannot be re-armed; drop it before calling [`Self::start`] again.
    pub fn frame_bytes(&self) -> &[u8] {
        self.capture.as_ref().map(|c| c.buffer()).unwrap_or(&[])
    }

    /// Stop streaming and disarm the capture.
    pub fn stop(&mut self) {
        if let Some(cap) = self.capture.as_mut() {
            cap.stop();
        }
        let _ = self.sensor.stop_stream();
    }
}
