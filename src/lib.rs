//! OV5647 MIPI CSI-2 camera driver for ESP32-P4 **pre-v3** silicon (chip revision 1.3).
//!
//! This crate drives an OV5647 sensor through the ESP32-P4's SCCB (I2C), MIPI CSI-2 host, CSI
//! bridge, ISP and DW-GDMA on silicon **revision 1.3**. It is a library so a board application
//! (see the `examples/` directory) can own the hardware initialization — the pin choices, the
//! PSRAM configuration, the stack and clock setup — while this crate owns the camera control and
//! capture pipeline. Two modes are verified end to end on this silicon, 800x640 RAW8 and
//! 1920x1080 RAW10; see [`CameraMode`].
//!
//! # Hardware revision caveat
//!
//! The pre-v3 ESP32-P4 (revision 1.3) differs from revision 3.x in peripheral base addresses and
//! cache layout. The driver is written against the pre-v3 HAL and PAC and will **not** work
//! unchanged on revision 3.x silicon; [`crate::csi`] documents the revision-1.3 facts this driver
//! depends on, among them the 64-byte data-cache line and a CSI bridge whose colour-conversion
//! block does not exist.
//!
//! # Safety
//!
//! The camera pipeline writes captured frames into memory that is **caller-owned**. The driver uses
//! unsafe only where the hardware (DMA) needs raw addresses and cache maintenance on buffers that
//! the caller has placed correctly. The public API is safe, but correctness depends on the caller
//! providing a buffer that is:
//!
//! * aligned to a 64-byte cache line,
//! * large enough for one frame,
//! * reachable by the DW-GDMA (for example, in PSRAM),
//! * not in use by something else while a transfer is armed.
//!
//! # Modules
//!
//! * [`crate::mode`] and [`crate::camera`] — the typed mode selector and the lifecycle object that
//!   ties the sensor to the capture pipeline.
//! * [`crate::sensor`] — SCCB transport, identification, register tables, stream control.
//! * [`crate::csi`] — the CSI host, bridge, ISP configuration, and frame completion/re-arm.
//! * [`crate::isp`], [`crate::gdma`], [`crate::framebuffer`] — the ISP bypass/geometry, the
//!   DW-GDMA channel, and buffer ownership/alignment.
//! * [`crate::frame`] — frame analysis and transfer to a host.
//! * [`crate::regs`] / [`crate::registers`] — the OV5647 register addresses and the mode tables.
//!
//! The hardware-independent core ([`ov5647_core`]) holds the geometry, DMA bounds, frame integrity
//! and CRC logic that is unit-tested on the host.

#![no_std]
#![warn(missing_docs)]
#![deny(rustdoc::broken_intra_doc_links)]

pub mod camera;
pub mod console;
pub mod csi;
pub mod error;
pub mod format;
pub mod frame;
pub mod framebuffer;
pub mod gdma;
pub mod isp;
pub mod mode;
pub mod registers;
pub mod regs;
pub mod sccb;
pub mod sensor;

pub use camera::Camera;
pub use csi::CsiCapture;
pub use error::CsiError;
pub use format::PixelFormat;
pub use frame::{FrameSink, TransferError, TransferReport};
pub use mode::CameraMode;
pub use sensor::{ModeSpec, Ov5647};

/// The `ov5647-core` crate, re-exported for convenience so a user of this crate does not need to
/// also depend on it directly.
pub use ov5647_core;
