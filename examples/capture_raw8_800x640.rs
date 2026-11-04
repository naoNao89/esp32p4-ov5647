//! Example: capture one 800x640 RAW8 frame and transfer it to the host.
//!
//! This is the minimal board application that exercises the library's public API end to end: it
//! configures the board (PSRAM, SCCB pins, console), constructs a [`Camera`] with the
//! [`CameraMode::Raw8_800x640`] mode, discards `WARMUP_FRAMES` frames while the sensor's automatic
//! exposure and gain converge, then waits for one whole frame in a caller-owned PSRAM buffer and
//! prints the frame header so a host can pull the raw payload.
//!
//! Hardware required: ESP32-P4 **revision 1.3** (pre-v3) with an OV5647 on the CSI connector and
//! the board's shared `ESP_I2C` SCCB bus (SDA=GPIO7, SCL=GPIO8).
//!
//! # Safety
//!
//! The example arranges a PSRAM buffer via `Psram::raw_parts()`; the slice it constructs meets the
//! [`DmaFrameBuffer`] contract (page-aligned, inside the published PSRAM range, not aliased). This
//! is the one place unsafe is needed — the library's public API is safe.

#![no_std]
#![no_main]

#[path = "common/app_desc.rs"]
mod app_desc;

use esp_hal::delay::Delay;
use esp_hal::peripherals::{GPIO7, GPIO8, PSRAM};
use esp_hal::psram::{Psram, PsramConfig, PsramMode, PsramSize, SpiRamFreq};

use esp32p4_ov5647::framebuffer::DmaFrameBuffer;
use esp32p4_ov5647::sccb::SccbGpio;
use esp32p4_ov5647::{Camera, CameraMode};

/// 2 MiB PSRAM is enough for a 512,000-byte RAW8 frame with room to spare.
const FRAME_LEN: usize = 512_000;
/// Frame region page-aligned up from PSRAM's published base, plus one 4 KiB guard granule.
const GRANULE: usize = 4096;
/// Total reservation: frame + one guard granule.
const RESERVE: usize = FRAME_LEN + GRANULE;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal::main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Console over USB Serial/JTAG.
    esp32p4_ov5647::console::init(peripherals.USB_DEVICE);

    let delay = Delay::new();

    // ---- PSRAM first: its init clears the MIPI D-PHY LDO rail, so it must
    // run before the capture (which re-enables the LDO) is constructed. ----
    let psram_config = PsramConfig {
        mode: PsramMode::Hex,
        size: PsramSize::AutoDetect,
        core_clock: None,
        ram_frequency: SpiRamFreq::Mhz200,
    };
    let psram = Psram::new(unsafe { PSRAM::steal() }, psram_config);
    let (raw_ptr, available) = psram.raw_parts();
    let base = ((raw_ptr as usize) + GRANULE - 1) & !(GRANULE - 1);
    assert!(
        base + RESERVE <= raw_ptr as usize + available,
        "PSRAM too small"
    );

    // SAFETY: `[base, base+RESERVE)` is page-aligned, inside the published PSRAM
    // range, and not aliased; the slice below is the only reference to it.
    let buf: &mut [u8] = unsafe { core::slice::from_raw_parts_mut(base as *mut u8, RESERVE) };

    // ---- Camera on the SCCB bus (software bit-banged SCCB) ------------------
    // The library's SCCB transport takes two pins that implement `Pin` and a
    // `Delay`. The pins are stolen rather than taken from `peripherals`, matching
    // the reference bring-up (the interrupts are not used here).
    let sccb = SccbGpio::new(
        unsafe { GPIO7::steal() },
        unsafe { GPIO8::steal() },
        Delay::new(),
    );

    let frame = match Camera::new(sccb, delay, CameraMode::Raw8_800x640, {
        // The DmaFrameBuffer validates alignment/reachability/frame-fit.
        DmaFrameBuffer::new(&mut buf[..RESERVE], FRAME_LEN).expect("buffer validation")
    }) {
        Ok(cam) => cam,
        Err(e) => {
            esp32p4_ov5647::console::_print(format_args!("camera init failed: {e:?}\r\n"));
            loop {}
        }
    };

    // Frames discarded while the sensor's automatic exposure and gain converge;
    // the frame after them is the one transferred.
    //
    // Same-session sweeps on this board (top-8-bit mean of the packed payload,
    // two 20-frame fresh streams): frame 0 = 0.015 and 3.679, frame 2 = 22.268 and
    // 17.769. Frames 8-19 stay within 0.6 of the frame-19 value in each run —
    // 24.554-24.796 and 18.360-18.907 against 24.796 and 18.907. The band's height
    // is scene- and lighting-dependent: the two runs sit ~25% apart, and the
    // bring-up notes recorded 4.7 at frame 0 and 12.0 at frame 8 in their session.
    // Those sweeps' console output is unarchived. `0` restores the previous
    // behaviour of transferring frame 0, which is the darkest one. 8 is a
    // conservative default inherited from the RAW10 example rather than the
    // minimum this mode needs.
    const WARMUP_FRAMES: usize = 8;

    let mut camera = frame;

    // Stream, then re-arm once per frame and discard `WARMUP_FRAMES` of them so
    // that the transferred frame is past the exposure transient. Each iteration's
    // `start()` re-issues the stream-on register writes (`0x4800`/`0x0100`); the
    // sensor is not stopped and the mode table is not reapplied.
    for i in 0..=WARMUP_FRAMES {
        if let Err(e) = camera.start() {
            esp32p4_ov5647::console::_print(format_args!("frame {i}: start failed: {e:?}\r\n"));
            loop {}
        }

        // Wait one frame (RAW8 at 50 fps nominal => ~20 ms; generous timeout).
        let frame_info = camera.wait_frame(esp_hal::time::Duration::from_millis(2000));
        match frame_info {
            Ok(info) => {
                if i < WARMUP_FRAMES {
                    // Deliberately not read: the next `start()` overwrites this
                    // buffer, and holding the slice across the re-arm would borrow
                    // the camera for the rest of the loop.
                    esp32p4_ov5647::console::_print(format_args!(
                        "warm-up {i}: words={} complete={}\r\n",
                        info.words,
                        info.complete()
                    ));
                    continue;
                }
                if !info.complete() {
                    // A short frame is not a frame to transfer and hope about.
                    esp32p4_ov5647::console::_print(format_args!(
                        "frame {i} incomplete: words={} expected={}\r\n",
                        info.words, info.expected_words
                    ));
                    loop {}
                }
                esp32p4_ov5647::console::_print(format_args!(
                    "frame complete: words={} expected={} complete={}\r\n",
                    info.words,
                    info.expected_words,
                    info.complete()
                ));

                // Build the header the host expects and transfer the frame payload.
                let bytes = camera.frame_bytes();
                let crc = ov5647_core::crc32_ieee(bytes);
                let summary = esp32p4_ov5647::frame::FrameSummary {
                    width: CameraMode::Raw8_800x640.width(),
                    height: CameraMode::Raw8_800x640.height(),
                    format: CameraMode::Raw8_800x640.format(),
                    len: bytes.len(),
                    crc32: crc,
                    nonzero_bytes: bytes.iter().filter(|&&b| b != 0).count(),
                    distinct_bytes: {
                        let mut seen = [false; 256];
                        let mut n = 0u16;
                        for &b in bytes {
                            if !seen[b as usize] {
                                seen[b as usize] = true;
                                n += 1;
                            }
                        }
                        n
                    },
                    elapsed_ms: 0,
                };
                let header = summary.header();
                let mut sink = esp32p4_ov5647::console::ConsoleSink;
                match esp32p4_ov5647::frame::transfer(&mut sink, &header, bytes) {
                    Ok(report) if report.complete() => {
                        esp32p4_ov5647::console::_print(format_args!(
                            "transfer complete: {}\r\n",
                            report.payload_bytes_written
                        ));
                    }
                    Ok(report) => {
                        esp32p4_ov5647::console::_print(format_args!(
                            "transfer aborted at {}\r\n",
                            report.payload_bytes_written
                        ));
                    }
                    Err(e) => {
                        esp32p4_ov5647::console::_print(format_args!("transfer error: {e:?}\r\n"));
                    }
                }
            }
            Err(e) => {
                esp32p4_ov5647::console::_print(format_args!(
                    "frame {i}: wait_frame failed: {e:?}\r\n"
                ));
                // No frame means no way to reach the frame that is meant to be
                // transferred; stop rather than re-arm a pipeline in an unknown state.
                loop {}
            }
        }
    }

    loop {}
}
