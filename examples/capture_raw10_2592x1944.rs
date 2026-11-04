//! Example: capture one 2592x1944 RAW10 frame and transfer it to the host.
//!
//! This is the minimal board application that exercises the library's public API end to end with
//! the full-resolution mode: it configures the board (PSRAM, SCCB pins, console), constructs a
//! [`Camera`] with [`CameraMode::Raw10_2592x1944`], discards `WARMUP_FRAMES` frames while the
//! sensor's automatic exposure and gain converge, then waits for one whole frame in a caller-owned
//! PSRAM buffer and prints the frame header so a host can pull the raw packed RAW10 payload.
//!
//! Hardware required: ESP32-P4 **revision 1.3** (pre-v3) with an OV5647 on the CSI connector and
//! the board's shared `ESP_I2C` SCCB bus (SDA=GPIO7, SCL=GPIO8).
//!
//! The RAW10 payload is *packed*: 4 pixels per 5 bytes, 2592 * 1944 * 10 / 8 = 6,298,560 bytes per
//! frame. The host tool (`tools/capture_frame.py`) unpacks it into 10-bit samples.
//!
//! Measured on this board: every frame completes with `words=787320` (= `BLOCK_TS` 787,319), and
//! printing the 6,298,560-byte payload to the console takes about 7.9 s (~800 kB/s over
//! USB-Serial-JTAG), so the host side needs a generous `--payload-timeout`.
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

/// 6,298,560 bytes for one RAW10 frame, plus a 4 KiB guard granule.
const FRAME_LEN: usize = 6_298_560;
const GRANULE: usize = 4096;
const RESERVE: usize = FRAME_LEN + GRANULE;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[esp_hal::main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp32p4_ov5647::console::init(peripherals.USB_DEVICE);

    let delay = Delay::new();

    // ---- PSRAM first (its init clears the D-PHY LDO rail). ----------
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

    // SAFETY: page-aligned, inside the published PSRAM range, not aliased.
    let buf: &mut [u8] = unsafe { core::slice::from_raw_parts_mut(base as *mut u8, RESERVE) };

    // ---- Camera on the SCCB bus ------------------------------------
    let sccb = SccbGpio::new(
        unsafe { GPIO7::steal() },
        unsafe { GPIO8::steal() },
        Delay::new(),
    );

    let camera = match Camera::new(sccb, delay, CameraMode::Raw10_2592x1944, {
        DmaFrameBuffer::new(&mut buf[..RESERVE], FRAME_LEN).expect("buffer validation")
    }) {
        Ok(cam) => cam,
        Err(e) => {
            esp32p4_ov5647::console::_print(format_args!("camera init failed: {e:?}\r\n"));
            loop {}
        }
    };

    // Frames discarded while the sensor's automatic exposure and gain converge; the frame after
    // them is the one transferred.
    //
    // No per-frame convergence sweep has been run for this mode, so the default is the value the
    // 1080p RAW10 example settled on from its own measurement (mean plateaued by ~frame 4 over a
    // 20-frame sweep; 8 is comfortably past it), not a measurement of this table. `0` transfers
    // the first frame, which is the darkest one. Measured here: all 9 frames of a run complete
    // with `words=787320`.
    const WARMUP_FRAMES: usize = 8;

    let mut camera = camera;

    // Stream, then re-arm once per frame and discard `WARMUP_FRAMES` of them. Each iteration's
    // `start()` re-issues the stream-on register writes (`0x4800`/`0x0100`); the sensor is not
    // stopped and the mode table is not reapplied.
    for i in 0..=WARMUP_FRAMES {
        if let Err(e) = camera.start() {
            esp32p4_ov5647::console::_print(format_args!("frame {i}: start failed: {e:?}\r\n"));
            loop {}
        }

        // 15 fps nominal => ~67 ms/frame; generous timeout.
        let frame_info = camera.wait_frame(esp_hal::time::Duration::from_millis(2000));
        match frame_info {
            Ok(info) => {
                if i < WARMUP_FRAMES {
                    // Deliberately not read: the next `start()` overwrites this buffer, and holding
                    // the slice across the re-arm would borrow the camera for the rest of the loop.
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
                    width: CameraMode::Raw10_2592x1944.width(),
                    height: CameraMode::Raw10_2592x1944.height(),
                    format: CameraMode::Raw10_2592x1944.format(),
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
                // No frame means no way to reach the frame that is meant to be transferred; stop
                // rather than re-arm a pipeline in an unknown state.
                loop {}
            }
        }
    }

    loop {}
}
