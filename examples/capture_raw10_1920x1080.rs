//! Example: capture one 1920x1080 RAW10 frame and transfer it to the host.
//!
//! This is the minimal board application that exercises the library's public API end to end with
//! the 1080p mode: it configures the board (PSRAM, SCCB pins, console), constructs a [`Camera`]
//! with [`CameraMode::Raw10_1920x1080`], discards `WARMUP_FRAMES` frames while the sensor's
//! automatic exposure and gain converge, then waits for one whole frame in a caller-owned PSRAM
//! buffer and prints the frame header so a host can pull the raw packed RAW10 payload.
//!
//! Hardware required: ESP32-P4 **revision 1.3** (pre-v3) with an OV5647 on the CSI connector and
//! the board's shared `ESP_I2C` SCCB bus (SDA=GPIO7, SCL=GPIO8).
//!
//! The RAW10 payload is *packed*: 4 pixels per 5 bytes. 2,592,000 bytes per frame. The host tool
//! (`tools/capture_frame.py --bayer GBRG`) unpacks it into 10-bit samples.
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

/// 2,592,000 bytes for one RAW10 frame, plus a 4 KiB guard granule.
const FRAME_LEN: usize = 2_592_000;
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

    let camera = match Camera::new(sccb, delay, CameraMode::Raw10_1920x1080, {
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
    // Measured on this board in this mode (top-8-bit mean of the packed payload),
    // two 20-frame sweeps of fresh streams: frame 0 = 0.000 in both, frame 1 =
    // 47.981 and 34.743, frame 2 = 53.645 and 50.506. Frames 8-19 stay within 0.6
    // of the frame-19 value in each run — 59.924-60.770 and 55.882-56.615 against
    // 60.318 and 56.069 — while run 1 is not monotone: frames 5, 6 and 7 read
    // 60.664, 57.636, 57.201 before frame 8 returns to 60.770. The band's height is
    // scene- and lighting-dependent: these two runs sit ~7% apart, and neither
    // matches the bring-up notes (40–43 minutes earlier, ~121 in a brighter
    // session). `0` restores the previous behaviour of transferring frame 0, the
    // darkest one.
    //
    // Those sweeps' console output was not archived: the figures are the ones
    // recorded in the 2026-11-04 session, not a log shipped with this repository.
    //
    // 8 is a conservative default — the band above holds from frame 8 in both runs —
    // not a claim that every OV5647 or every mode settles within this many frames.
    const WARMUP_FRAMES: usize = 8;

    let mut camera = camera;

    // Stream, then re-arm once per frame and discard `WARMUP_FRAMES` of them so
    // that the transferred frame is past the exposure transient. Each iteration's
    // `start()` re-issues the stream-on register writes (`0x4800`/`0x0100`); the
    // sensor is not stopped and the mode table is not reapplied.
    for i in 0..=WARMUP_FRAMES {
        if let Err(e) = camera.start() {
            esp32p4_ov5647::console::_print(format_args!("frame {i}: start failed: {e:?}\r\n"));
            loop {}
        }

        // RAW10 at 30 fps nominal => ~33 ms/frame; generous timeout.
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
                    width: CameraMode::Raw10_1920x1080.width(),
                    height: CameraMode::Raw10_1920x1080.height(),
                    format: CameraMode::Raw10_1920x1080.format(),
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
