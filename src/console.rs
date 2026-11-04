//! Console output for ESP32-P4 rev 1.3 bring-up.
//!
//! The only console path on this setup is the SoC's **native USB Serial/JTAG**
//! (`USB_DEVICE`, base `0x500d_2000`); there is no UART bridge attached, so UART0
//! output would not be observable.
//!
//! Bytes are pushed straight into the CDC-ACM IN FIFO through the PAC rather
//! than through `esp_hal::usb::usb_serial_jtag`. That is deliberate and was
//! established by experiment on this board, not preference — three builds of
//! the same program:
//!
//! | firmware      | `esp_hal::init` | transport           | output |
//! |---------------|-----------------|---------------------|--------|
//! | `raw-probe`   | no              | raw PAC FIFO        | works  |
//! | `init-probe`  | yes             | raw PAC FIFO        | works  |
//! | `console-probe` | yes           | HAL `UsbSerialJtag` | silent |
//!
//! The HAL path (`write_byte_nb` + `flush_tx_nb`, which sets `WR_DONE` on every
//! call) leaves the application silent on this silicon, while the same program
//! writing the same registers directly prints normally.
//!
//! Two properties are deliberately preserved from that experiment:
//!
//! * **Bounded waiting.** Each byte waits for `SERIAL_IN_EP_DATA_FREE`, but only
//!   for a bounded number of polls. A host that has not attached yet, or has
//!   gone away, must never park the boot path: a blocked console is
//!   indistinguishable from a crashed application.
//! * **No dropping while the host attaches.** The budget is large enough to
//!   cover the USB re-enumeration the SoC performs when the application takes
//!   the peripheral over, so early messages are not lost.
//!
//! `esp-println` is deliberately avoided: its `critical-section` path pulls
//! `esp-sync`, whose upstream implementation emits CSR `0x347` (`mintthresh`),
//! which traps as an illegal instruction on pre-v3 silicon.

use core::cell::RefCell;

use critical_section::Mutex;
use esp_hal::peripherals::USB_DEVICE;

/// Poll budget per byte. Generous enough to cover the USB re-enumeration that
/// happens while the host attaches, small enough that a detached host cannot
/// stall the main loop for long.
const SPIN_BUDGET: u32 = 20_000_000;

/// Token type for the console peripheral.
pub type Console = USB_DEVICE<'static>;

static INITIALISED: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

/// Takes ownership of the USB Serial/JTAG peripheral for the program's lifetime.
///
/// The peripheral is left exactly as the second stage bootloader left it: the
/// ROM has already configured and enumerated it, and re-initialising it is what
/// loses the host connection.
///
/// Blocks briefly before returning. When the application takes the peripheral
/// over from the second stage bootloader the USB device re-enumerates, and any
/// bytes pushed into the IN FIFO before the host has re-attached are discarded
/// with it. Without this settle period the *first* messages of a program are
/// reliably lost — which reads exactly like an application that crashed before
/// printing anything. It cost hours to find; do not remove it.
pub fn init(console: Console) {
    core::mem::forget(console);

    let delay = esp_hal::delay::Delay::new();
    delay.delay_millis(1500);

    critical_section::with(|cs| {
        *INITIALISED.borrow_ref_mut(cs) = true;
    });
}

#[inline(always)]
fn regs() -> &'static esp32p4::usb_device::RegisterBlock {
    // Safety: the peripheral singleton is owned for the program's lifetime by
    // `init`, and this is the only module that touches these registers.
    unsafe { &*esp32p4::USB_DEVICE::ptr() }
}

/// Pushes raw bytes into the CDC-ACM IN FIFO.
///
/// Returns the number of bytes accepted; a short count means the FIFO stopped
/// draining within the per-byte budget.
pub fn write_bytes(bytes: &[u8]) -> usize {
    let mut written = 0usize;
    if !critical_section::with(|cs| *INITIALISED.borrow_ref(cs)) {
        return 0;
    }
    critical_section::with(|_cs| {
        let regs = regs();
        for &byte in bytes {
            let mut spins = 0u32;
            while regs
                .ep1_conf()
                .read()
                .serial_in_ep_data_free()
                .bit_is_clear()
            {
                spins += 1;
                if spins > SPIN_BUDGET {
                    return written;
                }
                core::hint::spin_loop();
            }
            regs.ep1().write(|w| unsafe { w.rdwr_byte().bits(byte) });
            written += 1;
        }
        regs.ep1_conf().modify(|_, w| w.wr_done().set_bit());
        written
    })
}

#[doc(hidden)]
pub fn _print(args: core::fmt::Arguments) {
    let _ = core::fmt::Write::write_fmt(&mut FmtWriter, args);
}

struct FmtWriter;

impl core::fmt::Write for FmtWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write_bytes(s.as_bytes());
        Ok(())
    }
}

/// Writes a line to the USB Serial/JTAG console, with CRLF line endings.
#[macro_export]
macro_rules! println {
    () => {{
        $crate::console::log_line(format_args!(""));
    }};
    ($($arg:tt)*) => {{
        $crate::console::log_line(format_args!($($arg)*));
    }};
}

/// Writes to the USB Serial/JTAG console without a trailing newline.
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        $crate::console::_print(format_args!($($arg)*));
    }};
}

/// A [`crate::frame::FrameSink`] over the USB Serial/JTAG console.
///
/// Lets [`crate::frame::transfer`] emit a frame (header, payload, trailer) through the same
/// bounded `write_bytes` path the bring-up used. The bytes go out as-is; the host's
/// `tools/capture_frame.py` knows the framing.
#[derive(Debug, Clone, Copy)]
pub struct ConsoleSink;

impl crate::frame::FrameSink for ConsoleSink {
    fn write_bytes(&mut self, bytes: &[u8]) -> usize {
        crate::console::write_bytes(bytes)
    }

    fn write_text(&mut self, text: &str) -> bool {
        crate::console::write_bytes(text.as_bytes()) == text.len()
    }
}

/// Line-oriented logger: every message ends with CRLF so that a terminal and a
/// line-buffered host script both see complete lines (the USB Serial/JTAG
/// console does not translate newlines).
pub fn log_line(args: core::fmt::Arguments) {
    _print(args);
    write_bytes(b"\r\n");
}

/// Flushes and idles. Used by the panic handler and by fatal error paths.
pub fn halt() -> ! {
    loop {
        core::hint::spin_loop();
    }
}
