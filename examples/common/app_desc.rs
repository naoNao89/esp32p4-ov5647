//! ESP-IDF application descriptor, required by the board's second-stage bootloader.
//!
//! The image at flash offset `0x10000` is entered by the ESP-IDF second stage bootloader that
//! this board carries in its `0x0`/`0x2000` flash area. That bootloader reads an `esp_app_desc_t`
//! from the start of the app's first DROM segment — segment #0, which is why the ELF section below
//! exists and why `espflash` bubbles it to the front of the image — so the descriptor is written
//! even though the application itself never reads it.
//!
//! This is a self-contained transcription of the layout emitted by `esp-bootloader-esp-idf`'s
//! `esp_app_desc!()` macro. It is written out here rather than depending on that crate because no
//! pre-v3 build of it is published.
//!
//! Bounds matter: `min_efuse_blk_rev_full` must be `0` and `max_efuse_blk_rev_full`
//! `u16::MAX`, otherwise the bootloader prints `Image requires efuse blk rev >= ...` and refuses
//! to boot the image.

use core::ffi::c_char;

/// `ESP_APP_DESC_MAGIC_WORD`.
const MAGIC_WORD: u32 = 0xABCD_5432;

/// `ESP_APP_DESC` is 256 bytes. Layout from
/// `components/esp_app_format/include/esp_app_desc.h` in ESP-IDF, checked field for field against
/// v5.5.0 (which asserts the same 256-byte size). Newer `master` splits a `spi_flash_mode` byte out
/// of `reserv3`, so the size is unchanged but the last fields are not identical there.
#[repr(C)]
pub struct EspAppDesc {
    magic_word: u32,
    secure_version: u32,
    reserv1: [u32; 2],
    version: [c_char; 32],
    project_name: [c_char; 32],
    time: [c_char; 16],
    date: [c_char; 16],
    idf_ver: [c_char; 32],
    app_elf_sha256: [u8; 32],
    min_efuse_blk_rev_full: u16,
    max_efuse_blk_rev_full: u16,
    mmu_page_size: u8,
    reserv3: [u8; 3],
    reserv2: [u32; 18],
}

impl EspAppDesc {
    pub const fn new(
        version: &str,
        project_name: &str,
        build_time: &str,
        build_date: &str,
        idf_ver: &str,
        mmu_page_size: u32,
        min_efuse_blk_rev_full: u16,
        max_efuse_blk_rev_full: u16,
        secure_version: u32,
    ) -> Self {
        Self {
            magic_word: MAGIC_WORD,
            secure_version,
            reserv1: [0; 2],
            version: str_to_cstr_array(version),
            project_name: str_to_cstr_array(project_name),
            time: str_to_cstr_array(build_time),
            date: str_to_cstr_array(build_date),
            idf_ver: str_to_cstr_array(idf_ver),
            app_elf_sha256: [0; 32],
            min_efuse_blk_rev_full,
            max_efuse_blk_rev_full,
            mmu_page_size: mmu_page_size.ilog2() as u8,
            reserv3: [0; 3],
            reserv2: [0; 18],
        }
    }
}

/// NUL-padded, NUL-terminated copy of `s` into a fixed-size C string array.
const fn str_to_cstr_array<const C: usize>(s: &str) -> [c_char; C] {
    let bytes = s.as_bytes();
    let mut ret = [0 as c_char; C];
    let mut i = 0;
    loop {
        ret[i] = bytes[i] as c_char;
        i += 1;
        if i >= bytes.len() || i >= C {
            break;
        }
    }
    ret
}

/// The descriptor. `espflash` finds it by the `.flash.appdesc` section (or, failing that, by the
/// `esp_app_desc` symbol) — `espflash/src/image_format/idf.rs`; `esp_app_desc` is also the name
/// ESP-IDF gives the same structure.
///
/// `BUILD_TIME`/`BUILD_DATE` came from a `build.rs` in the abandoned bring-up tree; here they are
/// fixed strings, which is all the bootloader checks.
#[export_name = "esp_app_desc"]
#[link_section = ".flash.appdesc"]
#[used]
pub static ESP_APP_DESC: EspAppDesc = EspAppDesc::new(
    env!("CARGO_PKG_VERSION"),
    env!("CARGO_PKG_NAME"),
    "00:00:00",
    "1970-01-01",
    "esp-hal-p4-pre-v3 1.1.0-p4v13.2",
    // ESP32-P4 MMU page size is 64 KiB (`SOC_MMU_PAGE_SIZE` = 0x10000 in `ext_mem_defs.h`).
    64 * 1024,
    0,
    u16::MAX,
    0,
);
