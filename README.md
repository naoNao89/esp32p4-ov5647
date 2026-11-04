# esp32p4-ov5647

A `no_std` Rust driver for the OV5647 (Raspberry Pi Camera Module v1.3)
MIPI CSI-2 sensor on the **ESP32-P4 pre-v3 silicon (chip revision 1.3)**. It
programs the sensor over SCCB, configures the CSI host / D-PHY / ISP / DW-GDMA
directly (upstream `esp-hal` does not yet wrap them for the P4), captures a
frame, and transfers it to a host over USB Serial/JTAG with length and
CRC-32/ISO-HDLC verification.

## Hardware

- SoC: ESP32-P4, **chip revision 1.3 (pre-v3)** — the target this driver runs
  on. Upstream `esp-hal` targets P4 rev 3.x and later; pre-v3 silicon needs the
  pinned pre-v3 HAL and PAC (`esp-hal-p4-pre-v3`, `esp32p4-p4-pre-v3-…`).
- Sensor: OV5647, 2-lane MIPI CSI-2 (Raspberry Pi Camera Module Rev 1.3).
- Console / capture path: native USB Serial/JTAG (`/dev/cu.usbmodem*`).

## Modes

| Mode | Frame | Words | `BLOCK_TS` |
|---|---|---|---|
| RAW8 800x640 | 512,000 bytes | 64,000 64-bit words | 63,999 |
| RAW8 800x800 | 640,000 bytes | 80,000 64-bit words | 79,999 |
| RAW8 800x1280 | 1,024,000 bytes | 128,000 64-bit words | 127,999 |
| RAW10 1280x960 | 1,536,000 bytes packed | 192,000 words | 191,999 |
| RAW10 1920x1080 | 2,592,000 bytes packed | 324,000 words | 323,999 |
| RAW10 2592x1944 | 6,298,560 bytes packed | 787,320 64-bit words | 787,319 |

All six modes are captured end-to-end on hardware and transferred to a host with
CRC check. Frames whose table or window is wrong can still pass that check — a
whole, CRC-correct frame says nothing about exposure or crop — so the mode
documentation records what was measured for each table. The host tool
`tools/capture_frame.py` decodes RAW8 and RAW10, has a `--selftest` (85 checks),
and verifies CRC-32/ISO-HDLC.

## Build

```sh
rustup target add riscv32imafc-unknown-none-elf
```

Build the library **and the capture example** the Flash section below flashes:

```sh
ESP_HAL_CONFIG_MIN_CHIP_REVISION=103 \
  cargo build --release --target riscv32imafc-unknown-none-elf \
  --example capture_raw10_1920x1080
```

That produces
`target/riscv32imafc-unknown-none-elf/release/examples/capture_raw10_1920x1080`.
Use `--examples` to build all four capture examples, or omit the flag to build the
library alone (a plain `cargo build` does **not** produce anything to flash).

`ESP_HAL_CONFIG_MIN_CHIP_REVISION=103` is required for **ESP32-P4 rev 1.3
(pre-v3)** silicon: without it the HAL targets chip revision 300 (v3.x), whose
peripheral base addresses, cache layout and CLIC configuration differ from this
chip. The workspace `Cargo.toml` pins the pre-v3 HAL and PAC and the `riscv32`
rustflags in `.cargo/config.toml` include the `ld/stack-top.x` overlay.

Rustdoc (must be cross-targeted, or the host build cannot compile the pre-v3
HAL's `riscv` register access):

```sh
cargo doc --no-deps --target riscv32imafc-unknown-none-elf
```

## Flash

Install the flashing tool (`espflash` is the Espressif CLI; on macOS via
Homebrew, otherwise `cargo install espflash`):

```sh
brew install espflash
```

Check the board connects before flashing — the reported chip revision must be
`v1.3` (pre-v3):

```sh
espflash board-info --port /dev/cu.usbmodem*
```

Flash an example, forcing the pre-v3 revision (the same revision the HAL build
assumes):

```sh
PORT=$(ls /dev/cu.usbmodem* | head -1)
ELF=target/riscv32imafc-unknown-none-elf/release/examples/capture_raw10_1920x1080
espflash flash --port "$PORT" --non-interactive --min-chip-rev 1.3 "$ELF"
```

The port node changes when the board is replugged. Opening the port resets the
chip, so no separate reset is needed before a capture.

## Capture

The host tool `tools/capture_frame.py` needs `pyserial`:

```sh
pip install pyserial
```

Run its self-test first (85 checks, no hardware required):

```sh
python3 tools/capture_frame.py --selftest
```

Capture one RAW10 1920x1080 frame (2,592,000 bytes). The examples discard
`WARMUP_FRAMES = 8` frames while the sensor's automatic exposure and gain
converge, then transfer the following frame; the default output is a grayscale
PNG:

```sh
PORT=$(ls /dev/cu.usbmodem* | head -1)
python3 tools/capture_frame.py --port "$PORT" \
  --timeout 120 --payload-timeout 120 --out /tmp/raw10
```

Pass `--bayer GBRG` to demosaic the RAW10 into a colour PNG. The Bayer phase is
a user choice and is not verified by this driver; see Known limitations. Use a
different `--out` prefix, or the colour run overwrites the grayscale one:

```sh
python3 tools/capture_frame.py --port "$PORT" \
  --bayer GBRG --timeout 120 --payload-timeout 120 --out /tmp/raw10-color
```

Capture one RAW10 2592x1944 frame (6,298,560 bytes) from
`examples/capture_raw10_2592x1944`. Printing that payload to the console takes
about 7.9 s (roughly 800 kB/s), so `--payload-timeout` has to be much larger:

```sh
PORT=$(ls /dev/cu.usbmodem* | head -1)
python3 tools/capture_frame.py --port "$PORT" \
  --timeout 120 --payload-timeout 600 --out /tmp/raw10-2592
```

Add `--bayer GBRG` for a colour PNG, exactly as in the 1080p command above:

```sh
python3 tools/capture_frame.py --port "$PORT" \
  --bayer GBRG --timeout 120 --payload-timeout 600 --out /tmp/raw10-2592-color
```

`--payload-timeout` must be generous: a 2,592,000-byte payload over USB
Serial/JTAG takes tens of seconds, and the 6,298,560-byte 2592x1944 payload
about 7.9 s (~800 kB/s).

Capture one RAW8 800x1280 frame (1,024,000 bytes) from
`examples/capture_raw8_800x1280` — the mode whose table needed two entries
corrected from measurement (see `src/registers.rs`):

```sh
python3 tools/capture_frame.py --port "$PORT" \
  --timeout 120 --payload-timeout 120 --out /tmp/raw8-800x1280
```

## Version

- `main` declares package `0.1.0` (`Cargo.toml`). Tag `v0.1.0` marks this release.
- The driver targets ESP32-P4 **revision 1.3 (pre-v3)** only; see Hardware above.

## Known limitations

- The Bayer phase (**GBRG**) is a user choice and is not verified; the PNG is
  demosaiced with it, but the table's true phase is an open item.
- The first frames of a fresh stream are dark while the sensor's automatic
  exposure and gain converge. The examples discard `WARMUP_FRAMES = 8` frames
  and transfer the next one. Two fresh-stream sweeps per mode in the 2026-11-04
  session (top-8-bit mean of the packed payload, 20 frames per run) are dark at
  frame 0 — RAW10 1920x1080 read 0.000 in both runs, RAW8 800x640 read 0.015 and
  3.679 — and rise over the next frames: at frame 2 RAW10 reads 53.645 and 50.506,
  RAW8 reads 22.268 and 17.769. From frame 8 to frame 19 every run stays within
  0.6 of its own frame-19 value: RAW10 spans 59.924-60.770 and 55.882-56.615
  (frame 19: 60.318 and 56.069), RAW8 spans 24.554-24.796 and 18.360-18.907
  (frame 19: 24.796 and 18.907). Run 1 of RAW10 is not monotone — frames 5, 6 and
  7 read 60.664, 57.636 and 57.201 before frame 8 returns to 60.770 — so the
  series is described by that band and not by a convergence frame. The band's
  height is scene- and lighting-dependent: the two runs of this session sit about
  7% apart for RAW10 and about 25% apart for RAW8, and neither matches the
  bring-up notes (40-43 in three captures minutes earlier in that session, ~121 in
  a brighter one). These sweeps' raw logs are not kept in this repository.
- Earlier release logs recorded a DISCARD on the first frame of a stream. With
  this revision that does not reproduce: seven fresh streams (7,007 frames, no
  warm-up ahead of them) each reported `FrameInfo.bridge_first_discard` = `None`
  on every frame, frame 0 included, and the raw bridge latch read
  `INT_RAW & DISCARD` = 0 throughout (frame 0 = `0x00020`, bit 5 only). The
  sampling path was live in the same window (`bridge_first_depth` set on
  1,001/1,001 frames, `words` complete on every frame). This is what was
  observed in this build; it is not an explanation of the earlier log.
- The CSI bridge DISCARD interrupt is sampled only during `wait_frame()` and in
  `start()`, so a DISCARD outside those windows would not be recorded.
- Cadences measured on this board have not been shown to be the sensor's own
  frame periods, and no independent time source has measured what the sensor
  delivers. A 1,001-frame RAW10 soak reaches the nominal 30 fps when nothing else
  runs: 33.359 s in each of two runs, i.e. 30.01 fps and 33,325.9 us per frame
  (p50 cadence 33,323 us), and the interval's p50/p95/p99 are the same with and
  without the instrumentation below. The release log's 28.04 fps is reproduced in
  magnitude, as the *cost of instrumentation* rather than a minimal-workload
  figure: an instrumented soak took 35.747 s in each of two runs here (28.00 fps,
  35,711.2 us per frame) against 35.692 s in an earlier run of the same
  configuration (28.04 fps), so the logged value falls inside that configuration's
  spread while the log's own workload stays unrecorded. The 2.388 s difference is
  broken down by measurement, not inferred: the checkpoints account for 2.293 s
  (254.8 ms each — 153.9 ms histogram, 96.2 ms `nonzero`, 4.6 ms for six SCCB
  reads, 0.02 ms status snapshot), and the remaining 94.8 ms is the frames that
  follow a checkpoint (their interval is ~299.9 ms against the 33.32 ms median:
  90.9 ms) plus 3.6 ms of re-arm and 0.3 ms of loop bookkeeping. The log labels
  its 28.04 as minimal workload and its 25.9 as instrumented, and neither label
  reproduces here — so what is established is the cost of that instrumentation,
  not how the logged run was measured. The `25.9 fps` figure has no definition in
  any note. The RAW8 800x640 frame interval measured p50 27,985 us and p99
  28,031-28,034 us (the log's 27,993 / 28,020), i.e. ~35.7 fps against its
  nominal 50. The frame rates in the mode names are nominal, not measured.

## License

This repository is `MIT OR Apache-2.0` (`Cargo.toml`), at your option:
[LICENSE-MIT](LICENSE-MIT) or [LICENSE-APACHE](LICENSE-APACHE).

Third-party material redistributed here:

| Material | Upstream | License |
|---|---|---|
| Five OV5647 2-lane MIPI mode tables, plus the reset and stream sequences (`src/registers.rs`, tags `S1`–`S5`, `S10`, `S12`, `S14`) | `espressif/esp-video-components`, `esp_cam_sensor/sensors/ov5647/` | Apache-2.0 |
| 38-entry D-PHY PLL range table (`hs_freq_sel_for_mbps`, `src/csi.rs`) | ESP-IDF, `components/soc/esp32p4/mipi_csi_periph.c` | Apache-2.0 |
| 256-byte `esp_app_desc_t` layout (`examples/common/app_desc.rs`) | `esp-bootloader-esp-idf` (esp-rs); layout also defined by ESP-IDF `esp_app_desc.h` | MIT OR Apache-2.0 / Apache-2.0 |
| HAL, ROM and PAC register definitions (`esp-hal-p4-pre-v3`, `esp-rom-sys-p4-pre-v3`, `esp32p4-p4-pre-v3-5a07030`) | `esp-rs/esp-hal`, `esp-rs/esp-pacs` | MIT OR Apache-2.0 |

The full attribution notices, with the revision each item was checked against,
are in [NOTICE](NOTICE).

The OV5647 datasheet and the OmniVision SCCB specification are cited as
references only. Both are marked proprietary by OmniVision and neither file is
redistributed. Beyond register names, bit-field names and short field
descriptions, no extract from either document is reproduced here.

Only the sources listed above are attributed for `src/registers.rs`. The
2592x1944 mode is derived here from the OV5647 datasheet and the Apache-2.0
esp-video reference, and the 800x1280 table is esp-video's with two entries
corrected from measurement; `NOTICE`, section 6, records the provenance of every
table.
