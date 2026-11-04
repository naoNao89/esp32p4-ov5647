#!/usr/bin/env python3
"""Capture a frame from the ESP32-P4 OV5647 firmware over USB Serial/JTAG.

The firmware announces each frame as one text header line, the raw payload, and
a trailer::

    <<<FRAME w=800 h=640 fmt=RAW8 len=512000 crc32=0x1A2B3C4D>>>
    <payload_len raw bytes>
    <<<END>>>

This script scans the console (``/dev/cu.usbmodem*`` unless ``--port`` is given)
for that header, parses it with exactly the rules
``crates/ov5647-core/src/lib.rs`` (the ``<FrameHeader`` ``render`` for the wire
header, ``crc32_ieee`` for the CRC) uses to render it, reads
``len`` bytes, verifies CRC-32/ISO-HDLC and the length, then writes a raw dump
and a viewable PNG (grayscale for RAW8, 16-bit grayscale for unpacked RAW10, RGB
for RGB565/RGB888). PNG is written by hand with ``zlib`` + ``struct`` so the only
dependency is pyserial, and only for actual captures.

RAW8/RAW10 are demosaiced only when asked: the OV5647's Bayer phase is *not*
settled for this board (the Linux driver says BGGR, the Espressif vendor driver
says GBRG, and the IDF crop can shift it again), so ``--bayer`` takes an explicit
``RGGB``/``BGGR``/``GBRG``/``GRBG`` and the default is plain grayscale. Nothing is
assumed about the phase.

The ESP32-P4's USB Serial/JTAG peripheral re-enumerates when the board resets, so
the port can disappear mid-session or change name. The reader treats a vanished
port as a transport error, re-opens it (re-scanning ``/dev/cu.usbmodem*``) within a
bounded deadline, and restarts the frame scan from a clean buffer. Bounded
timeouts apply to the header, the payload, the trailer and each re-open, so no
code path waits forever.

Error classes are reported distinctly, because they mean different things on the
bench:

* **communication** -- port could not be opened (or re-opened), the header never
  arrived, or the payload read ended short. Nothing about the frame's content is
  implied.
* **header** -- a header-like line that violates the framing rules (unknown
  field or format token, unparsable value, or a ``len`` that disagrees with its
  own geometry). The firmware sent a mislabelled frame.
* **checksum** -- the full payload arrived but its CRC-32 (or length) does not
  match the header. The bytes are complete, the content is wrong.
* **incomplete** -- the payload arrived complete and CRC-verified, but the
  ``<<<END>>>`` trailer never did. The frame content is trustworthy; the stream
  was cut afterwards. Exit status 3, and the verified outputs are still written.
* **image** -- the payload verified but cannot be rendered as a PNG (e.g. a
  RAW10 width that is not a multiple of the 4-pixel packing group, or a Bayer
  request for an RGB format).

Usage:
    python3 tools/capture_frame.py [--port PORT] [--out PREFIX] [--timeout S]
                                   [--bayer none|RGGB|BGGR|GBRG|GRBG]
    python3 tools/capture_frame.py --selftest
"""

from __future__ import annotations

import argparse
import contextlib
import glob
import io
import os
import struct
import sys
import tempfile
import time
import zlib

try:
    import serial
except ImportError:  # Only needed for a real capture; --selftest stays usable.
    serial = None  # type: ignore[assignment]



# --------------------------------------------------------------------------- #
# Wire framing: mirrors crates/ov5647-core/src/lib.rs
# --------------------------------------------------------------------------- #

FRAME_HEADER_PREFIX = "<<<FRAME"
FRAME_TRAILER = "<<<END>>>"

#: Format token -> bits per pixel, i.e. `PixelFormat::name()` / `from_name()`.
FORMAT_BITS = {
    "RAW8": 8,
    "RAW10": 10,
    "RGB565": 16,
    "RGB888": 24,
}

_U32_MAX = 0xFFFF_FFFF
_HEX_DIGITS = set("0123456789abcdefABCDEF")

#: CFA 2x2 tile per Bayer order, ``[row & 1][col & 1]``. The OV5647's phase on
#: this board is *not* verified (see the module docstring), so this is a lookup
#: table and never a default.
BAYER_ORDERS = {
    "RGGB": (("R", "G"), ("G", "B")),
    "BGGR": (("B", "G"), ("G", "R")),
    "GBRG": (("G", "B"), ("R", "G")),
    "GRBG": (("G", "R"), ("B", "G")),
}

#: ``--bayer`` value meaning "leave the CFA plane as grayscale".
BAYER_NONE = "none"


class CommError(Exception):
    """Transport-level failure: port, timeout, or a short read."""


class HeaderError(CommError):
    """A header-like line that breaks the framing rules.

    A subclass of :class:`CommError` because it means the stream could not be
    interpreted at all, but it is reported separately: the transport worked and
    the firmware sent something malformed.
    """


class ChecksumError(Exception):
    """The complete payload does not match the header's length or CRC-32."""


class IncompleteFrameError(Exception):
    """The payload verified, but the ``<<<END>>>`` trailer never arrived.

    This is the host-side image of the firmware's ``TransferReport`` with
    ``trailer_sent = false`` (``src/frame.rs``): when the console stops
    accepting bytes mid-frame the firmware skips the trailer on purpose so the
    host sees a short frame rather than a closed one.

    Deliberately *not* a :class:`CommError`: the frame's bytes are complete and
    CRC-verified, so this says nothing about the payload's quality -- only that
    the stream stopped right after it. The verified ``header`` and ``payload``
    travel on the exception so a caller can still keep them.
    """

    def __init__(self, message: str, header: dict, payload: bytes):
        super().__init__(message)
        self.header = header
        self.payload = payload


class ImageError(Exception):
    """The verified payload cannot be rendered as a PNG."""


def frame_len_bytes(fmt: str, width: int, height: int) -> int:
    """Bytes a full frame occupies on the wire; mirrors `frame_len_bytes`.

    Unlike the firmware's `u32` return, this never truncates: an absurd geometry
    whose byte count exceeds ``u32::MAX`` is reported as such, and the header
    check then rejects the line instead of trying to read gigabytes.
    """
    return (width * height * FORMAT_BITS[fmt]) // 8


def crc32_ieee(data: bytes) -> int:
    """CRC-32/ISO-HDLC (reflected, init/final-xor 0xFFFF_FFFF).

    A bit-for-bit port of `crc32_ieee()` in `core/src/lib.rs`, so a mismatch
    between the two implementations cannot hide behind a matching library call.
    """
    crc = 0xFFFF_FFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            mask = (-(crc & 1)) & _U32_MAX
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask)
    return crc ^ 0xFFFF_FFFF


def _parse_u32(text: str) -> int:
    """`u32::from_str` semantics: ASCII digits, optional leading `+`, <= 2**32-1."""
    body = text[1:] if text[:1] == "+" else text
    if not body or any(ch not in "0123456789" for ch in body):
        raise ValueError(f"not a u32: {text!r}")
    value = int(body)
    if value > _U32_MAX:
        raise ValueError(f"u32 overflow: {text!r}")
    return value


def _parse_hex_u32(text: str) -> int:
    """`u32::from_str_radix(strip_prefix("0x"), 16)` semantics."""
    if not text.startswith("0x"):
        raise ValueError(f"missing 0x prefix: {text!r}")
    body = text[2:]
    if not body or any(ch not in _HEX_DIGITS for ch in body):
        raise ValueError(f"not a hex u32: {text!r}")
    value = int(body, 16)
    if value > _U32_MAX:
        raise ValueError(f"u32 overflow: {text!r}")
    return value


def render_header_line(width: int, height: int, fmt: str, payload_len: int, crc32: int) -> str:
    """`FrameHeader::write_into()`'s exact text, without the line ending."""
    return (
        f"{FRAME_HEADER_PREFIX} w={width} h={height} fmt={fmt} "
        f"len={payload_len} crc32=0x{crc32:08X}>>>"
    )


def parse_header_line(line: str):
    """Parses a header line; `None` when it is not one, `HeaderError` when broken.

    Same rules as `FrameHeader::parse()`: prefix and `>>>` suffix required,
    whitespace-separated `key=value` tokens, the five known keys mandatory,
    every value parsable, the format token one of `PixelFormat::name()`, and
    `len` equal to the length implied by the geometry and format. Returns a dict
    with `width`, `height`, `format`, `payload_len`, `crc32`.
    """
    text = line.strip()
    if not (text.startswith(FRAME_HEADER_PREFIX) and text.endswith(">>>")):
        return None
    body = text[len(FRAME_HEADER_PREFIX) : -len(">>>")].strip()

    values: dict[str, str] = {}
    for token in body.split():
        key, sep, value = token.partition("=")
        if not sep:
            raise HeaderError(f"header token without '=': {token!r}")
        if key not in ("w", "h", "fmt", "len", "crc32"):
            raise HeaderError(f"unknown header field: {key!r}")
        # A repeated field keeps the last occurrence, exactly like the Rust parser.
        values[key] = value

    missing = [key for key in ("w", "h", "fmt", "len", "crc32") if key not in values]
    if missing:
        raise HeaderError(f"header missing field(s): {', '.join(missing)}")

    try:
        width = _parse_u32(values["w"])
        height = _parse_u32(values["h"])
        payload_len = _parse_u32(values["len"])
        crc32 = _parse_hex_u32(values["crc32"])
    except ValueError as exc:
        raise HeaderError(f"header field not parsable: {exc}") from exc

    fmt = values["fmt"]
    if fmt not in FORMAT_BITS:
        raise HeaderError(f"unknown format token: {fmt!r}")

    implied = frame_len_bytes(fmt, width, height)
    if payload_len != implied:
        raise HeaderError(
            f"header is self-inconsistent: len={payload_len} but "
            f"{fmt} {width}x{height} implies {implied} bytes"
        )

    return {
        "width": width,
        "height": height,
        "format": fmt,
        "payload_len": payload_len,
        "crc32": crc32,
    }


# --------------------------------------------------------------------------- #
# PNG writer (zlib + struct only)
# --------------------------------------------------------------------------- #

_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"

#: (bit depth, PNG colour type): 0 = grayscale, 2 = truecolour RGB.
GRAY8 = (8, 0)
GRAY16 = (16, 0)
RGB8 = (8, 2)


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return (
        struct.pack(">I", len(data))
        + kind
        + data
        + struct.pack(">I", zlib.crc32(kind + data) & _U32_MAX)
    )


def build_png(width: int, height: int, bit_depth: int, color_type: int, rows: list[bytes]) -> bytes:
    """Assembles a non-interlaced PNG from already-filtered scanlines.

    Every row is prefixed with filter type 0 (None); each row must therefore be
    `width * bytes_per_pixel` long.
    """
    row_bytes = len(rows[0]) if rows else 0
    expected = row_bytes + 1 if rows else 0
    for index, row in enumerate(rows):
        if len(row) + 1 != expected:
            raise ImageError(
                f"row {index} is {len(row)} bytes, expected {row_bytes} for {width}x{height}"
            )
    if len(rows) != height:
        raise ImageError(f"got {len(rows)} rows, expected {height}")

    ihdr = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, 0)
    idat = zlib.compress(b"".join(b"\x00" + row for row in rows), 9)
    return _PNG_SIGNATURE + _png_chunk(b"IHDR", ihdr) + _png_chunk(b"IDAT", idat) + _png_chunk(b"IEND", b"")


def unpack_raw10(payload: bytes) -> list[int]:
    """Unpacks packed RAW10 (4 pixels per 5 bytes) into 10-bit values.

    Layout as the payload carries it: the first four bytes hold each pixel's high
    8 bits, the fifth holds the four low-bit pairs with pixel 0 in bits 1:0 and
    pixel 3 in bits 7:6. What is checked here is the round trip through
    :func:`_pack_raw10`, and agreement with the firmware's own payload.
    """
    if len(payload) % 5:
        raise ImageError(f"RAW10 payload of {len(payload)} bytes is not a whole 5-byte group count")
    values = []
    for offset in range(0, len(payload), 5):
        group = payload[offset : offset + 5]
        low = group[4]
        for index in range(4):
            high = group[index]
            values.append((high << 2) | ((low >> (2 * index)) & 0x03))
    return values


def _pack_raw10(values) -> bytes:
    """Reference MIPI CSI-2 RAW10 packer: the inverse of :func:`unpack_raw10`.

    The firmware packs and the host unpacks, so nothing on the capture path uses
    this; it exists so the unpacker can be round-tripped against an
    independently written packer instead of against itself.
    """
    if len(values) % 4:
        raise ValueError(f"RAW10 packs 4 pixels per group, got {len(values)}")
    packed = bytearray()
    for offset in range(0, len(values), 4):
        group = values[offset : offset + 4]
        for value in group:
            if not 0 <= value <= 0x3FF:
                raise ValueError(f"not a 10-bit sample: {value}")
            packed.append(value >> 2)
        packed.append(
            sum(((group[index] & 0x03) << (2 * index)) for index in range(4))
        )
    return bytes(packed)


def _rows_raw8(payload: bytes, width: int, height: int) -> list[bytes]:
    return [payload[y * width : (y + 1) * width] for y in range(height)]


def demosaic_bayer(samples, width: int, height: int, order: str):
    """Bilinear-demosaics a CFA plane into ``(r, g, b)`` planes at input scale.

    ``samples`` is a flat sequence of ``width * height`` values in raster order
    (RAW8 bytes, or RAW10 values from :func:`unpack_raw10`). The CFA phase is
    taken from :data:`BAYER_ORDERS`; nothing about the sensor's phase is assumed.

    Rules, per pixel:
      * the channel the pixel actually measures is copied verbatim;
      * a red/blue pixel takes green from its four orthogonal neighbours and the
        other chroma from its four diagonal neighbours;
      * a green pixel takes the two chroma from the horizontal and vertical
        neighbour pairs, whichever direction holds which colour for that tile.

    Neighbours at the frame edge clamp *inward* (not to themselves), so an edge
    sample is still a same-colour sample. Width and height must be even, or the
    2x2 CFA tiles do not line up; that is an :class:`ImageError`.

    This is a pure-Python preview path: it holds three int planes, so it costs
    roughly 0.3 s and tens of MB at 800x640 and appreciably more at 5 MP. That
    is fine for a bench preview; the raw dump is always written as well.
    """
    if order not in BAYER_ORDERS:
        raise ImageError(f"unknown Bayer order {order!r}; expected one of {', '.join(BAYER_ORDERS)}")
    if width < 2 or height < 2 or width % 2 or height % 2:
        raise ImageError(
            f"Bayer {order} needs an even width and height >= 2, got {width}x{height}"
        )
    if len(samples) != width * height:
        raise ImageError(
            f"Bayer {order} got {len(samples)} samples for {width}x{height} = {width * height}"
        )

    cfa = BAYER_ORDERS[order]
    last_x = width - 1
    last_y = height - 1
    red = [0] * len(samples)
    green = [0] * len(samples)
    blue = [0] * len(samples)

    for y in range(height):
        # Clamp inward so every neighbour is a real pixel of the expected colour.
        y_up = y - 1 if y else 1
        y_down = y + 1 if y < last_y else last_y - 1
        row = y * width
        row_up = y_up * width
        row_down = y_down * width
        pattern = cfa[y & 1]
        for x in range(width):
            index = row + x
            value = samples[index]
            x_left = x - 1 if x else 1
            x_right = x + 1 if x < last_x else last_x - 1
            channel = pattern[x & 1]
            if channel == "R" or channel == "B":
                diagonal = (
                    samples[row_up + x_left]
                    + samples[row_up + x_right]
                    + samples[row_down + x_left]
                    + samples[row_down + x_right]
                ) >> 2
                orthogonal = (
                    samples[row_up + x]
                    + samples[row_down + x]
                    + samples[row + x_left]
                    + samples[row + x_right]
                ) >> 2
                if channel == "R":
                    red[index] = value
                    green[index] = orthogonal
                    blue[index] = diagonal
                else:
                    blue[index] = value
                    green[index] = orthogonal
                    red[index] = diagonal
            else:
                green[index] = value
                vertical = (
                    samples[row_up + x] + samples[row_down + x]
                ) >> 1
                horizontal = (
                    samples[row + x_left] + samples[row + x_right]
                ) >> 1
                # Which chroma sits above/below this green, per the CFA tile?
                if cfa[y_up & 1][x & 1] == "R":
                    red[index] = vertical
                    blue[index] = horizontal
                else:
                    blue[index] = vertical
                    red[index] = horizontal
    return red, green, blue


def _rows_bayer(payload: bytes, fmt: str, width: int, height: int, order: str):
    """Demosaiced RGB rows plus the PNG ``(bit depth, colour type)`` to use.

    RAW8 stays 8-bit RGB; RAW10 becomes 16-bit RGB with the 10-bit samples
    left-justified (``value << 6``), mirroring the grayscale RAW10 rendering so
    it is exactly invertible.
    """
    if fmt == "RAW8":
        samples = payload
        bits = 8
    elif fmt == "RAW10":
        samples = unpack_raw10(payload)
        bits = 10
    else:
        raise ImageError(
            f"--bayer applies to RAW8/RAW10 only; {fmt} already carries colour"
        )

    red, green, blue = demosaic_bayer(samples, width, height, order)
    rows = []
    if bits == 8:
        for y in range(height):
            base = y * width
            row = bytearray(width * 3)
            for x in range(width):
                index = base + x
                row[3 * x] = red[index]
                row[3 * x + 1] = green[index]
                row[3 * x + 2] = blue[index]
            rows.append(bytes(row))
        return rows, 8, 2

    shift = 16 - bits
    layout = ">" + "H" * (width * 3)
    for y in range(height):
        base = y * width
        values = []
        for x in range(width):
            index = base + x
            values.append(red[index] << shift)
            values.append(green[index] << shift)
            values.append(blue[index] << shift)
        rows.append(struct.pack(layout, *values))
    return rows, 16, 2


def _rows_raw10(payload: bytes, width: int, height: int) -> list[bytes]:
    if width % 4:
        raise ImageError(
            f"RAW10 width {width} is not a multiple of 4, so the 5-byte packing groups "
            "do not align with the rows"
        )
    values = unpack_raw10(payload)
    if len(values) != width * height:
        raise ImageError(f"RAW10 payload unpacked to {len(values)} pixels, expected {width * height}")
    # 10-bit samples left-justified in 16 bits: viewable, and exactly invertible.
    rows = []
    for y in range(height):
        row = values[y * width : (y + 1) * width]
        rows.append(b"".join(struct.pack(">H", value << 6) for value in row))
    return rows


def _expand_rgb565(value: int) -> bytes:
    r5 = (value >> 11) & 0x1F
    g6 = (value >> 5) & 0x3F
    b5 = value & 0x1F
    return bytes(((r5 << 3) | (r5 >> 2), (g6 << 2) | (g6 >> 4), (b5 << 3) | (b5 >> 2)))


def _rows_rgb565(payload: bytes, width: int, height: int) -> list[bytes]:
    rows = []
    for y in range(height):
        row = bytearray()
        for x in range(width):
            offset = (y * width + x) * 2
            (value,) = struct.unpack_from("<H", payload, offset)  # core says little endian
            row += _expand_rgb565(value)
        rows.append(bytes(row))
    return rows


def _rows_rgb888(payload: bytes, width: int, height: int) -> list[bytes]:
    return [payload[(y * width) * 3 : ((y + 1) * width) * 3] for y in range(height)]


#: format token -> (row builder, bit depth, PNG colour type).
_ROW_BUILDERS = {
    "RAW8": (_rows_raw8, *GRAY8),
    "RAW10": (_rows_raw10, *GRAY16),
    "RGB565": (_rows_rgb565, *RGB8),
    "RGB888": (_rows_rgb888, *RGB8),
}


def png_from_frame(payload: bytes, fmt: str, width: int, height: int, bayer: str = BAYER_NONE) -> bytes:
    """Renders a verified payload as a viewable PNG.

    ``bayer`` selects a CFA phase for RAW8/RAW10 (see :func:`demosaic_bayer`);
    ``"none"`` keeps the plane as grayscale and is the default, because this
    project has not verified the sensor's phase on this board.
    """
    if width <= 0 or height <= 0:
        raise ImageError(f"cannot render a {width}x{height} frame")
    if bayer != BAYER_NONE:
        rows, bit_depth, color_type = _rows_bayer(payload, fmt, width, height, bayer)
        return build_png(width, height, bit_depth, color_type, rows)
    builder, bit_depth, color_type = _ROW_BUILDERS[fmt]
    rows = builder(payload, width, height)
    return build_png(width, height, bit_depth, color_type, rows)


# --------------------------------------------------------------------------- #
# Console reading
# --------------------------------------------------------------------------- #

_HEX_PREFIX = FRAME_HEADER_PREFIX.encode("ascii")
_TRAILER_BYTES = FRAME_TRAILER.encode("ascii")


def _echo_text(chunk: bytes, sink) -> None:
    """Prints the firmware's text lines that precede a frame (or a stray CRLF run)."""
    for line in chunk.split(b"\r\n"):
        text = line.decode("ascii", "replace").rstrip()
        if text:
            print(text, file=sink)


def _fill(port, buf: bytearray, deadline: float) -> bool:
    """Reads once into `buf`; False when nothing arrived before `deadline`.

    A port that raises (the board re-enumerated and the tty vanished) becomes a
    :class:`CommError`, so callers have one transport error class to handle.
    """
    if time.monotonic() > deadline:
        return False
    try:
        chunk = port.read(4096)
    except Exception as exc:  # serial.SerialException, OSError, pyserial's TypeError
        raise CommError(f"serial read failed: {exc}") from exc
    if chunk:
        buf += chunk
        return True
    time.sleep(0.01)
    return True


def read_exact(port, count: int, deadline: float, first: bytes, label: str) -> tuple[bytes, bytes]:
    """Reads exactly `count` bytes, starting from `first`; short reads are errors.

    Returns `(bytes, leftover)`: anything read past `count` is handed back rather
    than dropped, because the trailer follows immediately.
    """
    buf = bytearray(first)
    while len(buf) < count:
        if time.monotonic() > deadline:
            raise CommError(f"{label}: short read, {len(buf)} of {count} bytes arrived")
        _fill(port, buf, deadline)
    return bytes(buf[:count]), bytes(buf[count:])


def read_frame(
    port,
    timeout: float,
    payload_timeout: float,
    echo_sink=None,
    require_trailer: bool = False,
):
    """Reads one header + payload (and the trailer when it arrives) from `port`.

    Returns `(header, payload, trailer_seen)`. Raises :class:`CommError` for
    transport problems (including a header that never showed up) and
    :class:`HeaderError` for a header that breaks the framing rules.

    With ``require_trailer`` a payload whose ``<<<END>>>`` never shows up raises
    :class:`IncompleteFrameError` (carrying the complete, still-unverified
    header/payload) instead of only warning. Without it the payload is returned
    with ``trailer_seen=False`` and a warning, which is what the reader-only
    selftests want.

    ``echo_sink`` defaults to the *current* ``sys.stderr`` (not the one bound at
    import time), so redirecting stderr really captures the firmware's text.
    """
    if echo_sink is None:
        echo_sink = sys.stderr
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while True:
        start = buf.find(_HEX_PREFIX)
        if start >= 0:
            end = buf.find(b">>>", start + len(_HEX_PREFIX))
            if end >= 0:
                after = end + len(b">>>")
                # A serial read can deliver the CR before the LF, so anything
                # that is still a prefix of a line ending means "wait for more"
                # rather than "malformed".
                candidate = bytes(buf[after:])
                body_end = -1  # Line ending has not fully arrived yet.
                for ending in (b"\r\n", b"\n"):
                    if candidate.startswith(ending):
                        body_end = after + len(ending)
                        break
                else:
                    if candidate and not any(
                        ending.startswith(candidate) for ending in (b"\r\n", b"\n")
                    ):
                        # The firmware always ends the header with a line
                        # ending; a header glued to other bytes means the stream
                        # is not a frame stream, and guessing would eat a
                        # payload byte.
                        raise HeaderError(
                            "header line is not terminated by a line ending (found "
                            f"{candidate[:1]!r} right after '>>>')"
                        )
                if body_end >= 0:
                    break
        if not _fill(port, buf, deadline):
            raise CommError(
                f"no {FRAME_HEADER_PREFIX} header within {timeout:g}s "
                f"({len(buf)} bytes of console output read)"
            )

    if start:
        # Firmware log lines ahead of the frame; keep them for context.
        _echo_text(bytes(buf[:start]), echo_sink)
    line = bytes(buf[start : end + 3]).decode("ascii", "replace")
    print(f"header: {line}", file=echo_sink)

    header = parse_header_line(line)
    if header is None:  # Unreachable: the scan found prefix and suffix.
        raise HeaderError(f"not a frame header: {line!r}")

    rest = bytes(buf[body_end:])
    payload, leftover = read_exact(
        port,
        header["payload_len"],
        time.monotonic() + payload_timeout,
        rest,
        "payload",
    )

    trailer_deadline = time.monotonic() + min(timeout, 2.0)
    tail = bytearray(leftover)
    trailer_seen = _TRAILER_BYTES in tail
    while not trailer_seen and time.monotonic() <= trailer_deadline:
        if not _fill(port, tail, trailer_deadline):
            break
        trailer_seen = _TRAILER_BYTES in tail
    if not trailer_seen:
        message = (
            f"{FRAME_TRAILER} not seen after the payload; the payload itself is complete"
        )
        if require_trailer:
            raise IncompleteFrameError(message, header, payload)
        print(f"warning: {message}", file=echo_sink)

    return header, payload, trailer_seen


def verify_frame(header: dict, payload: bytes) -> None:
    """Re-checks length and CRC-32 of a complete payload; raises ChecksumError."""
    expected_len = frame_len_bytes(header["format"], header["width"], header["height"])
    if len(payload) != expected_len:
        raise ChecksumError(
            f"length mismatch: read {len(payload)} bytes, header implies {expected_len}"
        )
    actual = crc32_ieee(payload)
    if actual != header["crc32"]:
        raise ChecksumError(
            f"CRC-32/ISO-HDLC mismatch: payload is 0x{actual:08X}, "
            f"header says 0x{header['crc32']:08X}"
        )


def _open_serial(candidate: str, baud: int):
    """Opens `candidate` without driving DTR/RTS.

    `tools/read_serial.py` documents the board behaviour: asserting the control
    lines makes the USB Serial/JTAG reset the chip. The deferred
    ``Serial()`` form lets both lines be parked low *before* ``open()``, and it
    matters just as much for a reconnect (a re-open must not cost a reset).
    """
    handle = serial.Serial()
    handle.port = candidate
    handle.baudrate = baud
    handle.timeout = 0.1
    handle.dtr = False
    handle.rts = False
    handle.open()
    return handle


def open_port(port: str | None, baud: int, scan_fallback: bool = False):
    """Opens `port` (or the first `/dev/cu.usbmodem*`); raises :class:`CommError`.

    ``scan_fallback`` adds the ``/dev/cu.usbmodem*`` scan *after* a named port,
    for reconnects: the board's USB Serial/JTAG re-enumerates under a new tty
    name, so the old name alone would never come back.
    """
    if serial is None:
        raise CommError(
            "pyserial is not installed (pip install pyserial); --selftest does not need it"
        )
    candidates = [port] if port else []
    if not port or scan_fallback:
        scanned = sorted(glob.glob("/dev/cu.usbmodem*"))
        candidates += [candidate for candidate in scanned if candidate not in candidates]
    if not candidates:
        raise CommError("no /dev/cu.usbmodem* device found; pass --port")
    errors = []
    for candidate in candidates:
        try:
            return _open_serial(candidate, baud)
        except Exception as exc:  # serial.SerialException and OSError alike
            errors.append(f"{candidate}: {exc}")
    raise CommError("could not open any port -- " + "; ".join(errors))


class PortManager:
    """Owns the serial port and re-opens it when the board re-enumerates.

    A capture is restarted from a clean buffer after every successful re-open,
    so a header/payload split across the disconnect is discarded rather than
    misinterpreted. Re-opening is bounded twice over: by ``reconnect_timeout``
    seconds per event (with capped exponential backoff) and by
    ``max_reconnects`` events per capture.

    Re-opening is deliberately *not* automatic on every error: only transport
    failures reach it. A CRC or length mismatch is content, not transport, and
    re-opening there would both hide the real fault and (because opening the
    port asserts the control lines) reset the board.

    ``opener``, ``clock``, ``sleep`` and ``log`` are injectable so the selftest
    can exercise re-enumeration with no hardware and no real waiting.
    """

    def __init__(
        self,
        port: str | None,
        baud: int,
        reconnect_timeout: float = 10.0,
        max_reconnects: int = 6,
        opener=open_port,
        clock=time.monotonic,
        sleep=time.sleep,
        log=None,
    ):
        self.port_name = port
        self.baud = baud
        self.reconnect_timeout = reconnect_timeout
        self.max_reconnects = max_reconnects
        self.reconnects = 0
        self._opener = opener
        self._clock = clock
        self._sleep = sleep
        self._log = log
        self.port = None

    def _open_with_retry(self, label: str):
        deadline = self._clock() + self.reconnect_timeout
        delay = 0.2
        attempt = 0
        last = None
        while True:
            attempt += 1
            try:
                return self._opener(self.port_name, self.baud, True)
            except Exception as exc:  # CommError, or a bare serial/OS error
                last = exc
                if self._clock() + delay > deadline:
                    raise CommError(
                        f"{label} failed after {attempt} attempt(s) in "
                        f"{self.reconnect_timeout:g}s: {last}"
                    ) from last
                self._sleep(delay)
                delay = min(delay * 2, 1.0)

    def open(self):
        """Returns the open port, opening it (with bounded retries) if needed."""
        if self.port is None:
            self.port = self._open_with_retry("open")
        return self.port

    def close(self) -> None:
        if self.port is not None:
            with contextlib.suppress(Exception):
                self.port.close()
            self.port = None

    def reopen(self):
        """Closes the dead port and opens a fresh one; raises when exhausted."""
        self.close()
        if self.reconnects >= self.max_reconnects:
            raise CommError(
                f"giving up after {self.reconnects} reconnect(s); the board did not "
                "come back"
            )
        self.reconnects += 1
        self.port = self._open_with_retry(
            f"reconnect {self.reconnects}/{self.max_reconnects}"
        )
        print(
            f"reconnected to {getattr(self.port, 'port', self.port_name or '/dev/cu.usbmodem*')}",
            file=self._log or sys.stderr,
        )
        return self.port


def default_prefix() -> str:
    return "capture-" + time.strftime("%Y%m%d-%H%M%S", time.localtime())


def write_outputs(
    prefix: str, header: dict, payload: bytes, bayer: str = BAYER_NONE
) -> tuple[str, str | None]:
    """Writes `<prefix>.raw` and, when the payload renders, `<prefix>.png`."""
    directory = os.path.dirname(prefix)
    if directory:
        os.makedirs(directory, exist_ok=True)
    raw_path = prefix + ".raw"
    with open(raw_path, "wb") as handle:
        handle.write(payload)

    png_path = prefix + ".png"
    try:
        png = png_from_frame(
            payload, header["format"], header["width"], header["height"], bayer
        )
    except ImageError as exc:
        print(f"image error: {exc} (the .raw dump is still valid)", file=sys.stderr)
        return raw_path, None
    with open(png_path, "wb") as handle:
        handle.write(png)
    return raw_path, png_path


# --------------------------------------------------------------------------- #
# Selftest
# --------------------------------------------------------------------------- #


class NullSink:
    """Discards text; used as the reader's `echo_sink` so no handle is leaked."""

    def write(self, text: str) -> int:
        return len(text)

    def flush(self) -> None:
        pass


#: One shared discard sink for every selftest read (`sys.stderr` is also fine,
#: but the selftest should not need the caller to redirect it).
NULL_SINK = NullSink()


class FakePort:
    """Minimal stand-in for `serial.Serial` used by `--selftest`."""

    def __init__(self, data: bytes, chunk: int = 7):
        self.data = bytearray(data)
        self.chunk = chunk

    def read(self, count: int) -> bytes:
        take = min(count, self.chunk, len(self.data))
        out = bytes(self.data[:take])
        del self.data[:take]
        return out

    def close(self) -> None:
        pass


class UnpluggedPort(FakePort):
    """Delivers `data`, then raises the way a re-enumerated tty does.

    ``data=b""`` models a port that is already gone when the read starts.
    """

    def __init__(self, data: bytes = b"", chunk: int = 64, error=None):
        super().__init__(data, chunk)
        self.error = error or OSError("device not configured")
        self.closed = False
        self.reads = 0

    def read(self, count: int) -> bytes:
        self.reads += 1
        if not self.data:
            raise self.error
        return super().read(count)

    def close(self) -> None:
        self.closed = True


class ScriptedOpener:
    """`open_port`-shaped opener that returns queued ports or raises them."""

    def __init__(self, items):
        self.items = list(items)
        self.calls = []

    def __call__(self, port, baud, scan_fallback):
        self.calls.append((port, baud, scan_fallback))
        if not self.items:
            raise CommError("no /dev/cu.usbmodem* device found")
        item = self.items.pop(0)
        if isinstance(item, Exception):
            raise item
        return item


class FakeClock:
    """Monotonic clock whose `sleep` advances time instead of waiting."""

    def __init__(self):
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


class Args:
    """The `argparse.Namespace` fields `run_capture` reads, for selftest only."""

    def __init__(self, **overrides):
        self.port = None
        self.baud = 115200
        self.timeout = 0.2
        self.payload_timeout = 0.2
        self.reconnect_timeout = 1.0
        self.max_reconnects = 4
        self.bayer = BAYER_NONE
        self.out = None
        self.selftest = False
        self.__dict__.update(overrides)


def _round_trip(data: bytes, timeout: float = 0.05, chunk: int = 7) -> tuple[dict, bytes]:
    """Runs `data` through the real reader + verifier used on hardware."""
    header, payload, _trailer = read_frame(
        FakePort(data, chunk=chunk),
        timeout=timeout,
        payload_timeout=timeout,
        echo_sink=NULL_SINK,
    )
    verify_frame(header, payload)
    return header, payload


def selftest() -> int:
    """Checks framing, CRC, error classes and PNG authoring with no hardware."""
    checks = 0

    # 1. The canonical CRC-32/ISO-HDLC test vector.
    assert crc32_ieee(b"123456789") == 0xCBF43926, hex(crc32_ieee(b"123456789"))
    assert crc32_ieee(b"123456789") == zlib.crc32(b"123456789") & _U32_MAX
    assert crc32_ieee(b"") == 0x00000000
    checks += 3

    # 2. Synthetic frame -> header line -> parser -> reader -> verifier.
    # Geometry parity with the vectors core/src/lib.rs itself asserts.
    assert frame_len_bytes("RAW8", 800, 640) == 512_000
    assert frame_len_bytes("RAW10", 1920, 1080) == 2_592_000
    assert frame_len_bytes("RAW10", 4, 1) == 5
    assert frame_len_bytes("RGB565", 640, 480) == 614_400
    assert frame_len_bytes("RGB888", 640, 480) == 921_600
    checks += 5
    width, height, fmt = 8, 4, "RAW8"
    payload = bytes((x * 7 + y * 13) & 0xFF for y in range(height) for x in range(width))
    assert len(payload) == frame_len_bytes(fmt, width, height)
    line = render_header_line(width, height, fmt, len(payload), crc32_ieee(payload))
    assert line == (
        "<<<FRAME w=8 h=4 fmt=RAW8 len=32 crc32=0x" + f"{crc32_ieee(payload):08X}" + ">>>"
    ), line

    parsed = parse_header_line(line)
    assert parsed == {
        "width": width,
        "height": height,
        "format": fmt,
        "payload_len": len(payload),
        "crc32": crc32_ieee(payload),
    }, parsed
    checks += 2

    # 2b. Wire-format parity with the real Rust core, taken from a scratch
    # `cargo test` run against core/src/lib.rs: its `FrameHeader::write_into`
    # rendered the line below for 800x640 RAW8 over this payload, and the Python
    # parser re-reads it field for field.
    rust_payload = bytes([7]) * 512_000
    assert crc32_ieee(rust_payload) == 0x13A22E7B
    assert (
        render_header_line(800, 640, "RAW8", 512_000, crc32_ieee(rust_payload))
        == "<<<FRAME w=800 h=640 fmt=RAW8 len=512000 crc32=0x13A22E7B>>>"
    )
    assert parse_header_line(
        "<<<FRAME w=1920 h=1080 fmt=RAW10 len=2592000 crc32=0x1E5522BB>>>"
    ) == {
        "width": 1920,
        "height": 1080,
        "format": "RAW10",
        "payload_len": 2_592_000,
        "crc32": 0x1E5522BB,
    }
    checks += 3

    wire = b"boot: ov5647 chip id 0x5647\r\n" + line.encode() + b"\r\n" + payload + FRAME_TRAILER.encode() + b"\r\n"
    # Serial reads split anywhere, including between CR and LF and mid-header.
    for chunk in (1, 2, 3, 5, 7, 4096):
        header, read_back = _round_trip(wire, chunk=chunk)
        assert header["crc32"] == crc32_ieee(read_back), chunk
        assert read_back == payload, chunk
        checks += 1

    # 3. Error classes are distinct.
    corrupted = bytearray(payload)
    corrupted[0] ^= 0xFF
    try:
        _round_trip(line.encode() + b"\r\n" + bytes(corrupted))
    except ChecksumError as exc:
        assert "CRC-32/ISO-HDLC mismatch" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("corrupted payload with the header's CRC did not raise ChecksumError")

    # `verify_frame`'s length guard, reached directly: the reader itself can only
    # ever hand it exactly `payload_len` bytes, so a wrong payload is the way to
    # exercise it.
    try:
        verify_frame(parsed, payload[:-1])
    except ChecksumError as exc:
        assert "length mismatch" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("payload that does not match the header length was accepted")

    try:
        _round_trip(line.encode() + b"\r\n" + payload[: len(payload) // 2])
    except CommError as exc:
        assert "short read" in str(exc), exc
        assert not isinstance(exc, (ChecksumError, HeaderError)), exc
        checks += 1
    else:
        raise AssertionError("a payload truncated on the wire was not a communication error")

    try:
        read_frame(FakePort(b"nothing to see here\r\n"), 0.02, 0.02, echo_sink=NULL_SINK)
    except CommError as exc:
        assert "no <<<FRAME header" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a stream without a header was not a communication error")

    try:
        read_frame(
            FakePort(line.encode() + b"XX" + payload), 0.02, 0.02, echo_sink=NULL_SINK
        )
    except HeaderError as exc:
        assert "not terminated by a line ending" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a header glued to non-line-ending bytes was accepted")

    # 4. Malformed headers are all HeaderError (a CommError subclass, reported apart).
    malformed = [
        "<<<FRAME w=8 h=4 fmt=RAW8 len=31 crc32=0x00000000>>>",  # len vs geometry
        "<<<FRAME w=8 h=4 fmt=YUV len=32 crc32=0x00000000>>>",  # unknown format token
        "<<<FRAME w=8 h=4 fmt=RAW8 len=32>>>",  # missing field
        "<<<FRAME w=8 h=4 fmt=RAW8 len=32 crc32=0x00000000 extra=1>>>",  # unknown field
        "<<<FRAME w=8 h=4 fmt=RAW8 len=-1 crc32=0x00000000>>>",  # unparsable value
        "<<<FRAME w=8 h=4 fmt=RAW8 len=32 crc32=00000000>>>",  # no 0x prefix
        "<<<FRAME w=4294967296 h=4 fmt=RAW8 len=32 crc32=0x00000000>>>",  # u32 overflow
        "<<<FRAME w=8 h=4 fmt=raw8 len=32 crc32=0x00000000>>>",  # case sensitive token
    ]
    for candidate in malformed:
        try:
            parse_header_line(candidate)
        except HeaderError:
            checks += 1
        else:
            raise AssertionError(f"malformed header accepted: {candidate!r}")
    assert parse_header_line("") is None
    assert parse_header_line("<<<FRAME w=8 h=4 fmt=RAW8 len=32 crc32=0x00000000>>>") is not None
    checks += 2

    # 5. RAW10 packing round-trips, and every format renders a structurally valid PNG.
    values = [0, 1, 512, 1023] * 4
    packed = bytearray()
    for group in range(0, len(values), 4):
        chunk = values[group : group + 4]
        packed += bytes(value >> 2 for value in chunk)
        packed.append(sum(((chunk[i] & 0x03) << (2 * i)) for i in range(4)))
    assert unpack_raw10(bytes(packed)) == values
    checks += 1

    for fmt in ("RAW8", "RAW10", "RGB565", "RGB888"):
        width = 8 if fmt != "RGB888" else 4
        height = 2
        size = frame_len_bytes(fmt, width, height)
        sample = bytes((index * 31 + 5) & 0xFF for index in range(size))
        png = png_from_frame(sample, fmt, width, height)
        assert png.startswith(_PNG_SIGNATURE)
        offset, chunks = 8, {}
        while offset < len(png):
            (length,) = struct.unpack_from(">I", png, offset)
            kind = png[offset + 4 : offset + 8]
            data = png[offset + 8 : offset + 8 + length]
            (crc,) = struct.unpack_from(">I", png, offset + 8 + length)
            assert crc == zlib.crc32(kind + data) & _U32_MAX, f"{fmt} chunk {kind!r} CRC"
            chunks[kind] = data
            offset += 12 + length
        assert offset == len(png) and b"IEND" in chunks, fmt
        ihdr_width, ihdr_height, bit_depth, color_type, compression, filter_method, interlace = (
            struct.unpack(">IIBBBBB", chunks[b"IHDR"])
        )
        assert (ihdr_width, ihdr_height) == (width, height), (fmt, chunks[b"IHDR"])
        assert (compression, filter_method, interlace) == (0, 0, 0), fmt
        assert (bit_depth, color_type) == _ROW_BUILDERS[fmt][1:], (fmt, bit_depth, color_type)
        scanlines = zlib.decompress(chunks[b"IDAT"])
        # Derived from the PNG colour type: RGB expands to three bytes per pixel
        # (RGB565 included), grayscale to one or two by bit depth.
        bytes_per_pixel = (bit_depth // 8) * (3 if color_type == 2 else 1)
        assert len(scanlines) == height * (1 + width * bytes_per_pixel), (fmt, len(scanlines))
        checks += 1

    # 6. Files land where asked, and a RAW10 odd width is an ImageError, not a crash.
    with tempfile.TemporaryDirectory() as tmp:
        prefix = os.path.join(tmp, "nested", "frame")
        raw_path, png_path = write_outputs(
            prefix,
            {"width": 8, "height": 2, "format": "RAW8", "payload_len": 16, "crc32": 0},
            b"\x00" * 16,
        )
        assert os.path.getsize(raw_path) == 16 and png_path and os.path.getsize(png_path) > 0
        # An unrenderable geometry must be reported as an image error, and the
        # verified .raw dump must still land.
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            odd_raw, odd_png = write_outputs(
                os.path.join(tmp, "odd"),
                {"width": 6, "height": 2, "format": "RAW10", "payload_len": 15, "crc32": 0},
                b"\x00" * 15,
            )
        assert odd_png is None and os.path.getsize(odd_raw) == 15, (odd_raw, odd_png)
        assert str(stderr.getvalue()).startswith("image error:"), stderr.getvalue()
        checks += 3

    # 7. Packed RAW10: a hand-computed group pins the byte layout independently,
    #    then all 1024 sample values round-trip through pack -> unpack.
    assert unpack_raw10(bytes([0xFF, 0x00, 0x80, 0x55, 0x53])) == [0x3FF, 0x000, 0x201, 0x155]
    every_value = list(range(1024))
    assert len(_pack_raw10(every_value)) == 1280
    assert unpack_raw10(_pack_raw10(every_value)) == every_value
    assert unpack_raw10(_pack_raw10([1023, 0, 1, 512])) == [1023, 0, 1, 512]
    # A payload that is not a whole number of 5-byte groups is an ImageError, not
    # a silently short pixel list.
    try:
        unpack_raw10(b"\x00" * 6)
    except ImageError as exc:
        assert "5-byte group" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a RAW10 payload with a partial packing group was accepted")
    checks += 3

    # 8. Bayer: the phase decides which pixel measures which colour, so a
    #    constant scene must demosaic back to exactly that colour for every order.
    ideal = {"R": 200, "G": 100, "B": 50}
    for order, cfa in BAYER_ORDERS.items():
        width = height = 8
        samples = [ideal[cfa[y & 1][x & 1]] for y in range(height) for x in range(width)]
        red, green, blue = demosaic_bayer(samples, width, height, order)
        assert set(red) == {200} and set(green) == {100} and set(blue) == {50}, (
            order,
            sorted(set(red)),
            sorted(set(green)),
            sorted(set(blue)),
        )
        # Any other phase must not reproduce the same colour assignment, i.e. the
        # order is honoured rather than ignored.
        for other in BAYER_ORDERS:
            if other == order:
                continue
            wrong_red, wrong_green, _ = demosaic_bayer(samples, width, height, other)
            assert (set(wrong_red), set(wrong_green)) != ({200}, {100}), (order, other)
        checks += 1

    #    A second, deliberately naive implementation of the same standard rule,
    #    written directly from the neighbour table, must agree on every interior
    #    pixel for all four phases (borders clamp inward, so compare interiors).
    def _naive_demosaic(plane, width, height, order):
        phase = BAYER_ORDERS[order]

        def value(x, y):
            x = min(max(x, 0), width - 1)
            y = min(max(y, 0), height - 1)
            return plane[y * width + x]

        def channel(x, y):
            x = min(max(x, 0), width - 1)
            y = min(max(y, 0), height - 1)
            return phase[y & 1][x & 1]

        out = ([], [], [], [])
        for y in range(height):
            for x in range(width):
                here = channel(x, y)
                if here == "G":
                    vertical = (value(x, y - 1) + value(x, y + 1)) // 2
                    horizontal = (value(x - 1, y) + value(x + 1, y)) // 2
                    red, blue = (
                        (vertical, horizontal)
                        if channel(x, y - 1) == "R"
                        else (horizontal, vertical)
                    )
                    out[0].append(red)
                    out[1].append(value(x, y))
                    out[2].append(blue)
                else:
                    green = (
                        value(x - 1, y) + value(x + 1, y) + value(x, y - 1) + value(x, y + 1)
                    ) // 4
                    diagonal = (
                        value(x - 1, y - 1)
                        + value(x + 1, y - 1)
                        + value(x - 1, y + 1)
                        + value(x + 1, y + 1)
                    ) // 4
                    red, blue = (value(x, y), diagonal) if here == "R" else (diagonal, value(x, y))
                    out[0].append(red)
                    out[1].append(green)
                    out[2].append(blue)
        return out

    seed = 12345
    plane = []
    for _ in range(8 * 6):
        seed = (seed * 1103515245 + 12345) & 0x7FFF_FFFF
        plane.append((seed >> 16) & 0xFF)
    for order in BAYER_ORDERS:
        mine = demosaic_bayer(plane, 8, 6, order)
        reference = _naive_demosaic(plane, 8, 6, order)
        for y in range(1, 5):
            for x in range(1, 7):
                index = y * 8 + x
                assert all(mine[c][index] == reference[c][index] for c in range(3)), (
                    order,
                    x,
                    y,
                )
        checks += 1

    #    Non-CFA geometry and unknown orders are ImageErrors, never silent guesses.
    for bad_width, bad_height in ((7, 2), (2, 3), (1, 1)):
        try:
            demosaic_bayer([0] * (bad_width * bad_height), bad_width, bad_height, "RGGB")
        except ImageError as exc:
            assert "even width and height" in str(exc), exc
            checks += 1
        else:
            raise AssertionError(f"Bayer {bad_width}x{bad_height} was accepted")
    try:
        demosaic_bayer([0] * 4, 2, 2, "XYZW")
    except ImageError as exc:
        assert "unknown Bayer order" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("an unknown Bayer order was accepted")

    # 8b. RAW8 Bayer -> 8-bit RGB PNG, and RAW10 Bayer -> 16-bit RGB PNG with the
    #     10-bit samples left-justified exactly like the grayscale RAW10 path.
    def _png_chunks(png: bytes) -> dict:
        assert png.startswith(_PNG_SIGNATURE)
        offset, chunks = 8, {}
        while offset < len(png):
            (length,) = struct.unpack_from(">I", png, offset)
            kind = png[offset + 4 : offset + 8]
            data = png[offset + 8 : offset + 8 + length]
            (crc,) = struct.unpack_from(">I", png, offset + 8 + length)
            assert crc == zlib.crc32(kind + data) & _U32_MAX, f"chunk {kind!r} CRC"
            chunks[kind] = data
            offset += 12 + length
        assert offset == len(png) and b"IEND" in chunks, "PNG chunk walk"
        return chunks

    def _ihdr(chunks: dict) -> tuple:
        return struct.unpack(">IIBBBBB", chunks[b"IHDR"])

    def _png_ihdr(path: str) -> tuple:
        with open(path, "rb") as handle:
            return _ihdr(_png_chunks(handle.read()))

    width = height = 4
    rggb = BAYER_ORDERS["RGGB"]
    raw8 = bytes(ideal[rggb[y & 1][x & 1]] for y in range(height) for x in range(width))
    chunks = _png_chunks(png_from_frame(raw8, "RAW8", width, height, "RGGB"))
    assert _ihdr(chunks) == (width, height, 8, 2, 0, 0, 0), _ihdr(chunks)
    scanlines = zlib.decompress(chunks[b"IDAT"])
    assert len(scanlines) == height * (1 + width * 3), len(scanlines)
    assert scanlines[:4] == b"\x00" + bytes((200, 100, 50)), scanlines[:4]
    checks += 1

    scaled = {"R": 800, "G": 400, "B": 200}
    rggb10 = BAYER_ORDERS["GBRG"]
    values10 = [scaled[rggb10[y & 1][x & 1]] for y in range(height) for x in range(width)]
    chunks = _png_chunks(png_from_frame(_pack_raw10(values10), "RAW10", width, height, "GBRG"))
    assert _ihdr(chunks) == (width, height, 16, 2, 0, 0, 0), _ihdr(chunks)
    scanlines = zlib.decompress(chunks[b"IDAT"])
    assert scanlines[:7] == b"\x00" + struct.pack(">3H", 800 << 6, 400 << 6, 200 << 6), scanlines[:7]
    assert len(scanlines) == height * (1 + width * 6), len(scanlines)
    checks += 1

    #     RAW10 Bayer only needs even dimensions (unlike the grayscale renderer,
    #     which slices rows out of the 5-byte packing groups and needs width % 4).
    assert png_from_frame(bytes(15), "RAW10", 6, 2, "RGGB").startswith(_PNG_SIGNATURE)
    checks += 1

    #     Bayer is meaningless for the RGB formats and is refused, not ignored.
    for fmt in ("RGB565", "RGB888"):
        size = frame_len_bytes(fmt, 4, 2)
        try:
            png_from_frame(bytes(size), fmt, 4, 2, "RGGB")
        except ImageError as exc:
            assert "RAW8/RAW10 only" in str(exc), exc
            checks += 1
        else:
            raise AssertionError(f"--bayer was silently accepted for {fmt}")

    # 8c. The flag reaches the files: `write_outputs` renders the RGB PNG.
    with tempfile.TemporaryDirectory() as tmp:
        prefix = os.path.join(tmp, "bayer")
        plain_raw, plain_png = write_outputs(
            prefix,
            {"width": 4, "height": 4, "format": "RAW8", "payload_len": 16, "crc32": 0},
            raw8,
            BAYER_NONE,
        )
        assert _png_ihdr(plain_png)[3] == 0, "grayscale by default"
        demosaiced_raw, demosaiced_png = write_outputs(
            prefix + "-rg",
            {"width": 4, "height": 4, "format": "RAW8", "payload_len": 16, "crc32": 0},
            raw8,
            "RGGB",
        )
        assert os.path.getsize(plain_raw) == os.path.getsize(demosaiced_raw) == 16
        assert _png_ihdr(demosaiced_png) == (4, 4, 8, 2, 0, 0, 0)
        checks += 1

    # 9. An incomplete frame: the payload is complete and still verifies, but the
    #    trailer never arrives. That is its own class, carrying the frame, and it
    #    is not a CommError (the bytes are good).
    try:
        read_frame(
            FakePort(line.encode() + b"\r\n" + payload),
            0.05,
            0.05,
            echo_sink=NULL_SINK,
            require_trailer=True,
        )
    except IncompleteFrameError as exc:
        assert not isinstance(exc, CommError), exc
        assert exc.header == parsed and exc.payload == payload, exc
        verify_frame(exc.header, exc.payload)  # still a valid frame
        assert FRAME_TRAILER in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a missing trailer was not reported as an incomplete frame")

    #    Without `require_trailer` the same bytes are returned with the flag down.
    strictless_header, strictless_payload, seen = read_frame(
        FakePort(line.encode() + b"\r\n" + payload),
        0.05,
        0.05,
        echo_sink=NULL_SINK,
    )
    assert seen is False and strictless_payload == payload and strictless_header == parsed
    checks += 1

    # 10. Re-enumeration: a port that vanishes must become a CommError, and the
    #     capture must resume on a freshly opened port, from a clean buffer.
    devnull = NULL_SINK
    clock = FakeClock()
    opener = ScriptedOpener([UnpluggedPort(b"<<<FRAME w=8 h=4 fm"), FakePort(wire)])
    manager = PortManager(
        None, 115200, 1.0, 4, opener=opener, clock=clock, sleep=clock.sleep, log=devnull
    )
    with tempfile.TemporaryDirectory() as tmp:
        stdout = io.StringIO()
        out = os.path.join(tmp, "reconnect")
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(io.StringIO()):
            code = run_capture(Args(out=out, bayer="GBRG"), manager=manager)
        assert code == 0, (code, stdout.getvalue())
        assert os.path.getsize(out + ".raw") == len(payload)
        # The --bayer value travelled from args to the PNG (8-bit RGB here).
        assert _png_ihdr(out + ".png") == (8, 4, 8, 2, 0, 0, 0), _png_ihdr(out + ".png")
    assert len(opener.calls) == 2, opener.calls
    assert opener.calls[1][2] is True, "the re-open must re-scan /dev/cu.usbmodem*"
    assert manager.reconnects == 1 and manager.port is None, manager.reconnects
    checks += 2

    #     A port that is already gone when the first read happens is a CommError.
    try:
        read_frame(
            UnpluggedPort(b""), 0.05, 0.05, echo_sink=NULL_SINK
        )
    except CommError as exc:
        assert "serial read failed" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a read on a vanished port did not raise CommError")

    #     Re-opening is bounded in time: the fake clock advances only by the
    #     backoff sleeps, and the loop must stop inside the budget.
    clock = FakeClock()
    starved = PortManager(
        None,
        115200,
        1.0,
        8,
        opener=ScriptedOpener([]),
        clock=clock,
        sleep=clock.sleep,
        log=devnull,
    )
    try:
        starved.open()
    except CommError as exc:
        assert "open failed after 3 attempt(s) in 1s" in str(exc), exc
        assert 0 < clock.now <= 1.0, clock.now
        checks += 1
    else:
        raise AssertionError("a port that never opens did not exhaust its budget")

    #     And the reconnect *count* is bounded independently of the time budget.
    attempts = ScriptedOpener([FakePort(b""), CommError("gone"), CommError("gone")])
    bounded_clock = FakeClock()
    bounded = PortManager(
        None,
        115200,
        0.5,
        1,
        opener=attempts,
        clock=bounded_clock,
        sleep=bounded_clock.sleep,
        log=devnull,
    )
    bounded.open()
    try:
        bounded.reopen()
    except CommError as exc:
        assert "reconnect 1/1" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("an unopenable port did not fail the reconnect")
    try:
        bounded.reopen()
    except CommError as exc:
        assert "giving up after 1 reconnect(s)" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("reconnects were not capped by --max-reconnects")

    # 11. A corrupt payload is content, not transport: it must be a ChecksumError
    #     and must NOT be retried by re-opening.
    corrupted_wire = bytearray(wire)
    corrupted_wire[len(b"boot: ov5647 chip id 0x5647\r\n") + len(line) + 2] ^= 0xFF
    opener = ScriptedOpener([FakePort(bytes(corrupted_wire)), FakePort(wire)])
    manager = PortManager(
        None, 115200, 1.0, 4, opener=opener, clock=FakeClock(), sleep=FakeClock().sleep,
        log=devnull,
    )
    with tempfile.TemporaryDirectory() as tmp:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                run_capture(Args(out=os.path.join(tmp, "bad")), manager=manager)
            except ChecksumError as exc:
                assert "CRC-32/ISO-HDLC mismatch" in str(exc), exc
            else:
                raise AssertionError("a corrupt payload was not a ChecksumError")
    assert len(opener.calls) == 1 and manager.reconnects == 0, (opener.calls, manager.reconnects)
    checks += 2

    #     A malformed header is framing, not transport: HeaderError must surface
    #     as itself (not as a CommError) and must not trigger a reconnect.
    bad_header = b"<<<FRAME w=8 h=4 fmt=RAW8 len=31 crc32=0x00000000>>>\r\n" + payload
    opener = ScriptedOpener([FakePort(bad_header), FakePort(wire)])
    manager = PortManager(
        None, 115200, 1.0, 4, opener=opener, clock=FakeClock(), sleep=FakeClock().sleep,
        log=devnull,
    )
    with tempfile.TemporaryDirectory() as tmp:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                run_capture(Args(out=os.path.join(tmp, "malformed")), manager=manager)
            except HeaderError as exc:
                assert "self-inconsistent" in str(exc), exc
            else:
                raise AssertionError("a self-inconsistent header was not a HeaderError")
    assert len(opener.calls) == 1 and manager.reconnects == 0, (opener.calls, manager.reconnects)
    checks += 2

    #     A frame that never arrives at all does get retried, and the retry is
    #     itself bounded: the second header-less read exhausts --max-reconnects
    #     and the capture fails with a CommError.
    opener = ScriptedOpener([FakePort(b"no frame here\r\n"), FakePort(b"still nothing\r\n")])
    manager = PortManager(
        None, 115200, 0.5, 1, opener=opener, clock=FakeClock(), sleep=FakeClock().sleep,
        log=devnull,
    )
    with tempfile.TemporaryDirectory() as tmp:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                run_capture(Args(out=os.path.join(tmp, "none")), manager=manager)
            except CommError as exc:
                assert "giving up after 1 reconnect(s)" in str(exc), exc
            else:
                raise AssertionError("a header-less stream was not a CommError")
    assert manager.reconnects == 1 and len(opener.calls) == 2, (manager.reconnects, opener.calls)
    checks += 2

    # 12. An incomplete frame through the real capture path: exit status 3, the
    #     verified payload still written, and no reconnect attempted.
    opener = ScriptedOpener([FakePort(line.encode() + b"\r\n" + payload)])
    manager = PortManager(
        None, 115200, 0.5, 1, opener=opener, clock=FakeClock(), sleep=FakeClock().sleep,
        log=devnull,
    )
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "incomplete")
        with contextlib.redirect_stdout(io.StringIO()) as stdout:
            with contextlib.redirect_stderr(io.StringIO()) as stderr:
                code = run_capture(Args(out=out), manager=manager)
        assert code == 3, code
        assert "incomplete frame" in stderr.getvalue(), stderr.getvalue()
        assert os.path.getsize(out + ".raw") == len(payload), stdout.getvalue()
    assert manager.reconnects == 0, manager.reconnects
    checks += 2

    # 13. The CLI itself: --bayer defaults to "none" (no assumed phase), each
    #     order is selectable, and a bogus one is rejected by argparse.
    parser = build_parser()
    assert parser.parse_args([]).bayer == BAYER_NONE
    assert parser.parse_args([]).max_reconnects > 0
    for order in BAYER_ORDERS:
        assert parser.parse_args(["--bayer", order]).bayer == order, order
        checks += 1
    with contextlib.redirect_stderr(io.StringIO()):
        try:
            parser.parse_args(["--bayer", "GRBG-RG"])
        except SystemExit as exc:
            assert exc.code == 2, exc.code
            checks += 1
        else:
            raise AssertionError("argparse accepted an unknown --bayer value")

    print(f"selftest OK ({checks} checks)")
    return 0


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #


def run_capture(args, manager=None) -> int:
    """Captures one frame, restarting the read after USB re-enumeration.

    A ``CommError`` before a verified frame is transport trouble: re-open and
    restart the scan from a clean buffer. A ``ChecksumError`` means the bytes
    arrived but are wrong, which no amount of re-opening fixes, so it is never
    retried. ``manager`` is injectable for the selftest.
    """
    if manager is None:
        manager = PortManager(
            args.port,
            args.baud,
            getattr(args, "reconnect_timeout", 10.0),
            getattr(args, "max_reconnects", 6),
        )

    header = payload = None
    incomplete = None
    try:
        while True:
            port = manager.open()
            try:
                header, payload, _trailer = read_frame(
                    port,
                    args.timeout,
                    args.payload_timeout,
                    require_trailer=True,
                )
            except IncompleteFrameError as exc:
                # The trailer is missing but the payload is whole; verify and
                # keep it, reporting the incomplete stream separately.
                incomplete = exc
                header, payload = exc.header, exc.payload
                break
            except HeaderError:
                # HeaderError is a CommError subclass, but a malformed header is
                # framing/content, not transport: re-opening cannot fix it and
                # would reset the board for nothing. Report it as itself.
                raise
            except CommError as exc:
                print(f"communication error: {exc}", file=sys.stderr)
                manager.reopen()
                print("retrying from a clean buffer", file=sys.stderr)
                continue
            break
    finally:
        manager.close()

    verify_frame(header, payload)

    print(
        f"frame: {header['width']}x{header['height']} {header['format']} "
        f"len={header['payload_len']} crc32=0x{header['crc32']:08X}"
    )
    print("verified: length OK, CRC-32/ISO-HDLC OK")
    if payload:
        print(
            f"payload: nonzero={sum(1 for b in payload if b)} distinct={len(set(payload))} "
            f"min=0x{min(payload):02X} max=0x{max(payload):02X}"
        )
    else:
        print("payload: empty (a zero-sized geometry)")

    bayer = getattr(args, "bayer", BAYER_NONE)
    if bayer != BAYER_NONE:
        print(f"bayer: {bayer} (phase is a user choice; this project has not verified it)")

    raw_path, png_path = write_outputs(
        args.out or default_prefix(), header, payload, bayer
    )
    print(f"wrote {raw_path} ({len(payload)} bytes)")
    if png_path:
        print(f"wrote {png_path}")

    if incomplete is not None:
        print(f"incomplete frame: {incomplete}", file=sys.stderr)
        return 3
    return 0


def build_parser() -> argparse.ArgumentParser:
    """The CLI. Split out so ``--selftest`` can assert the flags themselves."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", help="serial port; default: first /dev/cu.usbmodem*")
    ap.add_argument("--baud", type=int, default=115200, help="nominal baud (USB CDC ignores it)")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the header")
    ap.add_argument(
        "--payload-timeout",
        type=float,
        default=30.0,
        help="seconds to read the whole payload (2.5 MB frames are slow)",
    )
    ap.add_argument(
        "--reconnect-timeout",
        type=float,
        default=10.0,
        help="seconds budgeted for each USB re-enumeration re-open",
    )
    ap.add_argument(
        "--max-reconnects",
        type=int,
        default=6,
        help="re-open events allowed for one capture before giving up",
    )
    ap.add_argument(
        "--bayer",
        choices=(BAYER_NONE, *BAYER_ORDERS),
        default=BAYER_NONE,
        help="demosaic RAW8/RAW10 with this CFA phase; default: none (grayscale)",
    )
    ap.add_argument("--out", help="output prefix; default: capture-<timestamp>")
    ap.add_argument("--selftest", action="store_true", help="verify the tool with no hardware")
    return ap


def main() -> int:
    args = build_parser().parse_args()

    if args.selftest:
        return selftest()

    try:
        return run_capture(args)
    except HeaderError as exc:
        print(f"header error: {exc}", file=sys.stderr)
        return 1
    except CommError as exc:
        print(f"communication error: {exc}", file=sys.stderr)
        return 1
    except ChecksumError as exc:
        print(f"checksum error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
