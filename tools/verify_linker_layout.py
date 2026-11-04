#!/usr/bin/env python3
"""Post-link validation of the stack/L2-cache-window layout.

Checks that the built ELF's stack top sits at or below the verified boundary
`0x4FFA0000`, which is where the pre-v3 L2 cache window (or the ROM's reserved
region) begins. A `_stack_start` above that boundary means the top of the stack
shares memory with a reserved band, which produced a whole class of
correct-where-produced/zero-where-used corruptions that cost this project a great
deal of debugging.

This is a *post-link* check: it reads the built ELF, so a bad stack placement is
caught at build time without ever flashing a firmware with a broken stack.

Usage:
    python3 tools/verify_linker_layout.py <path-to-elf>

Exit 0: layout is sound. Exit 1: stack top is above the boundary.
"""
import subprocess
import sys

# The verified boundary: on pre-v3, L2 cache / ROM reserved region starts at
# 0x4FFA0000 for a 128 KiB split.
STACK_TOP_MAX = 0x4FFA0000
STACK_TOP_MAX_STR = "0x4FFA0000"


def read_u32_with_nm(elf, symbol):
    """Return the value of a linker symbol via llvm-nm (aarch64 host)."""
    # Find llvm-nm next to llvm-objdump in the rust toolchain.
    from pathlib import Path
    import glob
    cands = glob.glob(
        str(Path.home())
        + "/.rustup/toolchains/*/lib/rustlib/aarch64-apple-darwin/bin/llvm-nm"
    )
    nm = cands[0] if cands else "llvm-nm"
    out = subprocess.check_output([nm, elf], text=True)
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] == symbol:
            return int(parts[0], 16)
    raise ValueError(f"symbol {symbol} not found in {elf}")


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: verify_linker_layout.py <elf>")
        return 2
    elf = sys.argv[1]
    top = read_u32_with_nm(elf, "_stack_start")
    print(f"_stack_start = {hex(top)}  (boundary {STACK_TOP_MAX_STR})")
    if top <= STACK_TOP_MAX:
        print("PASS: stack top is at or below the verified boundary.")
        return 0
    print(
        "FAIL: stack top is above the verified boundary; the stack overlaps the\n"
        "      reserved L2MEM band. Fix ld/stack-top.x.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
