/* Keep the stack out of the reserved band at the top of L2MEM.
 *
 * The vendored HAL's `memory.x` reserves its 256 KiB cache at the LOW end of L2MEM
 * and then puts the stack top at a *literal* 0x4FFAE000:
 *
 *     RAM : ORIGIN = 0x4FF00000 + 0x40000, LENGTH = 0x4FFAE000 - 0x40000 - 0x4FF00000
 *     .stack: . = ORIGIN(RWDATA) + LENGTH(RWDATA);  _stack_start = ABSOLUTE(.)
 *
 * so `.stack` runs 0x4FF92A74..0x4FFAE000 and the top 0x12000 of L2MEM is left alone.
 *
 * TWO CANDIDATE OWNERS FOR 0x4FFA0000..0x4FFC0000, AND THE EVIDENCE DOES NOT
 * SEPARATE THEM:
 *
 *   - `docs/research/stack-and-cache-window.md` reads that band as the real 128 KiB
 *     L2 cache (the cache at the TOP of L2MEM, not the low end this `memory.x`
 *     assumes), which would put 0x4FFAE000 - 0x4FFA0000 = 57,344 B of the stack's
 *     top inside live cache storage. It leaves that reading UNVERIFIED and names the
 *     measurement that would close it (`pre-v3-memory-map.md` §7).
 *   - the vendored `memory.x`'s own comment attributes the same band to the
 *     "v3.x ROM BSS/stack region", i.e. protected ROM memory.
 *
 * Both could produce the same symptom, and the experiment below is consistent with
 * either, so this file deliberately does **not** claim which. What it claims is only
 * what was measured:
 *
 *   With `_stack_start = 0x4FFAE000`, stack-resident values read zero after anything
 *   that displaced memory at the top of L2MEM -- writing 64 KiB of PSRAM was enough to
 *   zero the caller's `mode`, `plan.frame_len` read 512000 at one site and 0 at the
 *   next, `distinct_bytes` read 0 beside `nonzero_bytes = 1279`, `frame::analyze`
 *   faulted with `Instruction access fault mepc=0x00000000`, and every symptom moved
 *   when code changed. `.bss` below 0x4FFA0000 and every PSRAM and register read
 *   stayed sound. It survived `opt-level=1`.
 *
 *   With `_stack_start = 0x4FFA0000`, all of those stop: the same PSRAM sweeps leave
 *   `mode` intact, the scan's aggregates agree with each other, and `frame::analyze`
 *   runs and agrees with the independent scan to the byte.
 *
 * That is the one-variable experiment. It establishes that the top of the stack must
 * not live in 0x4FFA0000..0x4FFC0000; it does not establish who owns that band.
 *
 * DO NOT read this as "the 57 KiB can be reclaimed by changing the L2 cache split".
 * That would be wrong if the band belongs to the ROM. The second-stage bootloader and
 * its cache split are safety invariants and are untouched by this file: only the stack
 * *pointer* moves, and `.stack` is NOLOAD, so the band is simply left unused.
 *
 * Cost: the stack is 0x4FF92A74..0x4FFA0000, 54,668 B instead of 112,012 B. If a run
 * misbehaves in a new way after this, check for a stack-usage symptom before blaming
 * the capture chain.
 */
SECTIONS
{
  _stack_start = 0x4FFA0000;
  _stack_start_cpu0 = 0x4FFA0000;
}
