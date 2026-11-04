//! Pure checks on a captured frame's integrity and on the ownership/cache rules
//! that make reading it safe.
//!
//! These encode the failure modes this project actually hit, so they are
//! regression tests for the *classes* of bug, not for one instance:
//!
//! * an engine that stops early (`words < expected`) or overruns (`words > expected`);
//! * a bridge that dropped rows (`DISCARD`);
//! * a payload that is a prefix of data followed by zeros, which passes a naive
//!   "nonzero > 0" test but is not a frame;
//! * a poisoned buffer reaching the host;
//! * reading the destination while the DMA still owns it.

/// Snapshot of one transaction's outcome, as the driver reads it from the DMA and
/// bridge registers. No register access here: the firmware fills this in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransactionOutcome {
    /// `CH1_STATUS0.CH1_CMPLTD_BLK_TFR_SIZE`, converted to a true word count.
    pub words: u32,
    /// 64-bit words the block transfer was programmed for.
    pub expected_words: u32,
    /// `CH1_INTSTATUS0` completion bits were seen.
    pub completed: bool,
    /// A DMA error event is latched.
    pub dma_error: bool,
    /// `MIPI_CSI_BRIDGE.INT_RAW` error bits (DISCARD, VADR_NUM_GT/LT, overrun...).
    pub bridge_errors: u32,
    /// `MIPI_CSI_BRIDGE.DISCARD` specifically.
    pub bridge_discard: bool,
}

impl TransactionOutcome {
    /// True only when the DMA moved exactly the programmed count, completed, and
    /// neither the DMA nor the bridge reported an error.
    ///
    /// # Examples
    ///
    /// ```
    /// use ov5647_core::frame_integrity::TransactionOutcome;
    ///
    /// let ok = TransactionOutcome {
    ///     words: 64_000, expected_words: 64_000, completed: true,
    ///     dma_error: false, bridge_errors: 0, bridge_discard: false,
    /// };
    /// assert!(ok.whole());
    /// ```
    pub fn whole(&self) -> bool {
        self.words == self.expected_words
            && self.completed
            && !self.dma_error
            && self.bridge_errors == 0
    }
}

/// What the content scan found, so "a picture" can be required rather than
/// "some bytes are non-zero".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ContentSummary {
    /// Total bytes of the payload that were scanned.
    pub payload_len: usize,
    /// Count of non-zero bytes in the payload.
    pub nonzero: usize,
    /// Offset of the last non-zero byte, or `None` when the payload is all zero.
    pub last_nonzero: Option<usize>,
    /// Number of distinct byte values seen in the payload.
    pub distinct: u16,
}

impl ContentSummary {
    /// True when the data reaches at least the midpoint of the payload.
    ///
    /// `nonzero > 0` is far too weak: the frame that motivated this test had 1,295
    /// non-zero bytes (0.253 % of a 512,000-byte buffer) ending at offset 8,937 — the
    /// payload reached only 1.75 % of its length — and passed a
    /// `nonzero > 0 && distinct > 1` gate. A real 640-row image has content well below
    /// row 320.
    pub fn spans_frame(&self) -> bool {
        match self.last_nonzero {
            Some(off) => (off + 1) * 2 >= self.payload_len,
            None => false,
        }
    }

    /// True when the payload looks like an image rather than a constant or a
    /// prefix of zeros.
    pub fn plausible(&self) -> bool {
        self.nonzero > 0 && self.distinct > 1
    }

    /// The full announcement predicate: this is what the firmware must require
    /// before sending a frame to the host.
    pub fn may_announce(&self, poisoned: bool) -> bool {
        !poisoned && self.plausible() && self.spans_frame() && self.payload_len > 0
    }
}

/// Buffer ownership state, to be checked before touching the destination.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ownership {
    /// No transfer armed: the CPU may read and write the destination.
    CpuOwned,
    /// A transfer is armed: the DMA owns the destination; the CPU must not touch it.
    DmaOwned,
}

impl Ownership {
    /// Whether the CPU may take a mutable view of the destination.
    pub fn cpu_may_write(self) -> bool {
        matches!(self, Ownership::CpuOwned)
    }

    /// Whether the CPU may read the frame. Reading is only sound once the engine
    /// has stopped, which is the same state — the point is that it is *after*
    /// `wait_frame` has unwound, never during.
    pub fn cpu_may_read(self) -> bool {
        matches!(self, Ownership::CpuOwned)
    }

    /// The only legal transition to arming: from `CpuOwned`. Arming while already
    /// armed is what `CsiCapture::start` must refuse.
    pub fn may_arm(self) -> bool {
        matches!(self, Ownership::CpuOwned)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn whole_frame() -> TransactionOutcome {
        TransactionOutcome {
            words: 64_000,
            expected_words: 64_000,
            completed: true,
            dma_error: false,
            bridge_errors: 0,
            bridge_discard: false,
        }
    }

    #[test]
    fn whole_transaction_passes() {
        assert!(whole_frame().whole());
    }

    #[test]
    fn short_transfer_is_not_whole() {
        let mut o = whole_frame();
        o.words = 63_999;
        assert!(!o.whole());
    }

    #[test]
    fn overrun_is_rejected_as_firmly_as_short() {
        // `words > expected_words` is the one that walks DAR past the buffer.
        let mut o = whole_frame();
        o.words = 64_001;
        assert!(!o.whole());
    }

    #[test]
    fn any_error_bit_defeats_whole() {
        let mut o = whole_frame();
        o.dma_error = true;
        assert!(!o.whole());
        let mut o = whole_frame();
        o.bridge_errors = 1 << 2; // DISCARD
        assert!(!o.whole());
        let mut o = whole_frame();
        o.completed = false;
        assert!(!o.whole());
    }

    #[test]
    fn prefix_of_data_does_not_span_the_frame() {
        // The real failure: 1,295 non-zero bytes ending at offset 8,937 of 512,000.
        let s = ContentSummary {
            payload_len: 512_000,
            nonzero: 1_295,
            last_nonzero: Some(8_937),
            distinct: 26,
        };
        assert!(s.plausible()); // passes the old, weak test...
        assert!(!s.spans_frame()); // ...but must fail the real one
        assert!(!s.may_announce(false));
    }

    #[test]
    fn full_frame_may_be_announced() {
        let s = ContentSummary {
            payload_len: 512_000,
            nonzero: 511_987,
            last_nonzero: Some(511_999),
            distinct: 141,
        };
        assert!(s.may_announce(false));
    }

    #[test]
    fn poisoned_frame_may_never_be_announced() {
        let s = ContentSummary {
            payload_len: 512_000,
            nonzero: 512_000,
            last_nonzero: Some(511_999),
            distinct: 256,
        };
        assert!(s.may_announce(false));
        assert!(
            !s.may_announce(true),
            "a poisoned buffer must not be announced"
        );
    }

    #[test]
    fn all_zero_and_constant_are_not_plausible() {
        let zero = ContentSummary {
            payload_len: 100,
            nonzero: 0,
            last_nonzero: None,
            distinct: 0,
        };
        assert!(!zero.plausible());
        assert!(!zero.spans_frame());
        let constant = ContentSummary {
            payload_len: 100,
            nonzero: 100,
            last_nonzero: Some(99),
            distinct: 1,
        };
        assert!(!constant.plausible());
    }

    #[test]
    fn ownership_gates_the_destination() {
        assert!(Ownership::CpuOwned.cpu_may_write());
        assert!(Ownership::CpuOwned.cpu_may_read());
        assert!(Ownership::CpuOwned.may_arm());
        assert!(!Ownership::DmaOwned.cpu_may_write());
        assert!(!Ownership::DmaOwned.cpu_may_read());
        // Arming while armed is exactly the state that must be refused.
        assert!(!Ownership::DmaOwned.may_arm());
    }
}
