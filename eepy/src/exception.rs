use defmt::Formatter;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct StackFrame {
    pub r0: usize,
    pub r1: usize,
    pub r2: usize,
    pub r3: usize,
    pub r12: usize,
    pub lr: *const u8,
    pub pc: *const u8,
    pub xpsr: usize,
}

impl defmt::Format for StackFrame {
    fn format(&self, fmt: Formatter) {
        defmt::write!(
            fmt,
            "r0=0x{:x} r1=0x{:x} r2=0x{:x} r3=0x{:x} r12=0x{:x} lr={:x} pc={:x} xpsr=0x{:x}",
            self.r0,
            self.r1,
            self.r2,
            self.r3,
            self.r12,
            self.lr,
            self.pc,
            self.xpsr,
        )
    }
}
