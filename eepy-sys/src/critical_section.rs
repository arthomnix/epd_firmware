#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CsSyscall {
    Acquire = 0,
    Release = 1,
}