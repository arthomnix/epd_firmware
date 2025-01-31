#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CsSyscall {
    Acquire = 0,
    Release = 1,
}

impl TryFrom<usize> for CsSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == CsSyscall::Acquire as usize => Ok(CsSyscall::Acquire),
            x if x == CsSyscall::Release as usize => Ok(CsSyscall::Release),
            _ => Err(()),
        }
    }
}