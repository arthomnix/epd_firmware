#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UsbSyscall {
    UsbRet = 0,
}

impl TryFrom<usize> for UsbSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == UsbSyscall::UsbRet as usize => Ok(UsbSyscall::UsbRet),
            _ => Err(()),
        }
    }
}