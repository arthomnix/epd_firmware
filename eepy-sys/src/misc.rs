use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MiscSyscall {
    GetSerial = 0,
}

impl TryFrom<usize> for MiscSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == MiscSyscall::GetSerial as usize => Ok(MiscSyscall::GetSerial),
            _ => Err(()),
        }
    }
}

pub fn get_serial() -> &'static str {
    let mut ptr: *const [u8; 16];
    unsafe {
        syscall!(
            SyscallNumber::Misc,
            out ptr in MiscSyscall::GetSerial,
        );
        core::str::from_utf8_unchecked(&*ptr)
    }
}