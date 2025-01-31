use crate::syscall;
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MiscSyscall {
    GetSerial = 0,
    LogMessage = 1,
}

impl TryFrom<usize> for MiscSyscall {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            x if x == MiscSyscall::GetSerial as usize => Ok(MiscSyscall::GetSerial),
            x if x == MiscSyscall::LogMessage as usize => Ok(MiscSyscall::LogMessage),
            _ => Err(()),
        }
    }
}

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
}

impl TryFrom<usize> for LogLevel {
    type Error = ();

    fn try_from(value: usize) -> Result<Self, ()> {
        match value {
            x if x == LogLevel::Trace as usize => Ok(LogLevel::Trace),
            x if x == LogLevel::Info as usize => Ok(LogLevel::Info),
            x if x == LogLevel::Debug as usize => Ok(LogLevel::Debug),
            x if x == LogLevel::Warn as usize => Ok(LogLevel::Warn),
            x if x == LogLevel::Error as usize => Ok(LogLevel::Error),
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

pub fn log(message: &str, level: LogLevel) {
    let len = message.len();
    let ptr = message.as_ptr();

    unsafe {
        syscall!(
            SyscallNumber::Misc,
            in MiscSyscall::LogMessage,
            in level,
            in len,
            in ptr,
        );
    }
}

pub fn trace(message: &str) {
    log(message, LogLevel::Trace);
}

pub fn debug(message: &str) {
    log(message, LogLevel::Debug);
}

pub fn info(message: &str) {
    log(message, LogLevel::Info);
}

pub fn warn(message: &str) {
    log(message, LogLevel::Warn);
}

pub fn error(message: &str) {
    log(message, LogLevel::Error);
}