#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SyscallNumber {
    Misc = 0,
    Image = 1,
    Input = 2,
    Usb = 3,
}

impl TryFrom<u8> for SyscallNumber {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            x if x == SyscallNumber::Misc as u8 => Ok(SyscallNumber::Misc),
            x if x == SyscallNumber::Image as u8 => Ok(SyscallNumber::Image),
            x if x == SyscallNumber::Input as u8 => Ok(SyscallNumber::Input),
            x if x == SyscallNumber::Usb as u8 => Ok(SyscallNumber::Usb),
            _ => Err(()),
        }
    }
}

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


/// Perform a raw system call.
#[macro_export]
macro_rules! syscall {
    (
        $syscall_num:expr,
        $( out $r0out:ident )? $( in $r0in:expr )? $(,
            $( out $r1out:ident )? $( in $r1in:expr )? $(,
                $( out $r2out:ident )? $( in $r2in:expr )? $(,
                    $( out $r3out:ident )? $( in $r3in:expr )? $(,)?
                )?
            )?
        )?
    ) => {
        ::core::arch::asm!(
            "svc #{syscall_num}",
            $( in("r0") $r0in as usize, )?
            $( lateout("r0") $r0out, )?
            $(
                $( in("r1") $r1in as usize, )?
                $( lateout("r1") $r1out, )?
                $(
                    $( in("r2") $r2in as usize, )?
                    $( lateout("r2") $r2out, )?
                    $(
                        $( in("r3") $r3in as usize, )?
                        $( lateout("r3") $r3out, )?
                    )?
                )?
            )?
            syscall_num = const $syscall_num as u8,
        )
    }
}

pub fn get_serial() -> &'static str {
    let mut ptr: *const [u8; 16];
    unsafe { syscall!(SyscallNumber::Misc, out ptr in MiscSyscall::GetSerial) };
    unsafe { core::str::from_utf8_unchecked(&*ptr) }
}