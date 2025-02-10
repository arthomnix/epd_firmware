#[repr(u8)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SyscallNumber {
    Misc = 0,
    Image = 1,
    Input = 2,
    Usb = 3,
    Exec = 4,
    CriticalSection = 5,
    Flash = 6,
    KvStore = 7,
}

/// Perform a raw system call.
#[macro_export]
#[cfg(all(target_os = "none", target_arch = "arm"))]
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

#[macro_export]
#[cfg(not(all(target_os = "none", target_arch = "arm")))]
macro_rules! syscall {
    ( $( $_foo:tt )* ) => { compile_error!("Cannot use eepyOS syscalls on non-eepyOS platforms") };
}