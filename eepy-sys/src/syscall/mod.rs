/// Perform a raw system call.
#[macro_export]
macro_rules! syscall {
    (
        $syscall_num:expr,
        $( out $r0out:ident )? $( in $r0in:expr )?,
        $( out $r1out:ident )? $( in $r1in:expr )?,
        $( out $r2out:ident )? $( in $r2in:expr )?,
        $( out $r3out:ident )? $( in $r3in:expr )?,
    ) => {
        asm!(
            "svc #{syscall_num}",
            $( in("r0") $r0in, )?
            $( lateout("r0") $r0out, )?
            $( in("r1") $r1in, )?
            $( lateout("r1") $r1out, )?
            $( in("r2") $r2in, )?
            $( lateout("r2") $r2out, )?
            $( in("r3") $r3in, )?
            $( lateout("r3") $r3out, )?
            syscall_num = const $syscall_num,
        );
    }
}
