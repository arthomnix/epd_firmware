use core::arch::{asm, global_asm};
use defmt::trace;
use eepy_sys::exec::exec;
use eepy_sys::header::slot;
use eepy_sys::syscall::SyscallNumber;
use crate::exception::StackFrame;
use crate::SRAM_END;

global_asm!(include_str!("syscall.s"));

/// Main syscall (SVC) handler.
///
/// This function is called by the assembly trampoline code in `syscall.s`.
///
/// `sp` contains the caller's stack pointer value (taken from either MSP or PSP depending on which
/// stack was active). This is where the register values are pushed when the exception is triggered,
/// so this is where we need to read/write to interface with the caller.
/// This stack location contains (in order): r0, r1, r2, r3, r12, lr, pc, xPSR.
/// The pc value from the stack can be used to extract the literal operand from the SVC instruction
/// which triggered the syscall.
#[no_mangle]
extern "C" fn handle_syscall(sp: *mut StackFrame, using_psp: bool) {
    // Stack contains R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
    let stack_values = unsafe { &mut *sp };
    let syscall_num = unsafe { *stack_values.pc.sub(2) };

    trace!("syscall: sp={} imm={:x} {}", sp, syscall_num, stack_values);

    match SyscallNumber::try_from(syscall_num) {
        Ok(SyscallNumber::Misc) => misc::handle_misc(stack_values),
        Ok(SyscallNumber::Image) => image::handle_image(stack_values),
        Ok(SyscallNumber::Input) => input::handle_input(stack_values),
        Ok(SyscallNumber::Usb) => crate::usb::handle_usb(stack_values),
        Ok(SyscallNumber::Exec) => handle_exec(stack_values, using_psp),
        Ok(SyscallNumber::CriticalSection) => cs::handle_cs(stack_values),
        Ok(SyscallNumber::Flash) => flash::handle_flash(stack_values),
        Err(_) => panic!("illegal syscall"),
    }
}

fn handle_exec(stack_values: &mut StackFrame, using_psp: bool) {
    // cleanup previous program's USB
    crate::usb::handle_uninit();
    crate::usb::handle_clear_handler();

    // disable privileged mode
    unsafe {
        asm!(
            "msr control, {control_0b11}",
            "isb",
            control_0b11 = in(reg) 0b11,
        );
    }

    let slot_n = stack_values.r0 as u8;
    unsafe {
        let program = slot(slot_n);
        if !(*program).is_valid() {
            panic!("tried to exec invalid program");
        }

        stack_values.pc = core::mem::transmute((*program).entry);
        stack_values.lr = program_return_handler as *const u8;

        // Move the saved registers to the top of program memory
        // This makes sure all programs start with an empty program stack
        if using_psp {
            let ptr: *mut StackFrame = SRAM_END.sub(size_of::<StackFrame>()).cast();
            ptr.write(*stack_values);
            asm!(
                "msr psp, {ptr}",
                ptr = in(reg) ptr,
            );
        } else {
            // If the saved registers are on the main stack, just set the PSP to
            // the top of program memory
            asm!(
                "msr psp, {ptr}",
                ptr = in(reg) SRAM_END,
            )
        }

        (*program).load();
    }
}

// NOTE: this function runs in unprivileged thread mode
extern "C" fn program_return_handler() -> ! {
    exec(0);
}

mod misc {
    use defmt::{debug, error, info, trace, warn};
    use eepy_sys::header::{SLOT_SIZE, XIP_BASE};
    use eepy_sys::misc::{LogLevel, MiscSyscall};
    use crate::SERIAL_NUMBER;
    use super::StackFrame;

    pub(super) fn handle_misc(stack_values: &mut StackFrame) {
        match MiscSyscall::try_from(stack_values.r0) {
            Ok(MiscSyscall::GetSerial) => handle_get_serial(stack_values),
            Ok(MiscSyscall::LogMessage) => handle_log_message(stack_values),
            Err(_) => panic!("illegal syscall"),
        }
    }

    fn handle_get_serial(stack_values: &mut StackFrame) {
        let buf = stack_values.r1 as *mut [u8; 16];
        unsafe {
            (*buf).copy_from_slice(SERIAL_NUMBER.get().unwrap());
        }
    }

    fn handle_log_message(stack_values: &mut StackFrame) {
        let log_level: LogLevel = stack_values.r1.try_into().expect("invalid log level");
        let len = stack_values.r2;
        let ptr = stack_values.r3 as *const u8;
        let s = unsafe { core::str::from_utf8_unchecked(core::slice::from_raw_parts(ptr, len)) };
        let slot_n = (stack_values.pc as usize - XIP_BASE as usize) / SLOT_SIZE;

        match log_level {
            LogLevel::Trace => trace!("[PROGRAM:{}] {}", slot_n, s),
            LogLevel::Debug => debug!("[PROGRAM:{}] {}", slot_n, s),
            LogLevel::Info => info!("[PROGRAM:{}] {}", slot_n, s),
            LogLevel::Warn => warn!("[PROGRAM:{}] {}", slot_n, s),
            LogLevel::Error => error!("[PROGRAM:{}] {}", slot_n, s),
        }
    }
}

mod image {
    use core::sync::atomic::Ordering;
    use eepy_sys::image::{ImageSyscall, RefreshBlockMode};
    use tp370pgh01::IMAGE_BYTES;
    use crate::{DO_REFRESH, FAST_REFRESH, IMAGE_BUFFER, REFRESHING};
    use super::StackFrame;

    pub(super) fn handle_image(stack_values: &mut StackFrame) {
        match ImageSyscall::try_from(stack_values.r0) {
            Ok(ImageSyscall::WriteImage) => handle_write_image(stack_values),
            Ok(ImageSyscall::Refresh) => handle_refresh(stack_values),
            Err(_) => panic!("illegal syscall"),
        }
    }

    fn handle_write_image(stack_values: &mut StackFrame) {
        let image: &[u8; IMAGE_BYTES] = unsafe { &*(stack_values.r1 as *const [u8; IMAGE_BYTES]) };
        critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));
    }

    fn handle_refresh(stack_values: &mut StackFrame) {
        let fast_refresh = stack_values.r1 != 0;
        let blocking_mode = RefreshBlockMode::try_from(stack_values.r2).expect("illegal refresh blocking mode");

        DO_REFRESH.store(true, Ordering::Relaxed);
        FAST_REFRESH.store(fast_refresh, Ordering::Relaxed);
        cortex_m::asm::sev();

        if matches!(blocking_mode, RefreshBlockMode::BlockAcknowledge | RefreshBlockMode::BlockFinish) {
            while DO_REFRESH.load(Ordering::Relaxed) {}
        }

        if matches!(blocking_mode, RefreshBlockMode::BlockFinish) {
            while REFRESHING.load(Ordering::Relaxed) {}
        }
    }
}

mod input {
    use core::sync::atomic::Ordering;
    use eepy_sys::input::{Event, InputSyscall};
    use eepy_sys::SafeOption;
    use crate::{EVENT_QUEUE, TOUCH_ENABLED};
    use super::StackFrame;

    pub(super) fn handle_input(stack_values: &mut StackFrame) {
        match InputSyscall::try_from(stack_values.r0) {
            Ok(InputSyscall::NextEvent) => handle_next_event(stack_values),
            Ok(InputSyscall::SetTouchEnabled) => handle_set_touch_enabled(stack_values),
            Ok(InputSyscall::HasEvent) => handle_has_event(stack_values),
            Err(_) => panic!("illegal syscall"),
        }
    }

    fn handle_next_event(stack_values: &mut StackFrame) {
        let next_event = critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).pop());
        let ptr = stack_values.r1 as *mut SafeOption<Event>;
        unsafe { ptr.write(next_event.into()) };
    }

    fn handle_set_touch_enabled(stack_values: &mut StackFrame) {
        let enable = stack_values.r1 != 0;
        TOUCH_ENABLED.store(enable, Ordering::Relaxed);
        if !enable {
            critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).clear());
        }
    }

    fn handle_has_event(stack_values: &mut StackFrame) {
        let empty = critical_section::with(|cs| EVENT_QUEUE.borrow_ref(cs).is_empty());
        stack_values.r0 = (!empty) as usize;
    }
}

mod cs {
    use core::sync::atomic::Ordering;
    use eepy_sys::critical_section::CsSyscall;
    use fw16_epd_bsp::pac;
    use fw16_epd_bsp::pac::interrupt;
    use crate::exception::StackFrame;

    pub(super) fn handle_cs(stack_values: &mut StackFrame) {
        match CsSyscall::try_from(stack_values.r0) {
            Ok(CsSyscall::Acquire) => handle_acquire(stack_values),
            Ok(CsSyscall::Release) => handle_release(stack_values),
            Err(_) => panic!("illegal syscall"),
        }
    }

    // USBCTRL_IRQ is the only interrupt that might cause anything to happen
    // with program memory

    fn handle_acquire(stack_values: &mut StackFrame) {
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
        stack_values.r0 = pac::NVIC::is_enabled(interrupt::USBCTRL_IRQ) as usize;
        pac::NVIC::mask(interrupt::USBCTRL_IRQ);
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
    }

    fn handle_release(stack_values: &mut StackFrame) {
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
        if stack_values.r1 != 0 {
            unsafe {
                pac::NVIC::unmask(interrupt::USBCTRL_IRQ);
            }
        }
        core::sync::atomic::compiler_fence(Ordering::SeqCst);
    }
}

mod flash {
    use core::sync::atomic::Ordering;
    use eepy_sys::flash::FlashSyscall;
    use eepy_sys::header::{SLOT_SIZE, XIP_BASE};
    use fw16_epd_bsp::pac;
    use crate::exception::StackFrame;
    use crate::{FLASHING, FLASHING_ACK};

    pub(super) fn handle_flash(stack_values: &mut StackFrame) {
        match FlashSyscall::try_from(stack_values.r0) {
            Ok(FlashSyscall::Erase) => handle_erase(stack_values),
            Ok(FlashSyscall::Program) => handle_program(stack_values),
            Ok(FlashSyscall::EraseAndProgram) => handle_erase_and_program(stack_values),
            Ok(FlashSyscall::InvalidateCache) => handle_invalidate_cache(),
            Err(_) => panic!("illegal syscall"),
        }
    }

    fn assert_permissions(stack_values: &mut StackFrame, start: u32, len: u32) {
        let slot_n = (stack_values.pc as usize - XIP_BASE as usize) / SLOT_SIZE;
        let start_addr_xip = start as usize + XIP_BASE as usize;
        let end_addr_xip = start_addr_xip + len as usize;

        if slot_n == 0 {
            // Slot 0 (launcher) can write any flash except kernel
            if start_addr_xip < (XIP_BASE as usize + 128 * 1024) {
                panic!("illegal flash write");
            }
            return;
        }

        let slot_start = XIP_BASE as usize + (slot_n * SLOT_SIZE);
        let slot_end = slot_start + SLOT_SIZE;
        if start_addr_xip < slot_start || end_addr_xip > slot_end {
            panic!("illegal flash write");
        }
    }

    fn begin() {
        // Make sure core1 is running code from RAM with interrupts disabled
        FLASHING.store(true, Ordering::Relaxed);
        cortex_m::asm::sev();
        // Wait until core1 has acknowledged that it is now in RAM code
        while !FLASHING_ACK.load(Ordering::Relaxed) {}
        // Disable interrupts on this core
        cortex_m::interrupt::disable();
    }

    fn end() {
        // Enable interrupts
        unsafe { cortex_m::interrupt::enable() }
        // Wake up core1
        FLASHING.store(false, Ordering::Relaxed);
        cortex_m::asm::sev();
    }

    fn handle_erase(stack_values: &mut StackFrame) {
        let start_addr = stack_values.r1 as u32;
        let len = stack_values.r2 as u32;
        assert_permissions(stack_values, start_addr, len);
        if start_addr % 4096 != 0 || len % 4096 != 0 {
            panic!("unaligned flash erase");
        }

        begin();
        unsafe {
            rp2040_flash::flash::flash_range_erase(start_addr, len, true);
        }
        end();
    }

    fn handle_program(stack_values: &mut StackFrame) {
        let start_addr = stack_values.r1 as u32;
        let data = unsafe { core::slice::from_raw_parts(stack_values.r3 as *const u8, stack_values.r2) };
        assert_permissions(stack_values, start_addr, data.len() as u32);
        if start_addr % 256 != 0 || data.len() % 256 != 0 {
            panic!("unaligned flash program");
        }

        begin();
        unsafe {
            rp2040_flash::flash::flash_range_program(start_addr, data, true);
        }
        end();
    }

    fn handle_erase_and_program(stack_values: &mut StackFrame) {
        let start_addr = stack_values.r1 as u32;
        let data = unsafe { core::slice::from_raw_parts(stack_values.r3 as *const u8, stack_values.r2) };
        assert_permissions(stack_values, start_addr, data.len() as u32);
        if start_addr % 4096 != 0 || data.len() % 4096 != 0 {
            panic!("unaligned flash erase");
        }

        begin();
        unsafe {
            rp2040_flash::flash::flash_range_erase_and_program(start_addr, data, true);
        }
        end();
    }

    fn handle_invalidate_cache() {
        unsafe {
            let xip = pac::Peripherals::steal().XIP_CTRL;
            xip.flush().write(|w| w.flush().set_bit());
            xip.flush().read();
        };
    }
}