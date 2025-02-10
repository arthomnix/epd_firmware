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

    match SyscallNumber::from_repr(syscall_num) {
        Some(SyscallNumber::Misc) => misc::handle_misc(stack_values),
        Some(SyscallNumber::Image) => image::handle_image(stack_values),
        Some(SyscallNumber::Input) => input::handle_input(stack_values),
        Some(SyscallNumber::Usb) => crate::usb::handle_usb(stack_values),
        Some(SyscallNumber::Exec) => handle_exec(stack_values, using_psp),
        Some(SyscallNumber::CriticalSection) => cs::handle_cs(stack_values),
        Some(SyscallNumber::Flash) => flash::handle_flash(stack_values),
        Some(SyscallNumber::KvStore) => kv_store::handle_kv_store(stack_values),
        None => panic!("illegal syscall"),
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
    use fw16_epd_bsp::pac::Peripherals;
    use crate::{SERIAL_NUMBER};
    use super::StackFrame;

    pub(super) fn handle_misc(stack_values: &mut StackFrame) {
        match MiscSyscall::from_repr(stack_values.r0) {
            Some(MiscSyscall::GetSerial) => handle_get_serial(stack_values),
            Some(MiscSyscall::LogMessage) => handle_log_message(stack_values),
            Some(MiscSyscall::GetTimeMicros) => handle_get_time_micros(stack_values),
            None => panic!("illegal syscall"),
        }
    }

    fn handle_get_serial(stack_values: &mut StackFrame) {
        let buf = stack_values.r1 as *mut [u8; 16];
        unsafe {
            (*buf).copy_from_slice(SERIAL_NUMBER.get().unwrap());
        }
    }

    fn handle_log_message(stack_values: &mut StackFrame) {
        let log_level = LogLevel::from_repr(stack_values.r1).expect("invalid log level");
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

    fn handle_get_time_micros(stack_values: &mut StackFrame) {
        let timer = unsafe { Peripherals::steal() }.TIMER;
        stack_values.r1 = timer.timelr().read().bits() as usize;
        stack_values.r0 = timer.timehr().read().bits() as usize;
    }
}

mod image {
    use eepy_sys::image::ImageSyscall;
    use fw16_epd_bsp::hal::Sio;
    use fw16_epd_bsp::pac;
    use tp370pgh01::IMAGE_BYTES;
    use crate::IMAGE_BUFFER;
    use crate::core1::{ToCore0Message, ToCore1Message};
    use super::StackFrame;

    pub(super) fn handle_image(stack_values: &mut StackFrame) {
        match ImageSyscall::from_repr(stack_values.r0) {
            Some(ImageSyscall::Refresh) => handle_refresh(stack_values),
            Some(ImageSyscall::MaybeRefresh) => handle_maybe_refresh(stack_values),
            None => panic!("illegal syscall"),
        }
    }

    fn handle_refresh(stack_values: &mut StackFrame) {
        let fast_refresh = stack_values.r1 != 0;
        let image: &[u8; IMAGE_BYTES] = unsafe { &*(stack_values.r2 as *const [u8; IMAGE_BYTES]) };
        critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));

        let mut fifo = Sio::new(unsafe { pac::Peripherals::steal() }.SIO).fifo;
        let message = if fast_refresh { ToCore1Message::RefreshFast } else { ToCore1Message::RefreshNormal };
        fifo.write_blocking(message as u32);

        if fifo.read_blocking() != ToCore0Message::RefreshAck as u32 {
            panic!("received incorrect message");
        }
    }

    fn handle_maybe_refresh(stack_values: &mut StackFrame) {
        let fast_refresh = stack_values.r1 != 0;
        let image: &[u8; IMAGE_BYTES] = unsafe { &*(stack_values.r2 as *const [u8; IMAGE_BYTES]) };
        critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));

        let mut fifo = Sio::new(unsafe { pac::Peripherals::steal() }.SIO).fifo;
        let message = if fast_refresh { ToCore1Message::MaybeRefreshFast } else { ToCore1Message::MaybeRefreshNormal };
        if fifo.is_write_ready() {
            fifo.write(message as u32);
        }
    }
}

mod input {
    use core::sync::atomic::Ordering;
    use eepy_sys::input::InputSyscall;
    use eepy_sys::input_common::Event;
    use eepy_sys::SafeOption;
    use crate::{EVENT_QUEUE, TOUCH_ENABLED};
    use super::StackFrame;

    pub(super) fn handle_input(stack_values: &mut StackFrame) {
        match InputSyscall::from_repr(stack_values.r0) {
            Some(InputSyscall::NextEvent) => handle_next_event(stack_values),
            Some(InputSyscall::SetTouchEnabled) => handle_set_touch_enabled(stack_values),
            Some(InputSyscall::HasEvent) => handle_has_event(stack_values),
            None => panic!("illegal syscall"),
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
        match CsSyscall::from_repr(stack_values.r0) {
            Some(CsSyscall::Acquire) => handle_acquire(stack_values),
            Some(CsSyscall::Release) => handle_release(stack_values),
            None => panic!("illegal syscall"),
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
    use eepy_sys::flash::FlashSyscall;
    use eepy_sys::header::{SLOT_SIZE, XIP_BASE};
    use crate::exception::StackFrame;
    use crate::flash::{erase, erase_and_program, program};

    pub(super) fn handle_flash(stack_values: &mut StackFrame) {
        match FlashSyscall::from_repr(stack_values.r0) {
            Some(FlashSyscall::Erase) => handle_erase(stack_values),
            Some(FlashSyscall::Program) => handle_program(stack_values),
            Some(FlashSyscall::EraseAndProgram) => handle_erase_and_program(stack_values),
            None => panic!("illegal syscall"),
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

    fn handle_erase(stack_values: &mut StackFrame) {
        let start_addr = stack_values.r1 as u32;
        let len = stack_values.r2 as u32;
        assert_permissions(stack_values, start_addr, len);
        if start_addr % 4096 != 0 || len % 4096 != 0 {
            panic!("unaligned flash erase");
        }

        unsafe {
            erase(start_addr, len);
        }
    }

    fn handle_program(stack_values: &mut StackFrame) {
        let start_addr = stack_values.r1 as u32;
        let data = unsafe { core::slice::from_raw_parts(stack_values.r3 as *const u8, stack_values.r2) };
        assert_permissions(stack_values, start_addr, data.len() as u32);
        if start_addr % 256 != 0 || data.len() % 256 != 0 {
            panic!("unaligned flash program");
        }

        unsafe {
            program(start_addr, data);
        }
    }

    fn handle_erase_and_program(stack_values: &mut StackFrame) {
        let start_addr = stack_values.r1 as u32;
        let data = unsafe { core::slice::from_raw_parts(stack_values.r3 as *const u8, stack_values.r2) };
        assert_permissions(stack_values, start_addr, data.len() as u32);
        if start_addr % 4096 != 0 || data.len() % 4096 != 0 {
            panic!("unaligned flash erase");
        }

        unsafe {
            erase_and_program(start_addr, data);
        }
    }
}

mod kv_store {
    use core::hash::Hasher;
    use siphasher::sip::SipHasher;
    use tickv::{ErrorCode, TicKV};
    use eepy_sys::header::{slot, SLOT_SIZE};
    use eepy_sys::kv_store::{AppendKeyError, DeleteKeyError, KvStoreSyscall, PutKeyError, ReadKeyArgs, ReadKeyError, WriteKeyArgs};
    use eepy_sys::SafeResult;
    use crate::exception::StackFrame;
    use crate::tickv::{with_tickv, EepyFlashController};

    pub(super) fn handle_kv_store(stack_values: &mut StackFrame) {
        match KvStoreSyscall::from_repr(stack_values.r0) {
            Some(KvStoreSyscall::ReadKey) => handle_kv_get(stack_values),
            Some(KvStoreSyscall::AppendKey) => handle_kv_append(stack_values),
            Some(KvStoreSyscall::PutKey) => handle_kv_put(stack_values),
            Some(KvStoreSyscall::InvalidateKey) => handle_kv_invalidate(stack_values),
            Some(KvStoreSyscall::ZeroiseKey) => handle_kv_zeroise(stack_values),
            None => panic!("illegal syscall"),
        }
    }

    fn hash(stack_values: &mut StackFrame, key: &[u8]) -> u64 {
        let slot_number = (stack_values.pc as usize) / SLOT_SIZE;
        let slot_header = unsafe { slot(slot_number as u8) };
        let program_name = unsafe { core::slice::from_raw_parts((*slot_header).name_ptr, (*slot_header).name_len) };

        let mut hasher = SipHasher::new();
        hasher.write(program_name);
        hasher.write(key);
        hasher.finish()
    }

    fn append_key_gc(tickv: &TicKV<EepyFlashController, 4096>, key: u64, value: &[u8]) -> Result<(), ErrorCode> {
        let mut res = tickv.append_key(key, value).map(|_| ());
        if let Err(ErrorCode::RegionFull | ErrorCode::FlashFull) = res {
            res = tickv.garbage_collect().map(|_| ());
            if res.is_ok() {
                res = tickv.append_key(key, value).map(|_| ());
            }
        }
        res
    }

    fn handle_kv_get(stack_values: &mut StackFrame) {
        let args = stack_values.r1 as *const ReadKeyArgs;
        let key = unsafe { core::slice::from_raw_parts((*args).key_ptr, (*args).key_len) };
        let buf = unsafe { core::slice::from_raw_parts_mut((*args).buf_ptr, (*args).buf_len) };

        let hashed_key = hash(stack_values, key);
        let res = unsafe { with_tickv(|tickv| tickv.get_key(hashed_key, buf)) };

        let res_ptr = stack_values.r2 as *mut SafeResult<usize, ReadKeyError>;
        let res: SafeResult<usize, ReadKeyError> = res
            .map(|(_, read)| read)
            .map_err(|e| e.try_into().unwrap())
            .into();
        unsafe { res_ptr.write(res) };
    }

    fn handle_kv_append(stack_values: &mut StackFrame) {
        let args = stack_values.r1 as *const WriteKeyArgs;
        let key = unsafe { core::slice::from_raw_parts((*args).key_ptr, (*args).key_len) };
        let value = unsafe { core::slice::from_raw_parts((*args).value_ptr, (*args).value_len) };

        let hashed_key = hash(stack_values, key);
        let res = unsafe { with_tickv(|tickv| append_key_gc(tickv, hashed_key, value)) };

        let res_ptr = stack_values.r2 as *mut SafeResult<(), AppendKeyError>;
        let res: SafeResult<(), AppendKeyError> = res
            .map(|_| ())
            .map_err(|e| e.try_into().unwrap())
            .into();
        unsafe { res_ptr.write(res) };
    }

    fn handle_kv_put(stack_values: &mut StackFrame) {
        let args = stack_values.r1 as *const WriteKeyArgs;
        let key = unsafe { core::slice::from_raw_parts((*args).key_ptr, (*args).key_len) };
        let value = unsafe { core::slice::from_raw_parts((*args).value_ptr, (*args).value_len) };

        let hashed_key = hash(stack_values, key);
        let mut res = unsafe { with_tickv(|tickv| append_key_gc(tickv, hashed_key, value)) };

        // if there is already a value for this key, invalidate it and write the new value
        if let Err(ErrorCode::KeyAlreadyExists) = res {
            res = unsafe { with_tickv(|tickv| tickv.invalidate_key(hashed_key).map(|_| ())) };
            if res.is_ok() {
                res = unsafe { with_tickv(|tickv| append_key_gc(tickv, hashed_key, value)) };
            }
        }

        let res_ptr = stack_values.r2 as *mut SafeResult<(), PutKeyError>;
        let res: SafeResult<(), PutKeyError> = res
            .map(|_| ())
            .map_err(|e| e.try_into().unwrap())
            .into();
        unsafe { res_ptr.write(res) };
    }

    fn handle_kv_invalidate(stack_values: &mut StackFrame) {
        let key = unsafe { core::slice::from_raw_parts(stack_values.r1 as *const u8, stack_values.r2) };
        let hashed_key = hash(stack_values, key);
        let res = unsafe { with_tickv(|tickv| tickv.invalidate_key(hashed_key)) };

        let res_ptr = stack_values.r3 as *mut SafeResult<(), DeleteKeyError>;
        let res: SafeResult<(), DeleteKeyError> = res
            .map(|_| ())
            .map_err(|e| e.try_into().unwrap())
            .into();
        unsafe { res_ptr.write(res) };
    }

    fn handle_kv_zeroise(stack_values: &mut StackFrame) {
        let key = unsafe { core::slice::from_raw_parts(stack_values.r1 as *const u8, stack_values.r2) };
        let hashed_key = hash(stack_values, key);
        let res = unsafe { with_tickv(|tickv| tickv.zeroise_key(hashed_key)) };

        let res_ptr = stack_values.r3 as *mut SafeResult<(), DeleteKeyError>;
        let res: SafeResult<(), DeleteKeyError> = res
            .map(|_| ())
            .map_err(|e| e.try_into().unwrap())
            .into();
        unsafe { res_ptr.write(res) };
    }
}