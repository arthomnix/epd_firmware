use core::arch::global_asm;
use core::sync::atomic::Ordering;
use defmt::trace;
use eepy_sys::image::{ImageSyscall, RefreshBlockMode};
use eepy_sys::input::{Event, InputSyscall, ScEventType, TouchEvent, TouchEventType};
use eepy_sys::misc::MiscSyscall;
use eepy_sys::syscall::SyscallNumber;
use tp370pgh01::IMAGE_BYTES;
use crate::{DO_REFRESH, EVENT_QUEUE, FAST_REFRESH, IMAGE_BUFFER, REFRESHING, SERIAL_NUMBER, TOUCH_ENABLED};

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
extern "C" fn handle_syscall(sp: *mut usize) {
    // Stack contains R0, R1, R2, R3, R12, LR, ReturnAddress, xPSR
    let stack_values = unsafe { core::slice::from_raw_parts_mut(sp, 8) };
    let syscall_num = unsafe { (stack_values[6] as *const u8).sub(2).read() };

    trace!(
        "syscall: sp={} imm={:x} r0={:x} r1={:x} r2={:x} r3={:x} r12={:x} lr={:x} pc={:x} xpsr={:x}",
        sp,
        syscall_num,
        stack_values[0],
        stack_values[1],
        stack_values[2],
        stack_values[3],
        stack_values[4],
        stack_values[5],
        stack_values[6],
        stack_values[7],
    );

    match SyscallNumber::try_from(syscall_num) {
        Ok(SyscallNumber::Misc) => handle_misc(stack_values),
        Ok(SyscallNumber::Image) => handle_image(stack_values),
        Ok(SyscallNumber::Input) => handle_input(stack_values),
        Ok(SyscallNumber::Usb) => todo!("usb syscalls"),
        _ => panic!("illegal syscall"),
    }
}

fn handle_misc(stack_values: &mut [usize]) {
    match MiscSyscall::try_from(stack_values[0]) {
        Ok(MiscSyscall::GetSerial) => handle_get_serial(stack_values),
        _ => panic!("illegal syscall"),
    }
}

fn handle_get_serial(stack_values: &mut [usize]) {
    stack_values[0] = (&raw const *SERIAL_NUMBER.get().unwrap()) as usize;
}

fn handle_image(stack_values: &mut [usize]) {
    match ImageSyscall::try_from(stack_values[0]) {
        Ok(ImageSyscall::WriteImage) => handle_write_image(stack_values),
        Ok(ImageSyscall::Refresh) => handle_refresh(stack_values),
        _ => panic!("illegal syscall"),
    }
}

fn handle_write_image(stack_values: &mut [usize]) {
    let image: &[u8; IMAGE_BYTES] = unsafe { &*(stack_values[1] as *const [u8; IMAGE_BYTES]) };
    critical_section::with(|cs| IMAGE_BUFFER.borrow_ref_mut(cs).copy_from_slice(image));
}

fn handle_refresh(stack_values: &mut [usize]) {
    let fast_refresh = stack_values[1] != 0;
    let blocking_mode = RefreshBlockMode::try_from(stack_values[2]).expect("illegal refresh blocking mode");

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

fn handle_input(stack_values: &mut [usize]) {
    match InputSyscall::try_from(stack_values[0]) {
        Ok(InputSyscall::NextEvent) => handle_next_event(stack_values),
        Ok(InputSyscall::SetTouchEnabled) => handle_set_touch_enabled(stack_values),
        Ok(InputSyscall::HasEvent) => handle_has_event(stack_values),
        _ => panic!("illegal syscall"),
    }
}

fn handle_next_event(stack_values: &mut [usize]) {
    let next_event = critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).pop());
    let mut x = 0;
    let mut y = 0;
    let sc_ev_type = match next_event {
        None => ScEventType::NoEvent,
        Some(Event::Touch(TouchEvent { ev_type: TouchEventType::Down, x: px, y: py })) => {
            x = px;
            y = py;
            ScEventType::TouchDown
        },
        Some(Event::Touch(TouchEvent { ev_type: TouchEventType::Up, x: px, y: py })) => {
            x = px;
            y = py;
            ScEventType::TouchUp
        },
        Some(Event::Touch(TouchEvent { ev_type: TouchEventType::Move, x: px, y: py })) => {
            x = px;
            y = py;
            ScEventType::TouchMove
        },
        Some(Event::RefreshFinished) => ScEventType::RefreshFinished,
    };
    stack_values[0] = sc_ev_type as usize;
    stack_values[1] = x as usize;
    stack_values[2] = y as usize;
}

fn handle_set_touch_enabled(stack_values: &mut [usize]) {
    let enable = stack_values[1] != 0;
    TOUCH_ENABLED.store(enable, Ordering::Relaxed);
    if !enable {
        critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).clear());
    }
}

fn handle_has_event(stack_values: &mut [usize]) {
    let empty = critical_section::with(|cs| EVENT_QUEUE.borrow_ref(cs).is_empty());
    stack_values[0] = (!empty) as usize;
}