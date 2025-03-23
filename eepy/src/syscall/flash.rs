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