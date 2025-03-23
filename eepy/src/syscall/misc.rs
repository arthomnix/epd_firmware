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