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