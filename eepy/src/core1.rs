use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use critical_section::Mutex;
use defmt::{trace, warn};
use eepy_sys::input_common::Event;
use fw16_epd_bsp::hal::gpio::bank0::Gpio11;
use fw16_epd_bsp::hal::gpio::{FunctionSioOutput, PullNone};
use fw16_epd_bsp::hal::{Sio, Timer};
use fw16_epd_bsp::{pac, EpdBusy, EpdCs, EpdDc, EpdReset, EpdSck};
use fw16_epd_bsp::hal::sio::SioFifo;
use fw16_epd_bsp::pac::interrupt;
use tp370pgh01::{Tp370pgh01, IMAGE_BYTES};
use tp370pgh01::rp2040::{IoPin, Rp2040PervasiveSpiDelays};
use crate::{EVENT_QUEUE, IMAGE_BUFFER, TEMP};

type SdaIo = IoPin<Gpio11, FunctionSioOutput, PullNone>;
type Epd = Tp370pgh01<EpdCs, SdaIo, EpdSck, EpdDc, EpdBusy, EpdReset, Timer, Rp2040PervasiveSpiDelays>;

pub(crate) static GLOBAL_EPD: Mutex<RefCell<Option<Epd>>> = Mutex::new(RefCell::new(None));

pub(crate) static IDLE: AtomicBool = AtomicBool::new(true);

/// Function in RAM to be executed by core1 whilst flashing programs (core1 cannot be executing code
/// from flash whilst writing/erasing flash)
#[unsafe(link_section = ".data.ram_func")]
fn core1_flash_wait(fifo: &mut SioFifo) {
    cortex_m::interrupt::free(|_cs| {
        fifo.write_blocking(ToCore0Message::FlashAck as u32);
        trace!("core1 waiting for flash operation");

        while fifo.read_blocking() != ToCore1Message::FlashFinished as u32 {
            warn!("Received message other than FlashFinished while in flashing state");
        }
    });
}

pub(crate) fn core1_main() {
    unsafe {
        pac::NVIC::unmask(interrupt::SIO_IRQ_PROC1);
    }

    loop {
        cortex_m::asm::wfi();
    }
}

#[repr(u32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format, strum::FromRepr)]
pub(crate) enum ToCore1Message {
    FlashWait,
    FlashFinished,
    HardResetEpd,
    RefreshNormal,
    MaybeRefreshNormal,
    RefreshFast,
    MaybeRefreshFast,
}

#[repr(u32)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, defmt::Format, strum::FromRepr)]
pub(crate) enum ToCore0Message {
    FlashAck,
    RefreshAck,
}

fn refresh(fast: bool, image: &mut [u8; IMAGE_BYTES], prev_image: &mut [u8; IMAGE_BYTES], epd: &mut Epd) {
    if fast {
        prev_image.copy_from_slice(image);
        critical_section::with(|cs| image.copy_from_slice(IMAGE_BUFFER.borrow_ref(cs).as_ref()));
        epd.soft_reset().unwrap();
        epd.refresh_fast(image, prev_image, TEMP.load(Ordering::Relaxed)).unwrap();
    } else {
        critical_section::with(|cs| image.copy_from_slice(IMAGE_BUFFER.borrow_ref(cs).as_ref()));
        epd.soft_reset().unwrap();
        epd.refresh(image, TEMP.load(Ordering::Relaxed)).unwrap();
    }

    critical_section::with(|cs| EVENT_QUEUE.borrow_ref_mut(cs).push(Event::RefreshFinished));
    cortex_m::asm::sev();
}

#[interrupt]
fn SIO_IRQ_PROC1() {
    static mut EPD: Option<Epd> = None;
    static mut IMAGE: [u8; IMAGE_BYTES] = [0u8; IMAGE_BYTES];
    static mut PREV_IMAGE: [u8; IMAGE_BYTES] = [0u8; IMAGE_BYTES];

    trace!("SIO_IRQ_PROC1");

    IDLE.store(false, Ordering::Relaxed);

    if EPD.is_none() {
        critical_section::with(|cs| *EPD = GLOBAL_EPD.borrow(cs).take());
    }
    let epd = EPD.as_mut().unwrap();

    let sio = unsafe { pac::Peripherals::steal() }.SIO;
    unsafe { sio.fifo_st().write_with_zero(|x| x) };
    let mut fifo = Sio::new(sio).fifo;

    while let Some(val) = fifo.read() {
        let message = ToCore1Message::from_repr(val);
        match message {
            Some(ToCore1Message::FlashWait) => core1_flash_wait(&mut fifo),
            Some(ToCore1Message::FlashFinished) => warn!("Received FlashFinished but wasn't in flashing state"),
            Some(ToCore1Message::HardResetEpd) => epd.hard_reset().unwrap(),
            Some(ToCore1Message::RefreshNormal) => {
                fifo.write_blocking(ToCore0Message::RefreshAck as u32);
                refresh(false, IMAGE, PREV_IMAGE, epd);
            },
            Some(ToCore1Message::MaybeRefreshNormal) => {
                if !fifo.is_read_ready() {
                    refresh(false, IMAGE, PREV_IMAGE, epd);
                }
            },
            Some(ToCore1Message::RefreshFast) => {
                fifo.write_blocking(ToCore0Message::RefreshAck as u32);
                refresh(true, IMAGE, PREV_IMAGE, epd);
            },
            Some(ToCore1Message::MaybeRefreshFast) => {
                if !fifo.is_read_ready() {
                    refresh(true, IMAGE, PREV_IMAGE, epd);
                }
            },
            None => warn!("core1 received unknown FIFO message {}", val),
        }
    }

    IDLE.store(true, Ordering::Relaxed);
}