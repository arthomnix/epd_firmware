#![no_main]
#![no_std]

mod ringbuffer;
mod syscall;
mod launcher;
mod usb;
mod exception;
mod tickv;
mod flash;
mod core1;
mod temp;
mod gpio_irq;

extern crate panic_probe;
extern crate defmt_rtt;

use core::arch::asm;
use core::cell::RefCell;
use core::hint::unreachable_unchecked;
use cortex_m::peripheral::NVIC;
use critical_section::Mutex;
use defmt::info;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c;
use mcp9808::MCP9808;
use mcp9808::reg_res::{Resolution, ResolutionVal};
use once_cell::sync::OnceCell;
use portable_atomic::{AtomicBool, AtomicU8};
use fw16_epd_bsp::{entry, hal, pac, EpdBusy, EpdCs, EpdDc, EpdPowerSwitch, EpdReset, EpdSck, EpdSdaWrite, EpdTouchInt, EpdTouchReset, I2CScl, I2CSda, Pins};
use fw16_epd_bsp::hal::{Sio, Timer};
use fw16_epd_bsp::hal::clocks::ClockSource;
use fw16_epd_bsp::hal::fugit::RateExtU32;
use fw16_epd_bsp::hal::gpio::Interrupt::{EdgeHigh, EdgeLow};
use fw16_epd_bsp::hal::multicore::{Multicore, Stack};
use fw16_epd_bsp::hal::timer::{Alarm, Alarm0};
use fw16_epd_bsp::pac::{PPB, PSM};
use fw16_epd_bsp::pac::interrupt;
use eepy_sys::input_common::Event;
use eepy_sys::kv_store;
use eepy_sys::syscall::SyscallNumber;
use fw16_epd_bsp::hal::sio::SioFifo;
use tp370pgh01::rp2040::{Rp2040PervasiveSpiDelays, IoPin};
use tp370pgh01::{Tp370pgh01, IMAGE_BYTES};
use crate::core1::{core1_main, ToCore1Message, GLOBAL_EPD};
use crate::gpio_irq::{GLOBAL_EPD_POWER_PIN, GLOBAL_I2C, GLOBAL_SLEEP_PIN, GLOBAL_TOUCH_INT_PIN, GLOBAL_TOUCH_RESET_PIN};
use crate::ringbuffer::RingBuffer;

const SRAM_END: *mut u8 = 0x20042000 as *mut u8;

static CORE1_STACK: Stack<8192> = Stack::new();

static GLOBAL_ALARM0: Mutex<RefCell<Option<Alarm0>>> = Mutex::new(RefCell::new(None));

static IMAGE_BUFFER: Mutex<RefCell<[u8; IMAGE_BYTES]>> = Mutex::new(RefCell::new([0; IMAGE_BYTES]));
static TEMP: AtomicU8 = AtomicU8::new(20);
static SERIAL_NUMBER: OnceCell<[u8; 16]> = OnceCell::new();

static EVENT_QUEUE: Mutex<RefCell<RingBuffer<Event>>> = Mutex::new(RefCell::new(RingBuffer::new()));
static TOUCH_ENABLED: AtomicBool = AtomicBool::new(false);

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        fw16_epd_bsp::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    ).unwrap();

    core.SCB.set_sleepdeep();

    init_mpu(&mut core);

    unsafe {
        read_serial();
    }

    info!("eepyOS version {} (c) arthomnix 2025", env!("CARGO_PKG_VERSION"));
    info!("Serial number: {}", unsafe { core::str::from_utf8_unchecked(SERIAL_NUMBER.get().unwrap()) });

    let mut sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let cs: EpdCs = pins.spi3_epd_cs.reconfigure();
    let dc: EpdDc = pins.epd_dc.reconfigure();
    let busy: EpdBusy = pins.epd_busy.reconfigure();
    let rst: EpdReset = pins.epd_rst.reconfigure();
    let sda: EpdSdaWrite = pins.spi3_epd_sda.reconfigure();
    let sck: EpdSck = pins.spi3_epd_sck.reconfigure();
    let i2c_sda: I2CSda = pins.i2c_sda.reconfigure();
    let i2c_scl: I2CScl = pins.i2c_scl.reconfigure();
    let int: EpdTouchInt = pins.epd_touch_int.reconfigure();

    int.set_interrupt_enabled(EdgeLow, true);

    let mut i2c = hal::i2c::I2C::i2c0(pac.I2C0, i2c_sda, i2c_scl, 400.kHz(), &mut pac.RESETS, clocks.system_clock.get_freq());
    init_temp_sensor(&mut i2c);

    let mut epd_power: EpdPowerSwitch = pins.epd_pwr_sw.reconfigure();
    epd_power.set_low().unwrap();
    critical_section::with(|cs| GLOBAL_EPD_POWER_PIN.borrow_ref_mut(cs).replace(epd_power));

    let mut sleep = pins.laptop_sleep.into_pull_up_input();
    sleep.set_interrupt_enabled(EdgeLow, true);
    sleep.set_interrupt_enabled(EdgeHigh, true);
    sleep.clear_interrupt(EdgeLow);
    sleep.clear_interrupt(EdgeHigh);
    critical_section::with(|cs| GLOBAL_SLEEP_PIN.borrow_ref_mut(cs).replace(sleep));

    let mut touch_reset: EpdTouchReset = pins.epd_touch_rst.reconfigure();
    reset_touch(&mut touch_reset);
    critical_section::with(|cs| GLOBAL_TOUCH_RESET_PIN.borrow_ref_mut(cs).replace(touch_reset));

    critical_section::with(|cs| {
        GLOBAL_TOUCH_INT_PIN.borrow_ref_mut(cs).replace(int);
        GLOBAL_I2C.borrow_ref_mut(cs).replace(i2c);
    });

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    // make sure temperature sensor has read temperature
    timer.delay_ms(35);
    let mut alarm = timer.alarm_0().unwrap();
    alarm.enable_interrupt();
    critical_section::with(|cs| GLOBAL_ALARM0.borrow_ref_mut(cs).replace(alarm));

    usb::init_usb(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, clocks.usb_clock);

    let epd = Tp370pgh01::new(cs, IoPin::new(sda), sck, dc, busy, rst, timer, Rp2040PervasiveSpiDelays);
    critical_section::with(|cs| GLOBAL_EPD.borrow_ref_mut(cs).replace(epd));

    init_core1(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);

    configure_interrupts(&mut core.NVIC);

    let mut buf = [0u8; 1];
    // NOTE: since the kernel sits within the first 512K of flash, it counts as the same program
    // as the launcher for KV purposes, so we can read the autostart value set by the launcher
    let slot = if kv_store::get(b"autostart", &mut buf).is_ok() {
        buf[0]
    } else {
        0
    };

    enter_userspace(slot);
}

fn init_mpu(core: &mut pac::CorePeripherals) {
    // Set MPU regions for non-privileged code
    unsafe {
        // Region 0: Unprivileged RAM region (first 128K) - start 0x20020000, length 128K
        core.MPU.rnr.write(0);
        core.MPU.rbar.write(0x20020000);
        core.MPU.rasr.write(
            (0b1 << 28) // XN
                | (0b011 << 24) // AP: full access
                | (0b000 << 19) // TEX
                | (0b011 << 16) // S, C, B: non-shareable, writeback
                | (0b00000000 << 8) // all subregions enabled
                | (16 << 1) // size: 2^(16 + 1) = 128K
                | (0b1 << 0) // enable
        );

        // Region 1: Unprivileged RAM region (final 8K) - start 0x20040000, length 8K
        core.MPU.rnr.write(1);
        core.MPU.rbar.write(0x20040000);
        core.MPU.rasr.write(
            (0b1 << 28) // XN
                | (0b011 << 24) // AP: full access
                | (0b000 << 19) // TEX
                | (0b011 << 16) // S, C, B: non-shareable, writeback
                | (0b00000000 << 8) // all subregions enabled
                | (12 << 1) // size: 2^(12 + 1) = 8K
                | (0b1 << 0) // enable
        );

        // Region 3: Flash - start: 0x10000000, length 16M
        core.MPU.rnr.write(3);
        core.MPU.rbar.write(0x10000000);
        // NOTE: we of course can't actually write to the flash via the XIP memory map, but
        // performing writes to the flash memory region flushes the XIP cache for that area of
        // flash, so the kernel still needs write access
        core.MPU.rasr.write(
            (0b0 << 28) // disable XN
                | (0b010 << 24) // AP: privileged RW, unprivileged RO
                | (0b000 << 19) // TEX
                | (0b110 << 16) // S, C, B: shared, write-through
                | (0b00000000 << 8) // all subregions enabled
                | (23 << 1) // size: 2^(23 + 1) = 16M
                | (0b1 << 0) // enable
        );

        // Enable MPU
        core.MPU.ctrl.write(
            (0b1 << 2) // PRIVDEFENA: enable default memory map for privileged access
                | (0b0 << 1) // HFNMIENA: disable MPU for HardFault and NMI
                | (0b1 << 0) // ENABLE: enable MPU
        );
    }
}

/// SAFETY: must be run when core1 is in a safe state (not started yet or running code from RAM
/// with interrupts disabled)
unsafe fn read_serial() {
    // Read flash unique ID
    cortex_m::interrupt::free(|_cs| {
        let mut id = [0u8; 8];
        // SAFETY: interrupts are disabled, so this is safe as long as core1 is in a safe state
        unsafe { rp2040_flash::flash::flash_unique_id(&mut id, true) };
        let mut id = u64::from_be_bytes(id);
        let mut serial = [0u8; 16];
        for c in serial.iter_mut().rev() {
            let nibble = (id & 0x0f) as u8;
            *c = match nibble {
                0x0..=0x9 => b'0' + nibble,
                0xa..=0xf => b'A' + nibble - 0xa,
                _ => unreachable!(),
            };
            id >>= 4;
        }
        SERIAL_NUMBER.set(serial).unwrap();
    });
}

fn init_temp_sensor(i2c: &mut impl I2c) {
    let mut mcp9808 = MCP9808::new(i2c);
    let mut res = mcp9808::reg_res::new();
    res.set_resolution(ResolutionVal::Deg_0_5C);
    mcp9808.write_register(res).unwrap();
}

fn init_core1(psm: &mut PSM, ppb: &mut PPB, fifo: &mut SioFifo) {
    let mut mc = Multicore::new(psm, ppb, fifo);
    let core1 = &mut mc.cores()[1];
    core1.spawn(CORE1_STACK.take().unwrap(), core1_main).unwrap();
    fifo.write_blocking(ToCore1Message::HardResetEpd as u32);
}

fn configure_interrupts(nvic: &mut NVIC) {
    unsafe {
        nvic.set_priority(interrupt::TIMER_IRQ_0, 0b10000000);
        nvic.set_priority(interrupt::USBCTRL_IRQ, 0b11000000);
        nvic.set_priority(interrupt::IO_IRQ_BANK0, 0);
        nvic.set_priority(interrupt::SW0_IRQ, 0b01000000);

        NVIC::unmask(interrupt::TIMER_IRQ_0);
        NVIC::unmask(interrupt::IO_IRQ_BANK0);
        NVIC::unmask(interrupt::SW0_IRQ);
    }

    NVIC::pend(interrupt::TIMER_IRQ_0);
}

/// Switch to the PSP and start the specified program.
fn enter_userspace(slot: u8) -> ! {
    unsafe {
        asm!(
            "msr psp, {sram_end}",
            "msr control, {control_0b10}",
            "isb",
            "svc #{exec}",
            sram_end = in(reg) SRAM_END,
            control_0b10 = in(reg) 0b10,
            in("r0") slot,
            exec = const SyscallNumber::Exec as u8,
        );

        // SAFETY: the exec syscall never returns
        unreachable_unchecked();
    }
}

fn reset_touch(pin: &mut EpdTouchReset) {
    pin.set_high().unwrap();
    cortex_m::asm::delay(1000000);
    pin.set_low().unwrap();
    cortex_m::asm::delay(1000000);
    pin.set_high().unwrap();
    cortex_m::asm::delay(10000000);
}