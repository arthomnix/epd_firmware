use core::arch::{asm, global_asm};
use core::cell::RefCell;
use core::fmt::{Debug, Formatter};
use core::sync::atomic::Ordering;
use critical_section::Mutex;
use defmt::{trace, warn};
use portable_atomic::AtomicPtr;
use eepy_sys::usb::{SafePollResult, SafeUsbError, UsbEpAllocArgs, UsbReadArgs, UsbSyscall, UsbWriteArgs};
use fw16_epd_bsp::hal::clocks::UsbClock;
use fw16_epd_bsp::hal::usb::UsbBus;
use fw16_epd_bsp::pac;
use fw16_epd_bsp::pac::{interrupt, USBCTRL_DPRAM, USBCTRL_REGS};
use usb_device::class_prelude::{*, UsbBus as _};
use eepy_sys::SafeResult;
use crate::exception::StackFrame;

global_asm!(include_str!("usb.s"));

struct Usb {
    bus: Option<UsbBus>,
    freed: Option<(USBCTRL_REGS, USBCTRL_DPRAM, UsbClock)>,
}

impl Usb {
    fn init_bus(&mut self) {
        if let Some((regs, dpram, clock)) = self.freed.take() {
            self.bus.replace(UsbBus::new(
                regs,
                dpram,
                clock,
                true,
                &mut unsafe { pac::RESETS::steal() },
            ));
        }
    }

    fn free_bus(&mut self) {
        if let Some(bus) = self.bus.take() {
            let (regs, dpram, clock) = bus.free(
                &mut unsafe { pac::RESETS::steal() }
            );
            self.freed.replace((regs, dpram, clock));
        }
    }

    fn get_bus(&mut self) -> &mut UsbBus {
        self.bus.as_mut().expect("USB bus not initialised")
    }
}

impl Debug for Usb {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        if self.bus.is_some() {
            f.write_str("Active USB")
        } else if self.freed.is_some() {
            f.write_str("Freed USB")
        } else {
            f.write_str("Uninitialised USB")
        }
    }
}

static USB: Mutex<RefCell<Option<Usb>>> = Mutex::new(RefCell::new(None));

fn with_usb<R>(mut f: impl FnMut(&mut Usb) -> R) -> R {
    critical_section::with(|cs| {
        f(
            USB
                .borrow_ref_mut(cs)
                .as_mut()
                .expect("USB bus not initialised")
        )
    })
}

fn with_usb_bus<R>(mut f: impl FnMut(&mut UsbBus) -> R) -> R {
    critical_section::with(|cs| {
        f(
            USB
                .borrow_ref_mut(cs)
                .as_mut()
                .expect("USB bus not initialised")
                .get_bus()
        )
    })
}

pub(crate) fn init_usb(regs: USBCTRL_REGS, dpram: USBCTRL_DPRAM, clock: UsbClock) {
    critical_section::with(|cs| USB.borrow_ref_mut(cs).replace(
        Usb {
            bus: None,
            freed: Some((regs, dpram, clock)),
        }
    ));
}

static USB_HANDLER: AtomicPtr<u8> = AtomicPtr::new(core::ptr::null_mut());

extern "C" {
    fn usb_ret();
}

#[no_mangle]
extern "C" fn handle_usb_irq(sp: *mut StackFrame, using_psp: bool) {
    trace!("USBCTRL_IRQ");

    if !using_psp {
        return;
    }

    let handler = USB_HANDLER.load(Ordering::Relaxed);
    if handler.is_null() {
        warn!("USB handler is null but interrupt was not masked");
        return;
    }

    // make sure the interrupt doesn't fire again whilst the unprivileged handler
    // is running (we unmask again after the handler is done as part of the UsbRet
    // syscall
    trace!("masking USBCTRL_IRQ");
    pac::NVIC::mask(interrupt::USBCTRL_IRQ);

    unsafe {
        let new_sp = sp.sub(1);
        new_sp.write(StackFrame {
            r0: 0,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: usb_ret as *const u8,
            pc: handler,
            xpsr: 0x1000000,
        });

        asm!(
            "msr psp, {new_psp}",
            new_psp = in(reg) new_sp,
        );
    }
}

pub(crate) fn handle_usb(stack_values: &mut StackFrame) {
    match UsbSyscall::try_from(stack_values.r0) {
        Ok(UsbSyscall::UsbRet) => handle_usb_ret(stack_values),
        Ok(UsbSyscall::SetHandler) => handle_set_handler(stack_values),
        Ok(UsbSyscall::ClearHandler) => handle_clear_handler(),
        Ok(UsbSyscall::Init) => handle_init(),
        Ok(UsbSyscall::Uninit) => handle_uninit(),
        Ok(UsbSyscall::AllocEp) => handle_alloc_ep(stack_values),
        Ok(UsbSyscall::Enable) => handle_enable(),
        Ok(UsbSyscall::Reset) => handle_reset(),
        Ok(UsbSyscall::SetDeviceAddr) => handle_set_device_addr(stack_values),
        Ok(UsbSyscall::Write) => handle_write(stack_values),
        Ok(UsbSyscall::Read) => handle_read(stack_values),
        Ok(UsbSyscall::SetStalled) => handle_set_stalled(stack_values),
        Ok(UsbSyscall::IsStalled) => handle_is_stalled(stack_values),
        Ok(UsbSyscall::Poll) => handle_poll(stack_values),
        Err(_) => panic!("illegal syscall"),
    }
}

pub(crate) fn handle_clear_handler() {
    USB_HANDLER.store(core::ptr::null_mut(), Ordering::Relaxed);
    trace!("masking USBCTRL_IRQ");
    pac::NVIC::mask(interrupt::USBCTRL_IRQ);
}

fn handle_set_handler(stack_values: &mut StackFrame) {
    let handler = stack_values.r1 as *mut u8;
    USB_HANDLER.store(handler, Ordering::Relaxed);
    unsafe {
        trace!("unmasking USBCTRL_IRQ");
        pac::NVIC::unmask(interrupt::USBCTRL_IRQ);
    }
}

fn handle_usb_ret(stack_values: &mut StackFrame) {
    trace!("usb_ret");
    unsafe {
        trace!("unmasking USBCTRL_IRQ");
        pac::NVIC::unmask(interrupt::USBCTRL_IRQ);

        let new_sp = (&raw mut *stack_values).add(1);
        asm!(
            "msr psp, {new_psp}",
            new_psp = in(reg) new_sp,
        );
    }
}

fn handle_init() {
    trace!("USB init");
    with_usb(|usb| usb.init_bus());
}

pub(crate) fn handle_uninit() {
    trace!("USB uninit");
    with_usb(|usb| usb.free_bus());
}

fn handle_alloc_ep(stack_values: &mut StackFrame) {
    trace!("USB alloc endpoint");
    let alloc_info = stack_values.r1 as *const UsbEpAllocArgs;
    trace!("{}", unsafe { *alloc_info });

    let ep_dir = unsafe { (*alloc_info).ep_dir };
    let ep_addr: Option<EndpointAddress> = unsafe {
        let addr: Option<u8> = (*alloc_info).ep_addr.into();
        addr.map(|v| v.into())
    };
    let ep_type: EndpointType = unsafe { (*alloc_info).ep_type.into() };
    let max_packet_size = unsafe { (*alloc_info).max_packet_size };
    let interval = unsafe { (*alloc_info).interval };

    let res_ptr = stack_values.r2 as *mut SafeResult<u8, SafeUsbError>;

    let res = with_usb_bus(|bus| {
        bus.alloc_ep(
            ep_dir,
            ep_addr,
            ep_type,
            max_packet_size,
            interval,
        )
    });

    trace!("{}", res);

    let res: Result<u8, SafeUsbError> = res.map(|v| v.into()).map_err(|e| e.into());
    unsafe {
        res_ptr.write(res.into());
    }
}

fn handle_enable() {
    trace!("USB enable");
    with_usb_bus(|bus| bus.enable());
}

fn handle_reset() {
    trace!("USB reset");
    with_usb_bus(|bus| bus.reset());
}

fn handle_set_device_addr(stack_values: &mut StackFrame) {
    trace!("USB set device address");
    with_usb_bus(|bus| bus.set_device_address(stack_values.r1 as u8));
}

fn handle_write(stack_values: &mut StackFrame) {
    trace!("USB write");
    let args = stack_values.r1 as *const UsbWriteArgs;

    let ep_addr = unsafe { (*args).ep_addr }.into();
    let buf = unsafe { core::slice::from_raw_parts((*args).buf, (*args).len) };

    let res = with_usb_bus(|bus| {
        bus.write(ep_addr, buf)
    });

    let res: Result<usize, SafeUsbError> = res.map_err(|e| e.into());
    unsafe {
        (*args).ptr.write(res.into());
    }
}

fn handle_read(stack_values: &mut StackFrame) {
    trace!("USB read");
    let args = stack_values.r1 as *const UsbReadArgs;

    let ep_addr = unsafe { (*args).ep_addr }.into();
    let buf = unsafe { core::slice::from_raw_parts_mut((*args).buf, (*args).len) };

    let res = with_usb_bus(|bus| {
        bus.read(ep_addr, buf)
    });

    let res: Result<usize, SafeUsbError> = res.map_err(|e| e.into());
    unsafe {
        (*args).ptr.write(res.into());
    }
}

fn handle_set_stalled(stack_values: &mut StackFrame) {
    trace!("USB set stalled");
    let ep_addr: EndpointAddress = (stack_values.r1 as u8).into();
    let stalled = stack_values.r2 != 0;
    with_usb_bus(|bus| bus.set_stalled(ep_addr, stalled));
}

fn handle_is_stalled(stack_values: &mut StackFrame) {
    trace!("USB query stalled");
    let ep_addr: EndpointAddress = (stack_values.r1 as u8).into();
    let stalled = with_usb_bus(|bus| bus.is_stalled(ep_addr));
    stack_values.r0 = stalled as usize;
}

fn handle_poll(stack_values: &mut StackFrame) {
    trace!("USB poll");
    let res_ptr = stack_values.r1 as *mut SafePollResult;
    let result = with_usb_bus(|bus| bus.poll());
    unsafe {
        res_ptr.write(result.into());
    }
}