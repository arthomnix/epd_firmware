use core::mem::MaybeUninit;
use usb_device::bus::PollResult;
use usb_device::class_prelude::{EndpointAddress, EndpointType};
use usb_device::{UsbDirection, UsbError};
use usb_device::endpoint::{IsochronousSynchronizationType, IsochronousUsageType};
use crate::{syscall, SafeOption, SafeResult};
use crate::syscall::SyscallNumber;

#[repr(usize)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, strum::FromRepr)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UsbSyscall {
    UsbRet = 0,
    SetHandler = 1,
    ClearHandler = 2,
    Init = 3,
    Uninit = 4,
    AllocEp = 5,
    Enable = 6,
    Reset = 7,
    SetDeviceAddr = 8,
    Write = 9,
    Read = 10,
    SetStalled = 11,
    IsStalled = 12,
    Poll = 13,
}

pub fn set_handler(f: extern "C" fn()) {
    unsafe {
        syscall!(
            SyscallNumber::Usb,
            in UsbSyscall::SetHandler,
            in f,
        );
    }
}

pub fn clear_handler() {
    unsafe {
        syscall!(
            SyscallNumber::Usb,
            in UsbSyscall::ClearHandler,
        )
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafePollResult {
    None,
    Reset,
    Data {
        ep_out: u16,
        ep_in_complete: u16,
        ep_setup: u16,
    },
    Suspend,
    Resume,
}

impl From<SafePollResult> for PollResult {
    fn from(value: SafePollResult) -> Self {
        match value {
            SafePollResult::None => PollResult::None,
            SafePollResult::Reset => PollResult::Reset,
            SafePollResult::Data { ep_out, ep_in_complete, ep_setup } => PollResult::Data { ep_out, ep_in_complete, ep_setup },
            SafePollResult::Suspend => PollResult::Suspend,
            SafePollResult::Resume => PollResult::Resume
        }
    }
}

impl From<PollResult> for SafePollResult {
    fn from(value: PollResult) -> Self {
        match value {
            PollResult::None => SafePollResult::None,
            PollResult::Reset => SafePollResult::Reset,
            PollResult::Data { ep_out, ep_in_complete, ep_setup } => SafePollResult::Data { ep_out, ep_in_complete, ep_setup },
            PollResult::Suspend => SafePollResult::Suspend,
            PollResult::Resume => SafePollResult::Resume,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafeUsbError {
    WouldBlock,
    ParseError,
    BufferOverflow,
    EndpointOverflow,
    EndpointMemoryOverflow,
    InvalidEndpoint,
    Unsupported,
    InvalidState,
}

impl From<SafeUsbError> for UsbError {
    fn from(value: SafeUsbError) -> Self {
        match value {
            SafeUsbError::WouldBlock => UsbError::WouldBlock,
            SafeUsbError::ParseError => UsbError::ParseError,
            SafeUsbError::BufferOverflow => UsbError::BufferOverflow,
            SafeUsbError::EndpointOverflow => UsbError::EndpointOverflow,
            SafeUsbError::EndpointMemoryOverflow => UsbError::EndpointMemoryOverflow,
            SafeUsbError::InvalidEndpoint => UsbError::InvalidEndpoint,
            SafeUsbError::Unsupported => UsbError::Unsupported,
            SafeUsbError::InvalidState => UsbError::InvalidState,
        }
    }
}

impl From<UsbError> for SafeUsbError {
    fn from(value: UsbError) -> Self {
        match value {
            UsbError::WouldBlock => SafeUsbError::WouldBlock,
            UsbError::ParseError => SafeUsbError::ParseError,
            UsbError::BufferOverflow => SafeUsbError::BufferOverflow,
            UsbError::EndpointOverflow => SafeUsbError::EndpointOverflow,
            UsbError::EndpointMemoryOverflow => SafeUsbError::EndpointMemoryOverflow,
            UsbError::InvalidEndpoint => SafeUsbError::InvalidEndpoint,
            UsbError::Unsupported => SafeUsbError::Unsupported,
            UsbError::InvalidState => SafeUsbError::InvalidState,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafeIsochronousSynchronizationType {
    NoSynchronization,
    Asynchronous,
    Adaptive,
    Synchronous,
}

impl From<IsochronousSynchronizationType> for SafeIsochronousSynchronizationType {
    fn from(value: IsochronousSynchronizationType) -> Self {
        match value {
            IsochronousSynchronizationType::NoSynchronization => SafeIsochronousSynchronizationType::NoSynchronization,
            IsochronousSynchronizationType::Asynchronous => SafeIsochronousSynchronizationType::Asynchronous,
            IsochronousSynchronizationType::Adaptive => SafeIsochronousSynchronizationType::Adaptive,
            IsochronousSynchronizationType::Synchronous => SafeIsochronousSynchronizationType::Synchronous,
        }
    }
}

impl From<SafeIsochronousSynchronizationType> for IsochronousSynchronizationType {
    fn from(value: SafeIsochronousSynchronizationType) -> Self {
        match value {
            SafeIsochronousSynchronizationType::NoSynchronization => IsochronousSynchronizationType::NoSynchronization,
            SafeIsochronousSynchronizationType::Asynchronous => IsochronousSynchronizationType::Asynchronous,
            SafeIsochronousSynchronizationType::Adaptive => IsochronousSynchronizationType::Adaptive,
            SafeIsochronousSynchronizationType::Synchronous => IsochronousSynchronizationType::Synchronous,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafeIsochronousUsageType {
    Data,
    Feedback,
    ImplicitFeedbackData,
}

impl From<IsochronousUsageType> for SafeIsochronousUsageType {
    fn from(value: IsochronousUsageType) -> Self {
        match value {
            IsochronousUsageType::Data => SafeIsochronousUsageType::Data,
            IsochronousUsageType::Feedback => SafeIsochronousUsageType::Feedback,
            IsochronousUsageType::ImplicitFeedbackData => SafeIsochronousUsageType::ImplicitFeedbackData,
        }
    }
}

impl From<SafeIsochronousUsageType> for IsochronousUsageType {
    fn from(value: SafeIsochronousUsageType) -> Self {
        match value {
            SafeIsochronousUsageType::Data => IsochronousUsageType::Data,
            SafeIsochronousUsageType::Feedback => IsochronousUsageType::Feedback,
            SafeIsochronousUsageType::ImplicitFeedbackData => IsochronousUsageType::ImplicitFeedbackData,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SafeEndpointType {
    Control,
    Isochronous {
        synchronization: SafeIsochronousSynchronizationType,
        usage: SafeIsochronousUsageType,
    },
    Bulk,
    Interrupt,
}

impl From<EndpointType> for SafeEndpointType {
    fn from(value: EndpointType) -> Self {
        match value {
            EndpointType::Control => SafeEndpointType::Control,
            EndpointType::Isochronous { synchronization, usage } => {
                SafeEndpointType::Isochronous { synchronization: synchronization.into(), usage: usage.into() }
            },
            EndpointType::Bulk => SafeEndpointType::Bulk,
            EndpointType::Interrupt => SafeEndpointType::Interrupt,
        }
    }
}

impl From<SafeEndpointType> for EndpointType {
    fn from(value: SafeEndpointType) -> Self {
        match value {
            SafeEndpointType::Control => EndpointType::Control,
            SafeEndpointType::Isochronous { synchronization, usage } => {
                EndpointType::Isochronous { synchronization: synchronization.into(), usage: usage.into() }
            },
            SafeEndpointType::Bulk => EndpointType::Bulk,
            SafeEndpointType::Interrupt => EndpointType::Interrupt,
        }
    }
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsbEpAllocArgs {
    pub ep_dir: UsbDirection,
    pub ep_addr: SafeOption<u8>,
    pub ep_type: SafeEndpointType,
    pub max_packet_size: u16,
    pub interval: u8,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsbWriteArgs {
    pub ep_addr: u8,
    pub len: usize,
    pub buf: *const u8,
    pub ptr: *mut SafeResult<usize, SafeUsbError>,
}

#[repr(C)]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsbReadArgs {
    pub ep_addr: u8,
    pub len: usize,
    pub buf: *mut u8,
    pub ptr: *mut SafeResult<usize, SafeUsbError>,
}

pub struct UsbBus {
    _dummy: (),
}

impl UsbBus {
    pub fn init() -> Self {
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Init,
            );
        }

        Self { _dummy: () }
    }
}

impl Drop for UsbBus {
    fn drop(&mut self) {
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Uninit,
            )
        }
    }
}

impl usb_device::bus::UsbBus for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        interval: u8
    ) -> usb_device::Result<EndpointAddress> {
        let mut res: MaybeUninit<SafeResult<u8, SafeUsbError>> = MaybeUninit::uninit();
        let res_ptr = res.as_mut_ptr();

        let alloc_info = UsbEpAllocArgs {
            ep_dir,
            ep_addr: ep_addr.map(|v| v.into()).into(),
            ep_type: ep_type.into(),
            max_packet_size,
            interval
        };

        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::AllocEp,
                in &raw const alloc_info,
                in res_ptr,
            );

            let res: Result<u8, SafeUsbError> = res.assume_init().into();
            res.map(|v| v.into()).map_err(|e| e.into())
        }
    }

    fn enable(&mut self) {
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Enable,
            );
        }
    }

    fn reset(&self) {
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Reset,
            );
        }
    }

    fn set_device_address(&self, addr: u8) {
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::SetDeviceAddr,
                in addr,
            );
        }
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        let ep_addr: u8 = ep_addr.into();
        let mut res: MaybeUninit<SafeResult<usize, SafeUsbError>> = MaybeUninit::uninit();
        let ptr = res.as_mut_ptr();
        let len = buf.len();
        let buf = buf.as_ptr();

        let args = UsbWriteArgs {
            ep_addr,
            len,
            buf,
            ptr,
        };

        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Write,
                in &raw const args,
            );

            let stdres: Result<usize, SafeUsbError> = res.assume_init().into();
            stdres.map_err(|e| e.into())
        }
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        let ep_addr: u8 = ep_addr.into();
        let mut res: MaybeUninit<SafeResult<usize, SafeUsbError>> = MaybeUninit::uninit();
        let ptr = res.as_mut_ptr();
        let len = buf.len();
        let buf = buf.as_mut_ptr();

        let args = UsbReadArgs {
            ep_addr,
            len,
            buf,
            ptr,
        };

        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Read,
                in &raw const args,
            );

            let stdres: Result<usize, SafeUsbError> = res.assume_init().into();
            stdres.map_err(|e| e.into())
        }
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let ep_addr: u8 = ep_addr.into();
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::SetStalled,
                in ep_addr,
                in stalled,
            )
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let ep_addr: u8 = ep_addr.into();

        let mut stalled: usize;
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                out stalled in UsbSyscall::IsStalled,
                in ep_addr,
            );
        }

        stalled != 0
    }

    fn suspend(&self) {
        // not implemented in rp2040 usb-device implementation
    }

    fn resume(&self) {
        // not implemented in rp2040 usb-device implementation
    }

    fn poll(&self) -> PollResult {
        let mut spr: MaybeUninit<SafePollResult> = MaybeUninit::uninit();
        let ptr = spr.as_mut_ptr();
        unsafe {
            syscall!(
                SyscallNumber::Usb,
                in UsbSyscall::Poll,
                in ptr,
            );

            spr.assume_init().into()
        }
    }
}