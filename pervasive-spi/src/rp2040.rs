use crate::{WithInput, WithOutput};
use rp2040_hal::gpio::{
    Function, FunctionSioInput, FunctionSioOutput, Pin, PinId, PullType, ValidFunction,
};

/// A GPIO pin that can be configured as an input or output.
pub struct IoPin<I, F, P>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput> + ValidFunction<F>,
    F: Function,
    P: PullType,
{
    pin: Option<Pin<I, F, P>>,
}

impl<I, F, P> IoPin<I, F, P>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput> + ValidFunction<F>,
    F: Function,
    P: PullType,
{
    /// Create a new instance of `IoPin` with a GPIO pin.
    pub fn new(pin: Pin<I, F, P>) -> Self {
        Self { pin: Some(pin) }
    }
}

impl<I, F, P> WithInput for IoPin<I, F, P>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput> + ValidFunction<F>,
    F: Function,
    P: PullType,
{
    type Input = Pin<I, FunctionSioInput, P>;

    fn with_input<R>(&mut self, f: impl Fn(&mut Self::Input) -> R) -> R {
        let mut input = self.pin.take().unwrap().reconfigure();
        let res = f(&mut input);
        self.pin.replace(input.reconfigure());
        res
    }
}

impl<I, F, P> WithOutput for IoPin<I, F, P>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<FunctionSioOutput> + ValidFunction<F>,
    F: Function,
    P: PullType,
{
    type Output = Pin<I, FunctionSioOutput, P>;

    fn with_output<R>(&mut self, f: impl Fn(&mut Self::Output) -> R) -> R {
        let mut output = self.pin.take().unwrap().reconfigure();
        let res = f(&mut output);
        self.pin.replace(output.reconfigure());
        res
    }
}