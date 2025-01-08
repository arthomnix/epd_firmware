use crate::{WithInput, WithOutput};
use rp2040_hal::gpio::{
    Function, FunctionSioInput, FunctionSioOutput, Pin, PinId, PullType, ValidFunction,
};

impl<I, F, P> WithInput for Option<Pin<I, F, P>>
where
    I: PinId + ValidFunction<FunctionSioInput> + ValidFunction<F>,
    F: Function,
    P: PullType,
{
    type Input = Pin<I, FunctionSioInput, P>;

    fn with_input<R>(&mut self, f: impl Fn(&mut Self::Input) -> R) -> R {
        let mut input = self.take().unwrap().reconfigure();
        let res = f(&mut input);
        self.replace(input.reconfigure());
        res
    }
}

impl<I, F, P> WithOutput for Option<Pin<I, F, P>>
where
    I: PinId + ValidFunction<FunctionSioOutput> + ValidFunction<F>,
    F: Function,
    P: PullType,
{
    type Output = Pin<I, FunctionSioOutput, P>;

    fn with_output<R>(&mut self, f: impl Fn(&mut Self::Output) -> R) -> R {
        let mut output = self.take().unwrap().reconfigure();
        let res = f(&mut output);
        self.replace(output.reconfigure());
        res
    }
}
