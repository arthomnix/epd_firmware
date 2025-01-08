//! Bitbanging driver for the SPI-derived protocol used by Pervasive Displays e-paper screens.

#![no_std]

#[cfg(feature = "rp2040")]
pub mod rp2040;

use defmt::trace;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin, PinState};

/// A trait for pin types that allows running a closure with the pin configured as an input.
pub trait WithInput {
    /// The type of the pin after it has been reconfigured to an input
    type Input: InputPin;

    /// Run the provided closure with the pin configured as an input.
    fn with_input<R>(&mut self, f: impl Fn(&mut Self::Input) -> R) -> R;
}

/// A trait for pin types that allows running a closure with the pin configured as an output.
pub trait WithOutput {
    /// The type of the pin after it has been reconfigured to an output
    type Output: OutputPin;

    /// Run the provided closure with the pin configured as an output.
    fn with_output<R>(&mut self, f: impl Fn(&mut Self::Output) -> R) -> R;
}

/// A trait defining delays to be inserted at specific points in the bitbanging process.
///
/// `DelayNs` is not used, because the delays involved are likely to be very short (up to and
/// including a single NOP). It is intended that these functions be implemented with NOPs or
/// functions such as `cortex_m::asm::delay`. Some of these delays may not be required on your
/// hardware at all, in which case you can leave the implementation blank. It is recommended to
/// mark implementations as `#[inline(always)]`.
///
/// An implementation designed for use with the RP2040 with the TP370PGH01 display is provided
/// in the `tp370pgh01` crate (with the `rp2040` feature enabled).
pub trait PervasiveSpiDelays {
    fn delay_read_after_sck_high(&self);
    fn delay_read_after_sck_low(&self);
    fn delay_read_after_byte(&self);

    fn delay_write_after_sda_set(&self);
    fn delay_write_after_sck_high(&self);
    fn delay_write_after_sck_low(&self);
    fn delay_write_after_byte(&self);
}

/// A bitbanging driver for the SPI-derived protocol used by Pervasive Displays e-paper screens.
pub struct PervasiveSpi<Cs, Sda, Sck, Dc, Delay> {
    cs: Cs,
    sda: Sda,
    sck: Sck,
    dc: Dc,
    delay: Delay,
}

impl<Cs, Sda, Sck, Dc, Delay, Error> PervasiveSpi<Cs, Sda, Sck, Dc, Delay>
where
    Cs: OutputPin<Error = Error>,
    Sda: WithInput + WithOutput,
    <Sda as WithInput>::Input: InputPin<Error = Error>,
    <Sda as WithOutput>::Output: OutputPin<Error = Error>,
    Sck: OutputPin<Error = Error>,
    Dc: OutputPin<Error = Error>,
    Delay: PervasiveSpiDelays,
{
    /// Create a new instance.
    ///
    /// * `cs`: The chip select pin (output).
    /// * `sda`: The SDA pin. The type must implement both the `WithInput` and `WithOutput` traits.
    ///          An implementation for both of these on the RP2040 is provided if the `rp2040`
    ///          feature is enabled. For internal reasons, the trait is implemented on `Option<Pin>`
    ///          instead of `Pin` on the RP2040, but reading and writing will panic if `None` is
    ///          provided.
    /// * `scl`: The SCL pin (output).
    /// * `dc`: The D/C (data/command) pin (output).
    /// * `delay`: An object implementing `PervasiveSpiDelays`, describing delays to be inserted at
    ///            particular points in the bitbanging process.
    pub fn new(cs: Cs, sda: Sda, sck: Sck, dc: Dc, delay: Delay) -> Self {
        Self {
            cs,
            sda,
            sck,
            dc,
            delay,
        }
    }

    fn _write(&mut self, data: &[u8], register: bool, reverse_bits: bool) -> Result<(), Error> {
        if register {
            self.dc.set_low()?;
        } else {
            self.dc.set_high()?;
        }

        for &(mut byte) in data {
            self.cs.set_low()?;

            for i in 0..8 {
                let bit = if reverse_bits {
                    byte & (1 << i)
                } else {
                    byte & (1 << (7 - i))
                };
                let state = PinState::from(bit != 0);

                self.sda.with_output(|sda| sda.set_state(state))?;
                self.delay.delay_write_after_sda_set();
                self.sck.set_high()?;
                self.delay.delay_write_after_sck_high();
                self.sck.set_low()?;
                self.delay.delay_write_after_sck_low();
            }

            self.dc.set_high()?;
            self.cs.set_high()?;
            self.delay.delay_write_after_byte();
        }

        Ok(())
    }

    /// Write the contents of `data` to the display, treating the first bytes in `data` as the
    /// register number. The D/C pin will be held low when sending the first byte, and high for
    /// the remaining bytes.
    pub fn write_register(&mut self, data: &[u8]) -> Result<(), Error> {
        self._write(data, true, false)
    }

    /// Write the contents of `data` to the display. All bytes in `data` are treated as data, i.e.
    /// the D/C pin will remain high for all bytes.
    pub fn write_data(&mut self, data: &[u8]) -> Result<(), Error> {
        self._write(data, false, false)
    }

    /// Write the contents of `data` to the display, reversing the order of the bits in each byte.
    ///
    /// This is useful for sending image data, as the more common way of encoding raw 1-bit mono
    /// images (e.g. with ImageMagick's MONO format) uses the opposite bit order from what Pervasive
    /// Displays screens expect.
    pub fn write_data_reversed(&mut self, data: &[u8]) -> Result<(), Error> {
        self._write(data, false, true)
    }

    /// Read data from the display into `buf`.
    ///
    /// All bytes in `buf` will be filled. It is the caller's responsibility to make sure `buf` is
    /// not longer than the number of bytes they want to read.
    pub fn read(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for byte in buf.iter_mut() {
            self.cs.set_low()?;

            for i in 0..8 {
                self.sck.set_high()?;
                self.delay.delay_read_after_sck_high();
                *byte |= (self.sda.with_input(|sda| sda.is_high())? as u8) << (7 - i);
                self.sck.set_low()?;
                self.delay.delay_read_after_sck_low();
            }

            trace!("read byte {}", byte);

            self.cs.set_high()?;
            self.delay.delay_read_after_byte();
        }

        Ok(())
    }

    /// Manually set the chip-select pin high. This is intended for performing hard display resets.
    pub fn set_cs_high(&mut self) -> Result<(), Error> {
        self.cs.set_high()
    }

    /// Manually et the chip-select pin low. This is intended for performing hard display resets.
    pub fn set_cs_low(&mut self) -> Result<(), Error> {
        self.cs.set_low()
    }
}
