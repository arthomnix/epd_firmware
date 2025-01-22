//! Bitbanging driver for the Pervasive Displays TP370PGH01 display.

#![no_std]

#[cfg(feature = "rp2040")]
pub mod rp2040;

#[cfg(feature = "defmt")]
use defmt::{debug, error, trace};

use core::error::Error;
use core::fmt::{Debug, Display, Formatter};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use pervasive_spi::{PervasiveSpi, PervasiveSpiDelays, WithInput, WithOutput};

/// The horizontal size of the display in pixels.
pub const DIM_X: usize = 240;
/// The vertical size of the display in pixels.
pub const DIM_Y: usize = 416;
/// The number of bytes in an image frame sent to the display, equal to `(DIM_X * DIM_Y) / 8`.
pub const IMAGE_BYTES: usize = (DIM_X * DIM_Y) / 8;

/// An error that can occur communicating with the display.
pub enum Tp370pgh01Error<GE> {
    /// An error occured reading or setting the GPIO pins,
    GpioError(GE),

    /// Reading the PSR value from the display's OTP memory failed.
    ///
    /// This usually indicates that there is a problem with the connection to the display, or
    /// not enough delays are being inserted.
    ReadPsrInvalid,
}

impl<GE> From<GE> for Tp370pgh01Error<GE> {
    fn from(value: GE) -> Self {
        Self::GpioError(value)
    }
}

impl<GE: Error> Debug for Tp370pgh01Error<GE> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::GpioError(e) => write!(f, "GPIO error: {e:?}"),
            Self::ReadPsrInvalid => write!(f, "Could not find PSR in display OTP memory"),
        }
    }
}

impl<GE: Error> Display for Tp370pgh01Error<GE> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::GpioError(e) => write!(f, "GPIO error: {e}"),
            Self::ReadPsrInvalid => write!(f, "Could not find PSR in display OTP memory"),
        }
    }
}

impl<GE: Error> Error for Tp370pgh01Error<GE> {}

/// A bitbanging driver for the Pervasive Displays TP370PGH01 display.
pub struct Tp370pgh01<Cs, Sda, Sck, Dc, Busy, Reset, Delay, SpiDelay> {
    spi: PervasiveSpi<Cs, Sda, Sck, Dc, SpiDelay>,
    busy: Busy,
    reset: Reset,
    delay: Delay,
    psr: Option<[u8; 2]>,
}

impl<Cs, Sda, Sck, Dc, Busy, Reset, Delay, SpiDelay, Error>
    Tp370pgh01<Cs, Sda, Sck, Dc, Busy, Reset, Delay, SpiDelay>
where
    Cs: OutputPin<Error = Error>,
    Sda: WithInput + WithOutput,
    <Sda as WithInput>::Input: InputPin<Error = Error>,
    <Sda as WithOutput>::Output: OutputPin<Error = Error>,
    Sck: OutputPin<Error = Error>,
    Dc: OutputPin<Error = Error>,
    Busy: InputPin<Error = Error>,
    Reset: OutputPin<Error = Error>,
    Delay: DelayNs,
    SpiDelay: PervasiveSpiDelays,
{
    /// Create a new instance of the driver.
    ///
    /// * `cs` - Chip select pin (output).
    /// * `sda` - SDA pin (bidirectional). This must implement both the `WithInput` and `WithOutput`
    ///          traits from the `pervasive-spi` crate. That crate provides a type implementing
    ///          these traits for the RP2040 if the `rp2040` feature is enabled.
    /// * `sck` - SCK pin (output).
    /// * `dc` - D/C (data/command) pin (output).
    /// * `reset` - Display reset pin (output).
    /// * `delay` - An instance of `DelayNs`, used for timing some display commands such as resets.
    /// * `spi_delay` - An instance of `PervasiveSpiDelays` from the `pervasive-spi` crate. This
    ///                 crate provides an implementation for the RP2040 with this display if the
    ///                 `rp2040` feature is activated.
    pub fn new(
        cs: Cs,
        sda: Sda,
        sck: Sck,
        dc: Dc,
        busy: Busy,
        reset: Reset,
        delay: Delay,
        spi_delay: SpiDelay,
    ) -> Self {
        let spi = PervasiveSpi::new(cs, sda, sck, dc, spi_delay);
        Self {
            spi,
            busy,
            reset,
            delay,
            psr: None,
        }
    }

    /// Hard-reset the display using the reset pin, then perform a soft reset.
    pub fn hard_reset(&mut self) -> Result<(), Tp370pgh01Error<Error>> {
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: hard resetting display");
        self.spi.set_cs_low()?;
        self.reset.set_high()?;
        self.delay.delay_ms(5);
        self.reset.set_low()?;
        self.delay.delay_ms(10);
        self.reset.set_high()?;
        self.delay.delay_ms(5);
        self.spi.set_cs_high()?;
        self.soft_reset()?;
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: hard reset display");

        Ok(())
    }

    /// Perform a soft reset.
    pub fn soft_reset(&mut self) -> Result<(), Tp370pgh01Error<Error>> {
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: soft resetting display");
        self.spi.write_register(&[0x00, 0x0e])?;
        self.delay.delay_ms(5);
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: soft reset display");
        Ok(())
    }

    fn get_psr(&mut self) -> Result<[u8; 2], Tp370pgh01Error<Error>> {
        if let Some(psr) = self.psr {
            return Ok(psr);
        }

        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: reading PSR");
        self.spi.write_register(&[0xa2])?;
        let mut buf = [0u8; 2];
        self.delay.delay_ms(10);
        self.spi.read(&mut buf)?;

        let bank0 = buf[1] == 0xa5;
        #[cfg(feature = "defmt")]
        debug!(
            "tp370pgh01: PSR is in bank {}",
            if bank0 { '0' } else { '1' }
        );

        let offset_psr: u16 = if bank0 { 0x0fb4 } else { 0x1fb4 };
        let offset_a5: u16 = if bank0 { 0x0000 } else { 0x1000 };

        if offset_a5 > 0 {
            for _ in 1..offset_a5 {
                self.spi.read(&mut [0])?;
            }

            let mut buf = [0];
            self.spi.read(&mut buf)?;
            if buf[0] != 0xa5 {
                #[cfg(feature = "defmt")]
                error!("tp370pgh01: failed to find PSR");
                return Err(Tp370pgh01Error::ReadPsrInvalid);
            }
        }

        for _ in offset_a5 + 1..offset_psr {
            self.spi.read(&mut [0])?;
        }

        self.spi.read(&mut buf)?;
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: found PSR: {} {}", buf[0], buf[1]);

        self.psr.replace(buf);

        Ok(buf)
    }

    fn busy_wait(&mut self) -> Result<(), Tp370pgh01Error<Error>> {
        #[cfg(feature = "defmt")]
        trace!("tp370pgh01: busy waiting");
        while self.busy.is_low()? {}
        Ok(())
    }

    fn trigger_refresh(&mut self) -> Result<(), Tp370pgh01Error<Error>> {
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: turning on DC/DC");
        // turn on DC/DC
        self.spi.write_register(&[0x04])?;
        self.busy_wait()?;
        // refresh
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: refreshing");
        self.spi.write_register(&[0x12])?;
        self.busy_wait()?;
        // turn off DC/DC
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: turning off DC/DC");
        self.spi.write_register(&[0x02])?;
        self.busy_wait()?;

        Ok(())
    }

    /// Refresh the display using a normal (non-fast) refresh.
    ///
    /// This type of refresh takes a couple of seconds, and the screen will flash between black and
    /// white multiple times before the image appears.
    ///
    /// * `image` - The image data to write to the display.
    /// * `temperature` - The ambient temperature in degrees Celsius. According to the display
    ///                   datasheet, this should be clamped between 0 and 60 degrees.
    pub fn refresh(
        &mut self,
        image: &[u8; IMAGE_BYTES],
        temperature: u8,
    ) -> Result<(), Tp370pgh01Error<Error>> {
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: begin normal refresh");

        let psr = self.get_psr()?;

        // set temperature
        self.spi.write_register(&[0xe5, temperature])?;
        self.spi.write_register(&[0xe0, 0x02])?;

        // set PSR
        self.spi.write_register(&[0x00])?;
        self.spi.write_data(&psr)?;

        // write image data
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: writing image");
        self.spi.write_register(&[0x10])?;
        self.spi.write_data_reversed(image)?;
        // write dummy data
        self.spi.write_register(&[0x13])?;
        for _ in 0..IMAGE_BYTES {
            self.spi.write_data(&[0x00])?;
        }
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: finished writing image");

        self.trigger_refresh()?;

        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: end normal refresh");
        Ok(())
    }

    /// Refresh the display using a fast refresh.
    ///
    /// This type of refresh takes around half a second and does not flash the screen, but is
    /// susceptible to ghosting.
    ///
    /// * `image` - The image data to write to the display.
    /// * `prev_image` The image data currently being displayed.
    /// * `temperature` - The ambient temperature in degrees Celsius. According to the display
    ///                   datasheet, this should be clamped between 0 and 60 degrees.
    pub fn refresh_fast(
        &mut self,
        image: &[u8; IMAGE_BYTES],
        prev_image: &[u8; IMAGE_BYTES],
        temperature: u8,
    ) -> Result<(), Tp370pgh01Error<Error>> {
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: begin fast refresh");

        let psr = self.get_psr()?;

        // set temperature
        self.spi.write_register(&[0xe5, temperature | 0x40])?;
        self.spi.write_register(&[0xe0, 0x02])?;
        // set PSR
        self.spi
            .write_register(&[0x00, psr[0] | 0x10, psr[1] | 0x02])?;
        // set "Vcom and data interval setting"
        self.spi.write_register(&[0x50, 0x07])?;

        // set "border setting"
        self.spi.write_register(&[0x50, 0x27])?;
        // send previous image
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: writing image");
        self.spi.write_register(&[0x10])?;
        self.spi.write_data_reversed(prev_image)?;
        // send new image
        self.spi.write_register(&[0x13])?;
        self.spi.write_data_reversed(image)?;
        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: finished writing image");
        // set "border setting"
        self.spi.write_register(&[0x50, 0x07])?;

        self.trigger_refresh()?;

        #[cfg(feature = "defmt")]
        debug!("tp370pgh01: end fast refresh");
        Ok(())
    }
}
