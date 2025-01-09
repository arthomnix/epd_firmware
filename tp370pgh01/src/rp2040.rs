use pervasive_spi::PervasiveSpiDelays;

pub use pervasive_spi::rp2040::IoPin;

/// A type implementing `PervasiveSpiDelays` designed for use with the TP370PGH01 display on the
/// RP2040 (running at its default speed).
pub struct Rp2040PervasiveSpiDelays;

impl PervasiveSpiDelays for Rp2040PervasiveSpiDelays {
    #[inline(always)]
    fn delay_read_after_sck_high(&self) {
        cortex_m::asm::delay(100);
    }

    #[inline(always)]
    fn delay_read_after_sck_low(&self) {
        cortex_m::asm::delay(100);
    }

    #[inline(always)]
    fn delay_read_after_byte(&self) {
        cortex_m::asm::delay(100);
    }

    #[inline(always)]
    fn delay_write_after_sda_set(&self) {
        // no delay
    }

    #[inline(always)]
    fn delay_write_after_sck_high(&self) {
        cortex_m::asm::nop();
    }

    #[inline(always)]
    fn delay_write_after_sck_low(&self) {
        // no delay
    }

    #[inline(always)]
    fn delay_write_after_byte(&self) {
        // no delay
    }
}
