use core::convert::Infallible;
use embedded_graphics::prelude::*;
use embedded_graphics::geometry::Dimensions;
use embedded_graphics::Pixel;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::primitives::Rectangle;
use eepy_sys::image::{refresh, write_image, RefreshBlockMode};
use tp370pgh01::{DIM_X, DIM_Y, IMAGE_BYTES};

pub struct EpdDrawTarget {
    buf: [u8; IMAGE_BYTES],
}

impl Dimensions for EpdDrawTarget {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::new(0, 0), Size::new(DIM_X as u32, DIM_Y as u32))
    }
}

impl DrawTarget for EpdDrawTarget {
    type Color = BinaryColor;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point @ Point { x, y }, colour) in pixels {
            if !self.bounding_box().contains(point) {
                continue;
            }

            let bit_index = (y as usize) * 240 + (x as usize);
            let i = bit_index >> 3;
            let j = 1 << (bit_index & 0x07);

            match colour {
                BinaryColor::Off => self.buf[i] &= !j,
                BinaryColor::On => self.buf[i] |= j,
            }
        }

        Ok(())
    }

    fn clear(&mut self, colour: Self::Color) -> Result<(), Self::Error> {
        match colour {
            BinaryColor::Off => self.buf.copy_from_slice(&[0; IMAGE_BYTES]),
            BinaryColor::On => self.buf.copy_from_slice(&[u8::MAX; IMAGE_BYTES]),
        }

        Ok(())
    }
}

impl EpdDrawTarget {
    pub const fn new() -> Self {
        Self {
            buf: [0; IMAGE_BYTES]
        }
    }

    pub fn refresh(&self, fast_refresh: bool, block_mode: RefreshBlockMode) {
        write_image(&self.buf);
        refresh(fast_refresh, block_mode);
    }
}