#![no_std]
#![no_main]

extern crate panic_halt;

use core::fmt::Write;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::Text;
use heapless::String;
use eepy_gui::draw_target::EpdDrawTarget;
use eepy_gui::element::button::Button;
use eepy_gui::element::{Gui, DEFAULT_TEXT_STYLE};
use eepy_sys::input::{eep, next_event, set_touch_enabled};
use eepy_sys::{eepy_app, kv_store};

fn load_counter() -> u32 {
    let mut buf = [0u8; size_of::<i32>()];
    match kv_store::get(b"counter", &mut buf) {
        Ok(_) => u32::from_ne_bytes(buf),
        Err(_) => 0,
    }
}

fn save_counter(counter: u32) {
    let _ = kv_store::put(b"counter", &counter.to_ne_bytes());
}

fn render_counter(draw_target: &mut EpdDrawTarget, counter: u32) {
    let mut s = String::<16>::new();
    write!(s, "{counter}").unwrap();
    Text::new(&s, Point::new(10, 80), DEFAULT_TEXT_STYLE)
        .draw(draw_target)
        .unwrap();
}

#[eepy_app(name = "ExampleApp")]
fn main() {
    set_touch_enabled(true);

    let mut draw_target = EpdDrawTarget::default();

    let text = Text::new("Example App", Point::new(10, 20), DEFAULT_TEXT_STYLE);
    let mut button = Button::with_default_style_auto_sized(Point::new(10, 40), "Click me", true);
    let mut exit_button = Button::with_default_style_auto_sized(Point::new(10, 386), "Exit", false);

    let mut counter = load_counter();

    text.draw(&mut draw_target).unwrap();
    button.draw_init(&mut draw_target);
    exit_button.draw_init(&mut draw_target);
    render_counter(&mut draw_target, counter);
    draw_target.refresh(false);


    loop {
        while let Some(ev) = next_event() {
            let mut needs_refresh = false;

            let exit_res = exit_button.tick(&mut draw_target, ev);
            if exit_res.clicked {
                save_counter(counter);
                return;
            } else if exit_res.needs_refresh {
                draw_target.refresh(true);
            }

            let response = button.tick(&mut draw_target, ev);
            if response.clicked {
                draw_target.clear(BinaryColor::Off).unwrap();
                text.draw(&mut draw_target).unwrap();
                button.draw_init(&mut draw_target);
                exit_button.draw_init(&mut draw_target);

                counter += 1;
                render_counter(&mut draw_target, counter);
                needs_refresh = true;
            }
            needs_refresh |= response.needs_refresh;

            if needs_refresh {
                draw_target.maybe_refresh(true);
            }
        }

        eep();
    }
}