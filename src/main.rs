#![no_std]
#![no_main]

extern crate alloc;

mod hardware;

use cortex_m::delay::Delay;
use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal::gpio::DynPin;
use rp_pico::hal::Timer;
use rp_pico::Pins;

use crate::hardware::serial;

fn start(mut delay: Delay, _timer: Timer, pins: Pins) -> ! {
    // Get the LED pin.
    let mut led_pin = DynPin::from(pins.led);
    led_pin.into_push_pull_output();

    loop {
        // Blink the pin.
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        if serial::available() {
            // Read and write serial.
            let line = serial::read_until(b'\r');
            let name = line.trim();

            serial::println!("Hello, {name}!");
        }
    }
}
