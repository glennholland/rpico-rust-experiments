#![no_std]
#![no_main]

use embedded_hal::digital::OutputPin;
use embedded_hal::pwm::SetDutyCycle;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM0
    let mut pwm = pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.set_div_int(250); // Integer divider
    pwm.set_div_frac(0); // Fractional divider
    pwm.set_top(10000); // Counter wrap value // For 50Hz with 125MHz system clock
    pwm.enable();

    let channel = &mut pwm.channel_a;
    channel.output_to(pins.gpio0);

    //let max_duty = channel.max_duty_cycle();

    // LED for debugging
    let mut led_pin = pins.led.into_push_pull_output();

    // Main loop
    loop {
        // Turn on LED to indicate we're setting PWM high
        led_pin.set_high().unwrap();

        // Set PWM to maximum (should be close to 3.3V)
        channel.set_duty_cycle_percent(12).unwrap();
        delay.delay_ms(1000);

        // Turn off LED to indicate we're setting PWM low
        led_pin.set_low().unwrap();

        // Set PWM to minimum (should be close to 0V)
        channel.set_duty_cycle_percent(40).unwrap();
        delay.delay_ms(1000);
    }
}
