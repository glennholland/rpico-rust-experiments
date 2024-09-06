#![no_std]
#![no_main]

use core::fmt::Debug;
use core::str::from_utf8;
use cortex_m::asm::nop;
use embedded_hal::digital::OutputPin;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal::pwm::SetDutyCycle;
use heapless::String;
use numtoa::NumToA;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;
#[entry]
fn main() -> ! {
    ////////////////////////////////////////////////////////////
    // Setup
    ////////////////////////////////////////////////////////////

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

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    ////////////////////////////////////////////////////////////
    // USB serial Setup
    ////////////////////////////////////////////////////////////

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    ////////////////////////////////////////////////////////////
    // PWM Setup
    ////////////////////////////////////////////////////////////

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

    ////////////////////////////////////////////////////////////
    // LED Setup
    ////////////////////////////////////////////////////////////

    // LED for debugging
    let mut led_pin = pins.led.into_push_pull_output();

    ////////////////////////////////////////////////////////////
    // Main Loop
    ////////////////////////////////////////////////////////////

    let mut initialised = false;
    let mut last_changed: u64 = 0;
    loop {
        if !initialised && timer.get_counter().ticks() >= 10_000_000 {
            initialised = true;
            send_to_serial_str(&mut serial, "Initialised");
            send_to_serial_newline(&mut serial);
            last_changed = timer.get_counter().ticks();
            led_pin.set_high().unwrap();
        }

        if (timer.get_counter().ticks() - last_changed) >= 1_000_000 {
            last_changed = timer.get_counter().ticks();
            led_pin.toggle().unwrap();
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_) => {
                    nop();
                }
                Ok(0) => {
                    send_to_serial_str(&mut serial, "Empty");
                    send_to_serial_newline(&mut serial);
                }
                Ok(count) => {
                    match from_utf8(&buf[..count]) {
                        Ok(input_str) => {
                            send_to_serial_str(&mut serial, "Received: ");
                            send_to_serial_str(&mut serial, input_str);
                            send_to_serial_newline(&mut serial);
                            let trimmed_input = input_str.trim();
                            // First, check if the input consists only of digits
                            if trimmed_input.chars().all(|c| c.is_ascii_digit()) {
                                // Then, parse it as a u32 to safely handle any number
                                match trimmed_input.parse::<u32>() {
                                    Ok(number) if number <= 180 => {
                                        let mut buffer = [0u8; 20];
                                        let parsed_number = number.numtoa(10, &mut buffer);
                                        send_to_serial_raw(&mut serial, parsed_number);
                                        send_to_serial_newline(&mut serial);

                                        let duty_cycle = (number * 100) / 180;
                                        channel.set_duty_cycle_percent(duty_cycle as u8).unwrap();
                                    }
                                    _ => {
                                        // If the number is out of range or parsing fails
                                        send_to_serial_str(&mut serial, "Invalid angle: ");
                                        send_to_serial_str(&mut serial, trimmed_input);
                                        send_to_serial_newline(&mut serial);
                                    }
                                }
                            } else {
                                send_to_serial_str(&mut serial, "It was not all digits!");
                                send_to_serial_newline(&mut serial);
                            }
                        }

                        Err(_) => {
                            send_to_serial_str(&mut serial, "Received invalid UTF-8 data");
                            send_to_serial_newline(&mut serial);
                            send_to_serial_str(&mut serial, "Raw bytes: ");
                            send_to_serial_newline(&mut serial);
                            send_to_serial_raw(&mut serial, &buf[..count]);
                        }
                    }
                }
            }
        }
    }
}

fn send_to_serial_str<B: UsbBus>(serial: &mut SerialPort<'_, B>, message: &str) {
    let mut response_str: String<40> = String::new();
    response_str.push_str(message).unwrap();
    serial.write(response_str.as_bytes()).unwrap();
}

fn send_to_serial_newline<B: UsbBus>(serial: &mut SerialPort<'_, B>) {
    serial.write(b"\n").unwrap();
}

fn send_to_serial_raw<B: UsbBus>(serial: &mut SerialPort<'_, B>, raw: &[u8]) {
    serial.write(raw).unwrap();
    serial.write(b"\n").unwrap();
}
