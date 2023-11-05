//! This example shows how to read from and write to PIO using DMA.
//!
//! If a LED is connected to that pin, like on a Pico board, it will continously output "HELLO
//! WORLD" in morse code. The example also tries to read the data back. If reading the data fails,
//! the message will only be shown once, and then the LED remains dark.
//!
//! See the `Cargo.toml` file for Copyright and licence details.
#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m_rt::entry;
use fugit::RateExtU32;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::sio::Sio;
use panic_halt as _;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig};

const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let encoder_pin1: Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
    // PIN id for use inside of PIO
    let encoder_pin1_id = encoder_pin1.id().num;

    // Define a PIO program which reads data from the TX FIFO bit by bit, configures the LED
    // according to the data, and then writes the data back to the RX FIFO.
    let program = pio_proc::pio_file!("src/quadrature_encoder.pio");

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, mut rx, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .jmp_pin(encoder_pin1_id)
        .in_pin_base(encoder_pin1_id)
        .clock_divisor_fixed_point(16, 0)
        .autopull(false)
        .autopush(false)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([
        (encoder_pin1_id, hal::pio::PinDir::Input),
        (encoder_pin1_id + 1, hal::pio::PinDir::Input),
    ]);

    sm.start();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );

    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart.write_full_blocking(b"UART example\r\n");
    writeln!(uart, "Debug {encoder_pin1_id}").unwrap();

    loop {
        // let value = rx.read().unwrap() as i32;
        let mut value = 0;
        while let Some(encoder_value) = rx.read() {
            value = encoder_value as i32;
        }
        writeln!(uart, "value: {value}\r").unwrap();
        delay.delay_ms(100);
    }
}
