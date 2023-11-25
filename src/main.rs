#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::prelude::_embedded_hal_PwmPin;
use cortex_m_rt::entry;
use embedded_hal::serial::Read;
use fugit::RateExtU32;
use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::sio::Sio;
use hal::uart::{DataBits, StopBits, UartConfig};
use heapless::String;
use panic_halt as _;
use pid::Pid;
use rp2040_hal as hal;
use rp2040_hal::clocks::Clock;

const MOTOR_PWM_DIV: u8 = 64;

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

    let encoder_pin0: Pin<_, FunctionPio0, _> = pins.gpio16.into_function();
    // PIN id for use inside of PIO
    let encoder_pin0_id = encoder_pin0.id().num;

    let encoder_program = pio_proc::pio_file!("src/quadrature_encoder.pio");

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&encoder_program.program).unwrap();
    let (mut sm, mut rx, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .out_shift_direction(hal::pio::ShiftDirection::Right)
        .jmp_pin(encoder_pin0_id)
        .in_pin_base(encoder_pin0_id)
        .clock_divisor_fixed_point(16, 0)
        .autopull(false)
        .autopush(false)
        .build(sm0);

    sm.set_pindirs([
        (encoder_pin0_id, hal::pio::PinDir::Input),
        (encoder_pin0_id + 1, hal::pio::PinDir::Input),
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

    const PID_LIMIT: f32 = 13.0;
    let mut pid_controller: Pid<f32> = Pid::new(0.0, PID_LIMIT);
    pid_controller.p(0.01, PID_LIMIT);

    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let motor_pwm = &mut pwm_slices.pwm1;
    motor_pwm.set_ph_correct();
    motor_pwm.set_div_int(MOTOR_PWM_DIV);
    motor_pwm.enable();

    let mut motor_ctl = hbridge::HBridge::new(motor_pwm);
    motor_ctl.set_output_pins(pins.gpio18, pins.gpio19);

    let mut value = 0;

    loop {
        // let mut encoder_measurement = 0;
        // while let Some(encoder_value) = rx.read() {
        //     encoder_measurement = encoder_value as i32;
        // }
        // writeln!(uart, "value: {encoder_measurement}\r").unwrap();
        // delay.delay_ms(100);

        if uart.uart_is_readable() {
            delay.delay_us(500);
            let mut uart_string: String<8> = String::new();
            while let Ok(uart_data) = uart.read() {
                if (uart_data as char) == '\n' {
                    writeln!(uart, "Got {uart_string}").unwrap();
                    break;
                }
                if uart_string.push(uart_data as char).is_err() {
                    break;
                }
            }
            if let Ok(uart_data) = uart_string.parse::<i32>() {
                value = uart_data;
            } else {
                writeln!(uart, "Invalid number").unwrap();
            }
        }
        motor_ctl.set_speed(value);
        // let controller_output = pid_controller.next_control_output(value as f32);
    }
}

mod hbridge {
    use core::cmp::Ordering;

    use embedded_hal::PwmPin;
    use hal::gpio::AnyPin;
    use hal::pwm;
    use hal::pwm::{Slice, SliceId, ValidPwmOutputPin, ValidSliceMode};
    use rp2040_hal as hal;

    pub struct HBridge<'a, I: SliceId, M: ValidSliceMode<I>> {
        pwm_slice: &'a mut Slice<I, M>,
        speed: i32,
    }

    impl<'a, I: SliceId, M: ValidSliceMode<I>> HBridge<'a, I, M> {
        pub fn new(pwm_slice: &'a mut Slice<I, M>) -> HBridge<'a, I, M> {
            HBridge {
                pwm_slice,
                speed: 0,
            }
        }

        pub fn set_output_pins<F: AnyPin, B: AnyPin>(
            &mut self,
            forward_pin: F,
            reverse_pin: B,
        ) -> &Self
        where
            <B as AnyPin>::Id: ValidPwmOutputPin<I, pwm::B>,
            <F as AnyPin>::Id: ValidPwmOutputPin<I, pwm::A>,
        {
            let _ = &self.pwm_slice.channel_a.output_to(forward_pin);
            let _ = &self.pwm_slice.channel_b.output_to(reverse_pin);
            self
        }

        pub fn set_speed(&mut self, speed: i32) -> &Self {
            self.speed = speed;
            match speed.cmp(&0) {
                Ordering::Greater => {
                    let _ = &self.pwm_slice.channel_a.set_duty(speed as u16);
                    let _ = &self.pwm_slice.channel_b.set_duty(0);
                }
                Ordering::Less => {
                    let _ = &self
                        .pwm_slice
                        .channel_b
                        .set_duty(speed.unsigned_abs() as u16);
                    let _ = &self.pwm_slice.channel_a.set_duty(0);
                }
                Ordering::Equal => {
                    let _ = &self.pwm_slice.channel_a.set_duty(0);
                    let _ = &self.pwm_slice.channel_b.set_duty(0);
                }
            }
            self
        }
    }
}

// fn hbridge_setup<I: SliceId, M: ValidSliceMode<I>>(pwm_slice: &mut Slice<I, M>) {
//     pwm_slice.set_ph_correct();
//     pwm_slice.set_div_int(MOTOR_PWM_DIV);
//     pwm_slice.set_top(65000);
//     pwm_slice.enable();
// }

// fn calculate_hbridge(speed: i8) -> Option<(u16, u16)> {
//     const SPEED_MULTIPLIER: u16 = 5000;
//     let converted_speed = speed.unsigned_abs() as u16;
//     match speed {
//         0 => Some((0, 0)),
//         1..=13 => Some((converted_speed * SPEED_MULTIPLIER, 0)),
//         -13..=-1 => Some((0, converted_speed * SPEED_MULTIPLIER)),
//         _ => None,
//     }
// }
