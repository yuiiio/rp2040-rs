//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;
use hal::pac::interrupt;

// Some traits we need
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;

use hal::prelude::*;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_human_interface_device::device::joystick::{JoystickReport, Joystick};
use usbd_human_interface_device::usb_class::UsbHidClass;
use usbd_human_interface_device::prelude::*;

use frunk::{HCons, HNil};

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID_JOY: Option<UsbHidClass<hal::usb::UsbBus, HCons<Joystick<hal::usb::UsbBus>, HNil>>> = None;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

const DPAD_UP: [bool; 4]= [false, false, false, true];
const DPAD_DOWN: [bool; 4]= [false, true, false, true];
const DPAD_RIGHT: [bool; 4]= [false, false, true, true];
const DPAD_LEFT: [bool; 4]= [false, true, true, true];

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    let usb_hid_joy = UsbHidClassBuilder::new()
        .add_device(usbd_human_interface_device::device::joystick::JoystickConfig::default())
        .build(bus_ref);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_HID_JOY = Some(usb_hid_joy);
    }

    //https://pid.codes
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x1209, 0x0001))
        .manufacturer("usbd-human-interface-device")
        .product("Rusty joystick")
        .serial_number("TEST")
        .max_packet_size_0(8) // should change 16, 32,, when over report size over 8 byte ?
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO{26, 27, 28, 29} as an ADC input
    let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26.into_floating_input());
    let mut adc_pin_1 = hal::adc::AdcPin::new(pins.gpio27.into_floating_input());
    let mut adc_pin_2 = hal::adc::AdcPin::new(pins.gpio28.into_floating_input());
    let mut adc_pin_3 = hal::adc::AdcPin::new(pins.gpio29.into_floating_input());
    
    // NOTE:
    // RP2040-datasheet.pdf say 
    // If the FIFO is full when a conversion completes, the sticky error flag FCS.OVER is set. 
    // The current FIFO contents are not changed by this event,
    // but any conversion that completes whilst the FIFO is full will be lost.
    // 
    // Is there always a two read interval delay?
    // After a long interval the next value to read is,
    // the next value read after a long interval is the value before that interval?
    //
    // Configure free-running mode:
    let mut adc_fifo = adc
        .build_fifo()
        // Set clock divider to target a sample rate of 1000 samples per second (1ksps).
        // The value was calculated by `(48MHz / 1ksps) - 1 = 47999.0`.
        // Please check the `clock_divider` method documentation for details.
        //.clock_divider(47999, 0)
        .clock_divider(0, 0) // default 48MHz / 96 = 500ksps
        //.set_channel(&mut adc_pin_0)
        // then alternate between GPIO26 and the temperature sensor
        .round_robin((&mut adc_pin_3, &mut adc_pin_2, &mut adc_pin_1, &mut adc_pin_0))
        // Uncomment this line to produce 8-bit samples, instead of 12 bit (lower bits are discarded)
        .shift_8bit()
        // start sampling
        .start();

    // Configure GPIO [8 ~ 23] as an input
    let in_pin_r3 = pins.gpio8.into_pull_up_input();
    let in_pin_l3 = pins.gpio9.into_pull_up_input();
    let in_pin_menu = pins.gpio10.into_pull_up_input();
    let in_pin_overview = pins.gpio11.into_pull_up_input();
    let in_pin_d_down = pins.gpio12.into_pull_up_input();
    let in_pin_d_left = pins.gpio13.into_pull_up_input();
    let in_pin_d_right = pins.gpio14.into_pull_up_input();
    let in_pin_d_up = pins.gpio15.into_pull_up_input();
    let in_pin_lt = pins.gpio16.into_pull_up_input();
    let in_pin_lz = pins.gpio17.into_pull_up_input();
    let in_pin_rz = pins.gpio18.into_pull_up_input();
    let in_pin_rt = pins.gpio19.into_pull_up_input();
    let in_pin_y = pins.gpio20.into_pull_up_input();
    let in_pin_x = pins.gpio21.into_pull_up_input();
    let in_pin_b = pins.gpio22.into_pull_up_input();
    let in_pin_a = pins.gpio23.into_pull_up_input();

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    
    // Move the cursor up and down every 200ms
    loop {
        led_pin.set_low().unwrap();

        // busy-wait until the FIFO contains at least 4 samples:
        while adc_fifo.len() < 4 {}

        led_pin.set_high().unwrap();

        // fetch 4 values from the fifo
        let adc_result_3 = adc_fifo.read();
        let adc_result_2 = adc_fifo.read();
        let adc_result_1 = adc_fifo.read();
        let adc_result_0 = adc_fifo.read();

        let lx = adc_result_3;
        let ly = adc_result_2;
        let rx = adc_result_1;
        let ry = adc_result_0;

        let mut buttons1 = 0b00000000;

        if in_pin_a.is_low().unwrap() {
            buttons1 |= 0b00000001;
        }
        if in_pin_b.is_low().unwrap() {
            buttons1 |= 0b00000010;
        }
        if in_pin_x.is_low().unwrap() {
            buttons1 |= 0b00000100;
        }
        if in_pin_y.is_low().unwrap() {
            buttons1 |= 0b00001000;
        }
        //report.buttons2 = [true, true, true, true];
        //report.hat_switch = DPAD_RIGHT;

        let report = JoystickReport {
            lx: lx,
            ly: ly,
            rx: rx,
            ry: ry,
            lz: 0, // 0~255 expect analog trigger but, rp2040 has only 4 analogin so 
            rz: 0, // use binary value 0, 1 and map to 0 , 255.
            buttons1: buttons1, // high [menu, overview, RT, LT, Y, X, B, A] low
            buttons2: [false; 4], // high [?, R3, L3, ?] low
            hat_switch: [false; 4], // see DPAD_*
        };

        push_gamepad_input(report).ok().unwrap_or(());
    }
    
    // Stop free-running mode (the returned `adc` can be reused for future captures)
    // let _adc = adc_fifo.stop();
}

/// Submit a new gamepad inpuit report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_gamepad_input(report: JoystickReport) -> Result<(), UsbHidError> {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID_JOY.as_mut().map(|hid_joy| hid_joy.device().write_report(&report))
    })
    .unwrap()
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid_joy = USB_HID_JOY.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid_joy]);
}

// End of file
