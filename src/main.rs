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
use fugit::RateExtU32;
use hal::clocks::Clock;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;
use hal::pac::interrupt;
use hal::gpio::PinState;

// Some traits we need
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::adc::OneShot;

use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;

use display_interface_spi::SPIInterfaceNoCS;
use embedded_graphics::image::*;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use st7789::{Orientation, ST7789};

mod xinput;
use xinput::{XINPUTClass, USB_XINPUT_VID, USB_XINPUT_PID, XinputControlReport};

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_XINPUT: Option<XINPUTClass<hal::usb::UsbBus>> = None;

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

    let usb_xinput = XINPUTClass::new(bus_ref);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_XINPUT = Some(usb_xinput);
    }

    //https://pid.codes
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(USB_XINPUT_VID, USB_XINPUT_PID))
        .manufacturer("atbjyk")
        .product("Rusty Xinput gamepad")
        .serial_number("TEST")
        .max_packet_size_0(32) // should change 16, 32,, when over report size over 8 byte ?
        .device_release(0x0114)
        .device_protocol(0xff)
        .device_class(0xff)
        .device_sub_class(0xff)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    /*
    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    */

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

    // for st7789 display
    let rst = pins.gpio4.into_push_pull_output_in_state(PinState::Low); // reset pin
    let dc = pins.gpio5.into_push_pull_output_in_state(PinState::Low); // dc pin
                                                             //
    let spi_mosi = pins.gpio3.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio2.into_function::<hal::gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_sclk));
    
    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_3,
    );

    // display interface abstraction from SPI and DC
    let di = SPIInterfaceNoCS::new(spi, dc);
    
    // create driver
    let mut display = ST7789::new(di, rst, 240, 240);

    // initialize
    display.init(&mut delay).unwrap();
    // set default orientation
    display.set_orientation(Orientation::LandscapeSwapped).unwrap();

    let raw_image_data = ImageRawLE::new(include_bytes!("../assets/ferris.raw"), 240);
    let ferris = Image::new(&raw_image_data, Point::new(80, 0));

    // draw image on black background
    display.clear(Rgb565::BLACK).unwrap();
    ferris.draw(&mut display).unwrap();

    // Enable ADC
    let mut adc = hal::Adc::new(pac.ADC, &mut pac.RESETS);

    // Configure GPIO{26, 27, 28, 29} as an ADC input
    let mut adc_pin_0 = hal::adc::AdcPin::new(pins.gpio26.into_floating_input());
    let mut adc_pin_1 = hal::adc::AdcPin::new(pins.gpio27.into_floating_input());
    let mut adc_pin_2 = hal::adc::AdcPin::new(pins.gpio28.into_floating_input());
    let mut adc_pin_3 = hal::adc::AdcPin::new(pins.gpio29.into_floating_input());
    
    /*
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
        // then alternate between GPIOS
        .round_robin((&mut adc_pin_3, &mut adc_pin_2, &mut adc_pin_1, &mut adc_pin_0))
        // Uncomment this line to produce 8-bit samples, instead of 12 bit (lower bits are discarded)
        .shift_8bit()
        // start sampling
        .start();
    */

    // Configure GPIO as an input
    let in_pin_r3 = pins.gpio24.into_pull_up_input();
    let in_pin_l3 = pins.gpio23.into_pull_up_input();
    let in_pin_menu = pins.gpio7.into_pull_up_input();
    let in_pin_overview = pins.gpio6.into_pull_up_input();
    let in_pin_d_down = pins.gpio18.into_pull_up_input();
    let in_pin_d_left = pins.gpio20.into_pull_up_input();
    let in_pin_d_right = pins.gpio19.into_pull_up_input();
    let in_pin_d_up = pins.gpio21.into_pull_up_input();
    let in_pin_lt = pins.gpio16.into_pull_up_input();
    let in_pin_lz = pins.gpio22.into_pull_up_input();
    let in_pin_rz = pins.gpio9.into_pull_up_input();
    let in_pin_rt = pins.gpio17.into_pull_up_input();
    let in_pin_y = pins.gpio15.into_pull_up_input();
    let in_pin_x = pins.gpio14.into_pull_up_input();
    let in_pin_b = pins.gpio13.into_pull_up_input();
    let in_pin_a = pins.gpio12.into_pull_up_input();

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    led_pin.set_high().unwrap();

    let mut toggle: bool = false;
    loop {
        if in_pin_overview.is_low().unwrap() && in_pin_menu.is_low().unwrap(){
            display.clear(Rgb565::WHITE).unwrap();
            toggle = true;
        } else {
            if toggle == true {
                ferris.draw(&mut display).unwrap();
                toggle = false;
            }
        }
        //led_pin.set_low().unwrap();

        // busy-wait until the FIFO contains at least 4 samples:
        // while adc_fifo.len() < 4 {}

        //led_pin.set_high().unwrap();

        // fetch 4 values from the fifo
        // let adc_result_3 = adc_fifo.read();
        // let adc_result_2 = adc_fifo.read();
        // let adc_result_1 = adc_fifo.read();
        // let adc_result_0 = adc_fifo.read();
        
        let adc_result_3: u16 = adc.read(&mut adc_pin_3).unwrap();
        let adc_result_2: u16 = adc.read(&mut adc_pin_2).unwrap();
        let adc_result_1: u16 = adc.read(&mut adc_pin_1).unwrap();
        let adc_result_0: u16 = adc.read(&mut adc_pin_0).unwrap();

        // u12 bit to i16 bit
        // norm is 0
        let adc_0: u16 = adc_result_0;
        let adc_1: u16 = adc_result_1;
        let adc_2: u16 = adc_result_2;
        let adc_3: u16 = adc_result_3;

        let lx: i16 = ((adc_0 as i16 - 2048) << 4) as i16;
        let ly: i16 = ((adc_1 as i16 - 2048) << 4) as i16 * -1;
        let rx: i16 = ((adc_2 as i16 - 2048) << 4) as i16;
        let ry: i16 = ((adc_3 as i16 - 2048) << 4) as i16 * -1;

        // clamp
        let lx: i16 = lx.clamp(-21844, 21844);
        let ly: i16 = ly.clamp(-21844, 21844);
        let rx: i16 = rx.clamp(-21844, 21844);
        let ry: i16 = ry.clamp(-21844, 21844);

        // scale
        // * 1.5 ( 1 + 1/2 ) = 3/2
        // x * 3/2 = 32,767, => x = 21844...
        let lx: i16 = lx + (lx >> 1); 
        let ly: i16 = ly + (ly >> 1);
        let rx: i16 = rx + (rx >> 1);
        let ry: i16 = ry + (ry >> 1);

        let (mut lz, mut rz): (u8, u8) = (0, 0);
        if in_pin_lz.is_low().unwrap() {
            lz = 255;
        }
        if in_pin_rz.is_low().unwrap() {
            rz = 255;
        }

        let xinput_report = XinputControlReport {
            // byte zero
            thumb_click_right: in_pin_r3.is_low().unwrap(),
            thumb_click_left: in_pin_l3.is_low().unwrap(),
            button_view: in_pin_overview.is_low().unwrap(),
            button_menu: in_pin_menu.is_low().unwrap(),
            dpad_right: in_pin_d_right.is_low().unwrap(),
            dpad_left: in_pin_d_left.is_low().unwrap(),
            dpad_down: in_pin_d_down.is_low().unwrap(),
            dpad_up: in_pin_d_up.is_low().unwrap(),
            // byte one
            button_y: in_pin_y.is_low().unwrap(),
            button_x: in_pin_x.is_low().unwrap(),
            button_b: in_pin_b.is_low().unwrap(),
            button_a: in_pin_a.is_low().unwrap(),
            // #[packed_field(bits = "12")]
            // pub reserved: bool,
            xbox_button: false,
            shoulder_right: in_pin_rt.is_low().unwrap(),
            shoulder_left: in_pin_lt.is_low().unwrap(),
            // others
            trigger_left: lz,
            trigger_right: rz,
            js_left_x: lx,
            js_left_y: ly,
            js_right_x: rx,
            js_right_y: ry,
        };
        
        push_input(&xinput_report);

        unsafe {
            let usb_dev = USB_DEVICE.as_mut().unwrap();
            let usb_xinput = USB_XINPUT.as_mut().unwrap();
            usb_dev.poll(&mut [usb_xinput]);
        }
    }
    
    // Stop free-running mode (the returned `adc` can be reused for future captures)
    // let _adc = adc_fifo.stop();
}

/// Submit a new gamepad inpuit report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_input(report: &XinputControlReport) -> () {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a XINPUT report
        USB_XINPUT.as_mut().map(|xinput| xinput.write_control(&xinput::report(report)))
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
    let usb_xinput = USB_XINPUT.as_mut().unwrap();
    usb_dev.poll(&mut [usb_xinput]);
}

// End of file
