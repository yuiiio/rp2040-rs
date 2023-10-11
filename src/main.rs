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

#[allow(unused)]
pub mod hid {
    use usb_device::class_prelude::*;
    use usb_device::Result;
    use packed_struct::prelude::*;

    pub const USB_CLASS_HID: u8 = 0x03;

    const USB_SUBCLASS_NONE: u8 = 0x00;
    const USB_SUBCLASS_BOOT: u8 = 0x01;

    const USB_INTERFACE_NONE: u8 = 0x00;
    const USB_INTERFACE_KEYBOARD: u8 = 0x01;
    const USB_INTERFACE_MOUSE: u8 = 0x02;

    const REQ_GET_REPORT: u8 = 0x01;
    const REQ_GET_IDLE: u8 = 0x02;
    const REQ_GET_PROTOCOL: u8 = 0x03;
    const REQ_SET_REPORT: u8 = 0x09;
    const REQ_SET_IDLE: u8 = 0x0a;
    const REQ_SET_PROTOCOL: u8 = 0x0b;

    #[derive(Clone, Copy, Debug, Eq, PartialEq, Default, PackedStruct)]
    #[packed_struct(endian = "lsb", size_bytes = "8", bit_numbering = "msb0")]
    pub struct JoystickReport {
        #[packed_field]
        pub lx: u8,
        #[packed_field]
        pub ly: u8,
        #[packed_field]
        pub rx: u8,
        #[packed_field]
        pub ry: u8,
        #[packed_field]
        pub lz: u8,
        #[packed_field]
        pub rz: u8,
        #[packed_field]
        pub buttons1: u8,

        // this 4 bits field pack to one byte,
        // so reversed array order compared pre descriptor.
        #[packed_field]
        pub hat_switch: [bool; 4],
        #[packed_field]
        pub buttons2: [bool; 4],
    }

    const REPORT_DESCR: &[u8] = &[
        0x05, 0x01, // Usage Page (Generic Desktop)         5,   1
        0x09, 0x04, // Usage (Joystick)                     9,   4
        0xa1, 0x01, // Collection (Application)             161, 1

        0x09, 0x01, //   Usage Page (Pointer)               9,   1
        0xa1, 0x00, //   Collection (Physical)              161, 0
        0x09, 0x30, //     Usage (LX)                        9,   48
        0x09, 0x31, //     Usage (LY)                        9,   49
        0x09, 0x33, //     Usage (RX)                        9,   51
        0x09, 0x34, //     Usage (RY)                        9,   52
        0x09, 0x32, //     Usage (LZ)                        9,   50
        0x09, 0x35, //     Usage (RZ)                        9,   53
        0x15, 0x00, //     Logical Minimum (0)              21,  0
        0x25, 0xff, //     Logical Maximum (255)            37,  255
        0x75, 0x08, //     Report Size (8)                  117, 8
        0x95, 0x06, //     Report count (6)                 149, 6,
        0x81, 0x02, //     Input (Data, Variable, Absolute) 129, 2,
        0xc0,       //   End Collection                     192,

        0x05, 0x09, //   Usage Page (Button)                5,   9,
        0x19, 0x01, //   Usage Minimum (1)                  25,  1,
        0x29, 0x0c, //   Usage Maximum (12)                 41,  12,
        0x15, 0x00, //   Logical Minimum (0)                21,  0
        0x25, 0x01, //   Logical Maximum (1)                37,  1,
        0x75, 0x01, //   Report Size (1)                    117, 1,
        0x95, 0x0c, //   Report Count (12)                   149, 12,
        0x81, 0x02, //   Input (Data, Variable, Absolute)   129, 2,


        // ^ 8 + 4 bits 
        // V 4 bits
        // repack to 2 bytes. {8buttons}, {4buttons, HatSwitch}

        /* Hat Switch */
        0x05, 0x01,							/*   USAGE_PAGE (Generic Desktop) */
        0x09, 0x39,							/*   USAGE (Hat switch) */
        0x15, 0x01,							/*   LOGICAL_MINIMUM (1) */
        0x25, 0x08,							/*   LOGICAL_MAXIMUM (8) */
        0x95, 0x01,							/*   REPORT_COUNT (1) */
        0x75, 0x04,							/*   REPORT_SIZE (4) */
        0x81, 0x02,							/*   INPUT (Data,Var,Abs) */

        0xc0,       // End Collection                       192
    ];

    pub fn report(report: JoystickReport) -> [u8; 8] {
        report.pack().unwrap()
    }

    pub struct HIDClass<'a, B: UsbBus> {
        report_if: InterfaceNumber,
        report_ep: EndpointIn<'a, B>,
    }

    impl<B: UsbBus> HIDClass<'_, B> {
        /// Creates a new HIDClass with the provided UsbBus and max_packet_size in bytes. For
        /// full-speed devices, max_packet_size has to be one of 8, 16, 32 or 64.
        pub fn new(alloc: &UsbBusAllocator<B>) -> HIDClass<'_, B> {
            HIDClass {
                report_if: alloc.interface(),
                report_ep: alloc.interrupt(8, 10),
            }
        }

        pub fn write(&mut self, data: &[u8]) {
            self.report_ep.write(data).ok();
        }
    }

    impl<B: UsbBus> UsbClass<B> for HIDClass<'_, B> {
        fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
            writer.interface(
                self.report_if,
                USB_CLASS_HID,
                USB_SUBCLASS_NONE,
                USB_INTERFACE_NONE,
            )?;

            let descr_len: u16 = REPORT_DESCR.len() as u16;
            writer.write(
                0x21,
                &[
                    0x01,                   // bcdHID
                    0x01,                   // bcdHID
                    0x00,                   // bContryCode
                    0x01,                   // bNumDescriptors
                    0x22,                   // bDescriptorType
                    descr_len as u8,        // wDescriptorLength
                    (descr_len >> 8) as u8, // wDescriptorLength
                ],
            )?;

            writer.endpoint(&self.report_ep)?;

            Ok(())
        }

        fn control_in(&mut self, xfer: ControlIn<B>) {
            let req = xfer.request();

            if req.request_type == control::RequestType::Standard {
                match (req.recipient, req.request) {
                    (control::Recipient::Interface, control::Request::GET_DESCRIPTOR) => {
                        let (dtype, _index) = req.descriptor_type_index();
                        if dtype == 0x21 {
                            // HID descriptor
                            cortex_m::asm::bkpt();
                            let descr_len: u16 = REPORT_DESCR.len() as u16;

                            // HID descriptor
                            let descr = &[
                                0x09,                   // length
                                0x21,                   // descriptor type
                                0x01,                   // bcdHID
                                0x01,                   // bcdHID
                                0x00,                   // bCountryCode
                                0x01,                   // bNumDescriptors
                                0x22,                   // bDescriptorType
                                descr_len as u8,        // wDescriptorLength
                                (descr_len >> 8) as u8, // wDescriptorLength
                            ];

                            xfer.accept_with(descr).ok();
                            return;
                        } else if dtype == 0x22 {
                            // Report descriptor
                            xfer.accept_with(REPORT_DESCR).ok();
                            return;
                        }
                    }
                    _ => {
                        return;
                    }
                };
            }

            if !(req.request_type == control::RequestType::Class
                && req.recipient == control::Recipient::Interface
                && req.index == u8::from(self.report_if) as u16)
            {
                return;
            }

            match req.request {
                REQ_GET_REPORT => {
                    // USB host requests for report
                    // I'm not sure what should we do here, so just send empty report
                    xfer.accept_with(&[0 as u8; 8]).ok();
                }
                _ => {
                    xfer.reject().ok();
                }
            }
        }

        fn control_out(&mut self, xfer: ControlOut<B>) {
            let req = xfer.request();

            if !(req.request_type == control::RequestType::Class
                && req.recipient == control::Recipient::Interface
                && req.index == u8::from(self.report_if) as u16)
            {
                return;
            }

            xfer.reject().ok();
        }
    }
}

use hid::{HIDClass, JoystickReport};

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

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

// DPAD 1~8 axis clock wise from up.
const DPAD_UP: [bool; 4]= [false, false, false, true];
const DPAD_DOWN: [bool; 4]= [false, true, false, true];
const DPAD_RIGHT: [bool; 4]= [false, false, true, true];
const DPAD_LEFT: [bool; 4]= [false, true, true, true];

const DPAD_DOWN_LEFT: [bool; 4] = [false, true, true, false];
const DPAD_DOWN_RIGHT: [bool; 4] = [false, true, false, false];
const DPAD_UP_RIGHT: [bool; 4] = [false, false, true, false];
const DPAD_UP_LEFT: [bool; 4] = [true, false, false, false];

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

    let usb_hid = HIDClass::new(bus_ref);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_HID = Some(usb_hid);
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
    let in_pin_r3 = pins.gpio23.into_pull_up_input();
    let in_pin_l3 = pins.gpio24.into_pull_up_input();
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

    // Move the cursor up and down every 200ms
    loop {
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

        // 12 bit to 9bit to 8bit
        // norm is 127
        let adc_0: u16 = adc_result_0 >> 3;
        let adc_1: u16 = adc_result_1 >> 3;
        let adc_2: u16 = adc_result_2 >> 3;
        let adc_3: u16 = adc_result_3 >> 3;

        // clamp
        let lx: u8 = (if adc_0 >> 7 == 0 { 0 } else { if adc_0 & 0b110000000 == 0b110000000 { 255 } else { adc_0 - 127 } }) as u8;
        let ly: u8 = (if adc_1 >> 7 == 0 { 0 } else { if adc_1 & 0b110000000 == 0b110000000 { 255 } else { adc_1 - 127 } }) as u8;
        let rx: u8 = (if adc_2 >> 7 == 0 { 0 } else { if adc_2 & 0b110000000 == 0b110000000 { 255 } else { adc_2 - 127 } }) as u8;
        let ry: u8 = (if adc_3 >> 7 == 0 { 0 } else { if adc_3 & 0b110000000 == 0b110000000 { 255 } else { adc_3 - 127 } }) as u8;

        let (mut lz, mut rz): (u8, u8) = (0, 0);
        if in_pin_lz.is_low().unwrap() {
            lz = 255;
        }
        if in_pin_rz.is_low().unwrap() {
            rz = 255;
        }

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
        if in_pin_lt.is_low().unwrap() {
            buttons1 |= 0b00010000;
        }
        if in_pin_rt.is_low().unwrap() {
            buttons1 |= 0b00100000;
        }
        if in_pin_overview.is_low().unwrap() {
            buttons1 |= 0b01000000;
        }
        if in_pin_menu.is_low().unwrap() {
            buttons1 |= 0b10000000;
        }

        let mut buttons2  = [false; 4];
        if in_pin_l3.is_low().unwrap() {
            buttons2[1] = true;
        }
        if in_pin_r3.is_low().unwrap() {
            buttons2[2] = true;
        }

        // up and down, left and right should not press at same time
        // hat_switch(dpad) expect only 8 axis info.
        let hat_switch: [bool; 4] =
            if in_pin_d_up.is_low().unwrap() {
                if in_pin_d_right.is_low().unwrap() {
                    DPAD_UP_RIGHT
                } else {
                    if in_pin_d_left.is_low().unwrap() {
                        DPAD_UP_LEFT
                    } else {
                        DPAD_UP
                    }
                }
            } else {
                if in_pin_d_down.is_low().unwrap() {
                    if in_pin_d_right.is_low().unwrap() {
                        DPAD_DOWN_RIGHT
                    } else {
                        if in_pin_d_left.is_low().unwrap() {
                            DPAD_DOWN_LEFT
                        } else {
                            DPAD_DOWN
                        }
                    }
                } else {
                    if in_pin_d_right.is_low().unwrap() {
                        DPAD_RIGHT
                    } else {
                        if in_pin_d_left.is_low().unwrap() {
                            DPAD_LEFT
                        } else {
                            [false; 4]
                        }
                    }
                }
            };

        let report = JoystickReport {
            lx: lx,
            ly: ly,
            rx: rx,
            ry: ry,
            lz: lz, // 0~255 expect analog trigger but, rp2040 has only 4 analogin so
            rz: rz, // use binary value 0, 1 and map to 0 , 255.
            buttons1: buttons1, // high [menu, overview, RT, LT, Y, X, B, A] low
            buttons2: buttons2, // high [?, R3, L3, ?] low
            hat_switch: hat_switch, // see DPAD_*
        };

        push_input(report);
    }
    
    // Stop free-running mode (the returned `adc` can be reused for future captures)
    // let _adc = adc_fifo.stop();
}

/// Submit a new gamepad inpuit report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_input(report: JoystickReport) -> () {
    critical_section::with(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.write(&hid::report(report)))
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
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);
}

// End of file
