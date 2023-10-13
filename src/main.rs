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
pub mod xinput {
    use usb_device::class_prelude::*;
    use usb_device::Result;
    use usb_device::UsbDirection;
    use usb_device::endpoint::EndpointAddress;
    use packed_struct::prelude::*;

    // just copied from a controller with Xinput support
    pub const USB_XINPUT_VID: u16 = 0x045e;
    pub const USB_XINPUT_PID: u16 = 0x028e;
    const USB_CLASS_VENDOR: u8 = 0xff;
    const USB_SUBCLASS_VENDOR: u8 = 0xff;
    const USB_PROTOCOL_VENDOR: u8 = 0xff;
    const USB_DEVICE_RELEASE: u16 = 0x0114;

    const XINPUT_DESC_DESCTYPE_STANDARD: u8 = 0x21; // a common descriptor type for all xinput interfaces
    const XINPUT_IFACE_SUBCLASS_STANDARD: u8 = 0x5D;
    const XINPUT_IFACE_PROTO_IF0: u8 = 0x01;

    const XINPUT_EP_MAX_PACKET_SIZE: u16 = 0x20;
    const XINPUT_RW_BUFFER_SIZE: usize = XINPUT_EP_MAX_PACKET_SIZE as usize;

    const REQ_GET_REPORT: u8 = 0x01;
    const REQ_GET_IDLE: u8 = 0x02;
    const REQ_GET_PROTOCOL: u8 = 0x03;
    const REQ_SET_REPORT: u8 = 0x09;
    const REQ_SET_IDLE: u8 = 0x0a;
    const REQ_SET_PROTOCOL: u8 = 0x0b;

    const XINPUT_DESC_IF0: &[u8] = &[
        // for control interface
        0x00, 0x01, 0x01, 0x25, // ???
        0x81, // bEndpointAddress (IN, 1)
        0x14, // bMaxDataSize
        0x00, 0x00, 0x00, 0x00, 0x13, // ???
        0x01, // bEndpointAddress (OUT, 1)
        0x08, // bMaxDataSize
        0x00, 0x00, // ???
    ];

    /// Store the input states of the controller
    #[derive(PackedStruct, Default, Debug, PartialEq)]
    #[packed_struct(endian = "lsb", bit_numbering = "msb0")]
    pub struct XinputControlReport {
        // byte zero
        #[packed_field(bits = "0")]
        pub thumb_click_right: bool,
        #[packed_field(bits = "1")]
        pub thumb_click_left: bool,
        #[packed_field(bits = "2")]
        pub button_view: bool,
        #[packed_field(bits = "3")]
        pub button_menu: bool,
        #[packed_field(bits = "4")]
        pub dpad_right: bool,
        #[packed_field(bits = "5")]
        pub dpad_left: bool,
        #[packed_field(bits = "6")]
        pub dpad_down: bool,
        #[packed_field(bits = "7")]
        pub dpad_up: bool,
        // byte one
        #[packed_field(bits = "8")]
        pub button_y: bool,
        #[packed_field(bits = "9")]
        pub button_x: bool,
        #[packed_field(bits = "10")]
        pub button_b: bool,
        #[packed_field(bits = "11")]
        pub button_a: bool,
        // #[packed_field(bits = "12")]
        // pub reserved: bool,
        #[packed_field(bits = "13")]
        pub xbox_button: bool,
        #[packed_field(bits = "14")]
        pub shoulder_right: bool,
        #[packed_field(bits = "15")]
        pub shoulder_left: bool,
        // others
        #[packed_field(bytes = "2")]
        pub trigger_left: u8,
        #[packed_field(bytes = "3")]
        pub trigger_right: u8,
        #[packed_field(bytes = "4..=5")]
        pub js_left_x: i16,
        #[packed_field(bytes = "6..=7")]
        pub js_left_y: i16,
        #[packed_field(bytes = "8..=9")]
        pub js_right_x: i16,
        #[packed_field(bytes = "10..=11")]
        pub js_right_y: i16,
    }

    pub fn report(xinput_report: &XinputControlReport) -> [u8; 20] {
        let packed = xinput_report.pack().unwrap();

        [
            0x00, // packet type id
            0x14, // packet length (20)
            packed[0],
            packed[1],
            packed[2],
            packed[3],
            packed[4],
            packed[5],
            packed[6],
            packed[7],
            packed[8],
            packed[9],
            packed[10],
            packed[11],
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    }

    pub struct XINPUTClass<'a, B: UsbBus> {
        report_if: InterfaceNumber,
        report_ep_in: EndpointIn<'a, B>,
        report_ep_out: EndpointOut<'a, B>,
    }

    impl<B: UsbBus> XINPUTClass<'_, B> {
        /// Creates a new XINPUTClass with the provided UsbBus and max_packet_size in bytes. For
        /// full-speed devices, max_packet_size has to be one of 8, 16, 32 or 64.
        pub fn new(alloc: &UsbBusAllocator<B>) -> XINPUTClass<'_, B> {
            XINPUTClass {
                report_if: alloc.interface(),

                report_ep_in: alloc.alloc(Some(EndpointAddress::from_parts(0x01, UsbDirection::In)),
                                EndpointType::Interrupt, XINPUT_EP_MAX_PACKET_SIZE, 4).expect("alloc_ep failed"), // (capacity, poll_interval)
                
                report_ep_out: alloc.alloc(Some(EndpointAddress::from_parts(0x01, UsbDirection::Out)),
                                EndpointType::Interrupt, XINPUT_EP_MAX_PACKET_SIZE, 8).expect("alloc_ep failed"), // (capacity, poll_interval)
            }
        }

        pub fn write_control(&mut self, data: &[u8]) {
            self.report_ep_in.write(data).ok();
        }
    }

    impl<B: UsbBus> UsbClass<B> for XINPUTClass<'_, B> {
        fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
            writer.interface_alt(
                self.report_if,
                0x00,
                USB_CLASS_VENDOR,
                XINPUT_IFACE_SUBCLASS_STANDARD,
                XINPUT_IFACE_PROTO_IF0,
                None,
            )?;

            writer.write(
                XINPUT_DESC_DESCTYPE_STANDARD,
                XINPUT_DESC_IF0,
            )?;

            writer.endpoint(&self.report_ep_in)?;
            writer.endpoint(&self.report_ep_out)?;

            Ok(())
        }

        fn control_in(&mut self, xfer: ControlIn<B>) {
            let req = xfer.request();

            if req.request_type == control::RequestType::Vendor {
                match (req.recipient, req.request) {
                    (control::Recipient::Interface, control::Request::CLEAR_FEATURE) => { //CLEAR_FEATURE=>
                                                                                          //0x01
                        if req.value == 0x100 && req.index == 0x00 { // see
                                                                     // linux/drivers/input/joystick/xpad.c#L1734
                                                                     // usb_control_msg_recv
                            xfer.accept_with_static(&[0 as u8; 20]).ok();
                            return;
                        }
                    }
                    _ => {
                        return;
                    }
                };
            }
        }

        /*
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
        */
    }
}

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
            js_left_x: -1500,
            js_left_y: 0,
            js_right_x: 1500,
            js_right_y: 0,
        };
        
        push_input(&xinput_report);
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
