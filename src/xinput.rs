#[allow(unused)]
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
