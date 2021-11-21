use serialport::{SerialPortInfo, SerialPortType, UsbPortInfo};
use crate::programmer::{ProgrammerError, ProgrammerResult};

pub fn detect_dsrp() -> ProgrammerResult<String> {
    const USB_VID_CH340: u16 = 0x1A86;
    const USB_PID_CH340: u16 = 0x7523;

    println!("Detecting port...");

    let ports = serialport::available_ports()?;

    let mut port_name = None;

    for port in &ports {
        if let SerialPortInfo {
            port_type: SerialPortType::UsbPort(UsbPortInfo {
                vid: USB_VID_CH340,
                pid: USB_PID_CH340,
                ..
            }),
            ..
        } = port {
            if port_name.is_some() {
                return Err(ProgrammerError::PortDetect(
                    "Detected multiple CH340 USB <-> UART devices, please specify \
                    port explicitly or disonnect unrelated devices".to_owned()
                ));
            }

            port_name = Some(port.port_name.clone());
        }
    }

    port_name.ok_or_else(|| {
        ProgrammerError::PortDetect("No suitable serial ports found".to_owned())
    })
}
