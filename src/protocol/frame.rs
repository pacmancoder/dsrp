//! #### DSRP-FP transport protocol (DSRP-FP)
//! This is simple transport protocol, similar to UDP - it
//! guarantees data frame integrity, but it does not guarantee
//! that frame indeed will be ever delivered. However, in contrary to
//! UDP, DSRP does guarantee that order will be preserved (because this
//! protocol is usually used for point-to-point serial connection)
//!
//! ##### Header
//! Header contain only u16 LE frame length. It should be equal to the
//! sum of an actual packet data length plus header and trailer size
//!
//! ##### Trailer
//! Trailer contain u16 LE with CRC16 of a packet (header + data) calculated
//! using "Kermit" favour of crc16.
use crc::{CRC_16_KERMIT, Crc};
use std::io::{Read, Write};
use bytes::{BufMut, Buf};

const CRC: Crc<u16> = crc::Crc::<u16>::new(&CRC_16_KERMIT);

const HEADER_SIZE: usize = std::mem::size_of::<u16>();
const TRAILER_SIZE: usize = std::mem::size_of::<u16>();

pub struct SerialTransport<T> {
    inner: T,
}

impl<T: Read + Write> SerialTransport<T> {
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    pub fn downgrade(self) -> T {
        self.inner
    }
}

pub trait Transport {
    fn send_frame(&mut self, frame: &[u8]) -> std::io::Result<()>;
    fn receive_frame(&mut self) -> std::io::Result<Vec<u8>>;
}

impl<T: Read + Write> Transport for SerialTransport<T> {
    fn send_frame(&mut self, frame: &[u8]) -> std::io::Result<()> {
        let packet_len = frame.len() + HEADER_SIZE + TRAILER_SIZE;
        if packet_len > u16::MAX as usize {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("DSRP transport protocol does not support frames bigger than 64 KiB"))
            );
        }

        let mut header = [0u8; HEADER_SIZE];
        {
            let mut header = &mut header[..];
            header.put_u16_le(packet_len as u16);
        }
        let mut trailer = [0u8; TRAILER_SIZE];
        {
            let mut trailer = &mut trailer[..];
            let mut digest = CRC.digest();
            digest.update(&header);
            digest.update(frame);
            let crc = digest.finalize();
            trailer.put_u16_le(crc)
        }

        self.inner.write_all(&header)?;
        self.inner.write_all(frame)?;
        self.inner.write_all(&trailer)?;

        Ok(())
    }

    fn receive_frame(&mut self) -> std::io::Result<Vec<u8>> {
        let mut digest = CRC.digest();

        let size = {
            let mut header = [0u8; HEADER_SIZE];
            self.inner.read_exact(&mut header)?;
            digest.update(&header);
            let mut header = &header[..];
            let size = header.get_u16_le();
            size as usize
        };

        let data = {
            let data_size = size - HEADER_SIZE - TRAILER_SIZE;
            let mut buffer = vec![0; data_size];
            self.inner.read_exact(&mut buffer)?;
            digest.update(&buffer);
            buffer
        };

        let expected_crc = {
            let mut trailer = [0u8; TRAILER_SIZE];
            self.inner.read_exact(&mut trailer)?;
            let mut trailer = &trailer[..];
            trailer.get_u16_le()
        };

        let actual_crc = digest.finalize();

        if actual_crc != expected_crc {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!("DSRP transport protocol detected invalid CRC"))
            );
        }

        Ok(data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use expect_test::expect;

    struct MockPort {
        tx: pipe::PipeWriter,
        rx: pipe::PipeReader,
        tx_dump: Vec<u8>,
        rx_dump: Vec<u8>,
    }

    impl MockPort {
        pub fn pair() -> (Self, Self) {
            let (host_rx, device_tx) = pipe::pipe();
            let (device_rx, host_tx) = pipe::pipe();

            let host = Self {
                tx: host_tx,
                rx: host_rx,
                tx_dump: vec![],
                rx_dump: vec![],
            };

            let device = Self {
                tx: device_tx,
                rx: device_rx,
                tx_dump: vec![],
                rx_dump: vec![],
            };

            (host, device)
        }

        pub fn tx_dump(&self) -> &[u8] {
            &self.tx_dump
        }

        pub fn rx_dump(&self) -> &[u8] {
            &self.rx_dump
        }
    }

    impl Read for MockPort {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            let bytes_read = self.rx.read(buf)?;
            self.rx_dump.extend_from_slice(&buf[0..bytes_read]);
            Ok(bytes_read)
        }
    }

    impl Write for MockPort {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            let bytes_written = self.tx.write(buf)?;
            self.tx_dump.extend_from_slice(&buf[0..bytes_written]);
            Ok(bytes_written)
        }

        fn flush(&mut self) -> std::io::Result<()> {
            self.tx.flush()
        }
    }

    #[test]
    fn smoke() {
        let (host_port, device_port) = MockPort::pair();

        let device_thread = std::thread::spawn(move || {
            let mut device_transport = SerialTransport::new(device_port);

            let mut data = device_transport.receive_frame().unwrap();
            data.reverse();
            device_transport.send_frame(&data).unwrap();
            device_transport.send_frame(b"HI!").unwrap();
        });

        let mut host_transport = SerialTransport::new(host_port);
        host_transport.send_frame(&[0x12, 0x34, 0x56, 0x78]).unwrap();
        let received_data = host_transport.receive_frame().unwrap();
        let received_hi = host_transport.receive_frame().unwrap();

        expect![[r#"[78, 56, 34, 12]"#]]
            .assert_eq(&format!("{:02X?}", received_data));

        expect![[r#"HI!"#]]
            .assert_eq(&String::from_utf8(received_hi).unwrap());

        device_thread.join().unwrap();

        let host_port = host_transport.downgrade();

        expect![[r#"[08, 00, 12, 34, 56, 78, A8, 46]"#]]
            .assert_eq(&format!("{:02X?}", host_port.tx_dump()));
        expect![[r#"[08, 00, 78, 56, 34, 12, AE, 29, 07, 00, 48, 49, 21, 9D, 51]"#]]
            .assert_eq(&format!("{:02X?}", host_port.rx_dump()));
    }
}
