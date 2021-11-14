//! Arduino Dead Simple Rom Programmer
use crate::programmer::{Programmer, ProgrammerResult, ProgressHandler, ProgrammerError};
use serialport::SerialPort;
use bincode::{Decode, Encode, de::{Decode, Decoder}, enc::{Encode, Encoder}, error::{DecodeError, EncodeError}};
use super::ProgressUpdate;
use std::time::Duration;
use from_variants::FromVariants;

mod protocol;
mod wire_types;

use wire_types::{ErrorCode, VariableBuffer};

const BAUD_RATE: u32 = 115200;
const PACKET_MARKER: u8 = 0xAD;

mod packet_body {
    use super::*;

    macro_rules! impl_packet_id {
        ($name:ty = $value:literal) => {
            impl $name {
                pub const fn packet_id() -> u8 {
                    $value
                }
            }
        };
    }

    #[derive(Debug, Default, Encode, Decode)]
    pub struct InitRequest {}

    #[derive(Debug, Encode, Decode)]
    pub struct InitResponse {
        /// Max size of a single read/write transaction
        pub max_block_size: u16,
    }

    #[derive(Debug, Encode, Decode)]
    pub struct ReadReqeust {
        pub offset: u32,
        pub bytes_to_read: u16,
    }

    #[derive(Debug, Encode, Decode)]
    pub struct ReadResponse {
        pub buffer: VariableBuffer,
    }

    #[derive(Debug, Encode, Decode)]
    pub struct WriteReqeust {
        pub offset: u32,
        pub buffer: VariableBuffer,
    }

    #[derive(Debug, Default, Encode, Decode)]
    pub struct WriteResponse {}

    #[derive(Debug, Default, Encode, Decode)]
    pub struct EraseRequest {}

    #[derive(Debug, Default, Encode, Decode)]
    pub struct EraseResponse {}

    #[derive(Debug, Encode, Decode)]
    pub struct ErrorResponse {
        pub error_code: ErrorCode,
    }

    impl_packet_id!(InitRequest = 0x00);
    impl_packet_id!(ReadReqeust = 0x01);
    impl_packet_id!(WriteReqeust = 0x02);
    impl_packet_id!(EraseRequest = 0x03);

    impl_packet_id!(InitResponse = 0x10);
    impl_packet_id!(ReadResponse = 0x11);
    impl_packet_id!(WriteResponse = 0x12);
    impl_packet_id!(EraseResponse = 0x13);

    impl_packet_id!(ErrorResponse = 0xFF);
}

macro_rules! adsrp_packet_body {
    ($($variant:ident),+) => {
        #[derive(FromVariants, Debug)]
        enum AdsrpPacketBody {
            $($variant(packet_body::$variant),)+
        }

        impl Encode for AdsrpPacketBody {
            fn encode<E: Encoder>(&self, mut encoder: E) -> Result<(), EncodeError> {
                PACKET_MARKER.encode(&mut encoder)?;

                match self {
                    $(Self::$variant(p) => {
                        packet_body::$variant::packet_id().encode(&mut encoder)?;
                        p.encode(&mut encoder)
                    },)+
                }
            }
        }

        impl Decode for AdsrpPacketBody {
            fn decode<D: Decoder>(mut decoder: D) -> Result<Self, DecodeError> {
                let packet_marker = u8::decode(&mut decoder)?;
                if packet_marker != PACKET_MARKER {
                    return Err(DecodeError::UnexpectedVariant {
                        type_name: "PACKET_MARKER",
                        min: PACKET_MARKER as u32,
                        max: PACKET_MARKER as u32,
                        found: packet_marker as u32,
                    })
                }

                let command_id = u8::decode(&mut decoder)?;

                match command_id {
                    $(value if value == packet_body::$variant::packet_id() => {
                        Ok(packet_body::$variant::decode(&mut decoder)?.into())
                    })+
                    value => Err(DecodeError::UnexpectedVariant {
                        type_name: std::any::type_name::<Self>(),
                        min: 0xFFFFFFFF,
                        max: 0xFFFFFFFF,
                        found: value as u32,
                    })
                }
            }
        }
    };
}

adsrp_packet_body! {
    InitRequest,
    InitResponse,
    ReadReqeust,
    ReadResponse,
    WriteReqeust,
    WriteResponse,
    EraseRequest,
    EraseResponse,
    ErrorResponse
}

#[derive(Debug, Encode, Decode)]
struct AdsrpPacket {
    body: AdsrpPacketBody,
    crc: u16,
}

#[cfg(test)]
mod tests {
    use super::*;
    use expect_test::expect;

    #[test]
    fn check_serialization() {
        let packet = AdsrpPacket {
            body: packet_body::InitRequest { }.into(),
            crc: 0x1234,
        };

        let encoded = bincode::encode_to_vec(packet, wire_encoding_config()).unwrap();

        expect![[r#"
            [
                173,
                16,
                0,
                2,
                52,
                18,
            ]
        "#]].assert_debug_eq(&encoded);
    }
}

fn wire_encoding_config() -> impl bincode::config::Config {
    bincode::config::Configuration::standard()
            .with_fixed_int_encoding()
            .skip_fixed_array_length()
}

fn execute_untyped(
    request_body: AdsrpPacketBody,
    port: &mut Box<dyn SerialPort>
) -> ProgrammerResult<AdsrpPacketBody> {
    let config = wire_encoding_config();
    let encoded_body = bincode::encode_to_vec(&request_body, config)
        .map_err(|e| ProgrammerError::PacketEncode(e))?;

    let packet = AdsrpPacket {
        body: request_body.into(),
        crc: crc16(&encoded_body),
    };

    let encoded = bincode::encode_to_vec(&packet, config)
        .map_err(|e| ProgrammerError::PacketEncode(e))?;

    port.write_all(&encoded)?;

    let decoded: AdsrpPacket = bincode::decode_from_std_read(port, config)
        .map_err(|e| ProgrammerError::PacketDecode(e))?;

    // Performing encode to calculate crc is not optimal, but allows to skip size field
    // from the packet
    let encoded_response_body = bincode::encode_to_vec(&decoded.body, config)
        .map_err(|e| ProgrammerError::PacketEncode(e))?;
    let actual_crc = crc16(&encoded_response_body);

    if actual_crc != decoded.crc {
        return Err(ProgrammerError::InvalidChecksum);
    }

    // Detect programmer internal error
    if let AdsrpPacketBody::ErrorResponse(
        packet_body::ErrorResponse { error_code }
    ) = &decoded.body {
        return Err(ProgrammerError::Internal(format!("{}", error_code)));
    }

    Ok(decoded.body)
}

trait Command {
    type RequestBody: Encode + Into<AdsrpPacketBody>;
    type ResponseBody: Decode + Into<AdsrpPacketBody>;

    fn execute(
        request_body: Self::RequestBody,
        port: &mut Box<dyn SerialPort>
    ) -> ProgrammerResult<Self::ResponseBody>;
}

mod command {
    use super::*;

    macro_rules! define_command {
        ($name:ident, $req:ident, $rsp:ident) => {
            pub struct $name;

            impl Command for $name {
                type RequestBody = packet_body::$req;
                type ResponseBody = packet_body::$rsp;

                fn execute(
                    request_body: Self::RequestBody,
                    port: &mut Box<dyn SerialPort>
                ) -> ProgrammerResult<Self::ResponseBody> {
                    if let AdsrpPacketBody::$rsp(body) = execute_untyped(request_body.into(), port)? {
                        Ok(body)
                    } else {
                        Err(ProgrammerError::InvalidResponseType)
                    }
                }
            }
        };
    }

    define_command!(Init, InitRequest, InitResponse);
    define_command!(Read, ReadReqeust, ReadResponse);
    define_command!(Write, WriteReqeust, WriteResponse);
    define_command!(Erase, EraseRequest, EraseResponse);
}

pub struct AdsrpProgrammer {
    max_block_size: u16,
    port: Box<dyn SerialPort>,
}

impl Programmer for AdsrpProgrammer {
    fn read(&mut self, mut offset: u32, mut buffer: &mut [u8], progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        progress.on_update(ProgressUpdate::Started);

        let initial_offset = offset;
        while !buffer.is_empty() {
            let (chunk, remains) = buffer.split_at_mut(buffer.len().min(self.max_block_size as usize));
            let request = packet_body::ReadReqeust{
                offset: offset,
                bytes_to_read: self.max_block_size,
            };

            let response = command::Read::execute(request, &mut self.port)?;

            let src_buffer = response.buffer.as_ref();
            chunk.copy_from_slice(&src_buffer[0..chunk.len()]);

            buffer = remains;
            offset += chunk.len() as u32;
            progress.on_update(
                ProgressUpdate::ProcessedBytesChanged((offset - initial_offset) as usize)
            );
        }

        progress.on_update(ProgressUpdate::Finished);

        Ok(())
    }

    fn write(&mut self, mut offset: u32, mut buffer: &[u8], progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        progress.on_update(ProgressUpdate::Started);

        let initial_offset = offset;
        while !buffer.is_empty() {
            let (chunk, remains) = buffer.split_at(buffer.len().min(self.max_block_size as usize));
            let request = packet_body::WriteReqeust {
                offset,
                buffer: chunk.to_vec().into(),
            };

            let _response = command::Write::execute(request, &mut self.port)?;

            buffer = remains;
            offset += chunk.len() as u32;
            progress.on_update(
                ProgressUpdate::ProcessedBytesChanged((offset - initial_offset) as usize)
            );
        }

        progress.on_update(ProgressUpdate::Finished);

        Ok(())
    }

    fn erase(&mut self, progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        progress.on_update(ProgressUpdate::Started);

        let request = packet_body::EraseRequest::default();

        let _response = command::Erase::execute(request, &mut self.port)?;

        progress.on_update(ProgressUpdate::Finished);

        Ok(())
    }
}

impl AdsrpProgrammer {
    pub fn from_port(port_name: String) -> ProgrammerResult<Self> {
        println!("Initializing adsrp programmer");

        let mut port = serialport::new(port_name, BAUD_RATE)
            .timeout(Duration::from_secs(3))
            .open()?;

        let response = command::Init::execute(
            packet_body::InitRequest::default(),
            &mut port
        )?;

        println!("Adsrp programmer initialization succeeded");
        println!(" - Block size: {}", bytesize::to_string(response.max_block_size as u64, true));

        Ok(Self {
            port,
            max_block_size: response.max_block_size,
        })
    }
}


fn crc16(data: &[u8]) -> u16 {
    let crc = crc::Crc::<u16>::new(&crc::CRC_16_KERMIT);
    let mut digest = crc.digest();
    digest.update(data);
    digest.finalize()
}
