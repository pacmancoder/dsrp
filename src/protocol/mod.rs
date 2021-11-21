use thiserror::Error;
use bincode::error::{DecodeError, EncodeError};

mod frame;
mod packet;

pub use frame::{Transport, SerialTransport};
use packet::{Request, Response};

pub use packet::{ManufacturerId, ChipId, Error as DeviceError};

pub type ProtocolResult<T> = Result<T, ProtocolError>;

#[derive(Debug, Error)]
pub enum ProtocolError {
    #[error(transparent)]
    Io {
        #[from]
        source: std::io::Error,
    },
    #[error("Failed to encode frame")]
    PacketEncodeError {
        #[from]
        source: EncodeError,
    },
    #[error("Failed to decode frame")]
    PacketDecodeError {
        #[from]
        source: DecodeError,
    },
    #[error(transparent)]
    DeviceError(DeviceError),
    #[error("Device returned invalid response: {0:#?}")]
    InvalidResponse(Response),
    #[error("Programmer detected invalid chip with \
        manufacturer id 0x{manufacturer_id:02X} and \
        chip id 0x{chip_id:02X}"
    )]
    InvalidChip {
        manufacturer_id: u32,
        chip_id: u32,
    }
}

pub struct Dsrp<T> {
    transport: T,
    manufacturer_id: u32,
    chip_id: u32,
    max_block_size: u32,
    rom_size: u32,
    sector_size: u32,
}

impl<T: Transport> Dsrp<T> {
    pub fn init(
            transport: T,
            manufacturer_id_hint: ManufacturerId,
            chip_id_hint: ChipId,
            require_exact_chip: bool
    ) -> ProtocolResult<Self> {
        let mut this = Self {
            transport,
            manufacturer_id: 0,
            chip_id: 0,
            max_block_size: 0,
            rom_size: 0,
            sector_size: 0,
        };

        let request = Request::Init {
            manufacturer_id_hint,
            chip_id_hint,
            require_exact_chip
        };

        match this.execute(request)? {
            Response::Init {
                manufacturer_id,
                chip_id,
                max_block_size,
                rom_size,
                sector_size,
            } => {
                this.manufacturer_id = manufacturer_id;
                this.chip_id = chip_id;
                this.max_block_size = max_block_size;
                this.rom_size = rom_size;
                this.sector_size = sector_size;
                Ok(this)
            },
            response => Err(ProtocolError::InvalidResponse(response))
        }
    }

    pub fn chip_description(&self) -> String {
        let manufacturer = ManufacturerId::try_from(self.manufacturer_id)
            .map(|m| m.to_string())
            .unwrap_or_else(|_| format!("Unknown(0x{:02X})", self.manufacturer_id));

        let chip = ChipId::try_from(self.chip_id)
            .map(|m| m.to_string())
            .unwrap_or_else(|_| format!("Unknown(0x{:02X})", self.manufacturer_id));

        format!("Manufacturer({}); Chip({})", manufacturer, chip)
    }
    pub fn max_block_size(&self) -> u32 { self.max_block_size }
    pub fn rom_size(&self) -> u32 { self.rom_size }
    pub fn sector_size(&self) -> u32 { self.sector_size }

    pub fn read(&mut self, offset: u32, block_size: u32) -> ProtocolResult<Vec<u8>> {
        match self.execute(Request::Read { offset, block_size })? {
            Response::Read { block } => Ok(block),
            response => Err(ProtocolError::InvalidResponse(response))
        }
    }

    pub fn write(&mut self, offset: u32, block: Vec<u8>) -> ProtocolResult<()> {
        match self.execute(Request::Write { offset, block })? {
            Response::Write => Ok(()),
            response => Err(ProtocolError::InvalidResponse(response))
        }
    }

    pub fn erase_sector(&mut self, sector: u32) -> ProtocolResult<()> {
        match self.execute(Request::EraseSector { sector })? {
            Response::EraseSector => Ok(()),
            response => Err(ProtocolError::InvalidResponse(response))
        }
    }

    fn execute(&mut self, request: Request) -> ProtocolResult<Response> {
        let request_data = request.serialize()?;
        //println!("OUT: {:?}", request);
        self.transport.send_frame(&request_data)?;

        loop {
            let response_data = self.transport.receive_frame()?;
            let response = Response::deserialize(&response_data)?;
            //println!("IN: {:?}", response);

            match response {
                Response::InProgress => {
                    // Device reported that it is still processing request, continue querying...
                    //println!("Device is still in progress...");
                },
                Response::Error(e) => break Err(ProtocolError::DeviceError(e)),
                response => break Ok(response),
            };
        }
    }
}
