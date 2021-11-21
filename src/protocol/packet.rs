use bincode::{Encode, Decode};
use thiserror::Error;
use crate::protocol::ProtocolResult;

#[allow(non_camel_case_types)]
#[repr(u32)]
#[derive(Encode, Decode, Debug, strum::Display, strum::EnumString, strum::EnumVariantNames, num_enum::TryFromPrimitive)]
#[strum(serialize_all = "snake_case")]
pub enum ManufacturerId {
    AMD = 0x01,
    SST = 0xBF,
}

#[allow(non_camel_case_types)]
#[repr(u32)]
#[derive(Encode, Decode, Debug, strum::Display, strum::EnumString, strum::EnumVariantNames, num_enum::TryFromPrimitive)]
#[strum(serialize_all = "snake_case")]
pub enum ChipId {
    // AMD
    AM29F040 = 0xA4,
    // SST
    SST39SF040 = 0xB7,
}

#[derive(Encode, Decode, Debug, Error)]
pub enum Error {
    #[error("Sent request is not supported by the programmer")]
    UnsupportedRequest,
    #[error("Requested ROM is not supported by the programmer")]
    UnsupportedRom,
    #[error("Failed to initialize ROM (is rom present?)")]
    RomInitFailed,
    #[error("Invalid offset")]
    InvalidOffset,
    #[error("Invalid data chunk size")]
    InvalidSize,
    #[error("Erase sector operation failed (bad sector?)")]
    EraseSectorFailed,
    #[error("Write operation failed (sector erase requried or bad sector detected)")]
    WriteFailed,
    #[error("Detected chip id does not match with the requested id")]
    RomNotMatch,
    #[error("Device is not initialized")]
    NotInitialized,
    #[error("Provided sector is invalid")]
    InvalidSector,
}

#[derive(Encode, Decode, Debug)]
pub enum Request {
    /// ID = 0x00
    /// manufacturer_id_hint and vendor_id_hint
    /// are used to suggest to the programmer
    /// chip routines and params if chip has
    /// unknown signature
    Init {
        manufacturer_id_hint: ManufacturerId,
        chip_id_hint: ChipId,
        require_exact_chip: bool,
    },
    /// ID = 0x01
    Read {
        offset: u32,
        /// Should be smaller or equal to max_block_size returned by
        /// the device
        block_size: u32,
    },
    /// ID = 0x02
    Write {
        offset: u32,
        block: Vec<u8>,
    },
    /// ID = 0x03
    EraseSector {
        sector: u32,
    },
}

impl Request {
    pub fn serialize(&self) -> ProtocolResult<Vec<u8>> {
        let data = bincode::encode_to_vec(self, bincode_config())?;
        Ok(data)
    }
}

#[derive(Encode, Decode, Debug)]
pub enum Response {
    /// ID = 0x00
    Init {
        max_block_size: u32,
        rom_size: u32,
        sector_size: u32,
        // Defined as u32 to allow unknown chips
        manufacturer_id: u32,
        // Defined as u32 to allow unknown chips
        chip_id: u32,
    },
    /// ID = 0x01
    Read {
        block: Vec<u8>,
    },
    /// ID = 0x02
    Write,
    /// ID = 0x03
    EraseSector,
    /// May be send by device to state that operation is
    /// still in progress (e.g erase or write is a long
    /// operation, device will send updates sometimes)
    /// ID = 0x04
    InProgress,
    /// ID = 0x05
    Error(Error),
}

impl Response {
    pub fn deserialize(data: &[u8]) -> ProtocolResult<Self> {
        let response = bincode::decode_from_slice(data, bincode_config())?;
        Ok(response)
    }
}

fn bincode_config() -> impl bincode::config::Config {
    bincode::config::Configuration::standard()
}
