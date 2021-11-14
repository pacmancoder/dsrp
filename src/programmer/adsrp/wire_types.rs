use bincode::{
    Encode,
    Decode,
    de::{Decode, Decoder},
    enc::{Encode, Encoder},
    error::{DecodeError, EncodeError}
};
use num_enum::{IntoPrimitive, TryFromPrimitive};
use strum::{EnumIter, IntoEnumIterator};
use thiserror::Error;
use std::{fmt::Display, io::Read};

#[derive(Default, Debug, Clone)]
pub struct VariableBuffer(Vec<u8>);

impl From<Vec<u8>> for VariableBuffer {
    fn from(inner: Vec<u8>) -> Self {
        Self(inner)
    }
}

impl AsRef<[u8]> for VariableBuffer {
    fn as_ref(&self) -> &[u8] {
        &self.0
    }
}

impl Encode for VariableBuffer {
    fn encode<E: bincode::enc::Encoder>(&self, mut encoder: E) -> Result<(), bincode::error::EncodeError> {
        if self.0.len() > u16::MAX as usize {
            return Err(bincode::error::EncodeError::Other("Given variable buffer is too big to encode"));
        }
        (self.0.len() as u16).encode(&mut encoder)?;
        for b in &self.0 {
            b.encode(&mut encoder)?;
        }
        Ok(())
    }
}

impl Decode for VariableBuffer {
    fn decode<D: bincode::de::Decoder>(mut decoder: D) -> Result<Self, bincode::error::DecodeError> {
        let len = u16::decode(&mut decoder)? as usize;
        let mut vec = Vec::with_capacity(len);
        for _ in 0..len {
            vec.push(u8::decode(&mut decoder)?);
        }
        Ok(Self(vec))
    }
}

#[derive(Clone, Copy, Debug, IntoPrimitive, TryFromPrimitive, EnumIter, Error)]
#[repr(u8)]
pub enum ErrorCode {
    #[error("Detected invalid crc in request")]
    InvalidCrc = 0x01,
    #[error("Rejected request due to offset value overflow")]
    OffsetOverflow = 0x02,
    #[error("Rejected request due to length value overflow")]
    LengthOverflow = 0x03,
    #[error("Requested memory range should be erased before programming")]
    EraseRequired = 0x04,
    #[error("Not implemented")]
    NotImplemented = 0x05,
}

impl Encode for ErrorCode {
    fn encode<E: Encoder>(&self, encoder: E) -> Result<(), EncodeError> {
        let value: u8 = (*self).into();
        value.encode(encoder)
    }
}

impl Decode for ErrorCode {
    fn decode<D: Decoder>(decoder: D) -> Result<Self, DecodeError> {
        let min: u8 = Self::iter().next().unwrap().into();
        let max: u8 = Self::iter().rev().next().unwrap().into();

        let decoded = u8::decode(decoder)?;
        Self::try_from(decoded).map_err(|_| DecodeError::UnexpectedVariant {
            type_name: std::any::type_name::<Self>(),
            min: min as u32,
            max: max as u32,
            found: decoded as u32,
        })
    }
}
