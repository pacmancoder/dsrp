use thiserror::Error;
use strum::{EnumString, EnumVariantNames};

pub mod adsrp;
pub mod port;

#[derive(Debug, Error)]
pub enum ProgrammerError {
    #[error("Serial port initialization failed")]
    Port(#[from] serialport::Error),
    #[error("Io error")]
    Io(#[from] std::io::Error),
    #[error("Failed to encode packet")]
    PacketEncode(bincode::error::EncodeError),
    #[error("Failed to decode packet")]
    PacketDecode(bincode::error::DecodeError),
    #[error("Failed to detect programmer port: {0}")]
    PortDetect(String),
    #[error("Received invalid checksum")]
    InvalidChecksum,
    #[error("Programmer reported error: {0}")]
    Internal(String),
    #[error("Programmer sent invalid response type")]
    InvalidResponseType,
}

pub type ProgrammerResult<T> = Result<T, ProgrammerError>;

pub enum ProgressUpdate {
    Started,
    ProcessedBytesChanged(usize),
    Finished,
}

pub trait ProgressHandler {
    fn on_update(&mut self, update: ProgressUpdate);
}

impl<F> ProgressHandler for F where F: FnMut(ProgressUpdate) -> () {
    fn on_update(&mut self, update: ProgressUpdate) {
        self(update);
    }
}

pub trait Programmer {
    fn read(&mut self, offset: u32, buffer: &mut [u8], progress: &mut dyn ProgressHandler) -> ProgrammerResult<()>;
    fn write(&mut self, offset: u32, buffer: &[u8], progress: &mut dyn ProgressHandler) -> ProgrammerResult<()>;
    fn erase(&mut self, progress: &mut dyn ProgressHandler) -> ProgrammerResult<()>;
}


#[derive(EnumString, EnumVariantNames)]
#[strum(serialize_all = "snake_case")]
pub enum ProgrammerKind {
    Adsrp,
}
