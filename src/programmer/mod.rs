use thiserror::Error;
use crate::protocol::{ChipId, Dsrp, ManufacturerId, ProtocolError, SerialTransport};
use std::time::Duration;

pub mod port;

const BAUD_RATE: u32 = 115200;
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Debug, Error)]
pub enum ProgrammerError {
    #[error("Serial port initialization failed")]
    Port(#[from] serialport::Error),
    #[error("Failed to detect programmer port: {0}")]
    PortDetect(String),
    #[error("Io error")]
    Io(#[from] std::io::Error),
    #[error("Dsrp protocol error")]
    ProtocolError(#[from] ProtocolError),
    #[error("Offset should me multiple of sector size (0x{sector_size:02X})")]
    OffsetNotMultipleOfSector {
        sector_size: u32,
    },
    #[error("Data length should me multiple of sector size (0x{sector_size:02X})")]
    DataLengthNotMultipleOfSector {
        sector_size: u32,
    },
    #[error("Write validation failed")]
    WriteValidationFailed,
}

pub type ProgrammerResult<T> = Result<T, ProgrammerError>;

pub enum ProgressUpdate {
    Started,
    ShowMessage(String),
    ProcessedDataChanged(usize),
    DataCountChanged(usize),
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

type DsrpDevice = Dsrp<SerialTransport<Box<dyn serialport::SerialPort>>>;


pub struct ProgrammerInitParams {
    pub port_name : String,
    pub manufacturer: ManufacturerId,
    pub chip: ChipId,
    pub exact: bool,
}

pub struct Programmer {
    device: DsrpDevice,
}

impl Programmer {
    pub fn init(params: ProgrammerInitParams) -> ProgrammerResult<Self> {
        let ProgrammerInitParams {
            port_name,
            manufacturer,
            chip,
            exact,
        } = params;
        println!("Initializing adsrp programmer");

        let port = serialport::new(port_name, BAUD_RATE)
            .timeout(DEFAULT_TIMEOUT)
            .open()?;

        let device = DsrpDevice::init(
            SerialTransport::new(port),
            manufacturer,
            chip,
            exact,
        )?;

        println!("Dsrp programmer initialization succeeded");
        println!(" - Chip: {}", device.chip_description());
        println!(" - Max block size: {}", bytesize::to_string(device.max_block_size() as u64, true));
        println!(" - ROM size: {}", bytesize::to_string(device.rom_size() as u64, true));
        println!(" - Sector size: {}", bytesize::to_string(device.sector_size() as u64, true));

        Ok(Self { device })
    }

    pub fn read(&mut self, mut offset: u32, mut buffer: &mut [u8], progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        if buffer.is_empty() {
            return Ok(())
        }

        progress.on_update(ProgressUpdate::DataCountChanged(buffer.len()));
        progress.on_update(ProgressUpdate::ShowMessage("Reading data".to_owned()));
        progress.on_update(ProgressUpdate::Started);

        let block_size = self.device.max_block_size();
        let initial_offset = offset;


        while !buffer.is_empty() {
            let (chunk, remains) = buffer.split_at_mut(buffer.len().min(block_size as usize));
            let src_buffer = self.device.read(offset as u32, block_size)?;

            chunk.copy_from_slice(&src_buffer[0..chunk.len()]);
            buffer = remains;
            offset += chunk.len() as u32;

            progress.on_update(
                ProgressUpdate::ProcessedDataChanged((offset - initial_offset) as usize)
            );
        }

        progress.on_update(ProgressUpdate::Finished);

        Ok(())
    }

    pub fn write(&mut self, mut offset: u32, mut buffer: &[u8], force: bool, progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        if buffer.is_empty() {
            return Ok(())
        }

        progress.on_update(ProgressUpdate::Started);

        let sector_size = self.device.sector_size();

        let start_sector = offset / sector_size;
        let end_sector = (offset + buffer.len() as u32 - 1) / sector_size;
        let initial_offset = offset;

        if !force {
            if offset % sector_size != 0 {
                return Err(ProgrammerError::OffsetNotMultipleOfSector { sector_size });
            }

            if buffer.len() as u32 % sector_size != 0 {
                return Err(ProgrammerError::DataLengthNotMultipleOfSector { sector_size });
            }

            progress.on_update(ProgressUpdate::ShowMessage("Erasing old sectors".to_owned()));
            progress.on_update(ProgressUpdate::DataCountChanged((end_sector - start_sector + 1) as usize));
            for sector in start_sector..=end_sector {
                self.device.erase_sector(sector)?;
                progress.on_update(ProgressUpdate::ProcessedDataChanged((sector - start_sector + 1) as usize));
            }
        }

        progress.on_update(ProgressUpdate::ShowMessage("Writing new data".to_owned()));
        progress.on_update(ProgressUpdate::ProcessedDataChanged(0));
        progress.on_update(ProgressUpdate::DataCountChanged(buffer.len()));
        let block_size = self.device.max_block_size();

        for sector in start_sector..=end_sector {
            let mut sector_data = vec![0xFFu8; sector_size as usize];

            let in_sector_bytes = (sector_size - (offset % sector_size)).min(buffer.len() as u32) as usize;
            let in_sector_offset = (offset % sector_size) as usize;
            let sector_offset = sector * sector_size;

            let (chunk, buffer_tail) = buffer.split_at(in_sector_bytes);
            buffer = buffer_tail;
            offset += in_sector_bytes as u32;

            // prepare sector data
            sector_data[in_sector_offset..in_sector_offset + in_sector_bytes].copy_from_slice(chunk);

            // sector write/verify loop
            const MAX_ATTEMPTS: usize = 10;
            let mut retry_attempt = 0;
            'sector_write: loop {
                // Erase previous data
                if retry_attempt != 0 {
                    self.device.erase_sector(sector)?;
                }

                retry_attempt += 1;

                let mut sector_chunk = sector_data.as_slice();
                let mut block_offset = sector_offset;
                // Start write
                while !sector_chunk.is_empty() {
                    let (block, sector_remains) = sector_chunk.split_at(sector_chunk.len().min(block_size as usize));
                    let write_result = self.device.write(block_offset as u32, block.to_vec());
                    if let Err(e) = write_result {
                        if retry_attempt > MAX_ATTEMPTS {
                            return Err(e.into());
                        } else {
                            continue 'sector_write;
                        }
                    }

                    // Validate
                    let mut written_block = vec![0; block.len()];
                    let mut fake_progress_handler = move |_| {};
                    let read_result = self.read(block_offset, &mut written_block, &mut fake_progress_handler);
                    if let Err(e) = read_result {
                        if retry_attempt > MAX_ATTEMPTS {
                            return Err(e.into());
                        } else {
                            continue 'sector_write;
                        }
                    }

                    if written_block != block {
                        if retry_attempt > MAX_ATTEMPTS {
                            return Err(ProgrammerError::WriteValidationFailed);
                        } else {
                            continue 'sector_write;
                        }
                    }

                    sector_chunk = sector_remains;
                    block_offset += block.len() as u32;
                    progress.on_update(
                        ProgressUpdate::ProcessedDataChanged((block_offset - initial_offset) as usize)
                    );
                }
                // Sector write succeeded without errors
                break 'sector_write;
            }
        }




        progress.on_update(ProgressUpdate::Finished);

        Ok(())
    }

    pub fn erase(&mut self, progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        let sector_count = self.device.rom_size() / self.device.sector_size();
        progress.on_update(ProgressUpdate::DataCountChanged(sector_count as usize));
        progress.on_update(ProgressUpdate::ShowMessage("Erasing sectors".to_owned()));
        progress.on_update(ProgressUpdate::Started);
        for sector in 0..sector_count {
            self.device.erase_sector(sector)?;
            progress.on_update(ProgressUpdate::ProcessedDataChanged(sector as usize));
        }
        progress.on_update(ProgressUpdate::Finished);
        Ok(())
    }

    pub fn erase_sector(&mut self, sector: u32, progress: &mut dyn ProgressHandler) -> ProgrammerResult<()> {
        progress.on_update(ProgressUpdate::DataCountChanged(1));
        progress.on_update(ProgressUpdate::ShowMessage("Erasing single sector".to_owned()));
        progress.on_update(ProgressUpdate::Started);
        self.device.erase_sector(sector)?;
        progress.on_update(ProgressUpdate::ProcessedDataChanged(1));
        progress.on_update(ProgressUpdate::Finished);
        Ok(())
    }
}
