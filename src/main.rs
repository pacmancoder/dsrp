use std::{fs::File, io::Write, path::PathBuf};
use indicatif::ProgressStyle;
use structopt::StructOpt;
use crate::programmer::{Programmer, ProgrammerInitParams, ProgressUpdate, port};
use programmer::ProgressHandler;

use strum::VariantNames;
use anyhow::Context;
use protocol::{ManufacturerId, ChipId};

mod programmer;
pub mod protocol;

/// Dead Simple Rom Programmer
#[derive(StructOpt)]
struct AdsrpApp {
    /// Explicitly specify serial port
    #[structopt(long)]
    port: Option<String>,
    /// Specify ROM manufacturer id
    #[structopt(long, possible_values = ManufacturerId::VARIANTS)]
    manufacturer: ManufacturerId,
    /// Specify ROM chip id
    #[structopt(long, possible_values = ChipId::VARIANTS)]
    chip: ChipId,
    /// Require exact chip even if programmer detected other chip,
    /// but with the same programming protocol
    #[structopt(long)]
    exact: bool,
    #[structopt(subcommand)]
    command: DsrpCommand,
}

#[derive(StructOpt)]
enum DsrpCommand {
    /// Read ROM
    Read {
        /// ROM offset to read from e.g `512`, `0x100`
        #[structopt(long, parse(try_from_str = parse_prefixed_num))]
        offset: Option<u64>,
        /// Size of a memory chunk to read from ROM
        #[structopt(long, parse(try_from_str = parse_prefixed_num))]
        size: u64,
        /// Output file path
        output: PathBuf,
    },
    /// Write ROM
    Write {
        /// ROM offset to write into e.g `512`, `0x100`
        #[structopt(long, parse(try_from_str = parse_prefixed_num))]
        offset: Option<u64>,
        /// Allow not alligned writes, auto erase is NOT performed in this mode
        #[structopt(long)]
        force: bool,
        /// Input file path
        input: PathBuf,
    },
    /// Just init programmer without performing any actions
    Init,
    /// Erase ROM
    Erase,
    /// Erase specific sector
    EraseSector {
        sector: u32,
    }
}

fn parse_prefixed_num(str: &str) -> Result<u64, anyhow::Error> {
    let value = parse_int::parse(str)?;
    Ok(value)
}

fn main() {
    if let Err(err) = run() {
        eprintln!("ERROR: {:#}", err);
    }
}

fn run() -> anyhow::Result<()> {
    let args = AdsrpApp::from_args();

    let port_name = if let Some(port) = args.port {
        port
    } else {
        port::detect_dsrp()?
    };

    let params = ProgrammerInitParams {
        port_name,
        manufacturer: args.manufacturer,
        chip: args.chip,
        exact: args.exact,
    };

    let mut programmer = Programmer::init(params)?;

    match args.command {
        DsrpCommand::Read { offset, size, output } => {
            let mut file = File::create(output)
                .with_context(|| "Failed to create output file")?;

            let mut buffer = vec![0; size as usize];
            let mut progress = create_progress_handler();
            // TODO: checked u64 => u32 conversion
            programmer.read(offset.unwrap_or_default() as u32, &mut buffer, &mut progress)?;

            file.write_all(&buffer).with_context(|| "Failed to write ROM to output file")?;
        },
        DsrpCommand::Write { offset, input, force } => {
            let rom = std::fs::read(input)
                .with_context(|| "Failed to read ROM file")?;

            let mut progress = create_progress_handler();
            // TODO: checked u64 => u32 conversion
            programmer.write(offset.unwrap_or_default() as u32, &rom, force, &mut progress)?;
        },
        DsrpCommand::Init => {}
        DsrpCommand::Erase => {
            let mut progress = create_progress_handler();
            programmer.erase(&mut progress)?;
        },
        DsrpCommand::EraseSector { sector } => {
            let mut progress = create_progress_handler();
            programmer.erase_sector(sector, &mut progress)?;
        },

    };
    Ok(())
}



fn create_progress_handler() -> impl ProgressHandler {
    let progress = indicatif::ProgressBar::new(1 as u64);

    progress.set_style(ProgressStyle::default_bar()
        .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}")
        .progress_chars("##-"));

    move |update| {
        match update {
            ProgressUpdate::Started => {},
            ProgressUpdate::ShowMessage(s) => {
                progress.set_message(s);
            },
            ProgressUpdate::ProcessedDataChanged(count) => {
                progress.set_position(count as u64);
            },
            ProgressUpdate::DataCountChanged(count) => {
                progress.set_length(count as u64);
            }
            ProgressUpdate::Finished => {
                progress.finish();
            }
        }
    }
}
