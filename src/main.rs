use std::{fs::File, io::Write, path::PathBuf};
use indicatif::ProgressStyle;
use structopt::StructOpt;
use crate::programmer::{ProgressUpdate, Programmer};
use programmer::{ProgrammerKind, ProgressHandler};

use strum::VariantNames;
use anyhow::Context;

mod programmer;

/// Dead Simple Rom Programmer
#[derive(StructOpt)]
struct AdsrpApp {
    /// Explicitly specify serial port
    #[structopt(long)]
    port: Option<String>,
    /// Explicitly specify programmer to use
    #[structopt(long, possible_values = ProgrammerKind::VARIANTS)]
    programmer: ProgrammerKind,
    #[structopt(subcommand)]
    command: AdsrpCommand,
}

#[derive(StructOpt)]
enum AdsrpCommand {
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
        /// Input file path
        input: PathBuf,
    },
    /// Erase ROM
    Erase,
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

    let port = match args.port {
        Some(port) => port,
        None => {
            match args.programmer {
                ProgrammerKind::Adsrp => programmer::port::detect_adsrp()?,
            }
        },
    };

    let mut programmer = match args.programmer {
        ProgrammerKind::Adsrp => {
            programmer::adsrp::AdsrpProgrammer::from_port(port)?
        },
    };

    match args.command {
        AdsrpCommand::Read { offset, size, output } => {
            let mut file = File::create(output)
                .with_context(|| "Failed to create output file")?;

            let mut buffer = vec![0; size as usize];
            let mut progress = create_progress_handler(size);
            // TODO: checked u64 => u32 conversion
            programmer.read(offset.unwrap_or_default() as u32, &mut buffer, &mut progress)?;

            file.write_all(&buffer).with_context(|| "Failed to write ROM to output file")?;
        },
        AdsrpCommand::Write { offset, input } => {
            let rom = std::fs::read(input)
                .with_context(|| "Failed to read ROM file")?;

            let mut progress = create_progress_handler(rom.len() as u64);
            // TODO: checked u64 => u32 conversion
            programmer.write(offset.unwrap_or_default() as u32, &rom, &mut progress)?;
        },
        AdsrpCommand::Erase => {
            let mut progress = create_progress_handler(1);
            programmer.erase(&mut progress)?;
        },
    };
    Ok(())
}



fn create_progress_handler(rom_size: u64) -> impl ProgressHandler {
    let progress = indicatif::ProgressBar::new(rom_size as u64);

    progress.set_style(ProgressStyle::default_bar()
        .template("[{elapsed_precise}] {bar:40.cyan/blue} {pos:>7}/{len:7} {msg}")
        .progress_chars("##-"));

    move |update| {
        match update {
            ProgressUpdate::Started => {},
            ProgressUpdate::ProcessedBytesChanged(bytes) => {
                progress.set_position(bytes as u64);
            },
            ProgressUpdate::Finished => {
                progress.finish();
            }
        }
    }
}
