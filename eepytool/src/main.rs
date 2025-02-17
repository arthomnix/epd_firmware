use std::fs::File;
use std::io::Read;
use std::path::PathBuf;
use clap::{Parser, Subcommand};
use tar::Archive;
use eepy_serial_host::{Normal, Serial};
use eepy_serial_host::image::IMAGE_BYTES;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short = 'p', long)]
    serial_port: String,

    #[command(subcommand)]
    command: Subcommands,
}

#[derive(Subcommand, Debug)]
enum Subcommands {
    EnterHostApp,
    ExitHostApp,

    Refresh {
        #[arg(long, action)]
        fast: bool,
        #[arg(long, action)]
        maybe: bool,
        #[arg(short, long)]
        image: PathBuf,
    },

    DisableTouch,
    EnableTouch,
    NextEvent,

    UploadProgram {
        package: PathBuf,
    },
}

use Subcommands::*;

fn upload_program(serial: &mut Serial<Normal>, path: PathBuf) -> color_eyre::Result<()> {
    let slot_n = serial.get_slot()?;

    let file = File::open(path)?;
    let zstd_reader = zstd::stream::read::Decoder::new(file)?;
    let mut tar = Archive::new(zstd_reader);
    for file in tar.entries()? {
        let mut file = file?;
        if file.path()?.to_str().unwrap().ends_with(&format!(".s{slot_n:02}.epb")) {
            println!("Uploading {}", file.path()?.to_str().unwrap());
            let mut buf = vec![0u8; file.size() as usize];
            file.read_exact(&mut buf)?;
            serial.upload(&buf)?;
            return Ok(());
        }
    }

    panic!("App package did not contain binary for slot {slot_n}");
}

fn main() -> color_eyre::Result<()> {
    let args = Args::parse();

    let port = Serial::new(&args.serial_port)?;

    match args.command {
        EnterHostApp => { port.host_app()?; },
        ExitHostApp => { port.normal()?; },
        Refresh { fast, maybe, image } => {
            let mut buf = [0u8; IMAGE_BYTES];
            let mut file = File::open(image)?;
            file.read_exact(&mut buf)?;

            if maybe {
                port.host_app()?.maybe_refresh(fast, &buf)?;
            } else {
                port.host_app()?.refresh(fast, &buf)?;
            }
        },
        DisableTouch => port.host_app()?.set_touch_enabled(false)?,
        EnableTouch => port.host_app()?.set_touch_enabled(true)?,
        NextEvent => println!("{:?}", port.host_app()?.next_event()?),
        UploadProgram { package } => upload_program(&mut port.normal()?, package)?,
    }

    Ok(())
}
