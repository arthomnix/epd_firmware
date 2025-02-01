use std::fs::File;
use std::io::{Read, Write};
use std::path::PathBuf;
use std::time::Duration;
use clap::{Parser, Subcommand};
use serialport::SerialPort;
use tar::Archive;
use eepy_serial::{Event, Response, SerialCommand, SerialError};

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

fn write(serial: &mut Box<dyn SerialPort>, command: SerialCommand, data: &[u8]) -> Result<(), SerialError> {
    serial.write_all(&[command as u8]).unwrap();
    serial.write_all(data).unwrap();
    let mut response_buf = [0u8];
    serial.read_exact(&mut response_buf).unwrap();
    Response::try_from(response_buf[0]).unwrap().into()
}

fn next_event(serial: &mut Box<dyn SerialPort>) -> Result<Option<Event>, SerialError> {
    write(serial, SerialCommand::NextEvent, &[])?;
    let mut event_buf = [0u8; 32];
    serial.read(&mut event_buf).unwrap();
    Ok(postcard::from_bytes(&event_buf).unwrap())
}

fn upload_program(serial: &mut Box<dyn SerialPort>, path: PathBuf) -> Result<(), SerialError> {
    write(serial, SerialCommand::GetProgramSlot, &[])?;
    let mut slot_n = [0u8];
    serial.read_exact(&mut slot_n).unwrap();
    let slot_n = slot_n[0];

    let file = File::open(path).unwrap();
    let zstd_reader = zstd::stream::read::Decoder::new(file).unwrap();
    let mut tar = Archive::new(zstd_reader);
    for file in tar.entries().unwrap() {
        let mut file = file.unwrap();
        if file.path().unwrap().to_str().unwrap().ends_with(&format!(".s{slot_n:02}.epb")) {
            println!("Uploading {}", file.path().unwrap().to_str().unwrap());
            let mut buf = vec![0u8; file.size() as usize];
            file.read_exact(&mut buf).unwrap();
            write(serial, SerialCommand::UploadProgram, &buf)?;
            return Ok(());
        }
    }

    panic!("App package did not contain binary for slot {slot_n}");
}

fn main() {
    let args = Args::parse();

    // Baud rate setting doesn't matter for pure USB serial so use 0
    let mut port = serialport::new(&args.serial_port, 0)
        .timeout(Duration::from_secs(60))
        .open()
        .expect(&format!("Failed to open serial port {}", args.serial_port));

    match args.command {
        EnterHostApp => write(&mut port, SerialCommand::EnterHostApp, &[]).unwrap(),
        ExitHostApp => write(&mut port, SerialCommand::ExitHostApp, &[]).unwrap(),
        Refresh { fast, image } => {
            let data = std::fs::read(image).unwrap();
            let cmd = if fast { SerialCommand::RefreshFast } else { SerialCommand::RefreshNormal };
            write(&mut port, cmd, &data).unwrap();
        },
        DisableTouch => write(&mut port, SerialCommand::DisableTouch, &[]).unwrap(),
        EnableTouch => write(&mut port, SerialCommand::EnableTouch, &[]).unwrap(),
        NextEvent => println!("{:?}", next_event(&mut port).unwrap()),
        UploadProgram { package } => upload_program(&mut port, package).unwrap(),
    };
}
