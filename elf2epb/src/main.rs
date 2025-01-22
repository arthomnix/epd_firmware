use std::path::PathBuf;
use clap::Parser;
use elf::abi::PT_LOAD;
use elf::ElfBytes;
use elf::endian::LittleEndian;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[arg(short)]
    in_file: PathBuf,

    #[arg(short)]
    out_file: PathBuf,
}

fn main() {
    let args = Args::parse();

    let elf_data = std::fs::read(args.in_file).expect("Failed to open ELF file");

    let elf = ElfBytes::<LittleEndian>::minimal_parse(&elf_data)
        .expect("Failed to parse ELF file");

    // Copy all the LOAD segments, like objcopy -O binary
    let mut bin: Vec<u8> = Vec::new();

    let mut data_paddr = 0u32;
    let mut data_vaddr = 0u32;
    let mut data_len = 0u32;

    for phdr in elf.segments().expect("Failed to parse ELF file") {
        if phdr.p_type == PT_LOAD {
            bin.extend_from_slice(elf.segment_data(&phdr).expect("Failed to parse ELF file"));
        }

        // Assume that there is at most one segment that needs to be copied from flash to RAM
        // (i.e. the segment containing .data)
        if phdr.p_paddr != phdr.p_vaddr {
            data_paddr = phdr.p_paddr as u32;
            data_vaddr = phdr.p_vaddr as u32;
            data_len = phdr.p_memsz as u32;
        }
    }

    let mut bss_paddr = 0u32;
    let mut bss_len = 0u32;
    if let Some(bss) = elf.section_header_by_name(".bss")
        .expect("Failed to parse ELF file")
    {
        bss_paddr = bss.sh_addr as u32;
        bss_len = bss.sh_size as u32;
    }

    let len = bin.len() as u32;

    // Add missing EPB header entries
    bin[8..12].copy_from_slice(&len.to_le_bytes());             // len
    bin[12..16].copy_from_slice(&data_len.to_le_bytes());       // data_len
    bin[16..20].copy_from_slice(&data_paddr.to_le_bytes());     // data_lma
    bin[20..24].copy_from_slice(&data_vaddr.to_le_bytes());     // data_vma
    bin[24..28].copy_from_slice(&bss_len.to_le_bytes());        // bss_len
    bin[28..32].copy_from_slice(&bss_paddr.to_le_bytes());      // bss_vma

    let crc = crc32fast::hash(&bin[8..]);
    bin[4..8].copy_from_slice(&crc.to_le_bytes());              // crc

    std::fs::write(args.out_file, &bin).expect("Failed to write output");
}
