#[unsafe(link_section = ".launcher")]
#[used]
static LAUNCHER: [u8; 0x20000] = *include_bytes!("../../eepy-launcher/out/eepy-launcher.s00.epb");