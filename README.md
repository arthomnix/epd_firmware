# eepyOS (WIP)

eepyOS is a barebones, application-specific, single-tasking operating system in Rust for my [Touchscreen E-Paper Input Module for the Framework 16](https://github.com/arthomnix/FW16_EPD), designed to allow safe execution of user programs. Currently a major work in progress.

Summary of crates:
* `eepy` - eepyOS kernel
* `eepy-sys` - library defining the kernel's syscall interface and binary format and providing safe abstractions
* `eepy-derive` - proc macro crate for eepy-sys
* `eepy-launcher` - the userspace component of eepyOS which handles the launcher UI
* `eepy-gui` - barebones UI library for eepyOS applications based on `embedded-graphics`
* `eepy-example-app` - example userspace application for eepyOS
* `eepy-serial` - library crate for the USB serial interface
* `eepytool` - CLI tool for interfacing with the input module over USB serial
* `elf2epb` - utility for converting ELF binaries to eepyOS's binary format
* `fw16-epd-bsp` - Board Support Package for the Touchscreen E-Paper Input Module
* `pervasive-spi` - Bitbanging driver for the 3-wire SPI protocol used by Pervasive Displays e-paper panels, such as the one on the Input Module
* `tp370pgh01` - Driver for the specific display used on the Touchscreen E-Paper Input Module (display only; touch is handled directly in the kernel)
