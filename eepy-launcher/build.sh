#!/bin/sh

rm -rf out
mkdir out
cargo build --release
elf2epb -i ../target/thumbv6m-none-eabi/release/eepy-launcher -o out/eepy-launcher.s00.epb
truncate -s 128K out/eepy-launcher.s00.epb