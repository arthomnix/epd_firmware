#!/bin/sh

rm -r out
mkdir out
mkdir out/bins

for slot in `seq -f "%02g" 31`
do
  EPD_PROG_SLOT=$slot cargo build --release
  elf2epb -i ../target/thumbv6m-none-eabi/release/eepy-example-app -o "out/bins/fw16-epd-example-app.s$slot.epb"
done

(cd out/bins && tar --zstd -cf ../eepy-example-app.epa *)