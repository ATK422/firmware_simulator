# RP2040 bootrom images

`b1.bin` is the official RP2040 bootrom, revision B1 (BSD-3-Clause, Copyright (c) 2020 Raspberry Pi (Trading) Ltd.,
source <https://github.com/raspberrypi/pico-bootrom-rp2040>). It was converted from the word array that
[rp2040js](https://github.com/wokwi/rp2040js) ships in `demo/bootrom.ts` (revision 00a4a191); SHA-256
`90b31e108faedfa6bbcfcc33ba05dac61e645108cfbbdc385f8dafc71fde44ae`.

Chips sold today carry B2. The B1→B2 differences are in the USB boot and flash code paths, which the emulator
traps at a high level anyway. To use B2, build it from the `b2` tag of pico-bootrom-rp2040 (Debug build, as the README
there says) and pass `--bootrom path/to/bootrom.bin`, or drop it in this directory as `b2.bin` and rebuild.
