#!/bin/sh
make -j8
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/6020_PTZ_Makefile.bin 0x08000000 verify exit"
