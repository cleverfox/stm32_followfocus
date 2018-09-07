##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = main

LDSCRIPT = stm32-h103.ld

LIBNAME		= opencm3_stm32f1
DEFS		+= -DSTM32F1

FP_FLAGS	?= -msoft-float
ARCH_FLAGS	= -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd
CFLAGS += -Iatomthreads/kernel -Iatomthreads/ports/cortex-m
# -I. -IUSB_Device/Core/inc -IUSB_OTG/inc
CFLAGS += -std=c99
CFLAGS += -D_XOPEN_SOURCE=0
CFLAGS += -Ilibopencm3/include -Ichargen
#CFLAGS += -I/usr/local/gcc-arm-embedded-5_4-2016q2-20160622/arm-none-eabi/include

OBJS = hw.o cortexm3_macro.o
#tools.o  hw.o sleep.o iic.o chargen.o serial.o

include Makefile.rules

#LDLIBS += -L/usr/local/gcc-arm-embedded-5_4-2016q2-20160622/arm-none-eabi/lib -lc_nano
OBJS += -L. -lm -latomthreads
LDLIBS += -lc_nano 

bin: main.elf
	arm-none-eabi-objcopy -Obinary main.elf main.bin

install: bin
	st-flash write main.bin 0x8000000

atom:
	./build_atom.sh
