#!/bin/sh
P=`pwd`
cd atomthreads/ports/cortex-m
gmake OPENCM3_DIR=$P/libopencm3
arm-none-eabi-ar -mc libatomthreads.a build/atom*.o
mv libatomthreads.a $P
#cp build/atom*.o ../../..
