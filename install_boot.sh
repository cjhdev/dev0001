#!/bin/sh
#
# install the bootloader
#
LOW=0xFF
HIGH=0xDA
EXTEND=0x05
UNLOCK=0x3F
LOCK=0x0F

PROG="-c dragon_isp -P usb"
PART=ATMEGA328P

avrdude $PROG -p $PART -U lock:w:$UNLOCK:m -U lfuse:w:$LOW:m -U hfuse:w:$HIGH:m -U efuse:w:$EXTEND:m -U flash:w:$1 -U lock:w:$LOCK:m
