#!/bin/sh
#
# load firmware image

#PROG="-c dragon_dw -P usb"
#PROG="-c dragon_isp -P usb"
PROG="-c arduino -P /dev/ttyUSB0 -b 19200"

PART=ATMEGA328P

avrdude $PROG -p $PART -U flash:w:$1

