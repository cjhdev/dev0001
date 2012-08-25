# dev0001: TWI Bridge

This project is an implementation of the server side of a TWI bridging protocol I made up. I call the protocol PR01. It is documented but I am yet to find somewhere to host my documents.

It compiles for an atmega328p (arduino demilanove) and I tend to use it with an stk500 compatible bootloader.

## How Could This Be Useful?
You can use this code to easily and reliably bridge a PC to a TWI/I2C device using a common dev board.

## Where Can I Get Client Software?
I have another repo in the works that implements the client side of PR01. Stay tuned.

## Build and Install Instructions
### Atmega328p
make clean
make m328p
sh install.sh


Cameron Harper 2012
(cam@cjh.id.au) 

