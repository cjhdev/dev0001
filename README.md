# dev0001: Two-Wire Bridge

This project is an implementation of the server side of a two-wire bridging protocol I made up. I call the protocol PR01 and the full details are published [here](doc/doc0002-0.02.pdf).

The code compiles for an atmega328p (arduino duemilanove) and I tend to use it with an stk500 compatible bootloader.

## License
Simplified BSD License; Notice is in the source.

## How Could This Be Useful?
You can use this code to easily bridge a PC to a two-wire device using a common dev board.

## Build and Install Instructions
### Atmega328p
    make clean
    make m328p
    sh install.sh bridge_m328p_{VERSION}.hex
Make sure the programmer settings in install.sh are suitable for your application.

Cameron Harper 2012
(cam@cjh.id.au) 

