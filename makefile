CC = avr-gcc

CFLAGS = -O2 -Wall -funsigned-char -funsigned-bitfields \
	-fpack-struct -fshort-enums -std=gnu99

#LDFLAGS = -Wl,-Map=bridge.map,--cref

OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump

# try to remember to bump version...
VERSION = 0.01

# build for an ATMEGA328P
m328p: CFLAGS+=-mmcu=atmega328p
m328p: PART=m328p
m328p: bridge

#bridge: bridge.o
#	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@.elf
#	$(OBJCOPY) -j .text -j .data -O ihex $@.elf $@.hex 
#	$(OBJDUMP) -h -S $< > $@.lss

bridge: OUT = bridge_$(PART)_v$(VERSION)
bridge: LDFLAGS = -Wl,-Map=$(OUT).map,--cref
bridge: bridge.o
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $(OUT).elf
	$(OBJCOPY) -j .text -j .data -O ihex $(OUT).elf $(OUT).hex 
	$(OBJDUMP) -h -S $< > $(OUT).lss

clean:
	$(RM) -rf *.o *.elf *.map *.lss *.hex
