# To hit RESET:
#  avrdude -p m168 -c bsd -E noreset

# To set fuses:
#  avrdude -p m168 -c bsd -U lfuse:w:0xe2:m

## mp3box:
# (run 'make ardheaders.tgz', copy to mp3box and tar xzf);
#  make ARD_DIR=.

F_CPU := 8000000UL
#MCU_TARGET     := atmega168
MCU_TARGET     := atmega328

HEX := eggbot.hex
MAP := eggbot.map

all: $(HEX) eggbot.lst

CC := avr-gcc
CXX := avr-g++
OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

ARD_DIR := /Applications/Arduino.app/Contents/Java/hardware/arduino/avr

AVR_DIR := /Applications/Arduino.app/Contents/Java/hardware/tools/avr/avr/include

CFLAGS := -O2 -DF_CPU=$(F_CPU) -I. -mmcu=$(MCU_TARGET) \
	-DARDUINO=100 \
	-I$(ARD_DIR)/cores/arduino \
	-I$(ARD_DIR)/variants/standard \
	-I$(AVR_DIR)/ \
	-DSTANDALONE=1

ARDOBJS := wiring.o wiring_digital.o hooks.o WMath.o

eggbot.elf: eggbot.o $(ARDOBJS)
	$(CC) $(CFLAGS) -Wl,-Map,$(MAP) -o $@ $^

ARD_H := \
	cores/arduino/Arduino.h \
	cores/arduino/binary.h \
	cores/arduino/WCharacter.h \
	cores/arduino/WString.h \
	cores/arduino/HardwareSerial.h \
	cores/arduino/Stream.h \
	cores/arduino/Print.h \
	cores/arduino/Printable.h \
	cores/arduino/USBAPI.h \
	cores/arduino/wiring_private.h \
	variants/standard/pins_arduino.h

ardheaders.tgz: $(addprefix $(ARD_DIR), $(ARD_H))
	tar czf $@ -C $(ARD_DIR) $(ARD_H)

program:
	avrdude -p m168 -c bsd -U flash:w:$(HEX)
.PHONY: program

reset:
	avrdude -p m168 -c bsd -E noreset
.PHONY: reset

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.ino
	$(CC) $(CFLAGS) -c -x c $< -o $@

%.o: %.cpp
	$(CXX) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -c $< -o $@

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

clean:
	rm *.elf *.o *.hex *.lst *.map $(USB)
