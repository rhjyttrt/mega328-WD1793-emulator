# WD1793 Floppy Disk Controller Emulator

## Directory Layout
- bare-mega328/ : ATmega328P MiniCore target
- arduino-nano/ : Standard Arduino Nano/Uno target

## Flashing Commands
Bare ATmega328: avrdude -c usbasp -p m328p -U flash:w:bare-mega328/firmware/main.hex:i
Arduino Nano: avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 57600 -U flash:w:arduino-nano/firmware/main_nano.hex:i
