# ATmega328 WD1793 Floppy Disk Controller Emulator
ATmega328/328P emulator for the Western Digital WD1793 Floppy Disk Controller.

## Flashing Firmware
avrdude -c usbasp -p m328p -U flash:w:firmware/main.hex:i
