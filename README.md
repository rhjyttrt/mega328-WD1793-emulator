# WD1793 Floppy Disk Controller Emulator

An ATmega328P-based hardware emulator for the Western Digital WD1793 (and compatible FD179X series) Floppy Disk Controller.

---

## Directory Layout
- bare-mega328/ : ATmega328P MiniCore target build and source files
- arduino-nano/ : Standard Arduino Nano / Uno R3 (16 MHz) target build and source files

---

## Hardware Wiring Guide

### 1. Host System Bus Pinout (PORTD & PORTC)

| Function | ATmega328P Register | Arduino Pin | Bare DIP-28 Pin | Direction | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| D0 | PD0 | Pin D0 | Pin 2 | Bidirectional | Host Data Bus Bit 0 |
| D1 | PD1 | Pin D1 | Pin 3 | Bidirectional | Host Data Bus Bit 1 |
| D2 | PD2 | Pin D2 | Pin 4 | Bidirectional | Host Data Bus Bit 2 |
| D3 | PD3 | Pin D3 | Pin 5 | Bidirectional | Host Data Bus Bit 3 |
| D4 | PD4 | Pin D4 | Pin 6 | Bidirectional | Host Data Bus Bit 4 |
| D5 | PD5 | Pin D5 | Pin 11 | Bidirectional | Host Data Bus Bit 5 |
| D6 | PD6 | Pin D6 | Pin 12 | Bidirectional | Host Data Bus Bit 6 |
| D7 | PD7 | Pin D7 | Pin 13 | Bidirectional | Host Data Bus Bit 7 |
| A0 | PC0 | Pin A0 | Pin 23 | Input | Register Select A0 |
| A1 | PC1 | Pin A1 | Pin 24 | Input | Register Select A1 |
| CS | PC2 | Pin A2 | Pin 25 | Input | Chip Select (Active Low) |
| R/W | PC3 | Pin A3 | Pin 26 | Input | Read / Write Line |
| DRQ | PC4 | Pin A4 | Pin 27 | Output | Data Request |
| INTRQ | PC5 | Pin A5 | Pin 28 | Output | Interrupt Request |

---

### 2. SD Card Module Wiring (SPI Bus - PORTB)

| Function | ATmega328P Register | Arduino Pin | Bare DIP-28 Pin | SD Module Pin |
| :--- | :--- | :--- | :--- | :--- |
| CS | PB2 | Pin D10 | Pin 16 | CS |
| MOSI | PB3 | Pin D11 | Pin 17 | MOSI / DI |
| MISO | PB4 | Pin D12 | Pin 18 | MISO / DO |
| SCK | PB5 | Pin D13 | Pin 19 | SCK / CLK |
| VCC | - | 5V / 3.3V | Pin 7 | VCC |
| GND | - | GND | Pin 8 / 22 | GND |

---

## Flashing Commands

### Bare ATmega328 (USBasp / ISP):
avrdude -c usbasp -p m328p -U flash:w:bare-mega328/firmware/main.hex:i

### Arduino Nano / Uno R3 (Serial Bootloader):
avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 57600 -U flash:w:arduino-nano/firmware/main_nano.hex:i

---

## License
[MIT License](LICENSE)
