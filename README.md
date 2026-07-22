# WD1793 Floppy Disk Controller Emulator

An ATmega328P-based hardware emulator for the Western Digital WD1793 (and compatible FD179X series) Floppy Disk Controller.

---

## Directory Layout
- bare-mega328/ : ATmega328P MiniCore target build and source files (16/20 MHz)
- arduino-nano/ : Standard Arduino Nano / Uno R3 target build and source files (16 MHz)

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

### Bare ATmega328 (USBasp / ISP Programmer):
avrdude -c usbasp -p m328p -U flash:w:bare-mega328/firmware/main.hex:i

### Arduino Nano / Uno R3 (Serial Bootloader):
avrdude -c arduino -p m328p -P /dev/ttyUSB0 -b 57600 -U flash:w:arduino-nano/firmware/main_nano.hex:i

---

## Detailed Usage Instructions

### Step 1: Prepare the SD Card Image
The emulator reads raw 512-byte blocks directly from the SD card starting at Logical Block Address (LBA) 2048 (1 MB offset) without a file system overhead.

1. Connect your MicroSD card to your Linux PC.
2. Identify the card device name using lsblk (e.g., /dev/sdb or /dev/mmcblk0).
3. Write your floppy disk raw image (.img or .dsk) directly to sector 2048 using dd:

sudo dd if=path/to/your_floppy_disk.img of=/dev/sdX seek=2048 bs=512 conv=notrunc

---

### Step 2: Host CPU Memory / IO Mapping
Connect the emulator lines to your host computer system (6502, 6809, Z80, etc.):
- Address Lines (A0, A1): Connect to host bus lower address bits to select controller registers:
  0x00 : Command Register (Write) / Status Register (Read)
  0x01 : Track Register (Read/Write)
  0x02 : Sector Register (Read/Write)
  0x03 : Data Register (Read/Write)
- Control Lines:
  CS: Connect to host decode logic (Active Low).
  R/W: Connect to host Read/Write line (High = Read, Low = Write).
  DRQ / INTRQ: Connect to host IRQ line or poll via status reads.

---

### Step 3: Powering & Operation
1. Insert the prepared SD card into the module.
2. Apply 5V power to the ATmega328P / Arduino Nano.
3. Power on the host system.
4. The host system can now issue standard WD1793 commands (e.g., Restore, Seek, Read Sector, Write Sector) to access sectors stored on the SD card.

---

## Troubleshooting & Important Hardware Notes
- USB Serial Interference (Arduino Nano): Pins D0 and D1 double as the hardware RX/TX UART lines. Do not open the Arduino Serial Monitor or serial terminal programs while connected to the host bus, as this will corrupt Data Bits 0 and 1.
- SD Card Voltage: ATmega328 outputs are 5V logic. Ensure your SD card breakout module features an onboard 3.3V regulator and level shifter to protect the card inputs.

---

## License
[MIT License](LICENSE)
