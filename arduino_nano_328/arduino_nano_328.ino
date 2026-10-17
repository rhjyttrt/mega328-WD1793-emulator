/*
 * WD1793 Floppy Disk Controller Emulator for Arduino Nano (ATmega328P)
 * 100% Pure AVR Assembly (.S) Implementation
 *
 * Designed for 1 MHz host CPUs (6502, Z80, 6809) with <= 250ns bus response.
 *
 * HOW TO UPLOAD VIA ARDUINO IDE:
 * 1. Open this file (arduino_nano_328.ino) in the Arduino IDE.
 * 2. In Tools -> Board, select: "Arduino Nano"
 * 3. In Tools -> Processor, select: "ATmega328P" (or "ATmega328P (Old Bootloader)")
 * 4. In Tools -> Port, select your Nano's COM port.
 * 5. Click the "Upload" button (Arrow icon).
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

extern "C" {
    void run_disk_controller_asm(void);
    void setup(void);
    void loop(void);
}

void setup() {
    TIMSK0 = 0;   // Disable Timer0 interrupts (eliminates millis/micros jitter)
    UCSR0B = 0;   // Disable USART0 (releases PD0/PD1 for host data bus)
    cli();        // Disable interrupts for hard real-time host bus timing

    run_disk_controller_asm();
}

void loop() {
}

