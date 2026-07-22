;
; Cycle-Accurate 6502 Bus Polling Core for ATmega328P (Uno/Nano R3)
;

#include <avr/io.h>

.global init_bus_asm
.global poll_6502_bus_asm

init_bus_asm:
    clr  r18
    out  _SFR_IO_ADDR(DDRD), r18
    out  _SFR_IO_ADDR(PORTD), r18
    
    sbi  _SFR_IO_ADDR(DDRC), PC4
    sbi  _SFR_IO_ADDR(DDRC), PC5
    cbi  _SFR_IO_ADDR(PORTC), PC4
    cbi  _SFR_IO_ADDR(PORTC), PC5
    ret

poll_6502_bus_asm:
poll_cs:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp poll_cs

    in   r19, _SFR_IO_ADDR(PINC)
    sbrs r19, PC3
    rjmp handle_write

    mov  r23, r19
    andi r23, 0x03
    ori  r23, 0x40

    mov  r25, r19
    andi r25, 0x03

    cpi  r25, 0x00
    breq send_status
    cpi  r25, 0x01
    breq send_track
    cpi  r25, 0x02
    breq send_sector

send_data:
    mov  r25, r18
    cbi  _SFR_IO_ADDR(PORTC), PC4
    rjmp drive_bus

send_status:
    mov  r25, r24
    cbi  _SFR_IO_ADDR(PORTC), PC5
    rjmp drive_bus

send_track:
    mov  r25, r22
    rjmp drive_bus

send_sector:
    mov  r25, r20

drive_bus:
    out  _SFR_IO_ADDR(PORTD), r25
    ser  r19
    out  _SFR_IO_ADDR(DDRD), r19

wait_release_read:
    sbis _SFR_IO_ADDR(PINC), PC2
    rjmp wait_release_read

    clr  r18
    out  _SFR_IO_ADDR(DDRD), r18
    out  _SFR_IO_ADDR(PORTD), r18

    mov  r25, r23
    clr  r24
    ret

handle_write:
    in   r24, _SFR_IO_ADDR(PIND)

wait_release_write:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp write_cs_released
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp wait_release_write

write_cs_released:
    andi r19, 0x03
    ori  r19, 0x80
    mov  r25, r19
    ret