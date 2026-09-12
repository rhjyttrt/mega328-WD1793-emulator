;
; 6502 Bus Polling Core for ATmega328P (Uno/Nano R3)
; Project: arduino-nano/bus_core_nano.S
;

#include <avr/io.h>

.global init_bus_asm
.global poll_6502_bus_asm

init_bus_asm:
    clr  r18
    out  _SFR_IO_ADDR(DDRD), r18
    out  _SFR_IO_ADDR(PORTD), r18
    
    cbi  _SFR_IO_ADDR(DDRC), PC0
    cbi  _SFR_IO_ADDR(DDRC), PC1
    cbi  _SFR_IO_ADDR(DDRC), PC2
    cbi  _SFR_IO_ADDR(DDRC), PC3
    sbi  _SFR_IO_ADDR(PORTC), PC2   ; Pull-up on CS
    sbi  _SFR_IO_ADDR(DDRC), PC4
    sbi  _SFR_IO_ADDR(DDRC), PC5
    cbi  _SFR_IO_ADDR(PORTC), PC4
    cbi  _SFR_IO_ADDR(PORTC), PC5
    ret

poll_6502_bus_asm:
    ser  r23                 ; 0xFF mask
    clr  r21                 ; 0x00 mask

poll_cs:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp poll_cs

    in   r19, _SFR_IO_ADDR(PINC)
    sbrs r19, PC3
    rjmp handle_write

    ; Read decode: PC1=A1, PC0=A0
    sbrs r19, PC1
    rjmp read_a1_low
    sbrs r19, PC0
    rjmp send_sector

send_data:
    out  _SFR_IO_ADDR(PORTD), r18
    out  _SFR_IO_ADDR(DDRD), r23
    cbi  _SFR_IO_ADDR(PORTC), PC4   ; Clear DRQ
    rjmp wait_release_read

read_a1_low:
    sbrs r19, PC0
    rjmp send_status

send_track:
    out  _SFR_IO_ADDR(PORTD), r22
    out  _SFR_IO_ADDR(DDRD), r23
    rjmp wait_release_read

send_status:
    out  _SFR_IO_ADDR(PORTD), r24
    out  _SFR_IO_ADDR(DDRD), r23
    cbi  _SFR_IO_ADDR(PORTC), PC5   ; Clear INTRQ
    rjmp wait_release_read

send_sector:
    out  _SFR_IO_ADDR(PORTD), r20
    out  _SFR_IO_ADDR(DDRD), r23

wait_release_read:
    sbis _SFR_IO_ADDR(PINC), PC2
    rjmp wait_release_read

    out  _SFR_IO_ADDR(DDRD), r21    ; Float PORTD
    out  _SFR_IO_ADDR(PORTD), r21

    andi r19, 0x03
    ori  r19, 0x40                 ; 0x40 = read
    mov  r25, r19
    clr  r24
    ret

handle_write:
    in   r24, _SFR_IO_ADDR(PIND)   ; Initial sample while CS is confirmed low
wait_release_write:
    sbic _SFR_IO_ADDR(PINC), PC2   ; If CS is released (high), proceed
    rjmp write_cs_released
    in   r24, _SFR_IO_ADDR(PIND)   ; Keep updating while CS remains asserted low
    rjmp wait_release_write

write_cs_released:
    andi r19, 0x03
    cpi  r19, 0x00                 ; Command reg
    breq write_cmd
    cpi  r19, 0x03                 ; Data reg
    breq write_data
    rjmp write_done

write_cmd:
    cbi  _SFR_IO_ADDR(PORTC), PC5   ; Clear INTRQ
    rjmp write_done

write_data:
    cbi  _SFR_IO_ADDR(PORTC), PC4   ; Clear DRQ

write_done:
    ori  r19, 0x80                 ; 0x80 = write
    mov  r25, r19
    ret