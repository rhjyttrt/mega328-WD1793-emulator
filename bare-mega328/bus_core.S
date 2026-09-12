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
    sbi  _SFR_IO_ADDR(PORTC), PC2
    sbi  _SFR_IO_ADDR(DDRC), PC4
    sbi  _SFR_IO_ADDR(DDRC), PC5
    cbi  _SFR_IO_ADDR(PORTC), PC4
    cbi  _SFR_IO_ADDR(PORTC), PC5
    ret

poll_6502_bus_asm:
    ser  r23
    clr  r21

poll_cs:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp poll_cs

    in   r19, _SFR_IO_ADDR(PINC)
    sbrs r19, PC3
    rjmp handle_write

    sbrs r19, PC1
    rjmp read_a1_low
    sbrs r19, PC0
    rjmp send_sector

send_data:
    out  _SFR_IO_ADDR(PORTD), r18
    out  _SFR_IO_ADDR(DDRD), r23
    cbi  _SFR_IO_ADDR(PORTC), PC4
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
    cbi  _SFR_IO_ADDR(PORTC), PC5
    rjmp wait_release_read

send_sector:
    out  _SFR_IO_ADDR(PORTD), r20
    out  _SFR_IO_ADDR(DDRD), r23

wait_release_read:
    sbis _SFR_IO_ADDR(PINC), PC2
    rjmp wait_release_read

    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21

    andi r19, 0x03
    ori  r19, 0x40
    mov  r25, r19
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
    cpi  r19, 0x00
    breq write_cmd
    cpi  r19, 0x03
    breq write_data
    rjmp write_done

write_cmd:
    cbi  _SFR_IO_ADDR(PORTC), PC5
    rjmp write_done

write_data:
    cbi  _SFR_IO_ADDR(PORTC), PC4

write_done:
    ori  r19, 0x80
    mov  r25, r19
    ret
