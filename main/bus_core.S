#include <avr/io.h>

.global init_bus_asm
.global poll_6502_bus_asm
.global spi_read_512_asm
.global spi_write_512_asm
.global stream_read_sector_asm
.global stream_write_sector_asm

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

spi_read_512_asm:
    movw r30, r24
    ldi  r25, 0xFF
    ldi  r26, lo8(511)
    ldi  r27, hi8(511)
    out  _SFR_IO_ADDR(SPDR), r25
1:
    in   r0, _SFR_IO_ADDR(SPSR)
    sbrs r0, SPIF
    rjmp 1b
    in   r22, _SFR_IO_ADDR(SPDR)
    out  _SFR_IO_ADDR(SPDR), r25
    st   Z+, r22
    sbiw r26, 1
    brne 1b
2:
    in   r0, _SFR_IO_ADDR(SPSR)
    sbrs r0, SPIF
    rjmp 2b
    in   r22, _SFR_IO_ADDR(SPDR)
    st   Z+, r22
    ret

spi_write_512_asm:
    movw r30, r24
    ldi  r26, lo8(511)
    ldi  r27, hi8(511)
    ld   r22, Z+
    out  _SFR_IO_ADDR(SPDR), r22
1:
    ld   r22, Z+
2:
    in   r0, _SFR_IO_ADDR(SPSR)
    sbrs r0, SPIF
    rjmp 2b
    out  _SFR_IO_ADDR(SPDR), r22
    sbiw r26, 1
    brne 1b
3:
    in   r0, _SFR_IO_ADDR(SPSR)
    sbrs r0, SPIF
    rjmp 3b
    ret

stream_read_sector_asm:
    movw r30, r24
    movw r26, r22
    ser  r20
    clr  r21
    ld   r18, Z+
    sbi  _SFR_IO_ADDR(PORTC), PC4

stream_read_loop:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp stream_read_loop

    in   r19, _SFR_IO_ADDR(PINC)
    sbrs r19, PC3
    rjmp stream_read_abort_write

    andi r19, 0x03
    cpi  r19, 0x03
    brne stream_read_non_data

    out  _SFR_IO_ADDR(PORTD), r18
    out  _SFR_IO_ADDR(DDRD), r20
    cbi  _SFR_IO_ADDR(PORTC), PC4

stream_read_rel:
    sbis _SFR_IO_ADDR(PINC), PC2
    rjmp stream_read_rel

    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21

    sbiw r26, 1
    breq stream_read_finish

    ld   r18, Z+
    sbi  _SFR_IO_ADDR(PORTC), PC4
    rjmp stream_read_loop

stream_read_non_data:
    ldi  r22, 0x03
    out  _SFR_IO_ADDR(PORTD), r22
    out  _SFR_IO_ADDR(DDRD), r20
stream_read_non_data_rel:
    sbis _SFR_IO_ADDR(PINC), PC2
    rjmp stream_read_non_data_rel
    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21
    rjmp stream_read_loop

stream_read_abort_write:
    in   r24, _SFR_IO_ADDR(PIND)
stream_read_abort_rel:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp stream_read_abort_done
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp stream_read_abort_rel

stream_read_abort_done:
    cbi  _SFR_IO_ADDR(PORTC), PC4
    ldi  r25, 1
    ret

stream_read_finish:
    cbi  _SFR_IO_ADDR(PORTC), PC4
    clr  r24
    clr  r25
    ret

stream_write_sector_asm:
    movw r30, r24
    movw r26, r22
    ser  r20
    clr  r21
    sbi  _SFR_IO_ADDR(PORTC), PC4

stream_write_loop:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp stream_write_loop

    in   r19, _SFR_IO_ADDR(PINC)
    sbrc r19, PC3
    rjmp stream_write_check_read

    andi r19, 0x03
    cpi  r19, 0x03
    brne stream_write_abort

    in   r18, _SFR_IO_ADDR(PIND)
stream_write_sample:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp stream_write_rel
    in   r18, _SFR_IO_ADDR(PIND)
    rjmp stream_write_sample

stream_write_rel:
    st   Z+, r18
    cbi  _SFR_IO_ADDR(PORTC), PC4

    sbiw r26, 1
    breq stream_write_finish

    sbi  _SFR_IO_ADDR(PORTC), PC4
    rjmp stream_write_loop

stream_write_check_read:
    ldi  r22, 0x03
    out  _SFR_IO_ADDR(PORTD), r22
    out  _SFR_IO_ADDR(DDRD), r20
stream_write_read_rel:
    sbis _SFR_IO_ADDR(PINC), PC2
    rjmp stream_write_read_rel
    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21
    rjmp stream_write_loop

stream_write_abort:
    in   r24, _SFR_IO_ADDR(PIND)
stream_write_abort_rel:
    sbic _SFR_IO_ADDR(PINC), PC2
    rjmp stream_write_abort_done
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp stream_write_abort_rel

stream_write_abort_done:
    cbi  _SFR_IO_ADDR(PORTC), PC4
    ldi  r25, 1
    ret

stream_write_finish:
    cbi  _SFR_IO_ADDR(PORTC), PC4
    clr  r24
    clr  r25
    ret
