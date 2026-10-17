#include "disk_defs.h"

.global init_bus_asm
.global poll_6502_bus_asm
.global stream_read_sector_asm
.global stream_write_sector_asm
.global stream_sink_bytes_asm

.section .text

/* =========================================================================
   init_bus_asm: Initialize 6502 Bus Interface Pins
   ========================================================================= */
init_bus_asm:
    clr  r18
    /* Port D: Data bus (inputs, tri-state, no pullups) */
    out  _SFR_IO_ADDR(DDRD), r18
    out  _SFR_IO_ADDR(PORTD), r18

    /* Port C: A0..A3 inputs, /CS pullup, DRQ and INTRQ outputs low */
    cbi  _SFR_IO_ADDR(DDRC), PIN_A0
    cbi  _SFR_IO_ADDR(DDRC), PIN_A1
    cbi  _SFR_IO_ADDR(DDRC), PIN_CS
    cbi  _SFR_IO_ADDR(DDRC), PIN_RW
    sbi  _SFR_IO_ADDR(PORTC), PIN_CS     /* Internal pull-up on /CS */

    sbi  _SFR_IO_ADDR(DDRC), PIN_DRQ    /* DRQ output */
    sbi  _SFR_IO_ADDR(DDRC), PIN_INTRQ  /* INTRQ output */
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    cbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    ret

/* =========================================================================
   poll_6502_bus_asm: Monitor bus for /CS assertion, handle register reads/writes
   Simulates Index Pulse for Type 1 status
   Output:
     r25 = Event type:
           0x40 | addr (Read event)
           0x80 | addr (Write event)
     r24 = Written data byte (for write event), or 0 (for read event)
   ========================================================================= */
poll_6502_bus_asm:
    ser  r23            /* r23 = 0xFF (DDRD output mask) */
    clr  r21            /* r21 = 0x00 (DDRD input mask) */
    ldi  r22, 64        /* Idle loop prescaler counter */
    lds  r24, status_reg/* Preload status_reg for instant status response */

poll_cs_wait:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp bus_cs_low
    dec  r22
    brne poll_cs_wait

    /* Prescaler expired (~13 microseconds): update index pulse */
    ldi  r22, 64
    lds  r26, index_pulse_counter
    lds  r27, index_pulse_counter + 1
    adiw r26, 1
    sts  index_pulse_counter, r26
    sts  index_pulse_counter + 1, r27

    /* If last command was Type 1, update index pulse bit in status_reg */
    lds  r18, drive_flags
    sbrs r18, 2         /* FLAG_TYPE1 */
    rjmp 2f

    /* Index pulse is active when (counter & 0x3FFF) < 0x0200 */
    mov  r18, r27
    andi r18, 0x3F      /* high 6 bits of 14-bit mask */
    cpi  r18, 0x02
    brsh 1f

    /* Index pulse active: set bit 1 in status_reg */
    lds  r18, status_reg
    ori  r18, ST_INDEX
    sts  status_reg, r18
    rjmp 2f

1:
    /* Index pulse inactive: clear bit 1 in status_reg */
    lds  r18, status_reg
    andi r18, ~ST_INDEX
    sts  status_reg, r18

2:
    lds  r24, status_reg/* Refresh preloaded status */
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp poll_cs_wait

bus_cs_low:
    /* /CS is asserted LOW */
    in   r19, _SFR_IO_ADDR(PINC)
    sbrs r19, PIN_RW
    rjmp handle_bus_write

    /* ===================== READ CYCLE ===================== */
    sbrc r19, PIN_A1
    rjmp read_a1_high

read_a1_low:
    sbrc r19, PIN_A0
    rjmp send_track

send_status:
    out  _SFR_IO_ADDR(PORTD), r24       /* r24 already preloaded with status_reg */
    out  _SFR_IO_ADDR(DDRD), r23        /* Drive bus immediately (300 ns) */
    cbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ /* Reading status clears INTRQ */
    rjmp wait_read_cs_rel

send_track:
    lds  r24, track_reg
    out  _SFR_IO_ADDR(PORTD), r24
    out  _SFR_IO_ADDR(DDRD), r23
    rjmp wait_read_cs_rel

read_a1_high:
    sbrc r19, PIN_A0
    rjmp send_data

send_sector:
    lds  r24, sector_reg
    out  _SFR_IO_ADDR(PORTD), r24
    out  _SFR_IO_ADDR(DDRD), r23
    rjmp wait_read_cs_rel

send_data:
    lds  r24, data_reg
    out  _SFR_IO_ADDR(PORTD), r24
    out  _SFR_IO_ADDR(DDRD), r23
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ   /* Reading data clears DRQ */

wait_read_cs_rel:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_read_cs_rel

    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21

    andi r19, 0x03
    ori  r19, 0x40
    mov  r25, r19
    clr  r24
    ret

    /* ===================== WRITE CYCLE ===================== */
handle_bus_write:
    in   r24, _SFR_IO_ADDR(PIND)
wait_write_cs_rel:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp write_cs_released
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp wait_write_cs_rel

write_cs_released:
    andi r19, 0x03
    cpi  r19, 0x00
    breq write_cmd_reg
    cpi  r19, 0x01
    breq write_track_reg
    cpi  r19, 0x02
    breq write_sector_reg
    cpi  r19, 0x03
    breq write_data_reg
    rjmp write_event_done

write_cmd_reg:
    cbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ /* Writing command clears INTRQ */
    rjmp write_event_done

write_track_reg:
    /* Update track_reg if not busy */
    lds  r18, status_reg
    sbrc r18, 0         /* ST_BUSY */
    rjmp write_event_done
    sts  track_reg, r24
    /* If last cmd was Type 1, update Track 0 bit */
    lds  r18, drive_flags
    sbrs r18, 2         /* FLAG_TYPE1 */
    rjmp write_event_done
    lds  r18, status_reg
    tst  r24
    breq 1f
    andi r18, ~ST_TRACK0
    rjmp 2f
1:  ori  r18, ST_TRACK0
2:  sts  status_reg, r18
    rjmp write_event_done

write_sector_reg:
    lds  r18, status_reg
    sbrc r18, 0         /* ST_BUSY */
    rjmp write_event_done
    sts  sector_reg, r24
    rjmp write_event_done

write_data_reg:
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ   /* Writing data clears DRQ */
    sts  data_reg, r24

write_event_done:
    ori  r19, 0x80
    mov  r25, r19
    ret

/* =========================================================================
   stream_read_sector_asm: High-Speed Streaming Sector Read
   Input:
     Z (r30:r31) = Buffer pointer
     X (r26:r27) = Length (e.g. 256)
   Output:
     r25:r24 = 0 on normal completion
     r25 = 1, r24 = written command byte if aborted by 6502 command write
   ========================================================================= */
stream_read_sector_asm:
    ser  r20            /* 0xFF */
    clr  r21            /* 0x00 */

    ld   r18, Z+
    sbi  _SFR_IO_ADDR(PORTC), PIN_DRQ

stream_rd_loop:
    /* Preload PORTD while bus is tri-stated */
    out  _SFR_IO_ADDR(PORTD), r18

wait_rd_cs_low:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_rd_cs_low

    in   r19, _SFR_IO_ADDR(PINC)
    mov  r22, r19
    andi r22, 0x0F
    cpi  r22, 0x0B      /* Read Data Register (PC3=1, PC2=0, PC1=1, PC0=1) */
    brne stream_rd_not_data

    /* Data Register read: drive bus immediately */
    out  _SFR_IO_ADDR(DDRD), r20
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ

wait_rd_cs_high:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_rd_cs_high

    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21

    sbiw r26, 1
    breq stream_rd_finish

    ld   r18, Z+
    sbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp stream_rd_loop

stream_rd_not_data:
    cpi  r22, 0x00      /* Command Register write (Abort: PC3=0, PC2=0, PC1=0, PC0=0) */
    breq stream_rd_abort_cmd

    sbrs r22, PIN_RW    /* Any other write (Address 1, 2, or 3)? */
    rjmp stream_rd_ignore_write

    /* Read non-data (Status, Track, Sector) */
    cpi  r22, 0x08
    brne 1f
    ldi  r23, (ST_BUSY | ST_DRQ)
    rjmp 3f
1:  cpi  r22, 0x09
    brne 2f
    lds  r23, track_reg
    rjmp 3f
2:  lds  r23, sector_reg
3:  out  _SFR_IO_ADDR(PORTD), r23
    out  _SFR_IO_ADDR(DDRD), r20

wait_rd_nondata_cs_high:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_rd_nondata_cs_high

    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21
    rjmp stream_rd_loop

stream_rd_abort_cmd:
    in   r24, _SFR_IO_ADDR(PIND)
wait_rd_abort_cs_high:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp rd_abort_done
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp wait_rd_abort_cs_high

rd_abort_done:
    out  _SFR_IO_ADDR(PORTD), r21   /* Tri-state preloaded PORTD */
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    ldi  r25, 1
    ret

stream_rd_ignore_write:
wait_rd_ign_cs_high:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_rd_ign_cs_high
    rjmp stream_rd_loop

stream_rd_finish:
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    clr  r24
    clr  r25
    ret

/* =========================================================================
   stream_write_sector_asm: High-Speed Streaming Sector Write
   Input:
     Z (r30:r31) = Buffer pointer
     X (r26:r27) = Length (e.g. 256)
   Output:
     r25:r24 = 0 on normal completion
     r25 = 1, r24 = written command byte if aborted by 6502 command write
   ========================================================================= */
stream_write_sector_asm:
    ser  r20
    clr  r21

    sbi  _SFR_IO_ADDR(PORTC), PIN_DRQ

stream_wr_loop:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp stream_wr_loop

    in   r19, _SFR_IO_ADDR(PINC)
    mov  r18, r19
    andi r18, 0x0F

    cpi  r18, 0x03      /* Write Data Register (PC3=0, PC2=0, PC1=1, PC0=1) */
    breq stream_wr_data

    cpi  r18, 0x00      /* Write Command Register (Abort: PC3=0, PC2=0, PC1=0, PC0=0) */
    breq stream_wr_abort

    sbrc r18, PIN_RW    /* Read cycle during write transfer? */
    rjmp stream_wr_check_read

    /* Write to Address 1 or 2 during transfer: ignore */
wait_wr_ign_cs_rel:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_wr_ign_cs_rel
    rjmp stream_wr_loop

stream_wr_data:
    in   r18, _SFR_IO_ADDR(PIND)
stream_wr_sample:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp stream_wr_cs_rel
    in   r18, _SFR_IO_ADDR(PIND)
    rjmp stream_wr_sample

stream_wr_cs_rel:
    st   Z+, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ

    sbiw r26, 1
    breq stream_wr_finish

    sbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp stream_wr_loop

stream_wr_check_read:
    cpi  r18, 0x08      /* Status read */
    brne 1f
    ldi  r22, (ST_BUSY | ST_DRQ)
    rjmp 3f
1:  cpi  r18, 0x09      /* Track read */
    brne 2f
    lds  r22, track_reg
    rjmp 3f
2:  lds  r22, sector_reg/* Sector read */
3:  out  _SFR_IO_ADDR(PORTD), r22
    out  _SFR_IO_ADDR(DDRD), r20

wait_wr_read_cs_rel:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_wr_read_cs_rel

    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21
    rjmp stream_wr_loop

stream_wr_abort:
    in   r24, _SFR_IO_ADDR(PIND)
wait_wr_abort_cs_rel:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wr_abort_done
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp wait_wr_abort_cs_rel

wr_abort_done:
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    ldi  r25, 1
    ret

stream_wr_finish:
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    clr  r24
    clr  r25
    ret

/* =========================================================================
   stream_sink_bytes_asm: Stream and Discard Bytes (for Format/Write Track)
   Input:
     X (r26:r27) = Length (e.g. 18 * 256 = 4608)
   Output:
     r25:r24 = 0 on normal completion
     r25 = 1, r24 = written command byte if aborted
   ========================================================================= */
stream_sink_bytes_asm:
    ser  r20
    clr  r21

    sbi  _SFR_IO_ADDR(PORTC), PIN_DRQ

sink_loop:
    sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp sink_loop

    in   r19, _SFR_IO_ADDR(PINC)
    mov  r18, r19
    andi r18, 0x0F

    cpi  r18, 0x03      /* Write Data Register */
    breq sink_data

    cpi  r18, 0x00      /* Write Command Register (Abort) */
    breq sink_abort

    sbrc r18, PIN_RW    /* Read cycle? */
    rjmp sink_read

wait_sink_ign_cs_rel:
    sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp wait_sink_ign_cs_rel
    rjmp sink_loop

sink_data:
1:  sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp sink_cs_rel
    rjmp 1b

sink_cs_rel:
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbiw r26, 1
    breq sink_done
    sbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp sink_loop

sink_read:
    ldi  r22, (ST_BUSY | ST_DRQ)
    out  _SFR_IO_ADDR(PORTD), r22
    out  _SFR_IO_ADDR(DDRD), r20
2:  sbis _SFR_IO_ADDR(PINC), PIN_CS
    rjmp 2b
    out  _SFR_IO_ADDR(DDRD), r21
    out  _SFR_IO_ADDR(PORTD), r21
    rjmp sink_loop

sink_abort:
    in   r24, _SFR_IO_ADDR(PIND)
3:  sbic _SFR_IO_ADDR(PINC), PIN_CS
    rjmp 4f
    in   r24, _SFR_IO_ADDR(PIND)
    rjmp 3b
4:  cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    ldi  r25, 1
    ret

sink_done:
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    clr  r24
    clr  r25
    ret
