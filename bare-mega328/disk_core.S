#include "disk_defs.h"

.global run_disk_controller_asm
.global compute_type1_status

/* Shared BSS Variables */
.section .bss
.global sector_buffer
.global sd_block_buf
.global status_reg
.global track_reg
.global sector_reg
.global data_reg
.global drive_flags
.global step_direction
.global sector_len_code
.global head_side
.global bytes_per_sector
.global index_pulse_counter
.global pending_cmd
.global has_pending_cmd

sector_buffer:
    .space 1024             /* 1024 bytes floppy sector buffer */
sd_block_buf:
    .space 512              /* 512 bytes SD card buffer */
status_reg:
    .space 1
track_reg:
    .space 1
sector_reg:
    .space 1
data_reg:
    .space 1
drive_flags:
    .space 1
step_direction:
    .space 1
sector_len_code:
    .space 1
head_side:
    .space 1
bytes_per_sector:
    .space 2
index_pulse_counter:
    .space 2
pending_cmd:
    .space 1
has_pending_cmd:
    .space 1

.section .text

/* =========================================================================
   compute_type1_status: Calculate Type 1 Status Byte
   Output:
     r24 = status byte
   ========================================================================= */
compute_type1_status:
    clr  r24
    lds  r18, drive_flags
    sbrs r18, 0         /* FLAG_READY */
    ori  r24, ST_NOTREADY

    sbrc r18, 1         /* FLAG_WRPROT */
    ori  r24, ST_WRPROT

    ori  r24, ST_HEADLOAD   /* Head loaded (0x20) */

    lds  r18, track_reg
    tst  r18
    brne 1f
    ori  r24, ST_TRACK0     /* Track 00 (0x04) */
1:
    /* Check index pulse window: (counter & 0x3FFF) < 0x0200 */
    lds  r18, index_pulse_counter + 1
    andi r18, 0x3F
    cpi  r18, 0x02
    brsh 2f
    ori  r24, ST_INDEX
2:
    ret

/* =========================================================================
   update_bytes_per_sector: Set bytes_per_sector = 128 << (code & 3)
   ========================================================================= */
update_bytes_per_sector:
    lds  r18, sector_len_code
    andi r18, 0x03
    ldi  r24, lo8(128)
    clr  r25
1:
    tst  r18
    breq 2f
    lsl  r24
    rol  r25
    dec  r18
    rjmp 1b
2:
    sts  bytes_per_sector, r24
    sts  bytes_per_sector + 1, r25
    ret

/* =========================================================================
   run_disk_controller_asm: Main Controller Entry Point
   ========================================================================= */
run_disk_controller_asm:
    rcall init_bus_asm

    /* Default register values */
    clr  r18
    sts  track_reg, r18
    sts  data_reg, r18
    sts  head_side, r18
    sts  has_pending_cmd, r18
    sts  index_pulse_counter, r18
    sts  index_pulse_counter + 1, r18

    ldi  r18, 1
    sts  sector_reg, r18
    sts  step_direction, r18
    sts  sector_len_code, r18

    ldi  r18, FLAG_TYPE1
    sts  drive_flags, r18

    rcall update_bytes_per_sector

    /* Initialize SD Card */
    rcall sd_init_asm
    tst  r24
    breq 1f
    lds  r18, drive_flags
    andi r18, ~FLAG_READY
    sts  drive_flags, r18
    rjmp 2f
1:
    lds  r18, drive_flags
    ori  r18, FLAG_READY
    sts  drive_flags, r18
2:
    rcall compute_type1_status
    sts  status_reg, r24

/* =========================================================================
   Main Event Dispatch Loop
   ========================================================================= */
main_controller_loop:
    /* Check for aborted pending command */
    lds  r18, has_pending_cmd
    tst  r18
    breq poll_bus_event

    clr  r18
    sts  has_pending_cmd, r18
    lds  r24, pending_cmd
    rjmp dispatch_command

poll_bus_event:
    rcall poll_6502_bus_asm
    sbrs r25, 7         /* Bit 7 set = Write event */
    rjmp main_controller_loop

    /* Target address in bits 1..0 */
    mov  r18, r25
    andi r18, 0x03
    tst  r18
    brne main_controller_loop

    /* Written to Command Register (Address 0) */
    /* Command byte is in r24 */

dispatch_command:
    /* If BUSY, ignore all commands except Force Interrupt (0xD0) */
    lds  r18, status_reg
    sbrs r18, 0         /* ST_BUSY */
    rjmp exec_command

    mov  r18, r24
    andi r18, 0xF0
    cpi  r18, 0xD0
    brne main_controller_loop

exec_command:
    cbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ

    /* Check command type */
    sbrs r24, 7         /* Bit 7 = 0 -> Type 1 Commands */
    rjmp handle_type1_cmd

    mov  r18, r24
    andi r18, 0xE0
    cpi  r18, 0x80
    breq handle_read_sector_entry
    cpi  r18, 0xA0
    breq handle_write_sector_entry

    mov  r18, r24
    andi r18, 0xF0
    cpi  r18, 0xC0
    breq handle_read_address_entry
    cpi  r18, 0xE0
    breq handle_read_track_entry
    cpi  r18, 0xF0
    breq handle_write_track_entry
    cpi  r18, 0xD0
    breq handle_force_interrupt_entry

    rjmp main_controller_loop

handle_read_sector_entry:
    rjmp handle_read_sector
handle_write_sector_entry:
    rjmp handle_write_sector
handle_read_address_entry:
    rjmp handle_read_address
handle_read_track_entry:
    rjmp handle_read_track
handle_write_track_entry:
    rjmp handle_write_track
handle_force_interrupt_entry:
    rjmp handle_force_interrupt

/* =========================================================================
   TYPE 1 COMMAND HANDLER: Restore, Seek, Step, Step-In, Step-Out
   ========================================================================= */
handle_type1_cmd:
    push r24            /* save command opcode */

    lds  r18, drive_flags
    ori  r18, FLAG_TYPE1
    sts  drive_flags, r18

    sbrc r18, 0         /* FLAG_READY */
    rjmp type1_ready
    rcall sd_init_asm
type1_ready:
    pop  r24
    mov  r18, r24
    andi r18, 0xF0

    cpi  r18, 0x00      /* Restore (Seek Track 0) */
    breq cmd_restore

    cpi  r18, 0x10      /* Seek */
    breq cmd_seek

    cpi  r18, 0x20      /* Step */
    breq cmd_step
    cpi  r18, 0x30
    breq cmd_step

    cpi  r18, 0x40      /* Step In */
    breq cmd_step_in
    cpi  r18, 0x50
    breq cmd_step_in

    cpi  r18, 0x60      /* Step Out */
    breq cmd_step_out
    cpi  r18, 0x70
    breq cmd_step_out

    rjmp type1_done

cmd_restore:
    clr  r18
    sts  track_reg, r18
    ldi  r18, -1
    sts  step_direction, r18
    rjmp type1_done

cmd_seek:
    lds  r19, data_reg
    lds  r20, track_reg
    cp   r19, r20
    breq 1f
    brlo 2f
    ldi  r18, 1
    sts  step_direction, r18
    rjmp 1f
2:
    ldi  r18, -1
    sts  step_direction, r18
1:
    sts  track_reg, r19
    rjmp type1_done

cmd_step:
    sbrs r24, 4         /* Bit 4: update track register flag (u) */
    rjmp type1_done
    lds  r18, step_direction
    tst  r18
    brmi step_dec
    rjmp step_inc

cmd_step_in:
    ldi  r18, 1
    sts  step_direction, r18
    sbrs r24, 4
    rjmp type1_done
step_inc:
    lds  r18, track_reg
    cpi  r18, 255
    brsh type1_done
    inc  r18
    sts  track_reg, r18
    rjmp type1_done

cmd_step_out:
    ldi  r18, -1
    sts  step_direction, r18
    sbrs r24, 4
    rjmp type1_done
step_dec:
    lds  r18, track_reg
    tst  r18
    breq type1_done
    dec  r18
    sts  track_reg, r18

type1_done:
    rcall compute_type1_status
    sts  status_reg, r24
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

/* =========================================================================
   TYPE 2 COMMAND: Read Sector
   ========================================================================= */
handle_read_sector:
    push r24
    lds  r18, drive_flags
    andi r18, ~FLAG_TYPE1
    sts  drive_flags, r18

    /* Set head_side from bit 3 */
    clr  r18
    sbrc r24, 3
    ldi  r18, 1
    sts  head_side, r18

    /* Check drive ready */
    lds  r18, drive_flags
    sbrc r18, 0         /* FLAG_READY */
    rjmp rd_sec_check_ready
    rcall sd_init_asm
    lds  r18, drive_flags
    sbrc r18, 0
    rjmp rd_sec_check_ready

    /* Not ready error */
    pop  r24
    ldi  r18, ST_NOTREADY
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rd_sec_check_ready:
    pop  r24
    mov  r17, r24
    andi r17, 0x10      /* r17 = multi_sector flag */

read_sector_loop:
    /* Validate sector_reg: 1 <= sector_reg <= SECTORS_PER_TRACK */
    lds  r18, sector_reg
    tst  r18
    breq rd_sec_rnf
    cpi  r18, SECTORS_PER_TRACK + 1
    brsh rd_sec_rnf

    /* Read sector from SD card */
    rcall sd_read_floppy_sector_asm
    tst  r24
    brne rd_sec_rnf

    /* Set BUSY and DRQ */
    ldi  r18, (ST_BUSY | ST_DRQ)
    sts  status_reg, r18

    /* Stream sector to 6502 bus */
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    lds  r26, bytes_per_sector
    lds  r27, bytes_per_sector + 1
    rcall stream_read_sector_asm
    tst  r25
    brne rd_sec_aborted

    /* Check multi-sector */
    tst  r17
    breq rd_sec_finish

    lds  r18, sector_reg
    cpi  r18, SECTORS_PER_TRACK
    brlo 1f
    inc  r18
    sts  sector_reg, r18
    rjmp rd_sec_finish
1:
    inc  r18
    sts  sector_reg, r18
    rjmp read_sector_loop

rd_sec_finish:
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rd_sec_rnf:
    ldi  r18, ST_RNF
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rd_sec_aborted:
    sts  pending_cmd, r24
    ldi  r18, 1
    sts  has_pending_cmd, r18
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp main_controller_loop

/* =========================================================================
   TYPE 2 COMMAND: Write Sector
   ========================================================================= */
handle_write_sector:
    push r24
    lds  r18, drive_flags
    andi r18, ~FLAG_TYPE1
    sts  drive_flags, r18

    clr  r18
    sbrc r24, 3
    ldi  r18, 1
    sts  head_side, r18

    /* Check write protect */
    lds  r18, drive_flags
    sbrs r18, 1         /* FLAG_WRPROT */
    rjmp wr_sec_check_ready

    pop  r24
    ldi  r18, ST_WRPROT
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wr_sec_check_ready:
    lds  r18, drive_flags
    sbrc r18, 0         /* FLAG_READY */
    rjmp wr_sec_ready
    rcall sd_init_asm
    lds  r18, drive_flags
    sbrc r18, 0
    rjmp wr_sec_ready

    pop  r24
    ldi  r18, ST_NOTREADY
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wr_sec_ready:
    pop  r24
    mov  r17, r24
    andi r17, 0x10      /* multi_sector */

write_sector_loop:
    lds  r18, sector_reg
    tst  r18
    breq wr_sec_rnf
    cpi  r18, SECTORS_PER_TRACK + 1
    brsh wr_sec_rnf

    ldi  r18, (ST_BUSY | ST_DRQ)
    sts  status_reg, r18

    /* Stream data from 6502 into sector_buffer */
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    lds  r26, bytes_per_sector
    lds  r27, bytes_per_sector + 1
    rcall stream_write_sector_asm
    tst  r25
    brne wr_sec_aborted

    /* Write sector_buffer to SD */
    rcall sd_write_floppy_sector_asm
    tst  r24
    brne wr_sec_fault

    tst  r17
    breq wr_sec_finish

    lds  r18, sector_reg
    cpi  r18, SECTORS_PER_TRACK
    brlo 1f
    inc  r18
    sts  sector_reg, r18
    rjmp wr_sec_finish
1:
    inc  r18
    sts  sector_reg, r18
    rjmp write_sector_loop

wr_sec_finish:
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wr_sec_fault:
    ldi  r18, ST_WRFAULT
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wr_sec_rnf:
    ldi  r18, ST_RNF
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wr_sec_aborted:
    sts  pending_cmd, r24
    ldi  r18, 1
    sts  has_pending_cmd, r18
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp main_controller_loop

/* =========================================================================
   TYPE 3 COMMAND: Read Address (0xC0..0xCF)
   ========================================================================= */
handle_read_address:
    lds  r18, drive_flags
    andi r18, ~FLAG_TYPE1
    sts  drive_flags, r18

    lds  r18, drive_flags
    sbrc r18, 0
    rjmp rda_ready
    rcall sd_init_asm
    lds  r18, drive_flags
    sbrc r18, 0
    rjmp rda_ready

    ldi  r18, ST_NOTREADY
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rda_ready:
    /* Prepare 6-byte address field in sector_buffer */
    lds  r18, track_reg
    sts  sector_buffer + 0, r18
    lds  r18, head_side
    sts  sector_buffer + 1, r18
    lds  r18, sector_reg
    sts  sector_buffer + 2, r18
    lds  r18, sector_len_code
    sts  sector_buffer + 3, r18
    ldi  r18, 0x12      /* CRC high */
    sts  sector_buffer + 4, r18
    ldi  r18, 0x34      /* CRC low */
    sts  sector_buffer + 5, r18

    ldi  r18, (ST_BUSY | ST_DRQ)
    sts  status_reg, r18

    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    ldi  r26, 6
    ldi  r27, 0
    rcall stream_read_sector_asm
    tst  r25
    brne rda_aborted

    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rda_aborted:
    sts  pending_cmd, r24
    ldi  r18, 1
    sts  has_pending_cmd, r18
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp main_controller_loop

/* =========================================================================
   TYPE 3 COMMAND: Read Track (0xE0..0xEF)
   Streams authentic IBM FM/MFM raw track structure:
   For each sector:
     - 6 sync bytes (0x00)
     - ID Address Mark (0xFE)
     - Track, Side, Sector, Length code (4 bytes)
     - CRC (0x12, 0x34)
     - 11 gap bytes (0x4E)
     - 6 sync bytes (0x00)
     - Data Address Mark (0xFB)
     - Sector data payload from sector_buffer
     - Data CRC (0x56, 0x78)
     - 16 gap bytes (0x4E)
   ========================================================================= */
handle_read_track:
    lds  r18, drive_flags
    andi r18, ~FLAG_TYPE1
    sts  drive_flags, r18

    lds  r18, drive_flags
    sbrc r18, 0
    rjmp rdtk_ready
    rcall sd_init_asm
    lds  r18, drive_flags
    sbrc r18, 0
    rjmp rdtk_ready

    ldi  r18, ST_NOTREADY
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rdtk_ready:
    ldi  r18, (ST_BUSY | ST_DRQ)
    sts  status_reg, r18

    ldi  r17, 1         /* r17 = current sector (1..18) */

rdtk_sector_loop:
    sts  sector_reg, r17
    rcall sd_read_floppy_sector_asm
    tst  r24
    breq 1f
    rjmp rdtk_fail
1:

    /* Build 32-byte sector header in sd_block_buf */
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)

    /* 6 bytes 0x00 (Sync) */
    clr  r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18

    /* ID Address Mark (0xFE) */
    ldi  r18, 0xFE
    st   Z+, r18

    /* Track, Side, Sector, Length */
    lds  r18, track_reg
    st   Z+, r18
    lds  r18, head_side
    st   Z+, r18
    st   Z+, r17        /* current sector */
    lds  r18, sector_len_code
    st   Z+, r18

    /* CRC (0x12, 0x34) */
    ldi  r18, 0x12
    st   Z+, r18
    ldi  r18, 0x34
    st   Z+, r18

    /* 12 bytes 0x4E (Gap 2) */
    ldi  r18, 0x4E
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18

    /* 6 bytes 0x00 (Sync) */
    clr  r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18
    st   Z+, r18

    /* Data Address Mark (0xFB) */
    ldi  r18, 0xFB
    st   Z+, r18

    /* Stream 32-byte header to bus */
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    ldi  r26, 32
    ldi  r27, 0
    rcall stream_read_sector_asm
    tst  r25
    breq 2f
    rjmp rdtk_aborted
2:

    /* Stream sector data payload */
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    lds  r26, bytes_per_sector
    lds  r27, bytes_per_sector + 1
    rcall stream_read_sector_asm
    tst  r25
    breq 3f
    rjmp rdtk_aborted
3:

    /* Build 18-byte trailer in sd_block_buf: 2 CRC bytes + 16 Gap bytes */
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    ldi  r18, 0x56
    st   Z+, r18
    ldi  r18, 0x78
    st   Z+, r18

    ldi  r18, 0x4E
    ldi  r16, 16
4:
    st   Z+, r18
    dec  r16
    brne 4b

    /* Stream 18-byte trailer to bus */
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    ldi  r26, 18
    ldi  r27, 0
    rcall stream_read_sector_asm
    tst  r25
    breq 5f
    rjmp rdtk_aborted
5:

    inc  r17
    cpi  r17, SECTORS_PER_TRACK + 1
    brsh 6f
    rjmp rdtk_sector_loop
6:

    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rdtk_fail:
    ldi  r18, ST_RNF
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

rdtk_aborted:
    sts  pending_cmd, r24
    ldi  r18, 1
    sts  has_pending_cmd, r18
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp main_controller_loop

/* =========================================================================
   TYPE 3 COMMAND: Write Track / Format (0xF0..0xFF)
   ========================================================================= */
handle_write_track:
    lds  r18, drive_flags
    andi r18, ~FLAG_TYPE1
    sts  drive_flags, r18

    lds  r18, drive_flags
    sbrs r18, 1         /* FLAG_WRPROT */
    rjmp wrtk_check_ready

    ldi  r18, ST_WRPROT
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wrtk_check_ready:
    lds  r18, drive_flags
    sbrc r18, 0
    rjmp wrtk_ready
    rcall sd_init_asm
    lds  r18, drive_flags
    sbrc r18, 0
    rjmp wrtk_ready

    ldi  r18, ST_NOTREADY
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wrtk_ready:
    ldi  r18, (ST_BUSY | ST_DRQ)
    sts  status_reg, r18

    /* Calculate total bytes to sink = SECTORS_PER_TRACK * bytes_per_sector (16-bit) */
    lds  r22, bytes_per_sector
    lds  r23, bytes_per_sector + 1
    ldi  r24, SECTORS_PER_TRACK

    mul  r22, r24       /* lo8 * 18 */
    movw r26, r0        /* r27:r26 */
    mul  r23, r24       /* hi8 * 18 */
    add  r27, r0        /* add to high byte */
    clr  r1             /* restore r1 = 0 */

    /* Stream and absorb raw format bytes from 6502 at bus speed */
    rcall stream_sink_bytes_asm
    tst  r25
    brne wrtk_aborted

    /* Fill sector_buffer with format fill byte (0xE5) */
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    lds  r26, bytes_per_sector
    lds  r27, bytes_per_sector + 1
    ldi  r18, 0xE5
fill_fmt_loop:
    st   Z+, r18
    sbiw r26, 1
    brne fill_fmt_loop

    /* Write 0xE5-filled sector to all sectors 1..SECTORS_PER_TRACK on SD */
    clr  r16            /* r16 = format error flag */
    ldi  r17, 1         /* r17 = sector 1 */
wrtk_write_all:
    sts  sector_reg, r17
    rcall sd_write_floppy_sector_asm
    tst  r24
    breq 1f
    ldi  r16, 1
1:
    inc  r17
    cpi  r17, SECTORS_PER_TRACK + 1
    brlo wrtk_write_all

    clr  r18
    tst  r16
    breq 2f
    ldi  r18, ST_WRFAULT
2:
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

wrtk_aborted:
    sts  pending_cmd, r24
    ldi  r18, 1
    sts  has_pending_cmd, r18
    lds  r18, status_reg
    andi r18, ~(ST_BUSY | ST_DRQ)
    sts  status_reg, r18
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ
    rjmp main_controller_loop

/* =========================================================================
   TYPE 4 COMMAND: Force Interrupt (0xD0..0xDF)
   ========================================================================= */
handle_force_interrupt:
    mov  r17, r24       /* Save command opcode (bit 3 = Immediate Interrupt) */

    lds  r18, drive_flags
    ori  r18, FLAG_TYPE1
    sts  drive_flags, r18

    rcall compute_type1_status
    sts  status_reg, r24
    cbi  _SFR_IO_ADDR(PORTC), PIN_DRQ

    /* If bit 3 of COMMAND opcode is set (0x08), generate immediate interrupt */
    sbrc r17, 3
    rjmp fi_assert_intrq
    cbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop

fi_assert_intrq:
    sbi  _SFR_IO_ADDR(PORTC), PIN_INTRQ
    rjmp main_controller_loop
