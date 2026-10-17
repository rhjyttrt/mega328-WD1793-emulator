#include "disk_defs.h"

.global sd_init_asm
.global sd_read_512_block_asm
.global sd_write_512_block_asm
.global sd_read_floppy_sector_asm
.global sd_write_floppy_sector_asm
.global spi_read_512_asm
.global spi_write_512_asm
.global spi_transfer_byte
.global spi_tx_ff

.section .text

/* =========================================================================
   SPI Low-Level Support Routines
   ========================================================================= */

spi_transfer_byte:
    out  _SFR_IO_ADDR(SPDR), r24
1:
    in   r0, _SFR_IO_ADDR(SPSR)
    sbrs r0, SPIF
    rjmp 1b
    in   r24, _SFR_IO_ADDR(SPDR)
    ret

spi_tx_ff:
    ldi  r24, 0xFF
    rjmp spi_transfer_byte

sd_cs_high_ff:
    sbi  _SFR_IO_ADDR(PORTB), PIN_SD_CS
    rcall spi_tx_ff
    ret

sd_cs_low:
    cbi  _SFR_IO_ADDR(PORTB), PIN_SD_CS
    ret

spi_init_low_speed:
    sbi  _SFR_IO_ADDR(PORTB), PIN_SD_CS
    sbi  _SFR_IO_ADDR(DDRB), PIN_SD_CS
    sbi  _SFR_IO_ADDR(DDRB), PIN_MOSI
    sbi  _SFR_IO_ADDR(DDRB), PIN_SCK
    cbi  _SFR_IO_ADDR(DDRB), PIN_MISO
    sbi  _SFR_IO_ADDR(PORTB), PIN_MISO     /* pull-up enabled */

    /* SPI Enable, Master, fosc/128 (125kHz @ 16MHz) */
    ldi  r24, (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0)
    out  _SFR_IO_ADDR(SPCR), r24
    in   r24, _SFR_IO_ADDR(SPSR)
    andi r24, ~(1 << SPI2X)
    out  _SFR_IO_ADDR(SPSR), r24
    ret

spi_set_high_speed:
    /* fosc/2 = 8 MHz @ 16MHz crystal */
    in   r24, _SFR_IO_ADDR(SPCR)
    andi r24, ~((1 << SPR1) | (1 << SPR0))
    out  _SFR_IO_ADDR(SPCR), r24
    in   r24, _SFR_IO_ADDR(SPSR)
    ori  r24, (1 << SPI2X)
    out  _SFR_IO_ADDR(SPSR), r24
    ret

/* Fast 512-byte read using pipelined SPI */
spi_read_512_asm:
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

/* Fast 512-byte write using pipelined SPI */
spi_write_512_asm:
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

/* Accurate delay routines for 16MHz CPU */
delay_1ms:
    ldi  r24, lo8(3998)
    ldi  r25, hi8(3998)
1:
    sbiw r24, 1
    brne 1b
    ret

delay_ms:
    push r22
1:
    rcall delay_1ms
    dec  r22
    brne 1b
    pop  r22
    ret

/* =========================================================================
   SD Command Dispatcher
   Input:
     r20 = command index (e.g. 0, 8, 16, 17, 24, 55, 58)
     r22:r23:r24:r25 = 32-bit argument (r25=MSB, r22=LSB)
     r21 = CRC byte
   Output:
     r24 = R1 response (or 0xFF on timeout)
   ========================================================================= */
sd_send_cmd_asm:
    push r16
    push r17
    push r18

    mov  r16, r24       /* save arg byte 2 */
    mov  r17, r20       /* save cmd index */

    rcall sd_cs_high_ff
    rcall sd_cs_low

    mov  r24, r17
    ori  r24, 0x40
    rcall spi_transfer_byte

    mov  r24, r25       /* arg byte 3 (MSB) */
    rcall spi_transfer_byte

    mov  r24, r16       /* arg byte 2 */
    rcall spi_transfer_byte

    mov  r24, r23       /* arg byte 1 */
    rcall spi_transfer_byte

    mov  r24, r22       /* arg byte 0 (LSB) */
    rcall spi_transfer_byte

    mov  r24, r21       /* CRC byte */
    rcall spi_transfer_byte

    ldi  r18, 100
1:
    rcall spi_tx_ff
    sbrs r24, 7         /* bit 7 clear = valid R1 response */
    rjmp 2f
    dec  r18
    brne 1b
2:
    pop  r18
    pop  r17
    pop  r16
    ret

/* =========================================================================
   sd_init_asm: Initialize SD / SDHC Card over SPI
   Output:
     r24 = 0 on success, non-zero on error
   ========================================================================= */
sd_init_asm:
    push r28
    push r29
    push r16
    push r17

    rcall spi_init_low_speed

    ldi  r22, 10
    rcall delay_ms

    /* Send >= 74 clocks with CS HIGH */
    ldi  r16, 10
1:
    rcall spi_tx_ff
    dec  r16
    brne 1b

    /* Send CMD0 (GO_IDLE_STATE) with retry */
    ldi  r17, 20
cmd0_loop:
    ldi  r20, 0
    clr  r22
    clr  r23
    clr  r24
    clr  r25
    ldi  r21, 0x95
    rcall sd_send_cmd_asm
    cpi  r24, 0x01
    breq cmd0_ok
    ldi  r22, 1
    rcall delay_ms
    dec  r17
    brne cmd0_loop

    rcall sd_cs_high_ff
    lds  r18, drive_flags
    andi r18, ~FLAG_READY
    sts  drive_flags, r18
    ldi  r24, 1         /* Error: CMD0 failed */
    rjmp sd_init_exit

cmd0_ok:
    /* Send CMD8 (SEND_IF_COND) to detect SDv2 */
    clr  r17            /* r17 = is_v2 flag */
    ldi  r20, 8
    ldi  r22, 0xAA
    ldi  r23, 0x01
    clr  r24
    clr  r25
    ldi  r21, 0x87
    rcall sd_send_cmd_asm
    cpi  r24, 0x01
    brne acmd41_init

    /* Read 4 bytes of R7 response */
    rcall spi_tx_ff
    rcall spi_tx_ff
    rcall spi_tx_ff
    rcall spi_tx_ff
    ldi  r17, 1         /* is_v2 = 1 */

acmd41_init:
    /* Loop sending CMD55 + ACMD41 until card is ready */
    ldi  r28, lo8(1000)
    ldi  r29, hi8(1000)

acmd41_loop:
    /* CMD55 (APP_CMD) */
    ldi  r20, 55
    clr  r22
    clr  r23
    clr  r24
    clr  r25
    ldi  r21, 0xFF
    rcall sd_send_cmd_asm

    /* ACMD41 */
    ldi  r20, 41
    clr  r22
    clr  r23
    clr  r24
    clr  r25
    tst  r17
    breq 1f
    ldi  r25, 0x40      /* HCS bit set for SDv2 */
1:
    ldi  r21, 0xFF
    rcall sd_send_cmd_asm
    tst  r24
    breq acmd41_ok

    ldi  r22, 1
    rcall delay_ms
    sbiw r28, 1
    brne acmd41_loop

    rcall sd_cs_high_ff
    lds  r18, drive_flags
    andi r18, ~FLAG_READY
    sts  drive_flags, r18
    ldi  r24, 3         /* Error: ACMD41 timeout */
    rjmp sd_init_exit

acmd41_ok:
    /* Check SDHC using CMD58 (READ_OCR) if v2 */
    clr  r16            /* r16 = is_sdhc */
    tst  r17
    breq set_block_len

    ldi  r20, 58
    clr  r22
    clr  r23
    clr  r24
    clr  r25
    ldi  r21, 0xFF
    rcall sd_send_cmd_asm
    tst  r24
    brne set_block_len

    /* Read OCR: byte 0 contains CCS bit (bit 6) */
    rcall spi_tx_ff
    sbrc r24, 6
    ldi  r16, 1         /* is_sdhc = 1 */

    rcall spi_tx_ff
    rcall spi_tx_ff
    rcall spi_tx_ff

set_block_len:
    tst  r16
    brne init_finish_fast

    /* Standard SD: lock block length to 512 bytes with CMD16 */
    ldi  r20, 16
    ldi  r22, lo8(512)
    ldi  r23, hi8(512)
    clr  r24
    clr  r25
    ldi  r21, 0xFF
    rcall sd_send_cmd_asm
    rcall sd_cs_high_ff

init_finish_fast:
    rcall spi_set_high_speed
    rcall sd_cs_high_ff

    /* Update drive_flags */
    lds  r24, drive_flags
    ori  r24, FLAG_READY
    tst  r16
    breq 2f
    ori  r24, FLAG_SDHC
    rjmp 3f
2:
    andi r24, ~FLAG_SDHC
3:
    sts  drive_flags, r24
    clr  r24            /* Success = 0 */

sd_init_exit:
    pop  r17
    pop  r16
    pop  r29
    pop  r28
    ret

/* =========================================================================
   sd_read_512_block_asm: Read 512-byte block from SD card
   Input:
     r22:r23:r24:r25 = 32-bit block LBA
     Z (r30:r31)     = Target buffer pointer
   Output:
     r24 = 0 on success, non-zero on error
   ========================================================================= */
sd_read_512_block_asm:
    push r28
    push r29
    push r16

    lds  r16, drive_flags
    sbrc r16, 3         /* FLAG_SDHC */
    rjmp 1f

    /* Byte addressing: multiply LBA by 512 (shift left 9 bits) */
    mov  r25, r24
    mov  r24, r23
    mov  r23, r22
    clr  r22
    lsl  r22
    rol  r23
    rol  r24
    rol  r25

1:
    ldi  r20, 17        /* CMD17 = READ_SINGLE_BLOCK */
    ldi  r21, 0xFF
    rcall sd_send_cmd_asm
    tst  r24
    breq 2f

    rcall sd_cs_high_ff
    ldi  r24, 1
    rjmp read_block_exit

2:
    /* Wait for Data Token (0xFE) */
    ldi  r28, lo8(65535)
    ldi  r29, hi8(65535)
wait_token:
    rcall spi_tx_ff
    cpi  r24, 0xFE
    breq token_found
    sbiw r28, 1
    brne wait_token

    rcall sd_cs_high_ff
    ldi  r24, 2
    rjmp read_block_exit

token_found:
    rcall spi_read_512_asm

    /* Read and discard 2 CRC bytes */
    rcall spi_tx_ff
    rcall spi_tx_ff

    rcall sd_cs_high_ff
    clr  r24

read_block_exit:
    pop  r16
    pop  r29
    pop  r28
    ret

/* =========================================================================
   sd_write_512_block_asm: Write 512-byte block to SD card
   Input:
     r22:r23:r24:r25 = 32-bit block LBA
     Z (r30:r31)     = Source buffer pointer
   Output:
     r24 = 0 on success, non-zero on error
   ========================================================================= */
sd_write_512_block_asm:
    push r28
    push r29
    push r16

    lds  r16, drive_flags
    sbrc r16, 3         /* FLAG_SDHC */
    rjmp 1f

    mov  r25, r24
    mov  r24, r23
    mov  r23, r22
    clr  r22
    lsl  r22
    rol  r23
    rol  r24
    rol  r25

1:
    ldi  r20, 24        /* CMD24 = WRITE_BLOCK */
    ldi  r21, 0xFF
    rcall sd_send_cmd_asm
    tst  r24
    breq 2f

    rcall sd_cs_high_ff
    ldi  r24, 1
    rjmp write_block_exit

2:
    rcall spi_tx_ff     /* One dummy byte */
    ldi  r24, 0xFE      /* Data Token */
    rcall spi_transfer_byte

    rcall spi_write_512_asm

    rcall spi_tx_ff     /* CRC 1 */
    rcall spi_tx_ff     /* CRC 2 */

    /* Poll Data Response Token (up to 64 bytes) */
    ldi  r16, 64
poll_resp:
    rcall spi_tx_ff
    mov  r18, r24
    andi r18, 0x11
    cpi  r18, 0x01
    breq got_resp
    dec  r16
    brne poll_resp

    rcall sd_cs_high_ff
    ldi  r24, 2
    rjmp write_block_exit

got_resp:
    andi r24, 0x1F
    cpi  r24, 0x05      /* 0x05 = Data accepted */
    breq wait_write_busy

    rcall sd_cs_high_ff
    ldi  r24, 2
    rjmp write_block_exit

wait_write_busy:
    /* Wait for programming busy (MISO held low) */
    ldi  r16, 10
outer_busy:
    ldi  r28, lo8(30000)
    ldi  r29, hi8(30000)
inner_busy:
    rcall spi_tx_ff
    tst  r24
    brne write_success
    sbiw r28, 1
    brne inner_busy
    dec  r16
    brne outer_busy

    rcall sd_cs_high_ff
    ldi  r24, 3
    rjmp write_block_exit

write_success:
    rcall sd_cs_high_ff
    clr  r24

write_block_exit:
    pop  r16
    pop  r29
    pop  r28
    ret

/* =========================================================================
   sd_read_floppy_sector_asm: Read sector into sector_buffer
   Uses current track_reg, sector_reg, bytes_per_sector
   Output:
     r24 = 0 on success, non-zero on error
   ========================================================================= */
sd_read_floppy_sector_asm:
    push r28
    push r29
    push r16
    push r17

    /* Calculate floppy_sector_idx = (track_reg * 18) + (sector_reg - 1) */
    lds  r24, track_reg
    ldi  r22, SECTORS_PER_TRACK
    mul  r24, r22
    movw r28, r0
    clr  r1

    lds  r16, sector_reg
    dec  r16
    add  r28, r16
    adc  r29, r1

    lds  r24, bytes_per_sector
    lds  r25, bytes_per_sector + 1

    cpi  r24, lo8(512)
    ldi  r16, hi8(512)
    cpc  r25, r16
    brne 1f
    rjmp read_sec_512
1:
    cpi  r24, lo8(1024)
    ldi  r16, hi8(1024)
    cpc  r25, r16
    brne 2f
    rjmp read_sec_1024
2:
    cpi  r24, lo8(128)
    ldi  r16, hi8(128)
    cpc  r25, r16
    brne 3f
    rjmp read_sec_128
3:

    /* Default: 256 bytes per sector */
read_sec_256:
    movw r22, r28
    lsr  r23
    ror  r22            /* floppy_sector_idx >> 1 */
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25

    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    rcall sd_read_512_block_asm
    tst  r24
    breq 4f
    rjmp read_sec_fail
4:

    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    sbrc r28, 0
    inc  r31            /* +256 bytes if odd floppy sector */

    ldi  r26, lo8(sector_buffer)
    ldi  r27, hi8(sector_buffer)
    ldi  r16, 0
copy_256_loop:
    ld   r0, Z+
    st   X+, r0
    dec  r16
    brne copy_256_loop
    clr  r24
    rjmp read_sec_exit

read_sec_512:
    movw r22, r28
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    rcall sd_read_512_block_asm
    rjmp read_sec_exit

read_sec_1024:
    movw r22, r28
    lsl  r22
    rol  r23
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    rcall sd_read_512_block_asm
    tst  r24
    breq 5f
    rjmp read_sec_fail
5:

    movw r22, r28
    lsl  r22
    rol  r23
    ldi  r20, lo8(IMAGE_START_LBA + 1)
    ldi  r21, hi8(IMAGE_START_LBA + 1)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sector_buffer + 512)
    ldi  r31, hi8(sector_buffer + 512)
    rcall sd_read_512_block_asm
    rjmp read_sec_exit

read_sec_128:
    movw r22, r28
    lsr  r23
    ror  r22
    lsr  r23
    ror  r22            /* floppy_sector_idx >> 2 */
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    rcall sd_read_512_block_asm
    tst  r24
    breq 6f
    rjmp read_sec_fail
6:

    mov  r16, r28
    andi r16, 0x03
    ldi  r17, 128
    mul  r16, r17
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    add  r30, r0
    adc  r31, r1
    clr  r1

    ldi  r26, lo8(sector_buffer)
    ldi  r27, hi8(sector_buffer)
    ldi  r16, 128
copy_128_loop:
    ld   r0, Z+
    st   X+, r0
    dec  r16
    brne copy_128_loop
    clr  r24
    rjmp read_sec_exit

read_sec_fail:
    tst  r24
    brne read_sec_exit
    ldi  r24, 1         /* Ensure non-zero error on failure */
read_sec_exit:
    pop  r17
    pop  r16
    pop  r29
    pop  r28
    ret

/* =========================================================================
   sd_write_floppy_sector_asm: Write sector from sector_buffer
   Uses current track_reg, sector_reg, bytes_per_sector
   Output:
     r24 = 0 on success, non-zero on error
   ========================================================================= */
sd_write_floppy_sector_asm:
    push r28
    push r29
    push r16
    push r17

    lds  r24, track_reg
    ldi  r22, SECTORS_PER_TRACK
    mul  r24, r22
    movw r28, r0
    clr  r1

    lds  r16, sector_reg
    dec  r16
    add  r28, r16
    adc  r29, r1

    lds  r24, bytes_per_sector
    lds  r25, bytes_per_sector + 1

    cpi  r24, lo8(512)
    ldi  r16, hi8(512)
    cpc  r25, r16
    brne 1f
    rjmp write_sec_512
1:
    cpi  r24, lo8(1024)
    ldi  r16, hi8(1024)
    cpc  r25, r16
    brne 2f
    rjmp write_sec_1024
2:
    cpi  r24, lo8(128)
    ldi  r16, hi8(128)
    cpc  r25, r16
    brne 3f
    rjmp write_sec_128
3:

    /* Default: 256 bytes per sector (read-modify-write) */
write_sec_256:
    movw r22, r28
    lsr  r23
    ror  r22
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25

    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    rcall sd_read_512_block_asm
    tst  r24
    breq 4f
    rjmp write_sec_fail
4:

    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    sbrc r28, 0
    inc  r31            /* +256 bytes if odd floppy sector */

    ldi  r26, lo8(sector_buffer)
    ldi  r27, hi8(sector_buffer)
    ldi  r16, 0
copy_wr_256:
    ld   r0, X+
    st   Z+, r0
    dec  r16
    brne copy_wr_256

    movw r22, r28
    lsr  r23
    ror  r22
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25

    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    rcall sd_write_512_block_asm
    rjmp write_sec_exit

write_sec_512:
    movw r22, r28
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    rcall sd_write_512_block_asm
    rjmp write_sec_exit

write_sec_1024:
    movw r22, r28
    lsl  r22
    rol  r23
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sector_buffer)
    ldi  r31, hi8(sector_buffer)
    rcall sd_write_512_block_asm
    tst  r24
    breq 5f
    rjmp write_sec_fail
5:

    movw r22, r28
    lsl  r22
    rol  r23
    ldi  r20, lo8(IMAGE_START_LBA + 1)
    ldi  r21, hi8(IMAGE_START_LBA + 1)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sector_buffer + 512)
    ldi  r31, hi8(sector_buffer + 512)
    rcall sd_write_512_block_asm
    rjmp write_sec_exit

write_sec_128:
    movw r22, r28
    lsr  r23
    ror  r22
    lsr  r23
    ror  r22
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    rcall sd_read_512_block_asm
    tst  r24
    breq 6f
    rjmp write_sec_fail
6:

    mov  r16, r28
    andi r16, 0x03
    ldi  r17, 128
    mul  r16, r17
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    add  r30, r0
    adc  r31, r1
    clr  r1

    ldi  r26, lo8(sector_buffer)
    ldi  r27, hi8(sector_buffer)
    ldi  r16, 128
copy_wr_128:
    ld   r0, X+
    st   Z+, r0
    dec  r16
    brne copy_wr_128

    movw r22, r28
    lsr  r23
    ror  r22
    lsr  r23
    ror  r22
    ldi  r20, lo8(IMAGE_START_LBA)
    ldi  r21, hi8(IMAGE_START_LBA)
    add  r22, r20
    adc  r23, r21
    clr  r24
    clr  r25
    ldi  r30, lo8(sd_block_buf)
    ldi  r31, hi8(sd_block_buf)
    rcall sd_write_512_block_asm
    rjmp write_sec_exit

write_sec_fail:
    tst  r24
    brne write_sec_exit
    ldi  r24, 1         /* Ensure non-zero error on failure */
write_sec_exit:
    pop  r17
    pop  r16
    pop  r29
    pop  r28
    ret
