#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <util/delay.h>

#define SD_CS_LOW()   (PORTB &= ~(1 << PB2))
#define SD_CS_HIGH()  (PORTB |=  (1 << PB2))

static uint8_t is_sdhc = 0;
static uint8_t sdhc_512_buf[512];

uint8_t spi_transfer(uint8_t data) {
    SPDR = data;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

uint8_t sd_send_command(uint8_t cmd, uint32_t arg, uint8_t crc) {
    SD_CS_HIGH();
    spi_transfer(0xFF);
    SD_CS_LOW();

    spi_transfer(cmd | 0x40);
    spi_transfer((uint8_t)(arg >> 24));
    spi_transfer((uint8_t)(arg >> 16));
    spi_transfer((uint8_t)(arg >> 8));
    spi_transfer((uint8_t)arg);
    spi_transfer(crc);

    uint8_t res, timeout = 100;
    do {
        res = spi_transfer(0xFF);
    } while ((res & 0x80) && --timeout);

    return res;
}

uint8_t sd_init(void) {
    SD_CS_HIGH();

    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    DDRB &= ~(1 << PB4);
    PORTB |= (1 << PB4);

    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
    SPSR &= ~(1 << SPI2X);

    _delay_ms(10);

    for (uint8_t i = 0; i < 10; i++) spi_transfer(0xFF);

    uint8_t r1 = 0xFF;
    for (uint8_t retry = 0; retry < 20; retry++) {
        r1 = sd_send_command(0, 0, 0x95);
        if (r1 == 0x01) break;
        _delay_ms(1);
    }
    if (r1 != 0x01) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 1;
    }

    uint8_t is_v2 = 0;
    if (sd_send_command(8, 0x000001AA, 0x87) == 0x01) {
        for (uint8_t i = 0; i < 4; i++) spi_transfer(0xFF);
        is_v2 = 1;
    }

    uint16_t timeout = 1000;
    while (timeout > 0) {
        sd_send_command(55, 0, 0xFF);
        if (sd_send_command(41, is_v2 ? 0x40000000UL : 0, 0xFF) == 0x00) break;
        _delay_ms(1);
        timeout--;
    }
    if (timeout == 0) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 3;
    }

    if (is_v2 && sd_send_command(58, 0, 0xFF) == 0x00) {
        uint8_t ocr0 = spi_transfer(0xFF);
        spi_transfer(0xFF); spi_transfer(0xFF); spi_transfer(0xFF);
        if (ocr0 & 0x40) is_sdhc = 1;
    } else {
        is_sdhc = 0;
    }

    if (!is_sdhc) {
        sd_send_command(16, 512, 0xFF);
        SD_CS_HIGH();
        spi_transfer(0xFF);
    }

    SPCR &= ~((1 << SPR1) | (1 << SPR0));
    SPSR |= (1 << SPI2X);

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 0;
}

uint8_t sd_read_512_block(uint32_t sdhc_lba, uint8_t *buf512) {
    uint32_t addr = is_sdhc ? sdhc_lba : (sdhc_lba << 9);
    if (sd_send_command(17, addr, 0xFF) != 0x00) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 1;
    }

    uint32_t timeout = 100000UL;
    uint8_t tok;
    while ((tok = spi_transfer(0xFF)) == 0xFF) {
        if (--timeout == 0) {
            SD_CS_HIGH();
            spi_transfer(0xFF);
            return 2;
        }
    }
    if (tok != 0xFE) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 2;
    }

    for (uint16_t i = 0; i < 512; i++) {
        buf512[i] = spi_transfer(0xFF);
    }

    spi_transfer(0xFF);
    spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 0;
}

uint8_t sd_write_512_block(uint32_t sdhc_lba, const uint8_t *buf512) {
    uint32_t addr = is_sdhc ? sdhc_lba : (sdhc_lba << 9);
    if (sd_send_command(24, addr, 0xFF) != 0x00) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 1;
    }

    spi_transfer(0xFF);
    spi_transfer(0xFE);

    for (uint16_t i = 0; i < 512; i++) {
        spi_transfer(buf512[i]);
    }

    spi_transfer(0xFF);
    spi_transfer(0xFF);

    uint8_t resp = 0xFF;
    for (uint8_t r = 0; r < 64; r++) {
        resp = spi_transfer(0xFF);
        if ((resp & 0x11) == 0x01) break;
    }
    if ((resp & 0x1F) != 0x05) {
        SD_CS_HIGH();
        spi_transfer(0xFF);
        return 2;
    }

    uint32_t timeout = 300000UL;
    while (spi_transfer(0xFF) == 0x00) {
        if (--timeout == 0) {
            SD_CS_HIGH();
            spi_transfer(0xFF);
            return 3;
        }
    }

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 0;
}

uint8_t sd_read_floppy_sector(uint32_t image_base_lba, uint32_t floppy_sector_idx, uint16_t sector_size, uint8_t *target_buf) {
    if (sector_size == 0) return 1;
    if (sector_size < 512) {
        uint16_t sectors_per_block = 512 / sector_size;
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx / sectors_per_block);
        uint16_t sub_offset = (floppy_sector_idx % sectors_per_block) * sector_size;

        uint8_t res = sd_read_512_block(sdhc_lba, sdhc_512_buf);
        if (res != 0) return res;

        for (uint16_t i = 0; i < sector_size; i++) {
            target_buf[i] = sdhc_512_buf[sub_offset + i];
        }
        return 0;
    } else if (sector_size == 512) {
        uint32_t sdhc_lba = image_base_lba + floppy_sector_idx;
        return sd_read_512_block(sdhc_lba, target_buf);
    } else {
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx * 2);
        uint8_t res = sd_read_512_block(sdhc_lba, target_buf);
        if (res != 0) return res;
        return sd_read_512_block(sdhc_lba + 1, target_buf + 512);
    }
}

uint8_t sd_write_floppy_sector(uint32_t image_base_lba, uint32_t floppy_sector_idx, uint16_t sector_size, const uint8_t *source_buf) {
    if (sector_size == 0) return 1;
    if (sector_size < 512) {
        uint16_t sectors_per_block = 512 / sector_size;
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx / sectors_per_block);
        uint16_t sub_offset = (floppy_sector_idx % sectors_per_block) * sector_size;

        uint8_t res = sd_read_512_block(sdhc_lba, sdhc_512_buf);
        if (res != 0) return res;

        for (uint16_t i = 0; i < sector_size; i++) {
            sdhc_512_buf[sub_offset + i] = source_buf[i];
        }

        return sd_write_512_block(sdhc_lba, sdhc_512_buf);
    } else if (sector_size == 512) {
        uint32_t sdhc_lba = image_base_lba + floppy_sector_idx;
        return sd_write_512_block(sdhc_lba, source_buf);
    } else {
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx * 2);
        uint8_t res = sd_write_512_block(sdhc_lba, source_buf);
        if (res != 0) return res;
        return sd_write_512_block(sdhc_lba + 1, source_buf + 512);
    }
}
