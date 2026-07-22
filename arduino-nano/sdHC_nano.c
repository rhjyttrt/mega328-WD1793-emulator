/*
 * Low-Level SDHC SPI Driver for Arduino Uno / Nano R3 (16 MHz)
 */

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>

#define MCU_FREQ_16MHZ
// #define MCU_FREQ_20MHZ

#define SD_CS_LOW()   (PORTB &= ~(1 << PB2))
#define SD_CS_HIGH()  (PORTB |=  (1 << PB2))

uint8_t is_sdhc = 0;
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

    uint8_t res, timeout = 10;
    do {
        res = spi_transfer(0xFF);
    } while ((res & 0x80) && --timeout);

    return res;
}

uint8_t sd_init(void) {
    // Configure SPI Pins: PB2 (CS/D10), PB3 (MOSI/D11), PB5 (SCK/D13) as Outputs
    DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
    DDRB &= ~(1 << PB4); // PB4 (MISO/D12) as Input

    // Set slow SPI clock for SD initialization step (F_CPU / 128)
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);

    SD_CS_HIGH();
    _delay_ms(10);

    for (uint8_t i = 0; i < 10; i++) spi_transfer(0xFF);

    if (sd_send_command(0, 0, 0x95) != 0x01) return 1;
    if (sd_send_command(8, 0x000001AA, 0x87) == 0x01) {
        for (uint8_t i = 0; i < 4; i++) spi_transfer(0xFF);
    } else {
        return 2;
    }

    uint16_t timeout = 1000;
    while (timeout--) {
        sd_send_command(55, 0, 0xFF);
        if (sd_send_command(41, 0x40000000, 0xFF) == 0x00) break;
        _delay_ms(1);
    }
    if (timeout == 0) return 3;

    if (sd_send_command(58, 0, 0xFF) == 0x00) {
        uint8_t ocr0 = spi_transfer(0xFF);
        spi_transfer(0xFF); spi_transfer(0xFF); spi_transfer(0xFF);
        if (ocr0 & 0x40) is_sdhc = 1;
    }

    // Switch SPI to Maximum Speed for 16 MHz (F_CPU / 2 = 8 MHz SPI clock)
    SPCR &= ~((1 << SPR1) | (1 << SPR0));
    SPSR |= (1 << SPI2X);

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 0;
}

uint8_t sd_read_512_block(uint32_t sdhc_lba, uint8_t *buf512) {
    if (sd_send_command(17, sdhc_lba, 0xFF) != 0x00) return 1;

    uint16_t timeout = 10000;
    while (spi_transfer(0xFF) != 0xFE) {
        if (--timeout == 0) return 2;
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
    if (sd_send_command(24, sdhc_lba, 0xFF) != 0x00) return 1;

    SD_CS_LOW();
    spi_transfer(0xFF);
    spi_transfer(0xFE);

    for (uint16_t i = 0; i < 512; i++) {
        spi_transfer(buf512[i]);
    }

    spi_transfer(0xFF);
    spi_transfer(0xFF);

    uint8_t resp = spi_transfer(0xFF);
    if ((resp & 0x1F) != 0x05) {
        SD_CS_HIGH();
        return 2;
    }

    uint16_t timeout = 65000;
    while (spi_transfer(0xFF) == 0x00) {
        if (--timeout == 0) {
            SD_CS_HIGH();
            return 3;
        }
    }

    SD_CS_HIGH();
    spi_transfer(0xFF);
    return 0;
}

uint8_t sd_read_floppy_sector(uint32_t image_base_lba, uint32_t floppy_sector_idx, uint16_t sector_size, uint8_t *target_buf) {
    if (sector_size <= 512) {
        uint16_t sectors_per_block = 512 / sector_size;
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx / sectors_per_block);
        uint16_t sub_offset = (floppy_sector_idx % sectors_per_block) * sector_size;

        uint8_t res = sd_read_512_block(sdhc_lba, sdhc_512_buf);
        if (res != 0) return res;

        for (uint16_t i = 0; i < sector_size; i++) {
            target_buf[i] = sdhc_512_buf[sub_offset + i];
        }
    } else {
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx * 2);
        sd_read_512_block(sdhc_lba, target_buf);
        sd_read_512_block(sdhc_lba + 1, target_buf + 512);
    }
    return 0;
}

uint8_t sd_write_floppy_sector(uint32_t image_base_lba, uint32_t floppy_sector_idx, uint16_t sector_size, const uint8_t *source_buf) {
    if (sector_size <= 512) {
        uint16_t sectors_per_block = 512 / sector_size;
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx / sectors_per_block);
        uint16_t sub_offset = (floppy_sector_idx % sectors_per_block) * sector_size;

        if (sd_read_512_block(sdhc_lba, sdhc_512_buf) != 0) {
            for (uint16_t i = 0; i < 512; i++) sdhc_512_buf[i] = 0x00;
        }

        for (uint16_t i = 0; i < sector_size; i++) {
            sdhc_512_buf[sub_offset + i] = source_buf[i];
        }

        return sd_write_512_block(sdhc_lba, sdhc_512_buf);
    } else {
        uint32_t sdhc_lba = image_base_lba + (floppy_sector_idx * 2);
        if (sd_write_512_block(sdhc_lba, source_buf) != 0) return 1;
        return sd_write_512_block(sdhc_lba + 1, source_buf + 512);
    }
}