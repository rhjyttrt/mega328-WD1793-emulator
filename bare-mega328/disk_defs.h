#ifndef DISK_DEFS_H
#define DISK_DEFS_H

#include <avr/io.h>

/* CPU Frequency (20MHz crystal) */
#ifndef F_CPU
#define F_CPU 20000000UL
#endif

/* Geometry and Image Constants */
#define MAX_SECTOR_SIZE     1024
#define SECTORS_PER_TRACK   18
#define IMAGE_START_LBA     2048

/* Host Bus Pin Definitions (Port C and Port D) */
/* Port D: 8-bit bidirectional data bus PD0..PD7 */
/* Port C: Control & Address lines */
#define PIN_A0          PC0     /* Input: Address line 0 */
#define PIN_A1          PC1     /* Input: Address line 1 */
#define PIN_CS          PC2     /* Input: Chip Select (active LOW) */
#define PIN_RW          PC3     /* Input: Read/Write (1 = Read, 0 = Write) */
#define PIN_DRQ         PC4     /* Output: Data Request (active HIGH) */
#define PIN_INTRQ       PC5     /* Output: Interrupt Request (active HIGH) */

/* SD Card SPI Pin Definitions (Port B) */
#define PIN_SD_CS       PB2     /* Output: SD Chip Select (active LOW) */
#define PIN_MOSI        PB3     /* Output: SPI MOSI */
#define PIN_MISO        PB4     /* Input:  SPI MISO (pull-up enabled) */
#define PIN_SCK         PB5     /* Output: SPI Clock */

/* Status Register Bit Masks */
#define ST_BUSY         0x01    /* Bit 0: Busy */
#define ST_DRQ          0x02    /* Bit 1: Data Request (Type 2/3) */
#define ST_INDEX        0x02    /* Bit 1: Index Pulse (Type 1) */
#define ST_TRACK0       0x04    /* Bit 2: Track 00 (Type 1) */
#define ST_LOSTDATA     0x04    /* Bit 2: Lost Data (Type 2/3) */
#define ST_CRCERR       0x08    /* Bit 3: CRC Error */
#define ST_SEEKERR      0x10    /* Bit 4: Seek Error (Type 1) */
#define ST_RNF          0x10    /* Bit 4: Record Not Found (Type 2/3) */
#define ST_HEADLOAD     0x20    /* Bit 5: Head Loaded (Type 1) */
#define ST_WRFAULT      0x20    /* Bit 5: Write Fault (Type 2/3) */
#define ST_WRPROT       0x40    /* Bit 6: Write Protect */
#define ST_NOTREADY     0x80    /* Bit 7: Not Ready */

/* Drive Flags Bit Masks */
#define FLAG_READY      0x01    /* Bit 0: Drive ready (SD card initialized) */
#define FLAG_WRPROT     0x02    /* Bit 1: Write protected */
#define FLAG_TYPE1      0x04    /* Bit 2: Last command was Type 1 */
#define FLAG_SDHC       0x08    /* Bit 3: SDHC card (block addressing) */

#endif /* DISK_DEFS_H */
