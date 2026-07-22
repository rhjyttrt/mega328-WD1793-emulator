/*
 * WD1773 Floppy Disk Controller Emulator for ATmega328P
 * Project: main/main.ino
 */

#include <Arduino.h>
#include <util/atomic.h>

// #define MCU_FREQ_16MHZ
#define MCU_FREQ_20MHZ

// #define HOST_6502_1000KHZ
#define HOST_6502_1193KHZ

#ifdef MCU_FREQ_16MHZ
  #ifndef F_CPU
    #define F_CPU 16000000UL
  #endif
#elif defined(MCU_FREQ_20MHZ)
  #ifndef F_CPU
    #define F_CPU 20000000UL
  #endif
#endif

#define MAX_SECTOR_SIZE   1024
#define SECTORS_PER_TRACK 18
#define IMAGE_START_LBA   2048UL

extern "C" {
  void init_bus_asm(void);
  uint16_t poll_6502_bus_asm(uint8_t status, uint8_t track, uint8_t sector, uint8_t data);
  uint8_t sd_init(void);
  uint8_t sd_read_floppy_sector(uint32_t image_base_lba, uint32_t floppy_sector_idx, uint16_t sector_size, uint8_t *target_buf);
  uint8_t sd_write_floppy_sector(uint32_t image_base_lba, uint32_t floppy_sector_idx, uint16_t sector_size, const uint8_t *source_buf);
}

volatile uint8_t status_reg = 0x00;
volatile uint8_t track_reg  = 0x00;
volatile uint8_t sector_reg = 0x01;
volatile uint8_t data_reg   = 0x00;

uint8_t head_side          = 0;
uint8_t sector_len_code    = 1;
uint16_t bytes_per_sector  = 256;
bool write_protected       = false;
bool drive_ready           = true;

uint8_t sector_buffer[MAX_SECTOR_SIZE];
uint16_t buffer_idx        = 0;
uint16_t active_transfer_len = 256;

uint8_t stream_current_sector = 1;
uint8_t stream_header_phase   = 0;

enum CommandType { CMD_IDLE, CMD_TYPE1, CMD_READ_SECTOR, CMD_WRITE_SECTOR, CMD_READ_ADDRESS, CMD_READ_TRACK, CMD_WRITE_TRACK };
CommandType active_cmd_type = CMD_IDLE;

uint32_t index_pulse_counter = 0;

uint8_t get_type1_status(void) {
  uint8_t st = 0x00;
  if (!drive_ready) st |= 0x80;
  if (write_protected) st |= 0x40;
  st |= 0x20;
  if (track_reg == 0) st |= 0x04;

  index_pulse_counter++;
  if ((index_pulse_counter & 0x0001FFFFUL) < 0x00000FFFUL) {
    st |= 0x02;
  }
  return st;
}

void run_disk_controller(void) {
  while (1) {
    uint16_t bus_result = poll_6502_bus_asm(status_reg, track_reg, sector_reg, data_reg);

    uint8_t event_type = (bus_result >> 8) & 0xFF;
    uint8_t event_val  = bus_result & 0xFF;

    if (event_type & 0x80) {
      uint8_t target_addr = event_type & 0x03;

      switch (target_addr) {
        case 0x00: {
          uint8_t cmd = event_val;

          PORTC &= ~(1 << PC5);

          if ((cmd & 0x80) == 0x00) {
            uint8_t cmd_type = cmd & 0xF0;

            if (cmd_type == 0x00) {
              track_reg = 0x00;
            } else if (cmd_type == 0x10) {
              track_reg = data_reg;
            } else if (cmd_type == 0x40 || cmd_type == 0x50) {
              if (track_reg < 255) track_reg++;
            } else if (cmd_type == 0x60 || cmd_type == 0x70) {
              if (track_reg > 0) track_reg--;
            }

            active_cmd_type = CMD_TYPE1;
            status_reg = get_type1_status();
            PORTC &= ~(1 << PC4);
            PORTC |=  (1 << PC5);
          }
          else if ((cmd & 0xE0) == 0x80) {
            bytes_per_sector = 128 << sector_len_code;
            active_transfer_len = bytes_per_sector;

            active_cmd_type = CMD_READ_SECTOR;
            status_reg = 0x01;

            uint8_t sec_offset = (sector_reg > 0) ? (sector_reg - 1) : 0;
            uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + sec_offset;

            sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer);

            buffer_idx = 0;
            data_reg = sector_buffer[buffer_idx++];
            status_reg &= ~0x01;
            status_reg |= 0x02;
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xE0) == 0xA0) {
            if (write_protected) {
              status_reg = 0x40;
              PORTC |= (1 << PC5);
              break;
            }

            bytes_per_sector = 128 << sector_len_code;
            active_transfer_len = bytes_per_sector;

            active_cmd_type = CMD_WRITE_SECTOR;
            buffer_idx = 0;
            status_reg = 0x03;
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xF0) == 0xC0) {
            active_cmd_type = CMD_READ_ADDRESS;
            status_reg = 0x03;
            
            sector_buffer[0] = track_reg;
            sector_buffer[1] = head_side;
            sector_buffer[2] = sector_reg;
            sector_buffer[3] = sector_len_code;
            sector_buffer[4] = 0x12;
            sector_buffer[5] = 0x34;

            buffer_idx = 0;
            active_transfer_len = 6;
            data_reg = sector_buffer[buffer_idx++];
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xF0) == 0xE0) {
            active_cmd_type = CMD_READ_TRACK;
            status_reg = 0x03;

            bytes_per_sector = 128 << sector_len_code;
            stream_current_sector = 1;
            stream_header_phase = 0;

            uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK);
            sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer);

            buffer_idx = 0;
            data_reg = 0x00;
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xF0) == 0xF0) {
            if (write_protected) {
              status_reg = 0x40;
              PORTC |= (1 << PC5);
              break;
            }

            active_cmd_type = CMD_WRITE_TRACK;
            buffer_idx = 0;
            bytes_per_sector = 128 << sector_len_code;
            active_transfer_len = (uint16_t)SECTORS_PER_TRACK * bytes_per_sector;
            status_reg = 0x03;
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xF0) == 0xD0) {
            active_cmd_type = CMD_IDLE;
            status_reg &= ~0x03;
            status_reg |= get_type1_status();
            PORTC &= ~(1 << PC4);
            PORTC |=  (1 << PC5);
          }
          break;
        }

        case 0x01: track_reg = event_val; break;
        case 0x02: sector_reg = event_val; break;

        case 0x03:
          if (active_cmd_type == CMD_WRITE_SECTOR) {
            if (!(status_reg & 0x02)) {
              status_reg |= 0x04;
            }

            data_reg = event_val;
            if (buffer_idx < active_transfer_len) {
              sector_buffer[buffer_idx++] = data_reg;
              
              if (buffer_idx < active_transfer_len) {
                status_reg |= 0x02;
                PORTC |= (1 << PC4);
              } else {
                status_reg &= ~0x02;
                PORTC &= ~(1 << PC4);

                uint8_t sec_offset = (sector_reg > 0) ? (sector_reg - 1) : 0;
                uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + sec_offset;
                
                sd_write_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer);

                active_cmd_type = CMD_IDLE;
                status_reg &= ~0x01;
                PORTC |= (1 << PC5);
              }
            }
          } 
          else if (active_cmd_type == CMD_WRITE_TRACK) {
            data_reg = event_val;
            if (buffer_idx < active_transfer_len) {
              buffer_idx++;
              status_reg |= 0x02;
              PORTC |= (1 << PC4);
            } 
            
            if (buffer_idx >= active_transfer_len) {
              for (uint16_t z = 0; z < bytes_per_sector; z++) sector_buffer[z] = 0xE5;
              
              for (uint8_t s = 1; s <= SECTORS_PER_TRACK; s++) {
                uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + (s - 1);
                sd_write_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer);
              }

              active_cmd_type = CMD_IDLE;
              status_reg &= ~0x03;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
            }
          } else {
            data_reg = event_val;
            status_reg &= ~0x02;
            PORTC &= ~(1 << PC4);
          }
          break;
      }
    }
    else if (event_type & 0x40) {
      uint8_t target_addr = event_type & 0x03;

      if (target_addr == 0x00) {
        PORTC &= ~(1 << PC5);
        if (active_cmd_type == CMD_TYPE1 || active_cmd_type == CMD_IDLE) {
          status_reg = get_type1_status();
        }
      }
      else if (target_addr == 0x03 && active_cmd_type != CMD_WRITE_SECTOR) {
        if (active_cmd_type == CMD_READ_TRACK) {
          switch (stream_header_phase) {
            case 0: case 1: case 2: case 3: case 4:
              data_reg = 0x00;
              stream_header_phase++;
              break;
            case 5:
              data_reg = 0xFE;
              stream_header_phase++;
              break;
            case 6:
              data_reg = track_reg;
              stream_header_phase++;
              break;
            case 7:
              data_reg = head_side;
              stream_header_phase++;
              break;
            case 8:
              data_reg = stream_current_sector;
              stream_header_phase++;
              break;
            case 9:
              data_reg = sector_len_code;
              stream_header_phase++;
              break;
            case 10:
              data_reg = 0x12;
              stream_header_phase++;
              break;
            case 11:
              data_reg = 0x34;
              stream_header_phase++;
              break;
            case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19: case 20:
              data_reg = 0x4E;
              stream_header_phase++;
              break;
            case 21:
              data_reg = 0xFB;
              stream_header_phase++;
              buffer_idx = 0;
              break;
            case 22:
              if (buffer_idx < bytes_per_sector) {
                data_reg = sector_buffer[buffer_idx++];
              } else {
                data_reg = 0x56;
                stream_header_phase++;
              }
              break;
            case 23:
              data_reg = 0x78;
              stream_current_sector++;

              if (stream_current_sector <= SECTORS_PER_TRACK) {
                uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + (stream_current_sector - 1);
                sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer);
                stream_header_phase = 0;
              } else {
                active_cmd_type = CMD_IDLE;
                status_reg &= ~0x03;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                continue;
              }
              break;
          }

          status_reg |= 0x02;
          PORTC |= (1 << PC4);
        } 
        else {
          if (!(status_reg & 0x02) && (buffer_idx < active_transfer_len)) {
            status_reg |= 0x04;
          }

          if (buffer_idx < active_transfer_len) {
            data_reg = sector_buffer[buffer_idx++];
            status_reg |= 0x02;
            PORTC |= (1 << PC4);
          } else {
            active_cmd_type = CMD_IDLE;
            status_reg &= ~0x03;
            PORTC &= ~(1 << PC4);
            PORTC |=  (1 << PC5);
          }
        }
      }
    }
  }
}

void setup() {
  TIMSK0 = 0;
  init_bus_asm();

  if (sd_init() != 0) {
    while (1);
  }

  run_disk_controller();
}

void loop() {

}