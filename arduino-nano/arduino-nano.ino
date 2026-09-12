#include <Arduino.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define MAX_SECTOR_SIZE   1024
#define SECTORS_PER_TRACK 18
#define IMAGE_START_LBA   2048UL

extern "C" {
  void init_bus_asm(void);
  uint16_t poll_6502_bus_asm(uint8_t status, uint8_t track, uint8_t sector, uint8_t data);
  uint16_t stream_read_sector_asm(uint8_t *buf, uint16_t len);
  uint16_t stream_write_sector_asm(uint8_t *buf, uint16_t len);
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
bool multi_sector          = false;
int8_t step_direction      = 1;
bool last_cmd_is_type1     = true;

uint8_t sector_buffer[MAX_SECTOR_SIZE];
uint16_t buffer_idx        = 0;
uint16_t active_transfer_len = 256;

uint8_t stream_current_sector = 1;
uint8_t stream_header_phase   = 0;

enum CommandType { CMD_IDLE, CMD_TYPE1, CMD_READ_SECTOR, CMD_WRITE_SECTOR, CMD_READ_ADDRESS, CMD_READ_TRACK, CMD_WRITE_TRACK };
CommandType active_cmd_type = CMD_IDLE;

uint32_t index_pulse_counter = 0;
uint8_t pending_cmd       = 0x00;
bool has_pending_cmd       = false;

uint8_t get_type1_status(void) {
  uint8_t st = 0x00;
  if (!drive_ready) st |= 0x80;
  if (write_protected) st |= 0x40;
  st |= 0x20;
  if (track_reg == 0) st |= 0x04;

  index_pulse_counter++;
  if ((index_pulse_counter & 0x3FFF) < 0x0200) {
    st |= 0x02;
  }
  return st;
}

void run_disk_controller(void) {
  while (1) {
    uint8_t event_type;
    uint8_t event_val;

    if (has_pending_cmd) {
      has_pending_cmd = false;
      event_type = 0x80;
      event_val  = pending_cmd;
    } else {
      uint16_t bus_result = poll_6502_bus_asm(status_reg, track_reg, sector_reg, data_reg);
      event_type = (bus_result >> 8) & 0xFF;
      event_val  = bus_result & 0xFF;
    }

    if (event_type & 0x80) {
      uint8_t target_addr = event_type & 0x03;

      switch (target_addr) {
        case 0x00: {
          uint8_t cmd = event_val;

          if ((status_reg & 0x01) && ((cmd & 0xF0) != 0xD0)) {
            break;
          }

          PORTC &= ~(1 << PC5);

          if ((cmd & 0x80) == 0x00) {
            last_cmd_is_type1 = true;
            multi_sector = false;
            if (!drive_ready) {
              if (sd_init() == 0) drive_ready = true;
            }
            uint8_t cmd_type = cmd & 0xF0;

            if (cmd_type == 0x00) {
              track_reg = 0x00;
              step_direction = -1;
            } else if (cmd_type == 0x10) {
              if (data_reg > track_reg) {
                step_direction = 1;
              } else if (data_reg < track_reg) {
                step_direction = -1;
              }
              track_reg = data_reg;
            } else if (cmd_type == 0x20 || cmd_type == 0x30) {
              if (cmd & 0x10) {
                if (step_direction > 0) {
                  if (track_reg < 255) track_reg++;
                } else {
                  if (track_reg > 0) track_reg--;
                }
              }
            } else if (cmd_type == 0x40 || cmd_type == 0x50) {
              step_direction = 1;
              if (cmd & 0x10) {
                if (track_reg < 255) track_reg++;
              }
            } else if (cmd_type == 0x60 || cmd_type == 0x70) {
              step_direction = -1;
              if (cmd & 0x10) {
                if (track_reg > 0) track_reg--;
              }
            }

            active_cmd_type = CMD_TYPE1;
            status_reg = get_type1_status();
            PORTC &= ~(1 << PC4);
            PORTC |=  (1 << PC5);
          }
          else if ((cmd & 0xE0) == 0x80) {
            last_cmd_is_type1 = false;
            multi_sector = (cmd & 0x10) != 0;
            head_side = (cmd & 0x08) ? 1 : 0;
            if (!drive_ready) {
              if (sd_init() == 0) {
                drive_ready = true;
              } else {
                active_cmd_type = CMD_IDLE;
                status_reg = 0x80;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            }

            if (sector_reg == 0 || sector_reg > SECTORS_PER_TRACK) {
              active_cmd_type = CMD_IDLE;
              status_reg = 0x10;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
              break;
            }

            bytes_per_sector = 128 << (sector_len_code & 0x03);
            if (bytes_per_sector > MAX_SECTOR_SIZE) bytes_per_sector = MAX_SECTOR_SIZE;

            do {
              uint8_t sec_offset = sector_reg - 1;
              uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + sec_offset;

              if (sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer) != 0) {
                status_reg = 0x10;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }

              status_reg = 0x03;
              uint16_t stream_res = stream_read_sector_asm(sector_buffer, bytes_per_sector);
              if (stream_res & 0xFF00) {
                active_cmd_type = CMD_IDLE;
                status_reg &= ~0x03;
                PORTC &= ~(1 << PC4);
                has_pending_cmd = true;
                pending_cmd = stream_res & 0xFF;
                break;
              }

              if (multi_sector && (sector_reg < SECTORS_PER_TRACK)) {
                sector_reg++;
              } else {
                if (multi_sector) sector_reg++;
                status_reg &= ~0x03;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            } while (multi_sector);

            active_cmd_type = CMD_IDLE;
          }
          else if ((cmd & 0xE0) == 0xA0) {
            last_cmd_is_type1 = false;
            multi_sector = (cmd & 0x10) != 0;
            head_side = (cmd & 0x08) ? 1 : 0;
            if (write_protected) {
              active_cmd_type = CMD_IDLE;
              status_reg = 0x40;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
              break;
            }

            if (!drive_ready) {
              if (sd_init() == 0) {
                drive_ready = true;
              } else {
                active_cmd_type = CMD_IDLE;
                status_reg = 0x80;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            }

            if (sector_reg == 0 || sector_reg > SECTORS_PER_TRACK) {
              active_cmd_type = CMD_IDLE;
              status_reg = 0x10;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
              break;
            }

            bytes_per_sector = 128 << (sector_len_code & 0x03);
            if (bytes_per_sector > MAX_SECTOR_SIZE) bytes_per_sector = MAX_SECTOR_SIZE;

            do {
              status_reg = 0x03;
              uint16_t stream_res = stream_write_sector_asm(sector_buffer, bytes_per_sector);
              if (stream_res & 0xFF00) {
                active_cmd_type = CMD_IDLE;
                status_reg &= ~0x03;
                PORTC &= ~(1 << PC4);
                has_pending_cmd = true;
                pending_cmd = stream_res & 0xFF;
                break;
              }

              uint8_t sec_offset = sector_reg - 1;
              uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + sec_offset;

              if (sd_write_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer) != 0) {
                status_reg = 0x20;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }

              if (multi_sector && (sector_reg < SECTORS_PER_TRACK)) {
                sector_reg++;
              } else {
                if (multi_sector) sector_reg++;
                status_reg &= ~0x03;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            } while (multi_sector);

            active_cmd_type = CMD_IDLE;
          }
          else if ((cmd & 0xF0) == 0xC0) {
            last_cmd_is_type1 = false;
            multi_sector = false;
            if (!drive_ready) {
              if (sd_init() == 0) {
                drive_ready = true;
              } else {
                active_cmd_type = CMD_IDLE;
                status_reg = 0x80;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            }

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
            last_cmd_is_type1 = false;
            multi_sector = false;
            if (!drive_ready) {
              if (sd_init() == 0) {
                drive_ready = true;
              } else {
                active_cmd_type = CMD_IDLE;
                status_reg = 0x80;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            }

            bytes_per_sector = 128 << (sector_len_code & 0x03);
            if (bytes_per_sector > MAX_SECTOR_SIZE) bytes_per_sector = MAX_SECTOR_SIZE;
            stream_current_sector = 1;
            stream_header_phase = 0;

            uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK);
            if (sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer) != 0) {
              active_cmd_type = CMD_IDLE;
              status_reg = 0x10;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
              break;
            }

            active_cmd_type = CMD_READ_TRACK;
            status_reg = 0x03;
            buffer_idx = 0;
            data_reg = 0x00;
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xF0) == 0xF0) {
            last_cmd_is_type1 = false;
            multi_sector = false;
            if (write_protected) {
              active_cmd_type = CMD_IDLE;
              status_reg = 0x40;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
              break;
            }

            if (!drive_ready) {
              if (sd_init() == 0) {
                drive_ready = true;
              } else {
                active_cmd_type = CMD_IDLE;
                status_reg = 0x80;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
                break;
              }
            }

            active_cmd_type = CMD_WRITE_TRACK;
            buffer_idx = 0;
            bytes_per_sector = 128 << (sector_len_code & 0x03);
            if (bytes_per_sector > MAX_SECTOR_SIZE) bytes_per_sector = MAX_SECTOR_SIZE;
            active_transfer_len = (uint16_t)SECTORS_PER_TRACK * bytes_per_sector;
            status_reg = 0x03;
            PORTC |= (1 << PC4);
          }
          else if ((cmd & 0xF0) == 0xD0) {
            last_cmd_is_type1 = true;
            multi_sector = false;
            active_cmd_type = CMD_IDLE;
            status_reg = get_type1_status();
            PORTC &= ~(1 << PC4);
            if (cmd & 0x08) {
              PORTC |= (1 << PC5);
            } else {
              PORTC &= ~(1 << PC5);
            }
          }
          break;
        }

        case 0x01:
          if (!(status_reg & 0x01)) {
            track_reg = event_val;
            if (last_cmd_is_type1) {
              status_reg = get_type1_status();
            }
          }
          break;
        case 0x02:
          if (!(status_reg & 0x01)) sector_reg = event_val;
          break;

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
                
                uint8_t wr_res = sd_write_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer);

                if (wr_res != 0) {
                  active_cmd_type = CMD_IDLE;
                  status_reg = 0x20;
                  PORTC |= (1 << PC5);
                } else if (multi_sector && (sector_reg < SECTORS_PER_TRACK)) {
                  sector_reg++;
                  buffer_idx = 0;
                  status_reg = 0x03;
                  PORTC |= (1 << PC4);
                } else {
                  if (multi_sector) sector_reg++;
                  active_cmd_type = CMD_IDLE;
                  status_reg &= ~0x03;
                  PORTC |= (1 << PC5);
                }
              }
            }
          } 
          else if (active_cmd_type == CMD_WRITE_TRACK) {
            data_reg = event_val;
            buffer_idx++;
            if (buffer_idx < active_transfer_len) {
              status_reg |= 0x02;
              PORTC |= (1 << PC4);
            } else {
              status_reg &= ~0x02;
              PORTC &= ~(1 << PC4);

              for (uint16_t z = 0; z < bytes_per_sector; z++) sector_buffer[z] = 0xE5;
              
              uint8_t fmt_err = 0;
              for (uint8_t s = 1; s <= SECTORS_PER_TRACK; s++) {
                uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + (s - 1);
                if (sd_write_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer) != 0) {
                  fmt_err = 1;
                }
              }

              active_cmd_type = CMD_IDLE;
              status_reg = fmt_err ? 0x20 : 0x00;
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
        if (last_cmd_is_type1) {
          status_reg = get_type1_status();
        }
      }
      else if (target_addr == 0x03) {
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
              stream_header_phase++;
              break;
            case 24:
              stream_current_sector++;

              if (stream_current_sector <= SECTORS_PER_TRACK) {
                uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + (stream_current_sector - 1);
                if (sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer) != 0) {
                  active_cmd_type = CMD_IDLE;
                  status_reg = 0x10;
                  PORTC &= ~(1 << PC4);
                  PORTC |=  (1 << PC5);
                  continue;
                }
                stream_header_phase = 0;
                data_reg = 0x00;
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
        else if (active_cmd_type == CMD_READ_SECTOR || active_cmd_type == CMD_READ_ADDRESS) {
          if (!(status_reg & 0x02) && (buffer_idx < active_transfer_len)) {
            status_reg |= 0x04;
          }

          if (buffer_idx < active_transfer_len) {
            data_reg = sector_buffer[buffer_idx++];
            status_reg |= 0x02;
            PORTC |= (1 << PC4);
          } else {
            if (multi_sector && (sector_reg < SECTORS_PER_TRACK)) {
              sector_reg++;
              uint8_t sec_offset = sector_reg - 1;
              uint32_t floppy_idx = ((uint32_t)track_reg * SECTORS_PER_TRACK) + sec_offset;
              if (sd_read_floppy_sector(IMAGE_START_LBA, floppy_idx, bytes_per_sector, sector_buffer) != 0) {
                active_cmd_type = CMD_IDLE;
                status_reg = 0x10;
                PORTC &= ~(1 << PC4);
                PORTC |=  (1 << PC5);
              } else {
                buffer_idx = 0;
                data_reg = sector_buffer[buffer_idx++];
                status_reg = 0x03;
                PORTC |= (1 << PC4);
              }
            } else {
              if (multi_sector) sector_reg++;
              active_cmd_type = CMD_IDLE;
              status_reg &= ~0x03;
              PORTC &= ~(1 << PC4);
              PORTC |=  (1 << PC5);
            }
          }
        }
        else {
          status_reg &= ~0x02;
          PORTC &= ~(1 << PC4);
        }
      }
    }
  }
}

void setup() {
  TIMSK0 = 0;
  UCSR0B = 0;
  cli();

  init_bus_asm();

  if (sd_init() != 0) {
    drive_ready = false;
  } else {
    drive_ready = true;
  }
  status_reg = get_type1_status();

  run_disk_controller();
}

void loop() {
}
