#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <Arduino.h>

// IO pins
#define RESET 10
#define LED_HB 3
#define LED_ERR 2
#define LED_PMODE 4
#define ENABLE_PGM 9
#define TARGET_PWR 8

//#define BAUDRATE 19200 // arduino as ISP default
#define BAUDRATE	115200 // AVR ISP

#define LED_PULSE_TIME 30
#define PROG_FLICKER true

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK 0x10
#define STK_FAILED 0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC 0x14
#define STK_NOSYNC 0x15
#define CRC_EOP 0x20
#define STK_PGM_TYPE "AVR ISP"

// Configure SPI clock (in Hz).
// E.g. for an ATtiny @ 128 kHz: the datasheet states that both the high and low
// SPI clock pulse must be > 2 CPU cycles, so take 3 cycles i.e. divide target
// f_cpu by 6:
//     #define SPI_CLOCK            (128000/6)
//
// A clock slow enough for an ATtiny85 @ 1 MHz, is a reasonable default:

#define SPI_CLOCK (1000000 / 6)

#define EECHUNK (32)

#define beget16(addr) (*addr * 256 + *(addr + 1))

typedef struct param {
  uint8_t devicecode;
  uint8_t revision;
  uint8_t progtype;
  uint8_t parmode;
  uint8_t polling;
  uint8_t selftimed;
  uint8_t lockbytes;
  uint8_t fusebytes;
  uint8_t flashpoll;
  uint16_t eeprompoll;
  uint16_t pagesize;
  uint16_t eepromsize;
  uint32_t flashsize;
} parameter;

parameter param;

#endif