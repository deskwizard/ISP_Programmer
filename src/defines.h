#ifndef _DEFINES_H_
#define _DEFINES_H_

#include <Arduino.h>


void handleVoltageRead();
void handleLEDs();
void avrisp();

// For programmer target power state and LEDs
#define ON  LOW
#define OFF HIGH

#define VTARGET_PIN A5
#define VREAD_COUNT 10 // Number of readings to take
#define VREAD_RATE 10 // In ms

// IO pins
// Error and OK LEDs are a dual color R/G LED
#define LED_ERROR 2
#define LED_HB 3
#define LED_OK 4
#define TARGET_PWR 8
#define ENABLE_PGM 9
#define RESET 10

//#define BAUDRATE 19200 // arduino as ISP default
#define BAUDRATE	115200 // AVR ISP

#define LED_PULSE_TIME 100
#define LED_MIN_PWM 16
#define LED_MAX_PWM 192

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