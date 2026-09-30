// Desktop stand-in for the SAM core's SPI library. Transfers go to the sensor and LED model in hal.cpp.
#ifndef _SPI_H_INCLUDED
#define _SPI_H_INCLUDED

#include "Arduino.h"

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

enum SPITransferMode {
  SPI_CONTINUE,
  SPI_LAST
};

#define SPI_CLOCK_DIV2   11
#define SPI_CLOCK_DIV4   21
#define SPI_CLOCK_DIV8   42
#define SPI_CLOCK_DIV16  84
#define SPI_CLOCK_DIV32  168
#define SPI_CLOCK_DIV64  255
#define SPI_CLOCK_DIV128 255

class SPIClass {
public:
  byte transfer(byte _pin, uint8_t _data, SPITransferMode _mode = SPI_LAST);
  void begin(void) {}
  void end(void) {}
  void begin(uint8_t) {}
  void end(uint8_t) {}
  void setBitOrder(uint8_t, BitOrder) {}
  void setDataMode(uint8_t, uint8_t) {}
  void setClockDivider(uint8_t, uint8_t) {}
};

extern SPIClass SPI;

#endif
