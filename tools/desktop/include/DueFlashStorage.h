// Desktop stand-in for the DueFlashStorage library: the SAM3X8E's second flash bank (0xC0000, 256 KB),
// where the settings live, as a byte array in hal.cpp. Addresses are offsets into that bank.
#ifndef DUEFLASHSTORAGE_H
#define DUEFLASHSTORAGE_H

#include <Arduino.h>
#include "flash_efc.h"
#include "efc.h"

#define DATA_LENGTH   ((IFLASH1_PAGE_SIZE/sizeof(byte))*4)

class DueFlashStorage {
public:
  byte read(uint32_t address);
  byte* readAddress(uint32_t address);
  boolean write(uint32_t address, byte value);
  boolean write(uint32_t address, byte* data, uint32_t dataLength);
};

#endif
