// Desktop stand-in for the SAM core's HardwareSerial/UARTClass: the one UART that carries both MIDI
// (DIN, or USB through the board's USB-MIDI chip) and serial mode. It's backed by the UART model in
// hal.cpp, which buffers 127 bytes and sends them at the baud rate, like the core's UARTClass.
#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <inttypes.h>
#include "Print.h"

class UARTClass : public Print {
public:
  void begin(const uint32_t dwBaudRate);
  void end(void);
  int available(void);
  int availableForWrite(void);
  int peek(void);
  int read(void);
  void flush(void);
  size_t write(const uint8_t c);
  using Print::write;
  operator bool() { return true; }
};

extern UARTClass Serial;

#endif
