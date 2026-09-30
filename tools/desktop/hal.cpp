// Desktop hardware model for the LinnStrument firmware: implements hal.h and the Arduino API
// declared in include/. See hal.h for the model.

// Standard headers first: the Arduino API below defines min/max/abs/round/true/false as macros.
#include <deque>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "hal.h"
#include "Arduino.h"
#include "SPI.h"
#include "DueFlashStorage.h"
#include "malloc.h"

// Hardware facts from the firmware (linnstrument-firmware.ino): Arduino pins used as SPI chip selects.
static const uint8_t PIN_SPI_LEDS = 10;
static const uint8_t PIN_SPI_SENSOR = 4;
static const uint8_t PIN_SPI_ADC = 52;
static const int UART_BUFFER_SIZE = 128;           // SERIAL_BUFFER_SIZE in the SAM core's RingBuffer.h

namespace hal {

Timing timing = {32500, 3000, 100000};

static uint64_t g_nowNs = 0;
static uint32_t g_stalledReads = 0;
static Counters g_counters = {0, 0, 0, 0, 0};

uint64_t nowNs() { return g_nowNs; }

void advanceNs(uint64_t ns) {
  g_nowNs += ns;
  g_stalledReads = 0;
}

// every clock read by the firmware; a firmware busy-wait with no time-consuming call inside it would
// never end on a clock that only moves on hardware events, so after enough reads it creeps forward
static void clockRead() {
  if (g_stalledReads < timing.stallReads) {
    ++g_stalledReads;
  }
  else {
    g_nowNs += 1000;
    g_counters.creepNs += 1000;
  }
}

const Counters& counters() { return g_counters; }

// ---- sensor
static SensorModel g_sensorModel = untouchedSurface;
static void* g_sensorUser = NULL;

void setSensorModel(SensorModel model, void* user) {
  g_sensorModel = model ? model : untouchedSurface;
  g_sensorUser = user;
}

uint16_t untouchedSurface(uint8_t, uint8_t, SensorAxis axis, uint64_t, void*) {
  return axis == SENSOR_Z ? 4095 : 2048;            // no pressure; X and Y at mid-scale
}

// ---- pins
static const int PIN_COUNT = 128;
static int g_inputLevel[PIN_COUNT];
static int g_outputLevel[PIN_COUNT];
static uint8_t g_pinMode[PIN_COUNT];

static void initPins() {
  for (int i = 0; i < PIN_COUNT; ++i) {
    g_inputLevel[i] = HIGH;
    g_outputLevel[i] = LOW;
    g_pinMode[i] = INPUT;
  }
}

static bool validPin(uint32_t pin) { return pin < (uint32_t)PIN_COUNT; }

void setInputPin(uint32_t pin, int level) {
  if (validPin(pin)) g_inputLevel[pin] = level ? HIGH : LOW;
}

int outputPin(uint32_t pin) { return validPin(pin) ? g_outputLevel[pin] : LOW; }

// ---- UART, modelled on the SAM core's UARTClass: a byte goes straight to the transmit holding
// register (THR) when the line is idle, otherwise into a 128-byte ring buffer (127 usable) that the
// TX interrupt empties into the THR each time the previous byte starts shifting out.
static uint32_t g_baud = 0;
static uint64_t g_prevWireStartNs = 0;   // when the previous byte started shifting out
static uint64_t g_lineFreeNs = 0;        // when the shift register finishes the last byte
static std::deque<uint64_t> g_ringLeaveNs;  // for bytes still in the ring buffer: when each moves to the THR
static std::deque<uint8_t> g_rx;
static UartTxHook g_txHook = NULL;
static void* g_txUser = NULL;

static uint64_t byteTimeNs() {
  return g_baud ? (uint64_t)10 * 1000000000ULL / g_baud : 0;   // start bit, 8 data bits, stop bit
}

static void uartExpire() {
  while (!g_ringLeaveNs.empty() && g_ringLeaveNs.front() <= g_nowNs) g_ringLeaveNs.pop_front();
}

static int uartAvailableForWrite() {
  uartExpire();
  return UART_BUFFER_SIZE - 1 - (int)g_ringLeaveNs.size();
}

static void uartWrite(uint8_t b) {
  // the core spins while the ring buffer is full
  if (uartAvailableForWrite() <= 0) {
    uint64_t wait = g_ringLeaveNs.front() - g_nowNs;
    g_counters.uartWaitNs += wait;
    advanceNs(wait);
    uartExpire();
  }
  uint64_t t = g_nowNs;
  uint64_t thrLoad = t > g_prevWireStartNs ? t : g_prevWireStartNs;   // the THR frees when the previous byte starts
  if (thrLoad > t) g_ringLeaveNs.push_back(thrLoad);
  uint64_t wireStart = thrLoad > g_lineFreeNs ? thrLoad : g_lineFreeNs;
  g_prevWireStartNs = wireStart;
  g_lineFreeNs = wireStart + byteTimeNs();
  ++g_counters.uartTxBytes;
  if (g_txHook) g_txHook(t, wireStart, b, g_txUser);
}

void setUartTxHook(UartTxHook hook, void* user) {
  g_txHook = hook;
  g_txUser = user;
}

void uartInject(const uint8_t* data, size_t n) {
  for (size_t i = 0; i < n; ++i) g_rx.push_back(data[i]);
}

uint32_t uartBaud() { return g_baud; }

// ---- flash
static const size_t FLASH_SIZE = 0x40000;
static uint8_t g_flash[FLASH_SIZE];

uint8_t* flash() { return g_flash; }
size_t flashSize() { return FLASH_SIZE; }

static void checkFlashRange(uint32_t address, uint32_t length) {
  if (address > FLASH_SIZE || length > FLASH_SIZE - address) {
    fprintf(stderr, "hal: flash access out of range: address %u, length %u\n", address, length);
    abort();
  }
}

// ---- power-on state, set before any static constructor of the sketch can run
static struct PowerOn {
  PowerOn() {
    memset(g_flash, 0xFF, sizeof(g_flash));
    initPins();
  }
} g_powerOn;

// ---- SPI: decodes the sensor's cell selection and serves ADC reads from the sensor model
static uint8_t g_sensorLsb = 0;
static uint8_t g_selCol = 0, g_selRow = 0;
static SensorAxis g_selAxis = SENSOR_Z;
static uint8_t g_adcLsb = 0;

static uint8_t spiTransfer(uint8_t pin, uint8_t data, SPITransferMode mode) {
  if (pin == PIN_SPI_SENSOR) {
    // selectSensorCell() sends the LSB (row and row switches) first, then the MSB (column and
    // column switches). X: MSB bit 7 and LSB bit 6. Y: MSB bit 6. Z: MSB bit 7 without LSB bit 6.
    if (mode == SPI_CONTINUE) {
      g_sensorLsb = data;
    }
    else {
      g_selCol = data & 0x1F;
      g_selRow = g_sensorLsb & 0x07;
      if (data & 0x40) g_selAxis = SENSOR_Y;
      else if (g_sensorLsb & 0x40) g_selAxis = SENSOR_X;
      else g_selAxis = SENSOR_Z;
    }
    return 0;
  }
  if (pin == PIN_SPI_ADC) {
    // spiAnalogRead(): two transfers; the 12-bit value sits in bits 13..2 of the 16 bits read
    if (mode == SPI_CONTINUE) {
      uint16_t v = g_sensorModel(g_selCol, g_selRow, g_selAxis, g_nowNs, g_sensorUser) & 0x0FFF;
      uint16_t raw = (uint16_t)(v << 2);
      g_adcLsb = raw & 0xFF;
      ++g_counters.adcReads;
      advanceNs(timing.adcReadNs);
      return (uint8_t)(raw >> 8);
    }
    return g_adcLsb;
  }
  if (pin == PIN_SPI_LEDS) {
    ++g_counters.ledSpiBytes;
    return 0;
  }
  return 0;
}

}  // namespace hal

using namespace hal;

// ---------------------------------------------------------------- Arduino API

void yield(void) {}

uint32_t micros(void) {
  clockRead();
  return (uint32_t)(g_nowNs / 1000);
}

uint32_t millis(void) {
  clockRead();
  return (uint32_t)(g_nowNs / 1000000);
}

uint32_t hal::busyWaitMicros() {
  advanceNs(timing.busyWaitPassNs);
  return micros();
}

uint32_t hal::busyWaitMillis() {
  advanceNs(timing.busyWaitPassNs);
  return millis();
}

// the core's delay() spins on the tick count calling yield(), which runs nothing here
void delay(uint32_t dwMs) { advanceNs((uint64_t)dwMs * 1000000); }
void delayMicroseconds(uint32_t dwUs) { advanceNs((uint64_t)dwUs * 1000); }

void pinMode(uint32_t dwPin, uint32_t dwMode) {
  if (validPin(dwPin)) g_pinMode[dwPin] = (uint8_t)dwMode;
}

void digitalWrite(uint32_t dwPin, uint32_t dwVal) {
  if (validPin(dwPin)) g_outputLevel[dwPin] = dwVal ? HIGH : LOW;
}

int digitalRead(uint32_t ulPin) {
  if (!validPin(ulPin)) return LOW;
  return g_pinMode[ulPin] == OUTPUT ? g_outputLevel[ulPin] : g_inputLevel[ulPin];
}

uint32_t analogRead(uint32_t) { return 0; }
void analogWrite(uint32_t, uint32_t) {}
void analogReadResolution(int) {}
void analogWriteResolution(int) {}

// newlib's rand(), so that random() gives the instrument's sequence for a given seed
static uint64_t g_randNext = 1;

static int32_t newlibRand() {
  g_randNext = g_randNext * 6364136223846793005ULL + 1;
  return (int32_t)((g_randNext >> 32) & 0x7FFFFFFF);
}

// WMath.cpp
int32_t random(int32_t howbig) {
  if (howbig == 0) return 0;
  return newlibRand() % howbig;
}

int32_t random(int32_t howsmall, int32_t howbig) {
  if (howsmall >= howbig) return howsmall;
  int32_t diff = howbig - howsmall;
  return random(diff) + howsmall;
}

void randomSeed(uint32_t dwSeed) {
  if (dwSeed != 0) g_randNext = dwSeed;
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void tone(uint32_t, uint32_t, uint32_t) {}
void noTone(uint32_t) {}

// ---- itoa.h
static char* formatUnsigned(uint32_t value, char* string, int radix, bool negative) {
  char tmp[34];
  int n = 0;
  if (radix < 2 || radix > 36) radix = 10;
  do {
    int d = value % radix;
    tmp[n++] = (char)(d < 10 ? '0' + d : 'a' + d - 10);
    value /= radix;
  } while (value);
  char* p = string;
  if (negative) *p++ = '-';
  while (n) *p++ = tmp[--n];
  *p = 0;
  return string;
}

char* itoa(int value, char* string, int radix) {
  return formatUnsigned(value < 0 && radix == 10 ? 0u - (uint32_t)value : (uint32_t)value, string, radix, value < 0 && radix == 10);
}
char* ltoa(int32_t value, char* string, int radix) { return itoa(value, string, radix); }
char* utoa(uint32_t value, char* string, int radix) { return formatUnsigned(value, string, radix, false); }
char* ultoa(uint32_t value, char* string, int radix) { return formatUnsigned(value, string, radix, false); }

// ---- WString.h
void String::set(const char* s, size_t n) {
  buffer = (char*)malloc(n + 1);
  memcpy(buffer, s, n);
  buffer[n] = 0;
  len = (unsigned int)n;
}

void String::append(const char* s, size_t n) {
  char* b = (char*)malloc(len + n + 1);
  memcpy(b, buffer, len);
  memcpy(b + len, s, n);
  b[len + n] = 0;
  free(buffer);
  buffer = b;
  len += (unsigned int)n;
}

String::String(const char* cstr) { set(cstr ? cstr : "", cstr ? strlen(cstr) : 0); }
String::String(const String& str) { set(str.buffer, str.len); }
String::String(char c) { set(&c, 1); }
String::String(unsigned char value, unsigned char base) { char b[34]; utoa(value, b, base); set(b, strlen(b)); }
String::String(int value, unsigned char base) { char b[34]; itoa(value, b, base); set(b, strlen(b)); }
String::String(unsigned int value, unsigned char base) { char b[34]; utoa(value, b, base); set(b, strlen(b)); }
String::String(long value, unsigned char base) { char b[34]; itoa((int)value, b, base); set(b, strlen(b)); }
String::String(unsigned long value, unsigned char base) { char b[34]; utoa((uint32_t)value, b, base); set(b, strlen(b)); }
String::~String() { free(buffer); }

String& String::operator=(const String& rhs) {
  if (this != &rhs) {
    free(buffer);
    set(rhs.buffer, rhs.len);
  }
  return *this;
}

String& String::operator+=(const String& rhs) { append(rhs.buffer, rhs.len); return *this; }
String& String::operator+=(const char* cstr) { append(cstr, strlen(cstr)); return *this; }
bool String::operator==(const String& rhs) const { return len == rhs.len && memcmp(buffer, rhs.buffer, len) == 0; }

String operator+(const String& lhs, const String& rhs) { String s(lhs); s += rhs; return s; }
String operator+(const String& lhs, const char* rhs) { String s(lhs); s += rhs; return s; }
String operator+(const char* lhs, const String& rhs) { String s(lhs); s += rhs; return s; }

// ---- Print.h
size_t Print::write(const uint8_t* buffer, size_t size) {
  size_t n = 0;
  while (size--) n += write(*buffer++);
  return n;
}

size_t Print::printNumber(unsigned long long n, uint8_t base) {
  char buf[8 * sizeof(n) + 1];
  char* str = &buf[sizeof(buf) - 1];
  *str = '\0';
  if (base < 2) base = 10;
  do {
    unsigned long long m = n;
    n /= base;
    char c = (char)(m - base * n);
    *--str = c < 10 ? c + '0' : c + 'A' - 10;
  } while (n);
  return write(str);
}

size_t Print::printSigned(long long n, int base) {
  if (base == 0) return write((uint8_t)n);
  if (base == 10 && n < 0) {
    size_t t = print('-');
    return printNumber(0ULL - (unsigned long long)n, 10) + t;
  }
  return printNumber((unsigned long long)n, (uint8_t)base);
}

size_t Print::print(const String& s) { return write(s.c_str(), s.length()); }
size_t Print::print(const char str[]) { return write(str); }
size_t Print::print(char c) { return write((uint8_t)c); }
size_t Print::print(unsigned char b, int base) { return print((unsigned long)b, base); }
size_t Print::print(int n, int base) { return printSigned(n, base); }
size_t Print::print(unsigned int n, int base) { return print((unsigned long)n, base); }
size_t Print::print(long n, int base) { return printSigned(n, base); }
size_t Print::print(unsigned long n, int base) {
  if (base == 0) return write((uint8_t)n);
  return printNumber(n, (uint8_t)base);
}
size_t Print::print(double number, int digits) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%.*f", digits, number);
  return write(buf);
}

size_t Print::println(void) { return write("\r\n"); }
size_t Print::println(const String& s) { return print(s) + println(); }
size_t Print::println(const char c[]) { return print(c) + println(); }
size_t Print::println(char c) { return print(c) + println(); }
size_t Print::println(unsigned char b, int base) { return print(b, base) + println(); }
size_t Print::println(int num, int base) { return print(num, base) + println(); }
size_t Print::println(unsigned int num, int base) { return print(num, base) + println(); }
size_t Print::println(long num, int base) { return print(num, base) + println(); }
size_t Print::println(unsigned long num, int base) { return print(num, base) + println(); }
size_t Print::println(double num, int digits) { return print(num, digits) + println(); }

// ---- HardwareSerial.h
UARTClass Serial;

void UARTClass::begin(const uint32_t dwBaudRate) {
  g_baud = dwBaudRate;
  g_rx.clear();                        // the core resets both ring buffers
  g_ringLeaveNs.clear();
}
void UARTClass::end(void) { g_baud = 0; }
int UARTClass::available(void) { return (int)g_rx.size(); }
int UARTClass::availableForWrite(void) { return uartAvailableForWrite(); }
int UARTClass::peek(void) { return g_rx.empty() ? -1 : g_rx.front(); }
int UARTClass::read(void) {
  if (g_rx.empty()) return -1;
  uint8_t b = g_rx.front();
  g_rx.pop_front();
  return b;
}
void UARTClass::flush(void) {
  if (g_lineFreeNs > g_nowNs) advanceNs(g_lineFreeNs - g_nowNs);
  uartExpire();
}
size_t UARTClass::write(const uint8_t c) {
  uartWrite(c);
  return 1;
}

// ---- SPI.h
SPIClass SPI;

byte SPIClass::transfer(byte _pin, uint8_t _data, SPITransferMode _mode) {
  return spiTransfer(_pin, _data, _mode);
}

// ---- DueFlashStorage.h
byte DueFlashStorage::read(uint32_t address) {
  checkFlashRange(address, 1);
  return g_flash[address];
}

byte* DueFlashStorage::readAddress(uint32_t address) {
  checkFlashRange(address, 0);
  return g_flash + address;
}

boolean DueFlashStorage::write(uint32_t address, byte value) {
  checkFlashRange(address, 1);
  g_flash[address] = value;
  return true;
}

boolean DueFlashStorage::write(uint32_t address, byte* data, uint32_t dataLength) {
  checkFlashRange(address, dataLength);
  memcpy(g_flash + address, data, dataLength);
  return true;
}

// ---- malloc.h and the linker symbol used by the sketch's debugFreeRam()
struct mallinfo mallinfo(void) {
  struct mallinfo mi;
  memset(&mi, 0, sizeof(mi));
  return mi;
}

char _end = 0;
