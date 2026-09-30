// Desktop stand-in for the SAM core's Print.h, with the same overload set. The core's `long`
// overloads are kept as `long` so that size_t and ptrdiff_t arguments (64-bit here) still resolve.
#ifndef Print_h
#define Print_h

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "WString.h"

class Print {
public:
  Print() : write_error(0) {}
  virtual ~Print() {}
  int getWriteError() { return write_error; }
  void clearWriteError() { write_error = 0; }

  virtual size_t write(uint8_t) = 0;
  size_t write(const char* str) {
    if (str == NULL) return 0;
    return write((const uint8_t*)str, strlen(str));
  }
  virtual size_t write(const uint8_t* buffer, size_t size);
  size_t write(const char* buffer, size_t size) { return write((const uint8_t*)buffer, size); }

  size_t print(const String&);
  size_t print(const char[]);
  size_t print(char);
  size_t print(unsigned char, int = 10);
  size_t print(int, int = 10);
  size_t print(unsigned int, int = 10);
  size_t print(long, int = 10);
  size_t print(unsigned long, int = 10);
  size_t print(double, int = 2);

  size_t println(const String& s);
  size_t println(const char[]);
  size_t println(char);
  size_t println(unsigned char, int = 10);
  size_t println(int, int = 10);
  size_t println(unsigned int, int = 10);
  size_t println(long, int = 10);
  size_t println(unsigned long, int = 10);
  size_t println(double, int = 2);
  size_t println(void);

private:
  int write_error;
  size_t printNumber(unsigned long long, uint8_t);
  size_t printSigned(long long, int);
};

#endif
