// Desktop stand-in for the SAM core's itoa.h (the core's `long` is int32_t here).
#ifndef _ITOA_
#define _ITOA_
#include <stdint.h>
extern char* itoa(int value, char* string, int radix);
extern char* ltoa(int32_t value, char* string, int radix);
extern char* utoa(uint32_t value, char* string, int radix);
extern char* ultoa(uint32_t value, char* string, int radix);
#endif
