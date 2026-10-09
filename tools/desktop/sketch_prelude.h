// Included first in the desktop build's sketch translation unit, before the sketch's own
// #include <Arduino.h>. Standard headers the HAL and harnesses need must come before the Arduino API
// defines min/max/abs/round/true/false as macros.
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>
#include <deque>
#include <map>
#include <algorithm>
#include <cctype>
#include "hal.h"

// newlib's <string.h> declares the plain C functions, whose result isn't const even for a const
// argument, and the sketch relies on that (char* p = strchr(font->chars, c)). The C++ library adds
// const overloads instead; these macros give the sketch the C behaviour. sketch_tail.h removes them.
#define strchr(s, c) ((char*)::strchr((s), (c)))
#define strrchr(s, c) ((char*)::strrchr((s), (c)))
#define strstr(s, t) ((char*)::strstr((s), (t)))
#define strpbrk(s, t) ((char*)::strpbrk((s), (t)))
#define memchr(s, c, n) ((void*)::memchr((s), (c), (n)))

// newlib's integer-only snprintf, which the sketch uses for formats with no floating-point conversion.
// This C library doesn't have it; its snprintf gives the same output for such formats.
inline int sniprintf(char* str, size_t size, const char* format, ...) {
  va_list args;
  va_start(args, format);
  int n = vsnprintf(str, size, format, args);
  va_end(args);
  return n;
}
