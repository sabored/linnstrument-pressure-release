// Desktop stand-in for the SAM core's WString.h. The sketch only builds messages from literals and
// numbers ("..." + String(n)) and reads them back with c_str(), so this is a small subset.
#ifndef String_class_h
#define String_class_h
#ifdef __cplusplus

#include <stdint.h>
#include <stddef.h>

class __FlashStringHelper;

class String {
public:
  String(const char* cstr = "");
  String(const String& str);
  explicit String(char c);
  explicit String(unsigned char value, unsigned char base = 10);
  explicit String(int value, unsigned char base = 10);
  explicit String(unsigned int value, unsigned char base = 10);
  explicit String(long value, unsigned char base = 10);
  explicit String(unsigned long value, unsigned char base = 10);
  ~String();
  String& operator=(const String& rhs);
  String& operator+=(const String& rhs);
  String& operator+=(const char* cstr);
  unsigned int length() const { return len; }
  const char* c_str() const { return buffer; }
  char operator[](unsigned int index) const { return index < len ? buffer[index] : 0; }
  bool operator==(const String& rhs) const;
  bool operator!=(const String& rhs) const { return !(*this == rhs); }
private:
  void set(const char* s, size_t n);
  void append(const char* s, size_t n);
  char* buffer;
  unsigned int len;
};

String operator+(const String& lhs, const String& rhs);
String operator+(const String& lhs, const char* rhs);
String operator+(const char* lhs, const String& rhs);

#endif
#endif
