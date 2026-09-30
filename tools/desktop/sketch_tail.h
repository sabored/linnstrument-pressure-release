// Included at the end of the desktop build's sketch translation unit, after all firmware code.
// The harness is compiled after this point, with the Arduino macros removed so that it can use the
// C++ standard library freely. It has access to all of the firmware's globals and functions.
#undef min
#undef max
#undef abs
#undef constrain
#undef round
#undef true
#undef false
#undef radians
#undef degrees
#undef sq
#undef bit
#undef lowByte
#undef highByte
#undef strchr
#undef strrchr
#undef strstr
#undef strpbrk
#undef memchr
