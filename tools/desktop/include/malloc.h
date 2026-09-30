// Desktop stand-in for newlib's malloc.h, used only by the sketch's debugFreeRam().
#ifndef _DESKTOP_MALLOC_H_
#define _DESKTOP_MALLOC_H_
#include <stddef.h>
struct mallinfo {
  size_t arena, ordblks, smblks, hblks, hblkhd, usmblks, fsmblks, uordblks, fordblks, keepcost;
};
struct mallinfo mallinfo(void);
#endif
