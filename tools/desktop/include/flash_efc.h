// Desktop stand-in for DueFlashStorage's flash_efc.h: only the flash geometry of the SAM3X8E.
#ifndef FLASH_H_INCLUDED
#define FLASH_H_INCLUDED
#define IFLASH0_ADDR      (0x00080000u)
#define IFLASH1_ADDR      (0x000C0000u)
#define IFLASH1_SIZE      (0x40000u)
#define IFLASH0_PAGE_SIZE (256u)
#define IFLASH1_PAGE_SIZE (256u)
#endif
