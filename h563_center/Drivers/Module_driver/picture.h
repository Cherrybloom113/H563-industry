
#ifndef _PICTURE_H
#define _PICTURE_H

#include <stdint.h>

typedef struct _HEADCOLOR
{
   uint8_t scan;
   uint8_t gray;
   uint16_t w;
   uint16_t h;
   uint8_t is565;
   uint8_t rgb;
}HEADCOLOR, *P_HEADCOLOR; 

extern const unsigned char gImage_picture[];


#endif /* _PICTURE_H */

