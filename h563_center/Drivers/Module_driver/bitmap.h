#ifndef _BITMAP_H
#define _BITMAP_H

#include <stdint.h>

typedef struct BitMap {
    /* 宽度/高度 */
    uint32_t width;
    uint32_t height;

    /* 像素数据,每个像素占据2字节,高字节在前低字节在后 */
    uint8_t *datas;
}BitMap, *P_BitMap;


#endif


