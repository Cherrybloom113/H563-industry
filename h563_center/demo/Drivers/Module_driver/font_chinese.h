#ifndef _FONT_CHINESE_H
#define _FONT_CHINESE_H

#include <stdint.h>

// ------------------  汉字字模的数据结构定义 ------------------------ //
typedef struct  Cn32CharTypeDef                   // 汉字字模数据结构 
{
	uint8_t Index[4];            // 汉字utf8编码, 占据3字节, 连同结束字符共4字节	
	uint8_t Msk[116];            // 点阵码数据(32*29/8) 
}Cn32CharTypeDef, *P_Cn32CharTypeDef;

extern  Cn32CharTypeDef const CnChar32x29[];

#endif /* _FONT_CHINESE_H */

