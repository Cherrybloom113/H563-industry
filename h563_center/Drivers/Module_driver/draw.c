// SPDX-License-Identifier: GPL-3.0-only
/*
 * Copyright (c) 2008-2023 100askTeam : Dongshan WEI <weidongshan@qq.com> 
 * Discourse:  https://forums.100ask.net
 */
 
/*  Copyright (C) 2008-2023 深圳百问网科技有限公司
 *  All rights reserved
 *
 * 免责声明: 百问网编写的文档, 仅供学员学习使用, 可以转发或引用(请保留作者信息),禁止用于商业用途！
 * 免责声明: 百问网编写的程序, 可以用于商业用途, 但百问网不承担任何后果！
 * 
 * 本程序遵循GPL V3协议, 请遵循协议
 * 百问网学习平台   : https://www.100ask.net
 * 百问网交流社区   : https://forums.100ask.net
 * 百问网官方B站    : https://space.bilibili.com/275908810
 * 本程序所用开发板 : DShanMCU-F103
 * 百问网官方淘宝   : https://100ask.taobao.com
 * 联系我们(E-mail): weidongshan@qq.com
 *
 *          版权所有，盗版必究。
 *  
 * 修改历史     版本号           作者        修改内容
 *-----------------------------------------------------
 * 2024.02.01      v01         百问科技      创建文件
 *-----------------------------------------------------
 */
 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "draw.h"
#include "bitmap.h"
#include "font_chinese.h"
#include "picture.h"
#include "spi_lcd.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"


extern const unsigned char fontdata_8x16[];

static uint32_t g_lcd_width, g_lcd_height;
static SemaphoreHandle_t DrawMutex;


/**********************************************************************
 * 函数名称： RGB888_To_LCDRGB565
 * 功能描述： 把RGB888的颜色转换为RGB565格式,大端对齐
 * 输入参数： dwColor, 32bit的颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 大字节序的16bit颜色
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
static uint16_t RGB888_To_LCDRGB565(uint32_t dwColor)
{
    uint32_t r = (dwColor >> 16) & 0xff;
    uint32_t g = (dwColor >> 8) & 0xff;
    uint32_t b = (dwColor >> 0) & 0xff;

    r = r >> 3;
    g = g >> 2;
    b = b >> 2;

    uint16_t wColor = (r << 11) | (g << 5) | b;

    /* SPI传输时,先传的是高位数据, 所以把高低字节互换位置 */
    wColor = (wColor >> 8) | (wColor << 8);
    return wColor;
}

/**********************************************************************
 * 函数名称： Draw_Init
 * 功能描述： Draw初始化,得到LCD的分辨率
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Init(void)
{
	/* create mutex */
	DrawMutex = xSemaphoreCreateMutex();
	
    LCD_GetInfo(&g_lcd_width, &g_lcd_height);
}

/**********************************************************************
 * 函数名称： Draw_Clear
 * 功能描述： 把屏幕清屏为某种颜色
 * 输入参数： dwColor, 颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Clear(uint32_t dwColor)
{
    uint16_t wColor = RGB888_To_LCDRGB565(dwColor);

    LCD_SetWindows(0, 0, g_lcd_width-1, g_lcd_height-1);
    LCD_SetDataLine();
    
    for(uint32_t x = 0; x < g_lcd_width; x++)
        for(uint32_t y = 0; y < g_lcd_height; y++)
            LCD_WriteDatas((uint8_t *)&wColor, 2);
}


/**********************************************************************
 * 函数名称： Draw_Region
 * 功能描述： 在指定位置绘制位图
 * 输入参数： x,y - 左上角坐标
 *            ptBitMap - 位图信息,里面含有位图的宽、高、数据
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
static void Draw_Region(uint32_t x, uint32_t y, P_BitMap ptBitMap)
{   
	/* get mutex */
	xSemaphoreTake(DrawMutex, portMAX_DELAY);
	
    /* 设置要显示的区域 */
    LCD_SetWindows(x, y, x + ptBitMap->width - 1, y + ptBitMap->height - 1);

    /* 设置D/C引脚表示后面要发送数据 */
    LCD_SetDataLine();
    
    /* 发送数据 */
    LCD_WriteDatas(ptBitMap->datas, ptBitMap->height * ptBitMap->width * 2);

	/* give mutex */
	xSemaphoreGive(DrawMutex);
}

/**********************************************************************
 * 函数名称： Draw_Pixel
 * 功能描述： 在指定位置像素颜色
 * 输入参数： x,y - 左上角坐标
 *            dwColor - 颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Pixel(int x, int y, uint32_t dwColor)
{
    uint16_t wColor = RGB888_To_LCDRGB565(dwColor);
    BitMap bitmap; 

    bitmap.width = 1;
    bitmap.height = 1;
    bitmap.datas = (uint8_t *)&wColor;
    
    Draw_Region(x, y, &bitmap);
}

/**********************************************************************
 * 函数名称： Draw_Pixel_In_Buf
 * 功能描述： 把像素颜色先填入buf
 * 输入参数： buf - 存储数据的buf
 *            x,y - 左上角坐标
 *            w,h - buf的分辨率
 *            dwColor - 颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Pixel_In_Buf(uint8_t *buf, int x, int y, int w, int h, uint32_t dwColor)
{
    uint16_t *tmpBuf = (uint16_t *)buf;
    
    dwColor = RGB888_To_LCDRGB565(dwColor);
    tmpBuf[y*w + x] = dwColor;    
}

/**********************************************************************
 * 函数名称： Draw_ASCII
 * 功能描述： 绘制ASCII字符
 * 输入参数： x,y - 左上角坐标
 *            c   - 字符
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 *            back_color  - 背景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 字符宽度(单位:像素)
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
int Draw_ASCII(uint32_t x, uint32_t y, char c, uint32_t front_color, uint32_t back_color)
{
    uint32_t color;

    uint16_t buf[8*16];

    int i, bit;
    unsigned char data;

    BitMap bitmap; 

    front_color = RGB888_To_LCDRGB565(front_color);
    back_color = RGB888_To_LCDRGB565(back_color);

    /* 8x16的点阵 */
    for (i = 0; i < 16; i++)
    {
        data = fontdata_8x16[c*16+i];
        for (bit = 7; bit >= 0; bit--)
        {
            if (data & (1<<bit))
            {
                color = front_color;
            }
            else
            {
                color = back_color;
            }
            
            buf[i * 8 + (7-bit)] = color;
        }
    }

    bitmap.width = 8;
    bitmap.height = 16;    
    bitmap.datas = (uint8_t *)buf;
    
    Draw_Region(x, y, &bitmap);

    return 8; /* 这个字符宽度为8 */
}

/**********************************************************************
 * 函数名称： Draw_String
 * 功能描述： 绘制ASCII字符串
 * 输入参数： x,y - 左上角坐标
 *            str - 字符串
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 *            back_color  - 背景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 字符串宽度(单位:像素)
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
int Draw_String(uint32_t x, uint32_t y, char *str, uint32_t front_color, uint32_t back_color)
{
    int i;
        int line_width = 0;
    for (i = 0; i < strlen(str); i++)
    {
        if (str[i] == '\r')
            x = 0;
        else if (str[i] == '\n')
            y += 16;
        else if (str[i] == '\b')
        {
            if (x >= 8)
                x -= 8;
        }
        else
        {
            if (x + 8 >= g_lcd_width)
            {
                x = 0;
                y += 16;
            }

            if (y + 16 >= g_lcd_height)
                return line_width;

            line_width += Draw_ASCII(x, y, str[i], front_color, back_color);
            x += 8;
        }
    }

    return line_width;
}

/**********************************************************************
 * 函数名称： Draw_Line
 * 功能描述： 画线
 * 输入参数： x1,y1 - 起点坐标
 *            x2,y2 - 终点坐标
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Line(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t front_color)
{
    uint32_t t;
    int xerr=0,yerr=0,delta_x,delta_y,distance; 
    int incx,incy,uRow,uCol; 
    delta_x=x2-x1; //计算坐标增量
    delta_y=y2-y1; 
    uRow=x1; 
    uCol=y1; 
    if(delta_x>0)incx=1; //设置单步方向
    else if(delta_x==0)incx=0;//垂直线
    else {incx=-1;delta_x=-delta_x;} 
    if(delta_y>0)incy=1; 
    else if(delta_y==0)incy=0;//水平线
    else{incy=-1;delta_y=-delta_y;} 
    if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
    else distance=delta_y; 
    for(t=0;t<=distance+1;t++ )//画线输出
    {  
        Draw_Pixel(uRow, uCol, front_color);//画点
        xerr+=delta_x ; 
        yerr+=delta_y ; 
        if(xerr>distance) 
        { 
            xerr-=distance; 
            uRow+=incx; 
        } 
        if(yerr>distance) 
        { 
            yerr-=distance; 
            uCol+=incy; 
        } 
    }  
} 

/**********************************************************************
 * 函数名称： Draw_Fill
 * 功能描述： 填充矩形区域
 * 输入参数： xsta,ysta - 起点坐标
 *            xend,yend - 终点坐标
 *            dwColor   - 颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Fill(uint32_t xsta, uint32_t ysta, uint32_t xend, uint32_t yend, uint32_t dwColor)
{
    for (; ysta <= yend; ysta++)
    {
        Draw_Line(xsta, ysta, xend, ysta, dwColor);
    }
}

/**********************************************************************
 * 函数名称： Draw_Sign
 * 功能描述： 画一个十字的标记
 * 输入参数： x,y - 起点坐标
 *            dwColor   - 颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Sign(uint32_t x, uint32_t y, uint32_t dwColor)
{
    /* 画实心矩形 */
    Draw_Fill(x-1, y-1, x+1, y+1, dwColor);

    /* 画横 */
    Draw_Line(x-4, y, x+4, y, dwColor);

    /* 画竖 */
    Draw_Line(x, y-4, x, y+4, dwColor);
}

/**********************************************************************
 * 函数名称： Draw_Rectangle
 * 功能描述： 画矩形
 * 输入参数： (x1,y1),(x2,y2):矩形的对角坐标
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Rectangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t front_color)
{
    Draw_Line(x1,y1,x2,y1,front_color);
    Draw_Line(x1,y1,x1,y2,front_color);
    Draw_Line(x1,y2,x2,y2,front_color);
    Draw_Line(x2,y1,x2,y2,front_color);
}

/**********************************************************************
 * 函数名称： Draw_Circle
 * 功能描述： 画圆
 * 输入参数： (x0,y0) - 中心点
 *            r       - 半径(单位:像素)
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Circle(uint32_t x0, uint32_t y0, uint32_t r, uint32_t front_color)
{
    int a,b;
    int di;
    a=0;b=r;      
    di=3-(r<<1);             //判断下个点位置的标志
    while(a<=b)
    {
        Draw_Pixel(x0+a, y0-b, front_color);             //5
        Draw_Pixel(x0+b, y0-a, front_color);             //0           
        Draw_Pixel(x0+b, y0+a, front_color);             //4               
        Draw_Pixel(x0+a, y0+b, front_color);             //6 
        Draw_Pixel(x0-a, y0+b, front_color);             //1       
        Draw_Pixel(x0-b, y0+a, front_color);             
        Draw_Pixel(x0-a, y0-b, front_color);             //2             
        Draw_Pixel(x0-b, y0-a, front_color);             //7                 
        a++;
        //使用Bresenham算法画圆
        if(di<0)di +=4*a+6;   
        else
        {
            di+=10+4*(a-b);   
            b--;
        }                           
    }
} 

/**********************************************************************
 * 函数名称： Draw_Number
 * 功能描述： 以十进制显示数字
 * 输入参数： (x,y) - 坐标
 *            num   - 数值
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 显示的字符的总宽度(单位:像素)
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
int Draw_Number(uint32_t x, uint32_t y, uint32_t num, uint32_t front_color)
{           
    char buf[11];
    
    sprintf(buf, "%d", num);
    return Draw_String(x, y, buf, front_color, 0);
} 

/**********************************************************************
 * 函数名称： Draw_HexNumber
 * 功能描述： 以16进制显示数字
 * 输入参数： (x,y) - 坐标
 *            num   - 数值
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 显示的字符的总宽度(单位:像素)
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
int Draw_HexNumber(uint32_t x, uint32_t y, uint32_t num, uint32_t front_color)
{           
    char buf[11];
    sprintf(buf, "0x%x", num);
    return Draw_String(x, y, buf, front_color, 0);  
} 

/**********************************************************************
 * 函数名称： Draw_ChineseFont
 * 功能描述： 显示汉字字符串
 * 输入参数： (x,y) - 坐标
 *            cn    - 汉字字符串(UTF-8编码),
 *                    先使用"取字模软件.EXE"生成点阵,存入font_chinese.c的CnChar32x29数组中
 *            front_color - 前景颜色, 格式为0x00RRGGBB
 *            back_color  - 背景颜色, 格式为0x00RRGGBB
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_ChineseFont(uint32_t x, uint32_t y, char *cn, uint32_t front_color, uint32_t back_color)
{
    uint32_t i, j, wordNum;
    uint32_t color;
    uint32_t x0=x;
    uint32_t y0=y;
    static uint8_t *buf;
    BitMap bitmap; 

    if (!buf)
    {
        buf = malloc(32*29*2);
        if (!buf)
            return;
    }
    
    while (*cn != '\0')
    {
        for (wordNum=0; CnChar32x29[wordNum].Index[0]; wordNum++)
        {   //wordNum扫描字库CnChar32x29的字数
            if ((CnChar32x29[wordNum].Index[0]==*cn)
                 &&(CnChar32x29[wordNum].Index[1]==*(cn+1))
                 &&(CnChar32x29[wordNum].Index[2]==*(cn+2)))
            {
                for(i=0; i<116; i++) 
                {   //MSK的位数
                    color=CnChar32x29[wordNum].Msk[i];
                    for(j=0;j<8;j++) 
                    {
                        if((color&0x80)==0x80)
                        {
                            //Draw_Pixel(x,y,front_color);
                            Draw_Pixel_In_Buf(buf, x-x0, y-y0, 32, 29, front_color);
                        }                       
                        else
                        {
                            //Draw_Pixel(x,y,back_color);
                            Draw_Pixel_In_Buf(buf, x-x0, y-y0, 32, 29, back_color);
                        } 
                        color<<=1;
                        x++;
                        if((x-x0)==32)
                        {
                            x=x0;
                            y++;
                            if((y-y0)==29)
                            {
                                y=y0;
                            }
                        }
                    }//for(j=0;j<8;j++)结束
                }   
                bitmap.width = 32;
                bitmap.height = 29;        
                bitmap.datas = buf;
                Draw_Region(x0, y0, &bitmap);
            }
            
        } //for (wordNum=0; wordNum<20; wordNum++)结束
        cn += 3;  /* 一个汉字的UTF8编码占据3字节 */
        x += 32;
        
        if (x >= g_lcd_width)
        {
            x = 0;
            y += 29;
        }

        if (y + 29 >= g_lcd_width)
            return;
        
        x0=x;
    }
}   

/**********************************************************************
 * 函数名称： Draw_Picture
 * 功能描述： 绘制图片
 * 输入参数： (x,y) - 坐标
 *            pic   - 使用image2lcd生成的图片(水平扫描,包含图像头数据,16位真彩色,高位在前)
 *                    先使用"image2lcd.EXE"生成图片的点阵,存入picture.c
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void Draw_Picture(uint32_t x, uint32_t y, const uint8_t *pic)
{
    P_HEADCOLOR pHead = (P_HEADCOLOR)pic;
    BitMap bitmap; 

    int w = pHead->w;
    int h = pHead->h;

    /* 转为小字节序 */
    w = ((w<<8)|(w>>8)) & 0xffff;
    h = ((h<<8)|(h>>8)) & 0xffff;

    /* 数据 */
    pic = pic + sizeof(HEADCOLOR);

    bitmap.width = w;
    bitmap.height = h;        
    bitmap.datas = (uint8_t *)pic;
    Draw_Region(x, y, &bitmap);
}

