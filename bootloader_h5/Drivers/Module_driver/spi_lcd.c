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
#include "spi_lcd.h"
#include "spi.h"

#define SPI_CS_Pin GPIO_PIN_11
#define SPI_CS_GPIO_Port GPIOD

#define RS_Pin GPIO_PIN_12
#define RS_GPIO_Port GPIOD

#define RESET_Pin GPIO_PIN_4
#define RESET_GPIO_Port GPIOB

#define PWM_Pin GPIO_PIN_11
#define PWM_GPIO_Port GPIOB

static int g_lcd_width, g_lcd_height;


/* GPIO操作 */
/* 选中SPI LCD */
static void LCD_Select(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET);
}

/* 不选中SPI LCD */
static void LCD_DeSelect(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET);
}

/* 复位SPI LCD */
static void LCD_Reset(void)
{
    HAL_GPIO_WritePin(RESET_GPIO_Port,RESET_Pin,GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(RESET_GPIO_Port,RESET_Pin,GPIO_PIN_SET);
    HAL_Delay(100);
}

/* 控制SPI LCD的背光, enable非零时表示打开背光,否则熄灭背光 */
static void LCD_BackLightControl(uint32_t enable)
{
    if (enable)
        HAL_GPIO_WritePin(PWM_GPIO_Port,PWM_Pin,GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(PWM_GPIO_Port,PWM_Pin,GPIO_PIN_RESET);
}

/**********************************************************************
 * 函数名称： LCD_SetDataLine
 * 功能描述： 设置SPI LCD的D/C引脚为高电平,表示要传输的是数据
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void LCD_SetDataLine(void)
{
    HAL_GPIO_WritePin(GPIOD,RS_Pin,GPIO_PIN_SET);
}

/* 设置SPI LCD的D/C引脚为低电平,表示要传输的是命令 */
static void LCD_SetCmdLine(void)
{
    HAL_GPIO_WritePin(GPIOD,RS_Pin,GPIO_PIN_RESET);
}

/* 给SPI LCD发送多个数据,调用此函数前应该先调用LCD_SetDataLine */
static int SPI_WriteDatas(uint8_t *TxData,uint16_t size)
{
    int err;
    
    LCD_Select();
    err = HAL_SPI_Transmit(&hspi2,TxData,size,1000);
    LCD_DeSelect();

    return -err;
}

/* 给SPI LCD发送1个命令 */
static int LCD_WriteCmd(uint8_t cmd)
{
    LCD_SetCmdLine();
    return SPI_WriteDatas(&cmd, 1);
}

/* 给SPI LCD发送1个参数,通常是先调用LCD_WriteCmd,再调用LCD_WritePara */
static int LCD_WritePara(uint8_t data)
{
    LCD_SetDataLine();
    return SPI_WriteDatas(&data, 1);
}

/**********************************************************************
 * 函数名称： LCD_WriteDatas
 * 功能描述： 给SPI LCD发送多个数据
 * 输入参数： data  - 数据buf
 *            count - 要发送的数据个数
 * 输出参数： 无
 * 返 回 值： 0 - 成功, (-1) - 错误, (-2) - 忙, (-3) - 超时
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
int LCD_WriteDatas(uint8_t *datas, uint32_t count)
{
    //HAL_GPIO_WritePin(GPIOD,RS_Pin,GPIO_PIN_SET);  /* 由调用者设置RS引脚 */
    return SPI_WriteDatas(datas, count);
}

/**********************************************************************
 * 函数名称： LCD_Init
 * 功能描述： 初始化LCD
 * 输入参数： rotation - 旋转角度, 取值如下
 *    LCD_DISPLAY_ROTATION_0,
 *    LCD_DISPLAY_ROTATION_90,
 *    LCD_DISPLAY_ROTATION_180,
 *    LCD_DISPLAY_ROTATION_270,
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void LCD_Init(lcd_display_rotation_t rotation)
{       
    LCD_Reset();    
    LCD_BackLightControl(1);
    
#if 1   
    // Positive Gamma Control
    LCD_WriteCmd( 0xe0);
    LCD_WritePara(0xf0);
    LCD_WritePara(0x3e);
    LCD_WritePara(0x30);
    LCD_WritePara(0x06);
    LCD_WritePara(0x0a);
    LCD_WritePara(0x03);
    LCD_WritePara(0x4d);
    LCD_WritePara(0x56);
    LCD_WritePara(0x3a);
    LCD_WritePara(0x06);
    LCD_WritePara(0x0f);
    LCD_WritePara(0x04);
    LCD_WritePara(0x18);
    LCD_WritePara(0x13);
    LCD_WritePara(0x00);

    // Negative Gamma Control
    LCD_WriteCmd(0xe1);
    LCD_WritePara(0x0f);
    LCD_WritePara(0x37);
    LCD_WritePara(0x31);
    LCD_WritePara(0x0b);
    LCD_WritePara(0x0d);
    LCD_WritePara(0x06);
    LCD_WritePara(0x4d);
    LCD_WritePara(0x34);
    LCD_WritePara(0x38);
    LCD_WritePara(0x06);
    LCD_WritePara(0x11);
    LCD_WritePara(0x01);
    LCD_WritePara(0x18);
    LCD_WritePara(0x13);
    LCD_WritePara(0x00);
    
    // Power Control 1
    LCD_WriteCmd(0xc0);
    LCD_WritePara(0x18);
    LCD_WritePara(0x17);

    // Power Control 2
    LCD_WriteCmd(0xc1);
    LCD_WritePara(0x41);

    // Power Control 3
    LCD_WriteCmd(0xc5);
    LCD_WritePara(0x00);

    // VCOM Control
    LCD_WriteCmd(0x1a);
    LCD_WritePara(0x80);

    // Memory Access Control
    LCD_WriteCmd(0x36);
    LCD_WritePara(0x48);

    // Pixel Interface Format
    LCD_WriteCmd(0x3a);
    LCD_WritePara(0x55);

    // Interface Mode Control
    LCD_WriteCmd(0xb0);
    LCD_WritePara(0x00);

    // Frame Rate Control
    LCD_WriteCmd(0xb1);
    LCD_WritePara(0xa0);

    // Display Inversion Control
    LCD_WriteCmd(0xb4);
    LCD_WritePara(0x02);

    // Display Function Control
    LCD_WriteCmd(0xb6);
    LCD_WritePara(0x02);
    LCD_WritePara(0x02);

    // Set image function
    LCD_WriteCmd(0xe9);
    LCD_WritePara(0x00);

    //Adjust Control 3
    LCD_WriteCmd(0xf7);
    LCD_WritePara(0xa9);
    LCD_WritePara(0x51);
    LCD_WritePara(0x2c);
    LCD_WritePara(0x82);

    // Write_memory_start
    LCD_WriteCmd(0x21);
    HAL_Delay(120);
    //Exit Sleep
    LCD_WriteCmd(0x11);
    HAL_Delay(120);

    switch (rotation)
    {
        case LCD_DISPLAY_ROTATION_0:
            LCD_WriteCmd(0x36);
            LCD_WritePara(0x48);
            g_lcd_height = 480;
            g_lcd_width  = 320;
            break;
        case LCD_DISPLAY_ROTATION_90:
            LCD_WriteCmd(0x36);
            LCD_WritePara(0xe8);
            g_lcd_height = 320;
            g_lcd_width  = 480;
            break;
        case LCD_DISPLAY_ROTATION_180:
            LCD_WriteCmd(0x36);
            LCD_WritePara(0x88);
            g_lcd_height = 480;
            g_lcd_width  = 320;
            break;
        case LCD_DISPLAY_ROTATION_270:
            LCD_WriteCmd(0x36);
            LCD_WritePara(0x28);
            g_lcd_height = 320;
            g_lcd_width  = 480;
            break;
        default:
            LCD_WriteCmd(0x36);
            LCD_WritePara(0x48);
            g_lcd_height = 480;
            g_lcd_width  = 320;
            break;
    }

    // set_screen_size
    LCD_WriteCmd(0x2a);
    LCD_WritePara(0x00);
    LCD_WritePara(0x00);
    LCD_WritePara(0x01);
    LCD_WritePara(0x3f);

    LCD_WriteCmd(0x2b);
    LCD_WritePara(0x00);
    LCD_WritePara(0x00);
    LCD_WritePara(0x01);
    LCD_WritePara(0xdf);

    //Display on
    LCD_WriteCmd(0x29);
    HAL_Delay(120);
#else
    LCD_WriteCmd(0x11);
    HAL_Delay(120);
    LCD_WriteCmd(0x36);     // Memory Data Access Control MY,MX~~
    LCD_WritePara(0x48);   

    LCD_WriteCmd(0x3A);     
    //LCDDrvWriteReg(0x55);
    LCD_WritePara(0x55);   //LCDDrvWriteDat(0x66);

    LCD_WriteCmd(0xF0);     // Command Set Control
    LCD_WritePara(0xC3);   

    LCD_WriteCmd(0xF0);     
    LCD_WritePara(0x96);   

    LCD_WriteCmd(0xB4);     
    LCD_WritePara(0x01);   

    LCD_WriteCmd(0xB7);     
    LCD_WritePara(0xC6);   

    LCD_WriteCmd(0xC0);     
    LCD_WritePara(0x80);   
    LCD_WritePara(0x45);   

    LCD_WriteCmd(0xC1);     
    LCD_WritePara(0x13);   //18  //00

    LCD_WriteCmd(0xC2);     
    LCD_WritePara(0xA7);   

    LCD_WriteCmd(0xC5);     
    LCD_WritePara(0x0A);   

    LCD_WriteCmd(0xE8);     
    LCD_WritePara(0x40);
    LCD_WritePara(0x8A);
    LCD_WritePara(0x00);
    LCD_WritePara(0x00);
    LCD_WritePara(0x29);
    LCD_WritePara(0x19);
    LCD_WritePara(0xA5);
    LCD_WritePara(0x33);

    LCD_WriteCmd(0xE0);
    LCD_WritePara(0xD0);
    LCD_WritePara(0x08);
    LCD_WritePara(0x0F);
    LCD_WritePara(0x06);
    LCD_WritePara(0x06);
    LCD_WritePara(0x33);
    LCD_WritePara(0x30);
    LCD_WritePara(0x33);
    LCD_WritePara(0x47);
    LCD_WritePara(0x17);
    LCD_WritePara(0x13);
    LCD_WritePara(0x13);
    LCD_WritePara(0x2B);
    LCD_WritePara(0x31);

    LCD_WriteCmd(0xE1);
    LCD_WritePara(0xD0);
    LCD_WritePara(0x0A);
    LCD_WritePara(0x11);
    LCD_WritePara(0x0B);
    LCD_WritePara(0x09);
    LCD_WritePara(0x07);
    LCD_WritePara(0x2F);
    LCD_WritePara(0x33);
    LCD_WritePara(0x47);
    LCD_WritePara(0x38);
    LCD_WritePara(0x15);
    LCD_WritePara(0x16);
    LCD_WritePara(0x2C);
    LCD_WritePara(0x32);
     

    LCD_WriteCmd(0xF0);     
    LCD_WritePara(0x3C);   

    LCD_WriteCmd(0xF0);     
    LCD_WritePara(0x69);   

    HAL_Delay(120);

    LCD_WriteCmd(0x21);     

    LCD_WriteCmd(0x29);     
#endif  

}

/**********************************************************************
 * 函数名称： LCD_GetInfo
 * 功能描述： 获得LCD的信息
 * 输入参数： 无
 * 输出参数： pWidth  - 用来保存LCD的宽度(单位:像素)
 *            pHeight - 用来保存LCD的高度(单位:像素)
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void LCD_GetInfo(uint32_t *pWidth, uint32_t *pHeight)
{
    *pHeight = g_lcd_height;
    *pWidth  = g_lcd_width;
}

/**********************************************************************
 * 函数名称： LCD_SetWindows
 * 功能描述： 设置LCD接收数据的窗口,它会划定一个矩形对应的显存,后面写入的数据都对应这个区域
 * 输入参数： 无
 * 输出参数： (x1,y1) - 左上角坐标(此坐标包含在这个窗口里面)
 *            (x2,y2) - 右下角坐标(此坐标包含在这个窗口里面)
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/02/01        V1.0     韦东山       创建
 ***********************************************************************/
void LCD_SetWindows(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
{
    LCD_WriteCmd(0x2a);
    LCD_WritePara((x1 >> 8) & 0xFF);
    LCD_WritePara(x1 & 0xFF);
    LCD_WritePara((x2 >> 8) & 0xFF);
    LCD_WritePara(x2 & 0xFF);
    

    LCD_WriteCmd(0x2b);
    LCD_WritePara((y1 >> 8) & 0xFF);
    LCD_WritePara(y1 & 0xFF);
    LCD_WritePara((y2 >> 8) & 0xFF);
    LCD_WritePara(y2 & 0xFF);

    LCD_WriteCmd(0x2C);
}

