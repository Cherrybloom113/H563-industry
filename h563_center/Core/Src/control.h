#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdint.h>

#define MAX_POINT_COUNT 500

typedef struct PointMap {
    char reg_type[4];
    uint16_t reg_addr_master; /* 主控的寄存器地址 */    
    uint16_t channel;         /* 0-主控本身, 1-通过CH1访问, 2-通过CH2访问 */
    uint16_t dev_addr;        /* 传感器的设备地址 */
    uint16_t reg_addr_salve;  /* 传感器的寄存器地址 */
}PointMap, *PPointMap;

/**********************************************************************
 * 函数名称： LibmodbusServerTask
 * 功能描述： 主控Modbus任务
 * 输入参数： pvParameters - 未使用
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
 ***********************************************************************/
void LibmodbusServerTask( void *pvParameters )	;

#endif

