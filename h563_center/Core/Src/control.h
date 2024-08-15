#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdint.h>

#define MAX_POINT_COUNT 500

typedef struct PointMap {
    char reg_type[4];
    uint16_t reg_addr_master; /* ���صļĴ�����ַ */    
    uint16_t channel;         /* 0-���ر���, 1-ͨ��CH1����, 2-ͨ��CH2���� */
    uint16_t dev_addr;        /* ���������豸��ַ */
    uint16_t reg_addr_salve;  /* �������ļĴ�����ַ */
}PointMap, *PPointMap;

/**********************************************************************
 * �������ƣ� LibmodbusServerTask
 * ���������� ����Modbus����
 * ��������� pvParameters - δʹ��
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
void LibmodbusServerTask( void *pvParameters )	;

#endif

