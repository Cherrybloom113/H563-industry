/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "draw.h"
#include "stdio.h"
#include "draw.h"
#include "ux_api.h"
#include "modbus.h"
#include "errno.h"
#include "uart_device.h"
#include "semphr.h"

#include "control.h"

static PointMap g_tPointMaps[MAX_POINT_COUNT]; /* ӳ��� */
static uint16_t g_iPointVals[MAX_POINT_COUNT]; /* ����ϴ���ֵ */
static uint8_t *g_pucPointMapCurPos;           /* ӳ���ĵ�ǰλ��:����ʹ��memcpy������ӳ��� */
static int g_iPointMapCnt;                     /* ӳ������ĸ��� */

/**********************************************************************
 * �������ƣ� clear_point_map
 * ���������� ���"��"��ӳ���
 * ��������� ��
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void clear_point_map(void)
{
	g_pucPointMapCurPos = (uint8_t *)g_tPointMaps;
    g_iPointMapCnt = 0;
	memset(g_iPointVals, 0xff, sizeof(g_iPointVals));
}

/**********************************************************************
 * �������ƣ� get_point_map_count
 * ���������� ���ӳ������ĸ���
 * ��������� ��
 * ��������� ��
 * �� �� ֵ�� ��ĸ���
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static int get_point_map_count(void)
{
    return g_iPointMapCnt;
}

/**********************************************************************
 * �������ƣ� add_point_map
 * ���������� ��ӳ��������һ�����ӳ���ϵ
 * ��������� ptPointMap - ���ӳ��ṹ��ָ��
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void add_point_map(PPointMap ptPointMap)
{
    if (g_iPointMapCnt < MAX_POINT_COUNT)
    {
        g_tPointMaps[g_iPointMapCnt] = *ptPointMap;
        g_iPointMapCnt++;
    }
}

/**********************************************************************
 * �������ƣ� BE32toLE32
 * ���������� �Ѵ��ֽ���ת��ΪС�ֽ���
 * ��������� buf - �������4���ֽڵĴ��ֽ�������
 * ��������� ��
 * �� �� ֵ�� С�ֽ����uint32_t��ֵ
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static uint32_t BE32toLE32(uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 0);
}

/**********************************************************************
 * �������ƣ� parse_map_info
 * ���������� ����ӳ����Ϣ, ����ӳ���
 * ��������� msg - ���������λ����������Ϣ, ��ϢΪ"write file record"�����
 *            msg_len - ��Ϣ����
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void parse_map_info(uint8_t *msg, uint16_t msg_len)
{
    static FileInfo tFileInfo;
    static int recv_len = 0;
    int cur_len;
    PPointMap ptPointMap;
    
    uint16_t file_no;
    uint16_t record_no;
    char buf[100];

    file_no = ((uint16_t)msg[4]<<8) | msg[5];
    record_no = ((uint16_t)msg[6]<<8) | msg[7];

    if (record_no == 0)
    {
		/* �յ��ļ�ͷ,������ļ���С,������ɵ�ӳ��� */
		/* ʹ��memcpy�Ƚϰ�ȫ,��Ϊ"&msg[10]"��һ���Ƕ���ĵ�ַ */
        memcpy(&tFileInfo, &msg[10], sizeof(tFileInfo));
        tFileInfo.file_len = BE32toLE32((uint8_t *)&tFileInfo.file_len);
        recv_len = 0;
        clear_point_map();
    }
    else
    {
        cur_len = msg[2] - 7;

		memcpy(g_pucPointMapCurPos, &msg[10], cur_len);
		g_pucPointMapCurPos += cur_len;
		g_iPointMapCnt = (uint32_t)(g_pucPointMapCurPos - (uint8_t *)g_tPointMaps) / sizeof(PointMap);
#if 0	
        /* ʹ��memcpy�Ƚϰ�ȫ,��Ϊ"&msg[10]"��һ���Ƕ���ĵ�ַ, ptPointMapָ�򲻶���ĵ�ַʱ,�ᵼ��add_point_map���� */
        ptPointMap = (PPointMap)&msg[10];
        cur_len = msg[2] - 7;
        for (int i = 0; i < cur_len / sizeof(PointMap); i++)
        {
            add_point_map(ptPointMap);
            ptPointMap++;
        }
#endif        
        recv_len += cur_len;
        if (recv_len >= tFileInfo.file_len)
        {
            sprintf(buf, "Get %d map info  ", get_point_map_count());
            Draw_String(0, 32, buf, 0xff0000, 0);
        }
    }
}

/**********************************************************************
 * �������ƣ� modbus_parse_file_record
 * ���������� �����ļ���Ϣ, Ŀǰ��������file_noΪ0����Ϣ(���ӳ���)
 * ��������� msg - ���������λ����������Ϣ
 *            msg_len - ��Ϣ����
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void modbus_parse_file_record(uint8_t *msg, uint16_t msg_len)
{
    uint16_t file_no;
    uint16_t record_no;
    
    if (msg[1] == MODBUS_FC_WRITE_FILE_RECORD)
    {
        file_no = ((uint16_t)msg[4]<<8) | msg[5];
        record_no = ((uint16_t)msg[6]<<8) | msg[7];

        if (file_no == 0)
        {
            /* ����"��"��ӳ����Ϣ */
            parse_map_info(msg, msg_len);
        }        
    }
}

/**********************************************************************
 * �������ƣ� local_read_point
 * ���������� ��ȡ�п��Լ��Ĵ�����
 * ��������� ptPointMap - ���ӳ��ṹ��ָ��
 * ��������� pVal - ���������������ֵ
 * �� �� ֵ�� 0-�ɹ�, (-1)-ʧ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static int local_read_point(PPointMap ptPointMap, int *pVal)
{
    if (!ptPointMap->channel)
    {
		if (ptPointMap->reg_addr_salve == 0 && !strcmp(ptPointMap->reg_type, "0x"))
		{
			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12))
				*pVal = 1;
			else
				*pVal = 0;
			return 0;
		}
    }
	return -1;
}

/**********************************************************************
 * �������ƣ� local_write_point
 * ���������� д�п��Լ��Ĵ�����
 * ��������� ptPointMap - ���ӳ��ṹ��ָ��
 *            val - Ҫд���ֵ
 * ��������� ��
 * �� �� ֵ�� 0-�ɹ�, (-1)-ʧ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static int local_write_point(PPointMap ptPointMap, int val)
{
    if (!ptPointMap->channel)
    {
		if (ptPointMap->reg_addr_salve == 0 && !strcmp(ptPointMap->reg_type, "0x"))
		{
			if (val)
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
			else
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
			return 0;
		}
    }
	return -1;
}

/**********************************************************************
 * �������ƣ� update_modbus_mapping_reg
 * ���������� ���ݵ��ӳ����Ϣ��������mb_mapping��ļĴ�����ֵ
 * ��������� ptPointMap - ���ӳ��ṹ��ָ��
 *            mb_mapping - ���溬��DI/DO/AI/AO�Ĵ���
 *            val - Ҫ���µ���ֵ
 * ��������� ��
 * �� �� ֵ�� 0-�ɹ�, (-1)-ʧ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void update_modbus_mapping_reg(PPointMap ptPointMap, modbus_mapping_t *mb_mapping, int val)
{
	if (!strcmp(ptPointMap->reg_type, "0x"))
		mb_mapping->tab_bits[ptPointMap->reg_addr_master] = val;
	if (!strcmp(ptPointMap->reg_type, "1x"))
		mb_mapping->tab_input_bits[ptPointMap->reg_addr_master] = val;
	if (!strcmp(ptPointMap->reg_type, "4x"))
		mb_mapping->tab_registers[ptPointMap->reg_addr_master] = val;
	if (!strcmp(ptPointMap->reg_type, "3x"))
		mb_mapping->tab_input_registers[ptPointMap->reg_addr_master] = val;
}

/**********************************************************************
 * �������ƣ� get_modbus_mapping_reg
 * ���������� ���ݵ��ӳ����Ϣ�������mb_mapping��ļĴ�����ֵ
 * ��������� ptPointMap - ���ӳ��ṹ��ָ��
 *            mb_mapping - ���溬��DI/DO/AI/AO�Ĵ���
 * ��������� pVal - ���������������ֵ
 * �� �� ֵ�� 0-�ɹ�, (-1)-ʧ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void get_modbus_mapping_reg(PPointMap ptPointMap, modbus_mapping_t *mb_mapping, int *pVal)
{
	if (!strcmp(ptPointMap->reg_type, "0x"))
		*pVal = mb_mapping->tab_bits[ptPointMap->reg_addr_master];
	if (!strcmp(ptPointMap->reg_type, "1x"))
		*pVal = mb_mapping->tab_input_bits[ptPointMap->reg_addr_master];
	if (!strcmp(ptPointMap->reg_type, "4x"))
		*pVal = mb_mapping->tab_registers[ptPointMap->reg_addr_master];
	if (!strcmp(ptPointMap->reg_type, "3x"))
		*pVal = mb_mapping->tab_input_registers[ptPointMap->reg_addr_master];
}


/**********************************************************************
 * �������ƣ� modbus_read_point
 * ���������� ���ݵ��ӳ����Ϣ��ȡ��ӵĴ�������ֵ
 * ��������� ctx - modbus������
 *            ptPointMap - ���ӳ��ṹ��ָ��
 * ��������� pVal - ���������������ֵ
 * �� �� ֵ�� 0-�ɹ�, (-1)-ʧ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static int modbus_read_point(modbus_t *ctx, PPointMap ptPointMap, int *pVal)
{
    int rc;
	
	if (!ctx)
		return -1;
	
    if (ptPointMap->channel)
    {

        *pVal = 0;        
        
        modbus_set_slave(ctx, ptPointMap->dev_addr);

        if (!strcmp(ptPointMap->reg_type, "0x"))
        {
            rc = modbus_read_bits(ctx, ptPointMap->reg_addr_salve, 1, (uint8_t *)pVal);
            if (rc == 1)
                return 0;
            else
            {
                return -1;
            }
        }
        else if (!strcmp(ptPointMap->reg_type, "1x"))
        {
            rc = modbus_read_input_bits(ctx, ptPointMap->reg_addr_salve, 1, (uint8_t *)pVal);
            if (rc == 1)
                return 0;
            else
            {
                return -1;
            }
        }
        else if (!strcmp(ptPointMap->reg_type, "4x"))
        {
            rc = modbus_read_registers(ctx, ptPointMap->reg_addr_salve, 1, ( uint16_t *)pVal);
            if (rc == 1)
                return 0;
            else
            {
                return -1;
            }
        }
        else if (!strcmp(ptPointMap->reg_type, "3x"))
        {
            rc = modbus_read_input_registers(ctx, ptPointMap->reg_addr_salve, 1, (uint16_t *)pVal);
            if (rc == 1)
                return 0;
            else
            {
                return -1;
            }
        }
        else
        {
            return -1;
        }
    }

    return -1;
}

/**********************************************************************
 * �������ƣ� modbus_write_point
 * ���������� ���ݵ��ӳ����Ϣд��ӵĴ�������ֵ
 * ��������� ctx - modbus������
 *            ptPointMap - ���ӳ��ṹ��ָ��
 *            val - Ҫд���ֵ
 * ��������� ��
 * �� �� ֵ�� 0-�ɹ�, (-1)-ʧ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static int modbus_write_point(modbus_t *ctx, PPointMap ptPointMap, int val)
{
    int rc;

	if (!ctx)
	{
		return -1;
	}

    if (ptPointMap->channel)
    {

        modbus_set_slave(ctx, ptPointMap->dev_addr);
        
        if (!strcmp(ptPointMap->reg_type, "0x"))
        {
            rc = modbus_write_bit(ctx, ptPointMap->reg_addr_salve, val);
            if (rc == 1)
                return 0;
            else 
            {
                return -1;
            }
        }
        else if (!strcmp(ptPointMap->reg_type, "4x"))
        {
            rc = modbus_write_register(ctx, ptPointMap->reg_addr_salve, val);
            if (rc == 1)
                return 0;
            else
            {
                return -1;
            }
        }
        else
        {
            //printf("can not write for %s\n", ptInfo->reg_type);
            return -1;
        }
    }
    else
    {
        return -1;
    }
}

/**********************************************************************
 * �������ƣ� loop_once
 * ���������� CH0_Task/CH1_Task/CH2_Task�����һ��ѭ������
 * ��������� ctx - modbus������
 *            channel - ͨ��
 *            mb_mapping - ���溬��DI/DO/AI/AO�Ĵ���
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void loop_once(modbus_t *ctx, int channel, modbus_mapping_t *mb_mapping)
{
	int count = get_point_map_count();
	int i;
	int val;
	int pre_val;

	/* ���modbus_mapping_t�����channel�ĵ㷢���˱仯,����д���� */
	for (i = 0; i < count; i++)
	{
		/* channel������0,��ʾҪʹ��modbus����������ӵĴ����� */
		if (g_tPointMaps[i].channel == channel && channel)
		{
			get_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, &val);
			if (val != g_iPointVals[i])
			{
				if (0 == modbus_write_point(ctx, &g_tPointMaps[i], val))
				{
					update_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, val);
					g_iPointVals[i] = val;
				}
			}
		}

		/* channel����0,��ʾҪ�����п��Լ��Ĵ����� */
		if (g_tPointMaps[i].channel == channel && !channel)
		{
			get_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, &val);
			if (val != g_iPointVals[i])
			{
				if (0 == local_write_point(&g_tPointMaps[i], val))
				{
					update_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, val);
					g_iPointVals[i] = val;
				}
			}
		}
	}

	/* Ȼ�����ӳ�����channelͨ�������е� */
	for (i = 0; i < count; i++)
	{
		/* channel������0,��ʾҪʹ��modbus����������ӵĴ����� */
		if (g_tPointMaps[i].channel == channel && channel)
		{
			if (0 == modbus_read_point(ctx, &g_tPointMaps[i], &val))
				update_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, val);
		}

		/* channel����0,��ʾҪ�����п��Լ��Ĵ����� */
		if (g_tPointMaps[i].channel == channel && !channel)
		{
			if (0 == local_read_point(&g_tPointMaps[i], &val))
				update_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, val);
		}
	}
}


/**********************************************************************
 * �������ƣ� CH0_Task
 * ���������� �п�����, ������д�п��Լ��Ĵ�����
 * ��������� pvParameters - ����mb_mapping(���溬��DI/DO/AI/AO�Ĵ���)
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void CH0_Task( void *pvParameters )	
{
	modbus_mapping_t * mb_mapping = pvParameters;
	
	while (1)
	{
		loop_once(NULL, 0, mb_mapping);

		vTaskDelay(100);
	}
	vTaskDelete(NULL);
}

/**********************************************************************
 * �������ƣ� CH1_Task
 * ���������� CH1����, ������д����CH1�ϵĴ�����
 * ��������� pvParameters - ����mb_mapping(���溬��DI/DO/AI/AO�Ĵ���)
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void CH1_Task( void *pvParameters )	
{
	modbus_mapping_t * mb_mapping = pvParameters;

	modbus_t *ctx;
	int rc;
	
	ctx = modbus_new_st_rtu("uart2", 115200, 'N', 8, 1);

	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}
	
	while (1)
	{
		loop_once(ctx, 1, mb_mapping);

		vTaskDelay(100);
	}
	vTaskDelete(NULL);
}


/**********************************************************************
 * �������ƣ� CH2_Task
 * ���������� CH2����, ������д����CH2�ϵĴ�����
 * ��������� pvParameters - ����mb_mapping(���溬��DI/DO/AI/AO�Ĵ���)
 * ��������� ��
 * �� �� ֵ�� ��
 * �޸����ڣ�      �汾��     �޸���       �޸�����
 * -----------------------------------------------
 * 2024/06/23        V1.0     Τ��ɽ       ����
 ***********************************************************************/
static void CH2_Task( void *pvParameters )	
{
	modbus_mapping_t * mb_mapping = pvParameters;

	modbus_t *ctx;
	int rc;
	
	ctx = modbus_new_st_rtu("uart4", 115200, 'N', 8, 1);

	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}
	
	while (1)
	{
		loop_once(ctx, 2, mb_mapping);

		vTaskDelay(100);
	}
	vTaskDelete(NULL);
}

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
void LibmodbusServerTask( void *pvParameters )	
{
    uint8_t *query;
    modbus_t *ctx;
    int rc;
    modbus_mapping_t *mb_mapping;
    
    ctx = modbus_new_st_rtu("usb", 115200, 'N', 8, 1);
    modbus_set_slave(ctx, 1);
    query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

    mb_mapping = modbus_mapping_new_start_address(0,
                                                  500,
                                                  0,
                                                  500,
                                                  0,
                                                  500,
                                                  0,
                                                  500);
    
    memset(mb_mapping->tab_bits, 0, mb_mapping->nb_bits);
    memset(mb_mapping->tab_registers, 0, mb_mapping->nb_registers*2);
    
    rc = modbus_connect(ctx);
    if (rc == -1) {
        //fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        vTaskDelete(NULL);;
    }
#if 1
	xTaskCreate(
		CH0_Task, // ����ָ��, ������
		"CH0_Task", // ���������
		400, // ջ��С
		mb_mapping, // ����������ʱ����Ĳ���
		osPriorityNormal, // ���ȼ�
		NULL); // ������, �Ժ�ʹ�����������������

	xTaskCreate(
		CH1_Task, // ����ָ��, ������
		"CH1_Task", // ���������
		400, // ջ��С
		mb_mapping, // ����������ʱ����Ĳ���
		osPriorityNormal, // ���ȼ�
		NULL); // ������, �Ժ�ʹ�����������������

	xTaskCreate(
		CH2_Task, // ����ָ��, ������
		"CH2_Task", // ���������
		400, // ջ��С
		mb_mapping, // ����������ʱ����Ĳ���
		osPriorityNormal, // ���ȼ�
		NULL); // ������, �Ժ�ʹ�����������������
#endif
    for (;;) {
        do {
            rc = modbus_receive(ctx, query);
            /* Filtered queries return 0 */
        } while (rc == 0);
 
        /* The connection is not closed on errors which require on reply such as
           bad CRC in RTU. */
        if (rc < 0 ) {
            /* Quit */
            continue;
        }

        modbus_parse_file_record(query, rc);

        rc = modbus_reply(ctx, query, rc, mb_mapping);
        if (rc == -1) {
            //break;
        }
    }

    modbus_mapping_free(mb_mapping);
    vPortFree(query);
    /* For RTU */
    modbus_close(ctx);
    modbus_free(ctx);

    vTaskDelete(NULL);
}

