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

static PointMap g_tPointMaps[MAX_POINT_COUNT]; /* 映射表 */
static uint16_t g_iPointVals[MAX_POINT_COUNT]; /* 点的上次数值 */
static uint8_t *g_pucPointMapCurPos;           /* 映射表的当前位置:我们使用memcpy来更新映射表 */
static int g_iPointMapCnt;                     /* 映射表里点的个数 */

/**********************************************************************
 * 函数名称： clear_point_map
 * 功能描述： 清除"点"的映射表
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
 ***********************************************************************/
static void clear_point_map(void)
{
	g_pucPointMapCurPos = (uint8_t *)g_tPointMaps;
    g_iPointMapCnt = 0;
	memset(g_iPointVals, 0xff, sizeof(g_iPointVals));
}

/**********************************************************************
 * 函数名称： get_point_map_count
 * 功能描述： 获得映射表里点的个数
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 点的个数
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
 ***********************************************************************/
static int get_point_map_count(void)
{
    return g_iPointMapCnt;
}

/**********************************************************************
 * 函数名称： add_point_map
 * 功能描述： 往映射表里添加一个点的映射关系
 * 输入参数： ptPointMap - 点的映射结构体指针
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： BE32toLE32
 * 功能描述： 把大字节序转换为小字节序
 * 输入参数： buf - 里面存有4个字节的大字节序数据
 * 输出参数： 无
 * 返 回 值： 小字节序的uint32_t数值
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
 ***********************************************************************/
static uint32_t BE32toLE32(uint8_t *buf)
{
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 0);
}

/**********************************************************************
 * 函数名称： parse_map_info
 * 功能描述： 解析映射信息, 存入映射表
 * 输入参数： msg - 里面存有上位机发来的消息, 消息为"write file record"请求包
 *            msg_len - 消息长度
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
		/* 收到文件头,则算出文件大小,并清除旧的映射表 */
		/* 使用memcpy比较安全,因为"&msg[10]"不一定是对齐的地址 */
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
        /* 使用memcpy比较安全,因为"&msg[10]"不一定是对齐的地址, ptPointMap指向不对齐的地址时,会导致add_point_map出错 */
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
 * 函数名称： modbus_parse_file_record
 * 功能描述： 解析文件信息, 目前仅仅解析file_no为0的信息(点的映射表)
 * 输入参数： msg - 里面存有上位机发来的消息
 *            msg_len - 消息长度
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
            /* 处理"点"的映射信息 */
            parse_map_info(msg, msg_len);
        }        
    }
}

/**********************************************************************
 * 函数名称： local_read_point
 * 功能描述： 读取中控自己的传感器
 * 输入参数： ptPointMap - 点的映射结构体指针
 * 输出参数： pVal - 用来保存读到的数值
 * 返 回 值： 0-成功, (-1)-失败
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： local_write_point
 * 功能描述： 写中控自己的传感器
 * 输入参数： ptPointMap - 点的映射结构体指针
 *            val - 要写入的值
 * 输出参数： 无
 * 返 回 值： 0-成功, (-1)-失败
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： update_modbus_mapping_reg
 * 功能描述： 根据点的映射信息更新它在mb_mapping里的寄存器数值
 * 输入参数： ptPointMap - 点的映射结构体指针
 *            mb_mapping - 里面含有DI/DO/AI/AO寄存器
 *            val - 要更新的数值
 * 输出参数： 无
 * 返 回 值： 0-成功, (-1)-失败
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： get_modbus_mapping_reg
 * 功能描述： 根据点的映射信息获得它在mb_mapping里的寄存器数值
 * 输入参数： ptPointMap - 点的映射结构体指针
 *            mb_mapping - 里面含有DI/DO/AI/AO寄存器
 * 输出参数： pVal - 用来保存读到的数值
 * 返 回 值： 0-成功, (-1)-失败
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： modbus_read_point
 * 功能描述： 根据点的映射信息读取外接的传感器数值
 * 输入参数： ctx - modbus上下文
 *            ptPointMap - 点的映射结构体指针
 * 输出参数： pVal - 用来保存读到的数值
 * 返 回 值： 0-成功, (-1)-失败
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： modbus_write_point
 * 功能描述： 根据点的映射信息写外接的传感器数值
 * 输入参数： ctx - modbus上下文
 *            ptPointMap - 点的映射结构体指针
 *            val - 要写入的值
 * 输出参数： 无
 * 返 回 值： 0-成功, (-1)-失败
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： loop_once
 * 功能描述： CH0_Task/CH1_Task/CH2_Task任务的一次循环函数
 * 输入参数： ctx - modbus上下文
 *            channel - 通道
 *            mb_mapping - 里面含有DI/DO/AI/AO寄存器
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
 ***********************************************************************/
static void loop_once(modbus_t *ctx, int channel, modbus_mapping_t *mb_mapping)
{
	int count = get_point_map_count();
	int i;
	int val;
	int pre_val;

	/* 如果modbus_mapping_t里这个channel的点发生了变化,发起写操作 */
	for (i = 0; i < count; i++)
	{
		/* channel不等于0,表示要使用modbus函数访问外接的传感器 */
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

		/* channel等于0,表示要访问中控自己的传感器 */
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

	/* 然后更新映射表里channel通道的所有点 */
	for (i = 0; i < count; i++)
	{
		/* channel不等于0,表示要使用modbus函数访问外接的传感器 */
		if (g_tPointMaps[i].channel == channel && channel)
		{
			if (0 == modbus_read_point(ctx, &g_tPointMaps[i], &val))
				update_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, val);
		}

		/* channel等于0,表示要访问中控自己的传感器 */
		if (g_tPointMaps[i].channel == channel && !channel)
		{
			if (0 == local_read_point(&g_tPointMaps[i], &val))
				update_modbus_mapping_reg(&g_tPointMaps[i], mb_mapping, val);
		}
	}
}


/**********************************************************************
 * 函数名称： CH0_Task
 * 功能描述： 中控任务, 用来读写中控自己的传感器
 * 输入参数： pvParameters - 就是mb_mapping(里面含有DI/DO/AI/AO寄存器)
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： CH1_Task
 * 功能描述： CH1任务, 用来读写接在CH1上的传感器
 * 输入参数： pvParameters - 就是mb_mapping(里面含有DI/DO/AI/AO寄存器)
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： CH2_Task
 * 功能描述： CH2任务, 用来读写接在CH2上的传感器
 * 输入参数： pvParameters - 就是mb_mapping(里面含有DI/DO/AI/AO寄存器)
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
 * 函数名称： LibmodbusServerTask
 * 功能描述： 主控Modbus任务
 * 输入参数： pvParameters - 未使用
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人       修改内容
 * -----------------------------------------------
 * 2024/06/23        V1.0     韦东山       创建
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
		CH0_Task, // 函数指针, 任务函数
		"CH0_Task", // 任务的名称
		400, // 栈大小
		mb_mapping, // 调用任务函数时传入的参数
		osPriorityNormal, // 优先级
		NULL); // 任务句柄, 以后使用它来操作这个任务

	xTaskCreate(
		CH1_Task, // 函数指针, 任务函数
		"CH1_Task", // 任务的名称
		400, // 栈大小
		mb_mapping, // 调用任务函数时传入的参数
		osPriorityNormal, // 优先级
		NULL); // 任务句柄, 以后使用它来操作这个任务

	xTaskCreate(
		CH2_Task, // 函数指针, 任务函数
		"CH2_Task", // 任务的名称
		400, // 栈大小
		mb_mapping, // 调用任务函数时传入的参数
		osPriorityNormal, // 优先级
		NULL); // 任务句柄, 以后使用它来操作这个任务
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

