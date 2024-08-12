/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "draw.h"
#include "uart_device.h"
#include "ux_api.h"
#include "modbus.h"
#include "errno.h"
   #include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* 浼犳劅鍣ㄦ墍鏈夌殑瀵勫瓨鍣ㄦ?诲拰 */
static modbus_mapping_t * g_mb_mapping; 

SemaphoreHandle_t g_SwitchSemaphore;
SemaphoreHandle_t g_MonitorSemaphore;
SemaphoreHandle_t g_TempHumiSemaphore;
SemaphoreHandle_t g_Uart2Mutex;

modbus_t *g_uart2_ctx;







/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* 负责上位机通信以及通知传感器需要修改数据 */

void LibModbusServerTask(void *param)
{
	uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
	uint8_t mb_mapping_backend[17];
	
	
	ctx = modbus_new_st_rtu("usb", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 1);
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

	mb_mapping = g_mb_mapping;
	memcpy(mb_mapping_backend, mb_mapping->tab_bits, 16);
	

	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		do {
			rc = modbus_receive(ctx, query);
			/* Filtered queries return 0 */
		} while (rc == 0);
 
		/* The connection is not closed on errors which require on reply such as
		   bad CRC in RTU. */
		if (rc == -1 && errno != EMBBADCRC) {
			/* Quit */
			continue;
		}

		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			//break;
		}

		/* notice task2 or task3 to update datas */
		if(0 != memcmp(&mb_mapping_backend[1], &mb_mapping->tab_bits[1], 5))
			{
				/* notice task 2 */
				xSemaphoreGive(g_SwitchSemaphore);
				memcpy(&mb_mapping_backend[1], &mb_mapping->tab_bits[1], 5);
			}
		if(0 != memcmp(&mb_mapping_backend[6], &mb_mapping->tab_bits[6], 5))
			{
				/* notice task 3 */
				xSemaphoreGive(g_MonitorSemaphore);
				memcpy(&mb_mapping_backend[6], &mb_mapping->tab_bits[6], 5);
			}		
			
			if(0 != memcmp(&mb_mapping_backend[11], &mb_mapping->tab_bits[11], 5))
			{
				/* notice task 3 */
				xSemaphoreGive(g_TempHumiSemaphore);
				memcpy(&mb_mapping_backend[11], &mb_mapping->tab_bits[11], 5);
			}
		
		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

	}

	modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);

}


void Task2SwitchTask(void *param)
{
	modbus_t *ctx;
	int rc = 0;
	uint16_t val;
	int nb = 1;
	uint8_t tab_bits[10];
	uint16_t vals[10];
	char buf[100];
	
	ctx = g_uart2_ctx;

	
	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		/* read switch key1~3 ID :1 
		* display on lcd
		*/
		xSemaphoreTake(g_Uart2Mutex, portMAX_DELAY);
		vTaskDelay(20);
		modbus_set_slave(ctx, 1);
		rc = modbus_read_input_bits(ctx, 0, 3, tab_bits);
		xSemaphoreGive(g_Uart2Mutex);
		
		if(rc == 3)
		{
			sprintf(buf, "switch tab input bit:%d %d %d",tab_bits[0], tab_bits[1], tab_bits[2]);
			Draw_String(0, 0, buf, 0x00ff0000, 0x00ffffff);
			memcpy(g_mb_mapping->tab_input_bits, tab_bits, 3);
		}

		
		/* wait for semphore to change state */
		if(pdTRUE == xSemaphoreTake(g_SwitchSemaphore, 500))
			{
				xSemaphoreTake(g_Uart2Mutex, portMAX_DELAY);
				vTaskDelay(20);
				modbus_set_slave(ctx, 1);
				modbus_write_bits(ctx, 0, 5, &g_mb_mapping->tab_bits[1]);
				xSemaphoreGive(g_Uart2Mutex);
			}

		
		vTaskDelay(30);

	}

}

void Task3MonitorTask(void *param)
{
	modbus_t *ctx;
	int rc = 0;
	uint16_t val;
	int nb = 1;
	uint8_t tab_bits[10];
	uint16_t vals[10];
	char buf[100];
	
	ctx = g_uart2_ctx;

	
	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {

		xSemaphoreTake(g_Uart2Mutex, portMAX_DELAY);
		vTaskDelay(20);
		modbus_set_slave(ctx, 2);
		rc = modbus_read_input_registers(ctx, 0, 2, vals);
		xSemaphoreGive(g_Uart2Mutex);

		if(rc == 2)
		{
			sprintf(buf, "ENV Senor : 0x%x 0x%x",vals[0], vals[1]);
			Draw_String(0, 16 , buf, 0x00ff0000, 0x00ffffff);
			memcpy(g_mb_mapping->tab_input_registers, vals, 4);
		}

		
		/* wait for semphore to change state */
		if(pdTRUE == xSemaphoreTake(g_MonitorSemaphore, 500))
			{
				xSemaphoreTake(g_Uart2Mutex, portMAX_DELAY);
				vTaskDelay(20);
				modbus_set_slave(ctx, 2);
				modbus_write_bits(ctx, 0, 5, &g_mb_mapping->tab_bits[6]);
				xSemaphoreGive(g_Uart2Mutex);
			}

		
		vTaskDelay(30);

	}

}

void Task4TempHumiTask(void *param)
{
	modbus_t *ctx;
	int rc = 0;
	uint16_t val;
	int nb = 1;
	uint8_t tab_bits[10];
	uint16_t vals[10];
	char buf[100];
	
	ctx = modbus_new_st_rtu("uart4", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 3);

	
	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {

		vTaskDelay(20);
		rc = modbus_read_input_registers(ctx, 0, 2, vals);

		if(rc == 2)
		{
			sprintf(buf, "TempHumiSenor : temp:%d humi:%d",vals[0], vals[1]);
			Draw_String(0, 32 , buf, 0x00ff0000, 0x00ffffff);
			memcpy(&g_mb_mapping->tab_input_registers[2], vals, 4);
		}

		
		/* wait for semphore to change state */
		if(pdTRUE == xSemaphoreTake(g_TempHumiSemaphore, 500))
			{
				vTaskDelay(20);
				modbus_set_slave(ctx, 3);
				modbus_write_bits(ctx, 0, 5, &g_mb_mapping->tab_bits[11]);
			}

		
		vTaskDelay(30);

	}

}





/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	g_mb_mapping = modbus_mapping_new_start_address(0,16,0,3,0,0,0,4);
  g_SwitchSemaphore = xSemaphoreCreateBinary();
	g_MonitorSemaphore = xSemaphoreCreateBinary();
	g_TempHumiSemaphore = xSemaphoreCreateBinary();
	g_Uart2Mutex = xSemaphoreCreateMutex();
	g_uart2_ctx = modbus_new_st_rtu("uart2", 115200, 'N', 8, 1);

	xTaskCreate(    LibModbusServerTask,
                  "LibModbusServerTask",
                   400,
                   NULL,
                   osPriorityNormal,
                   NULL
                          );	
			
	xTaskCreate(    Task2SwitchTask,
                  "Task2SwitchTask",
                   400,
                   NULL,
                   osPriorityNormal,
                   NULL
                          );		
	
	xTaskCreate(    Task3MonitorTask,
                  "Task3MonitorTask",
                   400,
                   NULL,
                   osPriorityNormal,
                   NULL
                          );	
													
	xTaskCreate(    Task4TempHumiTask,
                  "Task4TempHumiTask",
                   400,
                   NULL,
                   osPriorityNormal,
                   NULL
                          );	

		/* 测试uart的modbus协议是否正确工作
		* 测试结果 uart2和4均可以正常工作
		*/
//	xTaskCreate(    LibModbusClientTask,
//                  "LibModbusClientTask",
//                   200,
//                   NULL,
//                   osPriorityNormal,
//                   NULL
//                          );	
	
#if 0
													
	xTaskCreate(    USBtoRS485,
                  "USBtoRS485",
                   200,
                   NULL,
                   osPriorityNormal,
                   NULL
                          );
													
	xTaskCreate(    RS485toUSB,
                  "RS485toUSB",
                   200,
                   NULL,
                   osPriorityNormal,
                   NULL
                          );			

#endif

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
		ux_system_tasks_run();
  }
  /* USER CODE END defaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

