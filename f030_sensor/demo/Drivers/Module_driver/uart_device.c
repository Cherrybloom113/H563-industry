#include <stdio.h>

#include "uart_device.h"
#include "string.h"


extern UART_HandleTypeDef huart1;


static struct Uart_Data g_uart1_data = {
	&huart1,
	GPIOA,
	GPIO_PIN_8,
	};


int stm32_uart_init(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
int stm32_uart_recive(struct UART_Device *pDev, uint8_t *data, int timeout);
int stm32_uart_send(struct UART_Device *pDev,uint8_t *datas, uint32_t len, int timeout);
int stm32_uart_flush(struct UART_Device *pDev);


static struct UART_Device g_uart1_dev = {"uart1", stm32_uart_init, stm32_uart_send, stm32_uart_recive, stm32_uart_flush, &g_uart1_data};


static struct UART_Device *g_uart_devices[] = {&g_uart1_dev};

#define MAX_UART_DEVICES_NUM		(sizeof(g_uart_devices)/sizeof(g_uart_devices[0]))


struct UART_Device * GetUartDevice(char *name)
{
	for(int i = 0;i < MAX_UART_DEVICES_NUM;i++)
	{
		if(!strcmp(name, g_uart_devices[i]->name))
			return g_uart_devices[i];
	}
	
	return NULL;
}

/* 发送完成回调函数 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		struct Uart_Data *pDevData = (struct Uart_Data *)g_uart1_dev.pDevData;
		
		xSemaphoreGiveFromISR(pDevData->uart_send_semaphore, NULL);
		
		/* 忘记切换模式了 */
		 HAL_GPIO_WritePin(pDevData->_485_mode_port, pDevData->_485_mode_pin, GPIO_PIN_RESET);
	}
	
}

/* 接收完成回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		struct Uart_Data *pDevData = (struct Uart_Data *)g_uart1_dev.pDevData;
		/* write queue */
		xQueueSendFromISR(pDevData->uart_rx_queue, &pDevData->rxdata, NULL);
		
		/* restart rx */
		
		HAL_UART_Receive_IT(pDevData->huart, &pDevData->rxdata, 1);
	}	

}

/* 接收错误回调函数 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		struct Uart_Data *pDevData = (struct Uart_Data *)g_uart1_dev.pDevData;
		
		HAL_UART_DeInit(&huart1);
		HAL_UART_Init(&huart1);
		
		HAL_UART_Receive_IT(pDevData->huart, &pDevData->rxdata, 1);
	}		
	
}
