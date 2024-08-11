#include "uart_device.h"
#include "string.h"



extern UART_HandleTypeDef huart2;
static SemaphoreHandle_t uart2_send_semaphore;
static QueueHandle_t uart2_rx_queue;

extern UART_HandleTypeDef huart4;
static SemaphoreHandle_t uart4_send_semaphore;
static QueueHandle_t uart4_rx_queue;


static int UART_Rx_Start(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
static int UART_GET_DATA(struct UART_Device *pDev, uint8_t *data, int timeout);
static int UART_Send(struct UART_Device *pDev,uint8_t *datas, uint32_t len, int timeout);
static int UART_Flush(struct UART_Device *pDev);


static int USB_Rx_Start(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
static int USB_GET_DATA(struct UART_Device *pDev, uint8_t *data, int timeout);
static int USB_Send(struct UART_Device *pDev,uint8_t *datas, uint32_t len, int timeout);
static int USB_Flush(struct UART_Device *pDev);


static UART_Data UART2_Data = {&huart2};
static UART_Data UART4_Data = {&huart4};

static struct UART_Device g_uart2_dev = {"uart2", UART_Rx_Start, UART_Send, UART_GET_DATA, UART_Flush, &UART2_Data};
static struct UART_Device g_uart4_dev = {"uart4", UART_Rx_Start, UART_Send, UART_GET_DATA, UART_Flush, &UART4_Data};
static struct UART_Device g_usb_dev 	= {"usb", USB_Rx_Start, USB_Send, USB_GET_DATA, USB_Flush};



static struct UART_Device *g_uart_devices[] = {&g_uart2_dev, &g_uart4_dev, &g_usb_dev};

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
	PUART_Data pData;
	
	if(huart == &huart4)
	{
		pData = &UART4_Data;
	}
	
	if(huart == &huart2)
	{
		pData = &UART2_Data;
	}
	
	xSemaphoreGiveFromISR(pData->uart_send_semaphore, NULL);
}

/* IDLE中断回调函数 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	PUART_Data pData;
	
	if(huart == &huart4)
	{
		pData = &UART4_Data;
	}

	if(huart == &huart2)
	{
		pData = &UART2_Data;
	}	
		/* write queue */
	for(int i = 0;i < Size;i++)
	{
		 xQueueSendFromISR(pData->uart_rx_queue,  (const void *)&pData->uart_rx_buf[i], NULL);
	}
	
	/* restart rx */
	
	HAL_UARTEx_ReceiveToIdle_DMA(pData->huart, pData->uart_rx_buf, 100);
	

}

/* 接收完成回调函数 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	PUART_Data pData;
	
	if(huart == &huart4)
	{
		pData = &UART4_Data;
	}

	if(huart == &huart2)
	{
		pData = &UART2_Data;
	}	
	
	for(int i = 0;i < 100;i++)
	{
		 xQueueSendFromISR(pData->uart_rx_queue, &pData->uart_rx_buf[i], NULL);
	}	
	
	/* restart rx */
	
	HAL_UARTEx_ReceiveToIdle_DMA(pData->huart, pData->uart_rx_buf, MAX_UART_RX_BUF_LEN);	

}

/* 接收错误回调函数 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	PUART_Data pData;
	
	if(huart == &huart4)
	{
		pData = &UART4_Data;
	}

	if(huart == &huart2)
	{
		pData = &UART2_Data;
	}	
	
	/* 这么做才能重启串口 */
	if (pData)
	{
		HAL_UART_DeInit(pData->huart);
		HAL_UART_Init(pData->huart);
		/* re-start DMA+IDLE rx */
		HAL_UARTEx_ReceiveToIdle_DMA(pData->huart, pData->uart_rx_buf, MAX_UART_RX_BUF_LEN);
	}
	
}

static int UART_GET_DATA(struct UART_Device *pDev, uint8_t *data, int timeout)
{
	UART_Data *pUartData = (UART_Data *)pDev->pUartData;
	
	if(pdTRUE == xQueueReceive(pUartData->uart_rx_queue, data, timeout))
		return 0;
	else
		return -1;
}


static int UART_Rx_Start(UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit)
{
	UART_Data *pUartData = (UART_Data *)pDev->pUartData;
	
	if(!pUartData->uart_rx_queue)
	{
		pUartData->uart_rx_queue = xQueueCreate(200,1);
		pUartData->uart_send_semaphore = xSemaphoreCreateBinary();
		
		HAL_UARTEx_ReceiveToIdle_DMA(pUartData->huart, pUartData->uart_rx_buf, MAX_UART_RX_BUF_LEN);		
	}
	
	return 0;
}



static int UART_Send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout)
{
	UART_Data *pUartData = (UART_Data *)pDev->pUartData;

	HAL_UART_Transmit_DMA(pUartData->huart, datas, len);
	
	/* 等待一个信号量 */
	if(pdTRUE == xSemaphoreTake( pUartData->uart_send_semaphore, timeout ))
		return 0;
	else
		return -1;
	
}

static int UART_Flush(struct UART_Device *pDev)
{
	UART_Data *pUartData = (UART_Data *)pDev->pUartData;
	uint8_t *data;
	int count = 0;

	if(!pUartData->uart_rx_queue)
		return 0;
	
	while(pdTRUE == xQueueReceive(pUartData->uart_rx_queue, data, 0));

	return 1;

}

static int USB_Send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout)
{
	int ux_device_cdc_acm_send(uint8_t *datas, uint32_t len, uint32_t timeout);

	if (0 == ux_device_cdc_acm_send(datas, len, timeout))	
	    return 0; // write(ctx->s, req, req_length);
	else
	{
		return -1;
	}
	
}


static int USB_Rx_Start(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit)
{
	return 0;
}


static int USB_GET_DATA(struct UART_Device *pDev, uint8_t *data, int timeout)
{
	int ux_device_cdc_acm_getchar(uint8_t *datas, uint32_t timeout);
	
	if (ux_device_cdc_acm_getchar(data, timeout) == 0)
	    return 0; // read(ctx->s, rsp, rsp_length);
	else
		return -1;
}

static int USB_Flush(struct UART_Device *pDev)
{
	int ux_device_cdc_acm_flush(void);

	return ux_device_cdc_acm_flush();
}