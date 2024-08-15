#ifndef _UART_DEVICE_H
#define _UART_DEVICE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"
#include "usart.h"

#define MAX_UART_RX_BUF_LEN		260

typedef struct UART_Device {
		char *name;
		int (*Init)( struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
		int (*Send)( struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout);
		int (*RecvByte)( struct UART_Device *pDev, uint8_t *data, int timeout);
		int (*Flush)(struct UART_Device *pDev);
		void * pUartData;
}UART_Device,*PUART_Device;

typedef struct UART_Data
{
	UART_HandleTypeDef* huart;
	SemaphoreHandle_t uart_send_semaphore;
	uint8_t  uart_rx_buf[MAX_UART_RX_BUF_LEN];
	QueueHandle_t uart_rx_queue;
}UART_Data,*PUART_Data;

struct UART_Device * GetUartDevice(char *name);

#endif