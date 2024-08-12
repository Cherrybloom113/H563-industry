#ifndef _UART_DEVICE_H
#define _UART_DEVICE_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "stm32f0xx_hal.h"

struct UART_Device {
		char *name;
		int (*Init)( struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
		int (*Send)( struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout);
		int (*RecvByte)( struct UART_Device *pDev, uint8_t *data, int timeout);
		int (*Flush)(struct UART_Device *pDev);
		void *pDevData;
};

struct Uart_Data{
	UART_HandleTypeDef *huart;
	GPIO_TypeDef* _485_mode_port;
	uint16_t 	    _485_mode_pin;
	QueueHandle_t uart_rx_queue;
	SemaphoreHandle_t uart_send_semaphore;
	uint8_t rxdata;
	};

struct UART_Device * GetUartDevice(char *name);

#endif
