/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "uart_device.h"

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

int stm32_uart_init(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit)
{
	struct Uart_Data *pDevData = (struct Uart_Data *)pDev->pDevData;

	if(!pDevData->uart_rx_queue)
	{
		pDevData->uart_rx_queue = xQueueCreate(200,1);
		pDevData->uart_send_semaphore = xSemaphoreCreateBinary();
		
		/* 配置为接收 */
		HAL_GPIO_WritePin(pDevData->_485_mode_port, pDevData->_485_mode_pin, GPIO_PIN_RESET);
		
		HAL_UART_Receive_IT(pDevData->huart, &pDevData->rxdata, 1);		
	}
	
	return 0;
}


int stm32_uart_send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout)
{
	struct Uart_Data *pDevData = (struct Uart_Data *)pDev->pDevData;
	
	/* 配置为发送 */
	HAL_GPIO_WritePin(pDevData->_485_mode_port, pDevData->_485_mode_pin, GPIO_PIN_SET);
	
	HAL_UART_Transmit_IT(pDevData->huart, datas, len);
	
	/* 等待一个信号量 */
	if(pdTRUE == xSemaphoreTake(pDevData->uart_send_semaphore, timeout ))
	{
		HAL_GPIO_WritePin(pDevData->_485_mode_port, pDevData->_485_mode_pin, GPIO_PIN_RESET);
		
		return 0;
	}
	else
	{
		HAL_GPIO_WritePin(pDevData->_485_mode_port, pDevData->_485_mode_pin, GPIO_PIN_RESET);
		
		return -1;
	}
	
}


int stm32_uart_recive(struct UART_Device *pDev, uint8_t *data, int timeout)
{
	struct Uart_Data *pDevData = (struct Uart_Data *)pDev->pDevData;
	
	if(pdPASS == xQueueReceive(pDevData->uart_rx_queue, data, timeout))
		return 0;
	else
		return -1;
}

int stm32_uart_flush(struct UART_Device *pDev)
{
	struct Uart_Data *pDevData = (struct Uart_Data *)pDev->pDevData;
	
	uint8_t *data;

	if(!pDevData->uart_rx_queue)
		return 0;
	
	while(pdTRUE == xQueueReceive(pDevData->uart_rx_queue, data, 0));

	return 1;

}




/* USER CODE END 1 */
