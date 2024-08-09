/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device CDC ACM applicative source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define APP_RX_DATA_SIZE                          2048
#define APP_TX_DATA_SIZE                          2048

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA                      0x01
#define TX_NEW_TRANSMITTED_DATA                   0x02

/* Data length for vcp */
#define VCP_WORDLENGTH8                           8
#define VCP_WORDLENGTH9                           9

/* the minimum baudrate */
#define MIN_BAUDRATE                              9600

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static SemaphoreHandle_t g_xUSBUARTSend;
static QueueHandle_t g_xUSBUART_RX_Queue;

UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;
#if defined ( __ICCARM__ ) /* IAR Compiler */
#pragma location = ".UsbxAppSection"
#elif defined ( __CC_ARM ) || defined(__ARMCC_VERSION) /* ARM Compiler 5/6 */
__attribute__((section(".UsbxAppSection")))
#elif defined ( __GNUC__ ) /* GNU Compiler */
__attribute__((section(".UsbxAppSection")))
#endif
/* Data received over uart are stored in this buffer */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/* Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* Increment this pointer or roll it back to
start address when data are received over USART */
uint32_t UserTxBufPtrIn;

/* Increment this pointer or roll it back to
start address when data are sent over USB */
uint32_t UserTxBufPtrOut;

/* uart handler */
UART_HandleTypeDef *uart_handler;
//extern TX_EVENT_FLAGS_GROUP EventFlag;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern VOID USBX_APP_UART_Init(UART_HandleTypeDef **huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

static UINT ux_device_class_cdc_acm_write_callback(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, ULONG length)
{
	xSemaphoreGive(g_xUSBUARTSend);
	return 0;
}

static UINT ux_device_class_cdc_acm_read_callback(struct UX_SLAVE_CLASS_CDC_ACM_STRUCT *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length)
{
    
//    int Draw_String(uint32_t x, uint32_t y, char *str, uint32_t front_color, uint32_t back_color);
//    if (status == UX_SUCCESS)
//    {
//        data_pointer[length] = '\0';
//        Draw_String(0, 0, (char *)data_pointer, 0x0000ff00, 0);
//    }
	  for (int i = 0; i < length; i++)
		{
			xQueueSend(g_xUSBUART_RX_Queue, (const void *)&data_pointer[i], 0);
		}
		return 0;
}


/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */

  UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER parameter;

  /* Save the CDC instance */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

  parameter.ux_device_class_cdc_acm_parameter_write_callback = ux_device_class_cdc_acm_write_callback;
  parameter.ux_device_class_cdc_acm_parameter_read_callback = ux_device_class_cdc_acm_read_callback;
  ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START, (VOID *)&parameter);

	if (!g_xUSBUARTSend)
	{
		g_xUSBUARTSend = xSemaphoreCreateBinary( );
		g_xUSBUART_RX_Queue = xQueueCreate(10240, 1);
	}
	
//  /* Configure the UART peripheral */
//  USBX_APP_UART_Init(&uart_handler);

//  /* Get default UART parameters */
//  CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = uart_handler->Init.BaudRate;

//  /* Set the UART data type : only 8bits and 9bits are supported */
//  switch (uart_handler->Init.WordLength)
//  {
//    case UART_WORDLENGTH_8B:
//    {
//      /* Set UART data bit to 8 */
//      CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit = VCP_WORDLENGTH8;
//      break;
//    }

//    case UART_WORDLENGTH_9B:
//    {
//      /* Set UART data bit to 9 */
//      CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit = VCP_WORDLENGTH9;
//      break;
//    }

//    default :
//    {
//      /* By default set UART data bit to 8 */
//      CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_data_bit = VCP_WORDLENGTH8;
//      break;
//    }
//  }

//  /* Get UART Parity */
//  CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_parity = uart_handler->Init.Parity;

//  /* Get UART StopBits */
//  CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_stop_bit = uart_handler->Init.StopBits;

//  /* Set device class_cdc_acm with default parameters */
//  if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
//                                    &CDC_VCP_LineCoding) != UX_SUCCESS)
//  {
//    Error_Handler();
//  }

//  /* Receive an amount of data in interrupt mode */
//  if (HAL_UART_Receive_IT(uart_handler, (uint8_t *)UserTxBufferFS, 1) != HAL_OK)
//  {
//    /* Transfer error in reception process */
//    Error_Handler();
//  }

  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  /* Reset the cdc acm instance */
  cdc_acm = UX_NULL;

  /* DeInitialize the UART peripheral */
//  HAL_UART_DeInit(uart_handler);

  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  ULONG request;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;

  /* Get the pointer to the device.  */
  device = &_ux_system_slave -> ux_system_slave_device;

  /* Get the pointer to the transfer request associated with the control endpoint. */
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request)
  {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

      /* Get the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      /* Check if baudrate < 9600) then set it to 9600 */
      if (CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate < MIN_BAUDRATE)
      {
        CDC_VCP_LineCoding.ux_slave_class_cdc_acm_parameter_baudrate = MIN_BAUDRATE;

        /* Set the new configuration of ComPort */
//        USBD_CDC_VCP_Config(&CDC_VCP_LineCoding);
      }
      else
      {
        /* Set the new configuration of ComPort */
//        USBD_CDC_VCP_Config(&CDC_VCP_LineCoding);
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

      /* Set the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
    default :
      break;
  }

  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

int ux_device_cdc_acm_send(uint8_t *datas, uint32_t len, uint32_t timeout)
{
    if (cdc_acm)
    {
        if (UX_SUCCESS == ux_device_class_cdc_acm_write_with_callback(cdc_acm, datas, len))
        {
            /* wait semaphore */
					if (pdTRUE == xSemaphoreTake(g_xUSBUARTSend, timeout))
						return 0;
					else
						return -1; /* timeout */
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }
		return 0;
}

int ux_device_cdc_acm_getchar(uint8_t *pData, uint32_t timeout)
{
	if (g_xUSBUART_RX_Queue)
	{
		if (pdPASS == xQueueReceive(g_xUSBUART_RX_Queue, pData, timeout))
			return 0;
		else
			return -1;
	}
	else
	{
		return -1;
	}
}

int ux_device_cdc_acm_flush(void)
{
	int cnt = 0;
	uint8_t data;
	while (1)
	{
		if (pdPASS != xQueueReceive(g_xUSBUART_RX_Queue, &data, 0))
			break;
		cnt++;
	}
	return cnt;
}


/* USER CODE END 1 */
