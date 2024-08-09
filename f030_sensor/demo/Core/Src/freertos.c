/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "modbus.h"
#include "errno.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

extern int errno;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define 	SENOR_MONITOR

#ifdef SENOR_SWITCH

#define SLAVE_ADDR						1
#define NB_BITS							5
#define NB_INPUT_BITS					3
#define NB_REGISTERS					0
#define NB_INPUT_REGISTERS				0

#endif

#ifdef SENOR_MONITOR

extern ADC_HandleTypeDef hadc;

#define SLAVE_ADDR						2
#define NB_BITS							5
#define NB_INPUT_BITS					0
#define NB_REGISTERS					0
#define NB_INPUT_REGISTERS				2


#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	  
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  	GPIO_PinState val;
	uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
	
	ctx = modbus_new_st_rtu("uart1", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, SLAVE_ADDR);
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

#ifdef SENOR_MONITOR

 	/* caculate the adc */
	HAL_ADCEx_Calibration_Start(&hadc);

#endif

	mb_mapping = modbus_mapping_new_start_address(0,
												  NB_BITS,
												  0,
												  NB_INPUT_BITS,
												  0,
												  NB_REGISTERS,
												  0,
												  NB_INPUT_REGISTERS);
	

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
		
#ifdef SENOR_SWITCH
		/* update values of R/D bits */
		/* key1~3 -> PA3~5 
		*	keyddown -> posedge
		*/
		val = HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin);
		if(val)
			mb_mapping->tab_input_bits[0] = 0;
		else
			mb_mapping->tab_input_bits[0] = 1;

		val = HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin);
		if(val)
			mb_mapping->tab_input_bits[1] = 0;
		else
			mb_mapping->tab_input_bits[1] = 1;

		val = HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin);
		if(val)
			mb_mapping->tab_input_bits[2] = 0;
		else
			mb_mapping->tab_input_bits[2] = 1;		

#endif

#ifdef SENOR_MONITOR

		/* update adc values */
		for (int i = 0; i < 2; i++)
		{
			HAL_ADC_Start(&hadc);
			if (HAL_OK == HAL_ADC_PollForConversion(&hadc, 100))
			{
				mb_mapping->tab_input_registers[i] = HAL_ADC_GetValue(&hadc);
			}
		}

#endif

		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			//break;
		}

#ifdef SENOR_SWITCH
		if (mb_mapping->tab_bits[0])
      HAL_GPIO_WritePin(Relay_K1_GPIO_Port, Relay_K1_Pin, GPIO_PIN_SET); //Relay1
		else
			HAL_GPIO_WritePin(Relay_K1_GPIO_Port, Relay_K1_Pin, GPIO_PIN_RESET); 

		if (mb_mapping->tab_bits[1])
      HAL_GPIO_WritePin(Relay_K2_GPIO_Port, Relay_K2_Pin, GPIO_PIN_SET); //Relay2
		else
			HAL_GPIO_WritePin(Relay_K2_GPIO_Port, Relay_K2_Pin, GPIO_PIN_RESET); 

		if (mb_mapping->tab_bits[2])
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); //LED1
		else
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); 
		if (mb_mapping->tab_bits[3])
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); //LED2
		else
			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); 
		if (mb_mapping->tab_bits[4])
      HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); //LED3
		else
			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET); 		

#endif

#ifdef SENOR_MONITOR

		/* ctrl the led and beep */
	  if (mb_mapping->tab_bits[0])
	HAL_GPIO_WritePin(BEEP1_GPIO_Port, BEEP1_Pin, GPIO_PIN_SET); //beep1
	  else
		  HAL_GPIO_WritePin(BEEP1_GPIO_Port, BEEP1_Pin, GPIO_PIN_RESET); 
	
	  if (mb_mapping->tab_bits[1])
	HAL_GPIO_WritePin(BEEP2_GPIO_Port, BEEP2_Pin, GPIO_PIN_SET); //beep2
	  else
		  HAL_GPIO_WritePin(BEEP2_GPIO_Port, BEEP2_Pin, GPIO_PIN_RESET); 
	
	  if (mb_mapping->tab_bits[2])
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); //LED1
	  else
		  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); 
	  if (mb_mapping->tab_bits[3])
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); //LED2
	  else
		  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET); 
	  if (mb_mapping->tab_bits[4])
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); //LED3
	  else
		  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);	  


#endif

	}

	modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

