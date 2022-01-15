/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <stdio.h>
#include <stdlib.h>

#include "i2c.h"
#include "usart.h"
#include "tim.h"

#include "BMPXX80.h"
#include "LCD_HD44780.h"
#include "PID_Controller.h"
#include "printf.h"
#include "ring_buffer.h"
#include "Saturation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	float setpoint;
	float measured;
} LcdMessage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
RingBuffer_t buffer;
uint8_t received_char;
/* USER CODE END Variables */
/* Definitions for LCDTask */
osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCDTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle;
const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for HeartBeatTask */
osThreadId_t HeartBeatTaskHandle;
const osThreadAttr_t HeartBeatTask_attributes = {
  .name = "HeartBeatTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ParsingTask */
osThreadId_t ParsingTaskHandle;
const osThreadAttr_t ParsingTask_attributes = {
  .name = "ParsingTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempQueue */
osMessageQueueId_t TempQueueHandle;
const osMessageQueueAttr_t TempQueue_attributes = {
  .name = "TempQueue"
};
/* Definitions for SetpointQueue */
osMessageQueueId_t SetpointQueueHandle;
const osMessageQueueAttr_t SetpointQueue_attributes = {
  .name = "SetpointQueue"
};
/* Definitions for LcdDataTimer */
osTimerId_t LcdDataTimerHandle;
const osTimerAttr_t LcdDataTimer_attributes = {
  .name = "LcdDataTimer"
};
/* Definitions for UartMutex */
osMutexId_t UartMutexHandle;
const osMutexAttr_t UartMutex_attributes = {
  .name = "UartMutex"
};
/* Definitions for TempQueueSemaphore */
osSemaphoreId_t TempQueueSemaphoreHandle;
const osSemaphoreAttr_t TempQueueSemaphore_attributes = {
  .name = "TempQueueSemaphore"
};
/* Definitions for ReadLinesCountingSem */
osSemaphoreId_t ReadLinesCountingSemHandle;
const osSemaphoreAttr_t ReadLinesCountingSem_attributes = {
  .name = "ReadLinesCountingSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLCDTask(void *argument);
void StartControllerTask(void *argument);
void HeartBeatTaskTask(void *argument);
void StartParsingTask(void *argument);
void LcdDataTimerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of UartMutex */
  UartMutexHandle = osMutexNew(&UartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of TempQueueSemaphore */
  TempQueueSemaphoreHandle = osSemaphoreNew(1, 1, &TempQueueSemaphore_attributes);

  /* creation of ReadLinesCountingSem */
  ReadLinesCountingSemHandle = osSemaphoreNew(4, 0, &ReadLinesCountingSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of LcdDataTimer */
  LcdDataTimerHandle = osTimerNew(LcdDataTimerCallback, osTimerPeriodic, NULL, &LcdDataTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of TempQueue */
  TempQueueHandle = osMessageQueueNew (8, sizeof(LcdMessage_t), &TempQueue_attributes);

  /* creation of SetpointQueue */
  SetpointQueueHandle = osMessageQueueNew (8, sizeof(float), &SetpointQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LCDTask */
  LCDTaskHandle = osThreadNew(StartLCDTask, NULL, &LCDTask_attributes);

  /* creation of ControllerTask */
  ControllerTaskHandle = osThreadNew(StartControllerTask, NULL, &ControllerTask_attributes);

  /* creation of HeartBeatTask */
  HeartBeatTaskHandle = osThreadNew(HeartBeatTaskTask, NULL, &HeartBeatTask_attributes);

  /* creation of ParsingTask */
  ParsingTaskHandle = osThreadNew(StartParsingTask, NULL, &ParsingTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartLCDTask */
/**
  * @brief  Function implementing the LCDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
	LcdMessage_t message;
	uint8_t text[16];

	HAL_TIM_Base_Start(&htim4);
	LCD_Init();
  /* Infinite loop */
  for(;;)
  {
	  if (osOK == osMessageQueueGet(TempQueueHandle, &message, NULL, osWaitForever)) {
		  LCD_Cls();

		  sprintf((char*)text, "Zad. : %.2f C", message.setpoint);
		  taskENTER_CRITICAL();
		  LCD_Locate(0,0);
		  LCD_String((char*)text);
		  taskEXIT_CRITICAL();

		  sprintf((char*)text, "Temp.: %.2f C", message.measured);
		  taskENTER_CRITICAL();
		  LCD_Locate(0,1);
		  LCD_String((char*)text);
		  taskEXIT_CRITICAL();

		  printf("Zad. : %.2f C\r\n", message.setpoint);
		  printf("Temp.: %.2f C\r\n", message.measured);
	  }
  }
  /* USER CODE END StartLCDTask */
}

/* USER CODE BEGIN Header_StartControllerTask */
/**
* @brief Function implementing the ControllerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControllerTask */
void StartControllerTask(void *argument)
{
  /* USER CODE BEGIN StartControllerTask */
	LcdMessage_t message;
	float temperature;
	float setpoint = 27.f;
	float pid_output;

	Saturation saturation = {.lower_bound = 0, .upper_bound = 1};
	const float P = 1.3;
	const float I = 0.026;
	const float D = 5;
	PID_Controller controller = PID_Create_Saturation(P, I, D, 0.1, &saturation);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	BMP280_Init(&hi2c1, BMP280_TEMPERATURE_16BIT, BMP280_STANDARD, BMP280_FORCEDMODE);
	BMP280_SetConfig(BME280_STANDBY_MS_0_5, BME280_FILTER_X16);

	osTimerStart(LcdDataTimerHandle, 1000);

	uint32_t tick = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(SetpointQueueHandle, &setpoint, 0, 0);

	  taskENTER_CRITICAL();
	  temperature = BMP280_ReadTemperature();
	  pid_output = calculate_pid(&controller, setpoint - temperature);
	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)(pid_output * 999));
	  taskEXIT_CRITICAL();

	  message.setpoint = setpoint;
	  message.measured = temperature;
	  if (osOK == osSemaphoreAcquire(TempQueueSemaphoreHandle, 0)) {
		  osMessageQueuePut(TempQueueHandle, &message, 0, 0);
	  }

	  tick += 100;
	  osDelayUntil(tick);
  }
  /* USER CODE END StartControllerTask */
}

/* USER CODE BEGIN Header_HeartBeatTaskTask */
/**
* @brief Function implementing the HeartBeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HeartBeatTaskTask */
void HeartBeatTaskTask(void *argument)
{
  /* USER CODE BEGIN HeartBeatTaskTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
	  osDelay(500);
  }
  /* USER CODE END HeartBeatTaskTask */
}

/* USER CODE BEGIN Header_StartParsingTask */
/**
* @brief Function implementing the ParsingTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartParsingTask */
void StartParsingTask(void *argument)
{
  /* USER CODE BEGIN StartParsingTask */
	uint8_t received_data[16];
	float new_setpoint;
	Saturation saturation = {.lower_bound = 25.f, .upper_bound = 30.f};
	HAL_UART_Receive_IT(&huart3, &received_char, 1);
  /* Infinite loop */
  for(;;)
  {
	  if (osOK == osSemaphoreAcquire(ReadLinesCountingSemHandle, osWaitForever)) {
		  RB_TakeLine(&buffer, received_data);

		  new_setpoint = atoff((char*)received_data);
		  if (new_setpoint != 0.0f) {
			  new_setpoint = calculate_saturation(new_setpoint, &saturation);
			  osMessageQueuePut(SetpointQueueHandle, &new_setpoint, 0, 0);
		  }
	  }
  }
  /* USER CODE END StartParsingTask */
}

/* LcdDataTimerCallback function */
void LcdDataTimerCallback(void *argument)
{
  /* USER CODE BEGIN LcdDataTimerCallback */
	osSemaphoreRelease(TempQueueSemaphoreHandle);
  /* USER CODE END LcdDataTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character) {
	osMutexAcquire(UartMutexHandle, osWaitForever);
	HAL_UART_Transmit(&huart3, (uint8_t*)&character, 1, 1000);
	osMutexRelease(UartMutexHandle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		 if (RB_OK == RB_Write(&buffer, received_char)) {
			 if (received_char == '\n') {
				 osSemaphoreRelease(ReadLinesCountingSemHandle);
			 }
		 }

		 HAL_UART_Receive_IT(&huart3, &received_char, 1);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
