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

/* USER CODE END Variables */
/* Definitions for ultsnc_left */
osThreadId_t ultsnc_leftHandle;
const osThreadAttr_t ultsnc_left_attributes = {
  .name = "ultsnc_left",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ultsnc_center */
osThreadId_t ultsnc_centerHandle;
const osThreadAttr_t ultsnc_center_attributes = {
  .name = "ultsnc_center",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ultsnc_right */
osThreadId_t ultsnc_rightHandle;
const osThreadAttr_t ultsnc_right_attributes = {
  .name = "ultsnc_right",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for auto_drive */
osThreadId_t auto_driveHandle;
const osThreadAttr_t auto_drive_attributes = {
  .name = "auto_drive",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_ultsnc_left(void *argument);
void StartTask_ultsnc_center(void *argument);
void StartTask_ultsnc_right(void *argument);
void StartTask_auto_drive(void *argument);

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
  /* creation of ultsnc_left */
  ultsnc_leftHandle = osThreadNew(StartTask_ultsnc_left, NULL, &ultsnc_left_attributes);

  /* creation of ultsnc_center */
  ultsnc_centerHandle = osThreadNew(StartTask_ultsnc_center, NULL, &ultsnc_center_attributes);

  /* creation of ultsnc_right */
  ultsnc_rightHandle = osThreadNew(StartTask_ultsnc_right, NULL, &ultsnc_right_attributes);

  /* creation of auto_drive */
  auto_driveHandle = osThreadNew(StartTask_auto_drive, NULL, &auto_drive_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_ultsnc_left */
/**
  * @brief  Function implementing the ultsnc_left thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_ultsnc_left */
void StartTask_ultsnc_left(void *argument)
{
  /* USER CODE BEGIN StartTask_ultsnc_left */
  /* Infinite loop */
  for(;;)
  {
	  if (mode_auto_manu)
	  {
		  HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_SET);
		  delay_us(10);
		  HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_RESET);
		  __HAL_TIM_ENABLE_IT(&htim_echoMeasure, TIM_IT_CC1);
		  osDelay(100);
	  }
  }
  /* USER CODE END StartTask_ultsnc_left */
}

/* USER CODE BEGIN Header_StartTask_ultsnc_center */
/**
* @brief Function implementing the ultsnc_center thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ultsnc_center */
void StartTask_ultsnc_center(void *argument)
{
  /* USER CODE BEGIN StartTask_ultsnc_center */
  /* Infinite loop */
  for(;;)
  {
	  if (mode_auto_manu)
	  {
		  HAL_GPIO_WritePin(TRIG_CENTER_GPIO_Port, TRIG_CENTER_Pin, GPIO_PIN_SET);
		  delay_us(10);
		  HAL_GPIO_WritePin(TRIG_CENTER_GPIO_Port, TRIG_CENTER_Pin, GPIO_PIN_RESET);
		  __HAL_TIM_ENABLE_IT(&htim_echoMeasure, TIM_IT_CC2);
		  osDelay(100);
	  }
  }
  /* USER CODE END StartTask_ultsnc_center */
}

/* USER CODE BEGIN Header_StartTask_ultsnc_right */
/**
* @brief Function implementing the ultsnc_right thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_ultsnc_right */
void StartTask_ultsnc_right(void *argument)
{
  /* USER CODE BEGIN StartTask_ultsnc_right */
  /* Infinite loop */
  for(;;)
  {
	  if (mode_auto_manu)
	  {
		  HAL_GPIO_WritePin(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, GPIO_PIN_SET);
		  delay_us(10);
		  HAL_GPIO_WritePin(TRIG_RIGHT_GPIO_Port, TRIG_RIGHT_Pin, GPIO_PIN_RESET);
		  __HAL_TIM_ENABLE_IT(&htim_echoMeasure, TIM_IT_CC3);
		  osDelay(100);
	  }
  }
  /* USER CODE END StartTask_ultsnc_right */
}

/* USER CODE BEGIN Header_StartTask_auto_drive */
/**
* @brief Function implementing the auto_drive thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_auto_drive */
void StartTask_auto_drive(void *argument)
{
  /* USER CODE BEGIN StartTask_auto_drive */
  /* Infinite loop */
  for(;;)
  {
	if (mode_auto_manu)
	{
		for (uint8_t j=0; j<3; ++j)
		{
			uint32_t sum_tmp = 0;
			for (uint8_t i=0; i<10; ++i)
			{
				sum_tmp +=echo_time_queue[j][i];
			}
			echo_time_us[j] = sum_tmp / 10;
		}
		printf("%u\t%u\t%u\n",echo_time_us[0], echo_time_us[1], echo_time_us[2]);
		if (echo_time_us[1] > 1500) RCcar_go_forward(40);
		else if (echo_time_us[1] < 1000) RCcar_go_backward(40);
		else if (echo_time_us[2] < echo_time_us[0]) RCcar_go_soft_left(40);
		else RCcar_go_soft_right(40);
	    osDelay(100);
	}
  }
  /* USER CODE END StartTask_auto_drive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

