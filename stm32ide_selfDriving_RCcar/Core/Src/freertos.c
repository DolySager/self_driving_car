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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_ultsnc_left(void *argument);
void StartTask_ultsnc_center(void *argument);
void StartTask_ultsnc_right(void *argument);

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
	  uint32_t echo_pulse_width_us = 0;
	  HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_SET);
	  delay_us(10);
	  HAL_GPIO_WritePin(TRIG_LEFT_GPIO_Port, TRIG_LEFT_Pin, GPIO_PIN_RESET);
	  while (HAL_GPIO_ReadPin(ECHO_LEFT_GPIO_Port, ECHO_LEFT_Pin) == GPIO_PIN_RESET);
	  while (HAL_GPIO_ReadPin(ECHO_LEFT_GPIO_Port, ECHO_LEFT_Pin) == GPIO_PIN_SET)
	  {
		  ++echo_pulse_width_us;
		  delay_us(1);
	  }
	  printf("LEFT Pulse width: %d us\n", echo_pulse_width_us);
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

    osDelay(1);
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
    osDelay(1);
  }
  /* USER CODE END StartTask_ultsnc_right */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

