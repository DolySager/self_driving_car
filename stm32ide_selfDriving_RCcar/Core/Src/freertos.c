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
/* Definitions for auto_drive_task */
osThreadId_t auto_drive_taskHandle;
const osThreadAttr_t auto_drive_task_attributes = {
  .name = "auto_drive_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for get_echo_time */
osThreadId_t get_echo_timeHandle;
const osThreadAttr_t get_echo_time_attributes = {
  .name = "get_echo_time",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for print_sensor_va */
osThreadId_t print_sensor_vaHandle;
const osThreadAttr_t print_sensor_va_attributes = {
  .name = "print_sensor_va",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_auto_drive(void *argument);
void StartTask_get_echo_time(void *argument);
void StartTask_print_sensor_value(void *argument);

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
  /* creation of auto_drive_task */
  auto_drive_taskHandle = osThreadNew(StartTask_auto_drive, NULL, &auto_drive_task_attributes);

  /* creation of get_echo_time */
  get_echo_timeHandle = osThreadNew(StartTask_get_echo_time, NULL, &get_echo_time_attributes);

  /* creation of print_sensor_va */
  print_sensor_vaHandle = osThreadNew(StartTask_print_sensor_value, NULL, &print_sensor_va_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

	float k_straight = (echo_center_time_us - DIRECTION_THRESHOLD) * STRAIGHT_SENSITIVITY;
	if (k_straight > 100.0) k_straight = 100.0;

	float k_curve = (echo_left_time_us - echo_right_time_us) * TURN_SENSITIVITY;
	if (k_curve > 100.0) k_curve = 100.0;
	else if (k_curve < -100.0) k_curve = -100.0;

	float k_curve_factor;
	if (k_curve >= 0) k_curve_factor = k_curve;
	else k_curve_factor = -1 * k_curve;

	left_motor_duty_float = k_straight + ((100.0 - k_curve_factor) / 100.0 * k_curve);
	right_motor_duty_float = k_straight - ((100.0 - k_curve_factor) / 100.0 * k_curve);

	left_motor_duty_int_raw = (int) left_motor_duty_float;
	right_motor_duty_int_raw = (int) right_motor_duty_float;

	// left motor correction
	if (left_motor_duty_float > 100) left_motor_duty_int = 100;
	else if (left_motor_duty_float < -100) left_motor_duty_int = -100;
	else if (left_motor_duty_float < 40 && left_motor_duty_float >= 0) left_motor_duty_int = 40;
	else if (left_motor_duty_float > -40 && left_motor_duty_float < 0) left_motor_duty_int = -40;
	else left_motor_duty_int = (int) left_motor_duty_float;

	// right motor correction
	if (right_motor_duty_float > 100) right_motor_duty_int = 100;
	else if (right_motor_duty_float < -100) right_motor_duty_int = -100;
	else if (right_motor_duty_float < 40 && right_motor_duty_float >= 0) right_motor_duty_int = 40;
	else if (right_motor_duty_float > -40 && right_motor_duty_float < 0) right_motor_duty_int = -40;
	else right_motor_duty_int = (int) right_motor_duty_float;

	// arbitrary turn when too close to wall
	if ((left_motor_duty_int - right_motor_duty_int) < 25 && (left_motor_duty_int - right_motor_duty_int) > -25)
	{
		if (arbitrary_turn_right_left) right_motor_duty_int += ARBITRARY_TURN_POWER_COMPENSATION;
		else left_motor_duty_int += ARBITRARY_TURN_POWER_COMPENSATION;
	}

	if (mode_auto_manu) RCcar_set_motor_speed(left_motor_duty_int, right_motor_duty_int);

	osDelay(10);
  }
  /* USER CODE END StartTask_auto_drive */
}

/* USER CODE BEGIN Header_StartTask_get_echo_time */
/**
* @brief Function implementing the get_echo_time thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_get_echo_time */
void StartTask_get_echo_time(void *argument)
{
  /* USER CODE BEGIN StartTask_get_echo_time */
  /* Infinite loop */
  for(;;)
  {
	if (echo_left_fall_time > echo_left_rise_time) echo_left_time_us = echo_left_fall_time - echo_left_rise_time;
	if (echo_center_fall_time > echo_center_rise_time) echo_center_time_us = echo_center_fall_time - echo_center_rise_time;
	if (echo_right_fall_time > echo_right_rise_time) echo_right_time_us = echo_right_fall_time - echo_right_rise_time;

    osDelay(10);
  }
  /* USER CODE END StartTask_get_echo_time */
}

/* USER CODE BEGIN Header_StartTask_print_sensor_value */
/**
* @brief Function implementing the print_sensor_va thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_print_sensor_value */
void StartTask_print_sensor_value(void *argument)
{
  /* USER CODE BEGIN StartTask_print_sensor_value */
  /* Infinite loop */
  for(;;)
  {
	printf("%5lu %5lu %5lu  ", echo_left_time_us, echo_center_time_us, echo_right_time_us);
	printf("L: %+5d R:%+5d\n", left_motor_duty_int_raw, right_motor_duty_int_raw);

	arbitrary_turn_right_left = !arbitrary_turn_right_left;

    osDelay(1000);
  }
  /* USER CODE END StartTask_print_sensor_value */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

