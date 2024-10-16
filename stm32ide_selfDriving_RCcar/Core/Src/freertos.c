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
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
	// apply motor values one step later
	if (mode_auto_manu) RCcar_set_motor_speed(left_motor_duty_int, right_motor_duty_int);

	float distance_sensitivity = CONSTANT_CALIBRATION_MAX - (echo_left_time_us + echo_center_time_us + echo_right_time_us) / 3.0 / 2000.0;
	if (distance_sensitivity > CONSTANT_CALIBRATION_MAX) distance_sensitivity = CONSTANT_CALIBRATION_MAX;
	else if (distance_sensitivity < CONSTANT_CALIBRATION_MIN) distance_sensitivity = CONSTANT_CALIBRATION_MIN;

	float curve_sensitivity = CONSTANT_CALIBRATION_MIN + (echo_left_time_us - echo_right_time_us) / 2000.0;
	if (curve_sensitivity < 0) curve_sensitivity *= -1;
	if (curve_sensitivity > CONSTANT_CALIBRATION_MAX) curve_sensitivity = CONSTANT_CALIBRATION_MAX;
	else if (curve_sensitivity < CONSTANT_CALIBRATION_MIN) curve_sensitivity = CONSTANT_CALIBRATION_MIN;

	// and then calculate motor power for next step
	left_motor_duty_float = (   ( (K_SAMESIDE * distance_sensitivity * curve_sensitivity) * ( (float) echo_left_time_us - DIRECTION_THRESHOLD) / MAX_CURVE_SENSOR_VALUE) + \
								(K_CENTER * ( (float) echo_center_time_us - DIRECTION_THRESHOLD) / MAX_STRAIGHT_SENSOR_VALUE) + \
								( (K_OPPSIDE * distance_sensitivity * curve_sensitivity) * ( (float) echo_right_time_us - DIRECTION_THRESHOLD) / MAX_CURVE_SENSOR_VALUE ) )  \
								 / 3 * 100;

	right_motor_duty_float = (  ( (K_SAMESIDE * distance_sensitivity * curve_sensitivity) * ( (float) echo_right_time_us - DIRECTION_THRESHOLD) / MAX_CURVE_SENSOR_VALUE)+ \
								(K_CENTER * ( (float) echo_center_time_us - DIRECTION_THRESHOLD) / MAX_STRAIGHT_SENSOR_VALUE)+ \
								( (K_OPPSIDE * distance_sensitivity * curve_sensitivity) * ( (float) echo_left_time_us - DIRECTION_THRESHOLD) / MAX_CURVE_SENSOR_VALUE )) \
								 / 3 * 100;

	// deadlock escape
	if (((echo_left_time_us + echo_center_time_us + echo_right_time_us) / 3.0) < deadlock_threshold)
	{
		if (arbitrary_turn_right_left)
		{
			if (left_motor_duty_float > 0) left_motor_duty_float *= CONSTANT_DEADLOCK_MAX;
			else left_motor_duty_float *= CONSTANT_DEADLOCK_MIN;
			if (right_motor_duty_float > 0) right_motor_duty_float *= CONSTANT_DEADLOCK_MIN;
			else right_motor_duty_float *= CONSTANT_DEADLOCK_MAX;
		}
		else
		{
			if (left_motor_duty_float > 0) left_motor_duty_float *= CONSTANT_DEADLOCK_MIN;
			else left_motor_duty_float *= CONSTANT_DEADLOCK_MAX;
			if (right_motor_duty_float > 0) right_motor_duty_float *= CONSTANT_DEADLOCK_MAX;
			else right_motor_duty_float *= CONSTANT_DEADLOCK_MIN;
		}
		deadlock_threshold = DIRECTION_THRESHOLD + 400;
	}
	else
	{
		deadlock_threshold = DIRECTION_THRESHOLD;
	}

	// left motor value correction
	if (left_motor_duty_float > 100) left_motor_duty_float = 100;
	else if (left_motor_duty_float < -100) left_motor_duty_float = -100;
	else
	{
		if (left_motor_duty_float > 0) left_motor_duty_float = 0.6 * left_motor_duty_float + MINIMUM_MOTOR_DUTY;	// instead of 0 to 100, set to 40 to 100 linearly
		else if (left_motor_duty_float < 0) left_motor_duty_float = 0.6 * left_motor_duty_float - MINIMUM_MOTOR_DUTY;
	}


	// right motor value correction
	if (right_motor_duty_float > 100) right_motor_duty_float = 100;
	else if (right_motor_duty_float < -100) right_motor_duty_float = -100;
	else
	{
		if (right_motor_duty_float > 0) right_motor_duty_float = 0.6 * right_motor_duty_float + MINIMUM_MOTOR_DUTY;	// instead of 0 to 100, set to 40 to 100 linearly
		else if (right_motor_duty_float < 0) right_motor_duty_float = 0.6 * right_motor_duty_float - MINIMUM_MOTOR_DUTY;
		deadlock_threshold = DIRECTION_THRESHOLD;
	}


	left_motor_duty_int = (int) left_motor_duty_float;
	right_motor_duty_int = (int) right_motor_duty_float;

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
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
	int echo_left_time_us_temp, echo_center_time_us_temp, echo_right_time_us_temp;

	if (echo_left_fall_time > echo_left_rise_time && SENSOR_VALUE_THRESHOLD) echo_left_time_us_temp = echo_left_fall_time - echo_left_rise_time;
	if (echo_center_fall_time > echo_center_rise_time) echo_center_time_us_temp = echo_center_fall_time - echo_center_rise_time;
	if (echo_right_fall_time > echo_right_rise_time) echo_right_time_us_temp = echo_right_fall_time - echo_right_rise_time;

	if (echo_left_time_us_temp < SENSOR_VALUE_THRESHOLD) echo_left_time_queue[echo_left_time_queue_index++] = echo_left_time_us_temp;
	if (echo_center_time_us_temp < SENSOR_VALUE_THRESHOLD) echo_center_time_queue[echo_center_time_queue_index++] = echo_center_time_us_temp;
	if (echo_right_time_us_temp < SENSOR_VALUE_THRESHOLD) echo_right_time_queue[echo_right_time_queue_index++] = echo_right_time_us_temp;

	int echo_left_sum = 0, echo_center_sum = 0, echo_right_sum = 0;
	for (uint8_t i=0; i<VALUE_QUEUE_SIZE; ++i)
	{

		echo_left_sum += echo_left_time_queue[i];
		echo_center_sum += echo_center_time_queue[i];
		echo_right_sum += echo_right_time_queue[i];
	}

	echo_left_time_us = echo_left_sum / VALUE_QUEUE_SIZE;
	echo_center_time_us = echo_center_sum / VALUE_QUEUE_SIZE;
	echo_right_time_us = echo_right_sum / VALUE_QUEUE_SIZE;

	if (echo_left_time_queue_index >= VALUE_QUEUE_SIZE) echo_left_time_queue_index = 0;
	if (echo_center_time_queue_index >= VALUE_QUEUE_SIZE) echo_center_time_queue_index = 0;
	if (echo_right_time_queue_index >= VALUE_QUEUE_SIZE) echo_right_time_queue_index = 0;

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
  osDelay(100);
  /* Infinite loop */
  for(;;)
  {
	printf("%5lu %5lu %5lu  ", echo_left_time_us, echo_center_time_us, echo_right_time_us);
	printf("L: %+5d R:%+5d\n", left_motor_duty_int, right_motor_duty_int);

	arbitrary_turn_right_left = !arbitrary_turn_right_left;

    osDelay(1000);
  }
  /* USER CODE END StartTask_print_sensor_value */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

