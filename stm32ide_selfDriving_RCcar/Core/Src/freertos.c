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
float pid_process (float current_value, float target_value, float kp, float ki, float kd, float dt, float* integral_ptr, float* prev_value_ptr)
{
	float error = target_value - current_value;
	float p_term = kp * error;
	*integral_ptr += error * dt;
	float i_term = ki * *integral_ptr;
	float d_term = kd * (current_value - *prev_value_ptr) / dt;
	*prev_value_ptr = current_value;

	return p_term + i_term - d_term;
}

#define LEFT_MOTOR_MIN_SPEED 36
#define RIGHT_MOTOR_MIN_SPEED 34
#define STRAIGHT_SENSOR_VALUE_AT_MAX_SPEED 2000
#define STRAIGHT_SENSOR_VALUE_THRESHOLD 600
#define CURVE_SENSOR_VALUE_AT_MAX_SPEED 3000
#define MOTOR_ERROR_PERCENTAGE_TOLERANCE 5
#define DEADLOCK_THRESHOLD_VALUE 400
#define SAMPLE_TIME_S 0.01
float center_prev_value = 0, curve_prev_value_left = 0, curve_prev_value_right = 0;
/* USER CODE END Header_StartTask_auto_drive */
void StartTask_auto_drive(void *argument)
{
  /* USER CODE BEGIN StartTask_auto_drive */
	deadlock_threshold = DEADLOCK_THRESHOLD_VALUE;
	osDelay(1000);
	/* Infinite loop */
	for(;;)
	{
	  if (mode_auto_manu || mode_monitor_on_off)
	  {
		// deadlock escape mode if cornered
		if ((echo_left_time_us < deadlock_threshold) && (echo_center_time_us < deadlock_threshold) && (echo_right_time_us < deadlock_threshold))
		{
			if (!mode_deadlock_normal)
			{
				mode_deadlock_normal = 1;
				if (mode_monitor_on_off) printf("Deadlock mode on\n");
				deadlock_threshold = DEADLOCK_THRESHOLD_VALUE * 2;
			}
		}
		// reject sensor value if it is touching wall
		else if (echo_left_time_us < 5000 && echo_center_time_us < 5000 && echo_right_time_us < 5000)
		{
			if (mode_deadlock_normal)
			{
				mode_deadlock_normal = 0;
				if (mode_monitor_on_off) printf("Deadlock mode off\n");
				deadlock_threshold = STRAIGHT_SENSOR_VALUE_THRESHOLD;
			}
		}

		float left_motor_straight, left_motor_curve, right_motor_straight, right_motor_curve;

		// Straight speed calculate
		left_motor_straight = pid_process (echo_center_time_us, STRAIGHT_SENSOR_VALUE_THRESHOLD, -1 * kps, -1 * kis, -1 * kds, SAMPLE_TIME_S, &center_integral, &center_prev_value);
		right_motor_straight = left_motor_straight;

		// adjust sensor value to percentage
		left_motor_straight = left_motor_straight / STRAIGHT_SENSOR_VALUE_AT_MAX_SPEED * 100;
		right_motor_straight = right_motor_straight / STRAIGHT_SENSOR_VALUE_AT_MAX_SPEED * 100;

		// truncate if above 100
	//	if (left_motor_straight > 100) left_motor_straight = 100;
	//	else if (left_motor_straight < -100) left_motor_straight = -100;
	//	if (right_motor_straight > 100) right_motor_straight = 100;
	//	else if (right_motor_straight < -100) right_motor_straight = -100;

		// Curve speed calculate (if deadlocked, force curve value to hard curve)
		float curve_current_value = echo_left_time_us - echo_right_time_us;
		if (mode_deadlock_normal)
		{
			left_motor_straight = -50;
			right_motor_straight = -50;
			if (arbitrary_turn_right_left)
			{
				left_motor_curve = 100;
				right_motor_curve = -100;

			}
			else
			{
				left_motor_curve = -100;
				right_motor_curve = 100;
			}
		}
		else
		{
			left_motor_curve = pid_process (curve_current_value, 0, kpc, kic, kdc, SAMPLE_TIME_S, &curve_integral_left, &curve_prev_value_left);
			right_motor_curve = pid_process (curve_current_value, 0, -1 * kpc, -1 * kic, -1 * kdc, SAMPLE_TIME_S, &curve_integral_right, &curve_prev_value_right);

			// adjust sensor value to percentage
			left_motor_curve = left_motor_curve / CURVE_SENSOR_VALUE_AT_MAX_SPEED * 100;
			right_motor_curve = right_motor_curve / CURVE_SENSOR_VALUE_AT_MAX_SPEED * 100;

			// truncate above 100
	//		if (left_motor_curve > 100) left_motor_curve = 100;
	//		else if (left_motor_curve < -100) left_motor_curve = -100;
	//		if (right_motor_curve > 100) right_motor_curve = 100;
	//		else if (right_motor_curve < -100) right_motor_curve = -100;
		}

		// straight percentage calculate
		float left_motor_straight_percent, right_motor_straight_percent;
		if (left_motor_straight >= 0) left_motor_straight_percent = left_motor_straight;
		else left_motor_straight_percent = -1 * left_motor_straight;
		if (right_motor_straight >= 0) right_motor_straight_percent = right_motor_straight;
		else right_motor_straight_percent = -1 * right_motor_straight;

		// curve percentage calculate
		float left_curve_percentage, right_curve_percentage;
		if (left_motor_curve > 0) left_curve_percentage = left_motor_curve;
		else left_curve_percentage = -1 * left_motor_curve;
		if (right_motor_curve > 0) right_curve_percentage = right_motor_curve;
		else right_curve_percentage = -1 * right_motor_curve;

		// straight and curve percentage combine. The more value, the more weight
		float left_straight_curve_sum = left_motor_straight_percent + left_curve_percentage;
		float right_straight_curve_sum = right_motor_straight_percent + right_curve_percentage;
		left_motor_duty_float = left_motor_straight_percent / left_straight_curve_sum * left_motor_straight + left_curve_percentage / left_straight_curve_sum * left_motor_curve;
		right_motor_duty_float = right_motor_straight_percent / right_straight_curve_sum * right_motor_straight + right_curve_percentage / right_straight_curve_sum * right_motor_curve;

		// truncate above maximum value
		if (left_motor_duty_float > 100) left_motor_duty_float = 100;
		else if (left_motor_duty_float < -100) left_motor_duty_float = -100;
		if (right_motor_duty_float > 100) right_motor_duty_float = 100;
		else if (right_motor_duty_float < -100) right_motor_duty_float = -100;

		// motor value normalize (0~100 to MIN_VAL~100)
		if (left_motor_duty_float < 0.02 &&  left_motor_duty_float > -0.02) left_motor_duty_float = 0;
		else if (left_motor_duty_float > 0) left_motor_duty_float = left_motor_duty_float / 100 * (100 - 43) + 43;
		else if (left_motor_duty_float < 0) left_motor_duty_float = left_motor_duty_float / 100 * (100 - 43) - 43;
		if (right_motor_duty_float < 0.02 &&  right_motor_duty_float > -0.02) right_motor_duty_float = 0;
		else if (right_motor_duty_float > 0) right_motor_duty_float = right_motor_duty_float / 100 * (100 - 43) + 43;
		else if (right_motor_duty_float < 0) right_motor_duty_float = right_motor_duty_float / 100 * (100 - 43) - 43;

		// convert to integer value
		left_motor_duty_int = (int) left_motor_duty_float;
		right_motor_duty_int = (int) right_motor_duty_float;

		// motor output compensation
		if (left_motor_duty_int >= 0) left_motor_duty_int -= 3;
		else left_motor_duty_int += 3;

		if (mode_auto_manu) RCcar_set_motor_speed(left_motor_duty_int, right_motor_duty_int);
	  }

	  osDelay(SAMPLE_TIME_S * 1000.0);
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

	if (echo_left_fall_time > echo_left_rise_time) echo_left_time_us_temp = echo_left_fall_time - echo_left_rise_time;
	if (echo_center_fall_time > echo_center_rise_time) echo_center_time_us_temp = echo_center_fall_time - echo_center_rise_time;
	if (echo_right_fall_time > echo_right_rise_time) echo_right_time_us_temp = echo_right_fall_time - echo_right_rise_time;

	// reject too high values
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

    osDelay(SAMPLE_TIME_S * 1000.0);
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
  osDelay(1000);
  /* Infinite loop */
  for(;;)
  {
	  if (mode_monitor_on_off)
	  {
		  printf("%5lu %5lu %5lu || ", echo_left_time_us, echo_center_time_us, echo_right_time_us);
		  printf("L: %+3d R:%+3d\n", left_motor_duty_int, right_motor_duty_int);
	  }

	  arbitrary_turn_right_left = !arbitrary_turn_right_left;

	  osDelay(1000);
  }
  /* USER CODE END StartTask_print_sensor_value */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

