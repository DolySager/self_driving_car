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
// positive error means positive motor power. target and current value position swapped.
float pid_process (float current_value, float target_value, float kp, float ki, float kd, float dt, float* integral_ptr, float* prev_value_ptr)
{
	float error = current_value - target_value;
	float p_term = kp * error;
	*integral_ptr += error * dt;
	float i_term = ki * *integral_ptr;
	float d_term = kd * (current_value - *prev_value_ptr) / dt;
	*prev_value_ptr = current_value;

	return p_term + i_term - d_term;
}

#define SENSOR_VALUE_AT_MAX_SPEED 2000
#define SENSOR_VALUE_AT_DIR_CHANGE 600
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
		//////////////////////////
		// VARIABLE DECLARATION //
		//////////////////////////

		// Sensor value truncated
		float echo_left_time_us_trunc, echo_center_time_us_trunc, echo_right_time_us_trunc;

		// Straight direction motor power
		float straight_power;
		float straight_weight;

		// One-side motor power
		float left_side_power, right_side_power;
		float left_side_weight, right_side_weight;

		// Side-distance-difference motor power
		float diff_power;
		float diff_weight;

		// sum for weight calculation
		float left_motor_power_sum, right_motor_power_sum;

		if (mode_auto_manu || mode_monitor_on_off)
		{
			///////////////////////////
			// SENSOR VALUE TRUNCATE //
			///////////////////////////

			if (echo_left_time_us > SENSOR_VALUE_AT_MAX_SPEED) echo_left_time_us_trunc = SENSOR_VALUE_AT_MAX_SPEED;
			else echo_left_time_us_trunc = echo_left_time_us;
			if (echo_center_time_us > SENSOR_VALUE_AT_MAX_SPEED) echo_center_time_us_trunc = SENSOR_VALUE_AT_MAX_SPEED;
			else echo_center_time_us_trunc = echo_center_time_us;
			if (echo_right_time_us > SENSOR_VALUE_AT_MAX_SPEED) echo_right_time_us_trunc = SENSOR_VALUE_AT_MAX_SPEED;
			else echo_right_time_us_trunc = echo_right_time_us;


			////////////////////////
			// DEADLOCK DETECTION //
			////////////////////////

			if ((echo_left_time_us_trunc < deadlock_threshold) && (echo_center_time_us_trunc < deadlock_threshold) && (echo_right_time_us_trunc < deadlock_threshold))
			{
				if (!mode_deadlock_normal)
				{
					mode_deadlock_normal = 1;
					if (mode_monitor_on_off) printf("Deadlock mode on\n");
					deadlock_threshold = DEADLOCK_THRESHOLD_VALUE * 2;
				}
			}
			// reject sensor value if it is touching wall
			else if (echo_left_time_us_trunc > deadlock_threshold && echo_center_time_us_trunc > deadlock_threshold && echo_right_time_us_trunc > deadlock_threshold)
			{
				if (mode_deadlock_normal)
				{
					mode_deadlock_normal = 0;
					if (mode_monitor_on_off) printf("Deadlock mode off\n");
					deadlock_threshold = SENSOR_VALUE_AT_DIR_CHANGE;
				}
			}

			////////////////
			// AUTO-DRIVE //
			////////////////

			// Hard-coded value if deadlocked
			if (mode_deadlock_normal)
			{
				straight_power = -50;
				diff_power = -100;
				if (arbitrary_turn_right_left)
				{
					left_side_power = 100;
					right_side_power = -100;

				}
				else
				{
					left_side_power = -100;
					right_side_power = 100;
				}
			}

			// Normal operation
			else
			{
				straight_power = pid_process (echo_center_time_us_trunc, SENSOR_VALUE_AT_DIR_CHANGE, kps, kis, kds, SAMPLE_TIME_S, &center_integral, &center_prev_value);
				left_side_power = pid_process (echo_right_time_us_trunc, SENSOR_VALUE_AT_DIR_CHANGE, kps, kis, kds, SAMPLE_TIME_S, &center_integral, &center_prev_value);
				right_side_power = pid_process (echo_left_time_us_trunc, SENSOR_VALUE_AT_DIR_CHANGE, kps, kis, kds, SAMPLE_TIME_S, &center_integral, &center_prev_value);
				diff_power = pid_process ( (echo_left_time_us_trunc - echo_right_time_us_trunc), 0, kps, kis, kds, SAMPLE_TIME_S, &center_integral, &center_prev_value);
			}

			// get absolute value of power and put it into "weight" variable
			if (straight_power >= 0) straight_weight= straight_power;
			else straight_weight= -1 * straight_power;
			if (left_side_power >= 0) left_side_weight= left_side_power;
			else left_side_weight = -1 * left_side_power;
			if (right_side_power >= 0) right_side_weight= right_side_power;
			else right_side_weight= -1 * right_side_power;
			if (diff_power >= 0) diff_weight= diff_power;
			else diff_weight= -1 * diff_power;

			// straight and curve percentage combine. The more value, the more weight
			left_motor_power_sum = straight_weight + left_side_weight - diff_weight;
			right_motor_power_sum = straight_weight + right_side_weight + diff_weight;

			// get motor power
			left_motor_duty_float = straight_weight / left_motor_power_sum * straight_power + left_side_weight / left_motor_power_sum * left_side_power + diff_weight / left_motor_power_sum * diff_power;
			left_motor_duty_float = straight_weight / right_motor_power_sum * straight_power + right_side_weight / right_motor_power_sum * right_side_power + diff_weight / right_motor_power_sum * diff_power;

			// motor value normalize to percentage
			if (left_motor_duty_float < 100 &&  left_motor_duty_float > -100) left_motor_duty_float = 0;
			else if (left_motor_duty_float > 0) left_motor_duty_float = left_motor_duty_float / SENSOR_VALUE_AT_MAX_SPEED / 100 * (100 - 43) + 43;
			else if (left_motor_duty_float < 0) left_motor_duty_float = left_motor_duty_float / SENSOR_VALUE_AT_MAX_SPEED / 100 * (100 - 43) - 43;
			if (right_motor_duty_float < 100 &&  right_motor_duty_float > -100) right_motor_duty_float = 0;
			else if (right_motor_duty_float > 0) right_motor_duty_float = right_motor_duty_float / SENSOR_VALUE_AT_MAX_SPEED / 100 * (100 - 43) + 43;
			else if (right_motor_duty_float < 0) right_motor_duty_float = right_motor_duty_float / SENSOR_VALUE_AT_MAX_SPEED / 100 * (100 - 43) - 43;

			// convert to integer value
			left_motor_duty_int = (int) left_motor_duty_float;
			right_motor_duty_int = (int) right_motor_duty_float;

			// motor output compensation
			if (left_motor_duty_int > 0) left_motor_duty_int -= 3;
			else if (left_motor_duty_int < 0) left_motor_duty_int += 3;

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
	if (echo_right_echo_left_time_us_limited_temp < SENSOR_VALUE_THRESHOLD) echo_right_time_queue[echo_right_time_queue_index++] = echo_right_time_us_temp;

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

