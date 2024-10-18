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
float pid_process (float current_value, float target_value, float kp, float ki, float kd, float dt, float* integral_arr_ptr, uint8_t* integral_index_ptr, float* prev_value_ptr)
{
	float error = current_value - target_value;
	float p_term = kp * error;
	integral_arr_ptr[integral_index_ptr] = error * dt;
	if (*integral_index_ptr > INTEGRAL_ARR_NUM) *integral_index_ptr = 0;
	float integral_sum = 0;
	for (uint8_t i=0; i<INTEGRAL_ARR_NUM; ++i)
	{
		integral_sum += integral_arr_ptr[i];
	}
	float i_term = ki * integral_sum;
	float d_term = kd * (current_value - *prev_value_ptr) / dt;
	*prev_value_ptr = current_value;

	return p_term + i_term - d_term;
}

// return absolute value of float
float float_abs(float input_float)
{
	if (input_float > 0) return input_float;
	else return (-1 * input_float);
}

#define SAMPLE_TIME_S 0.01

float prev_value_straight = 0, prev_value_left = 0, prev_value_right = 0;
/* USER CODE END Header_StartTask_auto_drive */
void StartTask_auto_drive(void *argument)
{
	/* USER CODE BEGIN StartTask_auto_drive */

	osDelay(1000);
	deadlock_threshold = deadlock_threshold_value;
	/* Infinite loop */
	for(;;)
	{
		//////////////////////////
		// VARIABLE DECLARATION //
		//////////////////////////

		// Sensor value truncated
		float echo_left_time_us_trunc, echo_center_time_us_trunc, echo_right_time_us_trunc;

		float straight_power;	// Straight direction motor power
		float left_power, right_power;	// One-side motor power
		float opposite_left_power, opposite_right_power;	// The other-side motor power

		float left_weight_sum, right_weight_sum;	// sum for weight calculation

		if (mode_auto_manu || mode_monitor_on_off)
		{
			///////////////////////////
			// SENSOR VALUE TRUNCATE //
			///////////////////////////

			if (echo_left_time_us > sensor_value_max_speed) echo_left_time_us_trunc = sensor_value_max_speed;
			else echo_left_time_us_trunc = echo_left_time_us;
			if (echo_center_time_us > sensor_value_max_speed) echo_center_time_us_trunc = sensor_value_max_speed;
			else echo_center_time_us_trunc = echo_center_time_us;
			if (echo_right_time_us > sensor_value_max_speed) echo_right_time_us_trunc = sensor_value_max_speed;
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
					deadlock_threshold = deadlock_threshold_value * 2;
				}
			}
			// reject sensor value if it is touching wall
			else if (echo_left_time_us_trunc > deadlock_threshold && echo_center_time_us_trunc > deadlock_threshold && echo_right_time_us_trunc > deadlock_threshold)
			{
				if (mode_deadlock_normal)
				{
					mode_deadlock_normal = 0;
					if (mode_monitor_on_off) printf("Deadlock mode off\n");
					deadlock_threshold = deadlock_threshold_value;
				}
			}

			////////////////
			// AUTO-DRIVE //
			////////////////

			// Hard-coded value if deadlocked
			if (mode_deadlock_normal)
			{
				straight_power = -100;
				if (arbitrary_turn_right_left)
				{
					left_power = 100;
					right_power = -100;

				}
				else
				{
					left_power = -100;
					right_power = 100;
				}
			}

			// Normal operation
			else
			{
				straight_power = pid_process (echo_center_time_us_trunc, sensor_value_dir_change, kps, kis, kds, SAMPLE_TIME_S, integral_straight_arr, &integral_straight_index, &prev_value_straight);
				left_power = pid_process (echo_right_time_us_trunc, sensor_value_dir_change, kpl, kil, kdl, SAMPLE_TIME_S, integral_left_arr, &integral_left_index, &prev_value_left);
				right_power = pid_process (echo_left_time_us_trunc, sensor_value_dir_change, kpr, kir, kdr, SAMPLE_TIME_S, integral_right_arr, &integral_right_index, &prev_value_right);
			}

			opposite_left_power = left_power * opposite_constant;
			opposite_right_power = right_power * opposite_constant;

			// sum of weigth calculation
			left_weight_sum = float_abs(straight_power) + float_abs(left_power) + float_abs(opposite_right_power);
			right_weight_sum = float_abs(straight_power) + float_abs(right_power) + float_abs(opposite_left_power);

			// Get motor power. The more value, the more weight
			left_motor_duty_float = float_abs(straight_power) / left_weight_sum * straight_power + float_abs(left_power) / left_weight_sum * left_power - float_abs(opposite_right_power) / left_weight_sum * opposite_right_power;
			right_motor_duty_float = float_abs(straight_power) / right_weight_sum * straight_power + float_abs(right_power) / right_weight_sum * right_power - float_abs(opposite_left_power) / right_weight_sum * opposite_left_power;

			float sensor_max_value = sensor_value_max_speed - sensor_value_dir_change;

			// motor value normalize to percentage
			if (left_motor_duty_float < 1 &&  left_motor_duty_float > -1) left_motor_duty_float = 0;
			else if (left_motor_duty_float > 0) left_motor_duty_float = left_motor_duty_float / sensor_max_value * (100 - 43) + 43;
			else if (left_motor_duty_float < 0) left_motor_duty_float = left_motor_duty_float / sensor_max_value * (100 - 43) - 43;
			if (right_motor_duty_float < 1 &&  right_motor_duty_float > -1) right_motor_duty_float = 0;
			else if (right_motor_duty_float > 0) right_motor_duty_float = right_motor_duty_float / sensor_max_value * (100 - 43) + 43;
			else if (right_motor_duty_float < 0) right_motor_duty_float = right_motor_duty_float / sensor_max_value * (100 - 43) - 43;

			// convert to integer value
			left_motor_duty_int = (int) left_motor_duty_float;
			right_motor_duty_int = (int) right_motor_duty_float;

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

