/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "pwmMotor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define huart_bluetooth huart1
#define htim_echoMeasure_1 htim3
#define htim_echoMeasure_2 htim4
#define htim_triggerPulse htim1
#define htim_pwmMotor htim2
#define out_pwmMotor_A_Pin GPIO_PIN_0
#define out_pwmMotor_A_GPIO_Port GPIOA
#define out_pwmMotor_B_Pin GPIO_PIN_1
#define out_pwmMotor_B_GPIO_Port GPIOA
#define ECHO_LEFT_Pin GPIO_PIN_6
#define ECHO_LEFT_GPIO_Port GPIOA
#define ECHO_CENTER_Pin GPIO_PIN_0
#define ECHO_CENTER_GPIO_Port GPIOB
#define GPIO_motorDriver_IN1_Pin GPIO_PIN_1
#define GPIO_motorDriver_IN1_GPIO_Port GPIOB
#define GPIO_motorDriver_IN4_Pin GPIO_PIN_13
#define GPIO_motorDriver_IN4_GPIO_Port GPIOB
#define GPIO_motorDriver_IN3_Pin GPIO_PIN_14
#define GPIO_motorDriver_IN3_GPIO_Port GPIOB
#define GPIO_motorDriver_IN2_Pin GPIO_PIN_15
#define GPIO_motorDriver_IN2_GPIO_Port GPIOB
#define TRIG_LEFT_Pin GPIO_PIN_8
#define TRIG_LEFT_GPIO_Port GPIOA
#define TRIG_CENTER_Pin GPIO_PIN_9
#define TRIG_CENTER_GPIO_Port GPIOA
#define TRIG_RIGHT_Pin GPIO_PIN_10
#define TRIG_RIGHT_GPIO_Port GPIOA
#define ECHO_RIGHT_Pin GPIO_PIN_6
#define ECHO_RIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CHANNEL_MOTOR_A TIM_CHANNEL_1
#define CHANNEL_MOTOR_B TIM_CHANNEL_2

#define CHANNEL_ECHO_LEFT_RISING TIM_CHANNEL_1
#define CHANNEL_ECHO_LEFT_FALLING TIM_CHANNEL_2
#define CHANNEL_ECHO_CENTER_RISING TIM_CHANNEL_3
#define CHANNEL_ECHO_CENTER_FALLING TIM_CHANNEL_4
#define CHANNEL_ECHO_RIGHT_RISING TIM_CHANNEL_1
#define CHANNEL_ECHO_RIGHT_FALLING TIM_CHANNEL_2

#define CHANNEL_TRIG_LEFT TIM_CHANNEL_1
#define CHANNEL_TRIG_CENTER TIM_CHANNEL_2
#define CHANNEL_TRIG_RIGHT TIM_CHANNEL_3

#define ACTIVE_CHANNEL_ECHO_LEFT HAL_TIM_ACTIVE_CHANNEL_1
#define ACTIVE_CHANNEL_ECHO_CENTER HAL_TIM_ACTIVE_CHANNEL_2
#define ACTIVE_CHANNEL_ECHO_RIGHT HAL_TIM_ACTIVE_CHANNEL_3

#define VALUE_QUEUE_SIZE 5

// Autonomous Drive Constants
#define SENSOR_VALUE_THRESHOLD 30000

extern char rxBuffer[20];
extern int32_t echo_left_time_us, echo_center_time_us, echo_right_time_us;
extern uint8_t mode_auto_manu, mode_deadlock_normal, mode_monitor_on_off;
extern uint32_t echo_left_rise_time, echo_left_fall_time;
extern uint32_t echo_center_rise_time, echo_center_fall_time;
extern uint32_t echo_right_rise_time, echo_right_fall_time;
extern float left_motor_duty_float, right_motor_duty_float;
extern int left_motor_duty_int, right_motor_duty_int;
extern int left_motor_duty_int_raw, right_motor_duty_int_raw;
extern uint8_t arbitrary_turn_right_left;
extern uint32_t deadlock_threshold;
extern uint16_t echo_left_time_queue[VALUE_QUEUE_SIZE], echo_center_time_queue[VALUE_QUEUE_SIZE], echo_right_time_queue[VALUE_QUEUE_SIZE];
extern uint8_t echo_left_time_queue_index, echo_center_time_queue_index, echo_right_time_queue_index;
extern float kps, kis, kds, kpc, kic, kdc;
extern float center_integral, curve_integral_left, curve_integral_right;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
