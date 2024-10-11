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
#include "delay.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint8_t rxChar;
extern uint32_t echo_time_us[3];
extern uint8_t mode_auto_manu;
extern uint16_t echo_time_queue[3][10];
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define htim_pwmMotor htim3
#define huart_bluetooth huart1
#define htim_echoMeasure htim4
#define TRIG_LEFT_Pin GPIO_PIN_0
#define TRIG_LEFT_GPIO_Port GPIOC
#define TRIG_CENTER_Pin GPIO_PIN_1
#define TRIG_CENTER_GPIO_Port GPIOC
#define out_pwmMotor_A_Pin GPIO_PIN_6
#define out_pwmMotor_A_GPIO_Port GPIOA
#define out_pwmMotor_B_Pin GPIO_PIN_7
#define out_pwmMotor_B_GPIO_Port GPIOA
#define TRIG_RIGHT_Pin GPIO_PIN_0
#define TRIG_RIGHT_GPIO_Port GPIOB
#define GPIO_motorDriver_IN2_Pin GPIO_PIN_10
#define GPIO_motorDriver_IN2_GPIO_Port GPIOB
#define GPIO_motorDriver_IN1_Pin GPIO_PIN_8
#define GPIO_motorDriver_IN1_GPIO_Port GPIOA
#define GPIO_motorDriver_IN3_Pin GPIO_PIN_4
#define GPIO_motorDriver_IN3_GPIO_Port GPIOB
#define GPIO_motorDriver_IN4_Pin GPIO_PIN_5
#define GPIO_motorDriver_IN4_GPIO_Port GPIOB
#define ECHO_LEFT_Pin GPIO_PIN_6
#define ECHO_LEFT_GPIO_Port GPIOB
#define ECHO_CENTER_Pin GPIO_PIN_7
#define ECHO_CENTER_GPIO_Port GPIOB
#define ECHO_RIGHT_Pin GPIO_PIN_8
#define ECHO_RIGHT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define CHANNEL_MOTOR_A TIM_CHANNEL_1
#define CHANNEL_MOTOR_B TIM_CHANNEL_2
#define CHANNEL_ECHO_LEFT TIM_CHANNEL_1
#define CHANNEL_ECHO_CENTER TIM_CHANNEL_2
#define CHANNEL_ECHO_RIGHT TIM_CHANNEL_3
#define ACTIVE_CHANNEL_ECHO_LEFT HAL_TIM_ACTIVE_CHANNEL_1
#define ACTIVE_CHANNEL_ECHO_CENTER HAL_TIM_ACTIVE_CHANNEL_2
#define ACTIVE_CHANNEL_ECHO_RIGHT HAL_TIM_ACTIVE_CHANNEL_3
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
