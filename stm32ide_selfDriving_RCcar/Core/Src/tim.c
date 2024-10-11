/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#define ULTRASONIC_LEFT_BIT 0b1
#define ULTRASONIC_CENTER_BIT 0b10
#define ULTRASONIC_RIGHT_BIT 0b100
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	static uint8_t capture_flag = 0;
	static uint32_t echo_high_time_us[3], echo_low_time_us[3];
	if (htim->Channel == ACTIVE_CHANNEL_ECHO_LEFT)
	{
		if ( !(capture_flag & ULTRASONIC_LEFT_BIT) )
		{
			echo_high_time_us[0] = HAL_TIM_ReadCapturedValue(&htim_echoMeasure, CHANNEL_ECHO_LEFT);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_echoMeasure, CHANNEL_ECHO_LEFT, TIM_INPUTCHANNELPOLARITY_FALLING);
			capture_flag |= ULTRASONIC_LEFT_BIT;
		}
		else
		{
			static uint8_t echo_time_left_index = 0;
			echo_low_time_us[0] = HAL_TIM_ReadCapturedValue(&htim_echoMeasure, CHANNEL_ECHO_LEFT);
			uint16_t echo_time_tmp;
			if (echo_low_time_us[0] > echo_high_time_us[0])
			{
				echo_time_tmp = echo_low_time_us[0] - echo_high_time_us[0];
			}
			else /* overflow occurred in between */
			{
				echo_time_tmp = (0xffff - echo_high_time_us[0]) + echo_low_time_us[0];
			}
			if (echo_time_tmp < 2000) echo_time_queue[0][echo_time_left_index++] = echo_time_tmp;
			if (echo_time_left_index >= 10) echo_time_left_index = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_echoMeasure, CHANNEL_ECHO_LEFT, TIM_INPUTCHANNELPOLARITY_RISING);
			capture_flag &= ~ULTRASONIC_LEFT_BIT;
			__HAL_TIM_DISABLE_IT(&htim_echoMeasure, TIM_IT_CC1);
		}
	}
	else if (htim->Channel == ACTIVE_CHANNEL_ECHO_CENTER)
	{
		if ( !(capture_flag & ULTRASONIC_CENTER_BIT) )
		{
			echo_high_time_us[1] = HAL_TIM_ReadCapturedValue(&htim_echoMeasure, CHANNEL_ECHO_CENTER);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_echoMeasure, CHANNEL_ECHO_CENTER, TIM_INPUTCHANNELPOLARITY_FALLING);
			capture_flag |= ULTRASONIC_CENTER_BIT;
		}
		else
		{

			static uint8_t echo_time_center_index = 0;
			echo_low_time_us[1] = HAL_TIM_ReadCapturedValue(&htim_echoMeasure, CHANNEL_ECHO_CENTER);
			uint16_t echo_time_tmp;
			if (echo_low_time_us[1] > echo_high_time_us[1])
			{
				echo_time_tmp = echo_low_time_us[1] - echo_high_time_us[1];
			}
			else /* overflow occurred in between */
			{
				echo_time_tmp = (0xffff - echo_high_time_us[1]) + echo_low_time_us[1];
			}
			if (echo_time_tmp < 2000) echo_time_queue[1][echo_time_center_index++] = echo_time_tmp;
			if (echo_time_center_index >= 10) echo_time_center_index = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_echoMeasure, CHANNEL_ECHO_CENTER, TIM_INPUTCHANNELPOLARITY_RISING);
			capture_flag &= ~ULTRASONIC_CENTER_BIT;
			__HAL_TIM_DISABLE_IT(&htim_echoMeasure, TIM_IT_CC2);
		}
	}
	else if (htim->Channel == ACTIVE_CHANNEL_ECHO_RIGHT)
	{
		if ( !(capture_flag & ULTRASONIC_RIGHT_BIT) )
		{
			echo_high_time_us[2] = HAL_TIM_ReadCapturedValue(&htim_echoMeasure, CHANNEL_ECHO_RIGHT);
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_echoMeasure, CHANNEL_ECHO_RIGHT, TIM_INPUTCHANNELPOLARITY_FALLING);
			capture_flag |= ULTRASONIC_RIGHT_BIT;
		}
		else
		{
			static uint8_t echo_time_right_index = 0;
			echo_low_time_us[2] = HAL_TIM_ReadCapturedValue(&htim_echoMeasure, CHANNEL_ECHO_RIGHT);
			uint16_t echo_time_tmp;
			if (echo_low_time_us[2] > echo_high_time_us[2])
			{
				echo_time_tmp = echo_low_time_us[2] - echo_high_time_us[2];
			}
			else /* overflow occurred in between */
			{
				echo_time_tmp = (0xffff - echo_high_time_us[2]) + echo_low_time_us[2];
			}
			if (echo_time_tmp < 2000) echo_time_queue[2][echo_time_right_index++] = echo_time_tmp;
			if (echo_time_right_index >= 10) echo_time_right_index = 0;
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim_echoMeasure, CHANNEL_ECHO_RIGHT, TIM_INPUTCHANNELPOLARITY_RISING);
			capture_flag &= ~ULTRASONIC_RIGHT_BIT;
			__HAL_TIM_DISABLE_IT(&htim_echoMeasure, TIM_IT_CC3);
		}
	}
}
/* USER CODE END 0 */

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}
/* TIM4 init function */
void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 10;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}
/* TIM10 init function */
void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();
  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspInit 0 */

  /* USER CODE END TIM10_MspInit 0 */
    /* TIM10 clock enable */
    __HAL_RCC_TIM10_CLK_ENABLE();
  /* USER CODE BEGIN TIM10_MspInit 1 */

  /* USER CODE END TIM10_MspInit 1 */
  }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(tim_icHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* TIM4 clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8     ------> TIM4_CH3
    */
    GPIO_InitStruct.Pin = ECHO_LEFT_Pin|ECHO_CENTER_Pin|ECHO_RIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspPostInit 0 */

  /* USER CODE END TIM3_MspPostInit 0 */

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM3 GPIO Configuration
    PA6     ------> TIM3_CH1
    PA7     ------> TIM3_CH2
    */
    GPIO_InitStruct.Pin = out_pwmMotor_A_Pin|out_pwmMotor_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspPostInit 1 */

  /* USER CODE END TIM3_MspPostInit 1 */
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();
  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM10)
  {
  /* USER CODE BEGIN TIM10_MspDeInit 0 */

  /* USER CODE END TIM10_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM10_CLK_DISABLE();
  /* USER CODE BEGIN TIM10_MspDeInit 1 */

  /* USER CODE END TIM10_MspDeInit 1 */
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

  if(tim_icHandle->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM4_CLK_DISABLE();

    /**TIM4 GPIO Configuration
    PB6     ------> TIM4_CH1
    PB7     ------> TIM4_CH2
    PB8     ------> TIM4_CH3
    */
    HAL_GPIO_DeInit(GPIOB, ECHO_LEFT_Pin|ECHO_CENTER_Pin|ECHO_RIGHT_Pin);

    /* TIM4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
