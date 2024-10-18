/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
extern void RCcar_analogStick(uint8_t x, uint8_t y);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// RxEventCallback is called for two times per Rx: receive complete and Half complete.
	// Below code reject half complete callback.
	if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
	{
		// Android arduino bluetooth controller app
		if (huart == &huart_bluetooth)
		{

			if (Size == 1)
			{
				switch (rxBuffer[0])
				{
				case 'F':
					if (!mode_auto_manu) RCcar_go_forward(100);
					break;
				case 'B':
					if (!mode_auto_manu) RCcar_go_backward(100);
					break;
				case 'L':
					if (!mode_auto_manu) RCcar_go_soft_left(100);
					break;
				case 'R':
					if (!mode_auto_manu) RCcar_go_soft_right(100);
					break;
				case 'A':

					break;
				case '0':
					if (!mode_auto_manu) RCcar_stop();
					break;
				}
			}
			else /* sentence command */
			{
				printf("REMOTE> ");
				char *token = strtok(rxBuffer, " \n\r");
				if (!strcmp(token, "mode"))
				{
					mode_auto_manu = !mode_auto_manu;
					if (mode_auto_manu) printf("Autonomous Mode");
					else printf("Manual Mode");
					if (!mode_auto_manu) RCcar_stop();
					integral_straight = 0;
					integral_left = 0;
					integral_right = 0;
				}
				else if (!strcmp(token, "set"))
				{
					token = strtok(NULL, " \n\r");
					float num = atof(strtok(NULL, " \n\r"));
					if (!strcmp(token, "kps"))
					{
						kps = num;
						printf("kps set to %.5f", num);
					}
					else if (!strcmp(token, "kis"))
					{
						kis = num;
						printf("kis set to %.5f", num);
					}
					else if (!strcmp(token, "kds"))
					{
						kds = num;
						printf("kds set to %.5f", num);
					}
					else if (!strcmp(token, "kpl"))
					{
						kpl = num;
						printf("kpl set to %.5f", num);
					}
					else if (!strcmp(token, "kil"))
					{
						kil = num;
						printf("kil set to %.5f", num);
					}
					else if (!strcmp(token, "kdl"))
					{
						kdl = num;
						printf("kdl set to %.5f", num);
					}
					else if (!strcmp(token, "kpr"))
					{
						kpr = num;
						printf("kpr set to %.5f", num);
					}
					else if (!strcmp(token, "kir"))
					{
						kir = num;
						printf("kir set to %.5f", num);
					}
					else if (!strcmp(token, "kdr"))
					{
						kdr = num;
						printf("kdr set to %.5f", num);
					}
					else if (!strcmp(token, "opp"))
					{
						opposite_constant = num;
						printf("opposite constant set to %.5f", num);
					}
					else if (!strcmp(token, "max"))
					{
						sensor_value_max_speed = num;
						printf("max sensor value set to %.5f", num);
					}
					else if (!strcmp(token, "dir"))
					{
						sensor_value_dir_change = num;
						printf("direction change sensor value set to %.5f", num);
					}
					else if (!strcmp(token, "dl"))
					{
						deadlock_threshold_value = num;
						if (!mode_deadlock_normal) deadlock_threshold = deadlock_threshold_value * 2;
						else deadlock_threshold = deadlock_threshold_value;
						printf("deadlock distance set to %.5f", num);
					}
					else
					{
						printf("Command Error");
					}
				}
				else if (!strcmp(token, "show"))
				{
					printf("\nkps %.5f kis %.5f kds %.5f", kps, kis, kds);
					printf("\nkpl %.5f kil %.5f kdl %.5f", kpl, kil, kdl);
					printf("\nkpr %.5f kir %.5f kdr %.5f", kpr, kir, kdr);
					printf("\nopposite constant: %.5f", opposite_constant);
					printf("\nmax sensor value: %.5f", sensor_value_max_speed);
					printf("\ndir change value: %.5f", sensor_value_dir_change);
					printf("\ndeadlock value: %.5f", deadlock_threshold_value);
				}
				else if (!strcmp(token, "monitor"))
				{
					mode_monitor_on_off = !mode_monitor_on_off;
					if (mode_monitor_on_off) printf("monitor on");
					else printf("monitor off");
				}
				else if (!strcmp(token, "help"))
				{
					printf("\nmode: turn on/off autonomous mode");
					printf("\nshow: show constants");
					printf("\nmonitor: monitor sensor values & motor powers");
					printf("\nset k[p|i|d][s|l|r] <float>: set pid constant");
					printf("\nset opp <float: 0..1>: set opposite constant");
					printf("\nset max <float>: set maximum sensor value allowed");
					printf("\nset dir <float>: set direction changing sensor value");
					printf("\nset dl <float>: set deadlock distance");
					printf("\nhelp: display this message");
				}
				else
				{
					printf("Command Error");
				}
				printf("\n");
			}

		}
		for (uint8_t i=0; i<Size; ++i)
		{
			rxBuffer[i] = 0;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t*) rxBuffer, 20);
	}
}
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA15     ------> USART1_TX
    PB3     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA15     ------> USART1_TX
    PB3     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
