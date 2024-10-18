/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  * Pinout
  *
  *				C10		C11										C9		C8
  *				C12		D2										B8		C6
  *				VDD		E5V		BAT_POWER						B9		C5
  *				BOOT0	GND		BAT_GND							AVDD	U5V
  *				NC		NC										GND		NC
  *				NC		IOREF									A5		A12
  *				A13		RESET					ECHO_LEFT		A6		A11
  *				A14		3V3										A7		B12
  *	USART1 TX	A15		5V		BT_POWER		ECHO_RIGHT		B6		NC
  *				GND		GND		BT_GND							C7		GND
  *				B7		GND						TRIG_CENTER		A9		B2
  *				C13		VIN						TRIG_LEFT		A8		B1		IN1
  *				C14		NC										B10		B15		IN2
  *				C15		A0		MOTOR_A							B4		B14		IN3
  *				H0		A1		MOTOR_B							B5		B13		IN4
  *				H1		A4						USART1 RX		B3		AGND
  *				VBAT	B0		ECHO_CENTER		TRIG_RIGHT		A10		C4
  *				C2		C1										A2		NC
  *				C3		C0										A3		NC
  ******************************************************************************
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
char rxBuffer[20];
int32_t echo_left_time_us, echo_center_time_us, echo_right_time_us;
uint8_t mode_auto_manu = 0, mode_deadlock_normal = 0, mode_monitor_on_off = 0;
uint16_t echo_left_time_queue[VALUE_QUEUE_SIZE] = {0, }, echo_center_time_queue[VALUE_QUEUE_SIZE] = {0, }, echo_right_time_queue[VALUE_QUEUE_SIZE] = {0, };
uint8_t echo_left_time_queue_index = 0, echo_center_time_queue_index = 0, echo_right_time_queue_index = 0;
uint32_t echo_left_rise_time, echo_left_fall_time;
uint32_t echo_center_rise_time, echo_center_fall_time;
uint32_t echo_right_rise_time, echo_right_fall_time;
float left_motor_duty_float, right_motor_duty_float;
int left_motor_duty_int, right_motor_duty_int;
int left_motor_duty_int_raw, right_motor_duty_int_raw;
uint8_t arbitrary_turn_right_left = 0;
uint32_t deadlock_threshold = 800;
float kps = 1, kis = 0, kds = 0, kpc = 1, kic = 0, kdc = 0;
float center_integral, curve_integral_left, curve_integral_right;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart_bluetooth, (uint8_t *) &ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  pwmMotor_init(&htim_pwmMotor, CHANNEL_MOTOR_A);
  pwmMotor_init(&htim_pwmMotor, CHANNEL_MOTOR_B);

  HAL_TIM_PWM_Start(&htim_triggerPulse, CHANNEL_TRIG_LEFT);
  HAL_TIM_PWM_Start(&htim_triggerPulse, CHANNEL_TRIG_CENTER);
  HAL_TIM_PWM_Start(&htim_triggerPulse, CHANNEL_TRIG_RIGHT);

  HAL_TIM_IC_Start_DMA(&htim_echoMeasure_1, CHANNEL_ECHO_LEFT_RISING, &echo_left_rise_time, 1);
  HAL_TIM_IC_Start_DMA(&htim_echoMeasure_1, CHANNEL_ECHO_LEFT_FALLING, &echo_left_fall_time, 1);
  HAL_TIM_IC_Start_DMA(&htim_echoMeasure_1, CHANNEL_ECHO_CENTER_RISING, &echo_center_rise_time, 1);
  HAL_TIM_IC_Start_DMA(&htim_echoMeasure_1, CHANNEL_ECHO_CENTER_FALLING, &echo_center_fall_time, 1);
  HAL_TIM_IC_Start_DMA(&htim_echoMeasure_2, CHANNEL_ECHO_RIGHT_RISING, &echo_right_rise_time, 1);
  HAL_TIM_IC_Start_DMA(&htim_echoMeasure_2, CHANNEL_ECHO_RIGHT_FALLING, &echo_right_fall_time, 1);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart_bluetooth, (uint8_t*) rxBuffer, 20);

  printf("\nProgram start\nFor available commands, type \"help\".\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
