/*
 * pwmMotor.c
 *
 *  Created on: Jul 31, 2024
 *      Author: Harman 4th
 */


#include "pwmMotor.h"

static uint16_t max_duty;

void pwmMotor_init(TIM_HandleTypeDef *htim, uint32_t motor_channel)
{
	HAL_TIM_PWM_Start(htim, motor_channel);
	max_duty = htim->Instance->ARR - 1;
}
void pwmMotor_setDuty(TIM_HandleTypeDef *htim, uint32_t motor_channel, uint16_t duty)
{
	__HAL_TIM_SET_COMPARE(htim, motor_channel, duty);
}

void pwmMotor_deinit(TIM_HandleTypeDef *htim, uint32_t motor_channel)
{
	HAL_TIM_PWM_Stop(htim, motor_channel);
}

void pwmMotor_directionShift (uint32_t motor_channel, pwmMotor_direction dir)
{
	GPIO_PinState GPIO_PinState_IN_1st;
	GPIO_PinState GPIO_PinState_IN_2nd;
	switch (dir)
	{
	case NEUTRAL:
		GPIO_PinState_IN_1st = GPIO_PIN_RESET;
		GPIO_PinState_IN_2nd = GPIO_PIN_RESET;
		break;
	case FORWARD:
		GPIO_PinState_IN_1st = GPIO_PIN_SET;
		GPIO_PinState_IN_2nd = GPIO_PIN_RESET;
		break;
	case BACKWARD:
		GPIO_PinState_IN_1st = GPIO_PIN_RESET;
		GPIO_PinState_IN_2nd = GPIO_PIN_SET;
		break;
	case BREAK:
		GPIO_PinState_IN_1st = GPIO_PIN_SET;
		GPIO_PinState_IN_2nd = GPIO_PIN_SET;
		break;
	}

	if (motor_channel == CHANNEL_MOTOR_A)
	{
		HAL_GPIO_WritePin(GPIO_motorDriver_IN1_GPIO_Port, GPIO_motorDriver_IN1_Pin, GPIO_PinState_IN_1st);
		HAL_GPIO_WritePin(GPIO_motorDriver_IN2_GPIO_Port, GPIO_motorDriver_IN2_Pin, GPIO_PinState_IN_2nd);
	}
	else if (motor_channel == CHANNEL_MOTOR_B)
	{
		HAL_GPIO_WritePin(GPIO_motorDriver_IN3_GPIO_Port, GPIO_motorDriver_IN3_Pin, GPIO_PinState_IN_1st);
		HAL_GPIO_WritePin(GPIO_motorDriver_IN4_GPIO_Port, GPIO_motorDriver_IN4_Pin, GPIO_PinState_IN_2nd);
	}
}

void RCcar_set_motor_speed (int8_t left_speed_percent, int8_t right_speed_percent)
{
	uint16_t left_motor_duty, right_motor_duty;

	if (left_speed_percent > 0)
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_A, FORWARD);
		left_motor_duty = left_speed_percent * max_duty / 100;
	}
	else if (left_speed_percent < 0)
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_A, BACKWARD);
		left_motor_duty = -1.0 * left_speed_percent * max_duty / 100;
	}
	else /* (left_speed_percent == 0) */
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_A, BREAK);
		left_motor_duty = 0;
	}


	if (right_speed_percent > 0)
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_B, FORWARD);
		right_motor_duty = right_speed_percent * max_duty / 100;
	}
	else if (right_speed_percent < 0)
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_B, BACKWARD);
		right_motor_duty = -1.0 * right_speed_percent * max_duty / 100;
	}
	else /* (right_speed_percent == 0) */
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_B, BREAK);
		right_motor_duty = 0;
	}

	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, left_motor_duty);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, right_motor_duty);
}

void RCcar_go_forward(uint8_t speed_percent)
{
	uint16_t duty = speed_percent * max_duty / 100;
	pwmMotor_directionShift (CHANNEL_MOTOR_A, FORWARD);
	pwmMotor_directionShift (CHANNEL_MOTOR_B, FORWARD);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, duty);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, duty);

}

void RCcar_go_soft_left(uint8_t speed_percent)
{
	uint16_t duty = speed_percent * max_duty / 100;
	pwmMotor_directionShift (CHANNEL_MOTOR_A, FORWARD);
	pwmMotor_directionShift (CHANNEL_MOTOR_B, FORWARD);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, duty / 8);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, duty);
}

void RCcar_go_soft_right(uint8_t speed_percent)
{
	uint16_t duty = speed_percent * max_duty / 100;
	pwmMotor_directionShift (CHANNEL_MOTOR_A, FORWARD);
	pwmMotor_directionShift (CHANNEL_MOTOR_B, FORWARD);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, duty);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, duty / 8);
}

void RCcar_go_backward(uint8_t speed_percent)
{
	uint16_t duty = speed_percent * max_duty / 100;
	pwmMotor_directionShift (CHANNEL_MOTOR_A, BACKWARD);
	pwmMotor_directionShift (CHANNEL_MOTOR_B, BACKWARD);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, duty);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, duty);
}

void RCcar_stop()
{
	pwmMotor_directionShift (CHANNEL_MOTOR_A, BREAK);
	pwmMotor_directionShift (CHANNEL_MOTOR_B, BREAK);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, 0);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, 0);
}

void RCcar_analogStick(uint8_t x, uint8_t y)
{
	uint16_t acceleration;	// 0 to 128 value

	// X-axis
	if (123 < y && y < 133)	// dead zone
	{
		pwmMotor_directionShift (CHANNEL_MOTOR_A, BREAK);
		pwmMotor_directionShift (CHANNEL_MOTOR_B, BREAK);
		acceleration = 0;
	}
	else if (133 <= y)
	{
		acceleration = (y - 128);
		pwmMotor_directionShift (CHANNEL_MOTOR_A, FORWARD);
		pwmMotor_directionShift (CHANNEL_MOTOR_B, FORWARD);
	}
	else /* (y <= 123) */
	{
		acceleration = (128 - y);
		pwmMotor_directionShift (CHANNEL_MOTOR_A, BACKWARD);
		pwmMotor_directionShift (CHANNEL_MOTOR_B, BACKWARD);
	}

	uint16_t motor_A_duty, motor_B_duty;
	// Y-axis
	if (123 < x && x < 133)
	{
		motor_A_duty = (uint16_t) ( (float) acceleration * 7.8125 );
		motor_B_duty = (uint16_t) ( (float) acceleration * 7.8125 );
	}
	else if (133 <= x)	// right turn, motor B deceleration
	{
		motor_A_duty = (uint16_t) ( (float) acceleration * 7.8125 );
		motor_B_duty = (uint16_t) ( ((float) acceleration * ( 256.0 - (float) x ) ) * 0.06103515625 );
	}
	else /* (x <= 123) */
	{
		motor_A_duty = (uint16_t) ( ((float) acceleration * ( (float) x ) ) * 0.06103515625 );
		motor_B_duty = (uint16_t) ( (float) acceleration * 7.8125 );
	}

	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_A, motor_A_duty);
	pwmMotor_setDuty(&htim_pwmMotor, CHANNEL_MOTOR_B, motor_B_duty);
}
