/*
 * pwmMotor.h
 *
 *  Created on: Jul 31, 2024
 *      Author: Harman 4th
 */

#ifndef PWMMOTOR_H_
#define PWMMOTOR_H_

#include "tim.h"
#include "gpio.h"

typedef enum
{
	NEUTRAL = 0,
	FORWARD,
	BACKWARD,
	BREAK
} pwmMotor_direction;

void pwmMotor_init(TIM_HandleTypeDef *htim, uint32_t Channel);
void pwmMotor_setDuty(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t duty);
void pwmMotor_deinit(TIM_HandleTypeDef *htim, uint32_t Channel);
void pwmMotor_directionShift (uint32_t motor_channel, pwmMotor_direction dir);

void RCcar_set_motor_speed (int8_t left_speed_percent, int8_t right_speed_percent);
void RCcar_go_forward(uint8_t speed_percent);
void RCcar_go_soft_left(uint8_t speed_percent);
void RCcar_go_soft_right(uint8_t speed_percent);
void RCcar_go_backward(uint8_t speed_percent);
void RCcar_stop();
void RCcar_analogStick(uint8_t x, uint8_t y);

#endif /* PWMMOTOR_H_ */
