/*
 * delay.c
 *
 *  Created on: Jul 30, 2024
 *      Author: Harman 4th
 */

#include "delay.h"

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	while(__HAL_TIM_GET_COUNTER(&htim10) < us);
}
