/*
 * delay.h
 *
 *  Created on: 25 apr. 2018
 *      Author: Lucas
 */
#pragma once
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim11;

void delay_us(uint32_t delay);
