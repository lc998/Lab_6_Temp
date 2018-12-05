/*
 * delay.c
 *
 *  Created on: 25 apr. 2018
 *      Author: Lucas
 */
#include "delay.h"

uint32_t volatile sekTick = 0, usekTick = 0;

void delay_us(uint32_t delay){
	HAL_TIM_Base_Start_IT(&htim11); /* denna rad kan stå antingen I delay.c eller I main. Vilken skillnad fås?*/
	uint32_t current = usekTick;

	while((usekTick-current)<(delay-2)){
	}
	HAL_TIM_Base_Stop_IT(&htim11);

}
