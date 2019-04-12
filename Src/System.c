/*

 * System.c
 *
 *  Created on: 10 апр. 2019 г.
 *      Author: ilyal
 */

#include "System.h"

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {

 }

uint16_t System_ButtonScan()
{
	uint16_t button;
	uint16_t one = 0x01;
	if(HAL_GPIO_ReadPin(CH1_Start_GPIO_Port, CH1_Start_Pin) == GPIO_PIN_RESET)
		button |= one << CH1_Start;
	else
		button &=  ~(one << CH1_Start);
	if(HAL_GPIO_ReadPin(CH2_Start_GPIO_Port, CH2_Start_Pin) == GPIO_PIN_RESET)
		button |= one << CH2_Start;
	else
		button &= ~(one << CH2_Start);
	if(HAL_GPIO_ReadPin(CH3_Start_GPIO_Port, CH3_Start_Pin) == GPIO_PIN_RESET)
		button |= one << CH3_Start;
	else
		button &= ~(one << CH3_Start);
	if(HAL_GPIO_ReadPin(CH4_Start_GPIO_Port, CH4_Start_Pin) == GPIO_PIN_RESET)
		button |= one << CH4_Start;
	else
		button &= ~(one << CH4_Start);

}
