/*

 * System.c
 *
 *  Created on: 10 апр. 2019 г.
 *      Author: ilyal
 */

#include "System.h"

uint32_t numberBuffer;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{

	}
	if (htim->Instance == htim7.Instance)
	{
		if (CH_State[0] == ChannelState_ACTIVE)
		{
			--CH_time[0];
			if(CH_time[0] == 0)
			{
				Channel_StartStop(CH1);
			}
		}
		if (CH_State[1] == ChannelState_ACTIVE)
		{
			--CH_time[1];
			if(CH_time[1] == 0)
			{
				Channel_StartStop(CH2);
			}
		}
		if (CH_State[2] == ChannelState_ACTIVE)
		{
			--CH_time[2];
			if(CH_time[2] == 0)
			{
				Channel_StartStop(CH3);
			}
		}
		if (CH_State[3] == ChannelState_ACTIVE)
		{
			--CH_time[3];
			if(CH_time[3] == 0)
			{
				Channel_StartStop(CH4);
			}
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	volatile uint8_t exti1 = 0x00;
	volatile uint8_t exti12 = 0x00;
	if (GPIO_Pin == GPIO_PIN_1) /*CY8CMBR3xxx Host Interrupt*/
	{
		if (exti1 == 0x00)
		{
			void CY8CMBR3xxx_CheckButton();
			exti1 = 0xF0;
		}
		else
		{
			exti1 = 0x00;
		}
	}
	if (GPIO_Pin == GPIO_PIN_12) /*STOP Interrupt*/
	{
		if (exti12 == 0x00)
		{
			HAL_GPIO_WritePin(CH1_EXTI_GPIO_Port, CH1_EXTI_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CH2_EXTI_GPIO_Port, CH2_EXTI_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CH3_EXTI_GPIO_Port, CH3_EXTI_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(CH4_EXTI_GPIO_Port, CH4_EXTI_Pin, GPIO_PIN_SET);
			exti12 = 0xF0;
			CH_State[0] = ChannelState_ESTOP;
			CH_State[1] = ChannelState_ESTOP;
			CH_State[2] = ChannelState_ESTOP;
			CH_State[3] = ChannelState_ESTOP;
		}
		else
		{
			HAL_GPIO_WritePin(CH1_EXTI_GPIO_Port, CH1_EXTI_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CH2_EXTI_GPIO_Port, CH2_EXTI_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CH3_EXTI_GPIO_Port, CH3_EXTI_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(CH4_EXTI_GPIO_Port, CH4_EXTI_Pin, GPIO_PIN_RESET);
			exti12 = 0x00;
			CH_State[0] = ChannelState_ACTIVE;
			CH_State[1] = ChannelState_ACTIVE;
			CH_State[2] = ChannelState_ACTIVE;
			CH_State[3] = ChannelState_ACTIVE;
		}
	}
}
