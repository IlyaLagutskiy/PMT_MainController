/*
 * Channels.c
 *
 *  Created on: 13 апр. 2019 г.
 *      Author: ilyal
 */

#include "Channels.h"

uint8_t selectedChannel = 0x10;
uint8_t CH_State[4] = {0x00, 0x00, 0x00, 0x00};
uint16_t CH_time[4] = {0x00, 0x00, 0x00, 0x00};
ChannelData CH_data[4] = {0};

void Channel_Select(uint8_t channel)
{
	selectedChannel = channel;
	Display_ChangeActiveChannel();
	Display_UpdateData();
}

void Channel_StartStop(uint8_t channel)
{
	StartParams params = {0};
	if(CH_State[channel] < 0xF0)
	{
		params.Amplitude = CH_data[selectedChannel].Amplitude;
		params.Frequency = CH_data[selectedChannel].Frequency;
		UART_Send(Channel_GetAddress(selectedChannel), Command_START, (uint8_t*) &params, 4);
		Channel_GetState(selectedChannel);
		Display_UpdateData();
		CH_State[channel] = ChannelState_ACTIVE;
	}
	else
	{
		Channel_ErrorHandler(channel);
	}

}

void Channel_ErrorHandler(uint8_t channel)
{
	if(channel == CH1)
	{
		HAL_GPIO_WritePin(CH1_State1_GPIO_Port, CH1_State1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH1_State2_GPIO_Port, CH1_State2_Pin, GPIO_PIN_SET);
	}
	if(channel == CH2)
	{
		HAL_GPIO_WritePin(CH2_State1_GPIO_Port, CH2_State1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH2_State2_GPIO_Port, CH2_State2_Pin, GPIO_PIN_SET);
	}
	if(channel == CH3)
	{
		HAL_GPIO_WritePin(CH3_State1_GPIO_Port, CH3_State1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH3_State2_GPIO_Port, CH3_State2_Pin, GPIO_PIN_SET);
	}
	if(channel == CH4)
	{
		HAL_GPIO_WritePin(CH4_State1_GPIO_Port, CH4_State1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH4_State2_GPIO_Port, CH4_State2_Pin, GPIO_PIN_SET);
	}
}

uint8_t Channel_GetAddress(uint8_t channel)
{
	switch(channel)
	{
		case CH1:
			return CH1_Addr;
		case CH2:
			return CH2_Addr;
		case CH3:
			return CH3_Addr;
		case CH4:
			return CH4_Addr;
	}
	return 0x00;
}

void Channel_GetState(uint8_t channel)
{
	UART_Send(Channel_GetAddress(channel), Command_STATE, 0x00, 0);
	UART_Receive();
}
