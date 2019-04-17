/*
 * UART_Protocol.c
 *
 *  Created on: 2 апр. 2019 г.
 *      Author: ilyal
 */

#include "UART_Protocol.h"

void UART_Send(uint8_t Address, uint8_t Command, uint8_t* Params, uint8_t ParamsLength) /*OK*/
{
	TxStruct message;
	message.Address = Address;
	message.Data = &Command;
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) &message, 2);
	if(ParamsLength)
	{
		HAL_Delay(1);
		message.Data = Params;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*) &message, ParamsLength+1);
	}
}

void UART_Receive() /*OK*/
{
	StateData data = {0};
	HAL_UART_Receive_IT(&huart1,(uint8_t*) &data, sizeof(data));
	if(data.Channel == CH1_Addr)
	{
		CH_State[0] = data.State;
		if(CH_State[0] >=0xF0)
		{
			Channel_ErrorHandler(CH1);
		}
		CH_data[0].InductorTemp = data.InductorTemp;
		CH_data[0].PCBTemp = data.PCBTemp;
	}
	if(data.Channel == CH2_Addr)
	{
		CH_State[1] = data.State;
		if(CH_State[1] >=0xF0)
		{
			Channel_ErrorHandler(CH2);
		}
		CH_data[1].InductorTemp = data.InductorTemp;
		CH_data[1].PCBTemp = data.PCBTemp;
	}
	if(data.Channel == CH3_Addr)
	{
		CH_State[2] = data.State;
		if(CH_State[2] >=0xF0)
		{
			Channel_ErrorHandler(CH3);
		}
		CH_data[2].InductorTemp = data.InductorTemp;
		CH_data[2].PCBTemp = data.PCBTemp;
	}
	if(data.Channel == CH4_Addr)
	{
		CH_State[3] = data.State;
		if(CH_State[3] >=0xF0)
		{
			Channel_ErrorHandler(CH4);
		}
		CH_data[3].InductorTemp = data.InductorTemp;
		CH_data[3].PCBTemp = data.PCBTemp;
	}
}
