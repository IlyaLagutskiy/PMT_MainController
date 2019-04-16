/*
 * Channels.h
 *
 *  Created on: 13 апр. 2019 г.
 *      Author: ilyal
 */

#ifndef CHANNELS_H_
#define CHANNELS_H_

#include "main.h"
#include "System.h"
#include "Display.h"
#include "UART_Protocol.h"
#include "stm32f0xx_hal.h"

typedef struct
{
	uint16_t Amplitude;
	uint16_t Frequency;
	uint16_t Time;
	uint16_t InductorTemp;
	uint16_t PCBTemp;
} ChannelData;

#define CH1 0x00
#define CH2 0x01
#define CH3 0x02
#define CH4 0x03
#define ChannelState_ACTIVE 0xF0
#define ChannelState_WAIT 0xF0
#define ChannelState_COOLDOWN 0xF0
#define ChannelState_ERROR 0xF0
#define ChannelState_STOP 0xF0

extern uint8_t selectedChannel;
extern uint8_t CH_State[4];
extern uint16_t CH_time[4];
extern ChannelData CH_data[4];

void Channel_Select(uint8_t channel);
void Channel_StartStop(uint8_t channel);
void Channel_ErrorHandler(uint8_t channel);
uint8_t Channel_GetAddress(uint8_t channel);
void Channel_GetState(uint8_t channel);

#endif /* CHANNELS_H_ */
