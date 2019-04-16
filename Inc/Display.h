/*
 * Display.h
 *
 *  Created on: 16 апр. 2019 г.
 *      Author: ilyal
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "stm32f0xx_hal.h"
#include "Channels.h"

#define Value_Amplitude 0x01
#define Value_Frequency 0x02
#define Value_TotalTime 0x03
#define Value_RemainTime 0x04
#define Value_IndTemp 0x10
#define Value_PcbTemp 0x11
#define Value_State 0xF0

void Display_ChangeActiveChannel();
void Display_ChangeSelectedValue(uint8_t valueType, uint32_t value);
void Display_NumberEnterWindow(uint8_t windowState, uint32_t value);
void Display_ErrorWindow();
void Display_UpdateData();

#endif /* DISPLAY_H_ */
