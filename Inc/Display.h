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
#define ON 0x00
#define OFF 0x01

void Display_ChangeActiveChannel();
void Display_ChangeSelectedValue(uint8_t valueType, uint32_t value);
void Display_NumberEnterWindow(uint8_t windowState, uint8_t valueType, uint32_t value);
void Display_ErrorWindow();
void Display_UpdateData();
void Display_UpdateNumberWindow(uint32_t value);
void Display_Message(uint8_t* title, uint8_t* message);

#endif /* DISPLAY_H_ */
