/*
 * Display.c
 *
 *  Created on: 16 апр. 2019 г.
 *      Author: ilyal
 */

#include "Display.h"

void Display_ChangeActiveChannel() /**/
{

}

void Display_ChangeSelectedValue(uint8_t valueType, uint32_t value) /**/
{

}

void Display_NumberEnterWindow(uint8_t windowState, uint8_t valueType, uint32_t value) /**/
{

}

void Display_ErrorWindow() /**/
{

}

void Display_UpdateData() /*OK*/
{
	Display_ChangeSelectedValue(Value_Amplitude, CH_data[selectedChannel].Amplitude);
	Display_ChangeSelectedValue(Value_Frequency, CH_data[selectedChannel].Frequency);
	Display_ChangeSelectedValue(Value_IndTemp, CH_data[selectedChannel].InductorTemp);
	Display_ChangeSelectedValue(Value_PcbTemp, CH_data[selectedChannel].PCBTemp);
	Display_ChangeSelectedValue(Value_TotalTime, CH_data[selectedChannel].Time);
	Display_ChangeSelectedValue(Value_RemainTime, CH_time[selectedChannel]);
}

void Display_UpdateNumberWindow(uint32_t value) /**/
{

}

void Display_Message(uint8_t* title, uint8_t* message)
{

}
