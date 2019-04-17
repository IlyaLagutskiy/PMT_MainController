/*
 * Buttons.c
 *
 *  Created on: 12 ���. 2019 �.
 *      Author: ilyal
 */

#include "Buttons.h"

extern uint32_t numberBuffer;

void TButton_0() /**/
{
	numberBuffer *= 10;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_1() /**/
{
	numberBuffer *= 10;
	numberBuffer += 1;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_2() /**/
{
	numberBuffer *= 10;
	numberBuffer += 2;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_3() /**/
{
	numberBuffer *= 10;
	numberBuffer += 3;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_4() /**/
{
	numberBuffer *= 10;
	numberBuffer += 4;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_5() /**/
{
	numberBuffer *= 10;
	numberBuffer += 5;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_6() /**/
{
	numberBuffer *= 10;
	numberBuffer += 6;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_7() /**/
{
	numberBuffer *= 10;
	numberBuffer += 7;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_8() /**/
{
	numberBuffer *= 10;
	numberBuffer += 8;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_9() /**/
{
	numberBuffer *= 10;
	numberBuffer += 9;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_10() /**/
{
	numberBuffer /= 10;
	Display_UpdateNumberWindow(numberBuffer);
}

void TButton_11() /**/
{

}

void TButton_12() /**/
{

}

void TButton_13() /**/
{

}

void TButton_14() /**/
{

}

//void TButton_15()
//{
//
//}

void ButtonScan() /**/
{
	if(HAL_GPIO_ReadPin(CH1_Start_GPIO_Port, CH1_Start_Pin) == GPIO_PIN_RESET)
	{
		 Channel_StartStop(CH1);
	}
	if(HAL_GPIO_ReadPin(CH2_Start_GPIO_Port, CH2_Start_Pin) == GPIO_PIN_RESET)
	{
		Channel_StartStop(CH2);
	}
	if(HAL_GPIO_ReadPin(CH3_Start_GPIO_Port, CH3_Start_Pin) == GPIO_PIN_RESET)
	{
		Channel_StartStop(CH3);
	}
	if(HAL_GPIO_ReadPin(CH4_Start_GPIO_Port, CH4_Start_Pin) == GPIO_PIN_RESET)
	{
		Channel_StartStop(CH4);
	}
	if(HAL_GPIO_ReadPin(CH1_Select_GPIO_Port, CH1_Select_Pin) == GPIO_PIN_RESET)
	{
		Channel_Select(CH1);
	}
	if(HAL_GPIO_ReadPin(CH2_Select_GPIO_Port, CH2_Select_Pin) == GPIO_PIN_RESET)
	{
		Channel_Select(CH2);
	}
	if(HAL_GPIO_ReadPin(CH3_Select_GPIO_Port, CH3_Select_Pin) == GPIO_PIN_RESET)
	{
		Channel_Select(CH3);
	}
	if(HAL_GPIO_ReadPin(CH4_Select_GPIO_Port, CH4_Select_Pin) == GPIO_PIN_RESET)
	{
		Channel_Select(CH4);
	}
	if(HAL_GPIO_ReadPin(Calibrate_GPIO_Port, Calibrate_Pin) == GPIO_PIN_RESET)
	{

	}
}

void TButton_Callback(uint8_t num)
{

}
