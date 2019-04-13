/*
 * Buttons.c
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: ilyal
 */

#include "Buttons.h"

void TButton_0()
{
	numberBuffer *= 10;
}

void TButton_1()
{
	numberBuffer *= 10;
	numberBuffer += 1;
}

void TButton_2()
{
	numberBuffer *= 10;
	numberBuffer += 2;
}

void TButton_3()
{
	numberBuffer *= 10;
	numberBuffer += 3;
}

void TButton_4()
{
	numberBuffer *= 10;
	numberBuffer += 4;
}

void TButton_5()
{
	numberBuffer *= 10;
	numberBuffer += 5;
}

void TButton_6()
{
	numberBuffer *= 10;
	numberBuffer += 6;
}

void TButton_7()
{
	numberBuffer *= 10;
	numberBuffer += 7;
}

void TButton_8()
{
	numberBuffer *= 10;
	numberBuffer += 8;
}

void TButton_9()
{
	numberBuffer *= 10;
	numberBuffer += 9;
}

void TButton_10()
{
	numberBuffer /= 10;
}

void TButton_11()
{

}

void TButton_12()
{

}

void TButton_13()
{

}

void TButton_14()
{

}

void TButton_15()
{

}

void ButtonScan()
{
	if(HAL_GPIO_ReadPin(CH1_Start_GPIO_Port, CH1_Start_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH2_Start_GPIO_Port, CH2_Start_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH3_Start_GPIO_Port, CH3_Start_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH4_Start_GPIO_Port, CH4_Start_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH1_Select_GPIO_Port, CH1_Select_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH2_Select_GPIO_Port, CH2_Select_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH3_Select_GPIO_Port, CH3_Select_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(CH4_Select_GPIO_Port, CH4_Select_Pin) == GPIO_PIN_RESET)
	{

	}
	if(HAL_GPIO_ReadPin(Calibrate_GPIO_Port, Calibrate_Pin) == GPIO_PIN_RESET)
	{

	}
}
