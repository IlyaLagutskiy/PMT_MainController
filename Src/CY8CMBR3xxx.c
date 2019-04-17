/*
 * CY8CMBR3xxx.c
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: ilyal
 */

#include "CY8CMBR3xxx.h"

void CY8CMBR3xxx_CheckButton() /*OK*/
{
	CY8CMBR3xxx_SENSORSTATUS buttons;
	uint16_t buttonChange;
	CY8CMBR3xxx_ReadSensorStatus(CY8CMBR3xxx_SlaveAddress, &buttons);
	buttonChange = buttons.buttonStatus - buttons.latchedButtonStatus;
	if(ButtonExtract(buttonChange, 0))
	{
		TButton_0();
	}
	if(ButtonExtract(buttonChange, 1))
	{
		TButton_1();
	}
	if(ButtonExtract(buttonChange, 2))
	{
		TButton_2();
	}
	if(ButtonExtract(buttonChange, 3))
	{
		TButton_3();
	}
	if(ButtonExtract(buttonChange, 4))
	{
		TButton_4();
	}
	if(ButtonExtract(buttonChange, 5))
	{
		TButton_5();
	}
	if(ButtonExtract(buttonChange, 6))
	{
		TButton_6();
	}
	if(ButtonExtract(buttonChange, 7))
	{
		TButton_7();
	}
	if(ButtonExtract(buttonChange, 8))
	{
		TButton_8();
	}
	if(ButtonExtract(buttonChange, 9))
	{
		TButton_9();
	}
	if(ButtonExtract(buttonChange, 10))
	{
		TButton_10();
	}
	if(ButtonExtract(buttonChange, 11))
	{
		TButton_11();
	}
	if(ButtonExtract(buttonChange, 12))
	{
		TButton_12();
	}
	if(ButtonExtract(buttonChange, 13))
	{
		TButton_13();
	}
	if(ButtonExtract(buttonChange, 14))
	{
		TButton_14();
	}
//	if(ButtonExtract(buttonChange, 15))
//	{
//		TButton_15();
//	}
}

uint8_t ButtonExtract(uint16_t buttons, uint8_t buttonNumber) /*OK*/
{
	buttons >>= buttonNumber;
	buttons <<= 15;
	return (buttons == 0x2000) ? 0xF0 : 0x00;
}
