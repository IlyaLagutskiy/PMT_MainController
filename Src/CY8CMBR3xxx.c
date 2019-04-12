/*
 * CY8CMBR3xxx.c
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: ilyal
 */

#include "CY8CMBR3xxx.h"

void CY8CMBR3xxx_CheckButton()
{
	CY8CMBR3xxx_SENSORSTATUS buttons;
	uint16_t buttonChange;
	CY8CMBR3xxx_ReadSensorStatus(CY8CMBR3xxx_SlaveAddress, &buttons);
	buttonChange = buttons.buttonStatus - buttons.latchedButtonStatus;
	if(ButtonExtract(buttonChange, 0))
	{

	}
	if(ButtonExtract(buttonChange, 1))
	{

	}
	if(ButtonExtract(buttonChange, 2))
	{

	}
	if(ButtonExtract(buttonChange, 3))
	{

	}
	if(ButtonExtract(buttonChange, 4))
	{

	}
	if(ButtonExtract(buttonChange, 5))
	{

	}
	if(ButtonExtract(buttonChange, 6))
	{

	}
	if(ButtonExtract(buttonChange, 7))
	{

	}
	if(ButtonExtract(buttonChange, 8))
	{

	}
	if(ButtonExtract(buttonChange, 9))
	{

	}
	if(ButtonExtract(buttonChange, 10))
	{

	}
	if(ButtonExtract(buttonChange, 11))
	{

	}
	if(ButtonExtract(buttonChange, 12))
	{

	}
	if(ButtonExtract(buttonChange, 13))
	{

	}
	if(ButtonExtract(buttonChange, 14))
	{

	}
	if(ButtonExtract(buttonChange, 15))
	{

	}
}

uint8_t ButtonExtract(uint16_t buttons, uint8_t buttonNumber)
{
	buttons >>= buttonNumber;
	buttons <<= 15;
	return (buttons == 0x2000) ? 0xF0 : 0x00;
}
