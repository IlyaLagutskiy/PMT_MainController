/*
 * CY8CMBR3xxx.h
 *
 *  Created on: 12 апр. 2019 г.
 *      Author: ilyal
 */

#ifndef CY8CMBR3XXX_H_
#define CY8CMBR3XXX_H_

#include "CY8CMBR3xxx_APIs.h"

#define CY8CMBR3xxx_SlaveAddress 0x37

void CY8CMBR3xxx_CheckButton();
uint8_t ButtonExtract(uint16_t buttons, uint8_t buttonNumber);

#endif /* CY8CMBR3XXX_H_ */
