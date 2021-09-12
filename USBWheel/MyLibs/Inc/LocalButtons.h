/*
 * LocalButtons.h
 *
 *  Created on: 09.02.2020
 *      Author: Yannick
 */

#ifndef LOCALBUTTONS_H_
#define LOCALBUTTONS_H_

#include "cppmain.h"
#include "ledEffects.h"
#include "i2ckeypad.h"
#include "ffb_defs.h"

extern I2C_HandleTypeDef hi2c1;

class LocalButtons {
public:
	LocalButtons();
	virtual ~LocalButtons();
	void readButtons(uint32_t* buf, FFBWheelConfig* conf);
	uint16_t getBtnNum();

	const uint16_t maxButtons = 1;
	uint16_t nButtons = 1;

private:
	const uint16_t button_pins[1] = {DIN1_Pin};
	GPIO_TypeDef* button_ports[1] = {DIN1_GPIO_Port};

	i2ckeypad keypad;

	uint8_t button_buf[3];

	uint8_t encoder_buf;
	uint8_t lastEnc;
	uint32_t lastTime[3];

	const uint8_t ButtonAddr = 0x27 ; // Use 8-bit address
	HAL_StatusTypeDef I2C_status;
};

#endif /* LOCALBUTTONS_H_ */
