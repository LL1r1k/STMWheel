/*
 * LocalButtons.cpp
 *
 *  Created on: 09.02.2020
 *      Author: Yannick
 */

#include <LocalButtons.h>

LocalButtons::LocalButtons() {
	//Initialize I2C buttons
	button_buf[0] = 0xff;
	button_buf[1] = 0xff;

	I2C_status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(0x27<<1), button_buf, 2, 10);
	if ( I2C_status != HAL_OK ) {
		pulseSysLed();
	}

}

LocalButtons::~LocalButtons() {
	// TODO Auto-generated destructor stub
}


void LocalButtons::readButtons(uint32_t* buf){
	I2C_status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(0x27<<1), button_buf, 2, 10);
	if(I2C_status == HAL_OK)
	{
		button_buf[0]=~button_buf[0];
		button_buf[1]=~button_buf[1];
		*buf |= button_buf[0];
		*buf <<= 8;
		*buf |= button_buf[1];
	}
	uint8_t buttons = this->nButtons;
	for(uint8_t i = 0;i<buttons;i++){
		*buf |= !HAL_GPIO_ReadPin(button_ports[i],button_pins[i]) << (i + 16);
	}
}


uint16_t LocalButtons::getBtnNum(){
	return this->nButtons;
}
