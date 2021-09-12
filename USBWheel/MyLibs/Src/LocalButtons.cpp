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

	encoder_buf = 0;
	lastEnc = 0;
	lastTime[0] = 0;
	lastTime[1] = 0;
	lastTime[2] = 0;
	lastTime[3] = 0;

	keypad = i2ckeypad();
}

LocalButtons::~LocalButtons() {
	// TODO Auto-generated destructor stub
}


void LocalButtons::readButtons(uint32_t* buf, FFBWheelConfig* conf){
	I2C_status = HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(0x27<<1), button_buf, 2, 10);

	//TODO different wheels
	if(I2C_status == HAL_OK)	//I2C
	{
		button_buf[0]=~button_buf[0];
		button_buf[1]=~button_buf[1];

		if(conf->wheelNUM == 1)	//Encoder
		{
			uint8_t startIndex = 2;
			uint8_t encoderCount = 3;
			uint8_t count = 0;

			for(uint8_t i = startIndex; i < startIndex + encoderCount * 2; i+=2, count++)
			{
				uint8_t a = (button_buf[1] >> i) & 1;
				uint8_t b = (button_buf[1] >> (i + 1)) & 1;

				uint8_t lastA = (lastEnc >> count) & 1;

				uint32_t curTime = HAL_GetTick();

				if(curTime - lastTime[count] >= 50)
				{
					encoder_buf&= ~(1 << (count * 2));
					encoder_buf&= ~(1 << (count * 2 + 1));
				}

				if(a ^ lastA)
				{
					lastTime[count] = curTime;
					if(b ^ a)
					{
						encoder_buf &= ~(1 << (count * 2));
						encoder_buf |= 1 << (count * 2 + 1);
					}
					else
					{
						encoder_buf &= ~(1 << (count * 2 + 1));
						encoder_buf |= 1 << (count * 2);
					}
				}

				lastEnc  ^= (-a ^ lastEnc) & (1 << count);
				uint8_t tmp = (encoder_buf >> (count * 2)) & 1;
				button_buf[1]  ^= (-tmp ^ button_buf[1]) & (1 << i);
				tmp = (encoder_buf >> (count * 2 + 1)) & 1;
				button_buf[1]  ^= (-tmp ^ button_buf[1]) & (1 << (i + 1));
			}
			*buf |= button_buf[1];
			*buf <<= 16;

			uint16_t keys = keypad.get_key();
			*buf |= keys;
		}
		else
		{
			*buf |= button_buf[1];
			*buf <<= 8;
			*buf |= button_buf[0];
		}
	}
}


uint16_t LocalButtons::getBtnNum(){
	return this->nButtons;
}
