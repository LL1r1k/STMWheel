/*
 * MotorPWM.h
 *
 *  Created on: Mar 29, 2020
 *      Author: Yannick
 */

#ifndef MOTORBTS7960_H_
#define MOTORBTS7960_H_

#include "cppmain.h"


extern TIM_HandleTypeDef htim1;
/*
 * Generates a 0-100% 2 PWM signals on 2 chanels of tim1
 */
class MotorBTS7960{
public:
	MotorBTS7960();
	virtual ~MotorBTS7960();

	void turn(int16_t power);
	void stop();
	void start();

private:
	const uint32_t period = 3599;
	bool active = false;
	const uint32_t channel1 = TIM_CHANNEL_1;
	const uint32_t channel2 = TIM_CHANNEL_2;

	TIM_HandleTypeDef* timer = &htim1;
};
#endif /* MOTORBTS7960_H_ */
