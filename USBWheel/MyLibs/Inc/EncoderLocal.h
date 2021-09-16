#ifndef ENCODERLOCAL_H_
#define ENCODERLOCAL_H_

#include "cppmain.h"
#include "constants.h"
#include "ExtiHandler.h"
#include "TimerHandler.h"

extern TIM_HandleTypeDef TIM_ENC;

class EncoderLocal: public ExtiHandler, TimerHandler {
public:

	EncoderLocal();
	virtual ~EncoderLocal();

	float getPos_f();

	int32_t getPos();
	uint32_t getPosCpr();
	void setPos(int32_t pos);
	void setPeriod(uint32_t period);
	void overflowCallback();
	void exti(uint16_t GPIO_Pin);
	void timerElapsed(TIM_HandleTypeDef* htim);

	uint32_t getPpr();
	void setPpr(uint32_t ppr);

	int32_t offset = 0;
	int32_t ppr = 2000;
	int32_t degree = 0;
	int32_t  currentPosition = 0;
	int32_t  lastPosition = 0;
	int32_t  currentSpeed = 0;
	int32_t  lastSpeed = 0;
	int32_t  currentAcceleration = 0;
	int32_t  currentTorque = 0;
	int32_t  lastTorque = 0;

private:
	TIM_HandleTypeDef* htim;
	uint8_t first = true;
};

#endif /* ENCODERLOCAL_H_ */
