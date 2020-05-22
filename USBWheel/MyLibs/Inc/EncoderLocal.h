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

	int32_t getPos();
	uint32_t getPosCpr();
	void setPos(int32_t pos);
	void setPeriod(uint32_t period);
	void overflowCallback();
	void exti(uint16_t GPIO_Pin);
	void timerElapsed(TIM_HandleTypeDef* htim);

	uint32_t getPpr();
	void setPpr(uint32_t ppr);

	int32_t ppr = 2000;
	uint32_t offset;
	uint16_t maxAngle;
	int32_t maxValue;
	int32_t minValue;
	int32_t  currentPosition;
	int32_t  lastPosition;
	int32_t  correctPosition;
	int32_t  currentVelocity;
	int32_t  lastVelocity;
	int32_t  maxVelocity;
	int32_t  currentAcceleration;
	int32_t  maxAcceleration;
	int32_t  positionChange;
	int32_t  maxPositionChange;
	uint32_t lastEncoderTime;

private:
	TIM_HandleTypeDef* htim;
	uint8_t first = true;
};

#endif /* ENCODERLOCAL_H_ */
