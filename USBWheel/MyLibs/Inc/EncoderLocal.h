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
	void setOffset(int32_t offset);
	void setPeriod(uint32_t period);
	void overflowCallback();
	void exti(uint16_t GPIO_Pin);
	void timerElapsed(TIM_HandleTypeDef* htim);

	uint32_t getPpr();
	void setPpr(uint32_t ppr);

private:
	TIM_HandleTypeDef* htim;
	int32_t offset = 0;
	uint8_t first = true;
	int32_t pos = 0; // Extra position counter for overflows
	int32_t ppr = 2000;
};

#endif /* ENCODERLOCAL_H_ */
