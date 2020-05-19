#include <MotorBTS7960.h>

void MotorBTS7960::turn(int16_t power){
	if(!active)
		return;

	int32_t val = (uint32_t)((abs(power) * period)/0x7fff);

	if(power < 0){
		timer->Instance->CCR1 = 0;
		timer->Instance->CCR2 = val;
	}else{
		timer->Instance->CCR1 = val;
		timer->Instance->CCR2 = 0;
	}

}

MotorBTS7960::MotorBTS7960() {
	timer->Instance->CCR1 = 0;
	timer->Instance->CCR2 = 0;
	HAL_TIM_PWM_Start(timer, channel1);
	HAL_TIM_PWM_Start(timer, channel2);
}

MotorBTS7960::~MotorBTS7960() {
	HAL_TIM_PWM_Stop(timer, channel1);
	HAL_TIM_PWM_Stop(timer, channel2);
}


void MotorBTS7960::start(){
	active = true;
}

void MotorBTS7960::stop(){
	active = false;
	timer->Instance->CCR1 = 0;
	timer->Instance->CCR2 = 0;
}

