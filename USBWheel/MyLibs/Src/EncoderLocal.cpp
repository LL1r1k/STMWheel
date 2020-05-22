/*
 * EncoderLocal.cpp
 *
 *  Created on: 02.02.2020
 *      Author: Yannick
 */

#include "EncoderLocal.h"

EncoderLocal::EncoderLocal() {
	this->htim = &TIM_ENC;
	setPos(0);

	this->htim->Instance->CR1 = 1;
	HAL_TIM_Base_Start_IT(htim);
	offset = 0;

}

EncoderLocal::~EncoderLocal() {
	this->htim->Instance->CR1 = 0;
}


int32_t EncoderLocal::getPos(){
	int32_t timpos = htim->Instance->CNT - 0x7fff;
	return timpos + offset;
}
void EncoderLocal::setPos(int32_t pos){
	this->currentPosition = pos;
	htim->Instance->CNT = pos+0x7fff;
}

void EncoderLocal::setPeriod(uint32_t period){
	this->htim->Instance->ARR = period-1;
}

void EncoderLocal::exti(uint16_t GPIO_Pin){
	if(GPIO_Pin == ENCODER_Z_Pin){
		// Encoder Z pin activated
	}
}

void EncoderLocal::timerElapsed(TIM_HandleTypeDef* htim){
	if(htim == this->htim){
		overflowCallback();
	}
}

void EncoderLocal::overflowCallback(){
	if(first)					//TODO: figure it out
	{
		first = false;
		return;
	}
	if(htim->Instance->CNT > this->htim->Instance->ARR/2){
		offset -= htim->Instance->ARR+1;
	}else{
		offset += htim->Instance->ARR+1;
	}
}

uint32_t EncoderLocal::getPpr(){
	return this->ppr;
}

uint32_t EncoderLocal::getPosCpr(){
	return this->ppr;
}

void EncoderLocal::setPpr(uint32_t ppr){
	this->ppr = ppr;
}
