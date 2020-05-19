#include <vector>
#include <global_callbacks.h>
#include "main.h"
#include "cppmain.h"
#include "constants.h"

#include "AdcHandler.h"
#include "FFBWheel.h"
#include "UsbHidHandler.h"
#include "ExtiHandler.h"
#include "TimerHandler.h"
#include "CommandHandler.h"

extern FFBWheel* mainclass;

std::vector<AdcHandler*> adcHandlers;
std::vector<CommandHandler*> cmdHandlers;

volatile uint32_t ADC_BUF[ADC_CHANNELS] = {0};


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
		for(AdcHandler* c : adcHandlers){
			c->adcUpd(ADC_BUF);
		}
}

std::vector<TimerHandler*> timerHandlers;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	for(TimerHandler* c : timerHandlers){
			c->timerElapsed(htim);
		}
}

std::vector<ExtiHandler*> extiHandlers;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	for(ExtiHandler* c : extiHandlers){
		c->exti(GPIO_Pin);
	}
}

void CDC_Callback(uint8_t* Buf, uint32_t *Len){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); //DEBUG
	if(mainclass!=nullptr)
		mainclass->cdcRcv((char*)Buf,Len);
}

UsbHidHandler* globalHidHandler = nullptr;
void USBD_OutEvent_HID(uint8_t* report){
	if(globalHidHandler!=nullptr)
			globalHidHandler->hidOut(report);
}
void USBD_GetEvent_HID(uint8_t id,uint16_t len,uint8_t** return_buf){
	if(globalHidHandler!=nullptr)
		globalHidHandler->hidGet(id, len, return_buf);
}

void USB_SOF(){
	if(mainclass!=nullptr)
		mainclass->SOF();
}
