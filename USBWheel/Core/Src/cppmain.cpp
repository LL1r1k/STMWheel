#include "cppmain.h"
#include "ws2812.h"
#include "FFBWheel.h"

bool running = true;

FFBWheel* mainclass;
extern uint32_t ADC_BUF[ADC_CHANNELS];

extern uint16_t BUF_DMA [ARRAY_LEN];
extern TIM_HandleTypeDef htim2;

USBD_HandleTypeDef hUsbDeviceFS;

void cppmain() {

	ws2812_init();
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)&BUF_DMA, ARRAY_LEN);
	HAL_ADC_Start_DMA(&HADC, ADC_BUF, ADC_CHANNELS);

	mainclass = new FFBWheel();
	mainclass->usbInit();

	while(running){
		mainclass->update();
		updateLeds();
	}

}





