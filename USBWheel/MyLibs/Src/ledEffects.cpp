#include "ledEffects.h"
#include "main.h"

uint32_t sysledtick=0;

void pulseSysLed(){
	sysledtick = HAL_GetTick();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void updateLeds(){
	if(sysledtick!=0 && HAL_GetTick() > sysledtick+35){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		sysledtick = 0;
	}
}
