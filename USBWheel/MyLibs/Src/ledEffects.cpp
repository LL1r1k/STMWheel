#include "ledEffects.h"
#include "main.h"

uint32_t lastTick = 0;

void updateLeds(){
	if(HAL_GetTick() - lastTick > 500){
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		lastTick = HAL_GetTick();
	}
}
