#include "cppmain.h"
#include "ws2812.h"
#include "FFBWheel.h"

bool running = true;

FFBWheel* mainclass;
extern uint32_t ADC_BUF[ADC_CHANNELS];

extern uint16_t BUF_DMA [ARRAY_LEN];
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

// emulated I2C RAM
uint8_t offset = 0;
uint8_t first = 1;
uint8_t buffer[5];

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	offset = 0;
	if(mainclass->needSave)
	{
		mainclass->saveFlash();
		mainclass->needSave = false;
	}

	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if( TransferDirection==I2C_DIRECTION_TRANSMIT )
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, &mainclass->i2cButtonsBuffer[offset++], 1, I2C_NEXT_FRAME);
	else
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*)(mainclass->pi2cBuf + offset++), 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Slave_Seq_Receive_IT(hi2c, &mainclass->i2cButtonsBuffer[offset++], 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c,  (uint8_t*)(mainclass->pi2cBuf + offset++), 1, I2C_NEXT_FRAME);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if( HAL_I2C_GetError(hi2c)==HAL_I2C_ERROR_AF )
		offset--;
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{

}

uint8_t initialized = false;
uint32_t lastTickInit;

void cppmain() {
	ws2812_init();
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)&BUF_DMA, ARRAY_LEN);
	HAL_ADC_Start_DMA(&hadc1, ADC_BUF, ADC_CHANNELS);

	mainclass = new FFBWheel();
	mainclass->usbInit();

	lastTickInit = HAL_GetTick();

	while(running){
		if(HAL_GetTick() - lastTickInit > 5000 && !initialized)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			initialized = true;
			HAL_I2C_EnableListen_IT(&hi2c1);
		}
		mainclass->update();
		if(initialized)
			updateLeds();
	}

}





