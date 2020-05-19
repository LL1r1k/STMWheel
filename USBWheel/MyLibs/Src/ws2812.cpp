#include <ws2812.h>

extern TIM_HandleTypeDef htim2;

uint16_t BUF_DMA [ARRAY_LEN] = {0};
uint8_t bright = 128;
RGB LED_RPM []= {
		{0, bright, 0},
		{0, bright, 0},
		{0, bright, 0},
		{0, bright, 0},

		{bright, bright, 0},
		{bright, bright, 0},
		{bright, bright, 0},
		{bright, bright, 0},

		{bright,0 , 0},
		{bright,0 , 0},
		{bright,0 , 0},
		{bright,0 , 0},

		{0, 0, bright},
		{0, 0, bright},
		{0, 0, bright},
		{0, 0, bright}
};

void ws2812_init(void)
{
  int i;
  for(i=DELAY_LEN;i<ARRAY_LEN;i++)
	  BUF_DMA[i] = LOW;
}

void ws2812_pixel_rgb_to_buf_dma(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX)
{
  for(uint16_t i = 0; i < 8; i++)
  {
      BUF_DMA[DELAY_LEN+posX*24+i+0] = BitIsSet(Rpixel,(7-i)) ? HIGH : LOW;
      BUF_DMA[DELAY_LEN+posX*24+i+8] = BitIsSet(Gpixel,(7-i)) ? HIGH : LOW;
      BUF_DMA[DELAY_LEN+posX*24+i+16] = BitIsSet(Bpixel,(7-i)) ? HIGH : LOW;
  }
}

void setup_rpm_ws2812(uint8_t rgb_array)
{
	for(uint16_t i = 0; i < LED_COUNT; i++)
	{
		if(i < rgb_array)
			ws2812_pixel_rgb_to_buf_dma(LED_RPM[i].R, LED_RPM[i].G, LED_RPM[i].B, i);
		else
			ws2812_pixel_rgb_to_buf_dma(0, 0, 0, i);
	}
}

void setup_ws2812(RGB* rgb_array, uint8_t size)
{
	for(uint16_t i = 0; i < size; i++)
	{
		ws2812_pixel_rgb_to_buf_dma(rgb_array[i].R, rgb_array[i].G, rgb_array[i].B, i);
	}
}
