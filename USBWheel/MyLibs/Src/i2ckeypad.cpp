#include "i2ckeypad.h"

#define COL0  0
#define COL1  1
#define COL2  2
#define COL3  7
#define ROW0  3
#define ROW1  4
#define ROW2  5
#define ROW3  6

uint8_t num_rows = 4;
uint8_t num_cols = 3;

static uint8_t row_select;

static uint8_t current_data;

const int hex_data[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

const int pcf8574_row_data[4] =
{
  hex_data[ROW1] | hex_data[ROW2] | hex_data[ROW3] |  hex_data[COL0] | hex_data[COL1] | hex_data[COL2] | hex_data[COL3],
  hex_data[ROW0] | hex_data[ROW2] | hex_data[ROW3] |  hex_data[COL0] | hex_data[COL1] | hex_data[COL2] | hex_data[COL3],
  hex_data[ROW0] | hex_data[ROW1] | hex_data[ROW3] |  hex_data[COL0] | hex_data[COL1] | hex_data[COL2] | hex_data[COL3],
  hex_data[ROW0] | hex_data[ROW1] | hex_data[ROW2] |  hex_data[COL0] | hex_data[COL1] | hex_data[COL2] | hex_data[COL3],
};

int col[4] = {hex_data[COL0], hex_data[COL1], hex_data[COL2], hex_data[COL3]};

i2ckeypad::i2ckeypad()
{
  row_select = 0;
}


uint16_t i2ckeypad::get_key()
{
	uint8_t tmp_data;
	uint8_t r;
	uint16_t button = 0;

	pcf8575_write(pcf8574_row_data[row_select]);

	for (r = 0; r < num_cols; r++)
	{
		tmp_data = pcf8575_read();

		tmp_data ^= current_data;

		if (col[r] == tmp_data)
		{
			button |= (1 << (row_select * 4 + r));
			return button;
		}
	}
	pcf8575_write(0xff);

	// Next row
	row_select++;
	if (row_select == num_rows)
	{
		row_select = 0;
	}

	return 0;
}


void i2ckeypad::pcf8575_write(uint8_t data)
{
  current_data = data;
  uint8_t buf[2] = {data, 0xff};

  HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(0x27<<1), buf, 2, 10);
}

uint8_t i2ckeypad::pcf8575_read()
{
	uint8_t buf[2];
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(0x27<<1), buf, 2, 10);
	return buf[0];
}

