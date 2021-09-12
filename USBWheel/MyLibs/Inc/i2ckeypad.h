#ifndef i2ckeypad_h
#define i2ckeypad_h

#include "cppmain.h"

extern I2C_HandleTypeDef hi2c1;

class i2ckeypad {
public:
  i2ckeypad();
  uint16_t get_key();

private:
  void pcf8575_write(uint8_t data);
  uint8_t pcf8575_read();
};

#endif

