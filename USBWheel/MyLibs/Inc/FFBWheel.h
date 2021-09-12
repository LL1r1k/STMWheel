
#ifndef SRC_FFBWHEEL_H_
#define SRC_FFBWHEEL_H_

#include <CmdParser.h>
#include <MotorBTS7960.h>
#include "LocalButtons.h"

#include "EncoderLocal.h"

#include "HidFFB.h"
#include "AdcHandler.h"
#include "TimerHandler.h"

#include "CommandHandler.h"
#include "usbd_cdc_if.h"

#include "ledEffects.h"
#include "eeprom.h"
#include "ws2812.h"


class FFBWheel: public AdcHandler, TimerHandler, CommandHandler{
public:
	FFBWheel();
	virtual ~FFBWheel();


	void executeCommands(std::vector<ParsedCommand> commands);
	bool command(ParsedCommand* cmd,std::string* reply);
	bool executeSysCommand(ParsedCommand* cmd,std::string* reply);
	void SOF();
	void usbInit(); // initialize a composite usb device

	void saveFlash();
	void restoreFlash();

	void setCfFilter(uint32_t f,uint8_t q);

	void update();
	void cdcRcv(char* Buf, uint32_t *Len);

	static FFBWheelConfig decodeConf();

	void adcUpd(volatile uint32_t* ADC_BUF);
	void timerElapsed(TIM_HandleTypeDef* htim);

	bool usb_update_flag = false;
	bool update_flag = false;
	uint8_t adcCount =0;

	int32_t getEncValue(EncoderLocal* enc,uint16_t degrees);
	void initEncoder();
	CmdParser parser = CmdParser();

	uint8_t* pi2cBuf;

	uint8_t i2cButtonsBuffer[9] = {0,};

	uint8_t needSave = false;
private:
	void send_report();
	int16_t updateEndstop();

	int16_t i2cBuffer[255] = {0,};
	uint16_t i2cSize = 0;

	HidFFB* ffb;
	TIM_HandleTypeDef* timer_update;
	int32_t torque = 0; // last torque
	int32_t endstopTorque = 0; // last torque
	FFBWheelConfig conf;

	MotorBTS7960* drv = nullptr;
	EncoderLocal* enc = nullptr;

	LocalButtons* btns = nullptr;

	volatile uint16_t adc_buf[ADC_PINS];
	volatile uint32_t adc_buf2[ADC_PINS];
	reportHID_t reportHID;

	int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);


	int32_t lastScaledEnc = 0;
	int32_t scaledEnc = 0;
	int32_t speed = 0;
	bool tmcFeedForward = true;

	int32_t torqueFFgain = 50000;
	int32_t torqueFFconst = 0;
	int32_t velocityFFgain = 30000;
	int32_t velocityFFconst = 0;

	uint16_t I2C_SLAVE_ADDR = 0x48;
};

#endif /* SRC_FFBWHEEL_H_ */
