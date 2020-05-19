
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

struct FFBWheelConfig{
	uint8_t check = 0x57;
	uint8_t axes = 0b00000111;
	uint8_t I2CButtons = 0x01;
	uint8_t nLocalButtons = 0;
	uint16_t degreesOfRotation = 900;
	uint16_t power = 2000;
	uint16_t encoderPPR = 2000;
	uint8_t maxAdcCount = 8;

	uint8_t inverted = false;
	uint16_t endstop_gain = 20;
	uint8_t constantGain = 100;
	uint8_t rampGain = 100;
	uint8_t squareGain = 100;
	uint8_t sinGain = 100;
	uint8_t triangleGain = 100;
	uint8_t sawToothDownGain = 100;
	uint8_t sawToothUpGain = 100;
	uint8_t springGain = 100;
	uint8_t damperGain = 100;
	uint8_t inertiaGain = 100;
	uint8_t frictionGain = 100;
	uint8_t totalGain = 100;

	bool isequal(FFBWheelConfig& conf)
	{
		if(	check == conf.check &&
			axes == conf.axes &&
			I2CButtons == conf.I2CButtons &&
			nLocalButtons == conf.nLocalButtons &&
			degreesOfRotation == conf.degreesOfRotation &&
			power == conf.power &&
			endstop_gain == conf.endstop_gain &&
			encoderPPR == conf.encoderPPR &&
			maxAdcCount == conf.maxAdcCount &&
			inverted == conf.inverted &&
			constantGain == conf.constantGain &&
			rampGain == conf.rampGain &&
			squareGain == conf.squareGain &&
			sinGain == conf.sinGain &&
			triangleGain == conf.triangleGain &&
			sawToothDownGain == conf.sawToothDownGain &&
			sawToothUpGain == conf.sawToothUpGain &&
			springGain == conf.springGain &&
			damperGain == conf.damperGain &&
			inertiaGain == conf.inertiaGain &&
			frictionGain == conf.frictionGain &&
			totalGain == conf.totalGain)
			return true;
		else
			return false;
	}
};




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

	void update();
	void cdcRcv(char* Buf, uint32_t *Len);

	static FFBWheelConfig decodeConf();
	static void encodeConf(FFBWheelConfig conf);


	void adcUpd(volatile uint32_t* ADC_BUF);
	void timerElapsed(TIM_HandleTypeDef* htim);

	bool usb_update_flag = false;
	bool update_flag = false;
	uint8_t adcCount =0;

	int32_t getEncValue(EncoderLocal* enc,uint16_t degrees);

	CmdParser parser = CmdParser();

private:
	void send_report();
	int16_t updateEndstop();

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


	int32_t lastScaledEnc = 0;
	int32_t scaledEnc = 0;
	int32_t speed = 0;
	bool tmcFeedForward = true;

	int32_t torqueFFgain = 50000;
	int32_t torqueFFconst = 0;
	int32_t velocityFFgain = 30000;
	int32_t velocityFFconst = 0;

};

#endif /* SRC_FFBWHEEL_H_ */
