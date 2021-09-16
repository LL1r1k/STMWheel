
#include "FFBWheel.h"
#include "FFBWheel_usb_init.h"
// TODO class type for parser? (Simhub for example)
//////////////////////////////////////////////

extern I2C_HandleTypeDef hi2c1;

FFBWheel::FFBWheel() {

	restoreFlash(); // Load parameters

	// Create HID FFB handler. Will receive all usb messages directly
	this->ffb = new HidFFB();
	ffb->set_config(&conf);
	//setCfFilter(conf.cfFilter_f, conf.cfFilter_q);

	pi2cBuf = (uint8_t*)&i2cBuffer;

	// Setup a timer
	extern TIM_HandleTypeDef htim3;
	this->timer_update = &htim3; // Timer setup with prescaler of sysclock
	this->timer_update->Instance->ARR = 250;
	this->timer_update->Instance->CR1 = 1;
	HAL_TIM_Base_Start_IT(this->timer_update);

	float effect_margin_scaler = ((float)conf.totalGain/255.0);
	this->torqueScaler = ((float)this->conf.maxpower / (float)0x7fff) * effect_margin_scaler;
}

FFBWheel::~FFBWheel() {
	delete drv;
	delete enc;
}

void FFBWheel::setCfFilter(uint32_t freq, uint8_t q){
	/*cfFilter_q = clip<uint8_t, uint8_t>(q,0,127);

	if(freq == 0)
		freq = 500;
	cfFilter_f = freq;
	float f = (float)cfFilter_f / (float)1000;

	ffb->setFilterFQ(f, (float)0.01 * (cfFilter_q+1));*/
}

void FFBWheel::restoreFlash(){

	conf = decodeConf();
	conf.check= 0x57;

	drv = new MotorBTS7960();
	enc = new EncoderLocal();
	//btns = new LocalButtons();

	drv->start();
	initEncoder();
}

// Saves parameters to flash
void FFBWheel::saveFlash(){
	FFBWheelConfig savedconf = decodeConf();

	uint8_t savedCofdArr[sizeof(FFBWheelConfig)];
	memcpy(savedCofdArr, &savedconf, sizeof(FFBWheelConfig));

	uint8_t confArr[sizeof(FFBWheelConfig)];
	memcpy(confArr, &conf, sizeof(FFBWheelConfig));

	if(savedCofdArr == confArr)
		return;
	uint32_t* buf = (uint32_t*)&conf;
	uint8_t len = sizeof(FFBWheelConfig);
	len = len / 4 + (len % 4 != 0 ? 1 : 0);

	EE_Format();
	EE_Writes(0x00, len, buf);
}

/*
 * Periodical update method. Called from main loop
 */

typedef struct{
	int16_t curHours;
	int16_t curMins;
	int16_t curSecs;
	int16_t gear;
	int16_t ersBar;
	int16_t fuelBar;
	int16_t mix;
	int16_t ers;
	int16_t fuel;
	int16_t fuelDelta;
	int16_t tireFL;
	int16_t tireFR;
	int16_t tireRL;
	int16_t tireRR;
	int16_t rpm;
	int16_t lapsRemains;
	int16_t position;
	int16_t brakeBias;
	int16_t speed;
	int16_t brakeFL;
	int16_t brakeFR;
	int16_t brakeRL;
	int16_t brakeRR;
	int16_t engineTemp;
	int16_t oilTemp;
	int16_t differential;
	int16_t curLapSec;
	int16_t curLapMs;
	int16_t bestLapSec;
	int16_t bestLapMs;
	int16_t lastLapSec;
	int16_t lastLapMs;
	int16_t deltaAheadSec;
	int16_t deltaAheadMs;
	int16_t deltaBehindSec;
	int16_t deltaBehindMs;
	int16_t deltaToBestSec;
	int16_t deltaToBestMs;
	int16_t safetyCarDeltaSec;
	int16_t safetyCarDeltaMs;
	int16_t flags;
	int16_t LEDBrightnes;
	int16_t led[5];
} Telemetry;
Telemetry tmpT = {0,};

void FFBWheel::update(){
	int16_t lasttorque = endstopTorque;
	bool updateTorque = false;
	if(drv == nullptr || enc == nullptr){
		return;
	}

	if(usb_update_flag || update_flag){

		torque = 0;
		scaledEnc = getEncValue(enc, conf.degreesOfRotation);
		update_flag = false;

		if(abs(scaledEnc) > 0xffff){
			drv->stop();
		}
		endstopTorque = updateEndstop();

	}
	if(usb_update_flag){
		lastScaledEnc = scaledEnc;

		usb_update_flag = false;
		torque = ffb->calculateEffects(enc);
		if(endstopTorque == 0 || (endstopTorque > 0 && torque > 0) || (endstopTorque < 0 && torque < 0))
			updateTorque = true;
		this->send_report();
	}

	if(endstopTorque!=lasttorque || updateTorque){
		torque *= torqueScaler;
		torque += endstopTorque;
		torque = (conf.inverted) ? torque : -torque;
		torque = clip<int32_t, int32_t>(torque, -0x7fff, 0x7fff);
		enc->currentTorque = torque;

		/*torque = clip<int32_t,int16_t>(torque, -this->conf.maxpower, this->conf.maxpower);
		if(torque > 0) torque = map(torque, 0, this->conf.maxpower, MIN(this->conf.minForce, this->conf.maxpower), this->conf.maxpower);
		else if (torque < 0) torque = map(torque, 0, -this->conf.maxpower, MAX(-this->conf.minForce, -this->conf.maxpower), -this->conf.maxpower);*/

		drv->turn(torque);

		/*tmpT.brakeFL = torque / 10;
		tmpT.flags = 8;
		memcpy(i2cBuffer, &tmpT, sizeof(tmpT));*/

	}

}

int32_t FFBWheel::map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int16_t FFBWheel::updateEndstop(){
	int8_t clipdir = cliptest<int32_t,int32_t>(lastScaledEnc, -0x7fff, 0x7fff);
	if(clipdir == 0){
		return 0;
	}
	int32_t addtorque = 0;
	addtorque += clip<int32_t,int32_t>(abs(lastScaledEnc)-0x7fff,-0x7fff,0x7fff);
	float scale = ((float)conf.endstop_gain * 50.00) / 255.00; // 0..50
	addtorque *= scale;
	addtorque *= -clipdir;

	return clip<int32_t,int32_t>(addtorque, -0x7fff ,0x7fff);
}

void FFBWheel::adcUpd(volatile uint32_t* ADC_BUF){
	for(uint8_t i = 0;i<ADC_PINS;i++)
	{
		this->adc_buf2[i] += ADC_BUF[i+ADC_CHAN_FPIN];
	}
	adcCount++;
	if(adcCount >= conf.maxAdcCount)
	{
		for(uint8_t i = 0;i<ADC_PINS;i++)
		{
			this->adc_buf[i] = this->adc_buf2[i]/ (conf.maxAdcCount + 1);
			this->adc_buf2[i]= 0;
		}
		adcCount = 0;
	}

}

int32_t FFBWheel::getEncValue(EncoderLocal* enc,uint16_t degrees){
	if(enc == nullptr){
			return 0x7fff; // Return center if no encoder present
	}

	float angle = 360.0 * enc->getPos_f();
	int32_t val = (0xffff / (float)degrees) * angle;
	if (conf.inverted)
		val= -val;

	enc->currentPosition = val;
	enc->currentSpeed= enc->currentPosition - enc->lastPosition;
	enc->currentAcceleration = enc->currentSpeed - enc->lastSpeed;
	enc->lastPosition = enc->currentPosition;
	enc->lastSpeed = enc->currentSpeed;
	enc->lastTorque = enc->currentTorque;
	enc->currentTorque = 0;

	return val;
}


void FFBWheel::send_report(){
	extern USBD_HandleTypeDef hUsbDeviceFS;

	// Read buttons
	reportHID.buttons = 0;
	reportHID.buttons2 = 0;
	reportHID.buttons3 = 0;

	/*uint32_t buf = 0;
	btns->readButtons(&buf, &conf);*/
	uint8_t rotaryPos[3] = {0, };
	rotaryPos[0] = i2cButtonsBuffer[2] & 0b1111;
	rotaryPos[1] = (i2cButtonsBuffer[2] >> 4) & 0b1111;
	rotaryPos[2] = i2cButtonsBuffer[3] & 0b1111;
	reportHID.buttons = i2cButtonsBuffer[0] | (i2cButtonsBuffer[1] << 8) | (i2cButtonsBuffer[4] << 16) | (((i2cButtonsBuffer[3] >> 4) & 0b1111) << 24);
	reportHID.buttons2 |= 1 <<rotaryPos[0];
	reportHID.buttons2 |= 1 << (12 + rotaryPos[1]);
	if(rotaryPos[2] < 8 )
		reportHID.buttons2 |= 1 << (24 + rotaryPos[2]);
	else
		reportHID.buttons3 |= 1 << (rotaryPos[2] - 8);

	reportHID.buttons ^= 1 << 7;
	reportHID.buttons ^= 1 << 10;


	// Encoder
	reportHID.X = clip(lastScaledEnc,-0x7fff,0x7fff);

	int16_t ry = i2cButtonsBuffer[6] | (i2cButtonsBuffer[5] << 8);
	int16_t rz = i2cButtonsBuffer[8] | (i2cButtonsBuffer[7] << 8);

	// Analog values read by DMA
	uint16_t axes = this->conf.axes;
	reportHID.Y 	=  	(axes & 0x01 << 3) ? ((adc_buf[2] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.Z		=  	(axes & 0x01 << 2) ? ((adc_buf[1] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.RX	=  	(axes & 0x01 << 1) ? ((adc_buf[0] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.RY	=	(axes & 0x01 << 4) ? ((ry & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.RZ	= 	(axes & 0x01 << 5) ? ((rz & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.Dial  =   0;
	reportHID.Slider= 	0;

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, reinterpret_cast<uint8_t*>(&reportHID), sizeof(reportHID_t));

}

void FFBWheel::timerElapsed(TIM_HandleTypeDef* htim){
	if(htim == this->timer_update){
		update_flag = true;
	}
}

void FFBWheel::cdcRcv(char* Buf, uint32_t *Len){
	if(this->parser.add(Buf, Len)){
		executeCommands(this->parser.parse());
	}
}

void FFBWheel::usbInit(){
	usbInit_HID_Wheel();
}
void FFBWheel::SOF(){
	usb_update_flag = true;
	// USB clocked update callback
}

FFBWheelConfig FFBWheel::decodeConf(){
	uint8_t len = sizeof(FFBWheelConfig);
	len = len / 4 + (len % 4 != 0 ? 1 : 0);
	uint32_t* buf = new uint32_t[len];
	FFBWheelConfig* conf;

	EE_Reads(0x00, len, buf);

	conf = (FFBWheelConfig*)buf;
	return *conf;
}

void FFBWheel::initEncoder()
{
	enc->setPpr(conf.encoderPPR);
	enc->degree = conf.degreesOfRotation;
}
