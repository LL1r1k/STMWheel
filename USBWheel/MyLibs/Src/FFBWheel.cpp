
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
	setCfFilter(conf.cfFilter_f, conf.cfFilter_q);

	pi2cBuf = (uint8_t*)&i2cBuffer;

	// Setup a timer
	extern TIM_HandleTypeDef htim3;
	this->timer_update = &htim3; // Timer setup with prescaler of sysclock
	this->timer_update->Instance->ARR = 250;
	this->timer_update->Instance->CR1 = 1;
	HAL_TIM_Base_Start_IT(this->timer_update);
}

FFBWheel::~FFBWheel() {
	delete drv;
	delete enc;
}

void FFBWheel::setCfFilter(uint32_t freq, uint8_t q){
	conf.cfFilter_q = clip<uint8_t, uint8_t>(q,0,127);

	if(freq == 0)
		freq = 500;
	conf.cfFilter_f = freq;
	float f = (float)conf.cfFilter_f / (float)1000;

	ffb->setFilterFQ(f, (float)0.01 * (conf.cfFilter_q+1));
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
	if(savedconf.isequal(conf))
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
		speed = scaledEnc - lastScaledEnc;
		lastScaledEnc = scaledEnc;

		usb_update_flag = false;
		torque = ffb->calculateEffects(enc);

		if(endstopTorque == 0 || (endstopTorque > 0 && torque > 0) || (endstopTorque < 0 && torque < 0))
		{
			torque *= ((float)this->conf.totalGain / (float)100.00);
			updateTorque = true;
		}
		this->send_report();
	}

	if(endstopTorque!=lasttorque || updateTorque){
		torque = clip<int32_t,int16_t>(torque, -this->conf.maxpower, this->conf.maxpower);
		if(torque > 0) torque = map(torque, 0, this->conf.maxpower, MIN(this->conf.minForce, this->conf.maxpower), this->conf.maxpower);
		else if (torque < 0) torque = map(torque, 0, -this->conf.maxpower, MAX(-this->conf.minForce, -this->conf.maxpower), -this->conf.maxpower);
		torque += endstopTorque;
		torque = clip<int32_t,int16_t>(torque, -0x7fff, 0x7fff);
		if(conf.inverted == false)
			torque *=-1;
		drv->turn(torque);
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

	enc->currentPosition = enc->getPos();
	if(conf.inverted == true)
		enc->currentPosition *=-1;
	enc->positionChange = enc->currentPosition - enc->lastPosition;
	uint32_t currentEncoderTime = (int32_t) HAL_GetTick();
	int16_t diffTime = (int16_t)(currentEncoderTime -  enc->lastEncoderTime) ;
	if (diffTime > 0) {
		enc->currentVelocity = enc->positionChange / diffTime;
		enc->currentAcceleration = (abs(enc->currentVelocity) - abs(enc->lastVelocity)) / diffTime;
		enc->lastEncoderTime = currentEncoderTime;
		enc->lastVelocity = enc->currentVelocity;
	}
	enc->lastPosition = enc->currentPosition;

	float angle = 360.0*((float)enc->currentPosition/(float)enc->getPosCpr());
	int32_t val = (0xffff / (float)degrees) * angle;
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
	reportHID.Slider= 	(axes & 0x01 << 6) ? ((adc_buf[5] & 0xFFF) << 4)	-0x7fff : 0;

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
	enc->maxAngle = conf.degreesOfRotation;
	enc->maxValue = (float)enc->maxAngle / 2 / 360 * enc->ppr;
	enc->minValue = -enc->maxValue;
	enc->currentPosition = 0;
	enc->lastPosition = 0;
	enc->correctPosition = 0;
	enc->lastEncoderTime = (uint32_t)HAL_GetTick();
	enc->lastVelocity = 0;
	enc->maxVelocity = conf.maxVelosity;
	enc->maxAcceleration = conf.maxAcceleration;
	enc->maxPositionChange = conf.maxPositionChange;
}
