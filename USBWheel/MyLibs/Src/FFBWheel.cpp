
#include "FFBWheel.h"
#include "FFBWheel_usb_init.h"
// TODO class type for parser? (Simhub for example)
//////////////////////////////////////////////

FFBWheel::FFBWheel() {

	// Create HID FFB handler. Will receive all usb messages directly
	this->ffb = new HidFFB();


	// Setup a timer
	extern TIM_HandleTypeDef htim3;
	this->timer_update = &htim3; // Timer setup with prescaler of sysclock
	this->timer_update->Instance->ARR = 250;
	this->timer_update->Instance->CR1 = 1;
	HAL_TIM_Base_Start_IT(this->timer_update);

	restoreFlash(); // Load parameters
	ffb->set_config(&conf);
}

FFBWheel::~FFBWheel() {
	delete drv;
	delete enc;
}


void FFBWheel::restoreFlash(){

	conf = decodeConf();
	conf.check= 0x57;

	drv = new MotorBTS7960();
	enc = new EncoderLocal();
	btns = new LocalButtons();

	drv->start();
	initEncoder();
}

// Saves parameters to flash
void FFBWheel::saveFlash(){
	FFBWheelConfig savedconf = decodeConf();
	if(savedconf.isequal(conf))
		return;
	uint32_t buf[8] = {0};
	buf[0] =(uint8_t)conf.check & 0xff;
	buf[0] |= ((uint8_t)conf.axes & 0xff) << 8;
	buf[0] |= ((uint8_t)conf.I2CButtons & 0xff) << 16;
	buf[0] |= ((uint8_t)conf.nLocalButtons & 0xff) << 24;

	buf[1] = (uint16_t)conf.degreesOfRotation & 0xffff;
	buf[1] |= ((uint16_t)conf.maxpower & 0xffff) << 16;

	buf[2] = (uint8_t)conf.endstop_gain & 0xff;
	//free 8 bit
	buf[2] |= ((uint16_t)conf.encoderPPR & 0xffff) << 16;

	buf[3] = (uint8_t)conf.maxAdcCount & 0xff;
	buf[3] |= ((uint8_t)conf.inverted & 0xff) << 8;
	buf[3] |= ((uint8_t)conf.constantGain & 0xff) << 16;
	buf[3] |= ((uint8_t)conf.rampGain & 0xff) << 24;

	buf[4] = (uint8_t)conf.squareGain & 0xff;
	buf[4] |= ((uint8_t)conf.sinGain & 0xff) << 8;
	buf[4] |= ((uint8_t)conf.triangleGain & 0xff) << 16;
	buf[4] |= ((uint8_t)conf.sawToothDownGain & 0xff) << 24;

	buf[5] = (uint8_t)conf.sawToothUpGain & 0xff;
	buf[5] |= ((uint8_t)conf.springGain & 0xff) << 8;
	buf[5] |= ((uint8_t)conf.damperGain & 0xff) << 16;
	buf[5] |= ((uint8_t)conf.inertiaGain & 0xff) << 24;

	buf[6] = (uint8_t)conf.frictionGain & 0xff;
	buf[6] |= ((uint8_t)conf.totalGain & 0xff) << 8;
	buf[6] |= ((uint8_t)conf.maxVelosity & 0xff) << 16;
	buf[6] |= ((uint8_t)conf.maxAcceleration & 0xff) << 24;

	buf[7] = (uint8_t)conf.maxAcceleration & 0xff;
	buf[7] |= ((uint8_t)conf.maxPositionChange & 0xff) << 8;
	buf[7] |= ((uint16_t)conf.minForce & 0xffff) << 16;

	EE_Format();
	EE_Writes(0x00, 8, buf);
}

/*
 * Periodical update method. Called from main loop
 */
void FFBWheel::update(){
	int16_t lasttorque = endstopTorque;
	bool updateTorque = false;
	if(drv == nullptr || enc == nullptr){
		pulseSysLed();
		return;
	}

	if(usb_update_flag || update_flag){

		torque = 0;
		scaledEnc = getEncValue(enc, conf.degreesOfRotation);
		scaledEnc *= conf.inverted ? 1 : -1;

		update_flag = false;

		if(abs(scaledEnc) > 0xffff){
			drv->stop();
			pulseSysLed();
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
		torque += +endstopTorque;
		torque = clip<int32_t,int16_t>(torque, -0x7fff, 0x7fff);
		torque *= conf.inverted ? 1 : -1;
		drv->turn(torque);
	}
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
			this->adc_buf[i] = this->adc_buf2[i]/conf.maxAdcCount;
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
	uint32_t buf = 0;
	btns->readButtons(&buf);
	reportHID.buttons = buf;

	// Encoder
	reportHID.X = clip(lastScaledEnc,-0x7fff,0x7fff);
	// Analog values read by DMA
	uint16_t axes = this->conf.axes;
	reportHID.Y 	=  	(axes & 0x01 << 1) ? ((adc_buf[0] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.Z		=  	(axes & 0x01 << 2) ? ((adc_buf[1] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.RX	=  	(axes & 0x01 << 3) ? ((adc_buf[2] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.RY	=	(axes & 0x01 << 4) ? ((adc_buf[3] & 0xFFF) << 4)	-0x7fff : 0;
	reportHID.RZ	= 	(axes & 0x01 << 5) ? ((adc_buf[4] & 0xFFF) << 4)	-0x7fff : 0;
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
	uint32_t buf[8] = {0};
	FFBWheelConfig conf;

	EE_Reads(0x00, 8, buf);
	conf.check = buf[0] & 0xff;
	if(conf.check != 0x57)
	{
		conf = FFBWheelConfig();
		conf.check = 0x00;
		return conf;
	}
	conf.axes = (buf[0] >> 8)& 0xff;
	conf.I2CButtons = (buf[0] >> 16) & 0xff;
	conf.nLocalButtons= (buf[0] >> 24) & 0xff;

	conf.degreesOfRotation= buf[1] & 0xffff;
	conf.maxpower= (buf[1] >> 16) & 0xffff;

	conf.endstop_gain= buf[2] & 0xff;
	//free 8 bit
	conf.encoderPPR= (buf[2] >> 16) & 0xffff;

	conf.maxAdcCount = buf[3] & 0xff;
	conf.inverted = (buf[3] >> 8) & 0xff;
	conf.constantGain = (buf[3] >> 16) & 0xff;
	conf.rampGain = (buf[3] >> 24) & 0xff;

	conf.squareGain = buf[4] & 0xff;
	conf.sinGain = (buf[4] >> 8) & 0xff;
	conf.triangleGain = (buf[4] >> 16) & 0xff;
	conf.sawToothDownGain = (buf[4] >> 24) & 0xff;

	conf.sawToothUpGain = buf[5] & 0xff;
	conf.springGain = (buf[5] >> 8) & 0xff;
	conf.damperGain = (buf[5] >> 16) & 0xff;
	conf.inertiaGain = (buf[5] >> 24) & 0xff;

	conf.frictionGain = buf[6] & 0xff;
	conf.totalGain = (buf[6] >> 8) & 0xff;
	conf.maxVelosity = (buf[6] >> 16) & 0xff;
	conf.maxAcceleration = (buf[6] >> 24) & 0xff;

	conf.maxAcceleration = buf[7] & 0xff;
	conf.maxPositionChange = (buf[7] >> 8) & 0xff;
	conf.minForce = (buf[7] >> 16) & 0xffff;

	return conf;
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
}
