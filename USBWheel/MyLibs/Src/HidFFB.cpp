#include "HidFFB.h"
#include "math.h"
#include "FFBWheel_usb_init.h"

HidFFB::HidFFB() {
	/*damperFilter = Biquad(BiquadType::lowpass, (float)damper_f / (float)calcfrequency, damper_q, (float)0.0);
	interiaFilter = Biquad(BiquadType::lowpass, (float)friction_f / (float)calcfrequency, friction_q, (float)0.0);
	frictionFilter = Biquad(BiquadType::lowpass, (float)inertia_f / (float)calcfrequency, inertia_q, (float)0.0);
	constantFilter = Biquad(BiquadType::lowpass, (float)500/ (float)calcfrequency, cfFilter_qfloatScaler * (71), (float)0.0);*/

	this->registerHidCallback();
}

HidFFB::~HidFFB() {

}

void HidFFB::setFilterFQ(float f, float q)
{
	constantFilter.setFc(f);
	constantFilter.setQ(q);
}

void HidFFB::hidOut(uint8_t* report){
	hid_out_period = HAL_GetTick() - lastOut; // For measuring update rate
	lastOut = HAL_GetTick();
	// FFB Output Message
	uint8_t event_idx = report[0] - FFB_ID_OFFSET;

	// -------- Out Reports --------
	switch(event_idx){
		case HID_ID_NEWEFREP: //add Effect Report. Feature
			new_effect((FFB_CreateNewEffect_Feature_Data_t*)(report));
			break;
		case HID_ID_EFFREP: // Set Effect
			set_effect((FFB_SetEffect_t*)(report));
			break;
		case HID_ID_CTRLREP: // Control report. 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
			ffb_control(report[1]);
			sendStatusReport(0);
			break;
		case HID_ID_GAINREP: // Set global gain
			gain = report[1];
			break;
		case HID_ID_ENVREP: // Envelope
			set_envelope((FFB_SetEnvelope_Data_t *)report);
			break;
		case HID_ID_CONDREP: // Spring, Damper, Friction, Inertia
			set_condition((FFB_SetCondition_Data_t*)report);
			break;
		case HID_ID_PRIDREP: // Periodic
			set_periodic((FFB_SetPeriodic_Data_t*)report);
			break;
		case HID_ID_CONSTREP: // Constant
			set_constant_effect((FFB_SetConstantForce_Data_t*)report);
			break;
		case HID_ID_RAMPREP: // Ramp
			set_ramp((FFB_SetRamp_Data_t *)report);
			break;
		case HID_ID_CSTMREP: // Custom. pretty much never used
			break;
		case HID_ID_SMPLREP: // Download sample
			break;
		case HID_ID_EFOPREP: //Effect operation
		{
			// Start or stop effect
			uint8_t id = report[1]-1;
			if(report[2] == 3){
				effects[id].state = 0; //Stop
			}else{
				effects[id].startTime = HAL_GetTick() + effects[id].startDelay; // + effects[id].startDelay;
				effects[id].state = 1; //Start
			}
			break;
		}
		case HID_ID_BLKFRREP: // Free a block
		{
			free_effect(report[1]-1);
			break;
		}

		default:
			break;
		}

}

void HidFFB::hidGet(uint8_t id,uint16_t len,uint8_t** return_buf){
	// Feature gets go here

	id = id - FFB_ID_OFFSET;

	switch(id){
	case HID_ID_BLKLDREP:
		*return_buf = (uint8_t*)(&this->blockLoad_report);
		break;
	case HID_ID_POOLREP:
		*return_buf = (uint8_t*)(&this->pool_report);
		break;
	}
}

void HidFFB::sendStatusReport(uint8_t effect){
	extern USBD_HandleTypeDef hUsbDeviceFS;

	this->reportFFBStatus.effectBlockIndex = effect;
	this->reportFFBStatus.status = HID_ACTUATOR_POWER;
	if(this->ffb_active){
		this->reportFFBStatus.status |= HID_ENABLE_ACTUATORS;
		this->reportFFBStatus.status |= HID_EFFECT_PLAYING;
	}else{
		this->reportFFBStatus.status |= HID_EFFECT_PAUSE;
	}
	if(effect > 0 && effects[effect-1].state == 1)
		this->reportFFBStatus.status |= HID_EFFECT_PLAYING;

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, reinterpret_cast<uint8_t*>(&this->reportFFBStatus), sizeof(reportFFB_status_t));
}

void HidFFB::ffb_control(uint8_t cmd)
{
	if(cmd & 0x01){ //enable
		start_FFB();
	}if(cmd & 0x02){ //disable
		stop_FFB();
	}if(cmd & 0x04){ //stop TODO Some games send wrong commands?
		stop_FFB();
		//start_FFB();
	}if(cmd & 0x08){ //reset
		//ffb_active = true;
		stop_FFB();
		reset_ffb();
		// reset effects
	}if(cmd & 0x10){ //pause
		stop_FFB();
	}if(cmd & 0x20){ //continue
		start_FFB();
	}
}

void HidFFB::new_effect(FFB_CreateNewEffect_Feature_Data_t* effect)
{
	// Allocates a new effect

	uint8_t index = find_free_effect(effect->effectType); // next effect
	if(index == 0){
		blockLoad_report.loadStatus = 2;
		return;
	}
	//CommandHandler::logSerial("Creating Effect: " + std::to_string(effect->effectType) +  " at " + std::to_string(index) + "\n");
	FFB_Effect new_effect;
	new_effect.type = effect->effectType;

	effects[index-1] = std::move(new_effect);
	// Set block load report
	reportFFBStatus.effectBlockIndex = index;
	blockLoad_report.effectBlockIndex = index;
	used_effects++;
	blockLoad_report.ramPoolAvailable = MAX_EFFECTS-used_effects;
	blockLoad_report.loadStatus = 1;
}

uint8_t HidFFB::find_free_effect(uint8_t type)
{
	for(uint8_t i=0;i<MAX_EFFECTS;i++){
		if(effects[i].type == FFB_EFFECT_NONE){
			return(i+1);
		}
	}
	return 0;
}

void HidFFB::set_effect(FFB_SetEffect_t* effect)
{
	uint8_t index = effect->effectBlockIndex;
	if(index > MAX_EFFECTS || index == 0)
		return;

	FFB_Effect* effect_p = &effects[index-1];

	if (effect_p->type != effect->effectType){
		effect_p->startTime = 0;
	}

	effect_p->gain = effect->gain;
	effect_p->type = effect->effectType;
	effect_p->samplePeriod = effect->samplePeriod;

	effect_p->enableAxis = effect->enableAxis;
	effect_p->directionX = effect->directionX;
	effect_p->directionY = effect->directionY;

	effect_p->duration = effect->duration;
	if(!ffb_active)
		start_FFB();
}

void HidFFB::set_envelope(FFB_SetEnvelope_Data_t *report)
{
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];

	effect->attackLevel = report->attackLevel;
	effect->attackTime = report->attackTime;
	effect->fadeLevel = report->fadeLevel;
	effect->fadeTime = report->fadeTime;
	effect->useEnvelope = true;
}

void HidFFB::set_condition(FFB_SetCondition_Data_t *cond)
{
	uint8_t axis = cond->parameterBlockOffset;
	if (axis >= MAX_AXIS){
		return; // sanity check!
	}
	FFB_Effect *effect = &effects[cond->effectBlockIndex - 1];
	effect->conditions[axis].cpOffset = cond->cpOffset;
	effect->conditions[axis].negativeCoefficient = cond->negativeCoefficient;
	effect->conditions[axis].positiveCoefficient = cond->positiveCoefficient;
	effect->conditions[axis].negativeSaturation = cond->negativeSaturation;
	effect->conditions[axis].positiveSaturation = cond->positiveSaturation;
	effect->conditions[axis].deadBand = cond->deadBand;
	effect->conditionsCount++;
	if(effect->conditions[axis].positiveSaturation == 0){
		effect->conditions[axis].positiveSaturation = 0x7FFF;
	}
	if(effect->conditions[axis].negativeSaturation == 0){
		effect->conditions[axis].negativeSaturation = 0x7FFF;
	}
}

void HidFFB::set_periodic(FFB_SetPeriodic_Data_t* report)
{
	FFB_Effect* effect = &effects[report->effectBlockIndex-1];

	effect->period = clip<uint32_t,uint32_t>(report->period,1,0x7fff); // Period is never 0
	effect->magnitude = report->magnitude;
	effect->offset = report->offset;
	effect->phase = report->phase;
	//effect->counter = 0;
}

void HidFFB::set_constant_effect(FFB_SetConstantForce_Data_t* effect){
	effects[effect->effectBlockIndex-1].magnitude = effect->magnitude;
}

void HidFFB::set_ramp(FFB_SetRamp_Data_t *report)
{
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];
	effect->magnitude = 0x7fff; // Full magnitude for envelope calculation. This effect does not have a periodic report
	effect->startLevel = report->startLevel;
	effect->endLevel = report->endLevel;
}

void HidFFB::start_FFB(){
	ffb_active = true;


}
void HidFFB::stop_FFB(){
	ffb_active = false;
}

void HidFFB::reset_ffb(){
	for(uint8_t i=0;i<MAX_EFFECTS;i++){
		free_effect(i);
	}
	this->reportFFBStatus.effectBlockIndex = 1;
	this->reportFFBStatus.status = (HID_ACTUATOR_POWER) | (HID_ENABLE_ACTUATORS);
	used_effects = 0;
}

void HidFFB::free_effect(uint16_t idx){
	if(idx < MAX_EFFECTS){
		effects[idx].type=FFB_EFFECT_NONE;
	}
}

int32_t HidFFB::updateIdleSpringForce(EncoderLocal* enc)
{
	float idlespringscale = clip<int32_t, int32_t>((int32_t)conf->idleSpring*50, 0, 10000);
	int16_t idlespringclip = 0.5f + ((float)conf->idleSpring * 0.01f);
	return clip<int32_t,int32_t>((int32_t)(-enc->currentPosition * idlespringscale), -idlespringclip, idlespringclip);
}

int32_t HidFFB::calculateEffects(EncoderLocal* encoder){
	int32_t idleForce = 0;
	if(!ffb_active){
		idleForce += updateIdleSpringForce();
	}

	if(damperIntensity != 0){
		float speedFiltered = damperFilter.process(encoder->currentSpeed) * (float)conf->idleDamper * 1.5;
		idleForce -= clip<float, int32_t>(speedFiltered, -10000, 10000);
	}

	int32_t forceX = 0;
	int32_t forceVector = 0;

	for(uint8_t i = 0;i<MAX_EFFECTS;i++){
		FFB_Effect* effect = &effects[i];

		if(effect->state == 0)
			continue;

		if (effect->conditionsCount == 0) {
			forceVector = calcNonConditionEffectForce(effect);
		}

		forceX += calcComponentForce(effect, forceVector, encoder);
		forceX = clip<int32_t, int32_t>(forceX, -0x7fff, 0x7fff);

	}
	return forceX;
}

int32_t HidFFB::calcNonConditionEffectForce(FFB_Effect *effect)
{
	int32_t force_vector = 0;
	switch (effect->type){

	case FFB_EFFECT_CONSTANT:
	{ // Constant force is just the force
		force_vector = ((int32_t)effect->magnitude * (int32_t)(1 + effect->gain)) >> 8;
		// Optional filtering to reduce spikes
		/*if (conf->cfFilter_f < calcfrequency / 2)
		{
			force_vector = constantFilter.process(force_vector);
		}*/
		break;
	}

	case FFB_EFFECT_RAMP:
	{
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		int32_t duration = effect->duration;
		float force = (int32_t)effect->startLevel + ((int32_t)elapsed_time * (effect->endLevel - effect->startLevel)) / duration;
		force_vector = (int32_t)(force * (1 + effect->gain)) >> 8;
		break;
	}

	case FFB_EFFECT_SQUARE:
	{
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		int32_t force = ((elapsed_time + effect->phase) % ((uint32_t)effect->period + 2)) < (uint32_t)(effect->period + 2) / 2 ? -effect->magnitude : effect->magnitude;
		force_vector = force + effect->offset;
		break;
	}

	case FFB_EFFECT_TRIANGLE:
	{
		int32_t force = 0;
		int32_t offset = effect->offset;
		int32_t magnitude = effect->magnitude;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		float periodF = period;

		int32_t maxMagnitude = offset + magnitude;
		int32_t minMagnitude = offset - magnitude;
		uint32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		float remainder = timeTemp % period;
		float slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
		if (remainder > (periodF / 2))
			force = slope * (periodF - remainder);
		else
			force = slope * remainder;
		force += minMagnitude;
		force_vector = force;
		break;
	}

	case FFB_EFFECT_SAWTOOTHUP:
	{
		float offset = effect->offset;
		float magnitude = effect->magnitude;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		uint32_t phase = effect->phase;
		uint32_t period = effect->period;
		float periodF = effect->period;

		float maxMagnitude = offset + magnitude;
		float minMagnitude = offset - magnitude;
		int32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		float remainder = timeTemp % period;
		float slope = (maxMagnitude - minMagnitude) / periodF;
		force_vector = (int32_t)(minMagnitude + slope * (period - remainder));
		break;
	}

	case FFB_EFFECT_SAWTOOTHDOWN:
	{
		float offset = effect->offset;
		float magnitude = effect->magnitude;
		uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
		float phase = effect->phase;
		uint32_t period = effect->period;
		float periodF = effect->period;

		float maxMagnitude = offset + magnitude;
		float minMagnitude = offset - magnitude;
		int32_t phasetime = (phase * period) / 35999;
		uint32_t timeTemp = elapsed_time + phasetime;
		float remainder = timeTemp % period;
		float slope = (maxMagnitude - minMagnitude) / periodF;
		force_vector = (int32_t)(minMagnitude + slope * (remainder)); // reverse time
		break;
	}

	case FFB_EFFECT_SINE:
	{
		uint32_t t = HAL_GetTick() - effect->startTime;
		float freq = 1.0f / (float)(std::max<uint16_t>(effect->period, 2));
		float phase = (float)effect->phase / (float)35999; //degrees
		float sine = sinf(2.0 * (float)M_PI * (t * freq + phase)) * effect->magnitude;
		force_vector = (int32_t)(effect->offset + sine);
		break;
	}
	default:
		break;
	}
	if(effect->useEnvelope) {
		force_vector = applyEnvelope(effect, (int32_t)force_vector);
	}
	return force_vector;
}

int32_t HidFFB::applyEnvelope(FFB_Effect *effect, int32_t value)
{
	int32_t magnitude = (effect->magnitude);
	int32_t attackLevel = (effect->attackLevel);
	int32_t fadeLevel = (effect->fadeLevel);
	int32_t newValue = magnitude;
	uint32_t elapsed_time = HAL_GetTick() - effect->startTime;
	if (elapsed_time < effect->attackTime)
	{
		newValue = (magnitude - attackLevel) * elapsed_time;
		newValue /= (int32_t)effect->attackTime;
		newValue += attackLevel;
	}
	if (effect->duration != 0xffff &&
		elapsed_time > (effect->duration - effect->fadeTime))
	{
		newValue = (magnitude - fadeLevel) * (effect->duration - elapsed_time);
		newValue /= (int32_t)effect->fadeTime;
		newValue += fadeLevel;
	}

	newValue *= value;
	newValue /= 0x7fff; // 16 bit
	return newValue;
}

int32_t HidFFB::calcComponentForce(FFB_Effect *effect, int32_t forceVector, EncoderLocal* encoder)
{
	int32_t result_torque = 0;
	uint16_t direction;
	uint8_t axis = 0;
	uint8_t axisCount = 1;
	uint8_t con_idx = 0; // condition block index

	if (effect->enableAxis == DIRECTION_ENABLE)
	{
		direction = effect->directionX;
		if (effect->conditionsCount > 1)
		{
			con_idx = axis;
		}
	}
	else
	{
		direction = axis == 0 ? effect->directionX : effect->directionY;
		con_idx = axis;
	}

	//bool useForceDirectionForConditionEffect = (effect->enableAxis == DIRECTION_ENABLE && axisCount > 1 && effect->conditionsCount == 1);
	bool rotateConditionForce = (axisCount > 1 && effect->conditionsCount < axisCount);
	float angle = ((float)direction * (2*M_PI) / 36000.0);
	float angle_ratio = axis == 0 ? sin(angle) : -1 * cos(angle);
	angle_ratio = rotateConditionForce ? angle_ratio : 1.0;

	switch (effect->type)
	{
	case FFB_EFFECT_CONSTANT:
	case FFB_EFFECT_RAMP:
	case FFB_EFFECT_SQUARE:
	case FFB_EFFECT_TRIANGLE:
	case FFB_EFFECT_SAWTOOTHUP:
	case FFB_EFFECT_SAWTOOTHDOWN:
	case FFB_EFFECT_SINE:
	{
		result_torque = -forceVector * angle_ratio;
		break;
	}
	case FFB_EFFECT_SPRING:
	{
		int32_t pos = encoder->currentPosition;
		int16_t offset = effect->conditions[con_idx].cpOffset;
		int16_t deadBand = effect->conditions[con_idx].deadBand;
		// Spring effect must also use deadband and offset so that it begins gradually from the sides of the deadband
		float metric = encoder->currentPosition - (offset + (deadBand * (pos < offset ? -1 : 1)) );
		result_torque -= calcConditionEffectForce(effect, metric, conf->springGain, pos,
									   con_idx, 0.0004f, angle_ratio);
		break;
	}
	case FFB_EFFECT_FRICTION:
	{
		float metric = /*frictionFilter.process(encoder->currentSpeed)*/encoder->currentSpeed * .25;
		result_torque -= calcConditionEffectForce(effect, metric, conf->frictionGain, encoder->currentPosition,
											   con_idx, .08f, angle_ratio);
		break;
	}
	case FFB_EFFECT_DAMPER:
	{
		float metric = /*damperFilter.process(encoder->currentSpeed)*/ encoder->currentSpeed * .0625f;
		result_torque -= calcConditionEffectForce(effect, metric,  conf->damperGain, encoder->currentPosition,
									   con_idx, 0.6f, angle_ratio);
		break;
	}
	case FFB_EFFECT_INERTIA:
	{
		float metric = /*interiaFilter.process(encoder->currentAcceleration*4)*/ encoder->currentAcceleration*4;
		result_torque -= calcConditionEffectForce(effect, metric,conf->inertiaGain, encoder->currentPosition,
									   con_idx, 0.5f, angle_ratio);
		break;
	}

	default:
		// Unsupported effect
		break;
	}
	return (result_torque * (gain+1)) >> 8; // Apply global gain
}

int32_t HidFFB::calcConditionEffectForce(FFB_Effect *effect, float  metric, uint8_t gain,int32_t pos,
										 uint8_t idx, float scale, float angle_ratio)
{
	int16_t offset = effect->conditions[idx].cpOffset;
	int16_t deadBand = effect->conditions[idx].deadBand;
	int32_t force = 0;
	// Effect is only active outside deadband + offset
	if (abs(pos - offset) > deadBand){
		force = clip<int32_t, int32_t>(((float)effect->conditions[idx].negativeCoefficient *
											scale * (float)(metric)),
										   -effect->conditions[idx].negativeSaturation,
										   effect->conditions[idx].positiveSaturation);
	}

	force = ((gain+1) * force) >> 8;
	return force * angle_ratio;
}

void HidFFB::set_config(FFBWheelConfig *conf)
{
	this->conf = conf;
}
