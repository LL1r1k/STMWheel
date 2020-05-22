#include "HidFFB.h"
#include "math.h"

HidFFB::HidFFB() {
	damperFilter = Filters(cutoff_freq_damper, sampling_time_damper, ORDER::OD1);
	interiaFilter = Filters(cutoff_freq_damper, sampling_time_damper, ORDER::OD1);
	frictionFilter = Filters(cutoff_freq_damper, sampling_time_damper, ORDER::OD1);

	this->registerHidCallback();
}

HidFFB::~HidFFB() {

}


void HidFFB::hidOut(uint8_t* report){
	hid_out_period = HAL_GetTick() - lastOut; // For measuring update rate
	lastOut = HAL_GetTick();
	// FFB Output Message
	report[0] -= FFB_ID_OFFSET;// if offset id was set correct this
	uint8_t event_idx = report[0];


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
		break;
	case HID_ID_GAINREP:
		gain = report[1];
		break;
	case HID_ID_ENVREP:
		set_envelope((FFB_SetEnvelope_Data_t*)(report));
		break;
	case HID_ID_CONDREP: // Condition
		set_condition((FFB_SetCondition_Data_t*)report);
		break;
	case HID_ID_PRIDREP: // Periodic
		set_periodic((FFB_SetPeriodic_Data_t*)report);
		break;
	case HID_ID_CONSTREP: // Constant
		set_constant_effect((FFB_SetConstantForce_Data_t*)report);
		break;
	case HID_ID_RAMPREP: // Ramp
		set_ramp_effect((FFB_SetRampForce_Data_t*)report);
		break;
	case HID_ID_CSTMREP: // Custom
		//TODO
		break;
	case HID_ID_SMPLREP: // Download sample
		//TODO
		break;
	case HID_ID_EFOPREP: //Effect operation
	{
		// Start or stop effect
		uint8_t id = report[1]-1;
		if(report[2] == 3){
			effects[id].state = 0; //Stop
		}else{
			effects[id].state = 1; //Start
			effects[id].counter = 0; // When an effect was stopped reset all parameters that could cause jerking
			effects[id].elapsedTime = 0;
			effects[id].startTime = HAL_GetTick();
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

void HidFFB::free_effect(uint16_t idx){
	if(idx < MAX_EFFECTS)
		effects[idx].type=FFB_EFFECT_NONE;
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

void HidFFB::start_FFB(){
	ffb_active = true;


}
void HidFFB::stop_FFB(){
	ffb_active = false;


	//TODO Callbacks?
}

void HidFFB::ffb_control(uint8_t cmd){
	if(cmd & 0x01){ //enable
		start_FFB();
	}if(cmd & 0x02){ //disable
		ffb_active = false;
	}if(cmd & 0x04){ //stop TODO Some games send wrong commands?
		stop_FFB();
		//start_FFB();
	}if(cmd & 0x08){ //reset
		//ffb_active = true;
		stop_FFB();
		reset_ffb();
		// reset effects
	}if(cmd & 0x10){ //pause
		ffb_active = false;
	}if(cmd & 0x20){ //continue
		ffb_active = true;
	}
	Bchg(this->reportFFBStatus.status,HID_ENABLE_ACTUATORS & ffb_active);
}


void HidFFB::set_constant_effect(FFB_SetConstantForce_Data_t* effect){
	effects[effect->effectBlockIndex-1].magnitude = effect->magnitude;
}

void HidFFB::new_effect(FFB_CreateNewEffect_Feature_Data_t* effect){
	// Allocates a new effect

	uint8_t index = find_free_effect(effect->effectType); // next effect
	if(index == 0){
		blockLoad_report.loadStatus = 2;
		return;
	}

	FFB_Effect new_effect;
	new_effect.type = effect->effectType;

	effects[index-1] = new_effect;
	// Set block load report
	reportFFBStatus.effectBlockIndex = index;
	blockLoad_report.effectBlockIndex = index;
	used_effects++;
	blockLoad_report.ramPoolAvailable = MAX_EFFECTS-used_effects;
	blockLoad_report.loadStatus = 1;


}
void HidFFB::set_effect(FFB_SetEffect_t* effect){
	uint8_t index = effect->effectBlockIndex;
	if(index > MAX_EFFECTS || index == 0)
		return;

	FFB_Effect* effect_p = &effects[index-1];
	effect_p->gain = effect->gain;
	effect_p->type = effect->effectType;
	effect_p->samplePeriod = effect->samplePeriod;
	if(effect->enableAxis & 0x4){
		// All axes
		effect_p->axis = 0x7;
	}else{
		effect_p->axis = effect->enableAxis;
	}
	if(effect_p->type != effect->effectType){
		effect_p->counter = 0;
		effect_p->last_value = 0;
	}

	effect_p->duration = effect->duration;
	effect_p->directionX = effect->directionX;
	effect_p->directionY = effect->directionY;

	if(!ffb_active)
		start_FFB();
}

void HidFFB::set_envelope(FFB_SetEnvelope_Data_t *envelop)
{
	FFB_Effect* effect = &effects[envelop->effectBlockIndex-1];

	effect->attackLevel = envelop->attackLevel;
	effect->fadeLevel = envelop->fadeLevel;
	effect->attackTime = envelop->attackTime;
	effect->fadeTime = envelop->fadeTime;
}

void HidFFB::set_ramp_effect(FFB_SetRampForce_Data_t *effect)
{
	FFB_Effect* effect_p = &effects[effect->effectBlockIndex-1];

	effect_p->startMagnitude = effect->startMagnitude;
	effect_p->endMagnitude = effect->endMagnitude;
}


void HidFFB::set_condition(FFB_SetCondition_Data_t* cond){
	if(cond->parameterBlockOffset != 0) //TODO if more axes are needed. Only X Axis is implemented now for the wheel.
		return;

	FFB_Effect* effect = &effects[cond->effectBlockIndex-1];

	effect->offset = cond->cpOffset;
	effect->negativeCoefficient  = cond->negativeCoefficient;
	effect->positiveCoefficient = cond->positiveCoefficient;
	effect->negativeSaturation = cond->negativeSaturation;
	effect->positiveSaturation = cond->positiveSaturation;
	effect->deadBand = cond->deadBand;
}

void HidFFB::set_periodic(FFB_SetPeriodic_Data_t* report){
	FFB_Effect* effect = &effects[report->effectBlockIndex-1];

	effect->period = report->period;
	effect->magnitude = report->magnitude;
	effect->offset = report->offset;
	effect->phase = report->phase;
	//effect->counter = 0;
}

uint8_t HidFFB::find_free_effect(uint8_t type){ //Will return the first effect index which is empty or the same type
	for(uint8_t i=0;i<MAX_EFFECTS;i++){
		if(effects[i].type == FFB_EFFECT_NONE){
			return(i+1);
		}
	}
	return 0;
}



void HidFFB::reset_ffb(){
	for(uint8_t i=0;i<MAX_EFFECTS;i++){
		free_effect(i);
	}
	this->reportFFBStatus.effectBlockIndex = 1;
	this->reportFFBStatus.status = (HID_ACTUATOR_POWER) | (HID_ENABLE_ACTUATORS);
	used_effects = 0;
}

int32_t HidFFB::ConstantForceCalculator(FFB_Effect *effect)
{
	float tempforce = (float)effect->magnitude * effect->gain / 255;
	return (int32_t)tempforce;
}

int32_t HidFFB::RampForceCalculator(FFB_Effect* effect)
{
	int32_t rampForce = effect->startMagnitude + effect->elapsedTime * (effect->endMagnitude - effect->startMagnitude) / effect->duration;
	return rampForce;
}

int32_t HidFFB::calculateEffects(EncoderLocal* encoder){
	if(!ffb_active){
		if(idlecenter){
			return clip<int32_t,int32_t>(-encoder->currentPosition,-5000,5000);
		}else{
			return 0;
		}
	}

	int32_t result_torque = 0;

	for(uint8_t i = 0;i<MAX_EFFECTS;i++){
		FFB_Effect* effect = &effects[i];
		// Filter out inactive effects
		if(effect->state == 0 || !(0x01 & effect->axis))
			continue;

		switch(effect->type){
		case FFB_EFFECT_CONSTANT:
			result_torque -= ConstantForceCalculator(effect) * (((float)conf->constantGain * 50.0) / 255.0);
			break;
	    case FFB_EFFECT_RAMP:
	    	result_torque -= RampForceCalculator(effect) * (((float)conf->rampGain * 50.0) / 255.0);
	        break;
		case FFB_EFFECT_SPRING:
			result_torque -= ConditionForceCalculator(effect, NormalizeRange(encoder->currentPosition, encoder->maxValue)) * (((float)conf->springGain * 50.0) / 255.0);
			break;
		case FFB_EFFECT_SQUARE:
			result_torque -= SquareForceCalculator(effect) * (((float)conf->squareGain * 50.0) / 255.0);
			break;
		case FFB_EFFECT_SINE:
			result_torque -= SinForceCalculator(effect) * (((float)conf->sinGain * 50.0) / 255.0);
			break;
        case FFB_EFFECT_TRIANGLE:
        	result_torque -= TriangleForceCalculator(effect) * (((float)conf->triangleGain * 50.0) / 255.0);
        	break;
        case FFB_EFFECT_SAWTOOTHDOWN:
        	result_torque -= SawtoothDownForceCalculator(effect) * (((float)conf->sawToothDownGain * 50.0) / 255.0);
        	break;
        case FFB_EFFECT_SAWTOOTHUP:
        	result_torque -= SawtoothUpForceCalculator(effect) * (((float)conf->sawToothUpGain * 50.0) / 255.0);
          	break;
		case FFB_EFFECT_DAMPER:
			result_torque -= ConditionForceCalculator(effect, NormalizeRange(encoder->currentVelocity, encoder->maxVelocity)) * (((float)conf->damperGain * 50.0) / 255.0);
		    break;
		case FFB_EFFECT_INERTIA:
	        if ( encoder->currentAcceleration < 0 and encoder->positionChange < 0) {
	        	result_torque -= ConditionForceCalculator(effect, abs(NormalizeRange(encoder->currentAcceleration, encoder->maxAcceleration))) * (((float)conf->inertiaGain * 50.0) / 255.0);
	        } else if ( encoder->currentAcceleration < 0 and encoder->positionChange > 0) {
	        	result_torque += ConditionForceCalculator(effect, abs(NormalizeRange(encoder->currentAcceleration, encoder->maxAcceleration))) * (((float)conf->inertiaGain * 50.0) / 255.0);
	        }
			break;
		case FFB_EFFECT_FRICTION:
			result_torque -= ConditionForceCalculator(effect, NormalizeRange(encoder->positionChange, encoder->maxPositionChange)) * (((float)conf->frictionGain * 50.0) / 255.0);
			break;
		default:
			break;
		}
		effect->elapsedTime = (uint64_t)HAL_GetTick() - effect->startTime;
		if(effect->counter++ > effect->duration){
			effect->state = 0;
		}
		result_torque =  clip(result_torque, -0x7fff, 0x7fff);
	}
	result_torque = (result_torque * (gain+1)) >> 8; // Apply global gain
	return clip(result_torque, -0x7fff, 0x7fff);
}

int32_t HidFFB::SquareForceCalculator(FFB_Effect *effect)
{
	 int32_t offset = effect->offset * 2;
	  uint32_t magnitude = effect->magnitude;
	  uint32_t elapsedTime = effect->elapsedTime;
	  uint32_t phase = effect->phase;
	  uint32_t period = effect->period;

	  int32_t maxMagnitude = offset + magnitude;
	  int32_t minMagnitude = offset - magnitude;
	  uint32_t phasetime = (phase * period) / 255;
	  uint32_t timeTemp = elapsedTime + phasetime;
	  uint32_t reminder = timeTemp % period;
	  int32_t tempforce;
	  if (reminder > (period / 2)) tempforce = minMagnitude;
	  else tempforce = maxMagnitude;
	  return ApplyEnvelope(effect, tempforce);
}

int32_t HidFFB::SinForceCalculator(FFB_Effect *effect)
{
	float offset = effect->offset * 2;
	float magnitude = effect->magnitude;
	float phase = effect->phase;
	float timeTemp = effect->elapsedTime;
	float period = effect->period;
	float angle = ((timeTemp / period) * 2 * (float)3.14159265359 + (float)(phase / 36000));
	float sine = sin(angle);
	float tempforce = sine * magnitude;
	tempforce += offset;
	return ApplyEnvelope(effect, tempforce);
}

int32_t HidFFB::TriangleForceCalculator(FFB_Effect *effect)
{
	  float offset = effect->offset * 2;
	  float magnitude = effect->magnitude;
	  float elapsedTime = effect->elapsedTime;
	  uint32_t phase = effect->phase;
	  uint32_t period = effect->period;
	  float periodF = effect->period;

	  float maxMagnitude = offset + magnitude;
	  float minMagnitude = offset - magnitude;
	  uint32_t phasetime = (phase * period) / 255;
	  uint32_t timeTemp = elapsedTime + phasetime;
	  float reminder = timeTemp % period;
	  float slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
	  float tempforce = 0;
	  if (reminder > (periodF / 2)) tempforce = slope * (periodF - reminder);
	  else tempforce = slope * reminder;
	  tempforce += minMagnitude;
	  return ApplyEnvelope(effect, tempforce);
}

int32_t HidFFB::SawtoothDownForceCalculator(FFB_Effect *effect)
{
	  float offset = effect->offset * 2;
	  float magnitude = effect->magnitude;
	  float elapsedTime = effect->elapsedTime;
	  float phase = effect->phase;
	  uint32_t period = effect->period;
	  float periodF = effect->period;

	  float maxMagnitude = offset + magnitude;
	  float minMagnitude = offset - magnitude;
	  int32_t phasetime = (phase * period) / 255;
	  uint32_t timeTemp = elapsedTime + phasetime;
	  float reminder = timeTemp % period;
	  float slope = (maxMagnitude - minMagnitude) / periodF;
	  float tempforce = 0;
	  tempforce = slope * (period - reminder);
	  tempforce += minMagnitude;
	  return ApplyEnvelope(effect, tempforce);
}

int32_t HidFFB::SawtoothUpForceCalculator(FFB_Effect *effect)
{
	  float offset = effect->offset * 2;
	  float magnitude = effect->magnitude;
	  float elapsedTime = effect->elapsedTime;
	  uint32_t phase = effect->phase;
	  uint32_t period = effect->period;
	  float periodF = effect->period;

	  float maxMagnitude = offset + magnitude;
	  float minMagnitude = offset - magnitude;
	  int32_t phasetime = (phase * period) / 255;
	  uint32_t timeTemp = elapsedTime + phasetime;
	  float reminder = timeTemp % period;
	  float slope = (maxMagnitude - minMagnitude) / periodF;
	  float tempforce = 0;
	  tempforce = slope * reminder;
	  tempforce += minMagnitude;
	  return ApplyEnvelope(effect, tempforce);
}

int32_t HidFFB::ConditionForceCalculator(FFB_Effect *effect, float metric)
{
	  float deadBand = effect->deadBand;
	  float cpOffset = effect->cpOffset;
	  float negativeCoefficient = -effect->negativeCoefficient;
	  float positiveSaturation = effect->positiveSaturation;
	  float positiveCoefficient = effect->positiveCoefficient;
	  float  tempForce = 0;
	  if (metric < (cpOffset - deadBand)) {
	    tempForce = ((float)1.00 * (cpOffset - deadBand) / 10000 - metric) * negativeCoefficient;
	  }
	  else if (metric > (cpOffset + deadBand)) {
	    tempForce = (metric - (float)1.00 * (cpOffset + deadBand) / 10000) * positiveCoefficient;
	    tempForce = (tempForce > positiveSaturation ? positiveSaturation : tempForce);
	  }
	  tempForce = tempForce * effect->gain / 255;
	  switch (effect->type) {
	    case FFB_EFFECT_DAMPER:
	      tempForce = damperFilter.filterIn(tempForce);
	      break;
	    case FFB_EFFECT_INERTIA:
	      tempForce = interiaFilter.filterIn(tempForce);
	      break;
	    case FFB_EFFECT_FRICTION:
	      tempForce = frictionFilter.filterIn(tempForce);
	      break;
	    default:
	      break;
	  }

	  return (int32_t) tempForce;
}

int32_t HidFFB::ApplyGain(uint32_t value, uint8_t gain)
{
	  return ((value * gain) / 255);
}

int32_t HidFFB::ApplyEnvelope(FFB_Effect* effect, int32_t value)
{
	  int32_t magnitude = ApplyGain(effect->magnitude, effect->gain);
	  int32_t attackLevel = ApplyGain(effect->attackLevel, effect->gain);
	  int32_t fadeLevel = ApplyGain(effect->fadeLevel, effect->gain);
	  int32_t newValue = magnitude;
	  int32_t attackTime = effect->attackTime;
	  int32_t fadeTime = effect->fadeTime;
	  int32_t elapsedTime = effect->elapsedTime;
	  int32_t duration = effect->duration;

	  if (elapsedTime < attackTime)
	  {
	    newValue = (magnitude - attackLevel) * elapsedTime;
	    newValue /= attackTime;
	    newValue += attackLevel;
	  }
	  if (elapsedTime > (duration - fadeTime))
	  {
	    newValue = (magnitude - fadeLevel) * (duration - elapsedTime);
	    newValue /= fadeTime;
	    newValue += fadeLevel;
	  }
	  float scale = (float)value / (int32_t)0x7fff;
	  float fvalue = scale * newValue;

	  return (int32_t)fvalue;
}

float HidFFB::NormalizeRange(int32_t x, int32_t maxValue)
{
	  return (float)x * 1.00 / maxValue;
}

void HidFFB::set_config(FFBWheelConfig *conf)
{
	this->conf = conf;
}
