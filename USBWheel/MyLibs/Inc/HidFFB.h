/*
 * HidFFB.h
 *
 *  Created on: 12.02.2020
 *      Author: Yannick
 */

#ifndef HIDFFB_H_
#define HIDFFB_H_

#include "main.h"
#include <Filters.h>
#include <UsbHidHandler.h>
#include "ffb_defs.h"
#include "EncoderLocal.h"

class HidFFB: public UsbHidHandler {
public:
	HidFFB();
	virtual ~HidFFB();
	void setFilterFQ(float f, float q);
	void hidOut(uint8_t* report);
	void hidGet(uint8_t id,uint16_t len,uint8_t** return_buf);
	int32_t calculateEffects(EncoderLocal* encoder); //Axis: 1/2 pos: current position scaled from -0x7fff to 0x7fff
    void set_config(FFBWheelConfig* conf);
	bool idlecenter = true;

	uint32_t hid_out_period = 0; // ms since last out report for measuring update rate

private:
	// HID

	uint8_t find_free_effect(uint8_t type);
	void new_effect(FFB_CreateNewEffect_Feature_Data_t* effect);
	void free_effect(uint16_t id);
	void ffb_control(uint8_t cmd);
	void reset_ffb();
	void set_effect(FFB_SetEffect_t* effect);
	void set_condition(FFB_SetCondition_Data_t* cond);
	void set_envelope(FFB_SetEnvelope_Data_t* envelop);

	void set_constant_effect(FFB_SetConstantForce_Data_t* effect);
	void set_ramp_effect(FFB_SetRampForce_Data_t* effect);
	void set_periodic(FFB_SetPeriodic_Data_t* report);
	void start_FFB();
	void stop_FFB();



	int32_t ConstantForceCalculator(FFB_Effect* effect);
	int32_t RampForceCalculator(FFB_Effect* effect);
	int32_t SquareForceCalculator(FFB_Effect* effect);
	int32_t SinForceCalculator(FFB_Effect* effect);
	int32_t TriangleForceCalculator(FFB_Effect* effect);
	int32_t SawtoothDownForceCalculator(FFB_Effect* effect);
	int32_t SawtoothUpForceCalculator(FFB_Effect* effect);
	int32_t ConditionForceCalculator(FFB_Effect* effect, float metric);

    int32_t ApplyGain(uint32_t value, uint8_t gain);
    int32_t ApplyEnvelope(FFB_Effect* effect, int32_t value);
    float NormalizeRange(int32_t x, int32_t maxValue);

	uint8_t report_counter = 0;
	uint16_t report_counter_hid = 0;
	uint8_t last_effect_id = 0;
	uint16_t used_effects = 0;
	uint8_t gain = 0xff;
	bool ffb_active = false;
	FFB_BlockLoad_Feature_Data_t blockLoad_report;
	FFB_PIDPool_Feature_Data_t pool_report;
	FFBWheelConfig* conf;

	reportFFB_status_t reportFFBStatus;
	FFB_Effect effects[MAX_EFFECTS];

	uint32_t lastOut = 0;

	float damper_f = 50 , damper_q = 0.2;
	float friction_f = 50 , friction_q = 0.2;
	float inertia_f = 20 , inertia_q = 0.2;
	const uint32_t calcfrequency = 1000;
	const float cfFilter_qfloatScaler = 0.01;

	Biquad damperFilter;
	Biquad interiaFilter;
	Biquad frictionFilter;
	Biquad constantFilter;
};

#endif /* HIDFFB_H_ */
