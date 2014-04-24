/*
 * LowPassFilter.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#include "controller/other/LowPassFilter.h"

namespace CVG_BlockDiagram {

LowPassFilter::LowPassFilter() {
	u_k = 0.0;
	y_k = 0.0;
	y_km1 = 0.0;

	tr = 1.0;

	saturation_enabled = false;
	max_output = 0.0;
	min_output = 0.0;

	max_speed  = -1.0; // <0 means disabled

	started = false;
	timer.restart(false);
}

LowPassFilter::~LowPassFilter() {
}

void LowPassFilter::reset() {
	started = false;
	timer.restart(false);
}

void LowPassFilter::enableSaturation(cvg_bool enable, cvg_double min, cvg_double max) {
	saturation_enabled = enable;

	if (saturation_enabled)
		if (min < max) {
			max_output = max;
			min_output = min;
			return;
		}

	saturation_enabled = enable;
	max_output = 0.0;
	min_output = 0.0;
}

void LowPassFilter::enableMaxSpeedSaturation(cvg_double max_speed) {
	this->max_speed = max_speed; // <0 means disabled
}

cvg_double LowPassFilter::getOutput(bool use_last_input) {

	cvg_double elapsed = timer.getElapsedSeconds();
	timer.restart(started);
	if (!started || tr<0 ) { // start sequence
		y_k = u_k;
		y_km1 = u_k;

		started = true;
		return y_k;
	}

	// calculate output
	cvg_double alpha = exp(-3*elapsed/tr);
	y_km1 = y_k;
	y_k = alpha*y_km1 + (1-alpha)*u_k;

	cvg_double current_speed = (y_k - y_km1)/elapsed;
	if ( (max_speed > 0.0) && ( fabs(current_speed) > max_speed ) ) { // < 0.0 means disabled
		current_speed = current_speed > 0 ? max_speed : -max_speed;
		y_k = y_km1 + elapsed*current_speed;
	}

	// saturate output
	if ( saturation_enabled && (y_k > max_output) )
		y_k = max_output;
	if ( saturation_enabled && (y_k < min_output) )
		y_k = min_output;

	return y_k;
}

} /* namespace CVG_BlockDiagram */
