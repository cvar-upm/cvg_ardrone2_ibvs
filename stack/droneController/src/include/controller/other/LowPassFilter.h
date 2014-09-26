/*
 * LowPassFilter.h
 *
 *  Created on: Nov 27, 2012
 *      Author: jespestana
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_

#include <atlante.h>
#include "Other/Timer.h"
//#include <droneproxy.h>

namespace CVG_BlockDiagram {

class LowPassFilter {
private:
	// Input/output
	cvg_double u_k; // input
	cvg_double y_k; // output

	// Parameters
	cvg_double tr;	// response time
	cvg_double max_output;
	cvg_double min_output;
	cvg_double max_speed;

	// Internal state
	cvg_double y_km1; // last output
	Timer timer;
//	DroneProxy::Timing::Timer timer;
	cvg_bool started, saturation_enabled;

public:
	LowPassFilter();
	virtual ~LowPassFilter();

	void reset();

	inline void setResponseTime(cvg_double tr) { if (tr > 0) this->tr = tr; else this->tr = 1.0; }
	void enableSaturation(cvg_bool enable, cvg_double min, cvg_double max);
	void enableMaxSpeedSaturation(cvg_double max_speed);

	inline void setInternalyk(cvg_double y_kint) { this->y_k = y_kint; }
	inline void setInput(cvg_double u_k) 	{ this->u_k = u_k; }
	inline cvg_double getInput() 			{ return u_k; }
	cvg_double getOutput(bool use_last_input = false);
};

} /* namespace CVG_BlockDiagram */
#endif /* LOWPASSFILTER_H_ */
