/*
 * PID.h
 *
 *  Created on: 19/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef PID_H_
#define PID_H_

#include <atlante.h>
//JL change
//#include <droneproxy.h>
#include "Other/Timer.h"

#define PID_D_FILTER_LENGTH	5

namespace CVG_BlockDiagram {

class PID {
private:
	// Input/output
	cvg_double reference;
	cvg_double feedback;
	cvg_double output;

	// Parameters
	cvg_double Kp, Ki, Kd;
	cvg_double actuatorSaturation;
	cvg_double maxOutput;

	// Internal state
	cvg_double integrator;
	cvg_double lastError;
	cvg_double dErrorHistory[PID_D_FILTER_LENGTH];
	//JL change ojo!!!
	//DroneProxy::Timing::Timer timer;
	Timer timer;
	cvg_bool started;

public:
	PID();
	virtual ~PID();

    inline void setGains(const cvg_double p, const cvg_double i, const cvg_double d) { Kp = p; Ki = i; Kd = d; }
    inline void getGains(cvg_double &p, cvg_double &i, cvg_double &d) { p = Kp; i = Ki; d = Kd; }
    void enableMaxOutput(cvg_bool enable, cvg_double max);
	void enableAntiWindup(cvg_bool enable, cvg_double actuatorSaturation);

	inline void setReference(cvg_double ref) { reference = ref; }
	inline void setFeedback(cvg_double measure) { feedback = measure; }
	cvg_double getOutput();

	void reset();
};

} /* namespace CVG_BlockDiagram */
#endif /* PID_H_ */
