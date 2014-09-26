/*
 * Timer.h
 *
 *  Created on: May 30, 2012
 *      Author: jespestana
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <sys/time.h>
#include <stdlib.h>

class Timer {
private:
	timeval t0, t1;
public:
	inline Timer() { restart(false); }
	virtual ~Timer() {};

	inline void restart(bool useLastSample = true) { if (useLastSample) t0 = t1; else gettimeofday(&t0, NULL); }
	inline double getElapsedSeconds() {
		gettimeofday(&t1, NULL);
		return (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) / 1e6;
	}
};


#endif /* TIMER_H_ */
