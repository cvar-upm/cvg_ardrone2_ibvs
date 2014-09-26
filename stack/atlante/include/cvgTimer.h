#ifndef CVGTIMER_INCLUDED__
#define CVGTIMER_INCLUDED__

#include <sys/time.h>
#include <stdlib.h>
#include "cvg_types.h"

class cvgTimer {
private:
	timeval t0, t1;
public:
	inline cvgTimer() { restart(false); }
	virtual ~cvgTimer();

	inline void restart(bool useLastSample = true) { if (useLastSample) t0 = t1; else gettimeofday(&t0, NULL); }
	inline double getElapsedSeconds() {
		gettimeofday(&t1, NULL);
		return (t1.tv_sec - t0.tv_sec) + (t1.tv_usec - t0.tv_usec) * 1e-6;
	}
	static cvg_double getSystemSeconds();
};

#endif
