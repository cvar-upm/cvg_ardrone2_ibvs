#include "../include/cvgTimer.h"

cvgTimer::~cvgTimer() {
}

cvg_double cvgTimer::getSystemSeconds() {
	timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec + tv.tv_usec * 1e-6;
}

