/*
 * Mutex.h
 *
 *  Created on: 02/09/2011
 *      Author: Ignacio Mellado
 *      Notes: This is class is currently only implemented for Linux. 
 */

#ifndef CVGMUTEX_H_
#define CVGMUTEX_H_

#include <pthread.h>
#include "cvg_types.h"
#include "cvgException.h"

class cvgMutex {
	friend class cvgCondition;
	friend class cvgSemaphore;
private:
	pthread_mutex_t handler;

public:
	cvgMutex(cvg_bool recursive = true);
	virtual ~cvgMutex();

	cvg_bool lock();
	cvg_bool unlock();
};

#endif /* MUTEX_H_ */
