#include "cvg_types.h"
#include "cvgString.h"
#include "cvgException.h"

#ifndef CVG_THREAD_INCLUDED__
#define CVG_THREAD_INCLUDED__

#ifdef CVG_WINDOWS
#	include <windows.h>
#else
#	include <pthread.h>
#	include <errno.h> 
#endif

class cvgThread {
private:
	cvgString name;
	volatile bool started;
	cvg_uint stopTimeout;
	bool created;
#ifdef CVG_WINDOWS
	HANDLE handle;
	HANDLE endEvent, endedEvent;
	DWORD threadId;
#else
	pthread_t handle;
	pthread_mutex_t endMutex;
	pthread_cond_t endedCondition;
#endif

protected:
	void create();
	void destroy();
#ifdef CVG_WINDOWS
	static DWORD WINAPI worker(void *p);
#else
	volatile bool loop;
	void setAbsoluteTimeout(timespec *ts, cvg_uint absTimeout);
	static void *worker(void *p);
#endif

public:
	cvgThread(cvgString *name);
	cvgThread(const cvg_char *name);
	virtual ~cvgThread();

	inline void setName(cvgString *name) { this->name = *name; }
	inline void setName(const cvg_char *name) { this->name = name; }
	inline cvgString &getName() { return name; }

	inline void setStopTimeout(cvg_uint tm) { stopTimeout = tm; }
	inline cvg_uint getStopTimeout() { return stopTimeout; }

	void start();
	void stop(bool killIfNecessary = true);
	bool kill();
	inline void selfStop() { loop = false; }
	virtual void run() { }

	inline cvg_bool isStarted() { return started; }
};

#endif
