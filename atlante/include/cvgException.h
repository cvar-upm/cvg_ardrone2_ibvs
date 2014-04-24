#include "cvg_types.h"
#include "cvgString.h"

#ifndef CVG_EXCEPTION_INCLUDED__
#define CVG_EXCEPTION_INCLUDED__

class cvgException {
private:
	cvgString message;

public:
	cvgException();
	cvgException(const cvgException &e);
	cvgException(const cvg_char *msg);
	cvgException(const cvgString &msg);
	cvgException(const std::exception &e);

	virtual ~cvgException();

	inline cvgString &getMessage() { return message; }
	inline cvgException &operator = (cvgException &e) { message = e.message; return *this; }
};

#endif
