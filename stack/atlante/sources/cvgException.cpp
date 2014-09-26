#include "../include/cvgException.h"
#include "../include/cvgString.h"

cvgException::cvgException() {
}

cvgException::cvgException(const cvg_char *msg) {
	message = msg;
}

cvgException::cvgException(const cvgString &msg) {
	message = msg;
}

cvgException::cvgException(const cvgException &e) {
	message = e.message;
}

cvgException::cvgException(const std::exception &e) {
	message = cvgString("C++ exception: ") + e.what();
}

cvgException::~cvgException() {
}

