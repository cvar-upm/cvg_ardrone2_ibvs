/**
 * Description: cvgString class. A bunch of operators and methods are defined 
 * to extend STL string functionality.
 * Author: Ignacio Mellado (CVG)
 * Last revision date: 24/11/2010
 */

#ifndef CVG_STRING_INCLUDED__
#define CVG_STRING_INCLUDED__

#include <string>
#include "cvg_types.h"
#include "cvgVector.h"

class cvgString : public std::string {
public:
	cvgString();
	cvgString(const std::string &s);
	cvgString(const cvgString &s);
	cvgString(const cvg_char *buff);

	cvgString(cvg_char n);
	cvgString(cvg_uchar n);
	cvgString(cvg_int n);
	cvgString(cvg_uint n);
	cvgString(cvg_short n);
	cvgString(cvg_ushort n);
	cvgString(cvg_long n);
	cvgString(cvg_ulong n);
	cvgString(cvg_float n);
	cvgString(cvg_double n);

	cvgString &operator = (const cvgString &s);
	/* Not strictly needed to compile, but defined for efficiency */
	cvgString &operator = (const cvg_char *buff);
/* Since constructors for all basic types are defined, there is no need to define 
	an equality operator for each type */

	cvgString operator + (const cvgString &str) const;
	cvgString operator + (const cvg_char *buff) const;

	/* We should not need to define the operator for each type, but some conversion
	problems were detected without doing it */
	cvgString operator + (cvg_char c) const;
	cvgString operator + (cvg_uchar c) const;
	cvgString operator + (cvg_short c) const;
	cvgString operator + (cvg_ushort c) const;
	cvgString operator + (cvg_int c) const;
	cvgString operator + (cvg_uint c) const;
	cvgString operator + (cvg_long c) const;
	cvgString operator + (cvg_ulong c) const;
	cvgString operator + (cvg_float c) const;
	cvgString operator + (cvg_double c) const;

	cvgString operator - (const cvgString &str) const;
	cvgString operator - (const cvg_char *buff) const;

	cvgString operator - (cvg_char c) const;
	cvgString operator - (cvg_uchar c) const;
	cvgString operator - (cvg_short c) const;
	cvgString operator - (cvg_ushort c) const;
	cvgString operator - (cvg_int c) const;
	cvgString operator - (cvg_uint c) const;
	cvgString operator - (cvg_long c) const;
	cvgString operator - (cvg_ulong c) const;
	cvgString operator - (cvg_float c) const;
	cvgString operator - (cvg_double c) const;

	cvgString &operator += (const cvgString &s);
	cvgString &operator += (const cvg_char *s);
	cvgString &operator += (cvg_char c);
	cvgString &operator += (cvg_uchar c);
	cvgString &operator += (cvg_short c);
	cvgString &operator += (cvg_ushort c);
	cvgString &operator += (cvg_int c);
	cvgString &operator += (cvg_uint c);
	cvgString &operator += (cvg_long c);
	cvgString &operator += (cvg_ulong c);
	cvgString &operator += (cvg_float c);
	cvgString &operator += (cvg_double c);

	cvgString &operator -= (const cvgString &s);
	cvgString &operator -= (const cvg_char *s);
	cvgString &operator -= (cvg_char c);
	cvgString &operator -= (cvg_uchar c);
	cvgString &operator -= (cvg_short c);
	cvgString &operator -= (cvg_ushort c);
	cvgString &operator -= (cvg_int c);
	cvgString &operator -= (cvg_uint c);
	cvgString &operator -= (cvg_long c);
	cvgString &operator -= (cvg_ulong c);
	cvgString &operator -= (cvg_float c);
	cvgString &operator -= (cvg_double c);

	cvgString operator * (cvg_int n) const;
	cvgString operator * (cvg_double n) const;

	cvgString &operator *= (cvg_int n);
	cvgString &operator *= (cvg_double n);

	inline cvgString subString(size_t offset = 0, size_t length = cvgString::npos) const { return cvgString(substr(offset, length)); }
	cvgStringVector split(const cvgString &token, size_t offset = 0) const;
	cvgString replace(const cvgString &str, const cvgString &substitute, size_t offset = 0) const;
	cvgString replace(cvg_double n, const cvgString &substitute, size_t offset = 0) const;
	cvgString replace(cvg_ulong n, const cvgString &substitute, size_t offset = 0) const;
	cvgString replace(cvg_long n, const cvgString &substitute, size_t offset = 0) const;

	cvg_int toInt(cvg_bool *result = NULL);
	cvg_float toFloat(cvg_bool *result = NULL);
	cvg_double toDouble(cvg_bool *result = NULL);
};

cvgString operator + (const cvg_char *left, const cvgString &right);
cvgString operator + (cvg_char left, const cvgString &right);
cvgString operator + (cvg_uchar left, const cvgString &right);
cvgString operator + (cvg_short left, const cvgString &right);
cvgString operator + (cvg_ushort left, const cvgString &right);
cvgString operator + (cvg_int left, const cvgString &right);
cvgString operator + (cvg_uint left, const cvgString &right);
cvgString operator + (cvg_long left, const cvgString &right);
cvgString operator + (cvg_ulong left, const cvgString &right);
cvgString operator + (cvg_char left, const cvgString &right);
cvgString operator + (cvg_float left, const cvgString &right);
cvgString operator + (cvg_double left, const cvgString &right);

cvgString operator * (cvg_int left, const cvgString &right);
cvgString operator * (cvg_double left, const cvgString &right);

#endif	/* CVG_STRING_INCLUDED__ */
