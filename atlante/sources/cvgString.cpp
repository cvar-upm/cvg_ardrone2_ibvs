/**
 * Description: cvgString class. A bunch of operators and methods are defined 
 * to extend STL string functionality.
 * Author: Ignacio Mellado (CVG)
 * Last revision date: 24/11/2010
 */

#include "../include/cvgString.h"
#include "../include/cvgException.h"
#include <stdio.h>

cvgString::cvgString() : std::string() {
}

cvgString::cvgString(const std::string &s) : std::string(s) {
}

cvgString::cvgString(const cvgString &s) : std::string((std::string)s) {
}

cvgString::cvgString(const cvg_char *buff) : std::string(buff) {
}

cvgString::cvgString(cvg_char n) {
	cvg_char buff[2];
	buff[0] = n;
	buff[1] = '\0';
	(*(std::string *)this) = buff;
}

cvgString::cvgString(cvg_uchar n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_UCHAR"u", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_int n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_INT"d", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_uint n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_UINT"u", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_short n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_SHORT"d", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_ushort n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_USHORT"u", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_long n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_LONG"d", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_ulong n) {
	cvg_char buff[32];
	snprintf(buff, sizeof(buff) - 1, "%"CVG_PRINTF_PREFIX_ULONG"u", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_float n) {
	cvg_char buff[256];
	snprintf(buff, sizeof(buff) - 1, "%f", n);
	(*this) = buff;
}

cvgString::cvgString(cvg_double n) {
	cvg_char buff[256];
	snprintf(buff, sizeof(buff) - 1, "%f", n);
	(*this) = buff;
}

cvgString &cvgString::operator = (const cvgString &s) {
	(*(std::string *)this) = s;
	return *this;
}

cvgString &cvgString::operator = (const cvg_char *buff) {
	(*(std::string *)this) = buff;
	return *this;
}

cvgString cvgString::operator + (const cvg_char *buff) const {
	return cvgString(*((std::string *)this) + buff);
}

cvgString cvgString::operator + (const cvgString &str) const {
	return cvgString(*((std::string *)this) + *((std::string *)&str));
}

cvgString cvgString::operator + (cvg_char c) const {
	return (*this) + cvgString(c);
}

cvgString cvgString::operator + (cvg_uchar n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_short n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_ushort n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_int n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_uint n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_long n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_ulong n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_float n) const {
	return (*this) + cvgString(n);
}

cvgString cvgString::operator + (cvg_double n) const {
	return (*this) + cvgString(n);
}

cvgString operator + (const cvg_char *left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_char left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_uchar left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_short left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_ushort left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_int left, const cvgString &right) {
	return cvgString(left) + right;
}
cvgString operator + (cvg_uint left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_long left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_ulong left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_float left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString operator + (cvg_double left, const cvgString &right) {
	return cvgString(left) + right;
}

cvgString cvgString::operator - (const cvgString &str) const {
	return replace(str, "");
}

cvgString cvgString::replace(const cvgString &str, const cvgString &substitute, size_t offset) const {
	cvgString strCopy = (*this);
	size_t occPos = offset;
	size_t maskLength = str.length();
	size_t substLength = substitute.length();
	// Loop while any match is found
	while(occPos != cvgString::npos) {
		occPos = strCopy.find(str, occPos);
		// If the substring matches the pattern, replace it
		if (occPos != cvgString::npos) {
			strCopy = ((std::string)strCopy).replace(occPos, maskLength, substitute);
			occPos += substLength;
		}
	}
	return strCopy;
}

cvgString cvgString::operator - (const cvg_char *buff) const {
	return replace(cvgString(buff), "");
}

cvgString cvgString::operator - (cvg_char c) const {
	return replace(cvgString(c), "");
}

cvgString cvgString::operator - (cvg_uchar n) const {
	return replace((cvg_ulong)n, "");
}

cvgString cvgString::operator - (cvg_short n) const {
	return replace((cvg_long)n, "");
}

cvgString cvgString::operator - (cvg_ushort n) const {
	return replace((cvg_ulong)n, "");
}

cvgString cvgString::operator - (cvg_int n) const {
	return replace((cvg_long)n, "");
}

cvgString cvgString::operator - (cvg_uint n) const {
	return replace((cvg_ulong)n, "");
}

cvgString cvgString::operator - (cvg_long n) const {
	return replace((cvg_long)n, "");
}

cvgString cvgString::replace(cvg_long n, const cvgString &substitute, size_t offset) const {
	cvgString strCopy = (*this);
	size_t occPos = offset;
	// Format the number as a string to compare with the potential matches
	char buffer[32];
	snprintf(buffer, sizeof(buffer) - 1, "%"CVG_PRINTF_PREFIX_LONG"d", n);
	cvgString strBuffer = buffer;
	size_t subsLength = substitute.length();
	const static char mask1[] = "0123456789-";
	const static char mask2[] = "0123456789";
	// Loop until no more numeric characters are found
	while(occPos != cvgString::npos) {
		// Find the start of any signed number
		occPos = strCopy.find_first_of(mask1, occPos);
		if (occPos != cvgString::npos) {
			// If a number was found, look for its end (any char that's not a digit)
			size_t i = strCopy.find_first_not_of(mask2, occPos + 1);
			size_t len;
			// Calculate the length of the subtring depending on the "end of string" condition
			if (i == cvgString::npos)
				len = strCopy.length() - occPos;
			else
				len = i - occPos;
			// Compare the potential match with the string formatted number 
			if (strCopy.substr(occPos, len) == strBuffer) {
				// Replace if a match is found
				strCopy = ((std::string)strCopy).replace(occPos, len, substitute);
				occPos += subsLength;
			}
			else occPos += len;
		}
	}
	return strCopy;
}

cvgString cvgString::operator - (cvg_ulong n) const {
	return replace((cvg_ulong)n, "");
}

cvgString cvgString::replace(cvg_ulong n, const cvgString &substitute, size_t offset) const {
	cvgString strCopy = (*this);
	size_t occPos = offset;
	// Format the number as a string to compare with the potential matches
	char buffer[32];
	snprintf(buffer, sizeof(buffer) - 1, "%"CVG_PRINTF_PREFIX_ULONG"u", n);
	cvgString strBuffer = buffer;
	size_t subsLength = substitute.length();
	const static char mask[] = "0123456789";
	// Loop until no more numeric characters are found
	while(occPos != cvgString::npos) {
		// Find the start of any unsigned number
		occPos = strCopy.find_first_of(mask, occPos);
		if (occPos != cvgString::npos) {
			// If a number was found, look for its end (any char that's not a digit)
			size_t i = strCopy.find_first_not_of(mask, occPos + 1);
			size_t len;
			// Calculate the length of the subtring depending on the "end of string" condition
			if (i == cvgString::npos)
				len = strCopy.length() - occPos;
			else
				len = i - occPos;
			// Compare the potential match with the string formatted number 
			if (strCopy.substr(occPos, len) == strBuffer) {
				// Replace if a match is found
				strCopy = ((std::string)strCopy).replace(occPos, len, substitute);
				occPos += subsLength;
			}
			else occPos += len;
		}
	}
	return strCopy;
}

cvgString cvgString::operator - (cvg_float n) const {
	return replace((cvg_double)n, "");
}

cvgString cvgString::operator - (cvg_double n) const {
	return replace((cvg_double)n, "");
}

cvgString cvgString::replace(cvg_double n, const cvgString &substitute, size_t offset) const {
	cvgString strCopy = (*this);
	size_t occPos = offset;
	size_t subsLength = substitute.length();
	const static char mask1[] = "0123456789-+";
	const static char mask2[] = "0123456789";
	// Loop until no more characters for mask1 are found
	while(occPos != cvgString::npos) {
		// Find the start of any signed number
		occPos = strCopy.find_first_of(mask1, occPos);
		if (occPos != cvgString::npos) {
			// If a number was found, look for its end or a decimal point (any char that's not a digit)
			size_t i = strCopy.find_first_not_of(mask2, occPos + 1);
			size_t len, decimals;
			if (i == cvgString::npos) {
				// If the end of the string was reached, we got the length of the number and no fractional part
				len = strCopy.length() - occPos;
				decimals = 0;
			}
			else {
				// Check if a decimal point was found and valid digit follows
				if (strCopy[i] == '.' && (i + 1 < strCopy.length()) && strCopy[i + 1] >= '0' && strCopy[i + 1] <= '9') {
					// Find the end of the number, after the fractional part (any character not being a digit)
					size_t j = strCopy.find_first_not_of(mask2, i + 1);
					// Calculate length of number and decimals depending on "end of string" condition
					if (j != cvgString::npos) { len = j - occPos; decimals = j - i - 1; }
					else { len = strCopy.length() - occPos; decimals = strCopy.length() - i - 1; }
				} else {
					// No decimals
					len = i - occPos;
					decimals = 0;
				}
			}
			// Format the number with as many decimal places as the detected numeric substring has and compare
			char buffer[256];
			snprintf(buffer, sizeof(buffer) - 1, (cvgString("%.") + (cvg_long)decimals + "f").c_str(), n);
			// If they match for the detected precission, replace the numeric substring
			if (strCopy.substr(occPos, len) == cvgString(buffer)) {
				strCopy = ((std::string)strCopy).replace(occPos, len, substitute);
				occPos += subsLength;
			}
			else occPos += (len - decimals);
		}
	}
	return strCopy;
}

cvgString &cvgString::operator += (const cvgString &s) {
	(*this) = (*this) + s;
	return (*this);
}

cvgString &cvgString::operator += (const cvg_char *s) {
	(*this) = (*this) + s;
	return (*this);
}

cvgString &cvgString::operator += (cvg_char c) {
	(*this) = (*this) + c;
	return (*this);
}

cvgString &cvgString::operator += (cvg_uchar n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_short n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_ushort n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_int n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_uint n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_long n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_ulong n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_float n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString &cvgString::operator += (cvg_double n) {
	(*this) = (*this) + n;
	return (*this);
}

cvgString cvgString::operator * (cvg_int n) const {
	cvgString strCopy;
	strCopy.reserve(this->length() * n);
	while(n > CVG_LITERAL_INT(0)) {
		strCopy += (*this);
		n--;
	}
	return strCopy;
}

cvgString cvgString::operator * (cvg_double n) const {
	cvgString strCopy = (*this) * (cvg_int)n;
	// Use the * operator to multiply the integer part
	cvg_int remaining = (cvg_int)((n - (cvg_int)n) * this->length());
	// Append the fractional part
	if (remaining > 0) strCopy += this->substr(0, remaining);
	return strCopy;
}

cvgString operator * (cvg_int left, const cvgString &right) {
	return (cvgString)right * left;
}

cvgString operator * (cvg_double left, const cvgString &right) {
	return (cvgString)right * left;
}

cvgString &cvgString::operator *= (cvg_int n) {
	(*this) = (*this) * n;
	return (*this);
}

cvgString &cvgString::operator *= (cvg_double n) {
	(*this) = (*this) * n;
	return (*this);
}

cvgStringVector cvgString::split(const cvgString &token, size_t offset) const {
	cvgStringVector output;
	size_t tokenLength = token.length();
	size_t pieceStart = offset;
	// Until the end of the string is reached
	bool loop = true;
	while(loop) {
		offset = find(token, offset);
		loop = offset != cvgString::npos;
		// If the loop condition is off, no more matches were found:
		// the substring will reach the end of the string
		if (!loop) offset = length();
		if (offset > pieceStart)
			// If there are characters between consecutive matches, put them in the output as a string
			output.push_back(subString(pieceStart, offset - pieceStart));
		// Go find matches after the last found pattern
		pieceStart = offset + tokenLength;
		offset = pieceStart;
	}
	return output;
}

cvgString &cvgString::operator -= (const cvgString &s) {
	(*this) = (*this) - s;
	return (*this);
}

cvgString &cvgString::operator -= (const cvg_char *s) {
	(*this) = (*this) - s;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_char c) {
	(*this) = (*this) - c;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_uchar n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_short n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_ushort n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_int n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_uint n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_long n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_ulong n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_float n) {
	(*this) = (*this) - n;
	return (*this);
}

cvgString &cvgString::operator -= (cvg_double n) {
	(*this) = (*this) - n;
	return (*this);
}

cvg_int cvgString::toInt(cvg_bool *result) {
	cvg_int value;
	if (sscanf(c_str(), "%d", &value) == 1) {
		if (result != NULL) (*result) = true;
		return value;
	} else {
		if (result == NULL)
			throw cvgException("[cvgString] Cannot convert string to integer");
		else (*result) = false;
		return 0;		
	} 
}

cvg_float cvgString::toFloat(cvg_bool *result) {
	cvg_float value;
	if (sscanf(c_str(), "%f", &value) == 1) {
		if (result != NULL) (*result) = true;
		return value;
	} else {
		if (result == NULL)
			throw cvgException("[cvgString] Cannot convert string to float");
		else (*result) = false;
		return 0;		
	} 
}

cvg_double cvgString::toDouble(cvg_bool *result) {
	cvg_double value;
	if (sscanf(c_str(), "%lf", &value) == 1) {
		if (result != NULL) (*result) = true;
		return value;
	} else {
		if (result == NULL)
			throw cvgException("[cvgString] Cannot convert string to double");
		else (*result) = false;
		return 0;		
	} 
}

