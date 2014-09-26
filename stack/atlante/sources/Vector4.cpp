/*
 * Vector4.cpp
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#include <Vector4.h>
#include <Matrix4.h>
#include <math.h>

Vector4 Vector4::operator *(Matrix4 &m) const {
	return Vector4(	m.value[0][0] * x + m.value[1][0] * y + m.value[2][0] * z + m.value[3][0] * w,
					m.value[0][1] * x + m.value[1][1] * y + m.value[2][1] * z + m.value[3][1] * w,
					m.value[0][2] * x + m.value[1][2] * y + m.value[2][2] * z + m.value[3][2] * w,
					m.value[0][3] * x + m.value[1][3] * y + m.value[2][3] * z + m.value[3][3] * w
					);
}

Vector4 Vector4::normalize() const {
	double n = sqrt(x * x + y * y + z * z + w * w);
	if (n == 0) return Vector4(*this);
	return Vector4(x / n, y / n, z / n, w / n);
}

Vector4 Vector4::operator * (double f) const {
	return Vector4(x * f, y * f, z * f, w * f);
}

Vector4 Vector4::operator / (double d) const {
	return Vector4(x / d, y / d, z / d, w / d);
}
