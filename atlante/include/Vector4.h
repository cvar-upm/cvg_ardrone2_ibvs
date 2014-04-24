/*
 * Vector4.h
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef VECTOR4_H_
#define VECTOR4_H_

#include "Vector3.h"

class Matrix4;

class Vector4
{
public:
	double x, y, z, w;

	inline Vector4(double x = 0.0, double y = 0.0, double z = 0.0, double w = 0.0) { Vector4::x = x; Vector4::y = y; Vector4::z = z; Vector4::w = w; }
	inline Vector4(double *a) { x = a[0]; y = a[1]; z = a[2]; w = a[3]; }
	inline Vector4(Vector3 &v, double w = 0.0) { x = v.x; y = v.y; z = v.z; Vector4::w = w; }
	inline virtual ~Vector4() {}

	inline Vector4 operator - (Vector4 &v) const { return Vector4(x - v.x, y - v.y, z - v.z, w - v.w); }
	Vector4 normalize() const;
	inline Vector4 operator + (Vector4 &v) const { return Vector4(x + v.x, y + v.y, z + v.z, w + v.w); }
	inline double operator * (Vector4 &v) const { return x * v.x + y * v.y + z * v.z + w * v.w; }
	Vector4 operator * (Matrix4 &m) const;
	Vector4 operator * (double f) const;
	Vector4 operator / (double d) const;
};

inline Vector4 operator * (double left, const Vector4 &right) { return right * left; }

#endif // !defined(AFX_VECTOR4_H__433671C9_B2F2_4AAE_8278_796F5D676396__INCLUDED_)
