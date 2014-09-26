/*
 * Vector3.h
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef VECTOR3_H_
#define VECTOR3_H_

#include "cvg_types.h"

class Matrix3;

class Vector3
{
public:
	double x, y, z;

	inline Vector3(double x = 0.0, double y = 0.0, double z = 0.0) { Vector3::x = x; Vector3::y = y; Vector3::z = z; }
	inline Vector3(double *a) { x = a[0]; y = a[1]; z = a[2]; }
	virtual ~Vector3();

	Vector3 operator - () const;
	Vector3 rotate(const Vector3 &point, const Vector3 &axis, double angle) const;
	double modulus() const;
	Vector3 normalize() const;
	Vector3 &operator = (const Vector3 &v);
	Vector3 operator + (const Vector3 &v) const;
	Vector3 operator - (const Vector3 &v) const;
	Vector3 operator * (double f) const;
	double operator * (const Vector3 &v) const;
	Vector3 operator * (const Matrix3 &m) const;
	Vector3 operator / (double d) const;
	Vector3 operator % (const Vector3 &v) const;
	Vector3 &operator += (const Vector3 &v);
	Vector3 &operator -= (const Vector3 &v);
	bool operator != (const Vector3 &v) const;
	bool operator == (const Vector3 &v) const;
	Vector3 operator > (const Vector3 &v) const;
	Vector3 operator > (double t) const;
	Vector3 operator < (const Vector3 &v) const;
	Vector3 operator < (double t) const;
	cvg_uint countValues(double v) const;
};

inline Vector3 operator * (double left, const Vector3 &right) { return right * left; }

#endif
