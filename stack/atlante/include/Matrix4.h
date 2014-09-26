/*
 * Matrix4.h
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef MATRIX4_H_
#define MATRIX4_H_

#include "Vector4.h"
#include "Vector3.h"
#include <string.h>
#include "cvgString.h"
#include "cvgException.h"

class Matrix4
{
public:
	double value[4][4];

	inline Matrix4() { }
	inline Matrix4(double *a) { memcpy(value, a, sizeof(value)); }
	Matrix4(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double  a13, double a20, double  a21, double a22, double a23, double a30, double a31, double a32, double a33);
	inline Matrix4(const Vector4 &row1, const Vector4 &row2, const Vector4 &row3, const Vector4 &row4) { setRows(row1, row2, row3, row4); }
	inline virtual ~Matrix4() { }

	Matrix4 &operator = (const Matrix4 &m);
	Matrix4 &setRows(const Vector4 &row1, const Vector4 &row2, const Vector4 &row3, const Vector4 &row4);
	Matrix4 &setCols(const Vector4 &col1, const Vector4 &col2, const Vector4 &col3, const Vector4 &col4);
	static Matrix4 identity() { return Matrix4(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0); }
	static Matrix4 zero() { return Matrix4(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

	// Operations are defined as virtual, so subtypes of 4x4 matrices may override them with faster algorithms
	virtual Matrix4 inverse() const;
	virtual double det() const;
	virtual bool operator != (const Matrix4 &m) const;
	virtual double trace() const;
	virtual Matrix4 transpose() const;
	virtual Matrix4 operator + (const Matrix4 &m) const;
	virtual Matrix4 operator - (const Matrix4 &m) const;
	virtual Vector4 operator * (const Vector3 &v) const;
	virtual Vector4 operator * (const Vector4 &v) const;
	virtual Matrix4 operator * (const Matrix4 &m) const;
	virtual Matrix4 operator * (double f) const;
	virtual Matrix4 operator / (double d) const;
	virtual Matrix4 operator < (const Matrix4 &m) const;
	virtual Matrix4 operator < (double t) const;
	virtual Matrix4 operator > (const Matrix4 &m) const;
	virtual Matrix4 operator > (double t) const;
	virtual Matrix4 scalarPow(double e) const;
	virtual Matrix4 scalarSqrt() const;
	virtual cvgString toString() const;
	virtual cvg_uint countValues(double v) const;

private:
	Vector4 res_vector;
};

inline Matrix4 operator * (double left, const Matrix4 &right) { return right * left; }

#endif
