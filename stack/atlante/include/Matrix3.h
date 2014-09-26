/*
 * Matrix3.h
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef MATRIX3_H_
#define MATRIX3_H_

class Vector3;

#include <string.h>
#include "cvg_types.h"
#include "cvgString.h"

class Matrix3
{
public:
	double value[3][3];

	inline Matrix3() { }
	Matrix3(const Matrix3 &m);
	inline Matrix3(double *a) { memcpy(value, a, sizeof(value)); }
	Matrix3(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22);
	inline Matrix3(const Vector3 &row1, const Vector3 &row2, const Vector3 &row3) { setRows(row1, row2, row3); }
	inline virtual ~Matrix3() { }

	Matrix3 &operator = (const Matrix3 &m);
	Matrix3 &setRows(const Vector3 &row1, const Vector3 &row2, const Vector3 &row3);
	Matrix3 &setCols(const Vector3 &col1, const Vector3 &col2, const Vector3 &col3);
	static Matrix3 identity() { return Matrix3(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0); }
	static Matrix3 zero() { return Matrix3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); }

	// Operations are defined as virtual, so subtypes of 3x3 matrices may override them with faster algorithms
	virtual double det() const;
	virtual Matrix3 transpose() const;
	virtual Matrix3 inverse() const;
	virtual Matrix3 operator * (double f) const;
	virtual Vector3 operator * (const Vector3 &v) const; 
	virtual Matrix3 operator * (const Matrix3 &m) const;
	virtual Matrix3 operator / (double d) const;
	virtual Matrix3 operator < (const Matrix3 &m) const;
	virtual Matrix3 operator < (double t) const;
	virtual Matrix3 operator > (const Matrix3 &m) const;
	virtual Matrix3 operator > (double t) const;
	virtual cvg_uint countValues(double v) const;
	virtual cvgString toString() const;
};

inline Matrix3 operator * (double left, const Matrix3 &right) { return right * left; }

#endif
