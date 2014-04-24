/*
 * Matrix4.cpp
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#include "Matrix4.h"
#include <math.h>

Matrix4 &Matrix4::operator = (const Matrix4 &m) {
	// long but fast...
	value[0][0] = m.value[0][0];
	value[0][1] = m.value[0][1];
	value[0][2] = m.value[0][2];
	value[0][3] = m.value[0][3];
	value[1][0] = m.value[1][0];
	value[1][1] = m.value[1][1];
	value[1][2] = m.value[1][2];
	value[1][3] = m.value[1][3];
	value[2][0] = m.value[2][0];
	value[2][1] = m.value[2][1];
	value[2][2] = m.value[2][2];
	value[2][3] = m.value[2][3];
	value[3][0] = m.value[3][0];
	value[3][1] = m.value[3][1];
	value[3][2] = m.value[3][2];
	value[3][3] = m.value[3][3];
	return *this;
}

Matrix4 Matrix4::operator *(const Matrix4 &m) const {
	Matrix4 matrix;
	matrix.value[0][0] = value[0][0] * m.value[0][0] + value[0][1] * m.value[1][0] + value[0][2] * m.value[2][0] + value[0][3] * m.value[3][0];
	matrix.value[0][1] = value[0][0] * m.value[0][1] + value[0][1] * m.value[1][1] + value[0][2] * m.value[2][1] + value[0][3] * m.value[3][1];
	matrix.value[0][2] = value[0][0] * m.value[0][2] + value[0][1] * m.value[1][2] + value[0][2] * m.value[2][2] + value[0][3] * m.value[3][2];
	matrix.value[0][3] = value[0][0] * m.value[0][3] + value[0][1] * m.value[1][3] + value[0][2] * m.value[2][3] + value[0][3] * m.value[3][3];
	matrix.value[1][0] = value[1][0] * m.value[0][0] + value[1][1] * m.value[1][0] + value[1][2] * m.value[2][0] + value[1][3] * m.value[3][0];
	matrix.value[1][1] = value[1][0] * m.value[0][1] + value[1][1] * m.value[1][1] + value[1][2] * m.value[2][1] + value[1][3] * m.value[3][1];
	matrix.value[1][2] = value[1][0] * m.value[0][2] + value[1][1] * m.value[1][2] + value[1][2] * m.value[2][2] + value[1][3] * m.value[3][2];
	matrix.value[1][3] = value[1][0] * m.value[0][3] + value[1][1] * m.value[1][3] + value[1][2] * m.value[2][3] + value[1][3] * m.value[3][3];
	matrix.value[2][0] = value[2][0] * m.value[0][0] + value[2][1] * m.value[1][0] + value[2][2] * m.value[2][0] + value[2][3] * m.value[3][0];
	matrix.value[2][1] = value[2][0] * m.value[0][1] + value[2][1] * m.value[1][1] + value[2][2] * m.value[2][1] + value[2][3] * m.value[3][1];
	matrix.value[2][2] = value[2][0] * m.value[0][2] + value[2][1] * m.value[1][2] + value[2][2] * m.value[2][2] + value[2][3] * m.value[3][2];
	matrix.value[2][3] = value[2][0] * m.value[0][3] + value[2][1] * m.value[1][3] + value[2][2] * m.value[2][3] + value[2][3] * m.value[3][3];
	matrix.value[3][0] = value[3][0] * m.value[0][0] + value[3][1] * m.value[1][0] + value[3][2] * m.value[2][0] + value[3][3] * m.value[3][0];
	matrix.value[3][1] = value[3][0] * m.value[0][1] + value[3][1] * m.value[1][1] + value[3][2] * m.value[2][1] + value[3][3] * m.value[3][1];
	matrix.value[3][2] = value[3][0] * m.value[0][2] + value[3][1] * m.value[1][2] + value[3][2] * m.value[2][2] + value[3][3] * m.value[3][2];
	matrix.value[3][3] = value[3][0] * m.value[0][3] + value[3][1] * m.value[1][3] + value[3][2] * m.value[2][3] + value[3][3] * m.value[3][3];
	return matrix;	
}

Vector4 Matrix4::operator * (const Vector4 &v) const {
	return Vector4(	value[0][0] * v.x + value[0][1] * v.y + value[0][2] * v.z + value[0][3] * v.w,
					value[1][0] * v.x + value[1][1] * v.y + value[1][2] * v.z + value[1][3] * v.w,
					value[2][0] * v.x + value[2][1] * v.y + value[2][2] * v.z + value[2][3] * v.w,
					value[3][0] * v.x + value[3][1] * v.y + value[3][2] * v.z + value[3][3] * v.w
					);
}

Vector4 Matrix4::operator * (const Vector3 &v) const {
	return Vector4(	value[0][0] * v.x + value[0][1] * v.y + value[0][2] * v.z + value[0][3],
					value[1][0] * v.x + value[1][1] * v.y + value[1][2] * v.z + value[1][3],
					value[2][0] * v.x + value[2][1] * v.y + value[2][2] * v.z + value[2][3],
					value[3][0] * v.x + value[3][1] * v.y + value[3][2] * v.z + value[3][3]
					);
}

Matrix4 Matrix4::transpose() const {
	Matrix4 m;
	for (int y = 0; y < 4; y ++)
		for (int x = 0; x < 4; x ++)
			m.value[y][x] = value[x][y];
	return m;
}

Matrix4 &Matrix4::setRows(const Vector4 &row1, const Vector4 &row2, const Vector4 &row3, const Vector4 &row4) {
	value[0][0] = row1.x;
	value[0][1] = row1.y;
	value[0][2] = row1.z;
	value[0][3] = row1.w;
	value[1][0] = row2.x;
	value[1][1] = row2.y;
	value[1][2] = row2.z;
	value[1][3] = row2.w;
	value[2][0] = row3.x;
	value[2][1] = row3.y;
	value[2][2] = row3.z;
	value[2][3] = row3.w;
	value[3][0] = row4.x;
	value[3][1] = row4.y;
	value[3][2] = row4.z;
	value[3][3] = row4.w;
	return *this;
}

Matrix4 &Matrix4::setCols(const Vector4 &col1, const Vector4 &col2, const Vector4 &col3, const Vector4 &col4) {
	value[0][0] = col1.x;
	value[0][1] = col2.x;
	value[0][2] = col3.x;
	value[0][3] = col4.x;
	value[1][0] = col1.y;
	value[1][1] = col2.y;
	value[1][2] = col3.y;
	value[1][3] = col4.y;
	value[2][0] = col1.z;
	value[2][1] = col2.z;
	value[2][2] = col3.z;
	value[2][3] = col4.z;
	value[3][0] = col1.w;
	value[3][1] = col2.w;
	value[3][2] = col3.w;
	value[3][3] = col4.w;
	return *this;
}

double Matrix4::trace() const {
	return value[0][0] + value[1][1] + value[2][2] + value[3][3];
}

bool Matrix4::operator != (const Matrix4 &m) const {
	return	value[0][0] != m.value[0][0] || value[0][1] != m.value[0][1] || value[0][2] != m.value[0][2] || value[0][3] != m.value[0][3] || 
			value[1][0] != m.value[1][0] || value[1][1] != m.value[1][1] || value[1][2] != m.value[1][2] || value[1][3] != m.value[1][3] || 
			value[2][0] != m.value[2][0] || value[2][1] != m.value[2][1] || value[2][2] != m.value[2][2] || value[2][3] != m.value[2][3] || 
			value[3][0] != m.value[3][0] || value[3][1] != m.value[3][1] || value[3][2] != m.value[3][2] || value[3][3] != m.value[3][3];
}

Matrix4::Matrix4(double a00, double a01, double a02, double a03, double a10, double a11, double a12, double a13, double a20, double a21, double a22, double a23, double a30, double a31, double a32, double a33) {
	value[0][0] = a00;
	value[0][1] = a01;
	value[0][2] = a02;
	value[0][3] = a03;
	value[1][0] = a10;
	value[1][1] = a11;
	value[1][2] = a12;
	value[1][3] = a13;
	value[2][0] = a20;
	value[2][1] = a21;
	value[2][2] = a22;
	value[2][3] = a23;
	value[3][0] = a30;
	value[3][1] = a31;
	value[3][2] = a32;
	value[3][3] = a33;
}

double Matrix4::det() const {
	double det00 = value[2][2] * value[3][3] - value[2][3] * value[3][2];
	double det11 = value[2][0] * value[3][3] - value[2][3] * value[3][0];
	double det22 = value[2][0] * value[3][1] - value[2][1] * value[3][0];
	double det01 = value[2][1] * value[3][3] - value[2][3] * value[3][1];
	double det02 = value[2][1] * value[3][2] - value[2][2] * value[3][1];
	double det12 = value[2][0] * value[3][2] - value[2][2] * value[3][0];
	return	value[0][0] * (value[1][1] * det00 - value[1][2] * det01 + value[1][3] * det02) -
			value[0][1] * (value[1][0] * det00 - value[1][2] * det11 + value[1][3] * det12) + 
			value[0][2] * (value[1][0] * det01 - value[1][1] * det11 + value[1][3] * det22) - 
			value[0][3] * (value[1][0] * det02 - value[1][1] * det12 + value[1][2] * det22);
}

#define det3(a00, a01, a02, a10, a11, a12, a20, a21, a22) (a00 * a11 * a22 + a01 * a12 * a20 + a02 * a10 * a21 - a02 * a11 * a20 - a01 * a10 * a22 - a00 * a12 * a21)

Matrix4 Matrix4::inverse() const {
	Matrix4 m;

	double determinant = det();
	if (determinant == 0.0) throw cvgException("[Matrix4] The matrix inverse does not exist");
	double inv_det = 1.0 / determinant;

	// Calculate determinant matrix already transposed to save speed

	m.value[0][0] = det3(value[1][1], value[1][2], value[1][3], value[2][1], value[2][2], value[2][3], value[3][1], value[3][2], value[3][3]) * inv_det;
	m.value[1][0] = -det3(value[1][0], value[1][2], value[1][3], value[2][0], value[2][2], value[2][3], value[3][0], value[3][2], value[3][3]) * inv_det;
	m.value[2][0] = det3(value[1][0], value[1][1], value[1][3], value[2][0], value[2][1], value[2][3], value[3][0], value[3][1], value[3][3]) * inv_det;
	m.value[3][0] = -det3(value[1][0], value[1][1], value[1][2], value[2][0], value[2][1], value[2][2], value[3][0], value[3][1], value[3][2]) * inv_det;

	m.value[0][1] = -det3(value[0][1], value[0][2], value[0][3], value[2][1], value[2][2], value[2][3], value[3][1], value[3][2], value[3][3]) * inv_det;
	m.value[1][1] = det3(value[0][0], value[0][2], value[0][3], value[2][0], value[2][2], value[2][3], value[3][0], value[3][2], value[3][3]) * inv_det;
	m.value[2][1] = -det3(value[0][0], value[0][1], value[0][3], value[2][0], value[2][1], value[2][3], value[3][0], value[3][1], value[3][3]) * inv_det;
	m.value[3][1] = det3(value[0][0], value[0][1], value[0][2], value[2][0], value[2][1], value[2][2], value[3][0], value[3][1], value[3][2]) * inv_det;

	m.value[0][2] = det3(value[0][1], value[0][2], value[0][3], value[1][1], value[1][2], value[1][3], value[3][1], value[3][2], value[3][3]) * inv_det;
	m.value[1][2] = -det3(value[0][0], value[0][2], value[0][3], value[1][0], value[1][2], value[1][3], value[3][0], value[3][2], value[3][3]) * inv_det;
	m.value[2][2] = det3(value[0][0], value[0][1], value[0][3], value[1][0], value[1][1], value[1][3], value[3][0], value[3][1], value[3][3]) * inv_det;
	m.value[3][2] = -det3(value[0][0], value[0][1], value[0][2], value[1][0], value[1][1], value[1][2], value[3][0], value[3][1], value[3][2]) * inv_det;

	m.value[0][3] = -det3(value[0][1], value[0][2], value[0][3], value[1][1], value[1][2], value[1][3], value[2][1], value[2][2], value[2][3]) * inv_det;
	m.value[1][3] = det3(value[0][0], value[0][2], value[0][3], value[1][0], value[1][2], value[1][3], value[2][0], value[2][2], value[2][3]) * inv_det;
	m.value[2][3] = -det3(value[0][0], value[0][1], value[0][3], value[1][0], value[1][1], value[1][3], value[2][0], value[2][1], value[2][3]) * inv_det;
	m.value[3][3] = det3(value[0][0], value[0][1], value[0][2], value[1][0], value[1][1], value[1][2], value[2][0], value[2][1], value[2][2]) * inv_det;

	return m;
}

#undef det3

Matrix4 Matrix4::operator * (double f) const {
	return Matrix4(	value[0][0] * f, value[0][1] * f, value[0][2] * f, value[0][3] * f,
					value[1][0] * f, value[1][1] * f, value[1][2] * f, value[1][3] * f,
					value[2][0] * f, value[2][1] * f, value[2][2] * f, value[2][3] * f,
					value[3][0] * f, value[3][1] * f, value[3][2] * f, value[3][3] * f
					);
}

Matrix4 Matrix4::operator / (double d) const {
	return Matrix4(	value[0][0] / d, value[0][1] / d, value[0][2] / d, value[0][3] / d,
					value[1][0] / d, value[1][1] / d, value[1][2] / d, value[1][3] / d,
					value[2][0] / d, value[2][1] / d, value[2][2] / d, value[2][3] / d,
					value[3][0] / d, value[3][1] / d, value[3][2] / d, value[3][3] / d
					);
}

cvgString Matrix4::toString() const {
	return 	cvgString(
			value[0][0]) + '\t' + value[0][1] + '\t' + value[0][2] + '\t' + value[0][3] + '\n' +
			value[1][0]  + '\t' + value[1][1] + '\t' + value[1][2] + '\t' + value[1][3] + '\n' +
			value[2][0]  + '\t' + value[2][1] + '\t' + value[2][2] + '\t' + value[2][3] + '\n' +
			value[3][0]  + '\t' + value[3][1] + '\t' + value[3][2] + '\t' + value[3][3];
}

Matrix4 Matrix4::scalarPow(double e) const {
	return Matrix4(	pow(value[0][0], e), pow(value[0][1], e), pow(value[0][2], e), pow(value[0][3], e),
					pow(value[1][0], e), pow(value[1][1], e), pow(value[1][2], e), pow(value[1][3], e),
					pow(value[2][0], e), pow(value[2][1], e), pow(value[2][2], e), pow(value[2][3], e),
					pow(value[3][0], e), pow(value[3][1], e), pow(value[3][2], e), pow(value[3][3], e)
					);
}

Matrix4 Matrix4::scalarSqrt() const {
	return Matrix4(	sqrt(value[0][0]), sqrt(value[0][1]), sqrt(value[0][2]), sqrt(value[0][3]),
					sqrt(value[1][0]), sqrt(value[1][1]), sqrt(value[1][2]), sqrt(value[1][3]),
					sqrt(value[2][0]), sqrt(value[2][1]), sqrt(value[2][2]), sqrt(value[2][3]),
					sqrt(value[3][0]), sqrt(value[3][1]), sqrt(value[3][2]), sqrt(value[3][3])
					);
}

cvg_uint Matrix4::countValues(double v) const {
	cvg_uint count = 0;
	for(cvg_int y = 0; y < 4; y++)
		for(cvg_int x = 0; x < 4; x++)
			if (value[y][x] == v) count++;
	return count;
}

Matrix4 Matrix4::operator + (const Matrix4 &m) const {
	Matrix4 out = *this;
	for (cvg_int y = 0; y < 4; y++)
		for (cvg_int x = 0; x < 4; x++)
			out.value[y][x] += m.value[y][x];
	return out;
}

Matrix4 Matrix4::operator - (const Matrix4 &m) const {
	Matrix4 out = *this;
	for (cvg_int y = 0; y < 4; y++)
		for (cvg_int x = 0; x < 4; x++)
			out.value[y][x] -= m.value[y][x];
	return out;
}

Matrix4 Matrix4::operator < (const Matrix4 &m) const {
	Matrix4 out;
	for (cvg_int y = 0; y < 4; y++)
		for (cvg_int x = 0; x < 4; x++)
			out.value[y][x] = value[y][x] < m.value[y][x] ? 1.0 : 0.0;
	return out;
}

Matrix4 Matrix4::operator < (double t) const {
	Matrix4 out;
	for (cvg_int y = 0; y < 4; y++)
		for (cvg_int x = 0; x < 4; x++)
			out.value[y][x] = value[y][x] < t ? 1.0 : 0.0;
	return out;
}

Matrix4 Matrix4::operator > (const Matrix4 &m) const {
	Matrix4 out;
	for (cvg_int y = 0; y < 4; y++)
		for (cvg_int x = 0; x < 4; x++)
			out.value[y][x] = value[y][x] > m.value[y][x] ? 1.0 : 0.0;
	return out;
}

Matrix4 Matrix4::operator > (double t) const {
	Matrix4 out;
	for (cvg_int y = 0; y < 4; y++)
		for (cvg_int x = 0; x < 4; x++)
			out.value[y][x] = value[y][x] > t ? 1.0 : 0.0;
	return out;
}
