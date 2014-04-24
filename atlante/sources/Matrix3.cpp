/*
 * Matrix3.cpp
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#include <Matrix3.h>
#include <Vector3.h>
#include <cvgException.h>

Matrix3 &Matrix3::setRows(const Vector3 &row1, const Vector3 &row2, const Vector3 &row3) {
	value[0][0] = row1.x;
	value[0][1] = row1.y;
	value[0][2] = row1.z;
	value[1][0] = row2.x;
	value[1][1] = row2.y;
	value[1][2] = row2.z;
	value[2][0] = row3.x;
	value[2][1] = row3.y;
	value[2][2] = row3.z;
	return *this;
}

Matrix3 &Matrix3::setCols(const Vector3 &col1, const Vector3 &col2, const Vector3 &col3) {
	value[0][0] = col1.x;
	value[0][1] = col2.x;
	value[0][2] = col3.x;
	value[1][0] = col1.y;
	value[1][1] = col2.y;
	value[1][2] = col3.y;
	value[2][0] = col1.z;
	value[2][1] = col2.z;
	value[2][2] = col3.z;
	return *this;
}

Matrix3 Matrix3::transpose() const {
	Matrix3 m;
	for (int y = 0; y < 3; y ++)
		for (int x = 0; x < 3; x ++)
			m.value[y][x] = value[x][y];
	return m;
}

Matrix3 &Matrix3::operator = (const Matrix3 &m) {
	value[0][0] = m.value[0][0];
	value[0][1] = m.value[0][1];
	value[0][2] = m.value[0][2];
	value[1][0] = m.value[1][0];
	value[1][1] = m.value[1][1];
	value[1][2] = m.value[1][2];
	value[2][0] = m.value[2][0];
	value[2][1] = m.value[2][1];
	value[2][2] = m.value[2][2];
	return *this;
}

double Matrix3::det() const {
	return	value[0][0] * value[1][1] * value[2][2] +
			value[0][1] * value[1][2] * value[2][0] +
			value[0][2] * value[1][0] * value[2][1] -
			value[0][2] * value[1][1] * value[2][0] -
			value[0][1] * value[1][0] * value[2][2] -
			value[0][0] * value[1][2] * value[2][1];
}

Matrix3::Matrix3(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22) {
	value[0][0] = a00;
	value[0][1] = a01;
	value[0][2] = a02;
	value[1][0] = a10;
	value[1][1] = a11;
	value[1][2] = a12;
	value[2][0] = a20;
	value[2][1] = a21;
	value[2][2] = a22;
}

Matrix3::Matrix3(const Matrix3 &m) {
	*this = m;
}

Matrix3 Matrix3::operator * (double f) const {
	return Matrix3(	value[0][0] * f, value[0][1] * f, value[0][2] * f,
					value[1][0] * f, value[1][1] * f, value[1][2] * f,
					value[2][0] * f, value[2][1] * f, value[2][2] * f
					);
}

Matrix3 Matrix3::operator / (double d) const {
	return Matrix3(	value[0][0] / d, value[0][1] / d, value[0][2] / d,
					value[1][0] / d, value[1][1] / d, value[1][2] / d,
					value[2][0] / d, value[2][1] / d, value[2][2] / d
					);
}

#define det2(a00, a01, a10, a11) (a00 * a11 - a01 * a10)

Matrix3 Matrix3::inverse() const {
	Matrix3 m;

	double determinant = det();
	if (determinant == 0.0) throw cvgException("[Matrix3] The matrix inverse does not exist");
	double inv_det = 1.0 / determinant;

	// Calculate determinant matrix already transposed to increase speed
	// Negative subdeterminants are computed exchanging columns for speed
	m.value[0][0] = det2(value[1][1], value[1][2], value[2][1], value[2][2]) * inv_det;
	m.value[0][1] = det2(value[0][2], value[0][1], value[2][2], value[2][1]) * inv_det;
	m.value[0][2] = det2(value[0][1], value[0][2], value[1][1], value[1][2]) * inv_det;

	m.value[1][0] = det2(value[1][2], value[1][0], value[2][2], value[2][0]) * inv_det;
	m.value[1][1] = det2(value[0][0], value[0][2], value[2][0], value[2][2]) * inv_det;
	m.value[1][2] = det2(value[0][2], value[0][0], value[1][2], value[1][0]) * inv_det;

	m.value[2][0] = det2(value[1][0], value[1][1], value[2][0], value[2][1]) * inv_det;
	m.value[2][1] = det2(value[0][1], value[0][0], value[2][1], value[2][0]) * inv_det;
	m.value[2][2] = det2(value[0][0], value[0][1], value[1][0], value[1][1]) * inv_det;

	return m;
}

#undef det2

Matrix3 Matrix3::operator < (const Matrix3 &m) const {
	Matrix3 out;
	for (cvg_int y = 0; y < 3; y++)
		for (cvg_int x = 0; x < 3; x++)
			out.value[y][x] = value[y][x] < m.value[y][x] ? 1.0 : 0.0;
	return out;
}

Matrix3 Matrix3::operator < (double t) const {
	Matrix3 out;
	for (cvg_int y = 0; y < 3; y++)
		for (cvg_int x = 0; x < 3; x++)
			out.value[y][x] = value[y][x] < t ? 1.0 : 0.0;
	return out;
}

Matrix3 Matrix3::operator > (const Matrix3 &m) const {
	Matrix3 out;
	for (cvg_int y = 0; y < 3; y++)
		for (cvg_int x = 0; x < 3; x++)
			out.value[y][x] = value[y][x] > m.value[y][x] ? 1.0 : 0.0;
	return out;
}

Matrix3 Matrix3::operator > (double t) const {
	Matrix3 out;
	for (cvg_int y = 0; y < 3; y++)
		for (cvg_int x = 0; x < 3; x++)
			out.value[y][x] = value[y][x] > t ? 1.0 : 0.0;
	return out;
}

cvg_uint Matrix3::countValues(double v) const {
	cvg_uint count = 0;
	for(cvg_int y = 0; y < 3; y++)
		for(cvg_int x = 0; x < 3; x++)
			if (value[y][x] == v) count++;
	return count;
}

cvgString Matrix3::toString() const {
	return 	cvgString(
			value[0][0]) + '\t' + value[0][1] + '\t' + value[0][2] + '\n' +
			value[1][0]  + '\t' + value[1][1] + '\t' + value[1][2] + '\n' +
			value[2][0]  + '\t' + value[2][1] + '\t' + value[2][2];
}

Vector3 Matrix3::operator * (const Vector3 &v) const {
	return Vector3(	value[0][0] * v.x + value[0][1] * v.y + value[0][2] * v.z, 
			value[1][0] * v.x + value[1][1] * v.y + value[1][2] * v.z,
			value[2][0] * v.x + value[2][1] * v.y + value[2][2] * v.z
			);
}

Matrix3 Matrix3::operator *(const Matrix3 &m) const {
	Matrix3 matrix;
	matrix.value[0][0] = value[0][0] * m.value[0][0] + value[0][1] * m.value[1][0] + value[0][2] * m.value[2][0];
	matrix.value[0][1] = value[0][0] * m.value[0][1] + value[0][1] * m.value[1][1] + value[0][2] * m.value[2][1];
	matrix.value[0][2] = value[0][0] * m.value[0][2] + value[0][1] * m.value[1][2] + value[0][2] * m.value[2][2];
	matrix.value[1][0] = value[1][0] * m.value[0][0] + value[1][1] * m.value[1][0] + value[1][2] * m.value[2][0];
	matrix.value[1][1] = value[1][0] * m.value[0][1] + value[1][1] * m.value[1][1] + value[1][2] * m.value[2][1];
	matrix.value[1][2] = value[1][0] * m.value[0][2] + value[1][1] * m.value[1][2] + value[1][2] * m.value[2][2];
	matrix.value[2][0] = value[2][0] * m.value[0][0] + value[2][1] * m.value[1][0] + value[2][2] * m.value[2][0];
	matrix.value[2][1] = value[2][0] * m.value[0][1] + value[2][1] * m.value[1][1] + value[2][2] * m.value[2][1];
	matrix.value[2][2] = value[2][0] * m.value[0][2] + value[2][1] * m.value[1][2] + value[2][2] * m.value[2][2];
	return matrix;	
}

