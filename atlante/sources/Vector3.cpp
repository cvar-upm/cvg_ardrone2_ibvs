/*
 * Vector3.h
 *
 *  Created on: 10/09/2011
 *      Author: Ignacio Mellado
 */

#include <Vector3.h>
#include <Matrix3.h>
#include <math.h>

Vector3::~Vector3() {
}

Vector3 &Vector3::operator = (const Vector3 &v) {
	x = v.x;
	y = v.y;
	z = v.z;
	return *this;
}

Vector3 Vector3::operator + (const Vector3 &v) const {
	return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator - (const Vector3 &v) const {
	return Vector3(x - v.x, y - v.y, z - v.z);
}

double Vector3::operator * (const Vector3 &v) const {
	return x * v.x + y * v.y + z * v.z;
}

Vector3 Vector3::operator * (double f) const {
	return Vector3(x * f, y * f, z * f);
}

Vector3 Vector3::operator / (double d) const {
	return Vector3(x / d, y / d, z / d);
}

Vector3 Vector3::operator * (const Matrix3 &m) const {
	return Vector3(	m.value[0][0] * x + m.value[1][0] * y + m.value[2][0] * z,
					m.value[0][1] * x + m.value[1][1] * y + m.value[2][1] * z,
					m.value[0][2] * x + m.value[1][2] * y + m.value[2][2] * z);
}

Vector3 Vector3::operator % (const Vector3 &v) const {
	return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

Vector3 Vector3::normalize() const {
	double mod = modulus();
	if (mod == 0.0) return Vector3(*this);
	return Vector3(x / mod, y / mod, z / mod);
}

double Vector3::modulus() const {
	return sqrt(x * x + y * y + z * z);
}

Vector3 &Vector3::operator += (const Vector3 &v) {
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

Vector3 &Vector3::operator -= (const Vector3 &v) {
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

Vector3 Vector3::rotate(const Vector3 &point, const Vector3 &axis, double angle) const {
	// Build an orthonormal frame (u, v, w) from the point and axis
	Vector3 w = axis.normalize();
	Vector3 A = point + (w * ((*this - point) * w));
	Vector3 v = *this - A;
	Vector3 u = v % w;

	// In the new reference frame, the vector is (0, |v|, 0). We use the rotation matrix to rotate it.
	Vector3 nv = Vector3(0.0, v.modulus(), 0.0) * Matrix3(Vector3(cos(angle), sin(angle), 0.0), Vector3(-sin(angle), cos(angle), 0.0), Vector3(0.0, 0.0, 1.0));

	// Volvemos a la base anterior con la matriz de cambio
	return nv * Matrix3(u.normalize(), v.normalize(), w) + A;
}

bool Vector3::operator != (const Vector3 &v) const {
	return !(*this == v);
}

bool Vector3::operator == (const Vector3 &v) const {
	return x == v.x && y == v.y && z == v.z;
}

Vector3 Vector3::operator - () const {
	return Vector3(-x, -y, -z);
}

Vector3 Vector3::operator > (const Vector3 &v) const {
	return Vector3(x > v.x, y > v.y, z > v.z);
}

Vector3 Vector3::operator > (double t) const {
	return Vector3(x > t, y > t, z > t);
}

Vector3 Vector3::operator < (const Vector3 &v) const {
	return Vector3(x < v.x, y < v.y, z < v.z);
}

Vector3 Vector3::operator < (double t) const {
	return Vector3(x < t, y < t, z < t);
}

cvg_uint Vector3::countValues(double v) const {
	cvg_uint count = 0;
	if (x == v) count++;
	if (y == v) count++;
	if (z == v) count++;
	return count;
}

