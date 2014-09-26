/*
 * RotMatrix3.cpp
 *
 *  Created on: 11/09/2011
 *      Author: Ignacio Mellado
 */

#include <Vector3.h>
#include <RotMatrix3.h>
#include <math.h>

RotMatrix3::RotMatrix3(const Vector3 &axis, double angle) {
	// Rotation matrix for a rotation of 'angle' radians around the axis 'axis'
	double ct = cos(angle); 
	double st = sin(angle);
	double vt = 1 - ct;
	double kx = axis.x, ky = axis.y, kz = axis.z;
	// De Rodrigues formula
	*this = RotMatrix3(	kx * kx * vt + ct, 	kx * ky * vt - kz * st,	kx * kz * vt + ky * st,
				kx * ky * vt + kz * st, ky * ky * vt + ct,	ky * kz * vt - kx * st,
				kx * kz * vt - ky * st,	ky * kz * vt + kx * st,	kz * kz * vt + ct
				);				
}

Vector3 RotMatrix3::getEulerAnglesZYX(cvg_bool thetaInQuadrants_I_IV) const {
	Vector3 output;
	if (value[2][0] != 1 && value[2][0] != -1) {
		output.y = -asin(value[2][0]);
		if (!thetaInQuadrants_I_IV) {
			output.y = M_PI - output.y;
		}
		double c_y = 1.0 / cos(output.y);
		output.x = atan2(value[2][1] * c_y, value[2][2] * c_y);
		output.z = atan2(value[1][0] * c_y, value[0][0] * c_y);
	} else {
		output.z = 0.0;
		if (value[2][0] == -1) {
			output.y = M_PI / 2;
			output.x = output.z + atan2(value[0][1], value[0][2]);
		} else {
			output.y = -M_PI / 2;
			output.x = -output.z + atan2(-value[0][1], -value[0][2]);
		}
	}
	return output;
}
