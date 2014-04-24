/*
 * HTMatrix4.cpp
 *
 *  Created on: 11/09/2011
 *      Author: Ignacio Mellado
 */

#include <HTMatrix4.h>

HTMatrix4::HTMatrix4(const RotMatrix3 &rotation, const Vector3 &translation) {
	value[0][0] = rotation.value[0][0];
	value[0][1] = rotation.value[0][1];
	value[0][2] = rotation.value[0][2];
	value[0][3] = translation.x;
	value[1][0] = rotation.value[1][0];
	value[1][1] = rotation.value[1][1];
	value[1][2] = rotation.value[1][2];
	value[1][3] = translation.y;
	value[2][0] = rotation.value[2][0];
	value[2][1] = rotation.value[2][1];
	value[2][2] = rotation.value[2][2];
	value[2][3] = translation.z;
	value[3][0] = 0.0;
	value[3][1] = 0.0;
	value[3][2] = 0.0;
	value[3][3] = 1.0;
}
