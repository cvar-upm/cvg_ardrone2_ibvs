/*
 * HTMatrix.h
 *
 *  Created on: 11/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef HTMATRIX_H_
#define HTMATRIX_H_

#include "Matrix4.h"
#include "RotMatrix3.h"
#include <string.h>

class HTMatrix4 : public virtual Matrix4 {
public:
	inline HTMatrix4() { }
	inline ~HTMatrix4() { }
	HTMatrix4(const RotMatrix3 &rotation, const Vector3 &translation);
	inline HTMatrix4(const Matrix4 &m) : Matrix4(m) { }

	inline RotMatrix3 getRotationMatrix() const {
		return RotMatrix3(value[0][0], value[0][1], value[0][2], value[1][0], value[1][1], value[1][2], value[2][0], value[2][1], value[2][2]);
	}
	inline Vector3 getTranslation() const {
		return Vector3(value[0][3], value[1][3], value[2][3]);
	}
};

#endif /* HTMATRIX_H_ */
