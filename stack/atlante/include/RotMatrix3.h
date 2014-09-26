/*
 * RotMatrix.h
 *
 *  Created on: 11/09/2011
 *      Author: Ignacio Mellado
 */

#ifndef ROTMATRIX_H_
#define ROTMATRIX_H_

#include "Matrix3.h"
#include "Vector3.h"

class RotMatrix3 : public virtual Matrix3 {
public:
	inline RotMatrix3() { }
	inline RotMatrix3(double a00, double a01, double a02, double a10, double a11, double a12, double a20, double a21, double a22)
			: Matrix3(a00, a01, a02, a10, a11, a12, a20, a21, a22) { }
	inline RotMatrix3(const Matrix3 &m) : Matrix3(m) { }
	RotMatrix3(const Vector3 &axis, double angle);
	inline virtual ~RotMatrix3() { }

	inline virtual Matrix3 inverse() const { return transpose(); }
	Vector3 getEulerAnglesZYX(cvg_bool thetaInQuadrants_I_IV = true) const;
};

#endif /* ROTMATRIX_H_ */
