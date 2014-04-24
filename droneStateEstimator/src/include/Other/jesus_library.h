/*
 * jesus_library.h
 *
 *  Created on: Apr 25, 2012
 *      Author: jespestana
 */

#ifndef JESUS_LIBRARY_H_
#define JESUS_LIBRARY_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

//Deleted atlante dependency
//#include <atlante.h>

#include "matrixLib.h"


namespace jesus_library {
	double interpolate(double x, Vector *x_val, Vector *y_val);
	double saturate(double x, double x_lim_inf, double x_lim_sup);
//	double mapAngleToMinusPlusPi(double angleRads);
//	std::string convertInt(int number);

	// cambia angleRads2Move si ambos angulos son fisicamente cercanos pero caen uno por un lado de -pi y el otro por el otro lado de pi
	double mapAnglesToBeNear_PIrads(double angleRads2move, double angleRads);


	// Vector related functions
	void flipVector( Vector &v, int sign);
	int fmod(int numerator, int denominator);
//	void unitaryVectorFrom2Points( Vector &ur, Vector &p0, Vector &p1);
	double unitaryVectorFrom2Points( Vector &ur, Vector &p0, Vector &p1); // from p0 to p1
	double normOfVector( Vector &r_p);
	void getVectorComponents( Vector &u, double &ux, double &uy, double &uz);
	void setVectorComponents( Vector &u, double &ux, double &uy, double &uz);
	double dotProduct( Vector &u, Vector&v);
	void crossProduct( Vector &result, Vector &u, Vector &v);
	void unitarizeVector( Vector &u);
	void multiplyDoubleVsVector( double c, Vector &u);
	double distanceAlongLine( Vector &p0, Vector &p, Vector &r_ur);
	double calculateVmax( Vector &r_ur, double vmax_xy, double vmax_z);
	void mostrarVector( std::string nombreVector, Vector &u);
};

#endif /* JESUS_LIBRARY_H_ */
