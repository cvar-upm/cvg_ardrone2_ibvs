/*
 * jesus_library.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: jespestana
 */

#include "jesus_library.h"

#define PI 3.14159265

double jesus_library::interpolate(double x, Vector *x_val, Vector *y_val) {

	int num_x = x_val->length();
	int num_y = y_val->length();

	if (num_x != num_y) {
//		exit(); // here I should report an error...
	}

	double y = 0.0;
	if (x < x_val->getValueData(1)) {
		y = y_val->getValueData(1);
	} else if ( x > x_val->getValueData( x_val->length()) ) {
		y = y_val->getValueData( x_val->length());
	} else {
		int i;
		for (i = x_val->length()-1; i >= 1; i--) {
			if (x >= x_val->getValueData(i))  {
				y = y_val->getValueData(i) + (x-x_val->getValueData(i))*(y_val->getValueData(i+1)-y_val->getValueData(i))/(x_val->getValueData(i+1)-x_val->getValueData(i));
				return y;
			}
		}
	}

	return y;
}

double jesus_library::saturate(double x, double x_lim_inf, double x_lim_sup) {

	double y = 0.0;

	if (x > x_lim_sup) {
		y = x_lim_sup;
		return y;
	} else if (x < x_lim_inf) {
		y = x_lim_inf;
		return y;
	} else {
		y = x;
		return y;
	}

}

//double jesus_library::mapAngleToMinusPlusPi(double angleRads) {
//	double mapped = fmod(angleRads, M_PI);
//	int turns = (int)floor(angleRads / M_PI);
//	if (turns & 1) {
//		if (turns > 0)
//			mapped -= M_PI;
//		else
//			mapped += M_PI;
//	}
//	return mapped;
//}

double jesus_library::mapAnglesToBeNear_PIrads(double angleRads2move, double angleRads) {

	double pos1 = angleRads2move - 2*PI;
	double pos2 = angleRads2move;
	double pos3 = angleRads2move + 2*PI;

	double diff1 = fabs(pos1 - angleRads);
	double diff2 = fabs(pos2 - angleRads);
	double diff3 = fabs(pos3 - angleRads);

	if ( (diff1 < diff2) && (diff1 < diff3) ) {
		return pos1;
	} else { if ( diff2 < diff3 ) {
			return pos2;
		} else {
			return pos3;
		}
	}

}

double jesus_library::distanceAlongLine( Vector &p0, Vector &p, Vector &r_ur) {
	// coordinates of line origin/initial point
	double x0 = p0.getValueData(1);
	double y0 = p0.getValueData(2);
	double z0 = p0.getValueData(3);

	// actual position
	double x = p.getValueData(1);
	double y = p.getValueData(2);
	double z = p.getValueData(3);

	// Vector indicating axis/line direction
	double ur_x = r_ur.getValueData(1);
	double ur_y = r_ur.getValueData(2);
	double ur_z = r_ur.getValueData(3);

	double s = ((x-x0)*ur_x + (y-y0)*ur_y + (z-z0)*ur_z)/sqrt(ur_x*ur_x + ur_y*ur_y + ur_z*ur_z);

	return s;
}

void jesus_library::flipVector( Vector &v, int sign) {

	int N = v.length();
	Vector v_aux( N );
	v_aux.copy(&v);
	sign = (sign>0) ? 1 : -1;


	for (int i=1; i <= N; i++) {
		v.setValueData( v_aux.getValueData( (N+1)-i )*sign, i);
	}

}

int jesus_library::fmod(int numerator, int denominator) {
	int quotient = numerator % denominator;
	if ( quotient < 0 )
		 quotient = quotient + denominator;
	return quotient;
}

double jesus_library::normOfVector( Vector &r_p) {

	double x = r_p.getValueData(1);
	double y = r_p.getValueData(2);
	double z = r_p.getValueData(3);

	return sqrt( pow(x,2) + pow(y,2) + pow(z,2) );
}

double jesus_library::unitaryVectorFrom2Points( Vector &ur, Vector &p0, Vector &p1) { // from p0 to p1

	ur.substraction( &p1, &p0);
	double deltaL = normOfVector(ur);
	ur.setValueData( ur.getValueData(1)/deltaL, 1);
	ur.setValueData( ur.getValueData(2)/deltaL, 2);
	ur.setValueData( ur.getValueData(3)/deltaL, 3);

	return deltaL;
}

void jesus_library::getVectorComponents( Vector &u, double &ux, double &uy, double &uz) {

	ux = u.getValueData(1);
	uy = u.getValueData(2);
	uz = u.getValueData(3);
}

void jesus_library::setVectorComponents( Vector &u, double &ux, double &uy, double &uz) {

	u.setValueData( ux, 1);
	u.setValueData( uy, 2);
	u.setValueData( uz, 3);
}

double jesus_library::dotProduct( Vector &u, Vector&v){

	double ux,uy,uz;
	getVectorComponents( u, ux, uy, uz);
	double vx,vy,vz;
	getVectorComponents( v, vx, vy, vz);

	return  ( ux*vx + uy*vy + uz*vz);
}

void jesus_library::crossProduct( Vector &result, Vector &u, Vector &v) {

	double ux,uy,uz;
	getVectorComponents( u, ux, uy, uz);
	double vx,vy,vz;
	getVectorComponents( v, vx, vy, vz);

	if ( result.length() != 3 ) {
		result.deletion();
		result.creation(3);
	}
	result.setValueData( uy*vz-uz*vy, 1);
	result.setValueData( uz*vx-ux*vz, 2);
	result.setValueData( ux*vy-uy*vx, 3);
}

void jesus_library::unitarizeVector( Vector &u) {

	double norm_u = normOfVector(u);

	if (norm_u > 1e-3) {
		u.setValueData( u.getValueData(1)/norm_u, 1);
		u.setValueData( u.getValueData(2)/norm_u, 2);
		u.setValueData( u.getValueData(3)/norm_u, 3);
	} else {
		u.setZeros();
	}
}

void jesus_library::multiplyDoubleVsVector( double c, Vector &u) {

	u.setValueData( u.getValueData(1)*c, 1);
	u.setValueData( u.getValueData(2)*c, 2);
	u.setValueData( u.getValueData(3)*c, 3);
}

double jesus_library::calculateVmax( Vector &r_ur, double vmax_xy, double vmax_z) {

	if ( r_ur.length() != 3 )
		return -1.0;

	double ur_x, ur_y, ur_z;
	jesus_library::getVectorComponents( r_ur, ur_x, ur_y, ur_z);
	double ur_xy = sqrt( ur_x*ur_x + ur_y*ur_y );
	ur_z = fabs(ur_z);

	double v1 = vmax_xy/ur_xy;
	double v2 = vmax_z/ur_z;

	return (v1 < v2) ? v1 : v2;
}

void jesus_library::mostrarVector( std::string nombreVector, Vector &u) {
	std::cout << nombreVector << ": [ ";
	for (int i = 1; i <= u.length(); i++) {
		std::cout << u.getValueData(i);
		if ( i < u.length())
			std::cout << ", ";
	}
	std::cout << "]";
}

//std::string jesus_library::convertInt(int number)
//{
//   stringstream ss;//create a stringstream
//   ss << number;//add number to the stream
//   return ss.str();//return a string with the contents of the stream
//}
