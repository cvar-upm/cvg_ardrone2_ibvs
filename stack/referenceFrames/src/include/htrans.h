// HomogTrans.h: interface for the HomogTrans class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_HOMOGTRANS_H__FBE48A38_AA84_4945_AC87_3586E23C5F2D__INCLUDED_)
#define AFX_HOMOGTRANS_H__FBE48A38_AA84_4945_AC87_3586E23C5F2D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "location.h"

#define PI 3.14159265358979
#define DEG2RAD 0.0174532925199
#define RAD2DEG 57.2957795130823
enum eje{EJE_X,EJE_Y,EJE_Z};

class HomogTrans
{
public:
	//Constructors
	HomogTrans();//default= Identity transformation
	HomogTrans(const Location& loc);//pure translational transformation
	HomogTrans(int eje,double ang);//pure rotational transformation along 1 principal axis, ang in RAD
	HomogTrans(double roll,double pitch,double yaw);//Roll, pitch and yaw pure rotational transform
	virtual ~HomogTrans();

	//complete set
	void Unit(); //set trans to Identity
		
	//composition operators
	void Rotate(int eje,double ang);//rotate current matrix along axis, post-mult
	void Translate(double  x,double  y,double  z); 

	Location GetOrigin() const;

	void RollPitchYaw(double roll,double pitch,double yaw);
	Location operator*(const Location& l)const;
	HomogTrans operator*(const HomogTrans& h)const;

	void DirectTrans(double x,double y,double z,double roll, double pitch, double yaw);
	void InvTrans(double x,double y,double z,double roll, double pitch, double yaw);
	double mat[4][4];
};

#endif // !defined(AFX_HOMOGTRANS_H__FBE48A38_AA84_4945_AC87_3586E23C5F2D__INCLUDED_)
