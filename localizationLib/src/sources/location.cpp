// Location.cpp: implementation of the Location class.
//
//////////////////////////////////////////////////////////////////////

#include "location.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Location::Location()
{
	vect[0]=vect[1]=vect[2]=0.0;
	vect[3]=1.0;
}

Location::~Location()
{

}
Location::Location(double x,double y,double z)
{
	vect[0]=x;
	vect[1]=y;
	vect[2]=z;
	vect[3]=1.0;
}
