// Location.h: interface for the Location class.
//
//////////////////////////////////////////////////////////////////////

#pragma once

#if !defined(AFX_LOCATION_H__0B7A5EDE_C26B_42E8_A513_19A98A626670__INCLUDED_)
#define AFX_LOCATION_H__0B7A5EDE_C26B_42E8_A513_19A98A626670__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class Location  
{
public:
	Location();
	Location(double x,double y,double z);
	virtual ~Location();

	double vect[4];

};

#endif // !defined(AFX_LOCATION_H__0B7A5EDE_C26B_42E8_A513_19A98A626670__INCLUDED_)
