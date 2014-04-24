// HomogTrans.cpp: implementation of the HomogTrans class.
//
//////////////////////////////////////////////////////////////////////

#include "htrans.h"
#include <math.h>
#include <cstring>
#include <iostream>

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

using namespace std;

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

HomogTrans::HomogTrans()
{
	Unit();
}

HomogTrans::~HomogTrans()
{

}
HomogTrans::HomogTrans(const Location& loc)
{
	Unit();
	mat[0][3]=loc.vect[0];
	mat[1][3]=loc.vect[1];
	mat[2][3]=loc.vect[2];
}
HomogTrans::HomogTrans(int eje,double ang)
{
	Unit();
	if(eje==EJE_X)//ang es alfa
	{
		mat[1][1]=cos(ang);
		mat[1][2]=-sin(ang);
		mat[2][1]=sin(ang);
		mat[2][2]=cos(ang);
	}
	if(eje==EJE_Y)//ang es fi
	{
		mat[0][0]=cos(ang);
		mat[0][2]=sin(ang);
		mat[2][0]=-sin(ang);
		mat[2][2]=cos(ang);
	}
	if(eje==EJE_Z)//ang es theta
	{
		mat[0][0]=cos(ang);
		mat[0][1]=-sin(ang);
		mat[1][0]=sin(ang);
		mat[1][1]=cos(ang);
	}
}
HomogTrans::HomogTrans(double roll, double pitch, double yaw)
{
	Unit();
	HomogTrans r(EJE_X,roll);
	HomogTrans p(EJE_Y,pitch);
	HomogTrans y(EJE_Z,yaw);
	(*this)=y*p*r;
}
void HomogTrans::Rotate(int eje,double ang)
{
	HomogTrans rot(eje,ang);
	(*this)=(*this)*rot;
}
void HomogTrans::Translate(double  x,double  y,double  z)
{
	Location loc(x,y,z);
	HomogTrans trans(loc);
	(*this)=(*this)*trans;
}
Location HomogTrans::operator*(const Location& l)const
{
	Location ret;
	for(int i=0;i<3;i++)
	{
		ret.vect[i]=0;
		for(int j=0;j<4;j++)
		{
			ret.vect[i]+=mat[i][j]*l.vect[j];
		}
	}
	return ret;
}
HomogTrans HomogTrans::operator*(const HomogTrans& h)const
{
	HomogTrans ret;
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<4;j++)
		{
			ret.mat[i][j]=0;
			for(int k=0;k<4;k++)
			{
				ret.mat[i][j]+=mat[i][k]*h.mat[k][j];
			}
		}
	}
	return ret;
}




void HomogTrans::Unit()
{
	int s=sizeof(mat);
	memset(mat,0,s);
	mat[0][0]=mat[1][1]=mat[2][2]=mat[3][3]=1;
}

void HomogTrans::RollPitchYaw(double roll, double pitch, double yaw)
{
	HomogTrans r(EJE_X,roll);
	HomogTrans p(EJE_Y,pitch);
	HomogTrans y(EJE_Z,yaw);
	(*this)=y*p*r;
}

void HomogTrans::DirectTrans(double ox,double oy,double oz,double r,double p,double y) //Roll pitch yaw ya desarrollada
{
	mat[0][0]=cos(r)*cos(p);mat[0][1]=cos(r)*sin(p)*sin(y)-sin(r)*cos(y);mat[0][2]=cos(r)*sin(p)*cos(y)+sin(r)*sin(y);mat[0][3]=ox;
	mat[1][0]=sin(r)*cos(p);mat[1][1]=sin(r)*sin(p)*sin(y)+cos(r)*cos(y);mat[1][2]=sin(r)*sin(p)*cos(y)-cos(r)*sin(y);mat[1][3]=oy;
	mat[2][0]=-sin(p);		mat[2][1]=cos(p)*sin(y);					 mat[2][2]=cos(p)*cos(y)					 ;mat[2][3]=oz;
	mat[3][0]=0;			mat[3][1]=0;								 mat[3][2]=0;								  mat[3][3]=1;

}

void HomogTrans::InvTrans(double ox,double oy,double oz,double r,double p,double y)
{
	mat[0][0]=cos(r)*cos(p);					    mat[0][1]=sin(r)*cos(p)						;		mat[0][2]=-sin(p);						mat[0][3]=-ox*cos(r)*cos(p)-oy*sin(r)*cos(p)+oz*sin(p);
	mat[1][0]=cos(r)*sin(p)*sin(y)-sin(r)*cos(y);  mat[1][1]=sin(r)*sin(p)*sin(y)+cos(r)*cos(y);		mat[1][2]=cos(p)*sin(y);				mat[1][3]=ox*(sin(r)*cos(y)-cos(r)*sin(p)*sin(y))-oy*(cos(r)*cos(y)+sin(p)*sin(r)*sin(y))-oz*cos(p)*sin(y);
	mat[2][0]=cos(r)*sin(p)*cos(y)+sin(r)*sin(y);	mat[2][1]=sin(r)*sin(p)*cos(y)-cos(r)*sin(y);		mat[2][2]=cos(p)*cos(y);				mat[2][3]=-ox*(sin(r)*sin(y)+cos(r)*sin(p)*cos(y))+oy*(cos(r)*sin(y)-sin(p)*sin(r)*cos(y))-oz*cos(p)*cos(y);
	mat[3][0]=0;									mat[3][1]=0;										mat[3][2]=0;							mat[3][3]=1;


}

Location HomogTrans::GetOrigin() const
{
	Location l;
	l.vect[0]=mat[0][3];
	l.vect[1]=mat[1][3];
	l.vect[2]=mat[2][3];
	return l;
}
