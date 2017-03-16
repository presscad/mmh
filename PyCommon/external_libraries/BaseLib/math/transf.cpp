#include "stdafx.h"
#include "mathclass.h"
#include "transf.h"


void transf::identity( )
{
	rotation.identity();
	translation.setValue(0,0,0);
}


transf
transf::inverse() const
{
    quater a = rotation.inverse();
	vector3 ti;
	ti.rotate(a, translation);
    return transf( a, -ti);
}

void transf::interpolate( m_real t, transf const& a, transf const& b )
{
	rotation.interpolate(t,a.rotation,b.rotation);
	translation.interpolate(t,a.translation,b.translation);
}

/*
transf rotate_transf( m_real angle, vector const& axis )
{
    return transf( exp( angle * axis / 2.0 ), vector(0,0,0) );
}

transf translate_transf( vector const& axis )
{
    return transf( quater(1,0,0,0), axis );
}

transf translate_transf( m_real x, m_real y, m_real z )
{
    return transf( quater(1,0,0,0), vector(x,y,z) );
}
*/


//--------------------------------------------------------------------------//

vector3& operator*=( vector3& a, transf const& b )
{
	a.rotate(b.rotation);
	a+=b.translation;
	return a;
}

vector3 operator*( transf const& b , vector3 const& a)
{
	vector3 c;
	c.rotate(b.rotation, a);
	c+=b.translation;
	return c;
}

transf operator*( transf const& a, transf const& b )
{
	transf c;
	c.mult(a,b);
	return c;
}

void transf::operator=(matrix4 const& a)
{
	rotation.setRotation(a); 
	translation.translation(a);
}

void transf::mult(transf const& a, transf const& b )
{
	rotation.mult(a.rotation, b.rotation);
	translation.rotate(a.rotation, b.translation);
	translation+=a.translation;
}
// this=a*this;
void transf::leftMult(const transf& a)
{
	translation.rotate(a.rotation);
	translation+=a.translation;
	rotation=a.rotation*rotation;
}

void transf::operator*=(const transf& b)
{
	transf a=*this;
	this->mult(a,b);
}
