#include "stdafx.h"
#include "mathclass.h"

complex_old operator-(complex_old const& a)
{
    return complex_old( -a.real, -a.imag);
}

complex_old inverse(complex_old const& a)
{
    return complex_old( a.real, -a.imag);
}

complex_old operator+ (complex_old const& a, complex_old const& b)
{
    return	complex_old(a.real + b.real, a.imag + b.imag);
}

complex_old operator- (complex_old const& a, complex_old const& b)
{
	return	complex_old(a.real - b.real, a.imag - b.imag);
}

complex_old operator* (m_real a, complex_old const& b)
{
	return	complex_old (a * b.real, a * b.imag);
}

complex_old operator* (complex_old const& a, m_real b)
{
    return	complex_old(a.real * b, a.imag * b);
}

complex_old operator/ (complex_old const& a, m_real b)
{
	return	complex_old(a.real / b, a.imag / b);
}

m_real operator% (complex_old const& a, complex_old const& b)
{
	return	(a.real * b.real + a.imag * b.imag);
}

complex_old operator* (complex_old const& a, complex_old const& b)
{
    complex_old c;
    c.real = a.real*b.real - a.imag*b.imag;
    c.imag = a.real*b.imag + a.imag*b.real;
    return c;
}

m_real len( complex_old const& v )
{
    return sqrt( v.real*v.real + v.imag*v.imag );
}

m_real
complex_old::length() const
{
    return sqrt( real*real + imag*imag );
}

complex_old
complex_old::normalize() const
{
	return (*this)/this->length();
}

/*
ostream& operator<<( ostream& os, complex_old const& a )
{
    os << "( " << a.real << " , " << a.imag << " )";
    return os;
}

istream& operator>>( istream& is, complex_old& a )
{
	static char	buf[256];
    //is >> "(" >> a.real >> "," >> a.imag >> ")";
	is >> buf >> a.real >> buf >> a.imag >> buf;
    return is;
}
*/
complex_old c_exp(m_real theta)
{
	return complex_old(cos(theta), sin(theta));
}

m_real c_ln(complex_old const& c)
{
	return	atan2(c.imag, c.real);
}

complex_old
slerp( complex_old const& a, complex_old const& b, m_real t )
{
	m_real c = a % b;

	if ( 1.0+c > EPS )
	{
		if ( 1.0-c > EPS )
		{
			m_real theta = acos( c );
			m_real sinom = sin( theta );
			return ( a*sin((1.0f-t)*theta) + b*sin(t*theta) ) / sinom;
		}
		else
			return (a*(1.0f-t) + b*t).normalize();
	}
	else	return a*sin((0.5f-t)*M_PI) + b*sin(t*M_PI);
}

complex_old
interpolate( m_real t, complex_old const& a, complex_old const& b )
{
	return slerp( a, b, t );
}

m_real
distance( complex_old const& a, complex_old const& b )
{
	return MIN( fabs(c_ln( a.inverse()* b)), fabs(c_ln( a.inverse()*-b)) );
}

m_real
difference( complex_old const& a, complex_old const& b )
{
	return c_ln( b.inverse() * a );
}
