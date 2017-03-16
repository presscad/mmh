#include "stdafx.h"
#include "mathclass.h"
#include "tetrahedron.h"

Tetrahedron::Tetrahedron(void)
{
}

Tetrahedron::~Tetrahedron(void)
{
}

void Tetrahedron::calcWeight(const vectorn& p, vectorn& weight)
{
	// given a point, p
	// and corner points A, B, C, D
	// 
	// A*x+B*y+C*z+D*(1-x-y-z)=p
	// (A1-D1 B1-D1 C1-D1 ) (x)   P1-D1
	// (A2-D2 B2-D2 C2-D2 ) (y) = P2-D2
	// (A3-D3 B3-D3 C3-D3 ) (z)   P3-D3

	const vectorn& a=cornerPoint(0);
	const vectorn& b=cornerPoint(1);
	const vectorn& c=cornerPoint(2);
	const vectorn& d=cornerPoint(3);

	matrix4 mat;

	mat.setValue(a.x()-d.x(), b.x()-d.x(), c.x()-d.x(), 
				a.y()-d.y(), b.y()-d.y(), c.y()-d.y(), 
				a.z()-d.z(), b.z()-d.z(), c.z()-d.z());
	matrix4 invMat;
	invMat.inverse(mat);

	vector3 w;
	w.mult(invMat, vector3(p.x()-d.x(), p.y()-d.y(), p.z()-d.z()));
		
	weight.setValues(4, w.x, w.y, w.z, 1.0-(w.x+w.y+w.z));
}

m_real Tetrahedron::maxTdistance(const vectorn& weight)
{
	vectorn temp(weight);	// 0 to 1 if inside the tetrahedron
	temp*=2.0;
	temp-=1.0;	// -1 to 1 if inside the tetrahedron
	temp.each0(s1::abs); // 0 to 1 if inside the tetrahedron
    
	return temp.maximum();
}
