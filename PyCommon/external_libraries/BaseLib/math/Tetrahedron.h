#pragma once

class Tetrahedron
{
public:
	Tetrahedron(void);
	~Tetrahedron(void);

	virtual const vectorn& cornerPoint(int ivertex)=0;	//!< 0<=ivertex<3


	void calcWeight(const vectorn& p, vectorn& weight);
	m_real maxTdistance(const vectorn& weight);
};
