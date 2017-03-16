#pragma once

class DelaunayTriangulation
{
public:
	
	DelaunayTriangulation();
	virtual ~DelaunayTriangulation(void);

	virtual int numSite()=0;
	virtual int dimension()=0;
	virtual const vectorn& site(int index)=0;

	void triangulate();
	//intvectorn simplex(int i)			{ return m_aSimplicials.row(i);}
	int numSimplex()					{ return m_aSimplicials.rows();}

private:	
	intmatrixn m_aSimplicials;
};
