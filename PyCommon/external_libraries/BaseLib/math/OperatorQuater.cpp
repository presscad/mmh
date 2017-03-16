#include "stdafx.h"
#include "mathclass.h"
#include "operator.h"
#include "nr/nr.h"
#include "operatorQuater.h"


void v2::interpolateQuater::calc(vectorn& c, const vectorn& a, const vectorn& b) const
{
	ASSERT(a.size()==b.size());
	ASSERT(a.size()%4==0);
	c.setSize(a.size());

	int nq=a.size()/4;

	for(int i=0; i<nq; i++)
	{
		int start=i*4;
		int end=(i+1)*4;
		quater q;
		q.interpolate(m_fT, a.toQuater(start), b.toQuater(start));
		c.range(start, end)=q;
	}
}

