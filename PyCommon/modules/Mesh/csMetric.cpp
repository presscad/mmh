#pragma once

#include "stdafx.h"

#include "../../common_sources/bputil.h"
#include "cBaseLibUtil.h"
#include "csMetric.h"

BOOST_PYTHON_MODULE(csMetric)
{
	def("calcPointCloudMetric", calcPointCloudMetric);
}


// pointsA[i] = outTransT * pointsB[i]
m_real calcPointCloudMetric( const bp::list& pointsA, const bp::list& pointsB, object& outTransT )
{
	int n = len(pointsA);
	vectorn a(n*3), b(n*3);
	for(int i=0; i<n ; ++i)
	{
		a.setVec3(i*3, PYSEQ_2_VECTOR3(pointsA[i]));
		b.setVec3(i*3, PYSEQ_2_VECTOR3(pointsB[i]));
	}	

	PointCloudMetric metric;
	m_real distance = metric.CalcDistance(a, b);

	PYT_FROM_MATRIX4(outTransT, metric.m_transfB);
	
	return distance;
}