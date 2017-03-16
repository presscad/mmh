#include "stdafx.h"
#include "mathclass.h"
#include "conversion.h"

vectornView vecView(matrixn const& a)
{
	int stride, n2, m2, on2;
	m_real* ptr;
	a._getPrivate(ptr, stride, n2, m2, on2);
	Msg::verify(a.cols()==stride, "vecView error! (a.cols()!=stride)");
	return vectornView(ptr, n2*m2, 1);
}

matrixnView matView(vectorn const& a, int nColumn)
{
	Msg::verify(a.size()%nColumn==0, "vectorn A cannot be viewed as matrixn");
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	return matrixnView(ptr, a.size()/nColumn, nColumn, nColumn);
}

matrixnView matView(matrixn const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	return a.range(start, end);
}

matrixnView matViewCol(matrixn const& a, int start, int end)
{
	if(end>a.cols()) end=a.cols();
	return a.range(0, a.rows(), start, end);
}

intmatrixnView intmatView(intmatrixn const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	return a.range(start, end);
}

intmatrixnView intmatViewCol(intmatrixn const& a, int start, int end)
{
	if(end>a.cols()) end=a.cols();
	return a.range(0, a.rows(), start, end);
}


matrixnView matView(quaterN const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	return matrixnView((m_real*)&a.row(0), end-start, 4, stride);
}

matrixnView matView(vector3N const& a, int start, int end)
{
	if(end>a.rows()) end=a.rows();
	m_real* ptr;
	int stride, n2, on2;
	a._getPrivate(ptr, stride, n2, on2);
	return matrixnView((m_real*)&a.row(0), end-start, 3, stride);
}

quaterNView quatViewCol(matrixn const& a, int start)
{
	return a.range(0, a.rows(), start, start+4).toQuaterN();
}

vector3NView vec3ViewCol(matrixn const& a, int start)
{
	return a.range(0, a.rows(), start, start+3).toVector3N();
}


vectornView vecViewOffset(vectorn const& a, int start)
{
	m_real *ptr;
	int stride, n, on;
	a._getPrivate(ptr, stride, n, on);

	ptr=((m_real*)(ptr-start*stride));
	return vectornView(ptr, n+start, stride);
}