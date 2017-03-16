
#include "stdafx.h"
#include "mathclass.h"
#include "float.h"
#include "filter.h"
#include "../utility/tfile.h"
#include "bspline.h"
#include "nr/nr.h"
#include "math_single_precision.h"


vectornSPView ::vectornSPView (float* ptrr, int size, int str)
:vectornSP(ptrr,size,str)
{
}

/////////////////////////////////////////////////////////////////////////////////


vectornSP::vectornSP( int n, float x)
:_tvectorn<float>()
{
	ASSERT(n==1);
	setSize(n);
	value(0)=x;
}

vectornSP::vectornSP( int n, float x, float y)
:_tvectorn<float>()
{
	ASSERT(n==2);
	setSize(n);
	value(0)=x;
	value(1)=y;
}
vectornSP::vectornSP( int n, float x, float y, float z)
:_tvectorn<float>()
{
	ASSERT(n==3);
	setSize(n);
	value(0)=x;
	value(1)=y;
	value(2)=z;
}


vectornSP::vectornSP( int n, float x, float y, float z, float w,...)	// n dimensional vector	(ex) : vectornSP(3, 1.0, 2.0, 3.0);
:_tvectorn<float>()
{
	va_list marker;
	va_start( marker, w);     /* Initialize variable arguments. */

	setSize(n);
	setValue(0, x);
	setValue(1, y);
	setValue(2, z);
	setValue(3, w);
	for(int i=4; i<n; i++)
	{
		setValue(i, va_arg( marker, float));
	}
	va_end( marker );              /* Reset variable arguments.      */	
}

vectornSPView vectornSP::range(int start, int end, int step)
{
	return _range<vectornSPView >(start,end,step);
}

const vectornSPView vectornSP::range(int start, int end, int step) const	{ return ((vectornSP*)this)->range(start, end, step);}




/////////////////////////////////////////////////////////////////////////////////
vectornSP::vectornSP()
:_tvectorn<float>()
{
}

matrixnSPView vectornSP::column() const
{
	return _column<matrixnSPView >();
}

matrixnSPView vectornSP::row() const		// return 1 by n matrix, which can be used as L-value (reference matrix)
{
	return _row<matrixnSPView >();
}

////////////////////////////////////////////////////////////////////////////////

matrixnSPView::matrixnSPView(float* ptr, int nrow, int ncol, int stride2)
:matrixnSP(ptr, nrow, ncol, stride2)
{
}

matrixnSPView::~matrixnSPView()
{
}
////////////////////////////////////////////////////////////////////////////////
matrixnSP ::matrixnSP ( int x, int y)
:_tmat<float>()
{
	setSize(x,y);
}

matrixnSP ::~matrixnSP ()
{
}

