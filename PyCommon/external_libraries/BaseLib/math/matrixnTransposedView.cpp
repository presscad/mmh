#include "stdafx.h"
#include "matrixnTransposedView.h"

matrixnTransposedView::matrixnTransposedView(matrixn const& other)
{
	int on;
	other._getPrivate(buffer, stride, n, m, on);
}

void matrixnTransposedView::setDiagonal(m_real v)
{
	ASSERT(rows()==cols());
	for(int i=0; i<n ;i++)
		value(i,i)=v;
}

void matrixnTransposedView::setDiagonal(const vectorn& v)
{
	ASSERT(rows()==cols());
	ASSERT(rows()==v.size());
	for(int i=0; i<n ;i++)
		value(i,i)=v[i];
}

TString matrixnTransposedView::output(const char* formatString) const
{
	TString	id;
	
	TString temp;
	id+="[";
	
	for(int i=0; i<rows(); i++)
	{
		temp.format("%d ", i);
		temp+=row(i).output(formatString);
		id+=temp;
		id+="\n";
	}
	id+="]";
	return id;
}

matrixnView matrixnTransposedView::transpose() const
{
	return matrixnView(buffer, n,m,stride);
}

matrixnTransposedView matrixnTransposedView::range(int startRow, int endRow, int startColumn, int endColumn)
{
	if(endColumn>cols()) endColumn=cols();

	matrixnView trm=transpose();
	return matrixnTransposedView (trm.range(startColumn, endColumn, startRow, endRow));
}

inline const matrixnTransposedView matrixnTransposedView::range(int startRow, int endRow, int startColumn, int endColumn) const
{ 
	if(endColumn>cols()) endColumn=cols();
	matrixnView trm=transpose();
	return matrixnTransposedView (trm.range(startColumn, endColumn, startRow, endRow));
}
