// TMatrix.h: interface for the TMatrix class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_TMATRIX_H__A5015D75_8F19_4952_A56C_89A22492202E__INCLUDED_)
#define AFX_TMATRIX_H__A5015D75_8F19_4952_A56C_89A22492202E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

template <class T>
class TMatrix  
{
public:
	TMatrix();
	TMatrix(int nrow, int ncolumn);
	virtual ~TMatrix();

	void setSize(int nrow, int ncolumn);
	void release();

	int rows() const;
	int cols() const;
	
	template <class T> class RangeCheck
	{
		T** mPtr;
		int mCol;
	public:
		RangeCheck(T** p, int col):mPtr(p), mCol(col){}
		T& operator[](int i) const	{ ASSERT(i>=0 && i<mCol); return *mPtr[i];}
	};

	RangeCheck<T> operator[](int i) const		{ ASSERT(i<rows() ); return RangeCheck<T>(m_aapElement[i], cols());}
	T& data(int nrow, int ncolumn);
private:
	T*** m_aapElement;
	int m_nRow;
	int m_nColumn;
};

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#include "util.h"

template <class T>
TMatrix<T>::TMatrix(int nrow, int ncolumn)
{
	m_nRow=0;
	m_nColumn=0;
	setSize(nrow,ncolumn);
}

template <class T>
TMatrix<T>::TMatrix()
{
	m_nRow=0;
	m_nColumn=0;
}

template <class T>
TMatrix<T>::~TMatrix()
{
	release();
}

template <class T>
int TMatrix<T>::rows() const
{
	return m_nRow;
}

template <class T>
int TMatrix<T>::cols() const
{
	return m_nColumn;
}

template <class T>
void TMatrix<T>::setSize(int nrow, int ncolumn)
{
	release();

	m_nRow=nrow;
	m_nColumn=ncolumn;
	m_aapElement=AllocMatrix<T*>(nrow, ncolumn);
	
	for(int i=0; i<nrow; i++)
	{
		for(int j=0; j<ncolumn; j++)
			m_aapElement[i][j]=new T();
	}	
}

template <class T>
void TMatrix<T>::release()
{
	if(m_nRow==0 && m_nColumn==0) return;

	for(int i=0; i<m_nRow; i++)
	{
		for(int j=0; j<m_nColumn; j++)
			delete m_aapElement[i][j];
	}

	FreeMatrix<T*>(m_nRow, m_aapElement);

	m_nRow=0;
	m_nColumn=0;
}

template <class T>
T& TMatrix<T>::data(int nrow, int ncolumn)
{
	return *m_aapElement[nrow][ncolumn];
}

#endif // !defined(AFX_TMATRIX_H__A5015D75_8F19_4952_A56C_89A22492202E__INCLUDED_)
