// Physics_LargeVector.h: interface for the Physics_LargeVector class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHYSICS_LARGEVECTOR_H__FDF65582_D36F_4061_A343_BDDA44E2DFDE__INCLUDED_)
#define AFX_PHYSICS_LARGEVECTOR_H__FDF65582_D36F_4061_A343_BDDA44E2DFDE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <assert.h>

class Physics_Matrix3x3;
class Physics_LargeVector  
{
public:
	int m_iElements;
	Physics_Vector3 *m_pData;

public:
	Physics_LargeVector( int iElements=1 );
	virtual ~Physics_LargeVector();

	inline int Size()	{ return m_iElements;}
	void Zero();
	void Resize( int iNewElements );
	bool Add( Physics_LargeVector &v, Physics_LargeVector &dst );
	bool Subtract( Physics_LargeVector &v, Physics_LargeVector &dst );
	Physics_t DotProduct( Physics_LargeVector &v );
	bool ElementMultiply( Physics_LargeVector &v, Physics_LargeVector &dst );
	bool ElementMultiply( Physics_Matrix3x3 S[], Physics_LargeVector &dst );
	bool Scale( Physics_t scale, Physics_LargeVector &dst );
	bool Invert( Physics_LargeVector &dst );
	Physics_LargeVector& operator=( Physics_LargeVector &copy );
	void Dump( char *szTitle = NULL );

	inline Physics_Vector3& operator[](int i) const	{ 
#ifdef _DEBUG
		assert(i>=0 && i<m_iElements); 
#endif
		return m_pData[i];}
	bool ElementPostMultiply( Physics_Matrix3x3 S[], Physics_LargeVector &dst );
};

#endif // !defined(AFX_PHYSICS_LARGEVECTOR_H__FDF65582_D36F_4061_A343_BDDA44E2DFDE__INCLUDED_)
