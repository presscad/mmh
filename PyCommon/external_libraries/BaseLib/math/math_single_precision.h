#pragma once
#include "template_math.h"

class vectornSPView;
class matrixnSPView ;

class vectornSP : public _tvectorn<float>
{	
protected:
	vectornSP(float* ptrr, int size, int stride):_tvectorn<float>(ptrr,size,stride){}
public:
	vectornSP();		

	// 값을 카피해서 받아온다.	
	template <class VecType>
	vectornSP(const VecType& other)	{ assign(other);}

	explicit vectornSP( int x):_tvectorn<float>() { setSize(x);}

	// n dimensional vector	(ex) : vectornSP(3, 1.0, 2.0, 3.0);
	explicit vectornSP( int n, float x);		
	explicit vectornSP( int n, float x, float y);		
	explicit vectornSP( int n, float x, float y, float z);		
	explicit vectornSP( int n, float x, float y, float z, float w, ...);	// n dimensional vector	(ex) : vectornSP(3, 1.0, 2.0, 3.0);

	~vectornSP(){}

	matrixnSPView column() const;	// return n by 1 matrix, which can be used as L-value (reference matrix)
	matrixnSPView row() const;	// return 1 by n matrix, which can be used as L-value (reference matrix)
	
	// L-value로 사용될수 있는, reference array를 만들어 return 한다. 
	// ex) v.range(0,2).setValues(2, 1.0, 2.0);
	vectornSPView range(int start, int end, int step=1);
	const vectornSPView range(int start, int end, int step=1) const	;

	template <class VecType>
	vectornSP& operator=(const VecType& other)	{ _tvectorn<float>::assign(other);return *this;}
};



class vectornSPView :public vectornSP
{
public:
	// L-value로 사용될수 있는, reference array로 만든다. 
	vectornSPView (float* ptrr, int size, int stride);	
	// 값을 reference로 받아온다.
	template <class VecType>
	vectornSPView(const VecType& other)	{ assignRef(other);}

	~vectornSPView (){}

	// L-value로 사용되는 경우, 값을 copy한다.
	template <class VecType>
	vectornSPView& operator=(const VecType& other)	{ _tvectorn<float>::assign(other);return *this;}

};


class matrixnSP : public _tmat<float>
{
protected:
	matrixnSP(float* _ptr, int _n, int _m, int _stride) :_tmat<float>(_ptr, _n, _m, _stride){}
public:
	matrixnSP():_tmat<float>(){}
	matrixnSP ( int x, int y);

	template <class MatType>
	matrixnSP (const MatType& other):_tmat<float>()	{ _tmat<float>::assign(other);	}

	virtual ~matrixnSP();

	// reference가 value로 카피된다.
	template <class MatType>
	matrixnSP& operator=(const MatType& other)	{ __super::assign(other); return *this;}

	// L-value로 사용될수 있는 reference vector 를 return한다.
	vectornSPView		row(int i)const				{ return _row<vectornSPView>(i);}
	vectornSPView		column(int i)const			{ return _column<vectornSPView>(i);}

	matrixnSPView range(int startRow, int endRow, int startColumn=0, int endColumn=INT_MAX);
	inline const matrixnSPView range(int startRow, int endRow, int startColumn=0, int endColumn=INT_MAX) const;	

};

class matrixnSPView : public matrixnSP
{
public:
	// copy constructors : get reference
	template <class MatType>
	matrixnSPView(const MatType& other)		{ _assignRef(other);	}

	// L-value로 사용할때는 copy
	template <class MatType>
	matrixnSPView& operator=(const MatType& other)	{ _tmat<float>::assign(other); return *this;}

	// L-value로 사용될수 있는, reference matrix로 만든다. 
	matrixnSPView (float* ptr, int nrow, int ncol, int stride);
	virtual ~matrixnSPView ();
};
