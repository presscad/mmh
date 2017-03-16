#pragma once


class vectorn;
class vectornView;
class smatrixn
{
public:
	smatrixn() {}
	~smatrixn()	{}

	int rows() const { return (_ij.size()==0)?0:_ij[0]-1;}
	int cols() const { return (_ij.size()==0)?0:_ij[0]-1;}

	// other should be a square matrix
	void assign(matrixn const& other);
	void operator=(matrixn const& other)	{ assign(other);}
	TString output();

	// unary operations
	void transpose(smatrixn const& other);

	// set matrix
	void setMatrix(matrixn& A);

	// binary operations
	void mult(smatrixn const& a, smatrixn const& b);
	void multABt(smatrixn const& a, smatrixn const& b);
	
	vectornView diagonal();
	
	// almost private:
	vectorn _s;
	intvectorn _ij;
	void _trim();

};

namespace sm
{
	// b=Ax
	void multmat(vectorn & b, smatrixn const& A, vectorn const& x);
	// b=A^T x
	void multmatAtx(vectorn & b, smatrixn const& A, vectorn const& x);

	// conjugate gradient solve
	void CGsolve(smatrixn const& A, vectorn const& b, vectorn& x);
	void CGsolve(matrixn const& A, vectorn const& b, vectorn & x);

	// test routines
	void testSparseMatrix();
};
