/*
VirtualPhysics v0.9

VirtualPhysics was developed as part of the project entitled "Development of 
Real-time Physics Simulation Engine for e-Entertainments" which was financially
supported by the grant from the strategic technology development program
(Project No. 2008-F-033-02) of both the MKE(Ministry of Knowledge Economy) and
MCST(Ministry of Culture, Sports and Tourism) of Korea.

Copyright (c) 2008-2010, Jinwook Kim, Korea Institute of Science and Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.

   3. Only research partners of the project "Development of Real-time Physics 
      Simulation Engine for e-Entertainments" can modify this list of 
	  conditions and the following disclaimer with the consent of the author, 
	  where the research partners refer to all principal investigators 
	  involved in the project. 

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	rmatrix3.h
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2003.9.2
//
//		Note		:
//
//////////////////////////////////////////////////////////////////////////////////

#ifndef _RMatrix3_
#define _RMatrix3_

#include <cmath>
#include <iostream>
#include <iomanip>
#include <assert.h>
#include <float.h>
#include <cstdlib>
#include "LieGroup.h"

#define DGEFA_EPS	1.0E-6
#define SVD_EPS		1.0E-6

using namespace std;

template <class TYPE> class _rmatrix;

/*!
	\class _rmatrix
	\brief General dense matrix and linear algebraic calculation
	
	_rmatrix is a template class to manipulate general dense matrices and calculate linear algebraic equations.
	EISPACK is used for funciton to get eigenvalue and LINPACK is used for solving linear equations.
*/

// start of declaration of friend function
template <class TYPE>
TYPE drand(TYPE range);

template <class TYPE>
TYPE drand(TYPE _min, TYPE _max);

template <class TYPE>
ostream &operator << (ostream &os, const _rmatrix<TYPE> &m);

template <class TYPE>
TYPE FNorm(const _rmatrix<TYPE> &A);

template <class TYPE>
TYPE SquareSum(const _rmatrix<TYPE> &A);

template <class TYPE>
TYPE AbsSum(const _rmatrix<TYPE> &A);

template <class TYPE>
TYPE MaxVec(const _rmatrix<TYPE> &x, int *idx);

template <class TYPE>
TYPE MinVec(const _rmatrix<TYPE> &x, int *idx);

template <class TYPE>
int t_imax(int n, TYPE *dx);

template <class TYPE>
void t_dgefa(TYPE *x, int lda, int n, int *jpvt, int &info);

template <class TYPE>
TYPE Det(_rmatrix<TYPE> A);

template <class TYPE>
_rmatrix<TYPE> Eye(int r, int c);

template <class TYPE>
_rmatrix<TYPE> Eye(int r);

template <class TYPE>
_rmatrix<TYPE> Zeros(int r, int c);

template <class TYPE>
_rmatrix<TYPE> Rand(int r, int c);

template <class TYPE>
TYPE Inner(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &y);

template <class TYPE>
TYPE AbsInner(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &y);

template <class TYPE>
TYPE Quadratic(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &y);

template <class TYPE>
TYPE Trace(const _rmatrix<TYPE> &A);

template <class TYPE>
_rmatrix<TYPE> Diag(_rmatrix<TYPE> &m);

template <class TYPE>
void AMultB(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

template <class TYPE>
void AMultBt(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

template <class TYPE>
void AtMultB(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

template <class TYPE>
void t_dgesl(TYPE *x, int lda, int n, int *jpvt, TYPE *b, int job);

template <class TYPE>
bool SolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B);

template <class TYPE>
bool SolveAxEqualB_(_rmatrix<TYPE> &A, _rmatrix<TYPE> &x);

template <class TYPE>
bool SolveAtxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B);

template <class TYPE>
int t_dpofa(TYPE *a, int n);

template <class TYPE>
void t_dposl(TYPE *a, int n, TYPE *b);

template <class TYPE>
bool SolvePosDefAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B);

template <class TYPE>
void t_fullpivoting(TYPE *x, int r, int c, int k, int *ipvt, int *jpvt);

template <class TYPE>
int t_gauss_elimination(TYPE *x, int r, int c, int *ipvt, int *jpvt, TYPE zero_tolerance);

template <class TYPE>
int GaussElimination(_rmatrix<TYPE> &A, _rmatrix<int> &row_pivot, _rmatrix<int> &column_pivot, TYPE eps);

template <class TYPE>
int GaussElimination(_rmatrix<TYPE> &A, _rmatrix<int> &row_pivot, _rmatrix<int> &column_pivot);

template <class TYPE>
bool t_drive_to_zero(int d, const _rmatrix<TYPE> &A, _rmatrix<TYPE> &a, _rmatrix<TYPE> &f, _rmatrix<int> &C, _rmatrix<int> &NC);

template <class TYPE>
bool t_check_lcp(const _rmatrix<TYPE> &a, const _rmatrix<TYPE> &f);

template <class TYPE>
bool DantzigSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);

template <class TYPE>
void lcp_lexicolemke(int nn, const TYPE *vec, const TYPE *q, TYPE *zlem, TYPE *wlem, int &info, int itermax, int &iter_count);

template <class TYPE>
bool LemkeSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);

template <class TYPE>
TYPE t_dsign(TYPE a, TYPE b);

template <class TYPE>
TYPE t_pythag(TYPE a, TYPE b);

template <class TYPE>
bool t_svdcmp(int m, int n, TYPE *a, TYPE *w, TYPE *v, bool matu, bool matv, TYPE *rv1);

template <class TYPE>
_rmatrix<TYPE> SVD(const _rmatrix<TYPE> &m);

template <class TYPE>
void SVD(const _rmatrix<TYPE> &M, _rmatrix<TYPE> &U, _rmatrix<TYPE> &S, _rmatrix<TYPE> &V);

template <class TYPE>
int Rank(const _rmatrix<TYPE> &m, TYPE eps);

template <class TYPE>
int Rank(const _rmatrix<TYPE> &m);

template <class TYPE>
bool SVDSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B, TYPE tol);

template <class TYPE>
_rmatrix<TYPE> Inv(const _rmatrix<TYPE> &A);

template <class TYPE>
bool FixedPointSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);

template <class TYPE>
_rmatrix<TYPE> Abs(const _rmatrix<TYPE> &M);

template <class TYPE>
void t_dqrdc(TYPE *x, int ldx, int n, int p, TYPE *qraux, int *jpvt, TYPE *work, int job);

template <class TYPE>
void t_dqrsl(TYPE *x, int ldx, int n, int k, TYPE *qraux, const TYPE *y, TYPE *qy, TYPE *qty, TYPE *b, TYPE *rsd, TYPE *xb, int job, int &info);

template <class TYPE>
bool QRSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &B);

template <class TYPE>
bool FixedPointSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

template <class TYPE>
bool SORSolveAxEqualB(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter);

template <class TYPE>
bool SORSolveLCP(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter);

//end of declaration of friend function

template <class TYPE>
class _rmatrix
{
public:
	_rmatrix();
	
	/** @name Construction and manipulation
	*/
	//@{
	/*!
		constructor
		make a r-by-c matrix.
	*/
	_rmatrix(int r, int c);
	
	/*!
		copy constructor
	*/
	_rmatrix(const _rmatrix<TYPE> &m);
	
	/*!
		constructor
		make a r-by-c matrix and its elements will be copied from an array d.
	*/
	_rmatrix(int r, int c, const TYPE d[]);

	/*!
		resize itself to be a r-by-c matrix. Elements of the matrix will not be initialized.
	*/
	void ReNew(int r, int c);
	void ReNew(int r);

	/*!
		set all the elements to be zero.
	*/
	void SetZero(void);

	/*!
		resize itself to be a r-by-c zero matrix.
	*/
	void SetZero(int r, int c);

	/*!
		make itself to be absolute.
	*/
	void SetAbs(void);

	/*!
		resize itself to be the identity matrix.
		Resized matrix is a r-by-c matrix with 1's on the diagonal and zeros elsewhere.
	*/
	void SetEye(int r, int c);
	
	/*!
		invert itself.
		\return true if the inversion exists.
	*/
	bool SetInv(void)
	{
		_rmatrix<TYPE> _copy = *this;
		bool re = SolveAxEqualB(_copy, *this, Eye<TYPE>(row, row));
		return re;
	}

	/*!
		normalize itself, where the matrix is assumed to be a vector.
		\return a length of the vector
	*/
	TYPE Normalize(void)
	{
		TYPE norm = FNorm(*this), inorm = SCALAR_1 / norm;
		*this *= inorm;
		return norm;
	}
	//@}
	
	~_rmatrix();

	/** @name Operators
	*/
	//@{
	/*!
		access the i th element, where the matrix is assumed to be a column order vector.
	*/
	TYPE &operator [] (int i);

	const TYPE &operator [] (int i) const;

	/*!
		access the i th row and the j th column element.
	*/
	TYPE &operator () (int i, int j );

	const TYPE &operator () (int i, int j ) const;

	/*!
		unary plus operator
	*/
	const _rmatrix<TYPE> &operator + (void) const;
	
	/*!
		unary minus operator
	*/
	_rmatrix<TYPE> operator - (void) const;
	
	/*!
		transpose operator
	*/
	_rmatrix<TYPE> operator ~ (void) const;
	
	/*!
		substitution operator
	*/
	const _rmatrix<TYPE> &operator = (const _rmatrix<TYPE> &m);

	/*!
		+= operator
	*/
	const _rmatrix<TYPE> &operator += (const _rmatrix<TYPE> &m);
	
	/*!
		-= operator
	*/
	const _rmatrix<TYPE> &operator -= (const _rmatrix<TYPE> &m);
	
	/*!
	&nbsp;*= operator with scalar
	*/
	const _rmatrix<TYPE> &operator *= (TYPE c);

	/*!
		/= operator with scalar
	*/
	const _rmatrix<TYPE> &operator /= (TYPE c);

	/*!
		addition operator
	*/
	_rmatrix<TYPE> operator + (const _rmatrix<TYPE> &m) const;

	/*!
		subtraction operator
	*/
	_rmatrix<TYPE> operator - (const _rmatrix<TYPE> &m) const;

	/*!
		matrix multiplication operator
	*/
	_rmatrix<TYPE> operator * (const _rmatrix<TYPE> &m) const;

	/*!
		matrix multiplication operator
		A | B = A * ~B
	*/
	_rmatrix<TYPE> operator | (const _rmatrix<TYPE> &m) const;

	/*!
		matrix multiplication operator
		A ^ B = ~A * B
	*/
	_rmatrix<TYPE> operator ^ (const _rmatrix<TYPE> &m) const;

	/*!
		scalar multiplication operator
	*/
	_rmatrix<TYPE> operator * (TYPE c) const;

	/*!
		scalar division operator
	*/
	_rmatrix<TYPE> operator / (TYPE c) const;

	/*!
		matrix inversion operator
		A \% B = Inv(A) * B
	*/
	_rmatrix<TYPE> operator % (const _rmatrix<TYPE> &m) const;

	/*!
		matrix inversion operator
		A & B = Inv(~A) * B
	*/
	_rmatrix<TYPE> operator & (const _rmatrix<TYPE> &m) const;

	/*!
		scalar multiplication operator
	*/
	friend _rmatrix<TYPE> operator * (TYPE c, _rmatrix<TYPE> m)
	{		
		int n = m.row * m.col;
		TYPE *_m = m.element;
		while ( n-- ) *(_m++) *= c;
		return m;
	}

	/*!
		standard output operator
	*/
	friend ostream &operator <<<TYPE>(ostream &os, const _rmatrix<TYPE> &m);

	//@}

	/** @name Attributes
	*/
	//@{
 		/*!
		get a number of rows.
	*/
	int RowSize(void) const;
	
	/*!
		get a number of columns.
	*/
	int ColSize(void) const;
	
	//@}
	
	TYPE GetSparsity(void) const;

	/*!
		get the maximum element in x.
		\param idx if not NULL, an index of the maximum element
	*/
	friend TYPE MaxVec<TYPE>(const _rmatrix<TYPE> &x, int *idx);

	/*!
		get the minimum element in x.
		\param idx if not NULL, an index of the minimum element
	*/
	friend TYPE MinVec<TYPE>(const _rmatrix<TYPE> &x, int *idx);
	
	/*!
		get a r-by-c zero matrix.
	*/
	friend _rmatrix<TYPE> Zeros<TYPE>(int r, int c);

	/*!
		get a r-by-c random matrix. call srand() to set a seed for random-number generation.
	*/
	friend _rmatrix<TYPE> Rand<TYPE>(int r, int c);

	/*!
		get the r-by-c identity matrix.
	*/
	friend _rmatrix<TYPE> Eye<TYPE>(int r, int c);

	/*!
		get the n-by-n identity matrix.
	*/
	friend _rmatrix<TYPE> Eye<TYPE>(int n);
	
	/*!
		get a squared sum of A.
	*/
	friend TYPE SquareSum<TYPE>(const _rmatrix<TYPE> &A);

	/*!
		get an absolute sum of A.
	*/
	friend TYPE AbsSum<TYPE>(const _rmatrix<TYPE> &A);

	/*!
		get an inner product of x and y, where x and y are assumed to be vectors.
	*/
	friend TYPE Inner<TYPE>(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &y);

	/*!
		get an inner product of abs(x) and abs(y), where x and y are assumed to be vectors.
	*/
	friend TYPE AbsInner<TYPE>(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &y);

	/*!
		get a quadratic product.
		Quadratic(x,A,y) = \f$\sum x_i A_{ij} y_j\f$.
		x and y will be assumed as vectors.
	*/
	friend TYPE Quadratic<TYPE>(const _rmatrix<TYPE> &x, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &y);

	/*!
		get the Frobenius norm of A.
		FNorm(A) = \f$\sqrt{\sum A_{ij}^2}\f$.
	*/
	
	friend TYPE FNorm<TYPE>(const _rmatrix<TYPE> &A);
	
	/*!
		get a determinant of \a m.
	*/
	friend TYPE Det<TYPE>(_rmatrix<TYPE> A);

	/*!
		get the sum of the diagonal elements of A.
	*/
	friend TYPE Trace<TYPE>(const _rmatrix<TYPE> &A);
	
	/*!
		diagonal matrix and diagonals of a matrix
		If m is a n-vector, Diag(m) is a sqaure matrix with the elements of m on the diagonal.
		If m is a square matrix, Diag(m) is the main diagonal of \a m.
	*/
	friend _rmatrix<TYPE> Diag<TYPE>(_rmatrix<TYPE> &m);
	
	/*!
		the inverse of the square matrix A
	*/
	friend _rmatrix<TYPE> Inv<TYPE>(const _rmatrix<TYPE> &A);
	
	/*!
		matrix multiplication
		C = A * B
	*/
	friend void AMultB<TYPE>(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

	/*!
		matrix multiplication
		C = A * ~B
	*/
	friend void AMultBt<TYPE>(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

	/*!
		matrix multiplication
		C = ~A * B
	*/
	friend void AtMultB<TYPE>(_rmatrix<TYPE> &C, const _rmatrix<TYPE> &A, const _rmatrix<TYPE> &B);

	/*!
		get abs(m)
	*/
	friend _rmatrix<TYPE> Abs<TYPE>(const _rmatrix<TYPE> &M);

	/** @name Solving linear equations
	*/
	//@{ 	
	/*!
		solve linear equation, A x = b.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool SolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);
	friend bool SolveAxEqualB_<TYPE>(_rmatrix<TYPE> &A, _rmatrix<TYPE> &B);

	/*!
		solve linear equation, ~A x = b.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool SolveAtxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b, where \a A should be positive definite.
		\param[out] x the solution
		\return true if the solution exists.
		\note elements of A change after return.
	*/
	friend bool SolvePosDefAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b using singular value decomposition.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool SVDSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, TYPE tol);

	/*!
		solve linear equation, A x = b using QR decomposition.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool QRSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);

	/*!
		solve linear equation, A x = b using fixed point problem.
		\param[out] x the solution
		\return true if the solution exists.
	*/
	friend bool FixedPointSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b);
	//@}

	friend int GaussElimination<TYPE>(_rmatrix<TYPE> &, _rmatrix<int> &, _rmatrix<int> &);
	friend int GaussElimination<TYPE>(_rmatrix<TYPE> &, _rmatrix<int> &, _rmatrix<int> &, TYPE);

	/** @name Singualr value decomposition
	*/
	//@{
 	/*!
		get singular values of A.
	*/
	friend _rmatrix<TYPE> SVD<TYPE>(const _rmatrix<TYPE> &A);

	/*!
		singular value decomposition
		A = U * Diag(S) * ~V
	*/
	friend void SVD<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &U, _rmatrix<TYPE> &S, _rmatrix<TYPE> &V);

	friend int Rank<TYPE>(const _rmatrix<TYPE> &, TYPE);
	
	/*!
		get an number of linearly independent rows or columns of A.
	*/
	friend int Rank<TYPE>(const _rmatrix<TYPE> &A);
	//@}

	/*!
		solve linear complementarity problem using Lemke's method
		\note Find \f$f\f$ such that \f$0 < f \perp Af + b > 0 \f$.
	*/
	friend bool LemkeSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);
	
	/*!
		solve linear complementarity problem using Dantzig's method
		\note Find \f$f\f$ such that \f$0 < f \perp Af + b > 0 \f$.
	*/
	friend bool DantzigSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);
	
	/*!
		solve linear complementarity problem using the fixed point problem
		\note Find \f$f\f$ such that \f$0 < f \perp Af + b > 0 \f$.
	*/
	friend bool FixedPointSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &f, const _rmatrix<TYPE> &b);

	friend bool SORSolveLCP<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter);

	/*!
		solve Ax=b using Successive over-relaxation method.
		\note w = SOR factor, iter = number of iteration
	*/
	friend bool SORSolveAxEqualB<TYPE>(const _rmatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int iter);

private:
	int		 row;
	int		 col;
	int		 mem_size;
	TYPE	*element;
};

#include <VP/rmatrix3friend.inl>
#include <VP/rmatrix3.inl>

#endif
