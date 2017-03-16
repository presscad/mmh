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

#ifndef SMATRIX
#define SMATRIX

#include <VP/rmatrix3.h>
#include <vector>

template <class TYPE> class _smatrix;

template <class TYPE>
bool MinRes(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, int maxiter, TYPE rtol);

template <class TYPE>
class _smatrix
{
public:
	_smatrix()
	{
		row = col = 0;
	}

	~_smatrix()
	{
	}

	void clear(int r, int c)
	{
		row = r;
		col = c;
		index.resize(r);
		value.resize(r);
		diag_recp.resize(r);
		for ( int i = 0; i < r; i++ )
		{
			index[i].clear();
			value[i].clear();
			diag_recp[i] = (TYPE)0.0;
		}
	}

	// assume that both setValue(r, c1, v) and setValue(r, c2, v) are never called at the same stage, if c1 == c2.
	void setValue(int r, int c, const TYPE &val)
	{
		if ( r == c ) diag_recp[r] = (TYPE)1.0 / val;
		else
		{
			index[r].push_back(c);
			value[r].push_back(val);

			index[c].push_back(r);
			value[c].push_back(val);
		}
	}

	friend int SORSolveAxEqualB(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int maxiter, TYPE tol)
	{
		TYPE sum, one_w = (TYPE)1.0 - w;
		int cnt = 1;

		while ( maxiter-- )
		{
			for ( int i = 0; i < A.row; i++ )
			{
				sum = b[i];
				for ( unsigned int j = 0; j < A.index[i].size(); j++ ) sum -= A.value[i][j] * x[A.index[i][j]];
				x[i] = one_w * x[i] + w * A.diag_recp[i] * sum;

				if ( x[i] > (TYPE)50.0 ) x[i] = (TYPE)50.0;
				if ( x[i] < (TYPE)-50.0 ) x[i] = (TYPE)-50.0;
			}

			if ( SquareSum(A * x - b) < tol ) return cnt;
			cnt++;
		}
		return cnt;
	}

	friend _rmatrix<TYPE> convert(const _smatrix<TYPE> &A)
	{
		_rmatrix<TYPE> re = Zeros<TYPE>(A.row, A.col);
		for ( unsigned int i = 0; i < A.index.size(); i++ )
		{
			re(i,i) = (TYPE)1.0 / A.diag_recp[i];
			for ( unsigned int j = 0; j < A.index[i].size(); j++ )
				re(i, A.index[i][j]) = A.value[i][j];
		}
		return re;
	}

	friend _rmatrix<TYPE> operator * (const _smatrix<TYPE> &A, const _rmatrix<TYPE> &x)
	{
		TYPE sum;
		_rmatrix<TYPE> re(A.row, 1);
		for ( int i = 0; i < A.row; i++ )
		{
			sum = x[i] / A.diag_recp[i];
			for ( unsigned int j = 0; j < A.index[i].size(); j++ ) sum += A.value[i][j] * x[A.index[i][j]];
			re[i] = sum;
		}
		return re;
	}
		
	friend int CGSolveAxEqualB(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, int maxiter, TYPE tol)
	{
		TYPE r2, alpha, beta, dAd;
		_rmatrix<TYPE> r = b - A * x;
		_rmatrix<TYPE> d = r;
		_rmatrix<TYPE> Ad;
		int iter = 0;

		while ( maxiter-- )
		{
			r2 =  Inner(r, r);
			if ( r2 < tol ) return iter;
			Ad = A * d;
			dAd = Inner(d, Ad);

			if ( abs(Inner(d, Ad)) < tol ) return -1;

			alpha =  r2 / dAd;
			x += alpha * d;
			r -= alpha * Ad;
			beta = Inner(r, r) / r2;
			d = r + beta *d;

			iter++;
		}
		return iter;
	}

	friend int SORSolveLCP(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, const TYPE &w, int maxiter, TYPE tol)
	{
		TYPE sum, one_w = (TYPE)1.0 - w;
		int iter = 0;
		_rmatrix<TYPE> a;

		while ( maxiter-- )
		{
			for ( int i = 0; i < A.row; i++ )
			{
				sum = b[i];
				for ( unsigned int j = 0; j < A.index[i].size(); j++ ) sum += A.value[i][j] * x[A.index[i][j]];
				x[i] = one_w * x[i] - w * A.diag_recp[i] * sum;
				if ( x[i] < (TYPE)0.0 ) x[i] = (TYPE)0.0;
				if ( x[i] > (TYPE)50.0 ) x[i] = (TYPE)50.0;
			}
			a = A * x + b;
			if ( AbsInner(a, x) < tol ) return iter;
			
			iter++;
		}

		return iter;
	}

	friend bool MinRes<TYPE>(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, int maxiter, TYPE rtol);

protected:
	int row, col;
	std::vector<TYPE>			diag_recp;
	std::vector< std::vector<int> >		index;
	std::vector< std::vector<TYPE> >	value;
};

#include <VP/smatrixfriend.inl>

#endif
