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
//		title		:	rmatrix3.inl
//						
//		version		:	v2.895
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2003.9.2
//
//		Note		:
//
//////////////////////////////////////////////////////////////////////////////////

template <class TYPE>
_rmatrix<TYPE>::_rmatrix(void) : row(0), col(0), mem_size(0), element(NULL)
{
}

template <class TYPE>
_rmatrix<TYPE>::_rmatrix(int r, int c) : row(r), col(c), mem_size(r*c), element(new TYPE[mem_size])
{
}

template <class TYPE>
_rmatrix<TYPE>::_rmatrix(const _rmatrix &m) : row(m.row), col(m.col), mem_size(row*col), element(new TYPE[mem_size])
{
	TYPE *_t = element, *_m = m.element;
	int n = mem_size;
	while ( n-- ) *(_t++) = *(_m++);
}

template <class TYPE>
_rmatrix<TYPE>::_rmatrix(int r, int c, const TYPE d[]) : row(r), col(c), mem_size(row*col), element(new TYPE[mem_size])
{
	TYPE *_t = element;
	int n = mem_size;
	while ( n-- ) *(_t++) = *(d++);
}

template <class TYPE>
void _rmatrix<TYPE>::ReNew(int r, int c)
{
	assert(r >= 0 && c >= 0 && "_rmatrix::ReNew(int, int) -> index exceeds matrix dimensions");
	
	int n = r * c;
	if ( n > mem_size )
	{
		delete [] element;
		element = new TYPE [mem_size = n << 1];
	}
	row = r;
	col = c;
}

template <class TYPE>
void _rmatrix<TYPE>::ReNew(int r)
{
	assert(r >= 0 && "_rmatrix::ReNew(int) -> index exceeds matrix dimensions");
	
	int n = r;
	if ( n > mem_size )
	{
		delete [] element;
		element = new TYPE [mem_size = n << 1];
	}
	row = r;
	col = 1;
}

template <class TYPE>
void _rmatrix<TYPE>::SetZero(void)
{
	int n = row * col;
	TYPE *_t = element;
	while ( n-- ) *(_t++) = (TYPE)0.0;
}

template <class TYPE>
void _rmatrix<TYPE>::SetZero(int r, int c)
{
	assert(r >= 0 && c >= 0 && "_rmatrix::SetZero(int, int) -> index exceeds matrix dimensions");

	int n = r * c;
	if ( n > mem_size )
	{
		delete [] element;
		element = new TYPE [mem_size = n << 1];
	}
	row = r;
	col = c;
	TYPE *_t = element;
	while ( n-- ) *(_t++) = (TYPE)0.0;
}

template <class TYPE>
void _rmatrix<TYPE>::SetEye(int r, int c)
{
	assert(r >= 0 && c >= 0 && "_rmatrix::ReNew(int, int) -> index exceeds matrix dimensions");
	
	int n = r * c;
	if ( n > mem_size )
	{
		delete [] element;
		element = new TYPE [mem_size = n << 1];
	}
	row = r;
	col = c;
	
	TYPE *_r = element;
	while ( n-- ) *(_r++) = (TYPE)0.0;
	
	if ( c > r ) c = r;
	r++;

	_r = element;
	while ( c-- )
	{
		*_r = (TYPE)1.0;
		_r += r;
	}
}

template <class TYPE>
void _rmatrix<TYPE>::SetAbs(void)
{
	TYPE *_m = element;
	int n = row * col;
	while ( n-- ) *_m = abs(*_m++);
}

template <class TYPE>
_rmatrix<TYPE>::~_rmatrix()
{
	delete [] element;
}

template <class TYPE>
TYPE &_rmatrix<TYPE>::operator [] (int i)
{
	assert(i >= 0 && i < row*col && "_rmatrix::operator[int] -> index over range");
	return element[i];
}

template <class TYPE>
const TYPE &_rmatrix<TYPE>::operator [] (int i) const
{
	assert(i >= 0 && i < row*col && "_rmatrix::operator[int] -> index over range");
	return element[i];
}

template <class TYPE>
TYPE &_rmatrix<TYPE>::operator () (int i, int j ) 
{
	assert(i >= 0 && i < row && j >= 0 && j < col && "_rmatrix::operator(int, int) -> index over range");
	return element[i+j*row];
}

template <class TYPE>
const TYPE &_rmatrix<TYPE>::operator () (int i, int j ) const
{
	assert(i >= 0 && i < row && j >= 0 && j < col && "_rmatrix::operator(int, int) -> index over range");
	return element[i+j*row];
}

template <class TYPE>
const _rmatrix<TYPE> &_rmatrix<TYPE>::operator + (void) const
{
	return *this;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator - (void) const
{
	_rmatrix<TYPE> re(row, col);
	TYPE *_m = re.element, *_t = element;
	int n = row * col;
	while ( n-- ) *(_m++) = -*(_t++);
	return re; 
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator ~ (void) const 
{ 
	_rmatrix<TYPE> re(col, row);
	int i = 0, r = row, c;
	TYPE *_m = re.element, *_mt;
	while ( r-- )
	{
		_mt = element + (i++);
		c = col;
		while ( c-- )
		{
			*(_m++) = *_mt;
			_mt += row;
		}
	}
	return re;
}

template <class TYPE>
const _rmatrix<TYPE> &_rmatrix<TYPE>::operator = (const _rmatrix &m)
{
	int n = m.row * m.col;
	if ( mem_size < n ) 
	{
		delete [] element;
		element = new TYPE [(mem_size = n << 1)];
	}
	row = m.row;	col = m.col;
	TYPE *_t = element, *_m = m.element;
	while ( n-- ) *(_t++) = *(_m++);		
	return *this;
}

template <class TYPE>
const _rmatrix<TYPE> &_rmatrix<TYPE>::operator += (const _rmatrix &m)
{
	assert(row == m.row && col == m.col && "_rmatrix::operator += (const _rmatrix &) -> size is not compatible");

	int n = row * col;
	TYPE *_t = element, *_m = m.element;
	while ( n-- ) *(_t++) += *(_m++);
	return *this;
}

template <class TYPE>
const _rmatrix<TYPE> &_rmatrix<TYPE>::operator -= (const _rmatrix &m)
{
	assert(row == m.row && col == m.col && "_rmatrix::operator -= (const _rmatrix &) -> size is not compatible");

	int n = row * col;
	TYPE *_t = element, *_m = m.element;
	while ( n-- ) *(_t++) -= *(_m++);
	return *this;
}

template <class TYPE>
const _rmatrix<TYPE> &_rmatrix<TYPE>::operator *= (TYPE c)
{
	int n = row * col;
	TYPE *_t = element;
	while ( n-- ) *(_t++) *= c;		
	return *this;
}

template <class TYPE>
const _rmatrix<TYPE> &_rmatrix<TYPE>::operator /= (TYPE c)
{
	int n = row * col;
	TYPE *_t = element, ci = (TYPE)1.0 / c;
	while ( n-- ) *(_t++) *= ci;		
	return *this;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator + (const _rmatrix &m) const
{
	assert(row == m.row && col == m.col && "_rmatrix::operator + (const _rmatrix &) -> size is not compatible");

	_rmatrix<TYPE> re(row, col);
	int n = row * col;
	TYPE *_t = element, *_r = re.element, *_m = m.element;
	while ( n-- ) *(_r++) = *(_t++) + *(_m++);
	return re;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator - (const _rmatrix &m) const
{
	assert(row == m.row && col == m.col && "_rmatrix::operator - (const _rmatrix &) -> size is not compatible");

	_rmatrix<TYPE> re(row, col);
	int n = row * col;
	TYPE *_t = element, *_r = re.element, *_m = m.element;
	while ( n-- ) *(_r++) = *(_t++) - *(_m++);
	return re;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator * (const _rmatrix &m) const
{
	_rmatrix<TYPE> re;
	AMultB(re, *this, m);
	return re;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator | (const _rmatrix &m) const
{
	_rmatrix<TYPE> re;
	AMultBt(re, *this, m);
	return re;	
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator ^ (const _rmatrix &m) const
{
	_rmatrix<TYPE> re;
	AtMultB(re, *this, m);
	return re;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator * (TYPE c) const
{
	_rmatrix<TYPE> re(row, col);
	int n = row * col;
	TYPE *_t = element, *_r = re.element;
	while ( n-- ) *(_r++) = c * *(_t++);
	return re;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator / (TYPE c) const
{
	_rmatrix<TYPE> re(row, col);
	int n = row * col;
	TYPE *_t = element, *_r = re.element, ci = (TYPE)1.0 / c;
	while ( n-- ) *(_r++) = ci * *(_t++);
	return re;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator % (const _rmatrix &m) const
{
	_rmatrix<TYPE> x;
	SolveAxEqualB(*this, x, m);
	return x;
}

template <class TYPE>
_rmatrix<TYPE> _rmatrix<TYPE>::operator & (const _rmatrix &m) const
{
	_rmatrix<TYPE> x;
	SolveAtxEqualB(*this, x, m);
	return x;
}

template <class TYPE>
_rmatrix<TYPE> operator * (TYPE c, _rmatrix<TYPE> m)
{ 
	int n = m.row * m.col;
	TYPE *_m = m.element;
	while ( n-- ) *(_m++) *= c;
	return m;		
}

template <class TYPE>
int _rmatrix<TYPE>::RowSize(void) const
{
	return row;
}
	
template <class TYPE>
int _rmatrix<TYPE>::ColSize(void) const
{
	return col;
}

template <class TYPE>
TYPE _rmatrix<TYPE>::GetSparsity(void) const
{
	int n = row * col, nnz = 0;
	for ( int i = 0; i < n; i++ ) 
		if ( abs(element[i]) < 1e-6 ) nnz++;
	
	return (TYPE)nnz / (TYPE)n;
}
