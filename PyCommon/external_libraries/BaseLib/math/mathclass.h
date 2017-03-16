#pragma once

#pragma message("Compiling math_macro.h - this should happen just once per project.\n")

// 만약 renderer에서 사용할 것이면
#include "../stdafx.h"

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#include <stdio.h>
//#include <tchar.h>

#include <math.h>
#include <assert.h>

#include <limits>

#ifndef	M_PI
#define	M_PI	3.14159265358979323846
#endif

#define EPS (1.0e-10)

#include <limits.h>
#include <float.h>
#include "math_macro.h"


// scalar binary operators
namespace s2
{
	m_real ADD(m_real a, m_real b) ;
	m_real SUB(m_real a, m_real b) ;
	m_real MULT(m_real a, m_real b);
	m_real DIV(m_real a, m_real b);
	m_real POW(m_real a, m_real b);
	m_real MINIMUM(m_real a, m_real b);
	m_real MAXIMUM(m_real a, m_real b);
	m_real GREATER(m_real a, m_real b);
	m_real GREATER_EQUAL(m_real a, m_real b);
	m_real SMALLER(m_real a, m_real b);
	m_real SMALLER_EQUAL(m_real a, m_real b);
	m_real EQUAL(m_real a, m_real b);
	m_real AVG(m_real a, m_real b);
	m_real BOUND(m_real a, m_real b);
	int INT_NOT_EQUAL(int a, int b);
	int INT_EQUAL(int a, int b);
}

namespace s1
{
	// (scalar->scalar연산)	
	void COS(m_real&b,m_real a);
	void SIN(m_real&b,m_real a);
	void EXP(m_real&b,m_real a);
	void NEG(m_real&b,m_real a); 
	void SQRT(m_real&b,m_real a);
	void SQUARE(m_real&b,m_real a);
	void ASSIGN(m_real&b,m_real a);
	void LOG(m_real&b,m_real a);
	void abs(m_real&b,m_real a);
	void SMOOTH_TRANSITION(m_real&b,m_real a);
	void RADD(m_real&b,m_real a);
	void RDIV(m_real&b,m_real a);
	void RSUB(m_real&b,m_real a);
	void RMULT(m_real&b,m_real a);
	void BOUND(m_real&b, m_real a);
	void INVERSE(m_real&b, m_real a);
}

class CAggregate
{
public:
	enum aggregateOP { LENGTH, RMS, SUM, AVG, SQUARESUM, MINIMUM, MAXIMUM } ;
	CAggregate(aggregateOP op)
	{
		switch(op)
		{
		case LENGTH:
		case RMS:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSquareSum;
			final_function=&CAggregate::FinalSqrt;
			break;
		case SUM:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSum;
			final_function=&CAggregate::FinalCur;
			break;
		case AVG:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSum;
			final_function=&CAggregate::FinalDivN;
			break;
		case SQUARESUM:
			init_function=&CAggregate::InitZero;
			update_function=&CAggregate::UpdateSquareSum;
			final_function=&CAggregate::FinalCur;
			break;
		case MINIMUM:
			init_function=&CAggregate::InitFMax;
			update_function=&CAggregate::UpdateMin;
			final_function=&CAggregate::FinalCur;
			break;
		case MAXIMUM:
			init_function=&CAggregate::InitFMin;
			update_function=&CAggregate::UpdateMax;
			final_function=&CAggregate::FinalCur;
			break;
		}
	};
	~CAggregate(){};

	inline m_real Init() { return (this->*init_function)();};
	inline void Update(m_real& cur, m_real v) { (this->*update_function)(cur,v);};
	inline m_real Final(m_real v, int n) { return (this->*final_function)(v,n);};
private:
	m_real (CAggregate::*init_function)() const;
	void (CAggregate::*update_function)(m_real &cur, m_real v) const;
	m_real (CAggregate::*final_function)(m_real cur, int n) const;

	m_real InitZero() const	;
	m_real InitFMax() const	;
	m_real InitFMin() const	;
	void UpdateSquareSum(m_real &cur, m_real v) const;
	void UpdateMin(m_real &cur, m_real v) const;
	void UpdateMax(m_real &cur, m_real v) const;
	void UpdateSum(m_real &cur, m_real v) const;
	m_real FinalSqrt(m_real cur, int n) const	;
	m_real FinalCur(m_real cur, int n) const	;
	m_real FinalDivN(m_real cur, int n) const	;
};

#include "complex.h"
#include "cmplxVectorN.h"

#include "interval.h"
#include "intervalN.h"

#include "vector_n.h"
#include "bitVectorN.h"
#include "matrix_n.h"
#include "hyperMatrixN.h"
#include "quater.h"
#include "vector.h"
#include "matrix.h"
#include "transf.h"
#include "DynamicTimeWarping.h"
#include "Metric.h"
//#include "smatrixn.h"// include only when needed.
//#include "optimize.h"// include only when needed.
//#include "operator.h"	// include only when needed.
//#include "quaterN.h" // include only when needed.
//#include "vector3N.h"// include only when needed.
//#include "BSpline.h" // include only when needed.
