#pragma once

#include "stdafx.h"

#include "../../common_sources/bputil.h"
#include "csEQP.h"
#include "BaseLibUtil.h"
//#include "../../external_libraries/BaseLib/baselib.h"
//#include "../../external_libraries/BaseLib/math/dependency_support/gmm.cpp"

BOOST_PYTHON_MODULE(csEQP)
{
	numeric::array::set_module_and_type("numpy", "ndarray");
	
	class_<EQP>("EQP", init<int, int>())
		.def("addSquaredTerm", &EQP::addSquaredTerm)
		.def("addSquaredTerm2", &EQP::addSquaredTerm2)
		.def("addConstraint", &EQP::addConstraint)
		.def("addConstraint2", &EQP::addConstraint2)
		.def("solve", &EQP::solve)
		.def("setVariableNum", &EQP::setVariableNum)
		.def("setConstraintNum", &EQP::setConstraintNum)
		.def("clear", &EQP::clear)
		.def("clearSquaredTerms", &EQP::clearSquaredTerms)
		.def("clearConstraints", &EQP::clearConstraints)
		;
}

void EQP::addSquaredTerm( double weight, const object& indexes, const object& coeffs)
{
	int n = len(indexes);
	intvectorn indexes_n(n);
	vectorn coeffs_n(n+1);
	
	for(int i=0; i<n; ++i)
	{
		indexes_n[i] = XI(indexes[i]);
		coeffs_n[i] = XD(coeffs[i]);
	}
	coeffs_n[n] = XD(coeffs[n]);

	if(weight == 1.)
		_solver.addSquared(indexes_n, coeffs_n);
	else
		_solver.addSquaredWeighted(weight, indexes_n, coeffs_n);
}

void EQP::addSquaredTerm2( double weight, const object& indexes, const object& coeffs, m_real constant )
{
	int n = len(indexes);
	intvectorn indexes_n(n);
	vectorn coeffs_n(n+1);
	
	for(int i=0; i<n; ++i)
	{
		indexes_n[i] = XI(indexes[i]);
		coeffs_n[i] = XD(coeffs[i]);
	}
	coeffs_n[n] = constant;

	if(weight == 1.)
		_solver.addSquared(indexes_n, coeffs_n);
	else
		_solver.addSquaredWeighted(weight, indexes_n, coeffs_n);
}

void EQP::addConstraint( const object& indexes, const object& coeffs )
{
	int n = len(indexes);
	intvectorn indexes_n(n);
	vectorn coeffs_n(n+1);

	for(int i=0; i<n; ++i)
	{
		indexes_n[i] = XI(indexes[i]);
		coeffs_n[i] = XD(coeffs[i]);
	}
	coeffs_n[n] = XD(coeffs[n]);

	_solver.addCon(indexes_n, coeffs_n);
}

void EQP::addConstraint2( const object& indexes, const object& coeffs, m_real constant )
{
	int n = len(indexes);
	intvectorn indexes_n(n);
	vectorn coeffs_n(n+1);

	for(int i=0; i<n; ++i)
	{
		indexes_n[i] = XI(indexes[i]);
		coeffs_n[i] = XD(coeffs[i]);
	}
	coeffs_n[n] = constant;

	_solver.addCon(indexes_n, coeffs_n);
}

dict EQP::solve()
{
	_solver.buildSystem(A, b);
	_solver.solve(A, b, x);

	dict result;

	bp::list ls1, ls2;
	for(int i=0; i<_solver.mNumVar; ++i)
		ls1.append(x[i]);
	for(int i=_solver.mNumVar; i<_solver.mNumVar+_solver.mNumCon; ++i)
		ls2.append(x[i]);

	result["x"] = ls1;
	result["lambda"] = ls2;

	return result;
}

