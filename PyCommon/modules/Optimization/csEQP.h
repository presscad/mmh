#pragma once

#include "../../external_libraries/BaseLib/baselib.h"
#include "../../external_libraries/baselib/math/optimize.h"


// Equality constrained Quadratic Program
// - analytic constrained SQP solver using Lagrange Multiplier
class EQP
{
private:
	SparseQuadraticFunctionHardCon _solver;
	ConstranedSQPsolverS::MAT_TYPE A;
	vectorn b, x;

public: // expose to python
	EQP(int numVar, int numCon):_solver(numVar, numCon) {}

	// objfunc에 4*(3x+4y+5z+1)^2 추가 -> addSquaredTerm(4, (0,1,2), (3,4,5,1));
	void addSquaredTerm(double weight, const object& indexes, const object& coeffs);
	// or -> addSquaredTerm2(4, (0,1,2), (3,4,5), 1);
	void addSquaredTerm2(double weight, const object& indexes, const object& coeffs, m_real constant);

	// constraint에 x + y + z = 1 추가 -> addConstraint((0,1,2), (1,1,1,-1));
	void addConstraint(const object& indexes, const object& coeffs);
	// or -> addConstraint2((0,1,2), (1,1,1), -1);
	void addConstraint2(const object& indexes, const object& coeffs, m_real constant);

	dict solve();

	void setVariableNum(int num) { _solver.mNumVar = num; }
	void setConstraintNum(int num) { _solver.mNumCon = num; }

	void clear() { _solver.clear(); }
	void clearSquaredTerms() { _solver.clearSquaredTerms(); }
	void clearConstraints() { _solver.clearConstraints(); }
};