#include "../baselib/math/optimize.h"

// Equality-constrained quadratic programs
void QuadraticProgramming_with_EquilityConstraints()
{
	ConstranedSQPsolverS::MAT_TYPE A;
	vectorn b, x;

	// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	// 4*(3x+4y+5z+1)^2 를 추가하고 싶으면, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2

	cout << "minimize :	f(x,y) = x^2 + y^2 + z^2" << endl;
	cout << "subject to :	x + y + z = 1" << endl;

	SparseQuadraticFunctionHardCon solver0(3, 1);
	solver0.addSquared(1, 1., 0, 0.);
	solver0.addSquared(1, 1., 1, 0.);
	solver0.addSquared(1, 1., 2, 0.);
	solver0.addCon(3, 1., 0, 1., 1, 1., 2, -1.);
	solver0.buildSystem(A, b);
	solver0.solve(A, b, x);

	cout << "A" << endl;
	cout << A.nrows() << " " << A.ncols() << endl;
	cout << "x" << endl;
	cout << x.output() << endl;
	cout << endl;

	SparseQuadraticFunctionHardCon solver1(3, 1);
	solver1.addSquared(intvectorn(1, 0), vectorn(2, 1., 0.));
	solver1.addSquared(intvectorn(1, 1), vectorn(2, 1., 0.));
	solver1.addSquared(intvectorn(1, 2), vectorn(2, 1., 0.));
	solver1.addCon(intvectorn(3, 0, 1, 2), vectorn(4, 1., 1., 1., -1.));
	solver1.buildSystem(A, b);
	solver1.solve(A, b, x);

	cout << "A" << endl;
	cout << A.nrows() << " " << A.ncols() << endl;
	cout << "x" << endl;
	cout << x.output() << endl;
	cout << endl;


	cout << "minimize :	f(x,y) = x^2 + y^2" << endl;
	cout << "subject to :	x + y = 1" << endl;
	SparseQuadraticFunctionHardCon solver2(2, 1);
	solver2.addSquared(1, 1., 0, 0.);
	solver2.addSquared(1, 1., 1, 0.);
	solver2.addCon(2, 1., 0, 1., 1, -1.);
	solver2.buildSystem(A, b);
	solver2.solve(A, b, x);

	cout << "A" << endl;
	cout << A.nrows() << " " << A.ncols() << endl;
	cout << "x" << endl;
	cout << x.output() << endl;
	cout << endl;

}

void main()
{
	QuadraticProgramming_with_EquilityConstraints();
}
