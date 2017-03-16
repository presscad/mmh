#pragma once


// IndexMapping can be many-to-one, or one-to-one mapping
// in case of many-to-one mapping, inverseMapping stores only the first index.
// one-to-many mapping is also supported since inverse of one-to-many mapping is many-to-one mapping.
class IndexMapping
{
public:
	intvectorn mapping;
	intvectorn inverseMapping;

	IndexMapping(int max=0){init(max);}

	// sizeRange is optional.
	void init(int sizeDomain, int sizeRange=INT_MAX)
	{
		mapping.setSize(sizeDomain);
		if(sizeRange>sizeDomain)
			sizeRange=sizeDomain;
		inverseMapping.setSize(sizeRange);
		mapping.setAllValue(-1);	// denotes no mapping
		inverseMapping.setAllValue(-1);
	}

	int size() const {return mapping.size();}
	int sizeRange() const { return inverseMapping.size();}

	// set mapping such that map(i)==j
	void map(int i, int j)
	{
		ASSERT(mapping(i)==-1 || mapping(i)==j);
		mapping(i)=j;

		ASSERT(j>=0 && j< sizeRange());
		//if(inverseMapping(j)==-1 || inverseMapping(j)>i);
		inverseMapping(j)=i;
	}

	int operator()(int i) const	{ return mapping(i);}

	void operator()(intvectorn const& domain, intvectorn & range) const
	{
		range.reserve(domain.size());
		range.setSize(0);
		for(int i=0; i<domain.size(); i++)	
		{ 
			int r=mapping(domain(i));
			if(r!=-1 && range.findFirstIndex(r)==-1)
				range.pushBack(r);
		}
	}

	int inverse(int j) const	{ return inverseMapping(j);}
};


// analytic constrained SQP solver 
// - template argument baseSolver는 
//   QuadraticFunction, SparseQuadraticFunction, 
//   QuadraticFunctionHardConm, SparseQuadraticFunctionHardCon등이 사용될 수 있다.
// QuadraticFunctionHardCon이나 SparseQuadraticFunctionHardCon과는 달리
// equality constraint를 예외처리해서 실제 최적화과정에 사용되는 변수 개수를 줄인다.
// 사용법: QuadraticFunctionHardCon 이나 SparseQuadraticFunctionHardCon과 동일하지만,
// 항상 모든 addCon이 addSquared나 addSquaredWeighted보다 먼저 수행되어야한다.
template <class baseSolver>
class ConstrainedSQPsolver
{
	int mNumVar;
	int mNumCon;
	bitvectorn mEqCon;
	intvectorn mEqConIndex;
	vectorn mEqConValue;
	baseSolver* mSolver;
	IndexMapping toReorderedIndex;
	int mCurCon;
public:
	
	ConstrainedSQPsolver(int numVar, int numCon, int numCon2=0):mNumVar(numVar), mNumCon(numCon)
	{
		Msg::verify(numCon2==0, "linear equality constraints are not supported in this class yet - use sparseQuadraticFunctionHardCon instead.");

		mSolver=NULL;
		mEqCon.setSize(numVar);
		mEqConIndex.setSize(numCon);
		mEqConValue.setSize(numCon);
		mCurCon=0;
	}
	virtual ~ConstrainedSQPsolver(){ delete mSolver;}

	void addCon(int n, m_real coef1, int index1, ...)
	{
		Msg::verify(n==1, "linear combination constraints are not supported in this class yet - use sparseQuadraticFunctionHardCon instead.");

		m_real coef2;
		va_list marker;
		va_start( marker, index1);
		coef2=va_arg(marker, m_real);
		va_end(marker);

		mEqConIndex[mCurCon]=index1;
		mEqConValue[mCurCon]=-1.0*coef2/coef1;	// ax+b=0 -> x=-b/a
		ASSERT(!mEqCon[index1]);
		mEqCon.setAt(index1);
		mCurCon++;

		if(mCurCon==mNumCon)
		{
			// index map을 구성한다.
			toReorderedIndex.init(mNumVar, mNumVar-mNumCon);

			int reorderedIndex=0;
			for(int i=0; i<mNumVar; i++)
			{
				if(mEqCon[i]) 
					toReorderedIndex.map(i, mEqConIndex.findFirstIndex(i));
				else
				{
					toReorderedIndex.map(i, reorderedIndex);
					reorderedIndex++;
				}
			}

			mSolver=new baseSolver(mNumVar-mNumCon);
		}
	}

	// 4*(3x+4y+5z+1)^2 를 추가하고 싶으면, addSquaredWeighted(4.0, 3, 3.0, x, 4.0, y, 5.0, z, 1.0); where x=0, y=1, z=2
	void addSquaredWeighted(m_real weight, int N, ...)
	{
		ASSERT(toReorderedIndex.size());
		QuadraticFunction::SquaredTerm* term=new QuadraticFunction::SquaredTerm;

		term->index.reserve(N);
		term->coef.reserve(N+1);


		va_list marker;
		va_start( marker, N);     /* Initialize variable arguments. */

		m_real constants=0.0;

		term->index.setSize(0);
		term->coef.setSize(0);
		int i;
		for(i=0; i<N; i++)
		{
			m_real coef=va_arg(marker, m_real);
			m_real index=va_arg(marker, int);


			if(mEqCon[index])
				constants+=coef*mEqConValue[toReorderedIndex(index)];
			else
			{
				term->coef.pushBack(coef);
				term->index.push_back(toReorderedIndex(index));
			}
		}

		term->coef.pushBack(va_arg(marker, m_real)+constants);
		va_end(marker);

		term->coef*=sqrt(weight);

		if(term->index.size())
			mSolver->mListSQTerms.push_back(term);
	}

	// (3x+4y+5z+1)^2 을 추가하고 싶으면, addSquared(3, 3.0, x, 4.0, y, 5.0, z, 1.0);	where x=0, y=1, z=2
	void addSquared(int N, m_real coef1, int index1, ...)
	{
	
		QuadraticFunction::SquaredTerm* term=new QuadraticFunction::SquaredTerm;

		term->index.reserve(N);
		term->coef.reserve(N+1);

		va_list marker;
		va_start( marker, N);     /* Initialize variable arguments. */

		m_real constants=0.0;

		term->index.setSize(0);
		term->coef.setSize(0);
		int i;
		for(i=0; i<N; i++)
		{
			m_real coef=va_arg(marker, m_real);
			m_real index=va_arg(marker, int);


			if(mEqCon[index])
				constants+=coef*mEqConValue[toReorderedIndex(index)];
			else
			{
				term->coef.pushBack(coef);
				term->index.push_back(toReorderedIndex(index));
			}
		}

		term->coef.pushBack(va_arg(marker, m_real)+constants);
		va_end(marker);

		mSolver->mListSQTerms.push_back(term);

	}

	template <class MAT_TYPE>
	void buildSystem(MAT_TYPE & A, vectorn &b)
	{
		mSolver->buildSystem(A, b);
	}
	template <class MAT_TYPE>
	void solve(MAT_TYPE const& A, vectorn const& b, vectorn & x2)
	{
		vectorn x;
		mSolver->solve(A,b,x);

		x2.setSize(mNumVar);
		for(int i=0; i<mNumVar; i++)
		{
			if(mEqCon[i])
				x2[i]=mEqConValue[toReorderedIndex(i)];
			else
				x2[i]=x[toReorderedIndex(i)];
		}
	}
};

#include "dependency_support/gmm.h"
class ConstranedSQPsolverS : public ConstrainedSQPsolver<SparseQuadraticFunction>
{
public:
	typedef gmm::wsmatrixn MAT_TYPE;
	ConstranedSQPsolverS (int numVar, int numCon):ConstrainedSQPsolver<SparseQuadraticFunction>(numVar, numCon){}
	virtual ~ConstranedSQPsolverS (){}
};

#include "nr/nr.h"
namespace NR_OLD
{
void gradient_descent(	vectorn&, int, m_real, int&, m_real&,
				m_real (*func)(vectorn const&),
				void (*dfunc)(vectorn const&, vectorn&));
}
namespace _NRSolver
{
	// non-thread-safe
	extern void* mFunc;
}

// do not need to inherit this class. Just to demonstrate the interface.
class FuncAbstractClass
{
public:
	// return value should contain the initial solution.
	vectorn& getInout(){}
	inline m_real func(vectorn const& x){}
	inline void dfunc(vectorn const& x, vectorn& dx){}
};

// minimization of Func (conjugate gradient and gradient descent solvers are supported.)
// There also exists GSLsolver which has exactly the same interface with a lot more options.)

// usage: 
// NRSolver<functionType> solver(pfunction);
// solver.conjugateGradientSolve();

template <class Func>	// Func should have three functions: getInout, func, dfunc
class NRSolver
{
public:
	NRSolver(Func* function)	
	{
		_NRSolver::mFunc=(void*)function;
	}

	void conjugateGradientSolve( m_real tolerance=1.0e-6)
	{
		vectorn& x=((Func*)_NRSolver::mFunc)->getInout();
		int iter; DP fret;
		NR::frprmn(x, tolerance, iter, fret, func, dfunc);
	}

	void gradientDescentSolve( m_real ftol=1.0e-6)
	{
		int iter; DP fret;
		vectorn& x=((Func*)_NRSolver::mFunc)->getInout();
		NR_OLD::gradient_descent(x, x.size(), ftol, iter, fret, func, dfunc);
	}

	static DP func(Vec_I_DP & x)
	{
		return ((Func*)_NRSolver::mFunc)->func(x);
	}

	static void dfunc(Vec_I_DP &x, Vec_O_DP &dx)
	{
		((Func*)_NRSolver::mFunc)->dfunc(x, dx);
	}
};



// deprecated..
namespace NR_OLD
{
    void mnbrak(m_real&, m_real&, m_real&,
			m_real&, m_real&, m_real&, m_real (*func)(m_real)) ;

	m_real brent(m_real, m_real, m_real,
				m_real (*f)(m_real), m_real, m_real&) ;

	void linmin(vectorn&, vectorn&, int, m_real&, m_real (*func)(vectorn const&));


	void frprmn(vectorn&, int, m_real, int&, m_real&,
				m_real (*func)(vectorn const&),
				m_real (*dfunc)(vectorn const&, vectorn&));

	void error( char* );

}
