
//
// Sparse Matrix:	Aug 11, 1998
//

#include "stdafx.h"
#include "mathclass.h"
#include "NR/nr.h"
#include "smatrixn.h"
#include <cmath>
using namespace std;
void smatrixn::assign(matrixn const& other)
{
	const DP TINY=1.0e-20;

	ASSERT(other.rows()==other.cols());

	int maxEntry=other.rows()*other.cols()+1;
	_s.setSize(maxEntry);
	_ij.setSize(maxEntry);
	NR::sprsin(other, TINY, _s, _ij);
	_trim();
}

void smatrixn::_trim()
{
	int k=_ij[rows()];
	// trim unnecessary elements.
	_s.resize(k);
	_ij.resize(k);
}

void smatrixn::mult(smatrixn const& a, smatrixn const& b)
{
	smatrixn bt;
	bt.transpose(b);
	multABt(a, bt);
}

void smatrixn::setMatrix(matrixn& A)
{
	int NP = A.cols();

	for (int i=0;i<NP;i++) 
		for (int j=0;j<NP;j++) 
			A[i][j]=0.0;

	for (int i=0;i<NP;i++) 
	{
		A[i][i]= _s[i];
		for (int j=_ij[i];j<=_ij[i+1]-1;j++) 
			A[i][_ij[j]]= _s[j];
	}
}

void smatrixn::multABt(smatrixn const& a, smatrixn const& b)
{
	const DP TINY=1.0e-20;

	int maxEntry=a.rows()*a.cols()+1;

	_ij.setSize(maxEntry);
	_s.setSize(maxEntry);
	_ij[0]=a._ij[0];
	NR::sprstm(a._s, a._ij, b._s, b._ij, TINY, _s, _ij);
	_trim();
}

TString smatrixn::output()
{
	/* raw output
	TString out;
	out.add("_s %s\n", _s.output().ptr());
	out.add("_ij %s\n", _ij.output().ptr());*/

	// readable output
	int n=rows();
	TString out;
	out.add("diagonal %s\n", _s.range(0,n).output().ptr());

	for(int i=0; i<n; i++)
	{
		
		out.add("[row %d]=", i);
		for(int j=_ij[i]; j<_ij[i+1]; j++)
		{
			// non-zero off-diagonal elements
			out.add("(%d ,%f), ", _ij[j], _s[j]);
		}
		out.add("\n");
	}
	return out;
}

vectornView smatrixn::diagonal()
{
	return _s.range(0, rows());
}

void smatrixn::transpose(smatrixn const& other)
{
	_s.setSize(other._s.size());
	_ij.setSize(other._ij.size());
	NR::sprstp(other._s, other._ij, _s, _ij);
}

void sm::multmat(vectorn & b, smatrixn const& A, vectorn const& x)
{
	b.setSize(x.size());
	NR::sprsax(A._s, A._ij, x, b);
}

void sm::multmatAtx(vectorn & b, smatrixn const& A, vectorn const& x)
{
	b.setSize(x.size());
	NR::sprstx(A._s, A._ij, x, b);
}


void sm::CGsolve(smatrixn const& A, vectorn const& b, vectorn& x)
{
	const bool verbose=false;// output debug message
	x=b;	// very bad initial solution -> should be improved
	m_real tol=1.0e-6;
	int iter; DP err;	// store results
	int itol=4;	// itol=1,2,3,4 specifying which convergence test is applied (see text)
	const int itmax=2000;
	
	struct solverHelper
	{
		smatrixn const& A;
		solverHelper(smatrixn const& a):A(a){}
		void atimes(vectorn const& x, vectorn& r, int useTranspose)
		{
			if(useTranspose)
				multmatAtx(r, A, x);
			else
				multmat(r, A, x);
		}

		void asolve(vectorn const&b, vectorn&x, int useTranspose)
		{
			int i;

			int n=b.size();
			for(i=0;i<n;i++) x[i]=((A._s[i] != 0.0) ? b[i]/A._s[i] : b[i]);
		}
	};
	
	solverHelper sh(A);

	// start of a copy of NR::linbcg
	{
		DP ak,akden,bk,bkden=1.0,bknum,bnrm,dxnrm,xnrm,zm1nrm,znrm;
		const DP EPS=1.0e-14;
		int j;

		int n=b.size();
		Vec_DP p(n),pp(n),r(n),rr(n),z(n),zz(n);
		iter=0;
		sh.atimes(x,r,0);
		for (j=0;j<n;j++) {
			r[j]=b[j]-r[j];
			rr[j]=r[j];
		}
		//atimes(r,rr,0);
		if (itol == 1) {
			bnrm=NR::snrm(b,itol);
			sh.asolve(r,z,0);
		}
		else if (itol == 2) {
			sh.asolve(b,z,0);
			bnrm=NR::snrm(z,itol);
			sh.asolve(r,z,0);
		}
		else if (itol == 3 || itol == 4) {
			sh.asolve(b,z,0);
			bnrm=NR::snrm(z,itol);
			sh.asolve(r,z,0);
			znrm=NR::snrm(z,itol);
		} else NR::nrerror("illegal itol in linbcg");
		if(verbose) cout << fixed ;
		while (iter < itmax) {
			++iter;
			sh.asolve(rr,zz,1);
			for (bknum=0.0,j=0;j<n;j++) bknum += z[j]*rr[j];
			if (iter == 1) {
				for (j=0;j<n;j++) {
					p[j]=z[j];
					pp[j]=zz[j];
				}
			} else {
				bk=bknum/bkden;
				for (j=0;j<n;j++) {
					p[j]=bk*p[j]+z[j];
					pp[j]=bk*pp[j]+zz[j];
				}
			}
			bkden=bknum;
			sh.atimes(p,z,0);
			for (akden=0.0,j=0;j<n;j++) akden += z[j]*pp[j];
			ak=bknum/akden;
			sh.atimes(pp,zz,1);
			for (j=0;j<n;j++) {
				x[j] += ak*p[j];
				r[j] -= ak*z[j];
				rr[j] -= ak*zz[j];
			}
			sh.asolve(r,z,0);
			if (itol == 1)
				err=NR::snrm(r,itol)/bnrm;
			else if (itol == 2)
				err=NR::snrm(z,itol)/bnrm;
			else if (itol == 3 || itol == 4) {
				zm1nrm=znrm;
				znrm=NR::snrm(z,itol);
				if (fabs(zm1nrm-znrm) > EPS*znrm) {
					dxnrm=fabs(ak)*NR::snrm(p,itol);
					err=znrm/fabs(zm1nrm-znrm)*dxnrm;
				} else {
					err=znrm/bnrm;
					continue;
				}
				xnrm=NR::snrm(x,itol);
				if (err <= 0.5*xnrm) err /= xnrm;
				else {
					err=znrm/bnrm;
					continue;
				}
			}
			if(verbose) cout << "iter=" << iter+1 << err << endl;
			if (err <= tol) break;
		}
	}
}

void sm::CGsolve(matrixn const& A, vectorn const& b, vectorn & x)
{
	smatrixn sA;
	sA.assign(A);

	CGsolve(sA, b, x);
}

void sm::testSparseMatrix()
{
	smatrixn sa;

	matrixn a(5,5);
	a.setAllValue(0);
	a.setDiagonal(vectorn(5, 3.0, 4.0, 5.0, 0.0, 5.0));
	a(0,2)=1.0;
	a(2,1)=7.0;
	a(2,3)=9.0;
	a(3,4)=2.0;
	a(4,3)=6.0;

	sa=a;
	cout<<a.output()<<endl;
	cout<< sa.output()<<endl;
	cout << sa.diagonal().output() <<endl;
	smatrixn at;
	at.transpose(sa);
	cout <<at.output()<<endl;

	cout << "matrixn multiplication test"<<endl;

	matrixn aat;
	aat.multABt(a, a);
	smatrixn saat;
	saat.multABt(sa, sa);
	cout << aat.output()<<endl;
	cout << saat.output()<<endl;

}