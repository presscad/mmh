#ifndef UMFSOLVER_H
#define UMFSOLVER_H

#include "../../../dependencies/UMFPACK5.2/UMFPACK/include/umfpack.h"
#include "../../../dependencies/gmm-3.0/include/gmm/gmm_interface.h"
#include "../../../dependencies/gmm-3.0/include/gmm/gmm_kernel.h"




// add ../dependencies/UMFPACK5.2/ to input directory.
#pragma comment(lib, "AMD.lib")
#pragma comment(lib, "UMFPACK_noblas.lib")

namespace gmm
{
	typedef wsvector<m_real> wsvectorn;	// sparse vector for write operations
	typedef rsvector<m_real> rsvectorn; // sparse vector for read operations

	typedef row_matrix<wsvectorn> wsmatrixn;	// sparse matrix for write operations
	typedef csc_matrix<m_real> rsmatrixn;		// sparse matrix for read operations

	typedef linalg_traits<wsvectorn>::iterator wsv_iterator;
	typedef linalg_traits<wsvectorn>::const_iterator wsv_const_iterator;
	void conversion(rsmatrixn & out, wsmatrixn & in);


	void zero(wsmatrixn& inout, int n, int m);

}

class Umfsolver{
	
	intvectorn Ap;
	intvectorn Ai;
	vectorn Ax;
	int _n, _m ;

public:
	Umfsolver()	{}
	~Umfsolver(){}

	// factorize matrix A
	void umf_factorize(matrixn const& A)
	{
		_n = A.rows();
		_m = A.cols();

		Ai.setSize(0);
		Ax.setSize(0);
		Ap.setSize(_n+1);

		for (int j =0; j<_m; j++)
		{
			Ap[j] = Ai.size() ;

			for (int i =0; i<_n; i++)
			{
				//cout << "[ " << i << ", " << j <<" ]" << A[i][j] << endl;

				if (A[i][j] != 0.0)
				{
					Ai.pushBack(i);
					Ax.pushBack(A[i][j]);
				}
			}
		}

		
		Ap[_n] = Ai.size();		
	}

	// factorize gmm::wsmatrixn A
	void umf_factorize(gmm::wsmatrixn const& A)
	{
		_n = gmm::mat_nrows(A);
		_m = gmm::mat_ncols(A);

		Ai.setSize(0);
		Ax.setSize(0);
		Ap.setSize(_n+1);

		for (int j =0; j<_m; j++)
		{
			Ap[j] = Ai.size() ;

			for(gmm::wsv_const_iterator i=A[j].begin(); i!=A[j].end(); i++)
			{
				//cout << "[ " << i << ", " << j <<" ]" << A[i][j] << endl;

				if (*i!= 0.0)
				{
					Ai.pushBack(i.index());
					Ax.pushBack(*i);
				}
			}
		}

		
		Ap[_n] = Ai.size();		
	}

	// solve Ax=b
	void umf_solve(vectorn & x, vectorn const& b) const
	{
		int n=_n;
		int m=_m;

		x.setSize(m);
		void *Symbolic, *Numeric ;
		double *null = (double *) NULL ;
		(void) umfpack_di_symbolic (n, m, Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), &Symbolic, null, null) ;
		(void) umfpack_di_numeric (Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), Symbolic, &Numeric, null, null) ;
		umfpack_di_free_symbolic (&Symbolic) ;
		(void) umfpack_di_solve (UMFPACK_A, Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), x.dataPtr(), b.dataPtr(), Numeric, null, null) ;
		umfpack_di_free_numeric (&Numeric) ;
	}

private:
};

namespace sm
{
	template <class MAT_TYPE>
	static void UMFsolve(MAT_TYPE const& A, vectorn const& b, vectorn & x)
	{
		Umfsolver umf_A;
		umf_A.umf_factorize(A);
		umf_A.umf_solve(x , b);
	}
}

#endif