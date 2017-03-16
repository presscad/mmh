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

/*
CG, MINRES, and SYMMLQ are Krylov subspace methods for solving large
symmetric systems of linear equations.  CG (the conjugate-gradient
method) is reliable on positive-definite systems, while MINRES and
SYMMLQ are designed for indefinite systems. When these methods are
applied to an inconsistent system (that is, a singular symmetric
least-squares problem), CG could break down and SYMMLQ's solution
could explode, while MINRES would give a least-squares solution but
not necessarily the minimum-length solution (often called the
pseudoinverse solution).
To get minimum length solution, minres
*/
template <class TYPE>
bool MinRes(const _smatrix<TYPE> &A, _rmatrix<TYPE> &x, const _rmatrix<TYPE> &b, int maxiter, TYPE rtol)
{
	int n = b.RowSize();
	int itn = 0;
	int istop = 0;
	TYPE Anorm = 0;
	TYPE Acond = 0;
	TYPE rnorm = 0;
	TYPE ynorm = 0;

	x = Zeros<TYPE>(n,1);
	_rmatrix<TYPE> y = b;
	_rmatrix<TYPE> r1 = b;

	TYPE beta1 = sqrt(Inner(b, y));
	TYPE oldb   = 0;
	TYPE beta   = beta1;
	TYPE dbar   = 0;
	TYPE epsln  = 0;
	TYPE qrnorm = beta1;
	TYPE phibar = beta1;
	TYPE rhs1   = beta1;
	TYPE rhs2   = 0;
	TYPE tnorm2 = 0;
	TYPE ynorm2 = 0;
	TYPE cs     = -1;
	TYPE sn     = 0;
	TYPE s, alfa, eps, gmax, gmin, delta, gbar, Arnorm, phi, denom, oldeps, root, gamma, diag, test1, test2;
	_rmatrix<TYPE> w = Zeros<TYPE>(n,1);
	_rmatrix<TYPE> w2 = Zeros<TYPE>(n,1);
	_rmatrix<TYPE> r2 = r1;
	_rmatrix<TYPE> v, w1;

	eps = 1e-4;

	while ( maxiter-- )
	{
		itn++;

		s = 1 / beta;
		v = s * y;

		y = A * v;

		if ( itn >= 2 )
			y -= (beta / oldb) * r1;

		alfa   = Inner(v, y);
		y     -= (alfa / beta) * r2;
		r1     = r2;
		r2     = y;
		
		oldb   = beta;
		beta   = Inner(r2, y);
		if ( beta < 0 ) 
		{
			istop = 9;
			break;
		}

		beta   = sqrt(beta);
		tnorm2 = tnorm2 + alfa * alfa + oldb * oldb + beta * beta;

		if ( itn == 1 ) 
		{
			if ( beta / beta1 <= 10 * eps ) // if beta2 = 0 or ~ 0, terminate later.
				istop = -1;
		 ////tnorm2 = alfa**2  ??
		  gmax   = abs(alfa);       // alpha1
		  gmin   = gmax;            // alpha1
		}
		
		oldeps = epsln;
		delta  = cs * dbar + sn * alfa; // delta1 = 0         deltak
		gbar   = sn * dbar - cs * alfa; // gbar 1 = alfa1     gbar k
		epsln  =           sn*beta; // epsln2 = 0         epslnk+1
		dbar   =         - cs*beta; // dbar 2 = beta2     dbar k+1
		root   = sqrt(gbar * gbar + dbar * dbar); //norm([gbar dbar]);
		Arnorm = phibar * root;       // ||Ar{k-1}||

		// Compute the next plane rotation Qk

		gamma  = sqrt(gbar * gbar + beta * beta); //norm([gbar beta]); // gammak
		gamma  = max(gamma, eps);
		cs     = gbar / gamma;        // ck
		sn     = beta / gamma;        // sk
		phi    = cs * phibar ;        // phik
		phibar = sn * phibar ;        // phibark+1

		// Update  x.

		denom = 1 / gamma;
		w1    = w2;
		w2    = w;
		w     = (v - oldeps * w1 - delta * w2) * denom;
		x     = x + phi * w;

		// Go round again.

		gmax   = max(gmax, gamma);
		gmin   = min(gmin, gamma);
		TYPE z      = rhs1 / gamma;
		ynorm2 = z * z  + ynorm2;
		rhs1   = rhs2 - delta * z;
		rhs2   =      - epsln * z;

		// Estimate various norms.

		Anorm  = sqrt(tnorm2);
		ynorm  = sqrt(ynorm2);
		TYPE epsa   = Anorm * eps;
		TYPE epsx   = Anorm * ynorm * eps;
		diag   = gbar;
		
		if ( diag == 0 ) diag = epsa;

		qrnorm = phibar;
		rnorm  = qrnorm;
		test1  = rnorm / (Anorm * ynorm);    //  ||r|| / (||A|| ||x||)
		test2  = root / Anorm;      // ||Ar{k-1}|| / (||A|| ||r_{k-1}||)

		// Estimate  cond(A).
		// In this version we look at the diagonals of  R  in the
		// factorization of the lower Hessenberg matrix,  Q * H = R,
		// where H is the tridiagonal matrix from Lanczos with one
		// extra row, beta(k+1) e_k^T.

		Acond  = gmax / gmin;

		// See if any of the stopping criteria are satisfied.
		// In rare cases, istop is already -1 from above (Abar = const*I).

		if ( istop == 0 )
		{
			TYPE t1 = 1 + test1;		// These tests work if rtol < eps
			TYPE t2 = 1 + test2;
			if ( t2 <= 1 )
				istop = 2;				// A least-squares solution was found, given rtol
			if ( t1 <= 1 )
				istop = 1;				// A solution to Ax = b was found, given rtol
      
			if ( Acond >= 0.1 / eps )
				istop = 4;				// x has converged to an eigenvector
			if ( epsx  >= beta1 )
				istop = 3;				// Reasonable accuracy achieved, given eps
			//if rnorm <= epsx   , istop = 2; end
			//if rnorm <= epsr   , istop = 1; end
			if ( test2 <= rtol )
				istop = 2;				// A least-squares solution was found, given rtol
			if ( test1 <= rtol )
				istop = 1;				// A solution to Ax = b was found, given rtol
		}

		// See if it is time to print something.
		/*
		prnt   = false;
		if n      <= 40       , prnt = true; end
		if itn    <= 10       , prnt = true; end
		if itn    >= itnlim-10, prnt = true; end
		if mod(itn,10)==0     , prnt = true; end
		if qrnorm <= 10*epsx  , prnt = true; end
		if qrnorm <= 10*epsr  , prnt = true; end
		if Acond  <= 1e-2/eps , prnt = true; end
		if istop  ~=  0       , prnt = true; end

		if show & prnt
		  if mod(itn,10)==0, disp(' '); end
		  str1 = sprintf('//6g //12.5e //10.3e', itn,x(1),test1);
		  str2 = sprintf(' //10.3e',           test2);
		  str3 = sprintf(' //8.1e //8.1e',      Anorm,Acond);
		  str4 = sprintf(' //8.1e',            gbar/Anorm);
		  str  = [str1 str2 str3 str4];
		  fprintf('\n //s', str)

		  debug = false;  // true;
		  if debug   // Print true Arnorm.
				 // This works only if no preconditioning.
		vv = b - minresxxxA(A,x)  + shift*x;    // vv = b - (A - shift*I)*x
		ww =     minresxxxA(A,vv) - shift*vv;   // ww = (A - shift*I)*vv = "Ar"
			trueArnorm = norm(ww);
			fprintf('\n Arnorm = //12.4e   True ||Ar|| = //12.4e', Arnorm,trueArnorm)
		  end
		end // show & prnt
		*/
		if ( istop != 0 ) break;
	}

	return true;
}
