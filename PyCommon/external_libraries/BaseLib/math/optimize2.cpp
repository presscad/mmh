#include "stdafx.h"
#include "mathclass.h"
#include "optimize.h"
namespace NR_OLD
{

#define TOL 2.0e-4

int    ncom;
m_real (*nrfunc) (vectorn const&);
static vectorn pcom, xicom;

m_real f1dim(m_real x)
{
	static vectorn xt; xt.setSize(ncom);

	for (int j=0; j<ncom; j++)
		xt[j] = pcom[j] + x * xicom[j];
	m_real f = (*nrfunc) (xt);

	return f;
}

void linmin( vectorn &p, vectorn &xi, int n, m_real &fret, m_real (*func) (vectorn const&))
{
	m_real xx, xmin, fx, fb, fa, bx, ax;

	ncom = n;
	pcom.setSize(n);
	xicom.setSize(n);
	nrfunc = func;

	for (int j=0; j<n; j++)
	{
		pcom[j] = p[j];
		xicom[j] = xi[j];
	}
  
	ax = 0.0;
	xx = 1.0;
	mnbrak(ax, xx, bx, fa, fx, fb, f1dim);
	fret = brent(ax, xx, bx, f1dim, TOL, xmin);
  
	for (int j=0; j<n; j++)
	{
		xi[j] *= xmin;
		p[j] += xi[j];
	}
}

#define ITMAX 200

void gradient_descent(vectorn &p, int n, m_real ftol, int &iter, m_real &fret,
			m_real (*func)  (vectorn const&),
			void (*dfunc) (vectorn const&,vectorn&))
{
	m_real fp;
	static vectorn xi; xi.setSize(n);

	for (iter=0; iter<ITMAX; iter++)
	{
		fp =(*func) (p);
		(*dfunc) (p, xi);

		linmin(p, xi, n, fret, func);
		if (2.0 * fabs(fret - fp) <= ftol * (fabs(fret) + fabs(fp) + DBL_EPSILON))
   	    	return;
	}

	error("Too many iterations in gradient_descent");
}
	
void frprmn(vectorn &p, int n, m_real ftol, int &iter, m_real &fret,
			m_real (*func)  (vectorn const&),
			m_real (*dfunc) (vectorn const&,vectorn&))
{
	m_real gg, gam, fp, dgg;

	static vectorn g;   g.setSize(n);
	static vectorn h;   h.setSize(n);
	static vectorn xi; xi.setSize(n);
  
	fp = (*dfunc) (p, xi);

	for (int j=0; j<n; j++)
	{
		g[j] = -xi[j];
		xi[j] = h[j] = g[j];
	}

	for (iter=0; iter<ITMAX; iter++)
	{
		linmin(p, xi, n, fret, func);
		if (2.0 * fabs(fret - fp) <= ftol * (fabs(fret) + fabs(fp) + DBL_EPSILON)) return;
		
		fp = (*dfunc) (p, xi);
		dgg = gg = 0.0;

		for (int j=0; j<n; j++)
		{
			gg += g[j] * g[j];
			dgg += (xi[j] + g[j]) * xi[j];
		}

		if (gg == 0.0) return;

		gam = dgg / gg;

		for (int j=0; j<n; j++)
		{
			g[j] = -xi[j];
			xi[j] = h[j] = g[j] + gam * h[j];
		}
	}

	error("Too many iterations in frprmn");
}

#undef ITMAX

}