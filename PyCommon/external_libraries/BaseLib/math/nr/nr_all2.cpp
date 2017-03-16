#include "stdafx.h"
#include <cmath>
#include "nr.h"

using namespace std;

#include <complex>

extern complex<DP> aa,bb,cc,z0,dz;

void NR::hypdrv(const DP s, Vec_I_DP &yy, Vec_O_DP &dyyds)
{
	complex<DP> z,y[2],dyds[2];

	y[0]=complex<DP>(yy[0],yy[1]);
	y[1]=complex<DP>(yy[2],yy[3]);
	z=z0+s*dz;
	dyds[0]=y[1]*dz;
	dyds[1]=(aa*bb*y[0]-(cc-(aa+bb+1.0)*z)*y[1])*dz/(z*(1.0-z));
	dyyds[0]=real(dyds[0]);
	dyyds[1]=imag(dyds[0]);
	dyyds[2]=real(dyds[1]);
	dyyds[3]=imag(dyds[1]);
}


int kmax,kount;
DP dxsav;
Vec_DP *xp_p;
Mat_DP *yp_p;

complex<DP> NR::hypgeo(const complex<DP> &a, const complex<DP> &b,
	const complex<DP> &c, const complex<DP> &z)
{
	const DP EPS=1.0e-14;
	int nbad,nok;
	complex<DP> ans,y[2];
	Vec_DP yy(4);

	kmax=0;
	if (norm(z) <= 0.25) {
		hypser(a,b,c,z,ans,y[1]);
		return ans;
	}
	else if (real(z) < 0.0) z0=complex<DP>(-0.5,0.0);
	else if (real(z) <= 1.0) z0=complex<DP>(0.5,0.0);
	else z0=complex<DP>(0.0,imag(z) >= 0.0 ? 0.5 : -0.5);
	aa=a;
	bb=b;
	cc=c;
	dz=z-z0;
	hypser(aa,bb,cc,z0,y[0],y[1]);
	yy[0]=real(y[0]);
	yy[1]=imag(y[0]);
	yy[2]=real(y[1]);
	yy[3]=imag(y[1]);
	odeint(yy,0.0,1.0,EPS,0.1,0.0001,nok,nbad,hypdrv,bsstep);
	y[0]=complex<DP>(yy[0],yy[1]);
	return y[0];
}


namespace {
	DP func(DP funk(const DP), const DP x)
	{
		return funk(-log(x))/x;
	}
}

DP NR::midexp(DP funk(const DP), const DP aa, const DP bb, const int n)
{
	DP x,tnm,sum,del,ddel,a,b;
	static DP s;
	int it,j;

	b=exp(-aa);
	a=0.0;
	if (n == 1) {
		return (s=(b-a)*func(funk,0.5*(a+b)));
	} else {
		for(it=1,j=1;j<n-1;j++) it *= 3;
		tnm=it;
		del=(b-a)/(3.0*tnm);
		ddel=del+del;
		x=a+0.5*del;
		sum=0.0;
		for (j=0;j<it;j++) {
			sum += func(funk,x);
			x += ddel;
			sum += func(funk,x);
			x += del;
		}
		s=(s+(b-a)*sum/tnm)/3.0;
		return s;
	}
}
