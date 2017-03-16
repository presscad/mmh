#include "stdafx.h"
#include <cmath>
#include "NR_single_precision.h"
#include <iostream>
#include <limits>

using namespace std;

namespace NR_single_precision
{

	// constants
	const NR_single_precision::DP TINY2=1.0e-5;// original value 1.0e-20;
	const NR_single_precision::DP MAXIMUM_CONSTANTS=FLT_MAX;

	// global variables:
	// used in many functions
	NR_single_precision::Vec_INT *ija_p;
	NR_single_precision::Vec_DP *sa_p;
	NR_single_precision::Mat_DP *y_p;
	int idum;
	NR_single_precision::DP tt;
	int kmax,kount;
	NR_single_precision::DP dxsav;
	NR_single_precision::Vec_DP *xp_p;
	NR_single_precision::Mat_DP *yp_p;
	NR_single_precision::Vec_DP *xx_p,*yy_p,*sx_p,*sy_p,*ww_p;
	NR_single_precision::DP aa,offs;
	int ncom;

// used only in quad3d
NR_single_precision::DP xmax;

NR_single_precision::DP z1(const NR_single_precision::DP x, const NR_single_precision::DP y)
{
        return -sqrt(xmax*xmax-x*x-y*y);
}

NR_single_precision::DP z2(const NR_single_precision::DP x, const NR_single_precision::DP y)
{
        return sqrt(xmax*xmax-x*x-y*y);
}

NR_single_precision::DP yy1(const NR_single_precision::DP x)
{
        return -sqrt(xmax*xmax-x*x);
}

NR_single_precision::DP yy2(const NR_single_precision::DP x)
{
        return sqrt(xmax*xmax-x*x);
}


namespace {
	inline void get_psum(NR_single_precision::Mat_I_DP &p, NR_single_precision::Vec_O_DP &psum)
	{
		int n,m;
		NR_single_precision::DP sum;

		int mpts=p.rows();
		int ndim=p.cols();
		for (n=0;n<ndim;n++) {
			for (sum=0.0,m=0;m<mpts;m++) sum += p(m,n);
			psum(n)=sum;
		}
	}

	inline void shft2(NR_single_precision::DP &a, NR_single_precision::DP &b, const NR_single_precision::DP c)
	{
		a=b;
		b=c;
	}

	inline void shft3(NR_single_precision::DP &a, NR_single_precision::DP &b, NR_single_precision::DP &c, const NR_single_precision::DP d)
	{
		a=b;
		b=c;
		c=d;
	}
}



void NR_single_precision::addint(NR_single_precision::Mat_O_DP &uf, NR_single_precision::Mat_I_DP &uc, NR_single_precision::Mat_O_DP &res)
{
	int i,j;

	int nf=uf.rows();
	interp(res,uc);
	for (j=0;j<nf;j++)
		for (i=0;i<nf;i++)
			uf(i,j) += res(i,j);
}

void NR_single_precision::airy(const NR_single_precision::DP x, NR_single_precision::DP &ai, NR_single_precision::DP &bi, NR_single_precision::DP &aip, NR_single_precision::DP &bip)
{
	const NR_single_precision::DP PI=3.141592653589793238, ONOVRT=0.577350269189626;
	const NR_single_precision::DP THIRD=(1.0/3.0), TWOTHR=2.0*THIRD;
	NR_single_precision::DP absx,ri,rip,rj,rjp,rk,rkp,rootx,ry,ryp,z;

	absx=fabs(x);
	rootx=sqrt(absx);
	z=TWOTHR*absx*rootx;
	if (x > 0.0) {
		bessik(z,THIRD,ri,rk,rip,rkp);
		ai=rootx*ONOVRT*rk/PI;
		bi=rootx*(rk/PI+2.0*ONOVRT*ri);
		bessik(z,TWOTHR,ri,rk,rip,rkp);
		aip = -x*ONOVRT*rk/PI;
		bip=x*(rk/PI+2.0*ONOVRT*ri);
	} else if (x < 0.0) {
		bessjy(z,THIRD,rj,ry,rjp,ryp);
		ai=0.5*rootx*(rj-ONOVRT*ry);
		bi = -0.5*rootx*(ry+ONOVRT*rj);
		bessjy(z,TWOTHR,rj,ry,rjp,ryp);
		aip=0.5*absx*(ONOVRT*ry+rj);
		bip=0.5*absx*(ONOVRT*rj-ry);
	} else {
		ai=0.355028053887817;
		bi=ai/ONOVRT;
		aip = -0.258819403792807;
		bip = -aip/ONOVRT;
	}
}







void NR_single_precision::amebsa(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_O_DP &pb, NR_single_precision::DP &yb, const NR_single_precision::DP ftol,
	NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &), int &iter, const NR_single_precision::DP temptr)
{
	int i,ihi,ilo,j,n;
	NR_single_precision::DP rtol,yhi,ylo,ynhi,ysave,yt,ytry;

	int mpts=p.rows();
	int ndim=p.cols();
	NR_single_precision::Vec_DP psum(ndim);
	tt = -temptr;
	get_psum(p,psum);
	for (;;) {
		ilo=0;
		ihi=1;
		ynhi=ylo=y(0)+tt*log(ran1(idum));
		yhi=y(1)+tt*log(ran1(idum));
		if (ylo > yhi) {
			ihi=0;
			ilo=1;
			ynhi=yhi;
			yhi=ylo;
			ylo=ynhi;
		}
		for (i=3;i<=mpts;i++) {
			yt=y(i-1)+tt*log(ran1(idum));
			if (yt <= ylo) {
				ilo=i-1;
				ylo=yt;
			}
			if (yt > yhi) {
				ynhi=yhi;
				ihi=i-1;
				yhi=yt;
			} else if (yt > ynhi) {
				ynhi=yt;
			}
		}
		rtol=2.0*fabs(yhi-ylo)/(fabs(yhi)+fabs(ylo));
		if (rtol < ftol || iter < 0) {
			SWAP(y(0),y(ilo));
			for (n=0;n<ndim;n++)
				SWAP(p(0,n),p(ilo,n));
			break;
		}
		iter -= 2;
		ytry=amotsa(p,y,psum,pb,yb,funk,ihi,yhi,-1.0);
		if (ytry <= ylo) {
			ytry=amotsa(p,y,psum,pb,yb,funk,ihi,yhi,2.0);
		} else if (ytry >= ynhi) {
			ysave=yhi;
			ytry=amotsa(p,y,psum,pb,yb,funk,ihi,yhi,0.5);
			if (ytry >= ysave) {
				for (i=0;i<mpts;i++) {
					if (i != ilo) {
						for (j=0;j<ndim;j++) {
							psum(j)=0.5*(p(i,j)+p(ilo,j));
							p(i,j)=psum(j);
						}
						y(i)=funk(psum);
					}
				}
				iter -= ndim;
				get_psum(p,psum);
			}
		} else ++iter;
	}
}






void NR_single_precision::amoeba(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_IO_DP &y, const NR_single_precision::DP ftol, NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &),
	int &nfunk)
{
	const int NMAX=5000;
	const NR_single_precision::DP TINY=1.0e-10;
	int i,ihi,ilo,inhi,j;
	NR_single_precision::DP rtol,ysave,ytry;

	int mpts=p.rows();
	int ndim=p.cols();
	NR_single_precision::Vec_DP psum(ndim);
	nfunk=0;
	get_psum(p,psum);
	for (;;) {
		ilo=0;
		ihi = y(0)>y(1) ? (inhi=1,0) : (inhi=0,1);
		for (i=0;i<mpts;i++) {
			if (y(i) <= y(ilo)) ilo=i;
			if (y(i) > y(ihi)) {
				inhi=ihi;
				ihi=i;
			} else if (y(i) > y(inhi) && i != ihi) inhi=i;
		}
		rtol=2.0*fabs(y(ihi)-y(ilo))/(fabs(y(ihi))+fabs(y(ilo))+TINY);
		if (rtol < ftol) {
			SWAP(y(0),y(ilo));
			for (i=0;i<ndim;i++) SWAP(p(0,i),p(ilo,i));
			break;
		}
		if (nfunk >= NMAX) NR::nrerror("NMAX exceeded");
		nfunk += 2;
		ytry=amotry(p,y,psum,funk,ihi,-1.0);
		if (ytry <= y(ilo))
			ytry=amotry(p,y,psum,funk,ihi,2.0);
		else if (ytry >= y(inhi)) {
			ysave=y(ihi);
			ytry=amotry(p,y,psum,funk,ihi,0.5);
			if (ytry >= ysave) {
				for (i=0;i<mpts;i++) {
					if (i != ilo) {
						for (j=0;j<ndim;j++)
							p(i,j)=psum(j)=0.5*(p(i,j)+p(ilo,j));
						y(i)=funk(psum);
					}
				}
				nfunk += ndim;
				get_psum(p,psum);
			}
		} else --nfunk;
	}
}



NR_single_precision::DP NR_single_precision::amotry(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_O_DP &y, NR_single_precision::Vec_IO_DP &psum, NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &),
	const int ihi, const NR_single_precision::DP fac)
{
	int j;
	NR_single_precision::DP fac1,fac2,ytry;

	int ndim=p.cols();
	NR_single_precision::Vec_DP ptry(ndim);
	fac1=(1.0-fac)/ndim;
	fac2=fac1-fac;
	for (j=0;j<ndim;j++)
		ptry(j)=psum(j)*fac1-p(ihi,j)*fac2;
	ytry=funk(ptry);
	if (ytry < y(ihi)) {
		y(ihi)=ytry;
		for (j=0;j<ndim;j++) {
			psum(j) += ptry(j)-p(ihi,j);
			p(ihi,j)=ptry(j);
		}
	}
	return ytry;
}






NR_single_precision::DP NR_single_precision::amotsa(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_O_DP &y, NR_single_precision::Vec_IO_DP &psum, NR_single_precision::Vec_O_DP &pb, NR_single_precision::DP &yb,
	NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &), const int ihi, NR_single_precision::DP &yhi, const NR_single_precision::DP fac)
{
	int j;
	NR_single_precision::DP fac1,fac2,yflu,ytry;

	int ndim=p.cols();
	NR_single_precision::Vec_DP ptry(ndim);
	fac1=(1.0-fac)/ndim;
	fac2=fac1-fac;
	for (j=0;j<ndim;j++)
		ptry(j)=psum(j)*fac1-p(ihi,j)*fac2;
	ytry=funk(ptry);
	if (ytry <= yb) {
		for (j=0;j<ndim;j++) pb(j)=ptry(j);
		yb=ytry;
	}
	yflu=ytry-tt*log(ran1(idum));
	if (yflu < yhi) {
		y(ihi)=ytry;
		yhi=yflu;
		for (j=0;j<ndim;j++) {
			psum(j) += ptry(j)-p(ihi,j);
			p(ihi,j)=ptry(j);
		}
	}
	return yflu;
}

#include <iostream>
#include <iomanip>

#include <cstdlib>



namespace {
	inline NR_single_precision::DP alen(const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP c, const NR_single_precision::DP d)
	{
		return sqrt((b-a)*(b-a)+(d-c)*(d-c));
	}
}

void NR_single_precision::anneal(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_IO_INT &iorder)
{
	const NR_single_precision::DP TFACTR=0.9;
	bool ans;
	int i,i1,i2,idec,idum,j,k,nn,nover,nlimit,nsucc;
	static NR_single_precision::Vec_INT n(6);
	unsigned long iseed;
	NR_single_precision::DP path,de,t;

	int ncity=x.size();
	nover=100*ncity;
	nlimit=10*ncity;
	path=0.0;
	t=0.5;
	for (i=0;i<ncity-1;i++) {
		i1=iorder(i);
		i2=iorder(i+1);
		path += alen(x(i1),x(i2),y(i1),y(i2));
	}
	i1=iorder(ncity-1);
	i2=iorder(0);
	path += alen(x(i1),x(i2),y(i1),y(i2));
	idum = -1;
	iseed=111;
	cout << fixed ;
	for (j=0;j<100;j++) {
		nsucc=0;
		for (k=0;k<nover;k++) {
			do {
				n(0)=int(ncity*ran3(idum));
				n(1)=int((ncity-1)*ran3(idum));
				if (n(1) >= n(0)) ++n(1);
				nn=(n(0)-n(1)+ncity-1) % ncity;
			} while (nn<2);
			idec=irbit1(iseed);
			if (idec == 0) {
				n(2)=n(1)+int(abs(nn-1)*ran3(idum))+1;
				n(2) %= ncity;
				de=trncst(x,y,iorder,n);
				ans=metrop(de,t);
				if (ans) {
					++nsucc;
					path += de;
					trnspt(iorder,n);
				}
			} else {
				de=revcst(x,y,iorder,n);
				ans=metrop(de,t);
				if (ans) {
					++nsucc;
					path += de;
					reverse(iorder,n);
				}
			}
			if (nsucc >= nlimit) break;
		}
		cout << endl << "T = " << t;
		cout << "	 Path Length = " << path << endl;
		cout << "Successful Moves: " << nsucc << endl;
		t *= TFACTR;
		if (nsucc == 0) return;
	}
}





NR_single_precision::DP NR_single_precision::anorm2(NR_single_precision::Mat_I_DP &a)
{
	int i,j;
	NR_single_precision::DP sum=0.0;

	int n=a.rows();
	for (j=0;j<n;j++)
		for (i=0;i<n;i++)
			sum += a(i,j)*a(i,j);
	return sqrt(sum)/n;
}

#include <limits>




void NR_single_precision::arcsum(NR_single_precision::Vec_I_ULNG &iin, NR_single_precision::Vec_O_ULNG &iout, unsigned long ja,
	const int nwk, const unsigned long nrad, const unsigned long nc)
{
	int karry=0;
	unsigned long j,jtmp;

	for (j=nwk-1;j>nc;j--) {
		jtmp=ja;
		ja /= nrad;
		iout(j)=iin(j)+(jtmp-ja*nrad)+karry;
		if (iout(j) >= nrad) {
			iout(j) -= nrad;
			karry=1;
		} else karry=0;
	}
	iout(nc)=iin(nc)+ja+karry;
}



void NR_single_precision::asolve(NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_O_DP &x, const int itrnsp)
{
	int i;

	int n=b.size();
	for(i=0;i<n;i++) x(i)=((*sa_p)(i) != 0.0 ? b(i)/(*sa_p)(i) : b(i));
}




void NR_single_precision::atimes(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &r, const int itrnsp)
{
	if (itrnsp) sprstx(*sa_p,*ija_p,x,r);
	else sprsax(*sa_p,*ija_p,x,r);
}



void NR_single_precision::avevar(NR_single_precision::Vec_I_DP &data, NR_single_precision::DP &ave, NR_single_precision::DP &var)
{
	NR_single_precision::DP s,ep;
	int j;

	int n=data.size();
	ave=0.0;
	for (j=0;j<n;j++) ave += data(j);
	ave /= n;
	var=ep=0.0;
	for (j=0;j<n;j++) {
		s=data(j)-ave;
		ep += s;
		var += s*s;
	}
	var=(var-ep*ep/n)/(n-1);
}

#include <iostream>
#include <iomanip>



namespace badluk
{
int main(void)	// Program badluk
{
	const int IYBEG=2000,IYEND=2100;
	const NR_single_precision::DP ZON=-5.0;
	int ic,icon,idwk,im,iyyy,jd,jday,n;
	NR_single_precision::DP timzon=ZON/24.0,frac;

	cout << endl << "Full moons on Friday the 13th from ";
	cout << IYBEG << " to " << IYEND << endl;
	for (iyyy=IYBEG;iyyy<=IYEND;iyyy++) {
		for (im=1;im<=12;im++) {
			jday=NR_single_precision::julday(im,13,iyyy);
			idwk=int((jday+1) % 7);
			if (idwk == 5) {
				n=int(12.37*(iyyy-1900+(im-0.5)/12.0));
				icon=0;
				for (;;) {
					NR_single_precision::flmoon(n,2,jd,frac);
					frac=24.0*(frac+timzon);
					if (frac < 0.0) {
						--jd;
						frac += 24.0;
					}
					if (frac > 12.0) {
						++jd;
						frac -= 12.0;
					} else
						frac += 12.0;
					if (jd == jday) {
						cout << endl << im;
						cout << "/13/" << iyyy << endl;
						cout << fixed ;
						cout << "Full moon" << frac;
						cout << " hrs after midnight (EST)" << endl;
						break;
					} else {
						ic=(jday >= jd ? 1 : -1);
						if (ic == (-icon)) break;
						icon=ic;
						n += ic;
					}
				}
			}
		}
	}
	return 0;
}

}
#include <limits>



void NR_single_precision::balanc(NR_single_precision::Mat_IO_DP &a)
{
	const NR_single_precision::DP RADIX = numeric_limits<NR_single_precision::DP>::radix;
	int i,j,last=0;
	NR_single_precision::DP s,r,g,f,c,sqrdx;

	int n=a.rows();
	sqrdx=RADIX*RADIX;
	while (last == 0) {
		last=1;
		for (i=0;i<n;i++) {
			r=c=0.0;
			for (j=0;j<n;j++)
				if (j != i) {
					c += fabs(a(j,i));
					r += fabs(a(i,j));
				}
			if (c != 0.0 && r != 0.0) {
				g=r/RADIX;
				f=1.0;
				s=c+r;
				while (c<g) {
					f *= RADIX;
					c *= sqrdx;
				}
				g=r*RADIX;
				while (c>g) {
					f /= RADIX;
					c /= sqrdx;
				}
				if ((c+r)/f < 0.95*s) {
					last=0;
					g=1.0/f;
					for (j=0;j<n;j++) a(i,j) *= g;
					for (j=0;j<n;j++) a(j,i) *= f;
				}
			}
		}
	}
}



void NR_single_precision::banbks(NR_single_precision::Mat_I_DP &a, const int m1, const int m2, NR_single_precision::Mat_I_DP &al,
	NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_IO_DP &b)
{
	int i,j,k,l,mm;
	NR_single_precision::DP dum;

	int n=a.rows();
	mm=m1+m2+1;
	l=m1;
	for (k=0;k<n;k++) {
		j=indx(k)-1;
		if (j!=k) SWAP(b(k),b(j));
		if (l<n) l++;
		for (j=k+1;j<l;j++) b(j) -= al(k,j-k-1)*b(k);
	}
	l=1;
	for (i=n-1;i>=0;i--) {
		dum=b(i);
		for (k=1;k<l;k++) dum -= a(i,k)*b(k+i);
		b(i)=dum/a(i,0);
		if (l<mm) l++;
	}
}





void NR_single_precision::bandec(NR_single_precision::Mat_IO_DP &a, const int m1, const int m2, NR_single_precision::Mat_O_DP &al,
	NR_single_precision::Vec_O_INT &indx, NR_single_precision::DP &d)
{
	const NR_single_precision::DP TINY=1.0e-20;
	int i,j,k,l,mm;
	NR_single_precision::DP dum;

	int n=a.rows();
	mm=m1+m2+1;
	l=m1;
	for (i=0;i<m1;i++) {
		for (j=m1-i;j<mm;j++) a(i,j-l)=a(i,j);
		l--;
		for (j=mm-l-1;j<mm;j++) a(i,j)=0.0;
	}
	d=1.0;
	l=m1;
	for (k=0;k<n;k++) {
		dum=a(k,0);
		i=k;
		if (l<n) l++;
		for (j=k+1;j<l;j++) {
			if (fabs(a(j,0)) > fabs(dum)) {
				dum=a(j,0);
				i=j;
			}
		}
		indx(k)=i+1;
		if (dum == 0.0) a(k,0)=TINY;
		if (i != k) {
			d = -d;
			for (j=0;j<mm;j++) SWAP(a(k,j),a(i,j));
		}
		for (i=k+1;i<l;i++) {
			dum=a(i,0)/a(k,0);
			al(k,i-k-1)=dum;
			for (j=1;j<mm;j++) a(i,j-1)=a(i,j)-dum*a(k,j);
			a(i,mm-1)=0.0;
		}
	}
}



void NR_single_precision::banmul(NR_single_precision::Mat_I_DP &a, const int m1, const int m2, NR_single_precision::Vec_I_DP &x,
	NR_single_precision::Vec_O_DP &b)
{
	int i,j,k,tmploop;

	int n=a.rows();
	for (i=0;i<n;i++) {
		k=i-m1;
		tmploop=MIN(m1+m2+1,int(n-k));
		b(i)=0.0;
		for (j=MAX(0,-k);j<tmploop;j++) b(i) += a(i,j)*x(j+k);
	}
}



void NR_single_precision::bcucof(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &y1, NR_single_precision::Vec_I_DP &y2, NR_single_precision::Vec_I_DP &y12,
	const NR_single_precision::DP d1, const NR_single_precision::DP d2, NR_single_precision::Mat_O_DP &c)
{
	static int wt_d[16*16]=
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
		-3, 0, 0, 3, 0, 0, 0, 0,-2, 0, 0,-1, 0, 0, 0, 0,
		2, 0, 0,-2, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0,-3, 0, 0, 3, 0, 0, 0, 0,-2, 0, 0,-1,
		0, 0, 0, 0, 2, 0, 0,-2, 0, 0, 0, 0, 1, 0, 0, 1,
		-3, 3, 0, 0,-2,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,-3, 3, 0, 0,-2,-1, 0, 0,
		9,-9, 9,-9, 6, 3,-3,-6, 6,-6,-3, 3, 4, 2, 1, 2,
		-6, 6,-6, 6,-4,-2, 2, 4,-3, 3, 3,-3,-2,-1,-1,-2,
		2,-2, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 2,-2, 0, 0, 1, 1, 0, 0,
		-6, 6,-6, 6,-3,-3, 3, 3,-4, 4, 2,-2,-2,-2,-1,-1,
		4,-4, 4,-4, 2, 2,-2,-2, 2,-2,-2, 2, 1, 1, 1, 1};
	int l,k,j,i;
	NR_single_precision::DP xx,d1d2;
	NR_single_precision::Vec_DP cl(16),x(16);
	NR_single_precision::Mat_INT wt(wt_d, 16,16);

	d1d2=d1*d2;
	for (i=0;i<4;i++) {
		x(i)=y(i);
		x(i+4)=y1(i)*d1;
		x(i+8)=y2(i)*d2;
		x(i+12)=y12(i)*d1d2;
	}
	for (i=0;i<16;i++) {
		xx=0.0;
		for (k=0;k<16;k++) xx += wt(i,k)*x(k);
		cl(i)=xx;
	}
	l=0;
	for (i=0;i<4;i++)
		for (j=0;j<4;j++) c(i,j)=cl(l++);
}



void NR_single_precision::bcuint(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &y1, NR_single_precision::Vec_I_DP &y2, NR_single_precision::Vec_I_DP &y12,
	const NR_single_precision::DP x1l, const NR_single_precision::DP x1u, const NR_single_precision::DP x2l, const NR_single_precision::DP x2u,
	const NR_single_precision::DP x1, const NR_single_precision::DP x2, NR_single_precision::DP &ansy, NR_single_precision::DP &ansy1, NR_single_precision::DP &ansy2)
{
	int i;
	NR_single_precision::DP t,u,d1,d2;
	NR_single_precision::Mat_DP c(4,4);

	d1=x1u-x1l;
	d2=x2u-x2l;
	bcucof(y,y1,y2,y12,d1,d2,c);
	if (x1u == x1l || x2u == x2l)
		NR::nrerror("Bad input in routine bcuint");
	t=(x1-x1l)/d1;
	u=(x2-x2l)/d2;
	ansy=ansy2=ansy1=0.0;
	for (i=3;i>=0;i--) {
		ansy=t*ansy+((c(i,3)*u+c(i,2))*u+c(i,1))*u+c(i,0);
		ansy2=t*ansy2+(3.0*c(i,3)*u+2.0*c(i,2))*u+c(i,1);
		ansy1=u*ansy1+(3.0*c(3,i)*t+2.0*c(2,i))*t+c(1,i);
	}
	ansy1 /= d1;
	ansy2 /= d2;
}



void NR_single_precision::beschb(const NR_single_precision::DP x, NR_single_precision::DP &gam1, NR_single_precision::DP &gam2, NR_single_precision::DP &gampl, NR_single_precision::DP &gammi)
{
	ASSERT(0);
	/*
	const int NUSE1=7, NUSE2=8;
	static const NR_single_precision::DP c1_d(7) = {
		-1.142022680371168e0,6.5165112670737e-3,
		3.087090173086e-4,-3.4706269649e-6,6.9437664e-9,
		3.67795e-11,-1.356e-13};
	static const NR_single_precision::DP c2_d(8) = {
		1.843740587300905e0,-7.68528408447867e-2,
		1.2719271366546e-3,-4.9717367042e-6,-3.31261198e-8,
		2.423096e-10,-1.702e-13,-1.49e-15};
	NR_single_precision::DP xx;
	static NR_single_precision::Vec_DP c1(c1_d,7),c2(c2_d,8);

	xx=8.0*x*x-1.0;
	gam1=chebev(-1.0,1.0,c1,NUSE1,xx);
	gam2=chebev(-1.0,1.0,c2,NUSE2,xx);
	gampl= gam2-x*gam1;
	gammi= gam2+x*gam1;*/
}


#include <limits>



NR_single_precision::DP NR_single_precision::bessi(const int n, const NR_single_precision::DP x)
{
	const NR_single_precision::DP ACC=200.0;
	const int IEXP=numeric_limits<NR_single_precision::DP>::max_exponent/2;
	int j,k;
	NR_single_precision::DP bi,bim,bip,dum,tox,ans;

	if (n < 2) NR::nrerror("Index n less than 2 in bessi");
	if (x*x <= 8.0*DBL_MIN) return 0.0;
	else {
		tox=2.0/fabs(x);
		bip=ans=0.0;
		bi=1.0;
		for (j=2*(n+int(sqrt(ACC*n)));j>0;j--) {
			bim=bip+j*tox*bi;
			bip=bi;
			bi=bim;
			dum=frexp(bi,&k);
			if (k > IEXP) {
				ans=ldexp(ans,-IEXP);
				bi=ldexp(bi,-IEXP);
				bip=ldexp(bip,-IEXP);
			}
			if (j == n) ans=bip;
		}
		ans *= bessi0(x)/bi;
		return x < 0.0 && (n & 1) ? -ans : ans;
	}
}





NR_single_precision::DP NR_single_precision::bessi0(const NR_single_precision::DP x)
{
	NR_single_precision::DP ax,ans,y;

	if ((ax=fabs(x)) < 3.75) {
		y=x/3.75;
		y*=y;
		ans=1.0+y*(3.5156229+y*(3.0899424+y*(1.2067492
			+y*(0.2659732+y*(0.360768e-1+y*0.45813e-2)))));
	} else {
		y=3.75/ax;
		ans=(exp(ax)/sqrt(ax))*(0.39894228+y*(0.1328592e-1
			+y*(0.225319e-2+y*(-0.157565e-2+y*(0.916281e-2
			+y*(-0.2057706e-1+y*(0.2635537e-1+y*(-0.1647633e-1
			+y*0.392377e-2))))))));
	}
	return ans;
}





NR_single_precision::DP NR_single_precision::bessi1(const NR_single_precision::DP x)
{
	NR_single_precision::DP ax,ans,y;

	if ((ax=fabs(x)) < 3.75) {
		y=x/3.75;
		y*=y;
		ans=ax*(0.5+y*(0.87890594+y*(0.51498869+y*(0.15084934
			+y*(0.2658733e-1+y*(0.301532e-2+y*0.32411e-3))))));
	} else {
		y=3.75/ax;
		ans=0.2282967e-1+y*(-0.2895312e-1+y*(0.1787654e-1
			-y*0.420059e-2));
		ans=0.39894228+y*(-0.3988024e-1+y*(-0.362018e-2
			+y*(0.163801e-2+y*(-0.1031555e-1+y*ans))));
		ans *= (exp(ax)/sqrt(ax));
	}
	return x < 0.0 ? -ans : ans;
}


#include <limits>



void NR_single_precision::bessik(const NR_single_precision::DP x, const NR_single_precision::DP xnu, NR_single_precision::DP &ri, NR_single_precision::DP &rk, NR_single_precision::DP &rip, NR_single_precision::DP &rkp)
{
	const int MAXIT=10000;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN/EPS;
	const NR_single_precision::DP XMIN=2.0, PI=3.141592653589793;
	NR_single_precision::DP a,a1,b,c,d,del,del1,delh,dels,e,f,fact,fact2,ff,gam1,gam2,
		gammi,gampl,h,p,pimu,q,q1,q2,qnew,ril,ril1,rimu,rip1,ripl,
		ritemp,rk1,rkmu,rkmup,rktemp,s,sum,sum1,x2,xi,xi2,xmu,xmu2;
	int i,l,nl;

	if (x <= 0.0 || xnu < 0.0) NR::nrerror("bad arguments in bessik");
	nl=int(xnu+0.5);
	xmu=xnu-nl;
	xmu2=xmu*xmu;
	xi=1.0/x;
	xi2=2.0*xi;
	h=xnu*xi;
	if (h < FPMIN) h=FPMIN;
	b=xi2*xnu;
	d=0.0;
	c=h;
	for (i=0;i<MAXIT;i++) {
		b += xi2;
		d=1.0/(b+d);
		c=b+1.0/c;
		del=c*d;
		h=del*h;
		if (fabs(del-1.0) <= EPS) break;
	}
	if (i >= MAXIT)
		NR::nrerror("x too large in bessik; try asymptotic expansion");
	ril=FPMIN;
	ripl=h*ril;
	ril1=ril;
	rip1=ripl;
	fact=xnu*xi;
	for (l=nl-1;l >= 0;l--) {
		ritemp=fact*ril+ripl;
		fact -= xi;
		ripl=fact*ritemp+ril;
		ril=ritemp;
	}
	f=ripl/ril;
	if (x < XMIN) {
		x2=0.5*x;
		pimu=PI*xmu;
		fact = (fabs(pimu) < EPS ? 1.0 : pimu/sin(pimu));
		d = -log(x2);
		e=xmu*d;
		fact2 = (fabs(e) < EPS ? 1.0 : sinh(e)/e);
		beschb(xmu,gam1,gam2,gampl,gammi);
		ff=fact*(gam1*cosh(e)+gam2*fact2*d);
		sum=ff;
		e=exp(e);
		p=0.5*e/gampl;
		q=0.5/(e*gammi);
		c=1.0;
		d=x2*x2;
		sum1=p;
		for (i=1;i<=MAXIT;i++) {
			ff=(i*ff+p+q)/(i*i-xmu2);
			c *= (d/i);
			p /= (i-xmu);
			q /= (i+xmu);
			del=c*ff;
			sum += del;
			del1=c*(p-i*ff);
			sum1 += del1;
			if (fabs(del) < fabs(sum)*EPS) break;
		}
		if (i > MAXIT) NR::nrerror("bessk series failed to converge");
		rkmu=sum;
		rk1=sum1*xi2;
	} else {
		b=2.0*(1.0+x);
		d=1.0/b;
		h=delh=d;
		q1=0.0;
		q2=1.0;
		a1=0.25-xmu2;
		q=c=a1;
		a = -a1;
		s=1.0+q*delh;
		for (i=1;i<MAXIT;i++) {
			a -= 2*i;
			c = -a*c/(i+1.0);
			qnew=(q1-b*q2)/a;
			q1=q2;
			q2=qnew;
			q += c*qnew;
			b += 2.0;
			d=1.0/(b+a*d);
			delh=(b*d-1.0)*delh;
			h += delh;
			dels=q*delh;
			s += dels;
			if (fabs(dels/s) <= EPS) break;
		}
		if (i >= MAXIT) NR::nrerror("bessik: failure to converge in cf2");
		h=a1*h;
		rkmu=sqrt(PI/(2.0*x))*exp(-x)/s;
		rk1=rkmu*(xmu+x+0.5-h)*xi;
	}
	rkmup=xmu*xi*rkmu-rk1;
	rimu=xi/(f*rkmu-rkmup);
	ri=(rimu*ril1)/ril;
	rip=(rimu*rip1)/ril;
	for (i=1;i <= nl;i++) {
		rktemp=(xmu+i)*xi2*rk1+rkmu;
		rkmu=rk1;
		rk1=rktemp;
	}
	rk=rkmu;
	rkp=xnu*xi*rkmu-rk1;
}


#include <limits>



NR_single_precision::DP NR_single_precision::bessj(const int n, const NR_single_precision::DP x)
{
	const NR_single_precision::DP ACC=160.0;
	const int IEXP=numeric_limits<NR_single_precision::DP>::max_exponent/2;
	bool jsum;
	int j,k,m;
	NR_single_precision::DP ax,bj,bjm,bjp,dum,sum,tox,ans;

	if (n < 2) NR::nrerror("Index n less than 2 in bessj");
	ax=fabs(x);
	if (ax*ax <= 8.0*DBL_MIN) return 0.0;
	else if (ax > NR_single_precision::DP(n)) {
		tox=2.0/ax;
		bjm=bessj0(ax);
		bj=bessj1(ax);
		for (j=1;j<n;j++) {
			bjp=j*tox*bj-bjm;
			bjm=bj;
			bj=bjp;
		}
		ans=bj;
	} else {
		tox=2.0/ax;
		m=2*((n+int(sqrt(ACC*n)))/2);
		jsum=false;
		bjp=ans=sum=0.0;
		bj=1.0;
		for (j=m;j>0;j--) {
			bjm=j*tox*bj-bjp;
			bjp=bj;
			bj=bjm;
			dum=frexp(bj,&k);
			if (k > IEXP) {
				bj=ldexp(bj,-IEXP);
				bjp=ldexp(bjp,-IEXP);
				ans=ldexp(ans,-IEXP);
				sum=ldexp(sum,-IEXP);
			}
			if (jsum) sum += bj;
			jsum=!jsum;
			if (j == n) ans=bjp;
		}
		sum=2.0*sum-bj;
		ans /= sum;
	}
	return x < 0.0 && (n & 1) ? -ans : ans;
}





NR_single_precision::DP NR_single_precision::bessj0(const NR_single_precision::DP x)
{
	NR_single_precision::DP ax,z,xx,y,ans,ans1,ans2;

	if ((ax=fabs(x)) < 8.0) {
		y=x*x;
		ans1=57568490574.0+y*(-13362590354.0+y*(651619640.7
			+y*(-11214424.18+y*(77392.33017+y*(-184.9052456)))));
		ans2=57568490411.0+y*(1029532985.0+y*(9494680.718
			+y*(59272.64853+y*(267.8532712+y*1.0))));
		ans=ans1/ans2;
	} else {
		z=8.0/ax;
		y=z*z;
		xx=ax-0.785398164;
		ans1=1.0+y*(-0.1098628627e-2+y*(0.2734510407e-4
			+y*(-0.2073370639e-5+y*0.2093887211e-6)));
		ans2 = -0.1562499995e-1+y*(0.1430488765e-3
			+y*(-0.6911147651e-5+y*(0.7621095161e-6
			-y*0.934945152e-7)));
		ans=sqrt(0.636619772/ax)*(cos(xx)*ans1-z*sin(xx)*ans2);
	}
	return ans;
}





NR_single_precision::DP NR_single_precision::bessj1(const NR_single_precision::DP x)
{
	NR_single_precision::DP ax,z,xx,y,ans,ans1,ans2;

	if ((ax=fabs(x)) < 8.0) {
		y=x*x;
		ans1=x*(72362614232.0+y*(-7895059235.0+y*(242396853.1
			+y*(-2972611.439+y*(15704.48260+y*(-30.16036606))))));
		ans2=144725228442.0+y*(2300535178.0+y*(18583304.74
			+y*(99447.43394+y*(376.9991397+y*1.0))));
		ans=ans1/ans2;
	} else {
		z=8.0/ax;
		y=z*z;
		xx=ax-2.356194491;
		ans1=1.0+y*(0.183105e-2+y*(-0.3516396496e-4
			+y*(0.2457520174e-5+y*(-0.240337019e-6))));
		ans2=0.04687499995+y*(-0.2002690873e-3
			+y*(0.8449199096e-5+y*(-0.88228987e-6
			+y*0.105787412e-6)));
		ans=sqrt(0.636619772/ax)*(cos(xx)*ans1-z*sin(xx)*ans2);
		if (x < 0.0) ans = -ans;
	}
	return ans;
}


#include <limits>



void NR_single_precision::bessjy(const NR_single_precision::DP x, const NR_single_precision::DP xnu, NR_single_precision::DP &rj, NR_single_precision::DP &ry, NR_single_precision::DP &rjp, NR_single_precision::DP &ryp)
{
	const int MAXIT=10000;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN/EPS;
	const NR_single_precision::DP XMIN=2.0, PI=3.141592653589793;
	NR_single_precision::DP a,b,br,bi,c,cr,ci,d,del,del1,den,di,dlr,dli,dr,e,f,fact,fact2,
		fact3,ff,gam,gam1,gam2,gammi,gampl,h,p,pimu,pimu2,q,r,rjl,
		rjl1,rjmu,rjp1,rjpl,rjtemp,ry1,rymu,rymup,rytemp,sum,sum1,
		temp,w,x2,xi,xi2,xmu,xmu2;
	int i,isign,l,nl;

	if (x <= 0.0 || xnu < 0.0)
		NR::nrerror("bad arguments in bessjy");
	nl=(x < XMIN ? int(xnu+0.5) : MAX(0,int(xnu-x+1.5)));
	xmu=xnu-nl;
	xmu2=xmu*xmu;
	xi=1.0/x;
	xi2=2.0*xi;
	w=xi2/PI;
	isign=1;
	h=xnu*xi;
	if (h < FPMIN) h=FPMIN;
	b=xi2*xnu;
	d=0.0;
	c=h;
	for (i=0;i<MAXIT;i++) {
		b += xi2;
		d=b-d;
		if (fabs(d) < FPMIN) d=FPMIN;
		c=b-1.0/c;
		if (fabs(c) < FPMIN) c=FPMIN;
		d=1.0/d;
		del=c*d;
		h=del*h;
		if (d < 0.0) isign = -isign;
		if (fabs(del-1.0) <= EPS) break;
	}
	if (i >= MAXIT)
		NR::nrerror("x too large in bessjy; try asymptotic expansion");
	rjl=isign*FPMIN;
	rjpl=h*rjl;
	rjl1=rjl;
	rjp1=rjpl;
	fact=xnu*xi;
	for (l=nl-1;l>=0;l--) {
		rjtemp=fact*rjl+rjpl;
		fact -= xi;
		rjpl=fact*rjtemp-rjl;
		rjl=rjtemp;
	}
	if (rjl == 0.0) rjl=EPS;
	f=rjpl/rjl;
	if (x < XMIN) {
		x2=0.5*x;
		pimu=PI*xmu;
		fact = (fabs(pimu) < EPS ? 1.0 : pimu/sin(pimu));
		d = -log(x2);
		e=xmu*d;
		fact2 = (fabs(e) < EPS ? 1.0 : sinh(e)/e);
		beschb(xmu,gam1,gam2,gampl,gammi);
		ff=2.0/PI*fact*(gam1*cosh(e)+gam2*fact2*d);
		e=exp(e);
		p=e/(gampl*PI);
		q=1.0/(e*PI*gammi);
		pimu2=0.5*pimu;
		fact3 = (fabs(pimu2) < EPS ? 1.0 : sin(pimu2)/pimu2);
		r=PI*pimu2*fact3*fact3;
		c=1.0;
		d = -x2*x2;
		sum=ff+r*q;
		sum1=p;
		for (i=1;i<=MAXIT;i++) {
			ff=(i*ff+p+q)/(i*i-xmu2);
			c *= (d/i);
			p /= (i-xmu);
			q /= (i+xmu);
			del=c*(ff+r*q);
			sum += del;
			del1=c*p-i*del;
			sum1 += del1;
			if (fabs(del) < (1.0+fabs(sum))*EPS) break;
		}
		if (i > MAXIT)
			NR::nrerror("bessy series failed to converge");
		rymu = -sum;
		ry1 = -sum1*xi2;
		rymup=xmu*xi*rymu-ry1;
		rjmu=w/(rymup-f*rymu);
	} else {
		a=0.25-xmu2;
		p = -0.5*xi;
		q=1.0;
		br=2.0*x;
		bi=2.0;
		fact=a*xi/(p*p+q*q);
		cr=br+q*fact;
		ci=bi+p*fact;
		den=br*br+bi*bi;
		dr=br/den;
		di = -bi/den;
		dlr=cr*dr-ci*di;
		dli=cr*di+ci*dr;
		temp=p*dlr-q*dli;
		q=p*dli+q*dlr;
		p=temp;
		for (i=1;i<MAXIT;i++) {
			a += 2*i;
			bi += 2.0;
			dr=a*dr+br;
			di=a*di+bi;
			if (fabs(dr)+fabs(di) < FPMIN) dr=FPMIN;
			fact=a/(cr*cr+ci*ci);
			cr=br+cr*fact;
			ci=bi-ci*fact;
			if (fabs(cr)+fabs(ci) < FPMIN) cr=FPMIN;
			den=dr*dr+di*di;
			dr /= den;
			di /= -den;
			dlr=cr*dr-ci*di;
			dli=cr*di+ci*dr;
			temp=p*dlr-q*dli;
			q=p*dli+q*dlr;
			p=temp;
			if (fabs(dlr-1.0)+fabs(dli) <= EPS) break;
		}
		if (i >= MAXIT) NR::nrerror("cf2 failed in bessjy");
		gam=(p-f)/q;
		rjmu=sqrt(w/((p-f)*gam+q));
		rjmu=SIGN(rjmu,rjl);
		rymu=rjmu*gam;
		rymup=rymu*(p+q/gam);
		ry1=xmu*xi*rymu-rymup;
	}
	fact=rjmu/rjl;
	rj=rjl1*fact;
	rjp=rjp1*fact;
	for (i=1;i<=nl;i++) {
		rytemp=(xmu+i)*xi2*ry1-rymu;
		rymu=ry1;
		ry1=rytemp;
	}
	ry=rymu;
	ryp=xnu*xi*rymu-ry1;
}



NR_single_precision::DP NR_single_precision::bessk(const int n, const NR_single_precision::DP x)
{
	int j;
	NR_single_precision::DP bk,bkm,bkp,tox;

	if (n < 2) NR::nrerror("Index n less than 2 in bessk");
	tox=2.0/x;
	bkm=bessk0(x);
	bk=bessk1(x);
	for (j=1;j<n;j++) {
		bkp=bkm+j*tox*bk;
		bkm=bk;
		bk=bkp;
	}
	return bk;
}





NR_single_precision::DP NR_single_precision::bessk0(const NR_single_precision::DP x)
{
	NR_single_precision::DP y,ans;

	if (x <= 2.0) {
		y=x*x/4.0;
		ans=(-log(x/2.0)*bessi0(x))+(-0.57721566+y*(0.42278420
			+y*(0.23069756+y*(0.3488590e-1+y*(0.262698e-2
			+y*(0.10750e-3+y*0.74e-5))))));
	} else {
		y=2.0/x;
		ans=(exp(-x)/sqrt(x))*(1.25331414+y*(-0.7832358e-1
			+y*(0.2189568e-1+y*(-0.1062446e-1+y*(0.587872e-2
			+y*(-0.251540e-2+y*0.53208e-3))))));
	}
	return ans;
}





NR_single_precision::DP NR_single_precision::bessk1(const NR_single_precision::DP x)
{
	NR_single_precision::DP y,ans;

	if (x <= 2.0) {
		y=x*x/4.0;
		ans=(log(x/2.0)*bessi1(x))+(1.0/x)*(1.0+y*(0.15443144
			+y*(-0.67278579+y*(-0.18156897+y*(-0.1919402e-1
			+y*(-0.110404e-2+y*(-0.4686e-4)))))));
	} else {
		y=2.0/x;
		ans=(exp(-x)/sqrt(x))*(1.25331414+y*(0.23498619
			+y*(-0.3655620e-1+y*(0.1504268e-1+y*(-0.780353e-2
			+y*(0.325614e-2+y*(-0.68245e-3)))))));
	}
	return ans;
}



NR_single_precision::DP NR_single_precision::bessy(const int n, const NR_single_precision::DP x)
{
	int j;
	NR_single_precision::DP by,bym,byp,tox;

	if (n < 2) NR::nrerror("Index n less than 2 in bessy");
	tox=2.0/x;
	by=bessy1(x);
	bym=bessy0(x);
	for (j=1;j<n;j++) {
		byp=j*tox*by-bym;
		bym=by;
		by=byp;
	}
	return by;
}





NR_single_precision::DP NR_single_precision::bessy0(const NR_single_precision::DP x)
{
	NR_single_precision::DP z,xx,y,ans,ans1,ans2;

	if (x < 8.0) {
		y=x*x;
		ans1 = -2957821389.0+y*(7062834065.0+y*(-512359803.6
			+y*(10879881.29+y*(-86327.92757+y*228.4622733))));
		ans2=40076544269.0+y*(745249964.8+y*(7189466.438
			+y*(47447.26470+y*(226.1030244+y*1.0))));
		ans=(ans1/ans2)+0.636619772*bessj0(x)*log(x);
	} else {
		z=8.0/x;
		y=z*z;
		xx=x-0.785398164;
		ans1=1.0+y*(-0.1098628627e-2+y*(0.2734510407e-4
			+y*(-0.2073370639e-5+y*0.2093887211e-6)));
		ans2 = -0.1562499995e-1+y*(0.1430488765e-3
			+y*(-0.6911147651e-5+y*(0.7621095161e-6
			+y*(-0.934945152e-7))));
		ans=sqrt(0.636619772/x)*(sin(xx)*ans1+z*cos(xx)*ans2);
	}
	return ans;
}





NR_single_precision::DP NR_single_precision::bessy1(const NR_single_precision::DP x)
{
	NR_single_precision::DP z,xx,y,ans,ans1,ans2;

	if (x < 8.0) {
		y=x*x;
		ans1=x*(-0.4900604943e13+y*(0.1275274390e13
			+y*(-0.5153438139e11+y*(0.7349264551e9
			+y*(-0.4237922726e7+y*0.8511937935e4)))));
		ans2=0.2499580570e14+y*(0.4244419664e12
			+y*(0.3733650367e10+y*(0.2245904002e8
			+y*(0.1020426050e6+y*(0.3549632885e3+y)))));
		ans=(ans1/ans2)+0.636619772*(bessj1(x)*log(x)-1.0/x);
	} else {
		z=8.0/x;
		y=z*z;
		xx=x-2.356194491;
		ans1=1.0+y*(0.183105e-2+y*(-0.3516396496e-4
			+y*(0.2457520174e-5+y*(-0.240337019e-6))));
		ans2=0.04687499995+y*(-0.2002690873e-3
			+y*(0.8449199096e-5+y*(-0.88228987e-6
			+y*0.105787412e-6)));
		ans=sqrt(0.636619772/x)*(sin(xx)*ans1+z*cos(xx)*ans2);
	}
	return ans;
}





NR_single_precision::DP NR_single_precision::beta(const NR_single_precision::DP z, const NR_single_precision::DP w)
{
	return exp(gammln(z)+gammln(w)-gammln(z+w));
}


#include <limits>



NR_single_precision::DP NR_single_precision::betacf(const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP x)
{
	const int MAXIT=100;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN/EPS;
	int m,m2;
	NR_single_precision::DP aa,c,d,del,h,qab,qam,qap;

	qab=a+b;
	qap=a+1.0;
	qam=a-1.0;
	c=1.0;
	d=1.0-qab*x/qap;
	if (fabs(d) < FPMIN) d=FPMIN;
	d=1.0/d;
	h=d;
	for (m=1;m<=MAXIT;m++) {
		m2=2*m;
		aa=m*(b-m)*x/((qam+m2)*(a+m2));
		d=1.0+aa*d;
		if (fabs(d) < FPMIN) d=FPMIN;
		c=1.0+aa/c;
		if (fabs(c) < FPMIN) c=FPMIN;
		d=1.0/d;
		h *= d*c;
		aa = -(a+m)*(qab+m)*x/((a+m2)*(qap+m2));
		d=1.0+aa*d;
		if (fabs(d) < FPMIN) d=FPMIN;
		c=1.0+aa/c;
		if (fabs(c) < FPMIN) c=FPMIN;
		d=1.0/d;
		del=d*c;
		h *= del;
		if (fabs(del-1.0) <= EPS) break;
	}
	if (m > MAXIT) NR::nrerror("a or b too big, or MAXIT too small in betacf");
	return h;
}





NR_single_precision::DP NR_single_precision::betai(const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP x)
{
	NR_single_precision::DP bt;

	if (x < 0.0 || x > 1.0) NR::nrerror("Bad x in routine betai");
	if (x == 0.0 || x == 1.0) bt=0.0;
	else
		bt=exp(gammln(a+b)-gammln(a)-gammln(b)+a*log(x)+b*log(1.0-x));
	if (x < (a+1.0)/(a+b+2.0))
		return bt*betacf(a,b,x)/a;
	else
		return 1.0-bt*betacf(b,a,1.0-x)/b;
}





NR_single_precision::DP NR_single_precision::bico(const int n, const int k)
{
	return floor(0.5+exp(factln(n)-factln(k)-factln(n-k)));
}



void NR_single_precision::bksub(const int ne, const int nb, const int jf, const int k1,
	const int k2, Mat3D_IO_DP &c)
{
	int nbf,im,kp,k,j,i;
	NR_single_precision::DP xx;

	nbf=ne-nb;
	im=1;
	for (k=k2-1;k>=k1;k--) {
		if (k == k1) im=nbf+1;
		kp=k+1;
		for (j=0;j<nbf;j++) {
			xx=c(j,jf,kp);
			for (i=im-1;i<ne;i++)
				c(i,jf,k) -= c(i,j,k)*xx;
		}
	}
	for (k=k1;k<k2;k++) {
		kp=k+1;
		for (i=0;i<nb;i++) c(i,0,k)=c(i+nbf,jf,k);
		for (i=0;i<nbf;i++) c(i+nb,0,k)=c(i,jf,kp);
	}
}





NR_single_precision::DP NR_single_precision::bnldev(const NR_single_precision::DP pp, const int n, int &idum)
{
	const NR_single_precision::DP PI=3.141592653589793238;
	int j;
	static int nold=(-1);
	NR_single_precision::DP am,em,g,angle,p,bnl,sq,t,y;
	static NR_single_precision::DP pold=(-1.0),pc,plog,pclog,en,oldg;

	p=(pp <= 0.5 ? pp : 1.0-pp);
	am=n*p;
	if (n < 25) {
		bnl=0.0;
		for (j=0;j<n;j++)
			if (ran1(idum) < p) ++bnl;
	} else if (am < 1.0) {
		g=exp(-am);
		t=1.0;
		for (j=0;j<=n;j++) {
			t *= ran1(idum);
			if (t < g) break;
		}
		bnl=(j <= n ? j : n);
	} else {
		if (n != nold) {
			en=n;
			oldg=gammln(en+1.0);
			nold=n;
		} if (p != pold) {
			pc=1.0-p;
			plog=log(p);
			pclog=log(pc);
			pold=p;
		}
		sq=sqrt(2.0*am*pc);
		do {
			do {
				angle=PI*ran1(idum);
				y=tan(angle);
				em=sq*y+am;
			} while (em < 0.0 || em >= (en+1.0));
			em=floor(em);
			t=1.2*sq*(1.0+y*y)*exp(oldg-gammln(em+1.0)
				-gammln(en-em+1.0)+em*plog+(en-em)*pclog);
		} while (ran1(idum) > t);
		bnl=em;
	}
	if (p != pp) bnl=n-bnl;
	return bnl;
}


#include <limits>



NR_single_precision::DP NR_single_precision::brent(const NR_single_precision::DP ax, const NR_single_precision::DP bx, const NR_single_precision::DP cx, NR_single_precision::DP f(const NR_single_precision::DP),
	const NR_single_precision::DP tol, NR_single_precision::DP &xmin)
{
	const int ITMAX=100;
	const NR_single_precision::DP CGOLD=0.3819660;
	const NR_single_precision::DP ZEPS=DBL_EPSILON*1.0e-3;
	int iter;
	NR_single_precision::DP a,b,d=0.0,etemp,fu,fv,fw,fx;
	NR_single_precision::DP p,q,r,tol1,tol2,u,v,w,x,xm;
	NR_single_precision::DP e=0.0;

	a=(ax < cx ? ax : cx);
	b=(ax > cx ? ax : cx);
	x=w=v=bx;
	fw=fv=fx=f(x);
	for (iter=0;iter<ITMAX;iter++) {
		xm=0.5*(a+b);
		tol2=2.0*(tol1=tol*fabs(x)+ZEPS);
		if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
			xmin=x;
			return fx;
		}
		if (fabs(e) > tol1) {
			r=(x-w)*(fx-fv);
			q=(x-v)*(fx-fw);
			p=(x-v)*q-(x-w)*r;
			q=2.0*(q-r);
			if (q > 0.0) p = -p;
			q=fabs(q);
			etemp=e;
			e=d;
			if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
				d=CGOLD*(e=(x >= xm ? a-x : b-x));
			else {
				d=p/q;
				u=x+d;
				if (u-a < tol2 || b-u < tol2)
					d=SIGN(tol1,xm-x);
			}
		} else {
			d=CGOLD*(e=(x >= xm ? a-x : b-x));
		}
		u=(fabs(d) >= tol1 ? x+d : x+SIGN(tol1,d));
		fu=f(u);
		if (fu <= fx) {
			if (u >= x) a=x; else b=x;
			shft3(v,w,x,u);
			shft3(fv,fw,fx,fu);
		} else {
			if (u < x) a=u; else b=u;
			if (fu <= fw || w == x) {
				v=w;
				w=u;
				fv=fw;
				fw=fu;
			} else if (fu <= fv || v == x || v == w) {
				v=u;
				fv=fu;
			}
		}
	}
	NR::nrerror("Too many iterations in brent");
	xmin=x;
	return fx;
}


#include <limits>



NR_single_precision::Vec_DP *fvec_p;
void (*nrfuncv)(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f);

void NR_single_precision::broydn(NR_single_precision::Vec_IO_DP &x, bool &check, void vecfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const int MAXITS=200;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP TOLF=1.0e-8, TOLX=EPS, STPMX=100.0, TOLMIN=1.0e-12;
	bool restrt,sing,skip;
	int i,its,j,k;
	NR_single_precision::DP den,f,fold,stpmax,sum,temp,test;

	int n=x.size();
	NR_single_precision::Mat_DP qt(n,n),r(n,n);
	NR_single_precision::Vec_DP c(n),d(n),fvcold(n),g(n),p(n),s(n),t(n),w(n),xold(n);
	fvec_p=new NR_single_precision::Vec_DP(n);
	nrfuncv=vecfunc;
	NR_single_precision::Vec_DP &fvec=*fvec_p;
	f=fmin(x);
	test=0.0;
	for (i=0;i<n;i++)
		if (fabs(fvec(i)) > test) test=fabs(fvec(i));
	if (test < 0.01*TOLF) {
		check=false;
		delete fvec_p;
		return;
	}
	for (sum=0.0,i=0;i<n;i++) sum += SQR(x(i));
	stpmax=STPMX*MAX(sqrt(sum),NR_single_precision::DP(n));
	restrt=true;
	for (its=1;its<=MAXITS;its++) {
		if (restrt) {
			fdjac(x,fvec,r,vecfunc);
			qrdcmp(r,c,d,sing);
			if (sing) NR::nrerror("singular Jacobian in broydn");
			for (i=0;i<n;i++) {
				for (j=0;j<n;j++) qt(i,j)=0.0;
				qt(i,i)=1.0;
			}
			for (k=0;k<n-1;k++) {
				if (c(k) != 0.0) {
					for (j=0;j<n;j++) {
						sum=0.0;
						for (i=k;i<n;i++)
							sum += r(i,k)*qt(i,j);
						sum /= c(k);
						for (i=k;i<n;i++)
							qt(i,j) -= sum*r(i,k);
					}
				}
			}
			for (i=0;i<n;i++) {
				r(i,i)=d(i);
				for (j=0;j<i;j++) r(i,j)=0.0;
			}
		} else {
			for (i=0;i<n;i++) s(i)=x(i)-xold(i);
			for (i=0;i<n;i++) {
				for (sum=0.0,j=i;j<n;j++) sum += r(i,j)*s(j);
				t(i)=sum;
			}
			skip=true;
			for (i=0;i<n;i++) {
				for (sum=0.0,j=0;j<n;j++) sum += qt(j,i)*t(j);
				w(i)=fvec(i)-fvcold(i)-sum;
				if (fabs(w(i)) >= EPS*(fabs(fvec(i))+fabs(fvcold(i)))) skip=false;
				else w(i)=0.0;
			}
			if (!skip) {
				for (i=0;i<n;i++) {
					for (sum=0.0,j=0;j<n;j++) sum += qt(i,j)*w(j);
					t(i)=sum;
				}
				for (den=0.0,i=0;i<n;i++) den += SQR(s(i));
				for (i=0;i<n;i++) s(i) /= den;
				qrupdt(r,qt,t,s);
				for (i=0;i<n;i++) {
					if (r(i,i) == 0.0) NR::nrerror("r singular in broydn");
					d(i)=r(i,i);
				}
			}
		}
		for (i=0;i<n;i++) {
			for (sum=0.0,j=0;j<n;j++) sum += qt(i,j)*fvec(j);
			p(i) = -sum;
		}
		for (i=n-1;i>=0;i--) {
			for (sum=0.0,j=0;j<=i;j++) sum -= r(j,i)*p(j);
			g(i)=sum;
		}
		for (i=0;i<n;i++) {
			xold(i)=x(i);
			fvcold(i)=fvec(i);
		}
		fold=f;
		rsolv(r,d,p);
		lnsrch(xold,fold,g,p,x,f,stpmax,check,fmin);
		test=0.0;
		for (i=0;i<n;i++)
			if (fabs(fvec(i)) > test) test=fabs(fvec(i));
		if (test < TOLF) {
			check=false;
			delete fvec_p;
			return;
		}
		if (check) {
			if (restrt) {
				delete fvec_p;
				return;
			} else {
				test=0.0;
				den=MAX(f,0.5*n);
				for (i=0;i<n;i++) {
					temp=fabs(g(i))*MAX(fabs(x(i)),1.0)/den;
					if (temp > test) test=temp;
				}
				if (test < TOLMIN) {
					delete fvec_p;
					return;
				}
				else restrt=true;
			}
		} else {
			restrt=false;
			test=0.0;
			for (i=0;i<n;i++) {
				temp=(fabs(x(i)-xold(i)))/MAX(fabs(x(i)),1.0);
				if (temp > test) test=temp;
			}
			if (test < TOLX) {
				delete fvec_p;
				return;
			}
		}
	}
	NR::nrerror("MAXITS exceeded in broydn");
	return;
}





NR_single_precision::Vec_DP *x_p;
NR_single_precision::Mat_DP *d_p;

void NR_single_precision::bsstep(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &xx, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const int KMAXX=8, IMAXX=(KMAXX+1);
	const NR_single_precision::DP SAFE1=0.25, SAFE2=0.7, REDMAX=1.0e-5, REDMIN=0.7;
	const NR_single_precision::DP TINY=1.0e-30, SCALMX=0.1;
	static const int nseq_d[IMAXX]={2,4,6,8,10,12,14,16,18};
	static int first=1,kmax,kopt;
	static NR_single_precision::DP epsold = -1.0,xnew;
	static NR_single_precision::Vec_DP a(IMAXX);
	static NR_single_precision::Mat_DP alf(KMAXX,KMAXX);
	bool exitflag=false;
	int i,iq,k,kk,km,reduct;
	NR_single_precision::DP eps1,errmax,fact,h,red,scale,work,wrkmin,xest;
	NR_single_precision::Vec_INT nseq(nseq_d,IMAXX);
	NR_single_precision::Vec_DP err(KMAXX);

	int nv=y.size();
	NR_single_precision::Vec_DP yerr(nv),ysav(nv),yseq(nv);
	x_p=new NR_single_precision::Vec_DP(KMAXX);
	d_p=new NR_single_precision::Mat_DP(nv,KMAXX);
	if (eps != epsold) {
		hnext = xnew = -1.0e29;
		eps1=SAFE1*eps;
		a(0)=nseq(0)+1;
		for (k=0;k<KMAXX;k++) a(k+1)=a(k)+nseq(k+1);
		for (iq=1;iq<KMAXX;iq++) {
			for (k=0;k<iq;k++)
				alf(k,iq)=pow(eps1,(a(k+1)-a(iq+1))/
					((a(iq+1)-a(0)+1.0)*(2*k+3)));
		}
		epsold=eps;
		for (kopt=1;kopt<KMAXX-1;kopt++)
			if (a(kopt+1) > a(kopt)*alf(kopt-1,kopt)) break;
		kmax=kopt;
	}
	h=htry;
	for (i=0;i<nv;i++) ysav(i)=y(i);
	if (xx != xnew || h != hnext) {
		first=1;
		kopt=kmax;
	}
	reduct=0;
	for (;;) {
		for (k=0;k<=kmax;k++) {
			xnew=xx+h;
			if (xnew == xx) NR::nrerror("step size underflow in bsstep");
			mmid(ysav,dydx,xx,h,nseq(k),yseq,derivs);
			xest=SQR(h/nseq(k));
			pzextr(k,xest,yseq,y,yerr);
			if (k != 0) {
				errmax=TINY;
				for (i=0;i<nv;i++) errmax=MAX(errmax,fabs(yerr(i)/yscal(i)));
				errmax /= eps;
				km=k-1;
				err(km)=pow(errmax/SAFE1,1.0/(2*km+3));
			}
			if (k != 0 && (k >= kopt-1 || first)) {
				if (errmax < 1.0) {
					exitflag=true;
					break;
				}
				if (k == kmax || k == kopt+1) {
					red=SAFE2/err(km);
					break;
				}
				else if (k == kopt && alf(kopt-1,kopt) < err(km)) {
					red=1.0/err(km);
					break;
				}
				else if (kopt == kmax && alf(km,kmax-1) < err(km)) {
					red=alf(km,kmax-1)*SAFE2/err(km);
					break;
				}
				else if (alf(km,kopt) < err(km)) {
					red=alf(km,kopt-1)/err(km);
					break;
				}
			}
		}
		if (exitflag) break;
		red=MIN(red,REDMIN);
		red=MAX(red,REDMAX);
		h *= red;
		reduct=1;
	}
	xx=xnew;
	hdid=h;
	first=0;
	wrkmin=1.0e35;
	for (kk=0;kk<=km;kk++) {
		fact=MAX(err(kk),SCALMX);
		work=fact*a(kk+1);
		if (work < wrkmin) {
			scale=fact;
			wrkmin=work;
			kopt=kk+1;
		}
	}
	hnext=h/scale;
	if (kopt >= k && kopt != kmax && !reduct) {
		fact=MAX(scale/alf(kopt-1,kopt),SCALMX);
		if (a(kopt+1)*fact <= wrkmin) {
			hnext=h/fact;
			kopt++;
		}
	}
	delete d_p;
	delete x_p;
}





void NR_single_precision::caldat(const int julian, int &mm, int &id, int &iyyy)
{
	const int IGREG=2299161;
	int ja,jalpha,jb,jc,jd,je;

	if (julian >= IGREG) {
		jalpha=int((NR_single_precision::DP(julian-1867216)-0.25)/36524.25);
		ja=julian+1+jalpha-int(0.25*jalpha);
	} else if (julian < 0) {
		ja=julian+36525*(1-julian/36525);
	} else
		ja=julian;
	jb=ja+1524;
	jc=int(6680.0+(NR_single_precision::DP(jb-2439870)-122.1)/365.25);
	jd=int(365*jc+(0.25*jc));
	je=int((jb-jd)/30.6001);
	id=jb-jd-int(30.6001*je);
	mm=je-1;
	if (mm > 12) mm -= 12;
	iyyy=jc-4715;
	if (mm > 2) --iyyy;
	if (iyyy <= 0) --iyyy;
	if (julian < 0) iyyy -= 100*(1-julian/36525);
}



void NR_single_precision::chder(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_O_DP &cder, const int n)
{
	int j;
	NR_single_precision::DP con;

	cder(n-1)=0.0;
	cder(n-2)=2*(n-1)*c(n-1);
	for (j=n-2;j>0;j--)
		cder(j-1)=cder(j+1)+2*j*c(j);
	con=2.0/(b-a);
	for (j=0;j<n;j++)
		cder(j) *= con;
}



NR_single_precision::DP NR_single_precision::chebev(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &c, const int m, const NR_single_precision::DP x)
{
	NR_single_precision::DP d=0.0,dd=0.0,sv,y,y2;
	int j;

	if ((x-a)*(x-b) > 0.0)
		NR::nrerror("x not in range in routine chebev");
	y2=2.0*(y=(2.0*x-a-b)/(b-a));
	for (j=m-1;j>0;j--) {
		sv=d;
		d=y2*d-dd+c(j);
		dd=sv;
	}
	return y*d-dd+0.5*c(0);
}





void NR_single_precision::chebft(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_O_DP &c, NR_single_precision::DP func(const NR_single_precision::DP))
{
	const NR_single_precision::DP PI=3.141592653589793;
	int k,j;
	NR_single_precision::DP fac,bpa,bma,y,sum;

	int n=c.size();
	NR_single_precision::Vec_DP f(n);
	bma=0.5*(b-a);
	bpa=0.5*(b+a);
	for (k=0;k<n;k++) {
		y=cos(PI*(k+0.5)/n);
		f(k)=func(y*bma+bpa);
	}
	fac=2.0/n;
	for (j=0;j<n;j++) {
		sum=0.0;
		for (k=0;k<n;k++)
			sum += f(k)*cos(PI*j*(k+0.5)/n);
		c(j)=fac*sum;
	}
}



void NR_single_precision::chebpc(NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_O_DP &d)
{
	int k,j;
	NR_single_precision::DP sv;

	int n=c.size();
	NR_single_precision::Vec_DP dd(n);
	for (j=0;j<n;j++) d(j)=dd(j)=0.0;
	d(0)=c(n-1);
	for (j=n-2;j>0;j--) {
		for (k=n-j;k>0;k--) {
			sv=d(k);
			d(k)=2.0*d(k-1)-dd(k);
			dd(k)=sv;
		}
		sv=d(0);
		d(0) = -dd(0)+c(j);
		dd(0)=sv;
	}
	for (j=n-1;j>0;j--)
		d(j)=d(j-1)-dd(j);
	d(0) = -dd(0)+0.5*c(0);
}



void NR_single_precision::chint(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_O_DP &cint, const int n)
{
	int j;
	NR_single_precision::DP sum=0.0,fac=1.0,con;

	con=0.25*(b-a);
	for (j=1;j<n-1;j++) {
		cint(j)=con*(c(j-1)-c(j+1))/j;
		sum += fac*cint(j);
		fac = -fac;
	}
	cint(n-1)=con*c(n-2)/(n-1);
	sum += fac*cint(n-1);
	cint(0)=2.0*sum;
}





NR_single_precision::DP NR_single_precision::chixy(const NR_single_precision::DP bang)
{
	const NR_single_precision::DP BIG=1.0e30;
	int j;
	NR_single_precision::DP ans,avex=0.0,avey=0.0,sumw=0.0,b;

	NR_single_precision::Vec_DP &xx=*xx_p, &yy=*yy_p;
	NR_single_precision::Vec_DP &sx=*sx_p, &sy=*sy_p, &ww=*ww_p;
	int nn=xx.size();
	b=tan(bang);
	for (j=0;j<nn;j++) {
		ww(j) = SQR(b*sx(j))+SQR(sy(j));
		sumw += (ww(j)=(ww(j) < 1.0/BIG ? BIG : 1.0/ww(j)));
		avex += ww(j)*xx(j);
		avey += ww(j)*yy(j);
	}
	avex /= sumw;
	avey /= sumw;
	aa=avey-b*avex;
	for (ans = -offs,j=0;j<nn;j++)
		ans += ww(j)*SQR(yy(j)-aa-b*xx(j));
	return ans;
}




void NR_single_precision::choldc(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &p)
{
	int i,j,k;
	NR_single_precision::DP sum;

	int n=a.rows();
	for (i=0;i<n;i++) {
		for (j=i;j<n;j++) {
			for (sum=a(i,j),k=i-1;k>=0;k--) sum -= a(i,k)*a(j,k);
			if (i == j) {
				if (sum <= 0.0)
					NR::nrerror("choldc failed");
				p(i)=sqrt(sum);
			} else a(j,i)=sum/p(i);
		}
	}
}


void NR_single_precision::cholsl(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_DP &p, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_O_DP &x)
{
	int i,k;
	NR_single_precision::DP sum;

	int n=a.rows();
	for (i=0;i<n;i++) {
		for (sum=b(i),k=i-1;k>=0;k--) sum -= a(i,k)*x(k);
		x(i)=sum/p(i);
	}
	for (i=n-1;i>=0;i--) {
		for (sum=x(i),k=i+1;k<n;k++) sum -= a(k,i)*x(k);
		x(i)=sum/p(i);
	}
}


void NR_single_precision::chsone(NR_single_precision::Vec_I_DP &bins, NR_single_precision::Vec_I_DP &ebins, const int knstrn, NR_single_precision::DP &df,
	NR_single_precision::DP &chsq, NR_single_precision::DP &prob)
{
	int j;
	NR_single_precision::DP temp;

	int nbins=bins.size();
	df=nbins-knstrn;
	chsq=0.0;
	for (j=0;j<nbins;j++) {
		if (ebins(j) <= 0.0) NR::nrerror("Bad expected number in chsone");
		temp=bins(j)-ebins(j);
		chsq += temp*temp/ebins(j);
	}
	prob=gammq(0.5*df,0.5*chsq);
}


void NR_single_precision::chstwo(NR_single_precision::Vec_I_DP &bins1, NR_single_precision::Vec_I_DP &bins2, const int knstrn, NR_single_precision::DP &df,
	NR_single_precision::DP &chsq, NR_single_precision::DP &prob)
{
	int j;
	NR_single_precision::DP temp;

	int nbins=bins1.size();
	df=nbins-knstrn;
	chsq=0.0;
	for (j=0;j<nbins;j++)
		if (bins1(j) == 0.0 && bins2(j) == 0.0)
			--df;
		else {
			temp=bins1(j)-bins2(j);
			chsq += temp*temp/(bins1(j)+bins2(j));
		}
	prob=gammq(0.5*df,0.5*chsq);
}

#include <complex>
#include <limits>



void NR_single_precision::cisi(const NR_single_precision::DP x, complex<NR_single_precision::DP> &cs)
{
	const int MAXIT=100;
	const NR_single_precision::DP EULER=0.577215664901533, PIBY2=1.570796326794897, TMIN=2.0;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN*4.0;
	const NR_single_precision::DP BIG=NR_single_precision::MAXIMUM_CONSTANTS*EPS;
	int i,k;
	bool odd;
	NR_single_precision::DP a,err,fact,sign,sum,sumc,sums,t,term;
	complex<NR_single_precision::DP> h,b,c,d,del;

	t=fabs(x);
	if (t == 0.0) {
		cs= -BIG;
		return;
	}
	if (t > TMIN) {
		b=complex<NR_single_precision::DP>(1.0,t);
		c=complex<NR_single_precision::DP>(BIG,0.0);
		d=h=1.0/b;
		for (i=1;i<MAXIT;i++) {
			a= -i*i;
			b += 2.0;
			d=1.0/(a*d+b);
			c=b+a/c;
			del=c*d;
			h *= del;
			if (fabs(real(del)-1.0)+fabs(imag(del)) <= EPS) break;
		}
		if (i >= MAXIT) NR::nrerror("cf failed in cisi");
		h=complex<NR_single_precision::DP>(cos(t),-sin(t))*h;
		cs= -conj(h)+complex<NR_single_precision::DP>(0.0,PIBY2);
	} else {
		if (t < sqrt(FPMIN)) {
			sumc=0.0;
			sums=t;
		} else {
			sum=sums=sumc=0.0;
			sign=fact=1.0;
			odd=true;
			for (k=1;k<=MAXIT;k++) {
				fact *= t/k;
				term=fact/k;
				sum += sign*term;
				err=term/fabs(sum);
				if (odd) {
					sign = -sign;
					sums=sum;
					sum=sumc;
				} else {
					sumc=sum;
					sum=sums;
				}
				if (err < EPS) break;
				odd=!odd;
			}
			if (k > MAXIT) NR::nrerror("maxits exceeded in cisi");
		}
		cs=complex<NR_single_precision::DP>(sumc+log(t)+EULER,sums);
	}
	if (x < 0.0) cs = conj(cs);
}




void NR_single_precision::cntab1(NR_single_precision::Mat_I_INT &nn, NR_single_precision::DP &chisq, NR_single_precision::DP &df, NR_single_precision::DP &prob, NR_single_precision::DP &cramrv, NR_single_precision::DP &ccc)
{
	const NR_single_precision::DP TINY=1.0e-30;
	int i,j,nnj,nni,minij;
	NR_single_precision::DP sum=0.0,expctd,temp;

	int ni=nn.rows();
	int nj=nn.cols();
	NR_single_precision::Vec_DP sumi(ni),sumj(nj);
	nni=ni;
	nnj=nj;
	for (i=0;i<ni;i++) {
		sumi(i)=0.0;
		for (j=0;j<nj;j++) {
			sumi(i) += nn(i,j);
			sum += nn(i,j);
		}
		if (sumi(i) == 0.0) --nni;
	}
	for (j=0;j<nj;j++) {
		sumj(j)=0.0;
		for (i=0;i<ni;i++) sumj(j) += nn(i,j);
		if (sumj(j) == 0.0) --nnj;
	}
	df=nni*nnj-nni-nnj+1;
	chisq=0.0;
	for (i=0;i<ni;i++) {
		for (j=0;j<nj;j++) {
			expctd=sumj(j)*sumi(i)/sum;
			temp=nn(i,j)-expctd;
			chisq += temp*temp/(expctd+TINY);
		}
	}
	prob=gammq(0.5*df,0.5*chisq);
	minij = nni < nnj ? nni-1 : nnj-1;
	cramrv=sqrt(chisq/(sum*minij));
	ccc=sqrt(chisq/(chisq+sum));
}




void NR_single_precision::cntab2(NR_single_precision::Mat_I_INT &nn, NR_single_precision::DP &h, NR_single_precision::DP &hx, NR_single_precision::DP &hy, NR_single_precision::DP &hygx, NR_single_precision::DP &hxgy,
	NR_single_precision::DP &uygx, NR_single_precision::DP &uxgy, NR_single_precision::DP &uxy)
{
	const NR_single_precision::DP TINY=1.0e-30;
	int i,j;
	NR_single_precision::DP sum=0.0,p;

	int ni=nn.rows();
	int nj=nn.cols();
	NR_single_precision::Vec_DP sumi(ni),sumj(nj);
	for (i=0;i<ni;i++) {
		sumi(i)=0.0;
		for (j=0;j<nj;j++) {
			sumi(i) += nn(i,j);
			sum += nn(i,j);
		}
	}
	for (j=0;j<nj;j++) {
		sumj(j)=0.0;
		for (i=0;i<ni;i++)
			sumj(j) += nn(i,j);
	}
	hx=0.0;
	for (i=0;i<ni;i++)
		if (sumi(i) != 0.0) {
			p=sumi(i)/sum;
			hx -= p*log(p);
		}
	hy=0.0;
	for (j=0;j<nj;j++)
		if (sumj(j) != 0.0) {
			p=sumj(j)/sum;
			hy -= p*log(p);
		}
	h=0.0;
	for (i=0;i<ni;i++)
		for (j=0;j<nj;j++)
			if (nn(i,j) != 0) {
				p=nn(i,j)/sum;
				h -= p*log(p);
			}
	hygx=h-hx;
	hxgy=h-hy;
	uygx=(hy-hygx)/(hy+TINY);
	uxgy=(hx-hxgy)/(hx+TINY);
	uxy=2.0*(hx+hy-h)/(hx+hy+TINY);
}



void NR_single_precision::convlv(NR_single_precision::Vec_I_DP &data, NR_single_precision::Vec_I_DP &respns, const int isign,
	NR_single_precision::Vec_O_DP &ans)
{
	int i,no2;
	NR_single_precision::DP mag2,tmp;

	int n=data.size();
	int m=respns.size();
	NR_single_precision::Vec_DP temp(n);
	temp(0)=respns(0);
	for (i=1;i<(m+1)/2;i++) {
		temp(i)=respns(i);
		temp(n-i)=respns(m-i);
	}
	for (i=(m+1)/2;i<n-(m-1)/2;i++)
		temp(i)=0.0;
	for (i=0;i<n;i++)
		ans(i)=data(i);
	realft(ans,1);
	realft(temp,1);
	no2=n>>1;
	if (isign == 1) {
		for (i=2;i<n;i+=2) {
			tmp=ans(i);
			ans(i)=(ans(i)*temp(i)-ans(i+1)*temp(i+1))/no2;
			ans(i+1)=(ans(i+1)*temp(i)+tmp*temp(i+1))/no2;
		}
		ans(0)=ans(0)*temp(0)/no2;
		ans(1)=ans(1)*temp(1)/no2;
	} else if (isign == -1) {
		for (i=2;i<n;i+=2) {
			if ((mag2=SQR(temp(i))+SQR(temp(i+1))) == 0.0)
				NR::nrerror("Deconvolving at response zero in convlv");
			tmp=ans(i);
			ans(i)=(ans(i)*temp(i)+ans(i+1)*temp(i+1))/mag2/no2;
			ans(i+1)=(ans(i+1)*temp(i)-tmp*temp(i+1))/mag2/no2;
		}
		if (temp(0) == 0.0 || temp(1) == 0.0)
			NR::nrerror("Deconvolving at response zero in convlv");
		ans(0)=ans(0)/temp(0)/no2;
		ans(1)=ans(1)/temp(1)/no2;
	} else NR::nrerror("No meaning for isign in convlv");
	realft(ans,-1);
}



void NR_single_precision::copy(NR_single_precision::Mat_O_DP &aout, NR_single_precision::Mat_I_DP &ain)
{
	int i,j;

	int n=ain.rows();
	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			aout(j,i)=ain(j,i);

}



void NR_single_precision::correl(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::Vec_O_DP &ans)
{
	int no2,i;
	NR_single_precision::DP tmp;

	int n=data1.size();
	NR_single_precision::Vec_DP temp(n);
	for (i=0;i<n;i++) {
		ans(i)=data1(i);
		temp(i)=data2(i);
	}
	realft(ans,1);
	realft(temp,1);
	no2=n>>1;
	for (i=2;i<n;i+=2) {
		tmp=ans(i);
		ans(i)=(ans(i)*temp(i)+ans(i+1)*temp(i+1))/no2;
		ans(i+1)=(ans(i+1)*temp(i)-tmp*temp(i+1))/no2;
	}
	ans(0)=ans(0)*temp(0)/no2;
	ans(1)=ans(1)*temp(1)/no2;
	realft(ans,-1);
}





void NR_single_precision::cosft1(NR_single_precision::Vec_IO_DP &y)
{
	const NR_single_precision::DP PI=3.141592653589793238;
	int j;
	NR_single_precision::DP sum,y1,y2,theta,wi=0.0,wpi,wpr,wr=1.0,wtemp;

	int n=y.size()-1;
	NR_single_precision::Vec_DP yy(n);
	theta=PI/n;
	wtemp=sin(0.5*theta);
	wpr = -2.0*wtemp*wtemp;
	wpi=sin(theta);
	sum=0.5*(y(0)-y(n));
	yy(0)=0.5*(y(0)+y(n));
	for (j=1;j<n/2;j++) {
		wr=(wtemp=wr)*wpr-wi*wpi+wr;
		wi=wi*wpr+wtemp*wpi+wi;
		y1=0.5*(y(j)+y(n-j));
		y2=(y(j)-y(n-j));
		yy(j)=y1-wi*y2;
		yy(n-j)=y1+wi*y2;
		sum += wr*y2;
	}
	yy(n/2)=y(n/2);
	realft(yy,1);
	for (j=0;j<n;j++) y(j)=yy(j);
	y(n)=y(1);
	y(1)=sum;
	for (j=3;j<n;j+=2) {
		sum += y(j);
		y(j)=sum;
	}
}





void NR_single_precision::cosft2(NR_single_precision::Vec_IO_DP &y, const int isign)
{
	const NR_single_precision::DP PI=3.141592653589793238;
	int i;
	NR_single_precision::DP sum,sum1,y1,y2,ytemp,theta,wi=0.0,wi1,wpi,wpr,wr=1.0,wr1,wtemp;

	int n=y.size();
	theta=0.5*PI/n;
	wr1=cos(theta);
	wi1=sin(theta);
	wpr = -2.0*wi1*wi1;
	wpi=sin(2.0*theta);
	if (isign == 1) {
		for (i=0;i<n/2;i++) {
			y1=0.5*(y(i)+y(n-1-i));
			y2=wi1*(y(i)-y(n-1-i));
			y(i)=y1+y2;
			y(n-1-i)=y1-y2;
			wr1=(wtemp=wr1)*wpr-wi1*wpi+wr1;
			wi1=wi1*wpr+wtemp*wpi+wi1;
		}
		realft(y,1);
		for (i=2;i<n;i+=2) {
			wr=(wtemp=wr)*wpr-wi*wpi+wr;
			wi=wi*wpr+wtemp*wpi+wi;
			y1=y(i)*wr-y(i+1)*wi;
			y2=y(i+1)*wr+y(i)*wi;
			y(i)=y1;
			y(i+1)=y2;
		}
		sum=0.5*y(1);
		for (i=n-1;i>0;i-=2) {
			sum1=sum;
			sum += y(i);
			y(i)=sum1;
		}
	} else if (isign == -1) {
		ytemp=y(n-1);
		for (i=n-1;i>2;i-=2)
			y(i)=y(i-2)-y(i);
		y(1)=2.0*ytemp;
		for (i=2;i<n;i+=2) {
			wr=(wtemp=wr)*wpr-wi*wpi+wr;
			wi=wi*wpr+wtemp*wpi+wi;
			y1=y(i)*wr+y(i+1)*wi;
			y2=y(i+1)*wr-y(i)*wi;
			y(i)=y1;
			y(i+1)=y2;
		}
		realft(y,-1);
		for (i=0;i<n/2;i++) {
			y1=y(i)+y(n-1-i);
			y2=(0.5/wi1)*(y(i)-y(n-1-i));
			y(i)=0.5*(y1+y2);
			y(n-1-i)=0.5*(y1-y2);
			wr1=(wtemp=wr1)*wpr-wi1*wpi+wr1;
			wi1=wi1*wpr+wtemp*wpi+wi1;
		}
	}
}



void NR_single_precision::covsrt(NR_single_precision::Mat_IO_DP &covar, NR_single_precision::Vec_I_BOOL &ia, const int mfit)
{
	int i,j,k;

	int ma=ia.size();
	for (i=mfit;i<ma;i++)
		for (j=0;j<i+1;j++) covar(i,j)=covar(j,i)=0.0;
	k=mfit-1;
	for (j=ma-1;j>=0;j--) {
		if (ia(j)) {
			for (i=0;i<ma;i++) SWAP(covar(i,k),covar(i,j));
			for (i=0;i<ma;i++) SWAP(covar(k,i),covar(j,i));
			k--;
		}
	}
}



void NR_single_precision::crank(NR_single_precision::Vec_IO_DP &w, NR_single_precision::DP &s)
{
	int j=1,ji,jt;
	NR_single_precision::DP t,rank;

	int n=w.size();
	s=0.0;
	while (j < n) {
		if (w(j) != w(j-1)) {
			w(j-1)=j;
			++j;
		} else {
			for (jt=j+1;jt<=n && w(jt-1)==w(j-1);jt++);
			rank=0.5*(j+jt-1);
			for (ji=j;ji<=(jt-1);ji++)
				w(ji-1)=rank;
			t=jt-j;
			s += (t*t*t-t);
			j=jt;
		}
	}
	if (j == n) w(n-1)=n;
}



void NR_single_precision::cyclic(NR_single_precision::Vec_I_DP &a, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_I_DP &c, const NR_single_precision::DP alpha,
	const NR_single_precision::DP beta, NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &x)
{
	int i;
	NR_single_precision::DP fact,gamma;

	int n=a.size();
	if (n <= 2) NR::nrerror("n too small in cyclic");
	NR_single_precision::Vec_DP bb(n),u(n),z(n);
	gamma = -b(0);
	bb(0)=b(0)-gamma;
	bb(n-1)=b(n-1)-alpha*beta/gamma;
	for (i=1;i<n-1;i++) bb(i)=b(i);
	tridag(a,bb,c,r,x);
	u(0)=gamma;
	u(n-1)=alpha;
	for (i=1;i<n-1;i++) u(i)=0.0;
	tridag(a,bb,c,u,z);
	fact=(x(0)+beta*x(n-1)/gamma)/
		(1.0+z(0)+beta*z(n-1)/gamma);
	for (i=0;i<n;i++) x(i) -= fact*z(i);
}



void NR_single_precision::daub4(NR_single_precision::Vec_IO_DP &a, const int n, const int isign)
{
	const NR_single_precision::DP C0=0.4829629131445341,C1=0.8365163037378079,
		C2=0.2241438680420134,C3=-0.1294095225512604;
	int nh,i,j;

	if (n < 4) return;
	NR_single_precision::Vec_DP wksp(n);
	nh=n >> 1;
	if (isign >= 0) {
		for (i=0,j=0;j<n-3;j+=2,i++) {
			wksp(i)=C0*a(j)+C1*a(j+1)+C2*a(j+2)+C3*a(j+3);
			wksp(i+nh)=C3*a(j)-C2*a(j+1)+C1*a(j+2)-C0*a(j+3);
		}
		wksp(i)=C0*a(n-2)+C1*a(n-1)+C2*a(0)+C3*a(1);
		wksp(i+nh)=C3*a(n-2)-C2*a(n-1)+C1*a(0)-C0*a(1);
	} else {
		wksp(0)=C2*a(nh-1)+C1*a(n-1)+C0*a(0)+C3*a(nh);
		wksp(1)=C3*a(nh-1)-C0*a(n-1)+C1*a(0)-C2*a(nh);
		for (i=0,j=2;i<nh-1;i++) {
			wksp(j++)=C2*a(i)+C1*a(i+nh)+C0*a(i+1)+C3*a(i+nh+1);
			wksp(j++)=C3*a(i)-C0*a(i+nh)+C1*a(i+1)-C2*a(i+nh+1);
		}
	}
	for (i=0;i<n;i++) a(i)=wksp(i);
}





NR_single_precision::DP NR_single_precision::dawson(const NR_single_precision::DP x)
{
	const int NMAX=6;
	const NR_single_precision::DP H=0.4, A1=2.0/3.0, A2=0.4, A3=2.0/7.0;
	int i,n0;
	static bool init = true;
	NR_single_precision::DP d1,d2,e1,e2,sum,x2,xp,xx,ans;
	static NR_single_precision::Vec_DP c(NMAX);

	if (init) {
		init=false;
		for (i=0;i<NMAX;i++) c(i)=exp(-SQR((2.0*i+1.0)*H));
	}
	if (fabs(x) < 0.2) {
		x2=x*x;
		ans=x*(1.0-A1*x2*(1.0-A2*x2*(1.0-A3*x2)));
	} else {
		xx=fabs(x);
		n0=2*int(0.5*xx/H+0.5);
		xp=xx-n0*H;
		e1=exp(2.0*xp*H);
		e2=e1*e1;
		d1=n0+1;
		d2=d1-2.0;
		sum=0.0;
		for (i=0;i<NMAX;i++,d1+=2.0,d2-=2.0,e1*=e2)
			sum += c(i)*(e1/d1+1.0/(d2*e1));
		ans=0.5641895835*SIGN(exp(-xp*xp),x)*sum;
	}
	return ans;
}


#include <limits>



namespace {
	inline void mov3(NR_single_precision::DP &a, NR_single_precision::DP &b, NR_single_precision::DP &c, const NR_single_precision::DP d, const NR_single_precision::DP e,
		const NR_single_precision::DP f)
	{
		a=d; b=e; c=f;
	}
}

NR_single_precision::DP NR_single_precision::dbrent(const NR_single_precision::DP ax, const NR_single_precision::DP bx, const NR_single_precision::DP cx, NR_single_precision::DP f(const NR_single_precision::DP),
	NR_single_precision::DP df(const NR_single_precision::DP), const NR_single_precision::DP tol, NR_single_precision::DP &xmin)
{
	const int ITMAX=100;
	const NR_single_precision::DP ZEPS=DBL_EPSILON*1.0e-3;
	bool ok1,ok2;
	int iter;
	NR_single_precision::DP a,b,d=0.0,d1,d2,du,dv,dw,dx,e=0.0;
	NR_single_precision::DP fu,fv,fw,fx,olde,tol1,tol2,u,u1,u2,v,w,x,xm;

	a=(ax < cx ? ax : cx);
	b=(ax > cx ? ax : cx);
	x=w=v=bx;
	fw=fv=fx=f(x);
	dw=dv=dx=df(x);
	for (iter=0;iter<ITMAX;iter++) {
		xm=0.5*(a+b);
		tol1=tol*fabs(x)+ZEPS;
		tol2=2.0*tol1;
		if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
			xmin=x;
			return fx;
		}
		if (fabs(e) > tol1) {
			d1=2.0*(b-a);
			d2=d1;
			if (dw != dx) d1=(w-x)*dx/(dx-dw);
			if (dv != dx) d2=(v-x)*dx/(dx-dv);
			u1=x+d1;
			u2=x+d2;
			ok1 = (a-u1)*(u1-b) > 0.0 && dx*d1 <= 0.0;
			ok2 = (a-u2)*(u2-b) > 0.0 && dx*d2 <= 0.0;
			olde=e;
			e=d;
			if (ok1 || ok2) {
				if (ok1 && ok2)
					d=(fabs(d1) < fabs(d2) ? d1 : d2);
				else if (ok1)
					d=d1;
				else
					d=d2;
				if (fabs(d) <= fabs(0.5*olde)) {
					u=x+d;
					if (u-a < tol2 || b-u < tol2)
						d=SIGN(tol1,xm-x);
				} else {
					d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
				}
			} else {
				d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
			}
		} else {
			d=0.5*(e=(dx >= 0.0 ? a-x : b-x));
		}
		if (fabs(d) >= tol1) {
			u=x+d;
			fu=f(u);
		} else {
			u=x+SIGN(tol1,d);
			fu=f(u);
			if (fu > fx) {
				xmin=x;
				return fx;
			}
		}
		du=df(u);
		if (fu <= fx) {
			if (u >= x) a=x; else b=x;
			mov3(v,fv,dv,w,fw,dw);
			mov3(w,fw,dw,x,fx,dx);
			mov3(x,fx,dx,u,fu,du);
		} else {
			if (u < x) a=u; else b=u;
			if (fu <= fw || w == x) {
				mov3(v,fv,dv,w,fw,dw);
				mov3(w,fw,dw,u,fu,du);
			} else if (fu < fv || v == x || v == w) {
				mov3(v,fv,dv,u,fu,du);
			}
		}
	}
	NR::nrerror("Too many iterations in routine dbrent");
	return 0.0;
}



void NR_single_precision::ddpoly(NR_single_precision::Vec_I_DP &c, const NR_single_precision::DP x, NR_single_precision::Vec_O_DP &pd)
{
	int nnd,j,i;
	NR_single_precision::DP cnst=1.0;

	int nc=c.size()-1;
	int nd=pd.size()-1;
	pd(0)=c(nc);
	for (j=1;j<nd+1;j++) pd(j)=0.0;
	for (i=nc-1;i>=0;i--) {
		nnd=(nd < (nc-i) ? nd : nc-i);
		for (j=nnd;j>0;j--)
			pd(j)=pd(j)*x+pd(j-1);
		pd(0)=pd(0)*x+c(i);
	}
	for (i=2;i<nd+1;i++) {
		cnst *= i;
		pd(i) *= cnst;
	}
}



bool NR_single_precision::decchk(string str, char &ch)
{
	Msg::error("decchk");
	/*
	char c;
	int j,k=0,m=0;
	static int ip[10][8]={{0,1,5,8,9,4,2,7},{1,5,8,9,4,2,7,0},
		{2,7,0,1,5,8,9,4},{3,6,3,6,3,6,3,6},{4,2,7,0,1,5,8,9},
		{5,8,9,4,2,7,0,1},{6,3,6,3,6,3,6,3},{7,0,1,5,8,9,4,2},
		{8,9,4,2,7,0,1,5},{9,4,2,7,0,1,5,8}};
	static int ij[10][8]={{0,1,2,3,4,5,6,7,8,9},{1,2,3,4,0,6,7,8,9,5},
		{2,3,4,0,1,7,8,9,5,6},{3,4,0,1,2,8,9,5,6,7},{4,0,1,2,3,9,5,6,7,8},
		{5,9,8,7,6,0,4,3,2,1},{6,5,9,8,7,1,0,4,3,2},{7,6,5,9,8,2,1,0,4,3},
		{8,7,6,5,9,3,2,1,0,4},{9,8,7,6,5,4,3,2,1,0}};

	int n=str.length();
	for (j=0;j<n;j++) {
		c=str(j);
		if (c >= 48 && c <= 57)
			k=ij(k,ip((c+2) % 10,7 & m++));
	}
	for (j=0;j<10;j++)
		if (ij(k,ip(j,m & 7)) == 0) break;
	ch=char(j+48);
	return k==0;*/
	return false;
}



extern int ncom;
extern NR_single_precision::DP (*nrfunc)(NR_single_precision::Vec_I_DP &);
extern void (*nrdfun)(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &);
extern NR_single_precision::Vec_DP *pcom_p,*xicom_p;

NR_single_precision::DP NR_single_precision::df1dim(const NR_single_precision::DP x)
{
	int j;
	NR_single_precision::DP df1=0.0;
	NR_single_precision::Vec_DP xt(ncom),df(ncom);

	NR_single_precision::Vec_DP &pcom=*pcom_p,&xicom=*xicom_p;
	for (j=0;j<ncom;j++) xt(j)=pcom(j)+x*xicom(j);
	nrdfun(xt,df);
	for (j=0;j<ncom;j++) df1 += df(j)*xicom(j);
	return df1;
}


#include <limits>



void NR_single_precision::dfpmin(NR_single_precision::Vec_IO_DP &p, const NR_single_precision::DP gtol, int &iter, NR_single_precision::DP &fret,
	NR_single_precision::DP func(NR_single_precision::Vec_I_DP &), void dfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const int ITMAX=200;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP TOLX=4*EPS,STPMX=100.0;
	bool check;
	int i,its,j;
	NR_single_precision::DP den,fac,fad,fae,fp,stpmax,sum=0.0,sumdg,sumxi,temp,test;

	int n=p.size();
	NR_single_precision::Vec_DP dg(n),g(n),hdg(n),pnew(n),xi(n);
	NR_single_precision::Mat_DP hessin(n,n);
	fp=func(p);
	dfunc(p,g);
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) hessin(i,j)=0.0;
		hessin(i,i)=1.0;
		xi(i) = -g(i);
		sum += p(i)*p(i);
	}
	stpmax=STPMX*MAX(sqrt(sum),NR_single_precision::DP(n));
	for (its=0;its<ITMAX;its++) {
		iter=its;
		lnsrch(p,fp,g,xi,pnew,fret,stpmax,check,func);
		fp=fret;
		for (i=0;i<n;i++) {
			xi(i)=pnew(i)-p(i);
			p(i)=pnew(i);
		}
		test=0.0;
		for (i=0;i<n;i++) {
			temp=fabs(xi(i))/MAX(fabs(p(i)),1.0);
			if (temp > test) test=temp;
		}
		if (test < TOLX)
			return;
		for (i=0;i<n;i++) dg(i)=g(i);
		dfunc(p,g);
		test=0.0;
		den=MAX(fret,1.0);
		for (i=0;i<n;i++) {
			temp=fabs(g(i))*MAX(fabs(p(i)),1.0)/den;
			if (temp > test) test=temp;
		}
		if (test < gtol)
			return;
		for (i=0;i<n;i++) dg(i)=g(i)-dg(i);
		for (i=0;i<n;i++) {
			hdg(i)=0.0;
			for (j=0;j<n;j++) hdg(i) += hessin(i,j)*dg(j);
		}
		fac=fae=sumdg=sumxi=0.0;
		for (i=0;i<n;i++) {
			fac += dg(i)*xi(i);
			fae += dg(i)*hdg(i);
			sumdg += SQR(dg(i));
			sumxi += SQR(xi(i));
		}
		if (fac > sqrt(EPS*sumdg*sumxi)) {
			fac=1.0/fac;
			fad=1.0/fae;
			for (i=0;i<n;i++) dg(i)=fac*xi(i)-fad*hdg(i);
			for (i=0;i<n;i++) {
				for (j=i;j<n;j++) {
					hessin(i,j) += fac*xi(i)*xi(j)
						-fad*hdg(i)*hdg(j)+fae*dg(i)*dg(j);
					hessin(j,i)=hessin(i,j);
				}
			}
		}
		for (i=0;i<n;i++) {
			xi(i)=0.0;
			for (j=0;j<n;j++) xi(i) -= hessin(i,j)*g(j);
		}
	}
	NR::nrerror("too many iterations in dfpmin");
}


#include <limits>



NR_single_precision::DP NR_single_precision::dfridr(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x, const NR_single_precision::DP h, NR_single_precision::DP &err)
{
	const int NTAB=10;
	const NR_single_precision::DP CON=1.4, CON2=(CON*CON);
	const NR_single_precision::DP BIG=NR_single_precision::MAXIMUM_CONSTANTS;
	const NR_single_precision::DP SAFE=2.0;
	int i,j;
	NR_single_precision::DP errt,fac,hh,ans;
	NR_single_precision::Mat_DP a(NTAB,NTAB);

	if (h == 0.0) NR::nrerror("h must be nonzero in dfridr.");
	hh=h;
	a(0,0)=(func(x+hh)-func(x-hh))/(2.0*hh);
	err=BIG;
	for (i=1;i<NTAB;i++) {
		hh /= CON;
		a(0,i)=(func(x+hh)-func(x-hh))/(2.0*hh);
		fac=CON2;
		for (j=1;j<=i;j++) {
			a(j,i)=(a(j-1,i)*fac-a(j-1,i-1))/(fac-1.0);
			fac=CON2*fac;
			errt=MAX(fabs(a(j,i)-a(j-1,i)),fabs(a(j,i)-a(j-1,i-1)));
			if (errt <= err) {
				err=errt;
				ans=a(j,i);
			}
		}
		if (fabs(a(i,i)-a(i-1,i-1)) >= SAFE*err) break;
	}
	return ans;
}





void NR_single_precision::dftcor(const NR_single_precision::DP w, const NR_single_precision::DP delta, const NR_single_precision::DP a, const NR_single_precision::DP b,
	NR_single_precision::Vec_I_DP &endpts, NR_single_precision::DP &corre, NR_single_precision::DP &corim, NR_single_precision::DP &corfac)
{
	NR_single_precision::DP a0i,a0r,a1i,a1r,a2i,a2r,a3i,a3r,arg,c,cl,cr,s,sl,sr,t,t2,t4,t6,
		cth,ctth,spth2,sth,sth4i,stth,th,th2,th4,tmth2,tth4i;

	th=w*delta;
	if (a >= b || th < 0.0e0 || th > 3.1416e0)
		NR::nrerror("bad arguments to dftcor");
	if (fabs(th) < 5.0e-2) {
		t=th;
		t2=t*t;
		t4=t2*t2;
		t6=t4*t2;
		corfac=1.0-(11.0/720.0)*t4+(23.0/15120.0)*t6;
		a0r=(-2.0/3.0)+t2/45.0+(103.0/15120.0)*t4-(169.0/226800.0)*t6;
		a1r=(7.0/24.0)-(7.0/180.0)*t2+(5.0/3456.0)*t4-(7.0/259200.0)*t6;
		a2r=(-1.0/6.0)+t2/45.0-(5.0/6048.0)*t4+t6/64800.0;
		a3r=(1.0/24.0)-t2/180.0+(5.0/24192.0)*t4-t6/259200.0;
		a0i=t*(2.0/45.0+(2.0/105.0)*t2-(8.0/2835.0)*t4+(86.0/467775.0)*t6);
		a1i=t*(7.0/72.0-t2/168.0+(11.0/72576.0)*t4-(13.0/5987520.0)*t6);
		a2i=t*(-7.0/90.0+t2/210.0-(11.0/90720.0)*t4+(13.0/7484400.0)*t6);
		a3i=t*(7.0/360.0-t2/840.0+(11.0/362880.0)*t4-(13.0/29937600.0)*t6);
	} else {
		cth=cos(th);
		sth=sin(th);
		ctth=cth*cth-sth*sth;
		stth=2.0e0*sth*cth;
		th2=th*th;
		th4=th2*th2;
		tmth2=3.0e0-th2;
		spth2=6.0e0+th2;
		sth4i=1.0/(6.0e0*th4);
		tth4i=2.0e0*sth4i;
		corfac=tth4i*spth2*(3.0e0-4.0e0*cth+ctth);
		a0r=sth4i*(-42.0e0+5.0e0*th2+spth2*(8.0e0*cth-ctth));
		a0i=sth4i*(th*(-12.0e0+6.0e0*th2)+spth2*stth);
		a1r=sth4i*(14.0e0*tmth2-7.0e0*spth2*cth);
		a1i=sth4i*(30.0e0*th-5.0e0*spth2*sth);
		a2r=tth4i*(-4.0e0*tmth2+2.0e0*spth2*cth);
		a2i=tth4i*(-12.0e0*th+2.0e0*spth2*sth);
		a3r=sth4i*(2.0e0*tmth2-spth2*cth);
		a3i=sth4i*(6.0e0*th-spth2*sth);
	}
	cl=a0r*endpts(0)+a1r*endpts(1)+a2r*endpts(2)+a3r*endpts(3);
	sl=a0i*endpts(0)+a1i*endpts(1)+a2i*endpts(2)+a3i*endpts(3);
	cr=a0r*endpts(7)+a1r*endpts(6)+a2r*endpts(5)+a3r*endpts(4);
	sr= -a0i*endpts(7)-a1i*endpts(6)-a2i*endpts(5)-a3i*endpts(4);
	arg=w*(b-a);
	c=cos(arg);
	s=sin(arg);
	corre=cl+c*cr-s*sr;
	corim=sl+s*cr+c*sr;
}





void NR_single_precision::dftint(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP w,
	NR_single_precision::DP &cosint, NR_single_precision::DP &sinint)
{
	static int init=0;
	static NR_single_precision::DP (*funcold)(const NR_single_precision::DP);
	static NR_single_precision::DP aold = -1.e30,bold = -1.e30,delta;
	const int M=64,NDFT=1024,MPOL=6;
	const NR_single_precision::DP TWOPI=6.283185307179586476;
	int j,nn;
	NR_single_precision::DP c,cdft,cerr,corfac,corim,corre,en,s,sdft,serr;
	static NR_single_precision::Vec_DP data(NDFT),endpts(8);
	NR_single_precision::Vec_DP cpol(MPOL),spol(MPOL),xpol(MPOL);

	if (init != 1 || a != aold || b != bold || func != funcold) {
		init=1;
		aold=a;
		bold=b;
		funcold=func;
		delta=(b-a)/M;
		for (j=0;j<M+1;j++)
			data(j)=func(a+j*delta);
		for (j=M+1;j<NDFT;j++)
			data(j)=0.0;
		for (j=0;j<4;j++) {
			endpts(j)=data(j);
			endpts(j+4)=data(M-3+j);
		}
		realft(data,1);
		data(1)=0.0;
	}
	en=w*delta*NDFT/TWOPI+1.0;
	nn=MIN(MAX(int(en-0.5*MPOL+1.0),1),NDFT/2-MPOL+1);
	for (j=0;j<MPOL;j++,nn++) {
		cpol(j)=data(2*nn-2);
		spol(j)=data(2*nn-1);
		xpol(j)=nn;
	}
	polint(xpol,cpol,en,cdft,cerr);
	polint(xpol,spol,en,sdft,serr);
	dftcor(w,delta,a,b,endpts,corre,corim,corfac);
	cdft *= corfac;
	sdft *= corfac;
	cdft += corre;
	sdft += corim;
	c=delta*cos(w*a);
	s=delta*sin(w*a);
	cosint=c*cdft-s*sdft;
	sinint=s*cdft+c*sdft;
}








NR_single_precision::DP (*nrfunc)(NR_single_precision::Vec_I_DP &);
void (*nrdfun)(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &);
NR_single_precision::Vec_DP *pcom_p,*xicom_p;

void NR_single_precision::dlinmin(NR_single_precision::Vec_IO_DP &p, NR_single_precision::Vec_IO_DP &xi, NR_single_precision::DP &fret, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &),
	void dfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const NR_single_precision::DP TOL=2.0e-8;
	int j;
	NR_single_precision::DP xx,xmin,fx,fb,fa,bx,ax;

	int n=p.size();
	ncom=n;
	pcom_p=new NR_single_precision::Vec_DP(n);
	xicom_p=new NR_single_precision::Vec_DP(n);
	nrfunc=func;
	nrdfun=dfunc;
	NR_single_precision::Vec_DP &pcom=*pcom_p,&xicom=*xicom_p;
	for (j=0;j<n;j++) {
		pcom(j)=p(j);
		xicom(j)=xi(j);
	}
	ax=0.0;
	xx=1.0;
	mnbrak(ax,xx,bx,fa,fx,fb,f1dim);
	fret=dbrent(ax,xx,bx,f1dim,df1dim,TOL,xmin);
	for (j=0;j<n;j++) {
		xi(j) *= xmin;
		p(j) += xi(j);
	}
	delete xicom_p;
	delete pcom_p;
}



void NR_single_precision::eclass(NR_single_precision::Vec_O_INT &nf, NR_single_precision::Vec_I_INT &lista, NR_single_precision::Vec_I_INT &listb)
{
	int l,k,j;

	int n=nf.size();
	int m=lista.size();
	for (k=0;k<n;k++) nf(k)=k;
	for (l=0;l<m;l++) {
		j=lista(l);
		while (nf(j) != j) j=nf(j);
		k=listb(l);
		while (nf(k) != k) k=nf(k);
		if (j != k) nf(j)=k;
	}
	for (j=0;j<n;j++)
		while (nf(j) != nf(nf(j))) nf(j)=nf(nf(j));
}



void NR_single_precision::eclazz(NR_single_precision::Vec_O_INT &nf, bool equiv(const int, const int))
{
	int kk,jj;

	int n=nf.size();
	nf(0)=0;
	for (jj=1;jj<n;jj++) {
		nf(jj)=jj;
		for (kk=0;kk<jj;kk++) {
			nf(kk)=nf(nf(kk));
			if (equiv(jj+1,kk+1)) nf(nf(nf(kk)))=jj;
		}
	}
	for (jj=0;jj<n;jj++) nf(jj)=nf(nf(jj));
}


#include <limits>



NR_single_precision::DP NR_single_precision::ei(const NR_single_precision::DP x)
{
	const int MAXIT=100;
	const NR_single_precision::DP EULER=0.577215664901533;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN/EPS;
	int k;
	NR_single_precision::DP fact,prev,sum,term;

	if (x <= 0.0) NR::nrerror("Bad argument in ei");
	if (x < FPMIN) return log(x)+EULER;
	if (x <= -log(EPS)) {
		sum=0.0;
		fact=1.0;
		for (k=1;k<=MAXIT;k++) {
			fact *= x/k;
			term=fact/k;
			sum += term;
			if (term < EPS*sum) break;
		}
		if (k > MAXIT) NR::nrerror("Series failed in ei");
		return sum+log(x)+EULER;
	} else {
		sum=0.0;
		term=1.0;
		for (k=1;k<=MAXIT;k++) {
			prev=term;
			term *= k/x;
			if (term < EPS) break;
			if (term < prev) sum += term;
			else {
				sum -= prev;
				break;
			}
		}
		return exp(x)*(1.0+sum)/x;
	}
}



void NR_single_precision::eigsrt(NR_single_precision::Vec_IO_DP &d, NR_single_precision::Mat_IO_DP &v)
{
	int i,j,k;
	NR_single_precision::DP p;

	int n=d.size();
	for (i=0;i<n-1;i++) {
		p=d(k=i);
		for (j=i;j<n;j++)
			if (d(j) >= p) p=d(k=j);
		if (k != i) {
			d(k)=d(i);
			d(i)=p;
			for (j=0;j<n;j++) {
				p=v(j,i);
				v(j,i)=v(j,k);
				v(j,k)=p;
			}
		}
	}
}





NR_single_precision::DP NR_single_precision::elle(const NR_single_precision::DP phi, const NR_single_precision::DP ak)
{
	NR_single_precision::DP cc,q,s;

	s=sin(phi);
	cc=SQR(cos(phi));
	q=(1.0-s*ak)*(1.0+s*ak);
	return s*(rf(cc,q,1.0)-(SQR(s*ak))*rd(cc,q,1.0)/3.0);
}





NR_single_precision::DP NR_single_precision::ellf(const NR_single_precision::DP phi, const NR_single_precision::DP ak)
{
	NR_single_precision::DP s;

	s=sin(phi);
	return s*rf(SQR(cos(phi)),(1.0-s*ak)*(1.0+s*ak),1.0);
}





NR_single_precision::DP NR_single_precision::ellpi(const NR_single_precision::DP phi, const NR_single_precision::DP en, const NR_single_precision::DP ak)
{
	NR_single_precision::DP cc,enss,q,s;

	s=sin(phi);
	enss=en*s*s;
	cc=SQR(cos(phi));
	q=(1.0-s*ak)*(1.0+s*ak);
	return s*(rf(cc,q,1.0)-enss*rj(cc,q,1.0,1.0+enss)/3.0);
}





void NR_single_precision::elmhes(NR_single_precision::Mat_IO_DP &a)
{
	int i,j,m;
	NR_single_precision::DP y,x;

	int n=a.rows();
	for (m=1;m<n-1;m++) {
		x=0.0;
		i=m;
		for (j=m;j<n;j++) {
			if (fabs(a(j,m-1)) > fabs(x)) {
				x=a(j,m-1);
				i=j;
			}
		}
		if (i != m) {
			for (j=m-1;j<n;j++) SWAP(a(i,j),a(m,j));
			for (j=0;j<n;j++) SWAP(a(j,i),a(j,m));
		}
		if (x != 0.0) {
			for (i=m+1;i<n;i++) {
				if ((y=a(i,m-1)) != 0.0) {
					y /= x;
					a(i,m-1)=y;
					for (j=m;j<n;j++) a(i,j) -= y*a(m,j);
					for (j=0;j<n;j++) a(j,m) += y*a(j,i);
				}
			}
		}
	}
}





NR_single_precision::DP NR_single_precision::erfcc(const NR_single_precision::DP x)
{
	NR_single_precision::DP t,z,ans;

	z=fabs(x);
	t=1.0/(1.0+0.5*z);
	ans=t*exp(-z*z-1.26551223+t*(1.00002368+t*(0.37409196+t*(0.09678418+
		t*(-0.18628806+t*(0.27886807+t*(-1.13520398+t*(1.48851587+
		t*(-0.82215223+t*0.17087277)))))))));
	return (x >= 0.0 ? ans : 2.0-ans);
}



NR_single_precision::DP NR_single_precision::erff(const NR_single_precision::DP x)
{
	return x < 0.0 ? -gammp(0.5,x*x) : gammp(0.5,x*x);
}



NR_single_precision::DP NR_single_precision::erffc(const NR_single_precision::DP x)
{
	return x < 0.0 ? 1.0+gammp(0.5,x*x) : gammq(0.5,x*x);
}





void NR_single_precision::eulsum(NR_single_precision::DP &sum, const NR_single_precision::DP term, const int jterm, NR_single_precision::Vec_IO_DP &wksp)
{
	int j;
	static int nterm;
	NR_single_precision::DP tmp,dum;

	if (jterm == 0) {
		nterm=1;
		sum=0.5*(wksp(0)=term);
	} else {
		if (nterm+1 > wksp.size()) NR::nrerror("wksp too small in euler");
		tmp=wksp(0);
		wksp(0)=term;
		for (j=1;j<nterm;j++) {
			dum=wksp(j);
			wksp(j)=0.5*(wksp(j-1)+tmp);
			tmp=dum;
		}
		wksp(nterm)=0.5*(wksp(nterm-1)+tmp);
		if (fabs(wksp(nterm)) <= fabs(wksp(nterm-1)))
			sum += (0.5*wksp(nterm++));
		else
			sum += wksp(nterm);
	}
}





NR_single_precision::DP NR_single_precision::evlmem(const NR_single_precision::DP fdt, NR_single_precision::Vec_I_DP &d, const NR_single_precision::DP xms)
{
	int i;
	NR_single_precision::DP sumr=1.0,sumi=0.0,wr=1.0,wi=0.0,wpr,wpi,wtemp,theta;

	int m=d.size();
	theta=6.28318530717959*fdt;
	wpr=cos(theta);
	wpi=sin(theta);
	for (i=0;i<m;i++) {
		wr=(wtemp=wr)*wpr-wi*wpi;
		wi=wi*wpr+wtemp*wpi;
		sumr -= d(i)*wr;
		sumi -= d(i)*wi;
	}
	return xms/(sumr*sumr+sumi*sumi);
}





NR_single_precision::DP NR_single_precision::expdev(int &idum)
{
	NR_single_precision::DP dum;

	do
		dum=ran1(idum);
	while (dum == 0.0);
	return -log(dum);
}


#include <limits>



NR_single_precision::DP NR_single_precision::expint(const int n, const NR_single_precision::DP x)
{
	const int MAXIT=100;
	const NR_single_precision::DP EULER=0.577215664901533;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP BIG=NR_single_precision::MAXIMUM_CONSTANTS*EPS;
	int i,ii,nm1;
	NR_single_precision::DP a,b,c,d,del,fact,h,psi,ans;

	nm1=n-1;
	if (n < 0 || x < 0.0 || (x==0.0 && (n==0 || n==1)))
	NR::nrerror("bad arguments in expint");
	else {
		if (n == 0) ans=exp(-x)/x;
		else {
			if (x == 0.0) ans=1.0/nm1;
			else {
				if (x > 1.0) {
					b=x+n;
					c=BIG;
					d=1.0/b;
					h=d;
					for (i=1;i<=MAXIT;i++) {
						a = -i*(nm1+i);
						b += 2.0;
						d=1.0/(a*d+b);
						c=b+a/c;
						del=c*d;
						h *= del;
						if (fabs(del-1.0) <= EPS) {
							ans=h*exp(-x);
							return ans;
						}
					}
					NR::nrerror("continued fraction failed in expint");
				} else {
					ans = (nm1!=0 ? 1.0/nm1 : -log(x)-EULER);
					fact=1.0;
					for (i=1;i<=MAXIT;i++) {
						fact *= -x/i;
						if (i != nm1) del = -fact/(i-nm1);
						else {
							psi = -EULER;
							for (ii=1;ii<=nm1;ii++) psi += 1.0/ii;
							del=fact*(-log(x)+psi);
						}
						ans += del;
						if (fabs(del) < fabs(ans)*EPS) return ans;
					}
					NR::nrerror("series failed in expint");
				}
			}
		}
	}
	return ans;
}



extern int ncom;
extern NR_single_precision::DP (*nrfunc)(NR_single_precision::Vec_I_DP &);
extern NR_single_precision::Vec_DP *pcom_p,*xicom_p;

NR_single_precision::DP NR_single_precision::f1dim(const NR_single_precision::DP x)
{
	int j;

	NR_single_precision::Vec_DP xt(ncom);
	NR_single_precision::Vec_DP &pcom=*pcom_p,&xicom=*xicom_p;
	for (j=0;j<ncom;j++)
		xt(j)=pcom(j)+x*xicom(j);
	return nrfunc(xt);
}



NR_single_precision::DP NR_single_precision::factln(const int n)
{
	static NR_single_precision::DP a[101];

	if (n < 0) NR::nrerror("Negative factorial in routine factln");
	if (n <= 1) return 0.0;
	if (n <= 100)
		return (a[n] != 0.0 ? a[n] : (a[n]=gammln(n+1.0)));
	else return gammln(n+1.0);
}





NR_single_precision::DP NR_single_precision::factrl(const int n)
{
	static int ntop=4;
	static NR_single_precision::DP a[33]={1.0,1.0,2.0,6.0,24.0};
	int j;

	if (n < 0) NR::nrerror("Negative factorial in routine factrl");
	if (n > 32) return exp(gammln(n+1.0));
	while (ntop<n) {
		j=ntop++;
		a[ntop]=a[j]*ntop;
	}
	return a[n];
}





void NR_single_precision::fasper(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, const NR_single_precision::DP ofac, const NR_single_precision::DP hifac,
	NR_single_precision::Vec_O_DP &wk1, NR_single_precision::Vec_O_DP &wk2, int &nout, int &jmax, NR_single_precision::DP &prob)
{
	const int MACC=4;
	int j,k,ndim,nfreq,nfreqt;
	NR_single_precision::DP ave,ck,ckk,cterm,cwt,den,df,effm,expy,fac,fndim,hc2wt,hs2wt,
		hypo,pmax,sterm,swt,var,xdif,xmax,xmin;

	int n=x.size();
	int nwk=wk1.size();
	nout=0.5*ofac*hifac*n;
	nfreqt=ofac*hifac*n*MACC;
	nfreq=64;
	while (nfreq < nfreqt) nfreq <<= 1;
	ndim=nfreq << 1;
	if (ndim > nwk) NR::nrerror("workspaces too small in fasper");
	avevar(y,ave,var);
	if (var == 0.0) NR::nrerror("zero variance in fasper");
	xmin=x(0);
	xmax=xmin;
	for (j=1;j<n;j++) {
		if (x(j) < xmin) xmin=x(j);
		if (x(j) > xmax) xmax=x(j);
	}
	xdif=xmax-xmin;
	NR_single_precision::Vec_DP wk1_t(0.0,ndim);
	NR_single_precision::Vec_DP wk2_t(0.0,ndim);
	fac=ndim/(xdif*ofac);
	fndim=ndim;
	for (j=0;j<n;j++) {
		ck=fmod((x(j)-xmin)*fac,fndim);
		ckk=2.0*(ck++);
		ckk=fmod(ckk,fndim);
		++ckk;
		spread(y(j)-ave,wk1_t,ck,MACC);
		spread(1.0,wk2_t,ckk,MACC);
	}
	realft(wk1_t,1);
	realft(wk2_t,1);
	df=1.0/(xdif*ofac);
	pmax = -1.0;
	for (k=2,j=0;j<nout;j++,k+=2) {
		hypo=sqrt(wk2_t(k)*wk2_t(k)+wk2_t(k+1)*wk2_t(k+1));
		hc2wt=0.5*wk2_t(k)/hypo;
		hs2wt=0.5*wk2_t(k+1)/hypo;
		cwt=sqrt(0.5+hc2wt);
		swt=SIGN(sqrt(0.5-hc2wt),hs2wt);
		den=0.5*n+hc2wt*wk2_t(k)+hs2wt*wk2_t(k+1);
		cterm=SQR(cwt*wk1_t(k)+swt*wk1_t(k+1))/den;
		sterm=SQR(cwt*wk1_t(k+1)-swt*wk1_t(k))/(n-den);
		wk1(j)=(j+1)*df;
		wk2(j)=(cterm+sterm)/(2.0*var);
		if (wk2(j) > pmax) pmax=wk2(jmax=j);
	}
	expy=exp(-pmax);
	effm=2.0*nout/ofac;
	prob=effm*expy;
	if (prob > 0.01) prob=1.0-pow(1.0-expy,effm);
}





void NR_single_precision::fdjac(NR_single_precision::Vec_IO_DP &x, NR_single_precision::Vec_I_DP &fvec, NR_single_precision::Mat_O_DP &df,
	void vecfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const NR_single_precision::DP EPS=1.0e-8;
	int i,j;
	NR_single_precision::DP h,temp;

	int n=x.size();
	NR_single_precision::Vec_DP f(n);
	for (j=0;j<n;j++) {
		temp=x(j);
		h=EPS*fabs(temp);
		if (h == 0.0) h=EPS;
		x(j)=temp+h;
		h=x(j)-temp;
		vecfunc(x,f);
		x(j)=temp;
		for (i=0;i<n;i++)
			df(i,j)=(f(i)-fvec(i))/h;
	}
}





void NR_single_precision::fgauss(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &a, NR_single_precision::DP &y, NR_single_precision::Vec_O_DP &dyda)
{
	int i;
	NR_single_precision::DP fac,ex,arg;

	int na=a.size();
	y=0.0;
	for (i=0;i<na-1;i+=3) {
		arg=(x-a(i+1))/a(i+2);
		ex=exp(-arg*arg);
		fac=a(i)*ex*2.0*arg;
		y += a(i)*ex;
		dyda(i)=ex;
		dyda(i+1)=fac/a(i+2);
		dyda(i+2)=fac*arg/a(i+2);
	}
}





void NR_single_precision::fit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, const bool mwt, NR_single_precision::DP &a,
	NR_single_precision::DP &b, NR_single_precision::DP &siga, NR_single_precision::DP &sigb, NR_single_precision::DP &chi2, NR_single_precision::DP &q)
{
	int i;
	NR_single_precision::DP wt,t,sxoss,sx=0.0,sy=0.0,st2=0.0,ss,sigdat;

	int ndata=x.size();
	b=0.0;
	if (mwt) {
		ss=0.0;
		for (i=0;i<ndata;i++) {
			wt=1.0/SQR(sig(i));
			ss += wt;
			sx += x(i)*wt;
			sy += y(i)*wt;
		}
	} else {
		for (i=0;i<ndata;i++) {
			sx += x(i);
			sy += y(i);
		}
		ss=ndata;
	}
	sxoss=sx/ss;
	if (mwt) {
		for (i=0;i<ndata;i++) {
			t=(x(i)-sxoss)/sig(i);
			st2 += t*t;
			b += t*y(i)/sig(i);
		}
	} else {
		for (i=0;i<ndata;i++) {
			t=x(i)-sxoss;
			st2 += t*t;
			b += t*y(i);
		}
	}
	b /= st2;
	a=(sy-sx*b)/ss;
	siga=sqrt((1.0+sx*sx/(ss*st2))/ss);
	sigb=sqrt(1.0/st2);
	chi2=0.0;
	q=1.0;
	if (!mwt) {
		for (i=0;i<ndata;i++)
			chi2 += SQR(y(i)-a-b*x(i));
		sigdat=sqrt(chi2/(ndata-2));
		siga *= sigdat;
		sigb *= sigdat;
	} else {
		for (i=0;i<ndata;i++)
			chi2 += SQR((y(i)-a-b*x(i))/sig(i));
		if (ndata>2) q=gammq(0.5*(ndata-2),0.5*chi2);
	}
}






void NR_single_precision::fitexy(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sigx, NR_single_precision::Vec_I_DP &sigy,
	NR_single_precision::DP &a, NR_single_precision::DP &b, NR_single_precision::DP &siga, NR_single_precision::DP &sigb, NR_single_precision::DP &chi2, NR_single_precision::DP &q)
{
	Msg::error("fitexy");
	/*
	int j;
	const NR_single_precision::DP POTN=1.571000,BIG=1.0e30,ACC=1.0e-3;
	const NR_single_precision::DP PI=3.141592653589793238;
	NR_single_precision::DP amx,amn,varx,vary,ang(7),ch(7),scale,bmn,bmx,d1,d2,r2,
		dum1,dum2,dum3,dum4,dum5;

	int ndat=x.size();
	xx_p=new NR_single_precision::Vec_DP(ndat);
	yy_p=new NR_single_precision::Vec_DP(ndat);
	sx_p=new NR_single_precision::Vec_DP(ndat);
	sy_p=new NR_single_precision::Vec_DP(ndat);
	ww_p=new NR_single_precision::Vec_DP(ndat);
	NR_single_precision::Vec_DP &xx=*xx_p, &yy=*yy_p;
	NR_single_precision::Vec_DP &sx=*sx_p, &sy=*sy_p, &ww=*ww_p;
	avevar(x,dum1,varx);
	avevar(y,dum1,vary);
	scale=sqrt(varx/vary);
	for (j=0;j<ndat;j++) {
		xx(j)=x(j);
		yy(j)=y(j)*scale;
		sx(j)=sigx(j);
		sy(j)=sigy(j)*scale;
		ww(j)=sqrt(SQR(sx(j))+SQR(sy(j)));
	}
	fit(xx,yy,ww,true,dum1,b,dum2,dum3,dum4,dum5);
	offs=ang(0)=0.0;
	ang(1)=atan(b);
	ang(3)=0.0;
	ang(4)=ang(1);
	ang(5)=POTN;
	for (j=3;j<6;j++) ch(j)=chixy(ang(j));
	mnbrak(ang(0),ang(1),ang(2),ch(0),ch(1),ch(2),chixy);
	chi2=brent(ang(0),ang(1),ang(2),chixy,ACC,b);
	chi2=chixy(b);
	a=aa;
	q=gammq(0.5*(ndat-2),chi2*0.5);
	r2=0.0;
	for (j=0;j<ndat;j++) r2 += ww(j);
	r2=1.0/r2;
	bmx=BIG;
	bmn=BIG;
	offs=chi2+1.0;
	for (j=0;j<6;j++) {
		if (ch(j) > offs) {
			d1=fabs(ang(j)-b);
			while (d1 >= PI) d1 -= PI;
			d2=PI-d1;
			if (ang(j) < b)
				SWAP(d1,d2);
			if (d1 < bmx) bmx=d1;
			if (d2 < bmn) bmn=d2;
		}
	}
	if (bmx < BIG) {
		bmx=zbrent(chixy,b,b+bmx,ACC)-b;
		amx=aa-a;
		bmn=zbrent(chixy,b,b-bmn,ACC)-b;
		amn=aa-a;
		sigb=sqrt(0.5*(bmx*bmx+bmn*bmn))/(scale*SQR(cos(b)));
		siga=sqrt(0.5*(amx*amx+amn*amn)+r2)/scale;
	} else sigb=siga=BIG;
	a /= scale;
	b=tan(b)/scale;
	delete ww_p; delete sy_p; delete sx_p; delete yy_p; delete xx_p;*/
}


#include <complex>



void NR_single_precision::fixrts(NR_single_precision::Vec_IO_DP &d)
{
	bool polish=true;
	int i,j;

	int m=d.size();
	Vec_CPLX_DP a(m+1),roots(m);
	a(m)=1.0;
	for (j=0;j<m;j++)
		a(j)= -d(m-1-j);
	zroots(a,roots,polish);
	for (j=0;j<m;j++)
		if (abs(roots(j)) > 1.0)
			roots(j)=1.0/conj(roots(j));
	a(0)= -roots(0);
	a(1)=1.0;
	for (j=1;j<m;j++) {
		a(j+1)=1.0;
		for (i=j;i>=1;i--)
			a(i)=a(i-1)-roots(j)*a(i);
		a(0)= -roots(j)*a(0);
	}
	for (j=0;j<m;j++)
		d(m-1-j) = -real(a(j));
}



void NR_single_precision::fleg(const NR_single_precision::DP x, NR_single_precision::Vec_O_DP &pl)
{
	int j;
	NR_single_precision::DP twox,f2,f1,d;

	int nl=pl.size();
	pl(0)=1.0;
	pl(1)=x;
	if (nl > 2) {
		twox=2.0*x;
		f2=x;
		d=1.0;
		for (j=2;j<nl;j++) {
			f1=d++;
			f2+=twox;
			pl(j)=(f2*pl(j-1)-f1*pl(j-2))/d;
		}
	}
}





void NR_single_precision::flmoon(const int n, const int nph, int &jd, NR_single_precision::DP &frac)
{
	const NR_single_precision::DP RAD=3.141592653589793238/180.0;
	int i;
	NR_single_precision::DP am,as,c,t,t2,xtra;

	c=n+nph/4.0;
	t=c/1236.85;
	t2=t*t;
	as=359.2242+29.105356*c;
	am=306.0253+385.816918*c+0.010730*t2;
	jd=2415020+28*n+7*nph;
	xtra=0.75933+1.53058868*c+((1.178e-4)-(1.55e-7)*t)*t2;
	if (nph == 0 || nph == 2)
		xtra += (0.1734-3.93e-4*t)*sin(RAD*as)-0.4068*sin(RAD*am);
	else if (nph == 1 || nph == 3)
		xtra += (0.1721-4.0e-4*t)*sin(RAD*as)-0.6280*sin(RAD*am);
	else NR::nrerror("nph is unknown in flmoon");
	i=int(xtra >= 0.0 ? floor(xtra) : ceil(xtra-1.0));
	jd += i;
	frac=xtra-i;
}



extern NR_single_precision::Vec_DP *fvec_p;
extern void (*nrfuncv)(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f);

NR_single_precision::DP NR_single_precision::fmin(NR_single_precision::Vec_I_DP &x)
{
	int i;
	NR_single_precision::DP sum;

	NR_single_precision::Vec_DP &fvec=*fvec_p;
	nrfuncv(x,fvec);
	int n=x.size();
	for (sum=0.0,i=0;i<n;i++) sum += SQR(fvec(i));
	return 0.5*sum;
}





void NR_single_precision::four1(NR_single_precision::Vec_IO_DP &data, const int isign)
{
	int n,mmax,m,j,istep,i;
	NR_single_precision::DP wtemp,wr,wpr,wpi,wi,theta,tempr,tempi;

	int nn=data.size()/2;
	n=nn << 1;
	j=1;
	for (i=1;i<n;i+=2) {
		if (j > i) {
			SWAP(data(j-1),data(i-1));
			SWAP(data(j),data(i));
		}
		m=nn;
		while (m >= 2 && j > m) {
			j -= m;
			m >>= 1;
		}
		j += m;
	}
	mmax=2;
	while (n > mmax) {
		istep=mmax << 1;
		theta=isign*(6.28318530717959/mmax);
		wtemp=sin(0.5*theta);
		wpr = -2.0*wtemp*wtemp;
		wpi=sin(theta);
		wr=1.0;
		wi=0.0;
		for (m=1;m<mmax;m+=2) {
			for (i=m;i<=n;i+=istep) {
				j=i+mmax;
				tempr=wr*data(j-1)-wi*data(j);
				tempi=wr*data(j)+wi*data(j-1);
				data(j-1)=data(i-1)-tempr;
				data(j)=data(i)-tempi;
				data(i-1) += tempr;
				data(i) += tempi;
			}
			wr=(wtemp=wr)*wpr-wi*wpi+wr;
			wi=wi*wpr+wtemp*wpi+wi;
		}
		mmax=istep;
	}
}

#include <fstream>



void NR_single_precision::fourew(Vec_FSTREAM_p &file, int &na, int &nb, int &nc, int &nd)
{
	int i;

	for (i=0;i<4;i++) (*file(i)).seekp(0);
	for (i=0;i<4;i++) (*file(i)).seekg(0);
	SWAP(file(1),file(3));
	SWAP(file(0),file(2));
	na=2;
	nb=3;
	nc=0;
	nd=1;
}

#include <iostream>
#include <fstream>




void NR_single_precision::fourfs(Vec_FSTREAM_p &file, NR_single_precision::Vec_I_INT &nn, const int isign)
{
	Msg::error("fourfs");
	/*
	const int KBF=128;
	static int mate(4)={1,0,3,2};
	int cc,cc0,j,j12,jk,k,kk,n=1,mm,kc=0,kd,ks,kr,na,nb,nc,nd,nr,ns,nv;
	NR_single_precision::DP tempr,tempi,wr,wi,wpr,wpi,wtemp,theta;
	NR_single_precision::Vec_DP afa(KBF),afb(KBF),afc(KBF);

	int ndim=nn.size();
	for (j=0;j<ndim;j++) {
		n *= nn(j);
		if (nn(j) <= 1) NR::nrerror("invalid NR_single_precision::DP or wrong ndim in fourfs");
	}
	nv=0;
	jk=nn(nv);
	mm=n;
	ns=n/KBF;
	nr=ns >> 1;
	kd=KBF >> 1;
	ks=n;
	fourew(file,na,nb,nc,nd);
	for (;;) {
		theta=isign*3.141592653589793/(n/mm);
		wtemp=sin(0.5*theta);
		wpr = -2.0*wtemp*wtemp;
		wpi=sin(theta);
		wr=1.0;
		wi=0.0;
		mm >>= 1;
		for (j12=0;j12<2;j12++) {
			kr=0;
			do {
				cc0=(*file(na)).tellg()/sizeof(NR_single_precision::DP);
				(*file(na)).read((char *) &afa(0),KBF*sizeof(NR_single_precision::DP));
				cc=(*file(na)).tellg()/sizeof(NR_single_precision::DP);
				if ((cc-cc0) != KBF) NR::nrerror("read error 1 in fourfs");
				cc0=(*file(nb)).tellg()/sizeof(NR_single_precision::DP);
				(*file(nb)).read((char *) &afb(0),KBF*sizeof(NR_single_precision::DP));
				cc=(*file(nb)).tellg()/sizeof(NR_single_precision::DP);
				if ((cc-cc0) != KBF) NR::nrerror("read error 2 in fourfs");
				for (j=0;j<KBF;j+=2) {
					tempr=wr*afb(j)-wi*afb(j+1);
					tempi=wi*afb(j)+wr*afb(j+1);
					afb(j)=afa(j)-tempr;
					afa(j) += tempr;
					afb(j+1)=afa(j+1)-tempi;
					afa(j+1) += tempi;
				}
				kc += kd;
				if (kc == mm) {
					kc=0;
					wr=(wtemp=wr)*wpr-wi*wpi+wr;
					wi=wi*wpr+wtemp*wpi+wi;
				}
				cc0=(*file(nc)).tellp()/sizeof(NR_single_precision::DP);
				(*file(nc)).write((char *) &afa(0),KBF*sizeof(NR_single_precision::DP));
				cc=(*file(nc)).tellp()/sizeof(NR_single_precision::DP);
				if ((cc-cc0) != KBF) NR::nrerror("write error 1 in fourfs");
				cc0=(*file(nd)).tellp()/sizeof(NR_single_precision::DP);
				(*file(nd)).write((char *) &afb(0),KBF*sizeof(NR_single_precision::DP));
				cc=(*file(nd)).tellp()/sizeof(NR_single_precision::DP);
				if ((cc-cc0) != KBF) NR::nrerror("write error 2 in fourfs");
			} while (++kr < nr);
			if (j12 == 0 && ks != n && ks == KBF) {
				na=mate(na);
				nb=na;
			}
			if (nr == 0) break;
		}
		fourew(file,na,nb,nc,nd);
		jk >>= 1;
		while (jk == 1) {
			mm=n;
			jk=nn(++nv);
		}
		ks >>= 1;
		if (ks > KBF) {
			for (j12=0;j12<2;j12++) {
				for (kr=0;kr<ns;kr+=ks/KBF) {
					for (k=0;k<ks;k+=KBF) {
						cc0=(*file(na)).tellg()/sizeof(NR_single_precision::DP);
						(*file(na)).read((char *) &afa(0),KBF*sizeof(NR_single_precision::DP));
						cc=(*file(na)).tellg()/sizeof(NR_single_precision::DP);
						if ((cc-cc0) != KBF) NR::nrerror("read error 3 in fourfs");
						cc0=(*file(nc)).tellp()/sizeof(NR_single_precision::DP);
						(*file(nc)).write((char *) &afa(0),KBF*sizeof(NR_single_precision::DP));
						cc=(*file(nc)).tellp()/sizeof(NR_single_precision::DP);
						if ((cc-cc0) != KBF) NR::nrerror("write error 3 in fourfs");
					}
					nc=mate(nc);
				}
				na=mate(na);
			}
			fourew(file,na,nb,nc,nd);
		} else if (ks == KBF) nb=na;
		else break;
	}
	j=0;
	for (;;) {
		theta=isign*3.141592653589793/(n/mm);
		wtemp=sin(0.5*theta);
		wpr = -2.0*wtemp*wtemp;
		wpi=sin(theta);
		wr=1.0;
		wi=0.0;
		mm >>= 1;
		ks=kd;
		kd >>= 1;
		for (j12=0;j12<2;j12++) {
			for (kr=0;kr<ns;kr++) {
				cc0=(*file(na)).tellg()/sizeof(NR_single_precision::DP);
				(*file(na)).read((char *) &afc(0),KBF*sizeof(NR_single_precision::DP));
				cc=(*file(na)).tellg()/sizeof(NR_single_precision::DP);
				if ((cc-cc0) != KBF) NR::nrerror("read error 4 in fourfs");
				kk=0;
				k=ks;
				for (;;) {
					tempr=wr*afc(kk+ks)-wi*afc(kk+ks+1);
					tempi=wi*afc(kk+ks)+wr*afc(kk+ks+1);
					afa(j)=afc(kk)+tempr;
					afb(j)=afc(kk)-tempr;
					afa(++j)=afc(++kk)+tempi;
					afb(j++)=afc(kk++)-tempi;
					if (kk < k) continue;
					kc += kd;
					if (kc == mm) {
						kc=0;
						wr=(wtemp=wr)*wpr-wi*wpi+wr;
						wi=wi*wpr+wtemp*wpi+wi;
					}
					kk += ks;
					if (kk > KBF-1) break;
					else k=kk+ks;
				}
				if (j > KBF-1) {
					cc0=(*file(nc)).tellp()/sizeof(NR_single_precision::DP);
					(*file(nc)).write((char *) &afa(0),KBF*sizeof(NR_single_precision::DP));
					cc=(*file(nc)).tellp()/sizeof(NR_single_precision::DP);
					if ((cc-cc0) != KBF) NR::nrerror("write error 4 in fourfs");
					cc0=(*file(nd)).tellp()/sizeof(NR_single_precision::DP);
					(*file(nd)).write((char *) &afb(0),KBF*sizeof(NR_single_precision::DP));
					cc=(*file(nd)).tellp()/sizeof(NR_single_precision::DP);
					if ((cc-cc0) != KBF) NR::nrerror("write error 5 in fourfs");
					j=0;
				}
			}
			na=mate(na);
		}
		fourew(file,na,nb,nc,nd);
		jk >>= 1;
		if (jk > 1) continue;
		mm=n;
		do {
			if (nv < ndim-1) jk=nn(++nv);
			else return;
		} while (jk == 1);
	}*/
}





void NR_single_precision::fourn(NR_single_precision::Vec_IO_DP &data, NR_single_precision::Vec_I_INT &nn, const int isign)
{
	int idim,i1,i2,i3,i2rev,i3rev,ip1,ip2,ip3,ifp1,ifp2;
	int ibit,k1,k2,n,nprev,nrem,ntot;
	NR_single_precision::DP tempi,tempr,theta,wi,wpi,wpr,wr,wtemp;

	int ndim=nn.size();
	ntot=data.size()/2;
	nprev=1;
	for (idim=ndim-1;idim>=0;idim--) {
		n=nn(idim);
		nrem=ntot/(n*nprev);
		ip1=nprev << 1;
		ip2=ip1*n;
		ip3=ip2*nrem;
		i2rev=0;
		for (i2=0;i2<ip2;i2+=ip1) {
			if (i2 < i2rev) {
				for (i1=i2;i1<i2+ip1-1;i1+=2) {
					for (i3=i1;i3<ip3;i3+=ip2) {
						i3rev=i2rev+i3-i2;
						SWAP(data(i3),data(i3rev));
						SWAP(data(i3+1),data(i3rev+1));
					}
				}
			}
			ibit=ip2 >> 1;
			while (ibit >= ip1 && i2rev+1 > ibit) {
				i2rev -= ibit;
				ibit >>= 1;
			}
			i2rev += ibit;
		}
		ifp1=ip1;
		while (ifp1 < ip2) {
			ifp2=ifp1 << 1;
			theta=isign*6.28318530717959/(ifp2/ip1);
			wtemp=sin(0.5*theta);
			wpr= -2.0*wtemp*wtemp;
			wpi=sin(theta);
			wr=1.0;
			wi=0.0;
			for (i3=0;i3<ifp1;i3+=ip1) {
				for (i1=i3;i1<i3+ip1-1;i1+=2) {
					for (i2=i1;i2<ip3;i2+=ifp2) {
						k1=i2;
						k2=k1+ifp1;
						tempr=wr*data(k2)-wi*data(k2+1);
						tempi=wr*data(k2+1)+wi*data(k2);
						data(k2)=data(k1)-tempr;
						data(k2+1)=data(k1+1)-tempi;
						data(k1) += tempr;
						data(k1+1) += tempi;
					}
				}
				wr=(wtemp=wr)*wpr-wi*wpi+wr;
				wi=wi*wpr+wtemp*wpi+wi;
			}
			ifp1=ifp2;
		}
		nprev *= n;
	}
}



void NR_single_precision::fpoly(const NR_single_precision::DP x, NR_single_precision::Vec_O_DP &p)
{
	int j;

	int np=p.size();
	p(0)=1.0;
	for (j=1;j<np;j++) p(j)=p(j-1)*x;
}



void NR_single_precision::fred2(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_O_DP &t, NR_single_precision::Vec_O_DP &f, NR_single_precision::Vec_O_DP &w,
	NR_single_precision::DP g(const NR_single_precision::DP), NR_single_precision::DP ak(const NR_single_precision::DP, const NR_single_precision::DP))
{
	int i,j;
	NR_single_precision::DP d;

	int n=t.size();
	NR_single_precision::Mat_DP omk(n,n);
	NR_single_precision::Vec_INT indx(n);
	gauleg(a,b,t,w);
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++)
			omk(i,j)=NR_single_precision::DP(i == j)-ak(t(i),t(j))*w(j);
		f(i)=g(t(i));
	}
	ludcmp(omk,indx,d);
	lubksb(omk,indx,f);
}

#include <iostream>
#include <iomanip>



namespace fredx
{
int main(void)	// Program fredex
{
	const int N=40;
	const NR_single_precision::DP PI=3.141592653589793238;
	int j;
	NR_single_precision::DP d,x;
	NR_single_precision::Vec_INT indx(N);
	NR_single_precision::Vec_DP g(N);
	NR_single_precision::Mat_DP a(N,N);

	NR_single_precision::quadmx(a);
	NR_single_precision::ludcmp(a,indx,d);
	for (j=0;j<N;j++)
		g(j)=sin(j*PI/(N-1));
	NR_single_precision::lubksb(a,indx,g);
	for (j=0;j<N;j++) {
		x=j*PI/(N-1);
		ASSERT(0);
//		cout << fixed (j+1);
		cout << x << g(j) << endl;
	}
	return 0;
}
}


NR_single_precision::DP NR_single_precision::fredin(const NR_single_precision::DP x, const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &t, NR_single_precision::Vec_I_DP &f,
	NR_single_precision::Vec_I_DP &w, NR_single_precision::DP g(const NR_single_precision::DP), NR_single_precision::DP ak(const NR_single_precision::DP, const NR_single_precision::DP))
{
	int i;
	NR_single_precision::DP sum=0.0;

	int n=t.size();
	for (i=0;i<n;i++) sum += ak(x,t(i))*w(i)*f(i);
	return g(x)+sum;
}


#include <complex>
#include <limits>



void NR_single_precision::frenel(const NR_single_precision::DP x, complex<NR_single_precision::DP> &cs)
{
	const int MAXIT=100;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN;
	const NR_single_precision::DP BIG=NR_single_precision::MAXIMUM_CONSTANTS*EPS;
	const NR_single_precision::DP PI=3.141592653589793238, PIBY2=(PI/2.0), XMIN=1.5;
	bool odd;
	int k,n;
	NR_single_precision::DP a,ax,fact,pix2,sign,sum,sumc,sums,term,test;
	complex<NR_single_precision::DP> b,cc,d,h,del;

	ax=fabs(x);
	if (ax < sqrt(FPMIN)) {
		cs=ax;
	} else if (ax <= XMIN) {
		sum=sums=0.0;
		sumc=ax;
		sign=1.0;
		fact=PIBY2*ax*ax;
		odd=true;
		term=ax;
		n=3;
		for (k=1;k<=MAXIT;k++) {
			term *= fact/k;
			sum += sign*term/n;
			test=fabs(sum)*EPS;
			if (odd) {
				sign = -sign;
				sums=sum;
				sum=sumc;
			} else {
				sumc=sum;
				sum=sums;
			}
			if (term < test) break;
			odd=!odd;
			n += 2;
		}
		if (k > MAXIT) NR::nrerror("series failed in frenel");
		cs=complex<NR_single_precision::DP>(sumc,sums);
	} else {
		pix2=PI*ax*ax;
		b=complex<NR_single_precision::DP>(1.0,-pix2);
		cc=BIG;
		d=h=1.0/b;
		n = -1;
		for (k=2;k<=MAXIT;k++) {
			n += 2;
			a = -n*(n+1);
			b += 4.0;
			d=1.0/(a*d+b);
			cc=b+a/cc;
			del=cc*d;
			h *= del;
			if (fabs(real(del)-1.0)+fabs(imag(del)) <= EPS) break;
		}
		if (k > MAXIT) NR::nrerror("cf failed in frenel");
		h *= complex<NR_single_precision::DP>(ax,-ax);
		cs=complex<NR_single_precision::DP>(0.5,0.5)
			*(1.0-complex<NR_single_precision::DP>(cos(0.5*pix2),sin(0.5*pix2))*h);
	}
	if (x < 0.0) {
		cs = -cs;
	}
	return;
}





void NR_single_precision::frprmn(NR_single_precision::Vec_IO_DP &p, const NR_single_precision::DP ftol, int &iter, NR_single_precision::DP &fret,
	NR_single_precision::DP func(NR_single_precision::Vec_I_DP &), void dfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const int ITMAX=200;
	const NR_single_precision::DP EPS=1.0e-18;
	int j,its;
	NR_single_precision::DP gg,gam,fp,dgg;

	int n=p.size();
	NR_single_precision::Vec_DP g(n),h(n),xi(n);
	fp=func(p);
	dfunc(p,xi);
	for (j=0;j<n;j++) {
		g(j) = -xi(j);
		xi(j)=h(j)=g(j);
	}
	for (its=0;its<ITMAX;its++) {
		iter=its;
		linmin(p,xi,fret,func);
		if (2.0*fabs(fret-fp) <= ftol*(fabs(fret)+fabs(fp)+EPS))
			return;
		fp=fret;
		dfunc(p,xi);
		dgg=gg=0.0;
		for (j=0;j<n;j++) {
			gg += g(j)*g(j);
//		  dgg += xi(j)*xi(j);
			dgg += (xi(j)+g(j))*xi(j);
		}
		if (gg == 0.0)
			return;
		gam=dgg/gg;
		for (j=0;j<n;j++) {
			g(j) = -xi(j);
			xi(j)=h(j)=g(j)+gam*h(j);
		}
	}
	NR::nrerror("Too many iterations in frprmn");
}



void NR_single_precision::ftest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &f, NR_single_precision::DP &prob)
{
	NR_single_precision::DP var1,var2,ave1,ave2,df1,df2;

	int n1=data1.size();
	int n2=data2.size();
	avevar(data1,ave1,var1);
	avevar(data2,ave2,var2);
	if (var1 > var2) {
		f=var1/var2;
		df1=n1-1;
		df2=n2-1;
	} else {
		f=var2/var1;
		df1=n2-1;
		df2=n1-1;
	}
	prob = 2.0*betai(0.5*df2,0.5*df1,df2/(df2+df1*f));
	if (prob > 1.0) prob=2.0-prob;
}





NR_single_precision::DP NR_single_precision::gamdev(const int ia, int &idum)
{
	int j;
	NR_single_precision::DP am,e,s,v1,v2,x,y;

	if (ia < 1) NR::nrerror("Error in routine gamdev");
	if (ia < 6) {
		x=1.0;
		for (j=1;j<=ia;j++) x *= ran1(idum);
		x = -log(x);
	} else {
		do {
			do {
				do {
					v1=ran1(idum);
					v2=2.0*ran1(idum)-1.0;
				} while (v1*v1+v2*v2 > 1.0);
				y=v2/v1;
				am=ia-1;
				s=sqrt(2.0*am+1.0);
				x=s*y+am;
			} while (x <= 0.0);
			e=(1.0+y*y)*exp(am*log(x/am)-s*y);
		} while (ran1(idum) > e);
	}
	return x;
}





NR_single_precision::DP NR_single_precision::gammln(const NR_single_precision::DP xx)
{
	int j;
	NR_single_precision::DP x,y,tmp,ser;
	static const NR_single_precision::DP cof[6]={76.18009172947146,-86.50532032941677,
		24.01409824083091,-1.231739572450155,0.1208650973866179e-2,
		-0.5395239384953e-5};

	y=x=xx;
	tmp=x+5.5;
	tmp -= (x+0.5)*log(tmp);
	ser=1.000000000190015;
	for (j=0;j<6;j++) ser += cof[j]/++y;
	return -tmp+log(2.5066282746310005*ser/x);
}



NR_single_precision::DP NR_single_precision::gammp(const NR_single_precision::DP a, const NR_single_precision::DP x)
{
	NR_single_precision::DP gamser,gammcf,gln;

	if (x < 0.0 || a <= 0.0)
		NR::nrerror("Invalid arguments in routine gammp");
	if (x < a+1.0) {
		gser(gamser,a,x,gln);
		return gamser;
	} else {
		gcf(gammcf,a,x,gln);
		return 1.0-gammcf;
	}
}



NR_single_precision::DP NR_single_precision::gammq(const NR_single_precision::DP a, const NR_single_precision::DP x)
{
	NR_single_precision::DP gamser,gammcf,gln;

	if (x < 0.0 || a <= 0.0)
		NR::nrerror("Invalid arguments in routine gammq");
	if (x < a+1.0) {
		gser(gamser,a,x,gln);
		return 1.0-gamser;
	} else {
		gcf(gammcf,a,x,gln);
		return gammcf;
	}
}





NR_single_precision::DP NR_single_precision::gasdev(int &idum)
{
	static int iset=0;
	static NR_single_precision::DP gset;
	NR_single_precision::DP fac,rsq,v1,v2;

	if (idum < 0) iset=0;
	if (iset == 0) {
		do {
			v1=2.0*ran1(idum)-1.0;
			v2=2.0*ran1(idum)-1.0;
			rsq=v1*v1+v2*v2;
		} while (rsq >= 1.0 || rsq == 0.0);
		fac=sqrt(-2.0*log(rsq)/rsq);
		gset=v1*fac;
		iset=1;
		return v2*fac;
	} else {
		iset=0;
		return gset;
	}
}





void NR_single_precision::gaucof(NR_single_precision::Vec_IO_DP &a, NR_single_precision::Vec_IO_DP &b, const NR_single_precision::DP amu0, NR_single_precision::Vec_O_DP &x,
	NR_single_precision::Vec_O_DP &w)
{
	int i,j;

	int n=a.size();
	NR_single_precision::Mat_DP z(n,n);
	for (i=0;i<n;i++) {
		if (i != 0) b(i)=sqrt(b(i));
		for (j=0;j<n;j++) z(i,j)=NR_single_precision::DP(i == j);
	}
	tqli(a,b,z);
	eigsrt(a,z);
	for (i=0;i<n;i++) {
		x(i)=a(i);
		w(i)=amu0*z(0,i)*z(0,i);
	}
}





void NR_single_precision::gauher(NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w)
{
	const NR_single_precision::DP EPS=1.0e-14,PIM4=0.7511255444649425;
	const int MAXIT=10;
	int i,its,j,m;
	NR_single_precision::DP p1,p2,p3,pp,z,z1;

	int n=x.size();
	m=(n+1)/2;
	for (i=0;i<m;i++) {
		if (i == 0) {
			z=sqrt(NR_single_precision::DP(2*n+1))-1.85575*pow(NR_single_precision::DP(2*n+1),-0.16667);
		} else if (i == 1) {
			z -= 1.14*pow(NR_single_precision::DP(n),0.426)/z;
		} else if (i == 2) {
			z=1.86*z-0.86*x(0);
		} else if (i == 3) {
			z=1.91*z-0.91*x(1);
		} else {
			z=2.0*z-x(i-2);
		}
		for (its=0;its<MAXIT;its++) {
			p1=PIM4;
			p2=0.0;
			for (j=0;j<n;j++) {
				p3=p2;
				p2=p1;
				p1=z*sqrt(2.0/(j+1))*p2-sqrt(NR_single_precision::DP(j)/(j+1))*p3;
			}
			pp=sqrt(NR_single_precision::DP(2*n))*p2;
			z1=z;
			z=z1-p1/pp;
			if (fabs(z-z1) <= EPS) break;
		}
		if (its >= MAXIT) NR::nrerror("too many iterations in gauher");
		x(i)=z;
		x(n-1-i) = -z;
		w(i)=2.0/(pp*pp);
		w(n-1-i)=w(i);
	}
}





void NR_single_precision::gaujac(NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP alf, const NR_single_precision::DP bet)
{
	const int MAXIT=10;
	const NR_single_precision::DP EPS=1.0e-14;
	int i,its,j;
	NR_single_precision::DP alfbet,an,bn,r1,r2,r3;
	NR_single_precision::DP a,b,c,p1,p2,p3,pp,temp,z,z1;

	int n=x.size();
	for (i=0;i<n;i++) {
		if (i == 0) {
			an=alf/n;
			bn=bet/n;
			r1=(1.0+alf)*(2.78/(4.0+n*n)+0.768*an/n);
			r2=1.0+1.48*an+0.96*bn+0.452*an*an+0.83*an*bn;
			z=1.0-r1/r2;
		} else if (i == 1) {
			r1=(4.1+alf)/((1.0+alf)*(1.0+0.156*alf));
			r2=1.0+0.06*(n-8.0)*(1.0+0.12*alf)/n;
			r3=1.0+0.012*bet*(1.0+0.25*fabs(alf))/n;
			z -= (1.0-z)*r1*r2*r3;
		} else if (i == 2) {
			r1=(1.67+0.28*alf)/(1.0+0.37*alf);
			r2=1.0+0.22*(n-8.0)/n;
			r3=1.0+8.0*bet/((6.28+bet)*n*n);
			z -= (x(0)-z)*r1*r2*r3;
		} else if (i == n-2) {
			r1=(1.0+0.235*bet)/(0.766+0.119*bet);
			r2=1.0/(1.0+0.639*(n-4.0)/(1.0+0.71*(n-4.0)));
			r3=1.0/(1.0+20.0*alf/((7.5+alf)*n*n));
			z += (z-x(n-4))*r1*r2*r3;
		} else if (i == n-1) {
			r1=(1.0+0.37*bet)/(1.67+0.28*bet);
			r2=1.0/(1.0+0.22*(n-8.0)/n);
			r3=1.0/(1.0+8.0*alf/((6.28+alf)*n*n));
			z += (z-x(n-3))*r1*r2*r3;
		} else {
			z=3.0*x(i-1)-3.0*x(i-2)+x(i-3);
		}
		alfbet=alf+bet;
		for (its=1;its<=MAXIT;its++) {
			temp=2.0+alfbet;
			p1=(alf-bet+temp*z)/2.0;
			p2=1.0;
			for (j=2;j<=n;j++) {
				p3=p2;
				p2=p1;
				temp=2*j+alfbet;
				a=2*j*(j+alfbet)*(temp-2.0);
				b=(temp-1.0)*(alf*alf-bet*bet+temp*(temp-2.0)*z);
				c=2.0*(j-1+alf)*(j-1+bet)*temp;
				p1=(b*p2-c*p3)/a;
			}
			pp=(n*(alf-bet-temp*z)*p1+2.0*(n+alf)*(n+bet)*p2)/(temp*(1.0-z*z));
			z1=z;
			z=z1-p1/pp;
			if (fabs(z-z1) <= EPS) break;
		}
		if (its > MAXIT) NR::nrerror("too many iterations in gaujac");
		x(i)=z;
		w(i)=exp(gammln(alf+n)+gammln(bet+n)-gammln(n+1.0)-
			gammln(n+alfbet+1.0))*temp*pow(2.0,alfbet)/(pp*p2);
	}
}





void NR_single_precision::gaulag(NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP alf)
{
	const int MAXIT=10;
	const NR_single_precision::DP EPS=1.0e-14;
	int i,its,j;
	NR_single_precision::DP ai,p1,p2,p3,pp,z,z1;

	int n=x.size();
	for (i=0;i<n;i++) {
		if (i == 0) {
			z=(1.0+alf)*(3.0+0.92*alf)/(1.0+2.4*n+1.8*alf);
		} else if (i == 1) {
			z += (15.0+6.25*alf)/(1.0+0.9*alf+2.5*n);
		} else {
			ai=i-1;
			z += ((1.0+2.55*ai)/(1.9*ai)+1.26*ai*alf/
				(1.0+3.5*ai))*(z-x(i-2))/(1.0+0.3*alf);
		}
		for (its=0;its<MAXIT;its++) {
			p1=1.0;
			p2=0.0;
			for (j=0;j<n;j++) {
				p3=p2;
				p2=p1;
				p1=((2*j+1+alf-z)*p2-(j+alf)*p3)/(j+1);
			}
			pp=(n*p1-(n+alf)*p2)/z;
			z1=z;
			z=z1-p1/pp;
			if (fabs(z-z1) <= EPS) break;
		}
		if (its >= MAXIT) NR::nrerror("too many iterations in gaulag");
		x(i)=z;
		w(i) = -exp(gammln(alf+n)-gammln(NR_single_precision::DP(n)))/(pp*n*p2);
	}
}





void NR_single_precision::gauleg(const NR_single_precision::DP x1, const NR_single_precision::DP x2, NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w)
{
	const NR_single_precision::DP EPS=1.0e-14;
	int m,j,i;
	NR_single_precision::DP z1,z,xm,xl,pp,p3,p2,p1;

	int n=x.size();
	m=(n+1)/2;
	xm=0.5*(x2+x1);
	xl=0.5*(x2-x1);
	for (i=0;i<m;i++) {
		z=cos(3.141592654*(i+0.75)/(n+0.5));
		do {
			p1=1.0;
			p2=0.0;
			for (j=0;j<n;j++) {
				p3=p2;
				p2=p1;
				p1=((2.0*j+1.0)*z*p2-j*p3)/(j+1);
			}
			pp=n*(z*p1-p2)/(z*z-1.0);
			z1=z;
			z=z1-p1/pp;
		} while (fabs(z-z1) > EPS);
		x(i)=xm-xl*z;
		x(n-1-i)=xm+xl*z;
		w(i)=2.0*xl/((1.0-z*z)*pp*pp);
		w(n-1-i)=w(i);
	}
}





void NR_single_precision::gaussj(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Mat_IO_DP &b)
{
	int i,icol,irow,j,k,l,ll;
	NR_single_precision::DP big,dum,pivinv;

	int n=a.rows();
	int m=b.cols();
	NR_single_precision::Vec_INT indxc(n),indxr(n),ipiv(n);
	for (j=0;j<n;j++) ipiv(j)=0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if (ipiv(j) != 1)
				for (k=0;k<n;k++) {
					if (ipiv(k) == 0) {
						if (fabs(a(j,k)) >= big) {
							big=fabs(a(j,k));
							irow=j;
							icol=k;
						}
					}
				}
		++(ipiv(icol));
		if (irow != icol) {
			for (l=0;l<n;l++) SWAP(a(irow,l),a(icol,l));
			for (l=0;l<m;l++) SWAP(b(irow,l),b(icol,l));
		}
		indxr(i)=irow;
		indxc(i)=icol;
		if (a(icol,icol) == 0.0) NR::nrerror("gaussj: Singular Matrix");
		pivinv=1.0/a(icol,icol);
		a(icol,icol)=1.0;
		for (l=0;l<n;l++) a(icol,l) *= pivinv;
		for (l=0;l<m;l++) b(icol,l) *= pivinv;
		for (ll=0;ll<n;ll++)
			if (ll != icol) {
				dum=a(ll,icol);
				a(ll,icol)=0.0;
				for (l=0;l<n;l++) a(ll,l) -= a(icol,l)*dum;
				for (l=0;l<m;l++) b(ll,l) -= b(icol,l)*dum;
			}
	}
	for (l=n-1;l>=0;l--) {
		if (indxr(l) != indxc(l))
			for (k=0;k<n;k++)
				SWAP(a(k,indxr(l)),a(k,indxc(l)));
	}
}


#include <limits>



void NR_single_precision::gcf(NR_single_precision::DP &gammcf, const NR_single_precision::DP a, const NR_single_precision::DP x, NR_single_precision::DP &gln)
{
	const int ITMAX=100;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	const NR_single_precision::DP FPMIN=DBL_MIN/EPS;
	int i;
	NR_single_precision::DP an,b,c,d,del,h;

	gln=gammln(a);
	b=x+1.0-a;
	c=1.0/FPMIN;
	d=1.0/b;
	h=d;
	for (i=1;i<=ITMAX;i++) {
		an = -i*(i-a);
		b += 2.0;
		d=an*d+b;
		if (fabs(d) < FPMIN) d=FPMIN;
		c=b+an/c;
		if (fabs(c) < FPMIN) c=FPMIN;
		d=1.0/d;
		del=d*c;
		h *= del;
		if (fabs(del-1.0) <= EPS) break;
	}
	if (i > ITMAX) NR::nrerror("a too large, ITMAX too small in gcf");
	gammcf=exp(-x+a*log(x)-gln)*h;
}






NR_single_precision::DP NR_single_precision::golden(const NR_single_precision::DP ax, const NR_single_precision::DP bx, const NR_single_precision::DP cx, NR_single_precision::DP f(const NR_single_precision::DP),
	const NR_single_precision::DP tol, NR_single_precision::DP &xmin)
{
	const NR_single_precision::DP R=0.61803399,C=1.0-R;
	NR_single_precision::DP f1,f2,x0,x1,x2,x3;

	x0=ax;
	x3=cx;
	if (fabs(cx-bx) > fabs(bx-ax)) {
		x1=bx;
		x2=bx+C*(cx-bx);
	} else {
		x2=bx;
		x1=bx-C*(bx-ax);
	}
	f1=f(x1);
	f2=f(x2);
	while (fabs(x3-x0) > tol*(fabs(x1)+fabs(x2))) {
		if (f2 < f1) {
			shft3(x0,x1,x2,R*x2+C*x3);
			shft2(f1,f2,f(x2));
		} else {
			shft3(x3,x2,x1,R*x1+C*x0);
			shft2(f2,f1,f(x1));
		}
	}
	if (f1 < f2) {
		xmin=x1;
		return f1;
	} else {
		xmin=x2;
		return f2;
	}
}


#include <limits>



void NR_single_precision::gser(NR_single_precision::DP &gamser, const NR_single_precision::DP a, const NR_single_precision::DP x, NR_single_precision::DP &gln)
{
	const int ITMAX=100;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	int n;
	NR_single_precision::DP sum,del,ap;

	gln=gammln(a);
	if (x <= 0.0) {
		if (x < 0.0) NR::nrerror("x less than 0 in routine gser");
		gamser=0.0;
		return;
	} else {
		ap=a;
		del=sum=1.0/a;
		for (n=0;n<ITMAX;n++) {
			++ap;
			del *= x/ap;
			sum += del;
			if (fabs(del) < fabs(sum)*EPS) {
				gamser=sum*exp(-x+a*log(x)-gln);
				return;
			}
		}
		NR::nrerror("a too large, ITMAX too small in routine gser");
		return;
	}
}



void NR_single_precision::hpsel(NR_single_precision::Vec_I_DP &arr, NR_single_precision::Vec_O_DP &heap)
{
	int i,j,k;

	int m=heap.size();
	int n=arr.size();
	if (m > n/2 || m < 1) NR::nrerror("probable misuse of hpsel");
	for (i=0;i<m;i++) heap(i)=arr(i);
	sort(heap);
	for (i=m;i<n;i++) {
		if (arr(i) > heap(0)) {
			heap(0)=arr(i);
			for (j=0;;) {
				k=(j << 1)+1;
				if (k > m-1) break;
				if (k != (m-1) && heap(k) > heap(k+1)) k++;
				if (heap(j) <= heap(k)) break;
				SWAP(heap(k),heap(j));
				j=k;
			}
		}
	}
}



namespace {
	void sift_down(NR_single_precision::Vec_IO_DP &ra, const int l, const int r)
	{
		int j,jold;
		NR_single_precision::DP a;

		a=ra(l);
		jold=l;
		j=l+1;
		while (j <= r) {
			if (j < r && ra(j) < ra(j+1)) j++;
			if (a >= ra(j)) break;
			ra(jold)=ra(j);
			jold=j;
			j=2*j+1;
		}
		ra(jold)=a;
	}
}

void NR_single_precision::hpsort(NR_single_precision::Vec_IO_DP &ra)
{
	int i;

	int n=ra.size();
	for (i=n/2-1; i>=0; i--)
		sift_down(ra,i,n-1);
	for (i=n-1; i>0; i--) {
		SWAP(ra(0),ra(i));
		sift_down(ra,0,i-1);
	}
}


#include <complex>



void NR_single_precision::hqr(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_CPLX_DP &wri)
{
	int nn,m,l,k,j,its,i,mmin;
	NR_single_precision::DP z,y,x,w,v,u,t,s,r,q,p,anorm;

	int n=a.rows();
	anorm=0.0;
	for (i=0;i<n;i++)
		for (j=MAX(i-1,0);j<n;j++)
			anorm += fabs(a(i,j));
	nn=n-1;
	t=0.0;
	while (nn >= 0) {
		its=0;
		do {
			for (l=nn;l>0;l--) {
				s=fabs(a(l-1,l-1))+fabs(a(l,l));
				if (s == 0.0) s=anorm;
				if (fabs(a(l,l-1)) + s == s) {
					a(l,l-1) = 0.0;
					break;
				}
			}
			x=a(nn,nn);
			if (l == nn) {
				wri(nn--)=x+t;
			} else {
				y=a(nn-1,nn-1);
				w=a(nn,nn-1)*a(nn-1,nn);
				if (l == nn-1) {
					p=0.5*(y-x);
					q=p*p+w;
					z=sqrt(fabs(q));
					x += t;
					if (q >= 0.0) {
						z=p+SIGN(z,p);
						wri(nn-1)=wri(nn)=x+z;
						if (z != 0.0) wri(nn)=x-w/z;
					} else {
						wri(nn)=complex<NR_single_precision::DP>(x+p,z);
						wri(nn-1)=conj(wri(nn));
					}
					nn -= 2;
				} else {
					if (its == 30) NR::nrerror("Too many iterations in hqr");
					if (its == 10 || its == 20) {
						t += x;
						for (i=0;i<nn+1;i++) a(i,i) -= x;
						s=fabs(a(nn,nn-1))+fabs(a(nn-1,nn-2));
						y=x=0.75*s;
						w = -0.4375*s*s;
					}
					++its;
					for (m=nn-2;m>=l;m--) {
						z=a(m,m);
						r=x-z;
						s=y-z;
						p=(r*s-w)/a(m+1,m)+a(m,m+1);
						q=a(m+1,m+1)-z-r-s;
						r=a(m+2,m+1);
						s=fabs(p)+fabs(q)+fabs(r);
						p /= s;
						q /= s;
						r /= s;
						if (m == l) break;
						u=fabs(a(m,m-1))*(fabs(q)+fabs(r));
						v=fabs(p)*(fabs(a(m-1,m-1))+fabs(z)+fabs(a(m+1,m+1)));
						if (u+v == v) break;
					}
					for (i=m;i<nn-1;i++) {
						a(i+2,i)=0.0;
						if (i != m) a(i+2,i-1)=0.0;
					}
					for (k=m;k<nn;k++) {
						if (k != m) {
							p=a(k,k-1);
							q=a(k+1,k-1);
							r=0.0;
							if (k+1 != nn) r=a(k+2,k-1);
							if ((x=fabs(p)+fabs(q)+fabs(r)) != 0.0) {
								p /= x;
								q /= x;
								r /= x;
							}
						}
						if ((s=SIGN(sqrt(p*p+q*q+r*r),p)) != 0.0) {
							if (k == m) {
								if (l != m)
								a(k,k-1) = -a(k,k-1);
							} else
								a(k,k-1) = -s*x;
							p += s;
							x=p/s;
							y=q/s;
							z=r/s;
							q /= p;
							r /= p;
							// row modification
							for (j=k;j<nn+1;j++) {
								p=a(k,j)+q*a(k+1,j);
								if (k+1 != nn) {
									p += r*a(k+2,j);
									a(k+2,j) -= p*z;
								}
								a(k+1,j) -= p*y;
								a(k,j) -= p*x;
							}
							mmin = nn < k+3 ? nn : k+3;
							// column moodification
							for (i=l;i<mmin+1;i++) {
								p=x*a(i,k)+y*a(i,k+1);
								if (k != (nn-1)) {
									p += z*a(i,k+2);
									a(i,k+2) -= p*r;
								}
								a(i,k+1) -= p*q;
								a(i,k) -= p;
							}
						}
					}
				}
			}
		} while (l+1 < nn);
	}
}



void NR_single_precision::hufapp(NR_single_precision::Vec_IO_ULNG &index, NR_single_precision::Vec_I_ULNG &nprob, const unsigned long n,
	const unsigned long m)
{
	unsigned long i=m,j,k;

	k=index(i);
	while (i < (n >> 1)) {
		if ((j = 2*i+1) < n-1
			&& nprob(index(j)) > nprob(index(j+1))) j++;
		if (nprob(k) <= nprob(index(j))) break;
		index(i)=index(j);
		i=j;
	}
	index(i)=k;
}

#include <string>



void NR_single_precision::hufdec(unsigned long &ich, string &code, const unsigned long lcode,
	unsigned long &nb, huffcode &hcode)
{
	Msg::error("hufdec");
	/*
	unsigned long nc;
	static unsigned char setbit[8]={0x1,0x2,0x4,0x8,0x10,0x20,0x40,0x80};

	int node=hcode.nodemax-1;
	for (;;) {
		nc=nb >> 3;
		if (nc >= lcode) {
			ich=hcode.nch;
			return;
		}
		node=((code(nc) & setbit[7 & nb++]) != 0 ?
			hcode.right(node) : hcode.left(node));
		if (node < hcode.nch) {
			ich=node;
			return;
		}
	}*/
}

#include <string>



void NR_single_precision::hufenc(const unsigned long ich, string &code, unsigned long &nb,
	huffcode &hcode)
{
	Msg::error("hufenc");
	/*
	int m,n;
	unsigned long k,nc;
	static unsigned long setbit(32)={0x1L,0x2L,0x4L,0x8L,0x10L,0x20L,
		0x40L,0x80L,0x100L,0x200L,0x400L,0x800L,0x1000L,0x2000L,
		0x4000L,0x8000L,0x10000L,0x20000L,0x40000L,0x80000L,0x100000L,
		0x200000L,0x400000L,0x800000L,0x1000000L,0x2000000L,0x4000000L,
		0x8000000L,0x10000000L,0x20000000L,0x40000000L,0x80000000L};

	k=ich;
	if (k >= hcode.nch)
		NR::nrerror("ich out of range in hufenc.");
	for (n=hcode.ncod(k)-1;n >= 0;n--,++nb) {
		nc=nb >> 3;
		if (code.length() < nc+1)
			code.resize(2*(nc+1));
		m=nb & 7;
		if (m == 0) code(nc)=0;
		if ((hcode.icod(k) & setbit(n)) != 0) code(nc) |= setbit(m);
	}*/
}



void NR_single_precision::hufmak(NR_single_precision::Vec_I_ULNG &nfreq, const unsigned long nchin,
	unsigned long &ilong, unsigned long &nlong, huffcode &hcode)
{
	Msg::error("hufmak");
	/*
	int ibit,j,node;
	unsigned long k,n,nused;
	static unsigned long setbit(32)={0x1L,0x2L,0x4L,0x8L,0x10L,0x20L,
		0x40L,0x80L,0x100L,0x200L,0x400L,0x800L,0x1000L,0x2000L,
		0x4000L,0x8000L,0x10000L,0x20000L,0x40000L,0x80000L,0x100000L,
		0x200000L,0x400000L,0x800000L,0x1000000L,0x2000000L,0x4000000L,
		0x8000000L,0x10000000L,0x20000000L,0x40000000L,0x80000000L};

	hcode.nch=nchin;
	Vec_ULNG index(2*hcode.nch-1);
	Vec_ULNG nprob(2*hcode.nch-1);
	NR_single_precision::Vec_INT up(2*hcode.nch-1);
	for (nused=0,j=0;j<hcode.nch;j++) {
		nprob(j)=nfreq(j);
		hcode.icod(j)=hcode.ncod(j)=0;
		if (nfreq(j) != 0) index(nused++)=j;
	}
	for (j=nused-1;j>=0;j--)
		hufapp(index,nprob,nused,j);
	k=hcode.nch;
	while (nused > 1) {
		node=index(0);
		index(0)=index((nused--)-1);
		hufapp(index,nprob,nused,0);
		nprob(k)=nprob(index(0))+nprob(node);
		hcode.left(k)=node;
		hcode.right(k++)=index(0);
		up(index(0)) = -int(k);
		index(0)=k-1;
		up(node)=k;
		hufapp(index,nprob,nused,0);
	}
	up((hcode.nodemax=k)-1)=0;
	for (j=0;j<hcode.nch;j++) {
		if (nprob(j) != 0) {
			for (n=0,ibit=0,node=up(j);node;node=up(node-1),ibit++) {
				if (node < 0) {
					n |= setbit(ibit);
					node = -node;
				}
			}
			hcode.icod(j)=n;
			hcode.ncod(j)=ibit;
		}
	}
	nlong=0;
	for (j=0;j<hcode.nch;j++) {
		if (hcode.ncod(j) > nlong) {
			nlong=hcode.ncod(j);
			ilong=j;
		}
	}*/
}



void NR_single_precision::hunt(NR_single_precision::Vec_I_DP &xx, const NR_single_precision::DP x, int &jlo)
{
	int jm,jhi,inc;
	bool ascnd;

	int n=xx.size();
	ascnd=(xx(n-1) >= xx(0));
	if (jlo < 0 || jlo > n-1) {
		jlo=-1;
		jhi=n;
	} else {
		inc=1;
		if (x >= xx(jlo) == ascnd) {
			if (jlo == n-1) return;
			jhi=jlo+1;
			while (x >= xx(jhi) == ascnd) {
				jlo=jhi;
				inc += inc;
				jhi=jlo+inc;
				if (jhi > n-1) {
					jhi=n;
					break;
				}
			}
		} else {
			if (jlo == 0) {
				jlo=-1;
				return;
			}
			jhi=jlo--;
			while (x < xx(jlo) == ascnd) {
				jhi=jlo;
				inc <<= 1;
				if (inc >= jhi) {
					jlo=-1;
					break;
				}
				else jlo=jhi-inc;
			}
		}
	}
	while (jhi-jlo != 1) {
		jm=(jhi+jlo) >> 1;
		if (x >= xx(jm) == ascnd)
			jlo=jm;
		else
			jhi=jm;
	}
	if (x == xx(n-1)) jlo=n-2;
	if (x == xx(0)) jlo=0;
}






#include <complex>



void NR_single_precision::hypser(const complex<NR_single_precision::DP> &a, const complex<NR_single_precision::DP> &b,
	const complex<NR_single_precision::DP> &c, const complex<NR_single_precision::DP> &z,
	complex<NR_single_precision::DP> &series, complex<NR_single_precision::DP> &deriv)
{
	int n;
	complex<NR_single_precision::DP> aa,bb,cc,fac,temp;

	deriv=0.0;
	fac=1.0;
	temp=fac;
	aa=a;
	bb=b;
	cc=c;
	for (n=1;n<=1000;n++) {
		fac *= ((aa*bb)/cc);
		deriv += fac;
		fac *= ((1.0/n)*z);
		series=temp+fac;
		if (series == temp) return;
		temp=series;
		aa += 1.0;
		bb += 1.0;
		cc += 1.0;
	}
	NR::nrerror("convergence failure in hypser");
}


namespace NR_CUS{
	inline unsigned char lobyte(const unsigned short x)
	{
		return (unsigned char)((x) & 0xff);
	}

	inline unsigned char hibyte(const unsigned short x)
	{
		return (unsigned char)((x >> 8) & 0xff);
	}
}

unsigned short NR_single_precision::icrc(const unsigned short crc, const string &bufptr,
	const short jinit, const int jrev)
{
	Msg::error("icrc");
	/*
	static unsigned short icrctb(256),init=0;
	static unsigned char rchr(256);
	unsigned short j,cword=crc;
	static unsigned char it(16)={0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};

	unsigned long len=bufptr.length();
	if (init == 0) {
		init=1;
		for (j=0;j<256;j++) {
			icrctb(j)=icrc1(j << 8,0);
			rchr(j)=(unsigned char)((it(j & 0xf) << 4) | (it(j >> 4)));
		}
	}
	if (jinit >= 0)
		cword=(jinit | (jinit << 8));
	else if (jrev < 0)
		cword=(rchr(NR_CUS::hibyte(cword)) | (rchr(NR_CUS::lobyte(cword)) << 8));
	for (j=0;j<len;j++) {
		cword=icrctb((jrev < 0 ? rchr((unsigned char) bufptr(j)) :
			(unsigned char) bufptr(j)) ^ NR_CUS::hibyte(cword)) ^ (NR_CUS::lobyte(cword) << 8);
	}
	return (jrev >= 0 ? cword :
		rchr(NR_CUS::hibyte(cword)) | (rchr(NR_CUS::lobyte(cword)) << 8));*/
	return 0;
}


unsigned short NR_single_precision::icrc1(const unsigned short crc, const unsigned char onech)
{
	int i;
	unsigned short ans=(crc ^ onech << 8);

	for (i=0;i<8;i++) {
		if (ans & 0x8000)
			ans = (ans <<= 1) ^ 4129;
		else
			ans <<= 1;
	}
	return ans;
}


unsigned long NR_single_precision::igray(const unsigned long n, const int is)
{
	int ish;
	unsigned long ans,idiv;

	if (is >= 0)
		return n ^ (n >> 1);
	ish=1;
	ans=n;
	for (;;) {
		ans ^= (idiv=ans >> ish);
		if (idiv <= 1 || ish == 16) return ans;
		ish <<= 1;
	}
}


void NR_single_precision::indexx(NR_single_precision::Vec_I_DP &arr, NR_single_precision::Vec_O_INT &indx)
{
	const int M=7,NSTACK=50;
	int i,indxt,ir,j,k,jstack=-1,l=0;
	NR_single_precision::DP a;
	NR_single_precision::Vec_INT istack(NSTACK);

	int n=arr.size();
	ir=n-1;
	for (j=0;j<n;j++) indx(j)=j;
	for (;;) {
		if (ir-l < M) {
			for (j=l+1;j<=ir;j++) {
				indxt=indx(j);
				a=arr(indxt);
				for (i=j-1;i>=l;i--) {
					if (arr(indx(i)) <= a) break;
					indx(i+1)=indx(i);
				}
				indx(i+1)=indxt;
			}
			if (jstack < 0) break;
			ir=istack(jstack--);
			l=istack(jstack--);
		} else {
			k=(l+ir) >> 1;
			SWAP(indx(k),indx(l+1));
			if (arr(indx(l)) > arr(indx(ir))) {
				SWAP(indx(l),indx(ir));
			}
			if (arr(indx(l+1)) > arr(indx(ir))) {
				SWAP(indx(l+1),indx(ir));
			}
			if (arr(indx(l)) > arr(indx(l+1))) {
				SWAP(indx(l),indx(l+1));
			}
			i=l+1;
			j=ir;
			indxt=indx(l+1);
			a=arr(indxt);
			for (;;) {
				do i++; while (arr(indx(i)) < a);
				do j--; while (arr(indx(j)) > a);
				if (j < i) break;
				SWAP(indx(i),indx(j));
			}
			indx(l+1)=indx(j);
			indx(j)=indxt;
			jstack += 2;
			if (jstack >= NSTACK) NR::nrerror("NSTACK too small in indexx.");
			if (ir-i+1 >= j-l) {
				istack(jstack)=ir;
				istack(jstack-1)=i;
				ir=j-1;
			} else {
				istack(jstack)=j-1;
				istack(jstack-1)=l;
				l=i;
			}
		}
	}
}

void NR_single_precision::indexx(NR_single_precision::Vec_I_INT &arr, NR_single_precision::Vec_O_INT &indx)
{
	const int M=7,NSTACK=50;
	int i,indxt,ir,j,k,jstack=-1,l=0;
	int a;
	NR_single_precision::Vec_INT istack(NSTACK);

	int n=arr.size();
	ir=n-1;
	for (j=0;j<n;j++) indx(j)=j;
	for (;;) {
		if (ir-l < M) {
			for (j=l+1;j<=ir;j++) {
				indxt=indx(j);
				a=arr(indxt);
				for (i=j-1;i>=l;i--) {
					if (arr(indx(i)) <= a) break;
					indx(i+1)=indx(i);
				}
				indx(i+1)=indxt;
			}
			if (jstack < 0) break;
			ir=istack(jstack--);
			l=istack(jstack--);
		} else {
			k=(l+ir) >> 1;
			SWAP(indx(k),indx(l+1));
			if (arr(indx(l)) > arr(indx(ir))) {
				SWAP(indx(l),indx(ir));
			}
			if (arr(indx(l+1)) > arr(indx(ir))) {
				SWAP(indx(l+1),indx(ir));
			}
			if (arr(indx(l)) > arr(indx(l+1))) {
				SWAP(indx(l),indx(l+1));
			}
			i=l+1;
			j=ir;
			indxt=indx(l+1);
			a=arr(indxt);
			for (;;) {
				do i++; while (arr(indx(i)) < a);
				do j--; while (arr(indx(j)) > a);
				if (j < i) break;
				SWAP(indx(i),indx(j));
			}
			indx(l+1)=indx(j);
			indx(j)=indxt;
			jstack += 2;
			if (jstack >= NSTACK) NR::nrerror("NSTACK too small in indexx.");
			if (ir-i+1 >= j-l) {
				istack(jstack)=ir;
				istack(jstack-1)=i;
				ir=j-1;
			} else {
				istack(jstack)=j-1;
				istack(jstack-1)=l;
				l=i;
			}
		}
	}
}


void NR_single_precision::interp(NR_single_precision::Mat_O_DP &uf, NR_single_precision::Mat_I_DP &uc)
{
	int ic,iif,jc,jf,nc;

	int nf=uf.rows();
	nc=nf/2+1;
	for (jc=0;jc<nc;jc++)
		for (ic=0;ic<nc;ic++) uf(2*ic,2*jc)=uc(ic,jc);
	for (jf=0;jf<nf;jf+=2)
		for (iif=1;iif<nf-1;iif+=2)
			uf(iif,jf)=0.5*(uf(iif+1,jf)+uf(iif-1,jf));
	for (jf=1;jf<nf-1;jf+=2)
		for (iif=0;iif<nf;iif++)
			uf(iif,jf)=0.5*(uf(iif,jf+1)+uf(iif,jf-1));
}


int NR_single_precision::irbit1(unsigned long &iseed)
{
	unsigned long newbit;

	newbit =  ((iseed >> 17) & 1)
		^ ((iseed >> 4) & 1)
		^ ((iseed >> 1) & 1)
		^ (iseed & 1);
	iseed=(iseed << 1) | newbit;
	return int(newbit);
}


int NR_single_precision::irbit2(unsigned long &iseed)
{
	const unsigned long IB1=1,IB2=2,IB5=16,IB18=131072;
	const unsigned long MASK=IB1+IB2+IB5;

	if (iseed & IB18) {
		iseed=((iseed ^ MASK) << 1) | IB1;
		return 1;
	} else {
		iseed <<= 1;
		return 0;
	}
}




namespace {
	inline void rot(NR_single_precision::Mat_IO_DP &a, const NR_single_precision::DP s, const NR_single_precision::DP tau, const int i,
		const int j, const int k, const int l)
	{
		NR_single_precision::DP g,h;

		g=a(i,j);
		h=a(k,l);
		a(i,j)=g-s*(h+g*tau);
		a(k,l)=h+s*(g-h*tau);
	}
}

void NR_single_precision::jacobi(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &d, NR_single_precision::Mat_O_DP &v, int &nrot)
{
	int i,j,ip,iq;
	NR_single_precision::DP tresh,theta,tau,t,sm,s,h,g,c;

	d.resize(a.rows());
	v.resize(d.size(), d.size());
	int n=d.size();
	NR_single_precision::Vec_DP b(n),z(n);
	for (ip=0;ip<n;ip++) {
		for (iq=0;iq<n;iq++) v(ip,iq)=0.0;
		v(ip,ip)=1.0;
	}
	for (ip=0;ip<n;ip++) {
		b(ip)=d(ip)=a(ip,ip);
		z(ip)=0.0;
	}
	nrot=0;
	for (i=1;i<=50;i++) {
		sm=0.0;
		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++)
				sm += fabs(a(ip,iq));
		}
		if (sm == 0.0)
			return;
		if (i < 4)
			tresh=0.2*sm/(n*n);
		else
			tresh=0.0;
		for (ip=0;ip<n-1;ip++) {
			for (iq=ip+1;iq<n;iq++) {
				g=100.0*fabs(a(ip,iq));
				if (i > 4 && (fabs(d(ip))+g) == fabs(d(ip))
					&& (fabs(d(iq))+g) == fabs(d(iq)))
						a(ip,iq)=0.0;
				else if (fabs(a(ip,iq)) > tresh) {
					h=d(iq)-d(ip);
					if ((fabs(h)+g) == fabs(h))
						t=(a(ip,iq))/h;
					else {
						theta=0.5*h/(a(ip,iq));
						t=1.0/(fabs(theta)+sqrt(1.0+theta*theta));
						if (theta < 0.0) t = -t;
					}
					c=1.0/sqrt(1+t*t);
					s=t*c;
					tau=s/(1.0+c);
					h=t*a(ip,iq);
					z(ip) -= h;
					z(iq) += h;
					d(ip) -= h;
					d(iq) += h;
					a(ip,iq)=0.0;
					for (j=0;j<ip;j++)
						rot(a,s,tau,j,ip,j,iq);
					for (j=ip+1;j<iq;j++)
						rot(a,s,tau,ip,j,j,iq);
					for (j=iq+1;j<n;j++)
						rot(a,s,tau,ip,j,iq,j);
					for (j=0;j<n;j++)
						rot(v,s,tau,j,ip,j,iq);
					++nrot;
				}
			}
		}
		for (ip=0;ip<n;ip++) {
			b(ip) += z(ip);
			d(ip)=b(ip);
			z(ip)=0.0;
		}
	}
	NR::nrerror("Too many iterations in routine jacobi");
}


void NR_single_precision::jacobn_s(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dfdx, NR_single_precision::Mat_O_DP &dfdy)
{
	int i;

	int n=y.size();
	for (i=0;i<n;i++) dfdx(i)=0.0;
	dfdy(0,0) = -0.013-1000.0*y(2);
	dfdy(0,1) = 0.0;
	dfdy(0,2) = -1000.0*y(0);
	dfdy(1,0) = 0.0;
	dfdy(1,1) = -2500.0*y(2);
	dfdy(1,2) = -2500.0*y(1);
	dfdy(2,0) = -0.013-1000.0*y(2);
	dfdy(2,1) = -2500.0*y(2);
	dfdy(2,2) = -1000.0*y(0)-2500.0*y(1);
}

void NR_single_precision::derivs_s(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dydx)
{
	dydx(0) = -0.013*y(0)-1000.0*y(0)*y(2);
	dydx(1) = -2500.0*y(1)*y(2);
	dydx(2) = -0.013*y(0)-1000.0*y(0)*y(2)-2500.0*y(1)*y(2);
}




int NR_single_precision::julday(const int mm, const int id, const int iyyy)
{
	const int IGREG=15+31*(10+12*1582);
	int ja,jul,jy=iyyy,jm;

	if (jy == 0) NR::nrerror("julday: there is no year zero.");
	if (jy < 0) ++jy;
	if (mm > 2) {
		jm=mm+1;
	} else {
		--jy;
		jm=mm+13;
	}
	jul = int(floor(365.25*jy)+floor(30.6001*jm)+id+1720995);
	if (id+31*(mm+12*iyyy) >= IGREG) {
		ja=int(0.01*jy);
		jul += 2-ja+int(0.25*ja);
	}
	return jul;
}




void NR_single_precision::kendl1(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &tau, NR_single_precision::DP &z, NR_single_precision::DP &prob)
{
	int is=0,j,k,n2=0,n1=0;
	NR_single_precision::DP svar,aa,a2,a1;

	int n=data1.size();
	for (j=0;j<n-1;j++) {
		for (k=j+1;k<n;k++) {
			a1=data1(j)-data1(k);
			a2=data2(j)-data2(k);
			aa=a1*a2;
			if (aa != 0.0) {
				++n1;
				++n2;
				aa > 0.0 ? ++is : --is;
			} else {
				if (a1 != 0.0) ++n1;
				if (a2 != 0.0) ++n2;
			}
		}
	}
	tau=is/(sqrt(NR_single_precision::DP(n1))*sqrt(NR_single_precision::DP(n2)));
	svar=(4.0*n+10.0)/(9.0*n*(n-1.0));
	z=tau/sqrt(svar);
	prob=erfcc(fabs(z)/1.4142136);
}




void NR_single_precision::kendl2(NR_single_precision::Mat_I_DP &tab, NR_single_precision::DP &tau, NR_single_precision::DP &z, NR_single_precision::DP &prob)
{
	int k,l,nn,mm,m2,m1,lj,li,kj,ki;
	NR_single_precision::DP svar,s=0.0,points,pairs,en2=0.0,en1=0.0;

	int i=tab.rows();
	int j=tab.cols();
	nn=i*j;
	points=tab(i-1,j-1);
	for (k=0;k<=nn-2;k++) {
		ki=(k/j);
		kj=k-j*ki;
		points += tab(ki,kj);
		for (l=k+1;l<=nn-1;l++) {
			li=l/j;
			lj=l-j*li;
			mm=(m1=li-ki)*(m2=lj-kj);
			pairs=tab(ki,kj)*tab(li,lj);
			if (mm != 0) {
				en1 += pairs;
				en2 += pairs;
				s += (mm > 0 ? pairs : -pairs);
			} else {
				if (m1 != 0) en1 += pairs;
				if (m2 != 0) en2 += pairs;
			}
		}
	}
	tau=s/sqrt(en1*en2);
	svar=(4.0*points+10.0)/(9.0*points*(points-1.0));
	z=tau/sqrt(svar);
	prob=erfcc(fabs(z)/1.4142136);
}




extern NR_single_precision::DP x;

void NR_single_precision::kermom(NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP y)
{
	NR_single_precision::DP d,df,clog,x2,x3,x4,y2;

	int m=w.size();
	if (y >= x) {
		d=y-x;
		df=2.0*sqrt(d)*d;
		w(0)=df/3.0;
		w(1)=df*(x/3.0+d/5.0);
		w(2)=df*((x/3.0 + 0.4*d)*x + d*d/7.0);
		w(3)=df*(((x/3.0 + 0.6*d)*x + 3.0*d*d/7.0)*x+d*d*d/9.0);
	} else {
		x3=(x2=x*x)*x;
		x4=x2*x2;
		y2=y*y;
		d=x-y;
		w(0)=d*((clog=log(d))-1.0);
		w(1) = -0.25*(3.0*x+y-2.0*clog*(x+y))*d;
		w(2)=(-11.0*x3+y*(6.0*x2+y*(3.0*x+2.0*y))
			+6.0*clog*(x3-y*y2))/18.0;
		w(3)=(-25.0*x4+y*(12.0*x3+y*(6.0*x2+y*
			(4.0*x+3.0*y)))+12.0*clog*(x4-(y2*y2)))/48.0;
	}
}




void NR_single_precision::ks2d1s(NR_single_precision::Vec_I_DP &x1, NR_single_precision::Vec_I_DP &y1, void quadvl(const NR_single_precision::DP, const NR_single_precision::DP,
	NR_single_precision::DP &, NR_single_precision::DP &, NR_single_precision::DP &, NR_single_precision::DP &), NR_single_precision::DP &d1, NR_single_precision::DP &prob)
{
	int j;
	NR_single_precision::DP dum,dumm,fa,fb,fc,fd,ga,gb,gc,gd,r1,rr,sqen;

	int n1=x1.size();
	d1=0.0;
	for (j=0;j<n1;j++) {
		quadct(x1(j),y1(j),x1,y1,fa,fb,fc,fd);
		quadvl(x1(j),y1(j),ga,gb,gc,gd);
		d1=MAX(d1,fabs(fa-ga));
		d1=MAX(d1,fabs(fb-gb));
		d1=MAX(d1,fabs(fc-gc));
		d1=MAX(d1,fabs(fd-gd));
	}
	pearsn(x1,y1,r1,dum,dumm);
	sqen=sqrt(NR_single_precision::DP(n1));
	rr=sqrt(1.0-r1*r1);
	prob=probks(d1*sqen/(1.0+rr*(0.25-0.75/sqen)));
}




void NR_single_precision::ks2d2s(NR_single_precision::Vec_I_DP &x1, NR_single_precision::Vec_I_DP &y1, NR_single_precision::Vec_I_DP &x2, NR_single_precision::Vec_I_DP &y2, NR_single_precision::DP &d,
	NR_single_precision::DP &prob)
{
	int j;
	NR_single_precision::DP d1,d2,dum,dumm,fa,fb,fc,fd,ga,gb,gc,gd,r1,r2,rr,sqen;

	int n1=x1.size();
	int n2=x2.size();
	d1=0.0;
	for (j=0;j<n1;j++) {
		quadct(x1(j),y1(j),x1,y1,fa,fb,fc,fd);
		quadct(x1(j),y1(j),x2,y2,ga,gb,gc,gd);
		d1=MAX(d1,fabs(fa-ga));
		d1=MAX(d1,fabs(fb-gb));
		d1=MAX(d1,fabs(fc-gc));
		d1=MAX(d1,fabs(fd-gd));
	}
	d2=0.0;
	for (j=0;j<n2;j++) {
		quadct(x2(j),y2(j),x1,y1,fa,fb,fc,fd);
		quadct(x2(j),y2(j),x2,y2,ga,gb,gc,gd);
		d2=MAX(d2,fabs(fa-ga));
		d2=MAX(d2,fabs(fb-gb));
		d2=MAX(d2,fabs(fc-gc));
		d2=MAX(d2,fabs(fd-gd));
	}
	d=0.5*(d1+d2);
	sqen=sqrt(n1*n2/NR_single_precision::DP(n1+n2));
	pearsn(x1,y1,r1,dum,dumm);
	pearsn(x2,y2,r2,dum,dumm);
	rr=sqrt(1.0-0.5*(r1*r1+r2*r2));
	prob=probks(d*sqen/(1.0+rr*(0.25-0.75/sqen)));
}




void NR_single_precision::ksone(NR_single_precision::Vec_IO_DP &data, NR_single_precision::DP func(const NR_single_precision::DP), NR_single_precision::DP &d, NR_single_precision::DP &prob)
{
	int j;
	NR_single_precision::DP dt,en,ff,fn,fo=0.0;

	int n=data.size();
	sort(data);
	en=n;
	d=0.0;
	for (j=0;j<n;j++) {
		fn=(j+1)/en;
		ff=func(data(j));
		dt=MAX(fabs(fo-ff),fabs(fn-ff));
		if (dt > d) d=dt;
		fo=fn;
	}
	en=sqrt(en);
	prob=probks((en+0.12+0.11/en)*d);
}




void NR_single_precision::kstwo(NR_single_precision::Vec_IO_DP &data1, NR_single_precision::Vec_IO_DP &data2, NR_single_precision::DP &d, NR_single_precision::DP &prob)
{
	int j1=0,j2=0;
	NR_single_precision::DP d1,d2,dt,en1,en2,en,fn1=0.0,fn2=0.0;

	int n1=data1.size();
	int n2=data2.size();
	sort(data1);
	sort(data2);
	en1=n1;
	en2=n2;
	d=0.0;
	while (j1 < n1 && j2 < n2) {
		if ((d1=data1(j1)) <= (d2=data2(j2))) fn1=j1++/en1;
		if (d2 <= d1) fn2=j2++/en2;
		if ((dt=fabs(fn2-fn1)) > d) d=dt;
	}
	en=sqrt(en1*en2/(en1+en2));
	prob=probks((en+0.12+0.11/en)*d);
}

#include <complex>
#include <limits>



void NR_single_precision::laguer(NR_single_precision::Vec_I_CPLX_DP &a, complex<NR_single_precision::DP> &x, int &its)
{
	Msg::error("languer");
		/*
	const int MR=8,MT=10,MAXIT=MT*MR;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	static const NR_single_precision::DP frac[MR+1]=
		{0.0,0.5,0.25,0.75,0.13,0.38,0.62,0.88,1.0};
	int iter,j;
	NR_single_precision::DP abx,abp,abm,err;
	complex<NR_single_precision::DP> dx,x1,b,d,f,g,h,sq,gp,gm,g2;

	int m=a.size()-1;
	for (iter=1;iter<=MAXIT;iter++) {
		its=iter;
		b=a(m);
		err=abs(b);
		d=f=0.0;
		abx=abs(x);
		for (j=m-1;j>=0;j--) {
			f=x*f+d;
			d=x*d+b;
			b=x*b+a(j);
			err=abs(b)+abx*err;
		}
		err *= EPS;
		if (abs(b) <= err) return;
		g=d/b;
		g2=g*g;
		h=g2-2.0*f/b;
		sq=sqrt(NR_single_precision::DP(m-1)*(NR_single_precision::DP(m)*h-g2));
		gp=g+sq;
		gm=g-sq;
		abp=abs(gp);
		abm=abs(gm);
		if (abp < abm) gp=gm;
		dx=MAX(abp,abm) > 0.0 ? NR_single_precision::DP(m)/gp : polar(1+abx,NR_single_precision::DP(iter));
		x1=x-dx;
		if (x == x1) return;
		if (iter % MT != 0) x=x1;
		else x -= frac(iter/MT)*dx;
	}
	NR::nrerror("too many iterations in laguer");
	return;*/
}


void NR_single_precision::lfit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_IO_DP &a,
	NR_single_precision::Vec_I_BOOL &ia, NR_single_precision::Mat_O_DP &covar, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_O_DP &))
{
	int i,j,k,l,m,mfit=0;
	NR_single_precision::DP ym,wt,sum,sig2i;

	int ndat=x.size();
	int ma=a.size();
	NR_single_precision::Vec_DP afunc(ma);
	NR_single_precision::Mat_DP beta(ma,1);
	for (j=0;j<ma;j++)
		if (ia(j)) mfit++;
	if (mfit == 0) NR::nrerror("lfit: no parameters to be fitted");
	for (j=0;j<mfit;j++) {
		for (k=0;k<mfit;k++) covar(j,k)=0.0;
		beta(j,0)=0.0;
	}
	for (i=0;i<ndat;i++) {
		funcs(x(i),afunc);
		ym=y(i);
		if (mfit < ma) {
			for (j=0;j<ma;j++)
				if (!ia(j)) ym -= a(j)*afunc(j);
		}
		sig2i=1.0/SQR(sig(i));
		for (j=0,l=0;l<ma;l++) {
			if (ia(l)) {
				wt=afunc(l)*sig2i;
				for (k=0,m=0;m<=l;m++)
					if (ia(m)) covar(j,k++) += wt*afunc(m);
				beta(j++,0) += ym*wt;
			}
		}
	}
	for (j=1;j<mfit;j++)
		for (k=0;k<j;k++)
			covar(k,j)=covar(j,k);
	NR_single_precision::Mat_DP temp(mfit,mfit);
	for (j=0;j<mfit;j++)
		for (k=0;k<mfit;k++)
			temp(j,k)=covar(j,k);
	gaussj(temp,beta);
	for (j=0;j<mfit;j++)
		for (k=0;k<mfit;k++)
			covar(j,k)=temp(j,k);
	for (j=0,l=0;l<ma;l++)
		if (ia(l)) a(l)=beta(j++,0);
	chisq=0.0;
	for (i=0;i<ndat;i++) {
		funcs(x(i),afunc);
		sum=0.0;
		for (j=0;j<ma;j++) sum += a(j)*afunc(j);
		chisq += SQR((y(i)-sum)/sig(i));
	}
	covsrt(covar,ia,mfit);
}
#include <iostream>
#include <iomanip>




void NR_single_precision::linbcg(NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_IO_DP &x, const int itol, const NR_single_precision::DP tol,
	const int itmax, int &iter, NR_single_precision::DP &err)
{
	NR_single_precision::DP ak,akden,bk,bkden=1.0,bknum,bnrm,dxnrm,xnrm,zm1nrm,znrm;
	const NR_single_precision::DP EPS=1.0e-14;
	int j;

	int n=b.size();
	NR_single_precision::Vec_DP p(n),pp(n),r(n),rr(n),z(n),zz(n);
	iter=0;
	atimes(x,r,0);
	for (j=0;j<n;j++) {
		r(j)=b(j)-r(j);
		rr(j)=r(j);
	}
	//atimes(r,rr,0);
	if (itol == 1) {
		bnrm=snrm(b,itol);
		asolve(r,z,0);
	}
	else if (itol == 2) {
		asolve(b,z,0);
		bnrm=snrm(z,itol);
		asolve(r,z,0);
	}
	else if (itol == 3 || itol == 4) {
		asolve(b,z,0);
		bnrm=snrm(z,itol);
		asolve(r,z,0);
		znrm=snrm(z,itol);
	} else NR::nrerror("illegal itol in linbcg");
	cout << fixed ;
	while (iter < itmax) {
		++iter;
		asolve(rr,zz,1);
		for (bknum=0.0,j=0;j<n;j++) bknum += z(j)*rr(j);
		if (iter == 1) {
			for (j=0;j<n;j++) {
				p(j)=z(j);
				pp(j)=zz(j);
			}
		} else {
			bk=bknum/bkden;
			for (j=0;j<n;j++) {
				p(j)=bk*p(j)+z(j);
				pp(j)=bk*pp(j)+zz(j);
			}
		}
		bkden=bknum;
		atimes(p,z,0);
		for (akden=0.0,j=0;j<n;j++) akden += z(j)*pp(j);
		ak=bknum/akden;
		atimes(pp,zz,1);
		for (j=0;j<n;j++) {
			x(j) += ak*p(j);
			r(j) -= ak*z(j);
			rr(j) -= ak*zz(j);
		}
		asolve(r,z,0);
		if (itol == 1)
			err=snrm(r,itol)/bnrm;
		else if (itol == 2)
			err=snrm(z,itol)/bnrm;
		else if (itol == 3 || itol == 4) {
			zm1nrm=znrm;
			znrm=snrm(z,itol);
			if (fabs(zm1nrm-znrm) > EPS*znrm) {
				dxnrm=fabs(ak)*snrm(p,itol);
				err=znrm/fabs(zm1nrm-znrm)*dxnrm;
			} else {
				err=znrm/bnrm;
				continue;
			}
			xnrm=snrm(x,itol);
			if (err <= 0.5*xnrm) err /= xnrm;
			else {
				err=znrm/bnrm;
				continue;
			}
		}
		cout << "iter=" << iter+1 << err << endl;
		if (err <= tol) break;
	}
}


//int ncom;
//NR_single_precision::DP (*nrfunc)(NR_single_precision::Vec_I_DP &);
//NR_single_precision::Vec_DP *pcom_p,*xicom_p;

void NR_single_precision::linmin(NR_single_precision::Vec_IO_DP &p, NR_single_precision::Vec_IO_DP &xi, NR_single_precision::DP &fret, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &))
{
	int j;
	const NR_single_precision::DP TOL=1.0e-8;
	NR_single_precision::DP xx,xmin,fx,fb,fa,bx,ax;

	int n=p.size();
	ncom=n;
	pcom_p=new NR_single_precision::Vec_DP(n);
	xicom_p=new NR_single_precision::Vec_DP(n);
	nrfunc=func;
	NR_single_precision::Vec_DP &pcom=*pcom_p,&xicom=*xicom_p;
	for (j=0;j<n;j++) {
		pcom(j)=p(j);
		xicom(j)=xi(j);
	}
	ax=0.0;
	xx=1.0;
	mnbrak(ax,xx,bx,fa,fx,fb,f1dim);
	fret=brent(ax,xx,bx,f1dim,TOL,xmin);
	for (j=0;j<n;j++) {
		xi(j) *= xmin;
		p(j) += xi(j);
	}
	delete xicom_p;
	delete pcom_p;
}

#include <limits>



void NR_single_precision::lnsrch(NR_single_precision::Vec_I_DP &xold, const NR_single_precision::DP fold, NR_single_precision::Vec_I_DP &g, NR_single_precision::Vec_IO_DP &p,
	NR_single_precision::Vec_O_DP &x, NR_single_precision::DP &f, const NR_single_precision::DP stpmax, bool &check, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &))
{
	const NR_single_precision::DP ALF=1.0e-4, TOLX=DBL_EPSILON;
	int i;
	NR_single_precision::DP a,alam,alam2=0.0,alamin,b,disc,f2=0.0;
	NR_single_precision::DP rhs1,rhs2,slope,sum,temp,test,tmplam;

	int n=xold.size();
	check=false;
	sum=0.0;
	for (i=0;i<n;i++) sum += p(i)*p(i);
	sum=sqrt(sum);
	if (sum > stpmax)
		for (i=0;i<n;i++) p(i) *= stpmax/sum;
	slope=0.0;
	for (i=0;i<n;i++)
		slope += g(i)*p(i);
	if (slope >= 0.0) NR::nrerror("Roundoff problem in lnsrch.");
	test=0.0;
	for (i=0;i<n;i++) {
		temp=fabs(p(i))/MAX(fabs(xold(i)),1.0);
		if (temp > test) test=temp;
	}
	alamin=TOLX/test;
	alam=1.0;
	for (;;) {
		for (i=0;i<n;i++) x(i)=xold(i)+alam*p(i);
		f=func(x);
		if (alam < alamin) {
			for (i=0;i<n;i++) x(i)=xold(i);
			check=true;
			return;
		} else if (f <= fold+ALF*alam*slope) return;
		else {
			if (alam == 1.0)
				tmplam = -slope/(2.0*(f-fold-slope));
			else {
				rhs1=f-fold-alam*slope;
				rhs2=f2-fold-alam2*slope;
				a=(rhs1/(alam*alam)-rhs2/(alam2*alam2))/(alam-alam2);
				b=(-alam2*rhs1/(alam*alam)+alam*rhs2/(alam2*alam2))/(alam-alam2);
				if (a == 0.0) tmplam = -slope/(2.0*b);
				else {
					disc=b*b-3.0*a*slope;
					if (disc < 0.0) tmplam=0.5*alam;
					else if (b <= 0.0) tmplam=(-b+sqrt(disc))/(3.0*a);
					else tmplam=-slope/(b+sqrt(disc));
				}
				if (tmplam>0.5*alam)
					tmplam=0.5*alam;
			}
		}
		alam2=alam;
		f2 = f;
		alam=MAX(tmplam,0.1*alam);
	}
}


void NR_single_precision::locate(NR_single_precision::Vec_I_DP &xx, const NR_single_precision::DP x, int &j)
{
	int ju,jm,jl;
	bool ascnd;

	int n=xx.size();
	jl=-1;
	ju=n;
	ascnd=(xx(n-1) >= xx(0));
	while (ju-jl > 1) {
		jm=(ju+jl) >> 1;
		if (x >= xx(jm) == ascnd)
			jl=jm;
		else
			ju=jm;
	}
	if (x == xx(0)) j=0;
	else if (x == xx(n-1)) j=n-2;
	else j=jl;
}


void NR_single_precision::lop(NR_single_precision::Mat_O_DP &out, NR_single_precision::Mat_I_DP &u)
{
	int i,j;
	NR_single_precision::DP h,h2i;

	int n=u.rows();
	h=1.0/(n-1);
	h2i=1.0/(h*h);
	for (j=1;j<n-1;j++)
		for (i=1;i<n-1;i++)
			out(i,j)=h2i*(u(i+1,j)+u(i-1,j)+u(i,j+1)+u(i,j-1)-
				4.0*u(i,j))+u(i,j)*u(i,j);
	for (i=0;i<n;i++)
		out(i,0)=out(i,n-1)=out(0,i)=out(n-1,i)=0.0;
}


void NR_single_precision::lubksb(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_IO_DP &b)
{
	int i,ii=0,ip,j;
	NR_single_precision::DP sum;

	int n=a.rows();
	for (i=0;i<n;i++) {
		ip=indx(i);
		sum=b(ip);
		b(ip)=b(i);
		if (ii != 0)
			for (j=ii-1;j<i;j++) sum -= a(i,j)*b(j);
		else if (sum != 0.0)
			ii=i+1;
		b(i)=sum;
	}
	for (i=n-1;i>=0;i--) {
		sum=b(i);
		for (j=i+1;j<n;j++) sum -= a(i,j)*b(j);
		b(i)=sum/a(i,i);
	}
}




void NR_single_precision::ludcmp(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_INT &indx, NR_single_precision::DP &d)
{	
	int i,imax,j,k;
	NR_single_precision::DP big,dum,sum,temp;

	NR_single_precision::DP originald=d;
	int n=a.rows();
	NR_single_precision::Vec_DP vv(n);
	d=1.0;
	for (i=0;i<n;i++) {
		big=0.0;
		for (j=0;j<n;j++)
			if ((temp=fabs(a(i,j))) > big) big=temp;

		if (big == 0.0) 
		{
			if(originald==NR_single_precision::MAXIMUM_CONSTANTS)	// do not report error. 
			{
				d=NR_single_precision::MAXIMUM_CONSTANTS;
				return;
			}
			else
				NR::nrerror("Singular matrix in routine ludcmp");
		}
		vv(i)=1.0/big;
	}
	for (j=0;j<n;j++) {
		for (i=0;i<j;i++) {
			sum=a(i,j);
			for (k=0;k<i;k++) sum -= a(i,k)*a(k,j);
			a(i,j)=sum;
		}
		big=0.0;
		for (i=j;i<n;i++) {
			sum=a(i,j);
			for (k=0;k<j;k++) sum -= a(i,k)*a(k,j);
			a(i,j)=sum;
			if ((dum=vv(i)*fabs(sum)) >= big) {
				big=dum;
				imax=i;
			}
		}
		if (j != imax) {
			for (k=0;k<n;k++) {
				dum=a(imax,k);
				a(imax,k)=a(j,k);
				a(j,k)=dum;
			}
			d = -d;
			vv(imax)=vv(j);
		}
		indx(j)=imax;
		if (a(j,j) == 0.0) a(j,j)=TINY2;
		if (j != n-1) {
			dum=1.0/(a(j,j));
			for (i=j+1;i<n;i++) a(i,j) *= dum;
		}
	}
}




void NR_single_precision::machar(int &ibeta, int &it, int &irnd, int &ngrd, int &machep,
	int &negep, int &iexp, int &minexp, int &maxexp, NR_single_precision::DP &eps, NR_single_precision::DP &epsneg,
	NR_single_precision::DP &xmin, NR_single_precision::DP &xmax)
{
	int i,itemp,iz,j,k,mx,nxres;
	NR_single_precision::DP a,b,beta,betah,betain,one,t,temp,temp1,tempa,two,y,z,zero;

	one=NR_single_precision::DP(1);
	two=one+one;
	zero=one-one;
	a=one;
	do {
		a += a;
		temp=a+one;
		temp1=temp-a;
	} while (temp1-one == zero);
	b=one;
	do {
		b += b;
		temp=a+b;
		itemp=int(temp-a);
	} while (itemp == 0);
	ibeta=itemp;
	beta=NR_single_precision::DP(ibeta);
	it=0;
	b=one;
	do {
		++it;
		b *= beta;
		temp=b+one;
		temp1=temp-b;
	} while (temp1-one == zero);
	irnd=0;
	betah=beta/two;
	temp=a+betah;
	if (temp-a != zero) irnd=1;
	tempa=a+beta;
	temp=tempa+betah;
	if (irnd == 0 && temp-tempa != zero) irnd=2;
	negep=it+3;
	betain=one/beta;
	a=one;
	for (i=1;i<=negep;i++) a *= betain;
	b=a;
	for (;;) {
		temp=one-a;
		if (temp-one != zero) break;
		a *= beta;
		--negep;
	}
	negep = -negep;
	epsneg=a;
	machep = -it-3;
	a=b;
	for (;;) {
		temp=one+a;
		if (temp-one != zero) break;
		a *= beta;
		++machep;
	}
	eps=a;
	ngrd=0;
	temp=one+eps;
	if (irnd == 0 && temp*one-one != zero) ngrd=1;
	i=0;
	k=1;
	z=betain;
	t=one+eps;
	nxres=0;
	for (;;) {
		y=z;
		z=y*y;
		a=z*one;
		temp=z*t;
		if (a+a == zero || fabs(z) >= y) break;
		temp1=temp*betain;
		if (temp1*beta == z) break;
		++i;
		k += k;
	}
	if (ibeta != 10) {
		iexp=i+1;
		mx=k+k;
	} else {
		iexp=2;
		iz=ibeta;
		while (k >= iz) {
			iz *= ibeta;
			++iexp;
		}
		mx=iz+iz-1;
	}
	for (;;) {
		xmin=y;
		y *= betain;
		a=y*one;
		temp=y*t;
		if (a+a != zero && fabs(y) < xmin) {
			++k;
			temp1=temp*betain;
			if (temp1*beta == y && temp != y) {
				nxres=3;
				xmin=y;
				break;
			}
		}
		else break;
	}
	minexp = -k;
	if (mx <= k+k-3 && ibeta != 10) {
		mx += mx;
		++iexp;
	}
	maxexp=mx+minexp;
	irnd += nxres;
	if (irnd >= 2) maxexp -= 2;
	i=maxexp+minexp;
	if (ibeta == 2 && !i) --maxexp;
	if (i > 20) --maxexp;
	if (a != y) maxexp -= 2;
	xmax=one-epsneg;
	if (xmax*one != xmax) xmax=one-beta*epsneg;
	xmax /= (xmin*beta*beta*beta);
	i=maxexp+minexp+3;
	for (j=1;j<=i;j++) {
		if (ibeta == 2) xmax += xmax;
		else xmax *= beta;
	}
}


void NR_single_precision::matadd(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &b, NR_single_precision::Mat_O_DP &c)
{
	int i,j;

	int n=a.rows();
	for (j=0;j<n;j++)
		for (i=0;i<n;i++)
			c(i,j)=a(i,j)+b(i,j);
}


void NR_single_precision::matsub(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &b, NR_single_precision::Mat_O_DP &c)
{
	int i,j;

	int n=a.rows();
	for (j=0;j<n;j++)
		for (i=0;i<n;i++)
			c(i,j)=a(i,j)-b(i,j);
}




//NR_single_precision::DP aa;
NR_single_precision::DP abdevt;
const NR_single_precision::Vec_I_DP *xt_p,*yt_p;

void NR_single_precision::medfit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::DP &a, NR_single_precision::DP &b, NR_single_precision::DP &abdev)
{
	int j;
	NR_single_precision::DP bb,b1,b2,del,f,f1,f2,sigb,temp;
	NR_single_precision::DP sx=0.0,sy=0.0,sxy=0.0,sxx=0.0,chisq=0.0;

	int ndata=x.size();
	xt_p= &x;
	yt_p= &y;
	for (j=0;j<ndata;j++) {
		sx += x(j);
		sy += y(j);
		sxy += x(j)*y(j);
		sxx += x(j)*x(j);
	}
	del=ndata*sxx-sx*sx;
	aa=(sxx*sy-sx*sxy)/del;
	bb=(ndata*sxy-sx*sy)/del;
	for (j=0;j<ndata;j++)
		chisq += (temp=y(j)-(aa+bb*x(j)),temp*temp);
	sigb=sqrt(chisq/del);
	b1=bb;
	f1=rofunc(b1);
	if (sigb > 0.0) {
		b2=bb+SIGN(3.0*sigb,f1);
		f2=rofunc(b2);
		if (b2 == b1) {
			a=aa;
			b=bb;
			abdev=abdevt/ndata;
			return;
		}
		while (f1*f2 > 0.0) {
			bb=b2+1.6*(b2-b1);
			b1=b2;
			f1=f2;
			b2=bb;
			f2=rofunc(b2);
		}
		sigb=0.01*sigb;
		while (fabs(b2-b1) > sigb) {
			bb=b1+0.5*(b2-b1);
			if (bb == b1 || bb == b2) break;
			f=rofunc(bb);
			if (f*f1 >= 0.0) {
				f1=f;
				b1=bb;
			} else {
				f2=f;
				b2=bb;
			}
		}
	}
	a=aa;
	b=bb;
	abdev=abdevt/ndata;
}




void NR_single_precision::memcof(NR_single_precision::Vec_I_DP &data, NR_single_precision::DP &xms, NR_single_precision::Vec_O_DP &d)
{
	int k,j,i;
	NR_single_precision::DP p=0.0;

	int n=data.size();
	int m=d.size();
	NR_single_precision::Vec_DP wk1(n),wk2(n),wkm(m);
	for (j=0;j<n;j++) p += SQR(data(j));
	xms=p/n;
	wk1(0)=data(0);
	wk2(n-2)=data(n-1);
	for (j=1;j<n-1;j++) {
		wk1(j)=data(j);
		wk2(j-1)=data(j);
	}
	for (k=0;k<m;k++) {
		NR_single_precision::DP num=0.0,denom=0.0;
		for (j=0;j<(n-k-1);j++) {
			num += (wk1(j)*wk2(j));
			denom += (SQR(wk1(j))+SQR(wk2(j)));
		}
		d(k)=2.0*num/denom;
		xms *= (1.0-SQR(d(k)));
		for (i=0;i<k;i++)
			d(i)=wkm(i)-d(k)*wkm(k-1-i);
		if (k == m-1)
			return;
		for (i=0;i<=k;i++) wkm(i)=d(i);
		for (j=0;j<(n-k-2);j++) {
			wk1(j) -= (wkm(k)*wk2(j));
			wk2(j)=wk2(j+1)-wkm(k)*wk1(j+1);
		}
	}
	NR::nrerror("never get here in memcof.");
}




bool NR_single_precision::metrop(const NR_single_precision::DP de, const NR_single_precision::DP t)
{
	static int gljdum=1;

	return de < 0.0 || ran3(gljdum) < exp(-de/t);
}


namespace {
	/*void mg(const int j, NR_single_precision::Mat_IO_DP &u, NR_single_precision::Mat_I_DP &rhs, Vec_NR_single_precision::Mat_DP_p &rho,
		NR_single_precision::DP &trerr)
	{
		ASSERT(0);
		
		using namespace NR;
		const int NPRE=1,NPOST=1;
		const NR_single_precision::DP ALPHA=0.33;
		int jpost,jpre,nc,nf;
		NR_single_precision::DP dum=-1.0;

		nf=u.rows();
		nc=(nf+1)/2;
		NR_single_precision::Mat_DP temp(nf,nf);
		if (j == 0) {
			matadd(rhs,*rho(j),temp);
			slvsm2(u,temp);
		} else {
			NR_single_precision::Mat_DP v(nc,nc),ut(nc,nc),tau(nc,nc),tempc(nc,nc);
			for (jpre=0;jpre<NPRE;jpre++) {
				if (trerr < 0.0) {
					matadd(rhs,*rho(j),temp);
					relax2(u,temp);
				}
				else
					relax2(u,*rho(j));
			}
			rstrct(ut,u);
			copy(v,ut);
			lop(tau,ut);
			lop(temp,u);
			if (trerr < 0.0)
				matsub(temp,rhs,temp);
			rstrct(tempc,temp);
			matsub(tau,tempc,tau);
			if (trerr > 0.0)
				trerr=ALPHA*anorm2(tau);
			mg(j-1,v,tau,rho,dum);
			matsub(v,ut,tempc);
			interp(temp,tempc);
			matadd(u,temp,u);
			for (jpost=0;jpost<NPOST;jpost++) {
				if (trerr < 0.0) {
					matadd(rhs,*rho(j),temp);
					relax2(u,temp);
				}
				else
					relax2(u,*rho(j));
			}
		}
	}
	*/
}

void NR_single_precision::mgfas(NR_single_precision::Mat_IO_DP &u, const int maxcyc)
{
	ASSERT(0);
	/*
	int j,jcycle,ng=0,ngrid,nn;
	NR_single_precision::DP res,trerr;
	NR_single_precision::Mat_DP *uj,*uj1;

	int n=u.rows();
	nn=n;
	while (nn >>= 1) ng++;
	if ((n-1) != (1 << ng))
		NR::nrerror("n-1 must be a power of 2 in mgfas.");
	Vec_NR_single_precision::Mat_DP_p rho(ng);
	nn=n;
	ngrid=ng-1;
	rho(ngrid)=new NR_single_precision::Mat_DP(nn,nn);
	copy(*rho(ngrid),u);
	while (nn > 3) {
		nn=nn/2+1;
		rho(--ngrid)=new NR_single_precision::Mat_DP(nn,nn);
		rstrct(*rho(ngrid),*rho(ngrid+1));
	}
	nn=3;
	uj=new NR_single_precision::Mat_DP(nn,nn);
	slvsm2(*uj,*rho(0));
	for (j=1;j<ng;j++) {
		nn=2*nn-1;
		uj1=uj;
		uj=new NR_single_precision::Mat_DP(nn,nn);
		NR_single_precision::Mat_DP temp(nn,nn);
		interp(*uj,*uj1);
		delete uj1;
		for (jcycle=0;jcycle<maxcyc;jcycle++) {
			trerr=1.0;
			mg(j,*uj,temp,rho,trerr);
			lop(temp,*uj);
			matsub(temp,*rho(j),temp);
			res=anorm2(temp);
			if (res < trerr) break;
		}
	}
	copy(u,*uj);
	delete uj;
	for (j=0;j<ng;j++)
		delete rho(j);*/
}


namespace {
	void mg(int j, NR_single_precision::Mat_IO_DP &u, NR_single_precision::Mat_I_DP &rhs)
	{
		ASSERT(0);
		/*
		using namespace NR;
		const int NPRE=1,NPOST=1;
		int jpost,jpre,nc,nf;

		nf=u.rows();
		nc=(nf+1)/2;
		if (j == 0)
			slvsml(u,rhs);
		else {
			NR_single_precision::Mat_DP res(nc,nc),v(0.0,nc,nc),temp(nf,nf);
			for (jpre=0;jpre<NPRE;jpre++)
				relax(u,rhs);
			resid(temp,u,rhs);
			rstrct(res,temp);
			mg(j-1,v,res);
			addint(u,v,temp);
			for (jpost=0;jpost<NPOST;jpost++)
				relax(u,rhs);
		}*/
	}
}

void NR_single_precision::mglin(NR_single_precision::Mat_IO_DP &u, const int ncycle)
{
	ASSERT(0);
	/*
	int j,jcycle,ng=0,ngrid,nn;
	NR_single_precision::Mat_DP *uj,*uj1;

	int n=u.rows();

	nn=n;
	while (nn >>= 1) ng++;
	if ((n-1) != (1 << ng))
		NR::nrerror("n-1 must be a power of 2 in mglin.");
	Vec_NR_single_precision::Mat_DP_p rho(ng);
	nn=n;
	ngrid=ng-1;
	rho(ngrid) = new NR_single_precision::Mat_DP(nn,nn);
	copy(*rho(ngrid),u);
	while (nn > 3) {
		nn=nn/2+1;
		rho(--ngrid)=new NR_single_precision::Mat_DP(nn,nn);
		rstrct(*rho(ngrid),*rho(ngrid+1));
	}
	nn=3;
	uj=new NR_single_precision::Mat_DP(nn,nn);
	slvsml(*uj,*rho(0));
	for (j=1;j<ng;j++) {
		nn=2*nn-1;
		uj1=uj;
		uj=new NR_single_precision::Mat_DP(nn,nn);
		interp(*uj,*uj1);
		delete uj1;
		for (jcycle=0;jcycle<ncycle;jcycle++)
			mg(j,*uj,*rho(j));
	}
	copy(u,*uj);
	delete uj;
	for (j=0;j<ng;j++)
		delete rho(j);*/
}






namespace {
	NR_single_precision::DP midinffunc(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP x)
	{
		return funk(1.0/x)/(x*x);
	}
}

NR_single_precision::DP NR_single_precision::midinf(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n)
{
	NR_single_precision::DP x,tnm,sum,del,ddel,b,a;
	static NR_single_precision::DP s;
	int it,j;

	b=1.0/aa;
	a=1.0/bb;
	if (n == 1) {
		return (s=(b-a)*midinffunc(funk,0.5*(a+b)));
	} else {
		for(it=1,j=1;j<n-1;j++) it *= 3;
		tnm=it;
		del=(b-a)/(3.0*tnm);
		ddel=del+del;
		x=a+0.5*del;
		sum=0.0;
		for (j=0;j<it;j++) {
			sum += midinffunc(funk,x);
			x += ddel;
			sum += midinffunc(funk,x);
			x += del;
		}
		return (s=(s+(b-a)*sum/tnm)/3.0);
	}
}


NR_single_precision::DP NR_single_precision::midpnt(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const int n)
{
	int it,j;
	NR_single_precision::DP x,tnm,sum,del,ddel;
	static NR_single_precision::DP s;

	if (n == 1) {
		return (s=(b-a)*func(0.5*(a+b)));
	} else {
		for(it=1,j=1;j<n-1;j++) it *= 3;
		tnm=it;
		del=(b-a)/(3.0*tnm);
		ddel=del+del;
		x=a+0.5*del;
		sum=0.0;
		for (j=0;j<it;j++) {
			sum += func(x);
			x += ddel;
			sum += func(x);
			x += del;
		}
		s=(s+(b-a)*sum/tnm)/3.0;
		return s;
	}
}




namespace {
	NR_single_precision::DP midsqlfunc(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP x)
	{
		return 2.0*x*funk(aa+x*x);
	}
}

NR_single_precision::DP NR_single_precision::midsql(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n)
{
	NR_single_precision::DP x,tnm,sum,del,ddel,a,b;
	static NR_single_precision::DP s;
	int it,j;

	b=sqrt(bb-aa);
	a=0.0;
	if (n == 1) {
		return (s=(b-a)*midsqlfunc(funk,aa,0.5*(a+b)));
	} else {
		for(it=1,j=1;j<n-1;j++) it *= 3;
		tnm=it;
		del=(b-a)/(3.0*tnm);
		ddel=del+del;
		x=a+0.5*del;
		sum=0.0;
		for (j=0;j<it;j++) {
			sum += midsqlfunc(funk,aa,x);
			x += ddel;
			sum += midsqlfunc(funk,aa,x);
			x += del;
		}
		s=(s+(b-a)*sum/tnm)/3.0;
		return s;
	}
}




namespace {
	NR_single_precision::DP midsqufunc(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP bb, const NR_single_precision::DP x)
	{
		return 2.0*x*funk(bb-x*x);
	}
}

NR_single_precision::DP NR_single_precision::midsqu(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n)
{
	NR_single_precision::DP x,tnm,sum,del,ddel,a,b;
	static NR_single_precision::DP s;
	int it,j;

	b=sqrt(bb-aa);
	a=0.0;
	if (n == 1) {
		return (s=(b-a)*midsqufunc(funk,bb,0.5*(a+b)));
	} else {
		for(it=1,j=1;j<n-1;j++) it *= 3;
		tnm=it;
		del=(b-a)/(3.0*tnm);
		ddel=del+del;
		x=a+0.5*del;
		sum=0.0;
		for (j=0;j<it;j++) {
			sum += midsqufunc(funk,bb,x);
			x += ddel;
			sum += midsqufunc(funk,bb,x);
			x += del;
		}
		s=(s+(b-a)*sum/tnm)/3.0;
		return s;
	}
}




void NR_single_precision::miser(NR_single_precision::DP func(NR_single_precision::Vec_I_DP &), NR_single_precision::Vec_I_DP &regn, const int npts,
	const NR_single_precision::DP dith, NR_single_precision::DP &ave, NR_single_precision::DP &var)
{
	const int MNPT=15, MNBS=60;
	const NR_single_precision::DP PFAC=0.1, TINY=1.0e-30, BIG=1.0e30;
	static int iran=0;
	int j,jb,n,ndim,npre,nptl,nptr;
	NR_single_precision::DP avel,varl,fracl,fval,rgl,rgm,rgr,s,sigl,siglb,sigr,sigrb;
	NR_single_precision::DP sum,sumb,summ,summ2;

	ndim=regn.size()/2;
	NR_single_precision::Vec_DP pt(ndim);
	if (npts < MNBS) {
		summ=summ2=0.0;
		for (n=0;n<npts;n++) {
			ranpt(pt,regn);
			fval=func(pt);
			summ += fval;
			summ2 += fval * fval;
		}
		ave=summ/npts;
		var=MAX(TINY,(summ2-summ*summ/npts)/(npts*npts));
	} else {
		NR_single_precision::Vec_DP rmid(ndim);
		npre=MAX(int(npts*PFAC),int(MNPT));
		NR_single_precision::Vec_DP fmaxl(ndim),fmaxr(ndim),fminl(ndim),fminr(ndim);
		for (j=0;j<ndim;j++) {
			iran=(iran*2661+36979) % 175000;
			s=SIGN(dith,NR_single_precision::DP(iran-87500));
			rmid(j)=(0.5+s)*regn(j)+(0.5-s)*regn(ndim+j);
			fminl(j)=fminr(j)=BIG;
			fmaxl(j)=fmaxr(j)=(-BIG);
		}
		for (n=0;n<npre;n++) {
			ranpt(pt,regn);
			fval=func(pt);
			for (j=0;j<ndim;j++) {
				if (pt(j)<=rmid(j)) {
					fminl(j)=MIN(fminl(j),fval);
					fmaxl(j)=MAX(fmaxl(j),fval);
				} else {
					fminr(j)=MIN(fminr(j),fval);
					fmaxr(j)=MAX(fmaxr(j),fval);
				}
			}
		}
		sumb=BIG;
		jb= -1;
		siglb=sigrb=1.0;
		for (j=0;j<ndim;j++) {
			if (fmaxl(j) > fminl(j) && fmaxr(j) > fminr(j)) {
				sigl=MAX(TINY,pow(fmaxl(j)-fminl(j),2.0/3.0));
				sigr=MAX(TINY,pow(fmaxr(j)-fminr(j),2.0/3.0));
				sum=sigl+sigr;
				if (sum<=sumb) {
					sumb=sum;
					jb=j;
					siglb=sigl;
					sigrb=sigr;
				}
			}
		}
		if (jb == -1) jb=(ndim*iran)/175000;
		rgl=regn(jb);
		rgm=rmid(jb);
		rgr=regn(ndim+jb);
		fracl=fabs((rgm-rgl)/(rgr-rgl));
		nptl=int(MNPT+(npts-npre-2*MNPT)*fracl*siglb
			/(fracl*siglb+(1.0-fracl)*sigrb));
		nptr=npts-npre-nptl;
		NR_single_precision::Vec_DP regn_temp(2*ndim);
		for (j=0;j<ndim;j++) {
			regn_temp(j)=regn(j);
			regn_temp(ndim+j)=regn(ndim+j);
		}
		regn_temp(ndim+jb)=rmid(jb);
		miser(func,regn_temp,nptl,dith,avel,varl);
		regn_temp(jb)=rmid(jb);
		regn_temp(ndim+jb)=regn(ndim+jb);
		miser(func,regn_temp,nptr,dith,ave,var);
		ave=fracl*avel+(1-fracl)*ave;
		var=fracl*fracl*varl+(1-fracl)*(1-fracl)*var;
	}
}


void NR_single_precision::mmid(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, const NR_single_precision::DP xs, const NR_single_precision::DP htot,
	const int nstep, NR_single_precision::Vec_O_DP &yout,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	int i,n;
	NR_single_precision::DP x,swap,h2,h;

	int nvar=y.size();
	NR_single_precision::Vec_DP ym(nvar),yn(nvar);
	h=htot/nstep;
	for (i=0;i<nvar;i++) {
		ym(i)=y(i);
		yn(i)=y(i)+h*dydx(i);
	}
	x=xs+h;
	derivs(x,yn,yout);
	h2=2.0*h;
	for (n=1;n<nstep;n++) {
		for (i=0;i<nvar;i++) {
			swap=ym(i)+h2*yout(i);
			ym(i)=yn(i);
			yn(i)=swap;
		}
		x += h;
		derivs(x,yn,yout);
	}
	for (i=0;i<nvar;i++)
		yout(i)=0.5*(ym(i)+yn(i)+h*yout(i));
}





void NR_single_precision::mnbrak(NR_single_precision::DP &ax, NR_single_precision::DP &bx, NR_single_precision::DP &cx, NR_single_precision::DP &fa, NR_single_precision::DP &fb, NR_single_precision::DP &fc,
	NR_single_precision::DP func(const NR_single_precision::DP))
{
	const NR_single_precision::DP GOLD=1.618034,GLIMIT=100.0,TINY=1.0e-20;
	NR_single_precision::DP ulim,u,r,q,fu;

	fa=func(ax);
	fb=func(bx);
	if (fb > fa) {
		SWAP(ax,bx);
		SWAP(fb,fa);
	}
	cx=bx+GOLD*(bx-ax);
	fc=func(cx);
	while (fb > fc) {
		r=(bx-ax)*(fb-fc);
		q=(bx-cx)*(fb-fa);
		u=bx-((bx-cx)*q-(bx-ax)*r)/
			(2.0*SIGN(MAX(fabs(q-r),TINY),q-r));
		ulim=bx+GLIMIT*(cx-bx);
		if ((bx-u)*(u-cx) > 0.0) {
			fu=func(u);
			if (fu < fc) {
				ax=bx;
				bx=u;
				fa=fb;
				fb=fu;
				return;
			} else if (fu > fb) {
				cx=u;
				fc=fu;
				return;
			}
			u=cx+GOLD*(cx-bx);
			fu=func(u);
		} else if ((cx-u)*(u-ulim) > 0.0) {
			fu=func(u);
			if (fu < fc) {
				shft3(bx,cx,u,cx+GOLD*(cx-bx));
				shft3(fb,fc,fu,func(u));
			}
		} else if ((u-ulim)*(ulim-cx) >= 0.0) {
			u=ulim;
			fu=func(u);
		} else {
			u=cx+GOLD*(cx-bx);
			fu=func(u);
		}
		shft3(ax,bx,cx,u);
		shft3(fa,fb,fc,fu);
	}
}






void NR_single_precision::mnewt(const int ntrial, NR_single_precision::Vec_IO_DP &x, const NR_single_precision::DP tolx, const NR_single_precision::DP tolf, void (*usrfun)(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &fvec, NR_single_precision::Mat_O_DP &fjac))
{
	int k,i;
	NR_single_precision::DP errx,errf,d;

	int n=x.size();
	NR_single_precision::Vec_INT indx(n);
	NR_single_precision::Vec_DP p(n),fvec(n);
	NR_single_precision::Mat_DP fjac(n,n);
	for (k=0;k<ntrial;k++) {
		usrfun(x,fvec,fjac);
		errf=0.0;
		for (i=0;i<n;i++) errf += fabs(fvec(i));
		if (errf <= tolf) return;
		for (i=0;i<n;i++) p(i) = -fvec(i);
		ludcmp(fjac,indx,d);
		lubksb(fjac,indx,p);
		errx=0.0;
		for (i=0;i<n;i++) {
			errx += fabs(p(i));
			x(i) += p(i);
		}
		if (errx <= tolx) return;
	}
	return;
}




void NR_single_precision::moment(NR_single_precision::Vec_I_DP &data, NR_single_precision::DP &ave, NR_single_precision::DP &adev, NR_single_precision::DP &sdev, NR_single_precision::DP &var, NR_single_precision::DP &skew,
	NR_single_precision::DP &curt)
{
	int j;
	NR_single_precision::DP ep=0.0,s,p;

	int n=data.size();
	if (n <= 1) NR::nrerror("n must be at least 2 in moment");
	s=0.0;
	for (j=0;j<n;j++) s += data(j);
	ave=s/n;
	adev=var=skew=curt=0.0;
	for (j=0;j<n;j++) {
		adev += fabs(s=data(j)-ave);
		ep += s;
		var += (p=s*s);
		skew += (p *= s);
		curt += (p *= s);
	}
	adev /= n;
	var=(var-ep*ep/n)/(n-1);
	sdev=sqrt(var);
	if (var != 0.0) {
		skew /= (n*var*sdev);
		curt=curt/(n*var*var)-3.0;
	} else NR::nrerror("No skew/kurtosis when variance = 0 (in moment)");
}


void NR_single_precision::mp2dfr(NR_single_precision::Vec_IO_UCHR &a, string &s)
{
	const unsigned int IAZ=48;
	int j,m;

	int n=a.size();
	m=int(2.408*n);
	mplsh(a);
	for (j=0;j<m;j++) {
		mpsmu(a,a,10);
		s += a(0)+IAZ;
		mplsh(a);
	}
}


void NR_single_precision::mpdiv(NR_single_precision::Vec_O_UCHR &q, NR_single_precision::Vec_O_UCHR &r, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	const int MACC=1;
	int i,is,mm;

	int n=u.size();
	int m=v.size();
	int p=r.size();
	int n_min=MIN(m,p);
	if (m > n) NR::nrerror("Divisor longer than dividend in mpdiv");
	mm=m+MACC;
	Vec_UCHR s(mm),rr(mm),ss(mm+1),qq(n-m+1),t(n);
	mpinv(s,v);
	mpmul(rr,s,u);
	mpsad(ss,rr,1);
	mplsh(ss);
	mplsh(ss);
	mpmov(qq,ss);
	mpmov(q,qq);
	mpmul(t,qq,v);
	mplsh(t);
	mpsub(is,t,u,t);
	if (is != 0) NR::nrerror("MACC too small in mpdiv");
	for (i=0;i<n_min;i++) r(i)=t(i+n-m);
	if (p>m)
		for (i=m;i<p;i++) r(i)=0;
}


void NR_single_precision::mpinv(NR_single_precision::Vec_O_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	const int MF=4;
	const NR_single_precision::DP BI=1.0/256.0;
	int i,j,mm;
	NR_single_precision::DP fu,fv;

	int n=u.size();
	int m=v.size();
	Vec_UCHR s(n+m),r(2*n+m);
	mm=MIN(MF,m);
	fv=NR_single_precision::DP(v(mm-1));
	for (j=mm-2;j>=0;j--) {
		fv *= BI;
		fv += v(j);
	}
	fu=1.0/fv;
	for (j=0;j<n;j++) {
		i=int(fu);
		u(j)=(unsigned char) i;
		fu=256.0*(fu-i);
	}
	for (;;) {
		mpmul(s,u,v);
		mplsh(s);
		mpneg(s);
		s(0) += (unsigned char) 2;
		mpmul(r,s,u);
		mplsh(r);
		mpmov(u,r);
		for (j=1;j<n-1;j++)
			if (s(j) != 0) break;
		if (j==n-1) return;
	}
}


void NR_single_precision::mpmul(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	const NR_single_precision::DP RX=256.0;
	int j,n_max,nn=1;
	NR_single_precision::DP cy,t;

	int n=u.size();
	int m=v.size();
	int p=w.size();
	n_max=MAX(m,n);
	while (nn < n_max) nn <<= 1;
	nn <<= 1;
	NR_single_precision::Vec_DP a(0.0,nn),b(0.0,nn);
	for (j=0;j<n;j++) a(j)=u(j);
	for (j=0;j<m;j++) b(j)=v(j);
	realft(a,1);
	realft(b,1);
	b(0) *= a(0);
	b(1) *= a(1);
	for (j=2;j<nn;j+=2) {
		b(j)=(t=b(j))*a(j)-b(j+1)*a(j+1);
		b(j+1)=t*a(j+1)+b(j+1)*a(j);
	}
	realft(b,-1);
	cy=0.0;
	for (j=nn-1;j>=0;j--) {
		t=b(j)/(nn >> 1)+cy+0.5;
		cy=(unsigned long) (t/RX);
		b(j)=t-cy*RX;
	}
	if (cy >= RX) NR::nrerror("cannot happen in mpmul");
	for (j=0;j<p;j++) w(j)=0;
	w(0)=(unsigned char) cy;
	for (j=1;j<MIN(n+m,p);j++)
		w(j)=(unsigned char) b(j-1);
}



namespace {
	unsigned char lobyte(unsigned short x)
	{return (x & 0xff);}

	unsigned char hibyte(unsigned short x)
	{return ((x >> 8) & 0xff);}
}

void NR_single_precision::mpadd(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	int j,n_min,p_min;
	unsigned short ireg=0;

	int n=u.size();
	int m=v.size();
	int p=w.size();
	n_min=MIN(n,m);
	p_min=MIN(n_min,p-1);
	for (j=p_min-1;j>=0;j--) {
		ireg=u(j)+v(j)+hibyte(ireg);
		w(j+1)=lobyte(ireg);
	}
	w(0)=hibyte(ireg);
	if (p > p_min+1)
		for (j=p_min+1;j<p;j++) w(j)=0;
}

void NR_single_precision::mpsub(int &is, NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	int j,n_min,p_min;
	unsigned short ireg=256;

	int n=u.size();
	int m=v.size();
	int p=w.size();
	n_min=MIN(n,m);
	p_min=MIN(n_min,p);
	for (j=p_min-1;j>=0;j--) {
		ireg=255+u(j)-v(j)+hibyte(ireg);
		w(j)=lobyte(ireg);
	}
	is=hibyte(ireg)-1;
	if (p > p_min)
		for (j=p_min;j<p;j++) w(j)=0;
}

void NR_single_precision::mpsad(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, const int iv)
{
	int j;
	unsigned short ireg;

	int n=u.size();
	int p=w.size();
	ireg=256*iv;
	for (j=n-1;j>=0;j--) {
		ireg=u(j)+hibyte(ireg);
		if (j+1 < p) w(j+1)=lobyte(ireg);
	}
	w(0)=hibyte(ireg);
	for (j=n+1;j<p;j++) w(j)=0;
}

void NR_single_precision::mpsmu(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, const int iv)
{
	int j;
	unsigned short ireg=0;

	int n=u.size();
	int p=w.size();
	for (j=n-1;j>=0;j--) {
		ireg=u(j)*iv+hibyte(ireg);
		if (j < p-1) w(j+1)=lobyte(ireg);
	}
	w(0)=hibyte(ireg);
	for (j=n+1;j<p;j++) w(j)=0;
}

void NR_single_precision::mpsdv(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, const int iv, int &ir)
{
	int i,j,p_min;

	int n=u.size();
	int p=w.size();
	p_min=MIN(n,p);
	ir=0;
	for (j=0;j<p_min;j++) {
		i=256*ir+u(j);
		w(j)=(unsigned char) (i/iv);
		ir=i % iv;
	}
	if (p > p_min)
		for (j=p_min;j<p;j++) w(j)=0;
}

void NR_single_precision::mpneg(NR_single_precision::Vec_IO_UCHR &u)
{
	int j;
	unsigned short ireg=256;

	int n=u.size();
	for (j=n-1;j>=0;j--) {
		ireg=255-u(j)+hibyte(ireg);
		u(j)=lobyte(ireg);
	}
}

void NR_single_precision::mpmov(NR_single_precision::Vec_O_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	int j,n_min;

	int n=u.size();
	int m=v.size();
	n_min=MIN(n,m);
	for (j=0;j<n_min;j++) u(j)=v(j);
	if (n > n_min)
		for(j=n_min;j<n-1;j++) u(j)=0;
}

void NR_single_precision::mplsh(NR_single_precision::Vec_IO_UCHR &u)
{
	int j;

	int n=u.size();
	for (j=0;j<n-1;j++) u(j)=u(j+1);
	u(n-1)=0;
}
#include <iostream>
#include <iomanip>



void NR_single_precision::mppi(const int np)
{
	const unsigned int IAOFF=48,MACC=2;
	int ir,j,n;
	unsigned char mm;
	string s;

	n=np+MACC;
	Vec_UCHR x(n),y(n),sx(n),sxi(n);
	Vec_UCHR z(n),t(n),pi(n);
	Vec_UCHR ss(2*n),tt(2*n);
	t(0)=2;
	for (j=1;j<n;j++) t(j)=0;
	mpsqrt(x,x,t);
	mpadd(pi,t,x);
	mplsh(pi);
	mpsqrt(sx,sxi,x);
	mpmov(y,sx);
	for (;;) {
		mpadd(z,sx,sxi);
		mplsh(z);
		mpsdv(x,z,2,ir);
		mpsqrt(sx,sxi,x);
		mpmul(tt,y,sx);
		mplsh(tt);
		mpadd(tt,tt,sxi);
		mplsh(tt);
		x(0)++;
		y(0)++;
		mpinv(ss,y);
		mpmul(y,tt,ss);
		mplsh(y);
		mpmul(tt,x,ss);
		mplsh(tt);
		mpmul(ss,pi,tt);
		mplsh(ss);
		mpmov(pi,ss);
		mm=tt(0)-1;
		for (j=1;j < n-1;j++)
			if (tt(j) != mm) break;
		if (j == n-1) {
			cout << endl << "pi        = ";
			s=pi(0)+IAOFF;
			s += '.';
			mp2dfr(pi,s);
			s.erase(2.408*np,s.length());
			cout << left << s << endl;
			return;
		}
	}
}


void NR_single_precision::mprove(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &alud, NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_I_DP &b,
	NR_single_precision::Vec_IO_DP &x)
{
	int i,j;

	int n=x.size();
	NR_single_precision::Vec_DP r(n);
	for (i=0;i<n;i++) {
		long double sdp = -b(i);
		for (j=0;j<n;j++)
			sdp += (long double) a(i,j)*(long double) x(j);
		r(i)=sdp;
	}
	lubksb(alud,indx,r);
	for (i=0;i<n;i++) x(i) -= r(i);
}




void NR_single_precision::mpsqrt(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_O_UCHR &u, NR_single_precision::Vec_I_UCHR &v)
{
	const int MF=3;
	const NR_single_precision::DP BI=1.0/256.0;
	int i,ir,j,mm;
	NR_single_precision::DP fu,fv;

	int n=u.size();
	int m=v.size();
	Vec_UCHR r(2*n),x(n+m),s(2*n+m),t(3*n+m);
	mm=MIN(m,MF);
	fv=NR_single_precision::DP(v(mm-1));
	for (j=mm-2;j>=0;j--) {
		fv *= BI;
		fv += v(j);
	}
	fu=1.0/sqrt(fv);
	for (j=0;j<n;j++) {
		i=int(fu);
		u(j)=(unsigned char) i;
		fu=256.0*(fu-i);
	}
	for (;;) {
		mpmul(r,u,u);
		mplsh(r);
		mpmul(s,r,v);
		mplsh(s);
		mpneg(s);
		s(0) += (unsigned char) 3;
		mpsdv(s,s,2,ir);
		for (j=1;j<n-1;j++) {
			if (s(j) != 0) {
				mpmul(t,s,u);
				mplsh(t);
				mpmov(u,t);
				break;
			}
		}
		if (j<n-1) continue;
		mpmul(x,u,v);
		mplsh(x);
		mpmov(w,x);
		return;
	}
}


void NR_single_precision::mrqcof(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_I_DP &a,
	NR_single_precision::Vec_I_BOOL &ia, NR_single_precision::Mat_O_DP &alpha, NR_single_precision::Vec_O_DP &beta, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &,NR_single_precision::DP &, NR_single_precision::Vec_O_DP &))
{
	int i,j,k,l,m,mfit=0;
	NR_single_precision::DP ymod,wt,sig2i,dy;

	int ndata=x.size();
	int ma=a.size();
	NR_single_precision::Vec_DP dyda(ma);
	for (j=0;j<ma;j++)
		if (ia(j)) mfit++;
	for (j=0;j<mfit;j++) {
		for (k=0;k<=j;k++) alpha(j,k)=0.0;
		beta(j)=0.0;
	}
	chisq=0.0;
	for (i=0;i<ndata;i++) {
		funcs(x(i),a,ymod,dyda);
		sig2i=1.0/(sig(i)*sig(i));
		dy=y(i)-ymod;
		for (j=0,l=0;l<ma;l++) {
			if (ia(l)) {
				wt=dyda(l)*sig2i;
				for (k=0,m=0;m<l+1;m++)
					if (ia(m)) alpha(j,k++) += wt*dyda(m);
				beta(j++) += dy*wt;
			}
		}
		chisq += dy*dy*sig2i;
	}
	for (j=1;j<mfit;j++)
		for (k=0;k<j;k++) alpha(k,j)=alpha(j,k);
}


void NR_single_precision::mrqmin(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_IO_DP &a,
	NR_single_precision::Vec_I_BOOL &ia, NR_single_precision::Mat_O_DP &covar, NR_single_precision::Mat_O_DP &alpha, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::DP &, NR_single_precision::Vec_O_DP &), NR_single_precision::DP &alamda)
{
	static int mfit;
	static NR_single_precision::DP ochisq;
	int j,k,l;

	int ma=a.size();
	static NR_single_precision::Mat_DP oneda(ma,1);
	static NR_single_precision::Vec_DP atry(ma),beta(ma),da(ma);
	if (alamda < 0.0) {
		mfit=0;
		for (j=0;j<ma;j++)
			if (ia(j)) mfit++;
		alamda=0.001;
		mrqcof(x,y,sig,a,ia,alpha,beta,chisq,funcs);
		ochisq=chisq;
		for (j=0;j<ma;j++) atry(j)=a(j);
	}
	NR_single_precision::Mat_DP temp(mfit,mfit);
	for (j=0;j<mfit;j++) {
		for (k=0;k<mfit;k++) covar(j,k)=alpha(j,k);
		covar(j,j)=alpha(j,j)*(1.0+alamda);
		for (k=0;k<mfit;k++) temp(j,k)=covar(j,k);
		oneda(j,0)=beta(j);
	}
	gaussj(temp,oneda);
	for (j=0;j<mfit;j++) {
		for (k=0;k<mfit;k++) covar(j,k)=temp(j,k);
		da(j)=oneda(j,0);
	}
	if (alamda == 0.0) {
		covsrt(covar,ia,mfit);
		covsrt(alpha,ia,mfit);
		return;
	}
	for (j=0,l=0;l<ma;l++)
		if (ia(l)) atry(l)=a(l)+da(j++);
	mrqcof(x,y,sig,atry,ia,covar,da,chisq,funcs);
	if (chisq < ochisq) {
		alamda *= 0.1;
		ochisq=chisq;
		for (j=0;j<mfit;j++) {
			for (k=0;k<mfit;k++) alpha(j,k)=covar(j,k);
				beta(j)=da(j);
		}
		for (l=0;l<ma;l++) a(l)=atry(l);
	} else {
		alamda *= 10.0;
		chisq=ochisq;
	}
}

#include <limits>



//NR_single_precision::Vec_DP *fvec_p;
//void (*nrfuncv)(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f);

void NR_single_precision::newt(NR_single_precision::Vec_IO_DP &x, bool &check, void vecfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const int MAXITS=200;
	const NR_single_precision::DP TOLF=1.0e-8,TOLMIN=1.0e-12,STPMX=100.0;
	const NR_single_precision::DP TOLX=DBL_EPSILON;
	int i,j,its;
	NR_single_precision::DP d,den,f,fold,stpmax,sum,temp,test;

	int n=x.size();
	NR_single_precision::Vec_INT indx(n);
	NR_single_precision::Vec_DP g(n),p(n),xold(n);
	NR_single_precision::Mat_DP fjac(n,n);
	fvec_p=new NR_single_precision::Vec_DP(n);
	nrfuncv=vecfunc;
	NR_single_precision::Vec_DP &fvec=*fvec_p;
	f=fmin(x);
	test=0.0;
	for (i=0;i<n;i++)
		if (fabs(fvec(i)) > test) test=fabs(fvec(i));
	if (test < 0.01*TOLF) {
		check=false;
		delete fvec_p;
		return;
	}
	sum=0.0;
	for (i=0;i<n;i++) sum += SQR(x(i));
	stpmax=STPMX*MAX(sqrt(sum),NR_single_precision::DP(n));
	for (its=0;its<MAXITS;its++) {
		fdjac(x,fvec,fjac,vecfunc);
		for (i=0;i<n;i++) {
			sum=0.0;
			for (j=0;j<n;j++) sum += fjac(j,i)*fvec(j);
			g(i)=sum;
		}
		for (i=0;i<n;i++) xold(i)=x(i);
		fold=f;
		for (i=0;i<n;i++) p(i) = -fvec(i);
		ludcmp(fjac,indx,d);
		lubksb(fjac,indx,p);
		lnsrch(xold,fold,g,p,x,f,stpmax,check,fmin);
		test=0.0;
		for (i=0;i<n;i++)
			if (fabs(fvec(i)) > test) test=fabs(fvec(i));
		if (test < TOLF) {
			check=false;
			delete fvec_p;
			return;
		}
		if (check) {
			test=0.0;
			den=MAX(f,0.5*n);
			for (i=0;i<n;i++) {
				temp=fabs(g(i))*MAX(fabs(x(i)),1.0)/den;
				if (temp > test) test=temp;
			}
			check=(test < TOLMIN);
			delete fvec_p;
			return;
		}
		test=0.0;
		for (i=0;i<n;i++) {
			temp=(fabs(x(i)-xold(i)))/MAX(fabs(x(i)),1.0);
			if (temp > test) test=temp;
		}
		if (test < TOLX) {
			delete fvec_p;
			return;
		}
	}
	NR::nrerror("MAXITS exceeded in newt");
}

#include <iostream>


// Function used to stall exit in Borland C++Builder

void nrexit(void)
{
        cin.get();
        return;
}

#pragma exit nrexit



























#include <iostream>
#include <iomanip>

#include <cstdlib>

#include <limits>

extern NR_single_precision::DP dxsav;
extern int kmax,kount;
extern NR_single_precision::Vec_DP *xp_p;
extern NR_single_precision::Mat_DP *yp_p;

void NR_single_precision::odeint(NR_single_precision::Vec_IO_DP &ystart, const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP eps,
	const NR_single_precision::DP h1, const NR_single_precision::DP hmin, int &nok, int &nbad,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &),
	void rkqs(NR_single_precision::Vec_IO_DP &, NR_single_precision::Vec_IO_DP &, NR_single_precision::DP &, const NR_single_precision::DP, const NR_single_precision::DP,
	NR_single_precision::Vec_I_DP &, NR_single_precision::DP &, NR_single_precision::DP &, void (*)(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &)))
{
	const int MAXSTP=10000;
	const NR_single_precision::DP TINY=1.0e-30;
	int i,nstp;
	NR_single_precision::DP xsav,x,hnext,hdid,h;

	int nvar=ystart.size();
	NR_single_precision::Vec_DP yscal(nvar),y(nvar),dydx(nvar);
	NR_single_precision::Vec_DP &xp=*xp_p;
	NR_single_precision::Mat_DP &yp=*yp_p;
	x=x1;
	h=SIGN(h1,x2-x1);
	nok = nbad = kount = 0;
	for (i=0;i<nvar;i++) y(i)=ystart(i);
	if (kmax > 0) xsav=x-dxsav*2.0;
	for (nstp=0;nstp<MAXSTP;nstp++) {
		derivs(x,y,dydx);
		for (i=0;i<nvar;i++)
			yscal(i)=fabs(y(i))+fabs(dydx(i)*h)+TINY;
		if (kmax > 0 && kount < kmax-1 && fabs(x-xsav) > fabs(dxsav)) {
			for (i=0;i<nvar;i++) yp(i,kount)=y(i);
			xp(kount++)=x;
			xsav=x;
		}
		if ((x+h-x2)*(x+h-x1) > 0.0) h=x2-x;
		rkqs(y,dydx,x,h,eps,yscal,hdid,hnext,derivs);
		if (hdid == h) ++nok; else ++nbad;
		if ((x-x2)*(x2-x1) >= 0.0) {
			for (i=0;i<nvar;i++) ystart(i)=y(i);
			if (kmax != 0) {
				for (i=0;i<nvar;i++) yp(i,kount)=y(i);
				xp(kount++)=x;
			}
			return;
		}
		if (fabs(hnext) <= hmin) NR::nrerror("Step size too small in odeint");
		h=hnext;
	}
	NR::nrerror("Too many steps in routine odeint");
}


void NR_single_precision::orthog(NR_single_precision::Vec_I_DP &anu, NR_single_precision::Vec_I_DP &alpha, NR_single_precision::Vec_I_DP &beta, NR_single_precision::Vec_O_DP &a,
	NR_single_precision::Vec_O_DP &b)
{
	int k,l,looptmp;

	int n=a.size();
	NR_single_precision::Mat_DP sig(2*n+1,2*n+1);
	looptmp=2*n;
	for (l=2;l<looptmp;l++) sig(0,l)=0.0;
	looptmp++;
	for (l=1;l<looptmp;l++) sig(1,l)=anu(l-1);
	a(0)=alpha(0)+anu(1)/anu(0);
	b(0)=0.0;
	for (k=2;k<n+1;k++) {
		looptmp=2*n-k+2;
		for (l=k;l<looptmp;l++) {
			sig(k,l)=sig(k-1,l+1)+(alpha(l-1)-a(k-2))*sig(k-1,l)
				-b(k-2)*sig(k-2,l)+beta(l-1)*sig(k-1,l-1);
		}
		a(k-1)=alpha(k-1)+sig(k,k+1)/sig(k,k)-sig(k-1,k)/sig(k-1,k-1);
		b(k-1)=sig(k,k)/sig(k-1,k-1);
	}
}




void NR_single_precision::pade(NR_single_precision::Vec_IO_DP &cof, NR_single_precision::DP &resid)
{
	const NR_single_precision::DP BIG=1.0e30;
	int j,k;
	NR_single_precision::DP d,rr,rrold,sum;

	int n=(cof.size()-1)/2;
	NR_single_precision::Mat_DP q(n,n),qlu(n,n);
	NR_single_precision::Vec_INT indx(n);
	NR_single_precision::Vec_DP x(n),y(n),z(n);
	for (j=0;j<n;j++) {
		y(j)=x(j)=cof(n+j+1);
		for (k=0;k<n;k++) {
			q(j,k)=cof(j-k+n);
			qlu(j,k)=q(j,k);
		}
	}
	ludcmp(qlu,indx,d);
	lubksb(qlu,indx,x);
	rr=BIG;
	do {
		rrold=rr;
		for (j=0;j<n;j++) z(j)=x(j);
		mprove(q,qlu,indx,y,x);
		for (rr=0.0,j=0;j<n;j++)
			rr += SQR(z(j)-x(j));
	} while (rr < rrold);
	resid=sqrt(rrold);
	for (k=0;k<n;k++) {
		for (sum=cof(k+1),j=0;j<=k;j++)
			sum -= z(j)*cof(k-j);
		y(k)=sum;
	}
	for (j=0;j<n;j++) {
		cof(j+1)=y(j);
		cof(j+1+n) = -z(j);
	}
}


void NR_single_precision::pccheb(NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_O_DP &c)
{
	int j,jm,jp,k;
	NR_single_precision::DP fac,pow;

	int n=d.size();
	pow=1.0;
	c(0)=2.0*d(0);
	for (k=1;k<n;k++) {
		c(k)=0.0;
		fac=d(k)/pow;
		jm=k;
		jp=1;
		for (j=k;j>=0;j-=2,jm--,jp++) {
			c(j) += fac;
			fac *= NR_single_precision::DP(jm)/NR_single_precision::DP(jp);
		}
		pow += pow;
	}
}


void NR_single_precision::pcshft(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_IO_DP &d)
{
	int k,j;
	NR_single_precision::DP fac,cnst;

	int n=d.size();
	cnst=2.0/(b-a);
	fac=cnst;
	for (j=1;j<n;j++) {
		d(j) *= fac;
		fac *= cnst;
	}
	cnst=0.5*(a+b);
	for (j=0;j<=n-2;j++)
		for (k=n-2;k>=j;k--)
			d(k) -= cnst*d(k+1);
}




void NR_single_precision::pearsn(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::DP &r, NR_single_precision::DP &prob, NR_single_precision::DP &z)
{
	const NR_single_precision::DP TINY=1.0e-20;
	int j;
	NR_single_precision::DP yt,xt,t,df;
	NR_single_precision::DP syy=0.0,sxy=0.0,sxx=0.0,ay=0.0,ax=0.0;

	int n=x.size();
	for (j=0;j<n;j++) {
		ax += x(j);
		ay += y(j);
	}
	ax /= n;
	ay /= n;
	for (j=0;j<n;j++) {
		xt=x(j)-ax;
		yt=y(j)-ay;
		sxx += xt*xt;
		syy += yt*yt;
		sxy += xt*yt;
	}
	r=sxy/(sqrt(sxx*syy)+TINY);
	z=0.5*log((1.0+r+TINY)/(1.0-r+TINY));
	df=n-2;
	t=r*sqrt(df/((1.0-r+TINY)*(1.0+r+TINY)));
	prob=betai(0.5*df,0.5,df/(df+t*t));
	// prob=erfcc(fabs(z*sqrt(n-1.0))/1.4142136);
}




void NR_single_precision::period(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, const NR_single_precision::DP ofac, const NR_single_precision::DP hifac,
	NR_single_precision::Vec_O_DP &px, NR_single_precision::Vec_O_DP &py, int &nout, int &jmax, NR_single_precision::DP &prob)
{
	const NR_single_precision::DP TWOPI=6.283185307179586476;
	int i,j;
	NR_single_precision::DP ave,c,cc,cwtau,effm,expy,pnow,pymax,s,ss,sumc,sumcy,sums,sumsh,
		sumsy,swtau,var,wtau,xave,xdif,xmax,xmin,yy,arg,wtemp;

	int n=x.size();
	int np=px.size();
	NR_single_precision::Vec_DP wi(n),wpi(n),wpr(n),wr(n);
	nout=0.5*ofac*hifac*n;
	if (nout > np) NR::nrerror("output arrays too short in period");
	avevar(y,ave,var);
	if (var == 0.0) NR::nrerror("zero variance in period");
	xmax=xmin=x(0);
	for (j=0;j<n;j++) {
		if (x(j) > xmax) xmax=x(j);
		if (x(j) < xmin) xmin=x(j);
	}
	xdif=xmax-xmin;
	xave=0.5*(xmax+xmin);
	pymax=0.0;
	pnow=1.0/(xdif*ofac);
	for (j=0;j<n;j++) {
		arg=TWOPI*((x(j)-xave)*pnow);
		wpr(j)= -2.0*SQR(sin(0.5*arg));
		wpi(j)=sin(arg);
		wr(j)=cos(arg);
		wi(j)=wpi(j);
	}
	for (i=0;i<nout;i++) {
		px(i)=pnow;
		sumsh=sumc=0.0;
		for (j=0;j<n;j++) {
			c=wr(j);
			s=wi(j);
			sumsh += s*c;
			sumc += (c-s)*(c+s);
		}
		wtau=0.5*atan2(2.0*sumsh,sumc);
		swtau=sin(wtau);
		cwtau=cos(wtau);
		sums=sumc=sumsy=sumcy=0.0;
		for (j=0;j<n;j++) {
			s=wi(j);
			c=wr(j);
			ss=s*cwtau-c*swtau;
			cc=c*cwtau+s*swtau;
			sums += ss*ss;
			sumc += cc*cc;
			yy=y(j)-ave;
			sumsy += yy*ss;
			sumcy += yy*cc;
			wr(j)=((wtemp=wr(j))*wpr(j)-wi(j)*wpi(j))+wr(j);
			wi(j)=(wi(j)*wpr(j)+wtemp*wpi(j))+wi(j);
		}
		py(i)=0.5*(sumcy*sumcy/sumc+sumsy*sumsy/sums)/var;
		if (py(i) >= pymax) pymax=py(jmax=i);
		pnow += 1.0/(ofac*xdif);
	}
	expy=exp(-pymax);
	effm=2.0*nout/ofac;
	prob=effm*expy;
	if (prob > 0.01) prob=1.0-pow(1.0-expy,effm);
}


void NR_single_precision::piksr2(NR_single_precision::Vec_IO_DP &arr, NR_single_precision::Vec_IO_DP &brr)
{
	int i,j;
	NR_single_precision::DP a,b;

	int n=arr.size();
	for (j=1;j<n;j++) {
		a=arr(j);
		b=brr(j);
		i=j;
		while (i > 0 && arr(i-1) > a) {
			arr(i)=arr(i-1);
			brr(i)=brr(i-1);
			i--;
		}
		arr(i)=a;
		brr(i)=b;
	}
}


void NR_single_precision::piksrt(NR_single_precision::Vec_IO_DP &arr)
{
	int i,j;
	NR_single_precision::DP a;

	int n=arr.size();
	for (j=1;j<n;j++) {
		a=arr(j);
		i=j;
		while (i > 0 && arr(i-1) > a) {
			arr(i)=arr(i-1);
			i--;
		}
		arr(i)=a;
	}
}




void NR_single_precision::pinvs(const int ie1, const int ie2, const int je1, const int jsf,
	const int jc1, const int k, Mat3D_O_DP &c, NR_single_precision::Mat_IO_DP &s)
{
	int jpiv,jp,je2,jcoff,j,irow,ipiv,id,icoff,i;
	NR_single_precision::DP pivinv,piv,dum,big;

	const int iesize=ie2-ie1;
	NR_single_precision::Vec_INT indxr(iesize);
	NR_single_precision::Vec_DP pscl(iesize);
	je2=je1+iesize;
	for (i=ie1;i<ie2;i++) {
		big=0.0;
		for (j=je1;j<je2;j++)
			if (fabs(s(i,j)) > big) big=fabs(s(i,j));
		if (big == 0.0)
			NR::nrerror("Singular matrix - row all 0, in pinvs");
		pscl(i-ie1)=1.0/big;
		indxr(i-ie1)=0;
	}
	for (id=0;id<iesize;id++) {
		piv=0.0;
		for (i=ie1;i<ie2;i++) {
			if (indxr(i-ie1) == 0) {
				big=0.0;
				for (j=je1;j<je2;j++) {
					if (fabs(s(i,j)) > big) {
						jp=j;
						big=fabs(s(i,j));
					}
				}
				if (big*pscl(i-ie1) > piv) {
					ipiv=i;
					jpiv=jp;
					piv=big*pscl(i-ie1);
				}
			}
		}
		if (s(ipiv,jpiv) == 0.0)
			NR::nrerror("Singular matrix in routine pinvs");
		indxr(ipiv-ie1)=jpiv+1;
		pivinv=1.0/s(ipiv,jpiv);
		for (j=je1;j<=jsf;j++) s(ipiv,j) *= pivinv;
		s(ipiv,jpiv)=1.0;
		for (i=ie1;i<ie2;i++) {
			if (indxr(i-ie1) != jpiv+1) {
				if (s(i,jpiv) != 0.0) {
					dum=s(i,jpiv);
					for (j=je1;j<=jsf;j++)
						s(i,j) -= dum*s(ipiv,j);
					s(i,jpiv)=0.0;
				}
			}
		}
	}
	jcoff=jc1-je2;
	icoff=ie1-je1;
	for (i=ie1;i<ie2;i++) {
		irow=indxr(i-ie1)+icoff;
		for (j=je2;j<=jsf;j++) c(irow-1,j+jcoff,k)=s(i,j);
	}
}




NR_single_precision::DP NR_single_precision::plgndr(const int l, const int m, const NR_single_precision::DP x)
{
	int i,ll;
	NR_single_precision::DP fact,pll,pmm,pmmp1,somx2;

	if (m < 0 || m > l || fabs(x) > 1.0)
		NR::nrerror("Bad arguments in routine plgndr");
	pmm=1.0;
	if (m > 0) {
		somx2=sqrt((1.0-x)*(1.0+x));
		fact=1.0;
		for (i=1;i<=m;i++) {
			pmm *= -fact*somx2;
			fact += 2.0;
		}
	}
	if (l == m)
		return pmm;
	else {
		pmmp1=x*(2*m+1)*pmm;
		if (l == (m+1))
			return pmmp1;
		else {
			for (ll=m+2;ll<=l;ll++) {
				pll=(x*(2*ll-1)*pmmp1-(ll+m-1)*pmm)/(ll-m);
				pmm=pmmp1;
				pmmp1=pll;
			}
			return pll;
		}
	}
}




NR_single_precision::DP NR_single_precision::poidev(const NR_single_precision::DP xm, int &idum)
{
	const NR_single_precision::DP PI=3.141592653589793238;
	static NR_single_precision::DP sq,alxm,g,oldm=(-1.0);
	NR_single_precision::DP em,t,y;

	if (xm < 12.0) {
		if (xm != oldm) {
			oldm=xm;
			g=exp(-xm);
		}
		em = -1;
		t=1.0;
		do {
			++em;
			t *= ran1(idum);
		} while (t > g);
	} else {
		if (xm != oldm) {
			oldm=xm;
			sq=sqrt(2.0*xm);
			alxm=log(xm);
			g=xm*alxm-gammln(xm+1.0);
		}
		do {
			do {
				y=tan(PI*ran1(idum));
				em=sq*y+xm;
			} while (em < 0.0);
				em=floor(em);
				t=0.9*(1.0+y*y)*exp(em*alxm-gammln(em+1.0)-g);
		} while (ran1(idum) > t);
	}
	return em;
}


void NR_single_precision::polcoe(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &cof)
{
	int k,j,i;
	NR_single_precision::DP phi,ff,b;

	int n=x.size();
	NR_single_precision::Vec_DP s(n);
	for (i=0;i<n;i++) s(i)=cof(i)=0.0;
	s(n-1)= -x(0);
	for (i=1;i<n;i++) {
		for (j=n-1-i;j<n-1;j++)
			s(j) -= x(i)*s(j+1);
		s(n-1) -= x(i);
	}
	for (j=0;j<n;j++) {
		phi=n;
		for (k=n-1;k>0;k--)
			phi=k*s(k)+x(j)*phi;
		ff=y(j)/phi;
		b=1.0;
		for (k=n-1;k>=0;k--) {
			cof(k) += b*ff;
			b=s(k)+x(j)*b;
		}
	}
}




void NR_single_precision::polcof(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, NR_single_precision::Vec_O_DP &cof)
{
	int k,j,i;
	NR_single_precision::DP xmin,dy;

	int n=xa.size();
	NR_single_precision::Vec_DP x(n),y(n);
	for (j=0;j<n;j++) {
		x(j)=xa(j);
		y(j)=ya(j);
	}
	for (j=0;j<n;j++) {
		NR_single_precision::Vec_DP x_t(n-j),y_t(n-j);
		for (k=0;k<n-j;k++) {
			x_t(k)=x(k);
			y_t(k)=y(k);
		}
		polint(x_t,y_t,0.0,cof(j),dy);
		xmin=1.0e38;
		k = -1;
		for (i=0;i<n-j;i++) {
			if (fabs(x(i)) < xmin) {
				xmin=fabs(x(i));
				k=i;
			}
			if (x(i) != 0.0)
				y(i)=(y(i)-cof(j))/x(i);
		}
		for (i=k+1;i<n-j;i++) {
			y(i-1)=y(i);
			x(i-1)=x(i);
		}
	}
}


void NR_single_precision::poldiv(NR_single_precision::Vec_I_DP &u, NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &q, NR_single_precision::Vec_O_DP &r)
{
	int k,j;

	int n=u.size()-1;
	int nv=v.size()-1;
	for (j=0;j<=n;j++) {
		r(j)=u(j);
		q(j)=0.0;
	}
	for (k=n-nv;k>=0;k--) {
		q(k)=r(nv+k)/v(nv);
		for (j=nv+k-1;j>=k;j--) r(j) -= q(k)*v(j-k);
	}
	for (j=nv;j<=n;j++) r(j)=0.0;
}


void NR_single_precision::polin2(NR_single_precision::Vec_I_DP &x1a, NR_single_precision::Vec_I_DP &x2a, NR_single_precision::Mat_I_DP &ya, const NR_single_precision::DP x1,
	const NR_single_precision::DP x2, NR_single_precision::DP &y, NR_single_precision::DP &dy)
{
	int j,k;

	int m=x1a.size();
	int n=x2a.size();
	NR_single_precision::Vec_DP ymtmp(m),ya_t(n);
	for (j=0;j<m;j++) {
		for (k=0;k<n;k++) ya_t(k)=ya(j,k);
		polint(x2a,ya_t,x2,ymtmp(j),dy);
	}
	polint(x1a,ymtmp,x1,y,dy);
}




void NR_single_precision::polint(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, const NR_single_precision::DP x, NR_single_precision::DP &y, NR_single_precision::DP &dy)
{
	int i,m,ns=0;
	NR_single_precision::DP den,dif,dift,ho,hp,w;

	int n=xa.size();
	NR_single_precision::Vec_DP c(n),d(n);
	dif=fabs(x-xa(0));
	for (i=0;i<n;i++) {
		if ((dift=fabs(x-xa(i))) < dif) {
			ns=i;
			dif=dift;
		}
		c(i)=ya(i);
		d(i)=ya(i);
	}
	y=ya(ns--);
	for (m=1;m<n;m++) {
		for (i=0;i<n-m;i++) {
			ho=xa(i)-x;
			hp=xa(i+m)-x;
			w=c(i+1)-d(i);
			if ((den=ho-hp) == 0.0) NR::nrerror("Error in routine polint");
			den=w/den;
			d(i)=hp*den;
			c(i)=ho*den;
		}
		y += (dy=(2*(ns+1) < (n-m) ? c(ns+1) : d(ns--)));
	}
}




void NR_single_precision::powell(NR_single_precision::Vec_IO_DP &p, NR_single_precision::Mat_IO_DP &xi, const NR_single_precision::DP ftol, int &iter,
	NR_single_precision::DP &fret, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &))
{
	const int ITMAX=200;
	const NR_single_precision::DP TINY=1.0e-25;
	int i,j,ibig;
	NR_single_precision::DP del,fp,fptt,t;

	int n=p.size();
	NR_single_precision::Vec_DP pt(n),ptt(n),xit(n);
	fret=func(p);
	for (j=0;j<n;j++) pt(j)=p(j);
	for (iter=0;;++iter) {
		fp=fret;
		ibig=0;
		del=0.0;
		for (i=0;i<n;i++) {
			for (j=0;j<n;j++) xit(j)=xi(j,i);
			fptt=fret;
			linmin(p,xit,fret,func);
			if (fptt-fret > del) {
				del=fptt-fret;
				ibig=i+1;
			}
		}
		if (2.0*(fp-fret) <= ftol*(fabs(fp)+fabs(fret))+TINY) {
			return;
		}
		if (iter == ITMAX) NR::nrerror("powell exceeding maximum iterations.");
		for (j=0;j<n;j++) {
			ptt(j)=2.0*p(j)-pt(j);
			xit(j)=p(j)-pt(j);
			pt(j)=p(j);
		}
		fptt=func(ptt);
		if (fptt < fp) {
			t=2.0*(fp-2.0*fret+fptt)*SQR(fp-fret-del)-del*SQR(fp-fptt);
			if (t < 0.0) {
				linmin(p,xit,fret,func);
				for (j=0;j<n;j++) {
					xi(j,ibig-1)=xi(j,n-1);
					xi(j,n-1)=xit(j);
				}
			}
		}
	}
}


void NR_single_precision::predic(NR_single_precision::Vec_I_DP &data, NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_O_DP &future)
{
	int k,j;
	NR_single_precision::DP sum,discrp;

	int ndata=data.size();
	int m=d.size();
	int nfut=future.size();
	NR_single_precision::Vec_DP reg(m);
	for (j=0;j<m;j++) reg(j)=data(ndata-1-j);
	for (j=0;j<nfut;j++) {
		discrp=0.0;
		sum=discrp;
		for (k=0;k<m;k++) sum += d(k)*reg(k);
		for (k=m-1;k>=1;k--) reg(k)=reg(k-1);
		future(j)=reg(0)=sum;
	}
}




NR_single_precision::DP NR_single_precision::probks(const NR_single_precision::DP alam)
{
	const NR_single_precision::DP EPS1=1.0e-6,EPS2=1.0e-16;
	int j;
	NR_single_precision::DP a2,fac=2.0,sum=0.0,term,termbf=0.0;

	a2 = -2.0*alam*alam;
	for (j=1;j<=100;j++) {
		term=fac*exp(a2*j*j);
		sum += term;
		if (fabs(term) <= EPS1*termbf || fabs(term) <= EPS2*sum) return sum;
		fac = -fac;
		termbf=fabs(term);
	}
	return 1.0;
}


void NR_single_precision::psdes(unsigned long &lword, unsigned long &irword)
{
	Msg::error("psdes");
	/*
	const int NITER=4;
	static const unsigned long c1(NITER)={
		0xbaa96887L, 0x1e17d32cL, 0x03bcdc3cL, 0x0f33d1b2L};
	static const unsigned long c2(NITER)={
		0x4b0f3b58L, 0xe874f0c3L, 0x6955c5a6L, 0x55a7ca46L};
	unsigned long i,ia,ib,iswap,itmph=0,itmpl=0;

	for (i=0;i<NITER;i++) {
		ia=(iswap=irword) ^ c1(i);
		itmpl = ia & 0xffff;
		itmph = ia >> 16;
		ib=itmpl*itmpl+ ~(itmph*itmph);
		irword=lword ^ (((ia = (ib >> 16) |
			((ib & 0xffff) << 16)) ^ c2(i))+itmpl*itmph);
		lword=iswap;
	}*/
}


extern wavefilt *wfilt_p;

void NR_single_precision::pwt(NR_single_precision::Vec_IO_DP &a, const int n, const int isign)
{
	Msg::error("pwt");
	/*
	NR_single_precision::DP ai,ai1;
	int i,ii,j,jf,jr,k,n1,ni,nj,nh,nmod;

	if (n < 4) return;
	wavefilt &wfilt=*wfilt_p;
	NR_single_precision::Vec_DP wksp(n);
	nmod=wfilt.ncof*n;
	n1=n-1;
	nh=n >> 1;
	for (j=0;j<n;j++) wksp(j)=0.0;
	if (isign >= 0) {
		for (ii=0,i=0;i<n;i+=2,ii++) {
			ni=i+1+nmod+wfilt.ioff;
			nj=i+1+nmod+wfilt.joff;
			for (k=0;k<wfilt.ncof;k++) {
				jf=n1 & (ni+k+1);
				jr=n1 & (nj+k+1);
				wksp(ii) += wfilt.cc(k)*a(jf);
				wksp(ii+nh) += wfilt.cr(k)*a(jr);
			}
		}
	} else {
		for (ii=0,i=0;i<n;i+=2,ii++) {
			ai=a(ii);
			ai1=a(ii+nh);
			ni=i+1+nmod+wfilt.ioff;
			nj=i+1+nmod+wfilt.joff;
			for (k=0;k<wfilt.ncof;k++) {
				jf=n1 & (ni+k+1);
				jr=n1 & (nj+k+1);
				wksp(jf) += wfilt.cc(k)*ai;
				wksp(jr) += wfilt.cr(k)*ai1;
			}
		}
	}
	for (j=0;j<n;j++) a(j)=wksp(j);*/

}


wavefilt *wfilt_p;

void NR_single_precision::pwtset(const int n)
{
	Msg::error("pwtset");
	/*
	const static NR_single_precision::DP c4_d(4)=
		{0.4829629131445341,0.8365163037378079,
		0.2241438680420134,-0.1294095225512604};
	const static NR_single_precision::DP c12_d(12)=
		{0.111540743350, 0.494623890398, 0.751133908021,
		0.315250351709,-0.226264693965,-0.129766867567,
		0.097501605587, 0.027522865530,-0.031582039318,
		0.000553842201, 0.004777257511,-0.001077301085};
	const static NR_single_precision::DP c20_d(20)=
		{0.026670057901, 0.188176800078, 0.527201188932,
		0.688459039454, 0.281172343661,-0.249846424327,
		-0.195946274377, 0.127369340336, 0.093057364604,
		-0.071394147166,-0.029457536822, 0.033212674059,
		0.003606553567,-0.010733175483, 0.001395351747,
		0.001992405295,-0.000685856695,-0.000116466855,
		0.000093588670,-0.000013264203};

	if (n == 4)
		wfilt_p=new wavefilt(c4_d,n);
	else if (n == 12)
		wfilt_p=new wavefilt(c12_d,n);
	else if (n == 20)
		wfilt_p=new wavefilt(c20_d,n);
	else NR::nrerror("unimplemented value n in pwtset");*/
}




NR_single_precision::DP NR_single_precision::pythag(const NR_single_precision::DP a, const NR_single_precision::DP b)
{
	NR_single_precision::DP absa,absb;

	absa=fabs(a);
	absb=fabs(b);
	if (absa > absb) return absa*sqrt(1.0+SQR(absb/absa));
	else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
}


extern NR_single_precision::Vec_DP *x_p;
extern NR_single_precision::Mat_DP *d_p;

void NR_single_precision::pzextr(const int iest, const NR_single_precision::DP xest, NR_single_precision::Vec_I_DP &yest, NR_single_precision::Vec_O_DP &yz,
	NR_single_precision::Vec_O_DP &dy)
{
	int j,k1;
	NR_single_precision::DP q,f2,f1,delta;

	int nv=yz.size();
	NR_single_precision::Vec_DP c(nv);
	NR_single_precision::Vec_DP &x=*x_p;
	NR_single_precision::Mat_DP &d=*d_p;
	x(iest)=xest;
	for (j=0;j<nv;j++) dy(j)=yz(j)=yest(j);
	if (iest == 0) {
		for (j=0;j<nv;j++) d(j,0)=yest(j);
	} else {
		for (j=0;j<nv;j++) c(j)=yest(j);
		for (k1=0;k1<iest;k1++) {
			delta=1.0/(x(iest-k1-1)-xest);
			f1=xest*delta;
			f2=x(iest-k1-1)*delta;
			for (j=0;j<nv;j++) {
				q=d(j,k1);
				d(j,k1)=dy(j);
				delta=c(j)-q;
				dy(j)=f1*delta;
				c(j)=f2*delta;
				yz(j) += dy(j);
			}
		}
		for (j=0;j<nv;j++) d(j,iest)=dy(j);
	}
}


NR_single_precision::DP NR_single_precision::qgaus(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b)
{
	Msg::error("qgaus");
	/*
	static const NR_single_precision::DP x()={0.1488743389816312,0.4333953941292472,
		0.6794095682990244,0.8650633666889845,0.9739065285171717};
	static const NR_single_precision::DP w()={0.2955242247147529,0.2692667193099963,
		0.2190863625159821,0.1494513491505806,0.0666713443086881};
	int j;
	NR_single_precision::DP xr,xm,dx,s;

	xm=0.5*(b+a);
	xr=0.5*(b-a);
	s=0;
	for (j=0;j<5;j++) {
		dx=xr*x(j);
		s += w(j)*(func(xm+dx)+func(xm-dx));
	}
	return s *= xr;*/
	return 0.0;
}




void NR_single_precision::qrdcmp(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &c, NR_single_precision::Vec_O_DP &d, bool &sing)
{
	int i,j,k;
	NR_single_precision::DP scale,sigma,sum,tau;

	int n=a.rows();
	sing=false;
	for (k=0;k<n-1;k++) {
		scale=0.0;
		for (i=k;i<n;i++) scale=MAX(scale,fabs(a(i,k)));
		if (scale == 0.0) {
			sing=true;
			c(k)=d(k)=0.0;
		} else {
			for (i=k;i<n;i++) a(i,k) /= scale;
			for (sum=0.0,i=k;i<n;i++) sum += SQR(a(i,k));
			sigma=SIGN(sqrt(sum),a(k,k));
			a(k,k) += sigma;
			c(k)=sigma*a(k,k);
			d(k) = -scale*sigma;
			for (j=k+1;j<n;j++) {
				for (sum=0.0,i=k;i<n;i++) sum += a(i,k)*a(i,j);
				tau=sum/c(k);
				for (i=k;i<n;i++) a(i,j) -= tau*a(i,k);
			}
		}
	}
	d(n-1)=a(n-1,n-1);
	if (d(n-1) == 0.0) sing=true;
}




NR_single_precision::DP NR_single_precision::qromb(NR_single_precision::DP func(const NR_single_precision::DP), NR_single_precision::DP a, NR_single_precision::DP b)
{
	const int JMAX=20, JMAXP=JMAX+1, K=5;
	const NR_single_precision::DP EPS=1.0e-10;
	NR_single_precision::DP ss,dss;
	NR_single_precision::Vec_DP s(JMAX),h(JMAXP),s_t(K),h_t(K);
	int i,j;

	h(0)=1.0;
	for (j=1;j<=JMAX;j++) {
		s(j-1)=trapzd(func,a,b,j);
		if (j >= K) {
			for (i=0;i<K;i++) {
				h_t(i)=h(j-K+i);
				s_t(i)=s(j-K+i);
			}
			polint(h_t,s_t,0.0,ss,dss);
			if (fabs(dss) <= EPS*fabs(ss)) return ss;
		}
		h(j)=0.25*h(j-1);
	}
	NR::nrerror("Too many steps in routine qromb");
	return 0.0;
}




NR_single_precision::DP NR_single_precision::qromo(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b,
	NR_single_precision::DP choose(NR_single_precision::DP (*)(const NR_single_precision::DP), const NR_single_precision::DP, const NR_single_precision::DP, const int))
{
	const int JMAX=14, JMAXP=JMAX+1, K=5;
	const NR_single_precision::DP EPS=3.0e-9;
	int i,j;
	NR_single_precision::DP ss,dss;
	NR_single_precision::Vec_DP h(JMAXP),s(JMAX),h_t(K),s_t(K);

	h(0)=1.0;
	for (j=1;j<=JMAX;j++) {
		s(j-1)=choose(func,a,b,j);
		if (j >= K) {
			for (i=0;i<K;i++) {
				h_t(i)=h(j-K+i);
				s_t(i)=s(j-K+i);
			}
			polint(h_t,s_t,0.0,ss,dss);
			if (fabs(dss) <= EPS*fabs(ss)) return ss;
		}
		h(j)=h(j-1)/9.0;
	}
	NR::nrerror("Too many steps in routine qromo");
	return 0.0;
}




void NR_single_precision::qroot(NR_single_precision::Vec_I_DP &p, NR_single_precision::DP &b, NR_single_precision::DP &c, const NR_single_precision::DP eps)
{
	const int ITMAX=20;
	const NR_single_precision::DP TINY=1.0e-14;
	int iter;
	NR_single_precision::DP sc,sb,s,rc,rb,r,dv,delc,delb;
	NR_single_precision::Vec_DP d(3);

	int n=p.size()-1;
	NR_single_precision::Vec_DP q(n+1),qq(n+1),rem(n+1);
	d(2)=1.0;
	for (iter=0;iter<ITMAX;iter++) {
		d(1)=b;
		d(0)=c;
		poldiv(p,d,q,rem);
		s=rem(0);
		r=rem(1);
		poldiv(q,d,qq,rem);
		sb = -c*(rc = -rem(1));
		rb = -b*rc+(sc = -rem(0));
		dv=1.0/(sb*rc-sc*rb);
		delb=(r*sc-s*rc)*dv;
		delc=(-r*sb+s*rb)*dv;
		b += (delb=(r*sc-s*rc)*dv);
		c += (delc=(-r*sb+s*rb)*dv);
		if ((fabs(delb) <= eps*fabs(b) || fabs(b) < TINY)
			&& (fabs(delc) <= eps*fabs(c) || fabs(c) < TINY)) {
			return;
		}
	}
	NR::nrerror("Too many iterations in routine qroot");
}


void NR_single_precision::qrsolv(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_IO_DP &b)
{
	int i,j;
	NR_single_precision::DP sum,tau;

	int n=a.rows();
	for (j=0;j<n-1;j++) {
		for (sum=0.0,i=j;i<n;i++) sum += a(i,j)*b(i);
		tau=sum/c(j);
		for (i=j;i<n;i++) b(i) -= tau*a(i,j);
	}
	rsolv(a,d,b);
}




void NR_single_precision::qrupdt(NR_single_precision::Mat_IO_DP &r, NR_single_precision::Mat_IO_DP &qt, NR_single_precision::Vec_IO_DP &u, NR_single_precision::Vec_I_DP &v)
{
	int i,k;

	int n=u.size();
	for (k=n-1;k>=0;k--)
		if (u(k) != 0.0) break;
	if (k < 0) k=0;
	for (i=k-1;i>=0;i--) {
		rotate(r,qt,i,u(i),-u(i+1));
		if (u(i) == 0.0)
			u(i)=fabs(u(i+1));
		else if (fabs(u(i)) > fabs(u(i+1)))
			u(i)=fabs(u(i))*sqrt(1.0+SQR(u(i+1)/u(i)));
		else u(i)=fabs(u(i+1))*sqrt(1.0+SQR(u(i)/u(i+1)));
	}
	for (i=0;i<n;i++) r(0,i) += u(0)*v(i);
	for (i=0;i<k;i++)
		rotate(r,qt,i,r(i,i),-r(i+1,i));
}




NR_single_precision::DP NR_single_precision::qsimp(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b)
{
	const int JMAX=20;
	const NR_single_precision::DP EPS=1.0e-10;
	int j;
	NR_single_precision::DP s,st,ost=0.0,os=0.0;

	for (j=0;j<JMAX;j++) {
		st=trapzd(func,a,b,j+1);
		s=(4.0*st-ost)/3.0;
		if (j > 5)
			if (fabs(s-os) < EPS*fabs(os) ||
				(s == 0.0 && os == 0.0)) return s;
		os=s;
		ost=st;
	}
	NR::nrerror("Too many steps in routine qsimp");
	return 0.0;
}




NR_single_precision::DP NR_single_precision::qtrap(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b)
{
	const int JMAX=20;
	const NR_single_precision::DP EPS=1.0e-10;
	int j;
	NR_single_precision::DP s,olds=0.0;

	for (j=0;j<JMAX;j++) {
		s=trapzd(func,a,b,j+1);
		if (j > 5)
			if (fabs(s-olds) < EPS*fabs(olds) ||
				(s == 0.0 && olds == 0.0)) return s;
		olds=s;
	}
	NR::nrerror("Too many steps in routine qtrap");
	return 0.0;
}


extern NR_single_precision::DP yy1(const NR_single_precision::DP),yy2(const NR_single_precision::DP);
extern NR_single_precision::DP z1(const NR_single_precision::DP, const NR_single_precision::DP);
extern NR_single_precision::DP z2(const NR_single_precision::DP, const NR_single_precision::DP);

namespace NRquad3d {
	NR_single_precision::DP xsav,ysav;
	NR_single_precision::DP (*nrfunc)(const NR_single_precision::DP, const NR_single_precision::DP, const NR_single_precision::DP);

	NR_single_precision::DP f3(const NR_single_precision::DP z)
	{
		return nrfunc(xsav,ysav,z);
	}

	NR_single_precision::DP f2(const NR_single_precision::DP y)
	{
		ysav=y;
		return NR_single_precision::qgaus(f3,z1(xsav,y),z2(xsav,y));
	}

	NR_single_precision::DP f1(const NR_single_precision::DP x)
	{
		xsav=x;
		return NR_single_precision::qgaus(f2,yy1(x),yy2(x));
	}
}

NR_single_precision::DP NR_single_precision::quad3d(NR_single_precision::DP func(const NR_single_precision::DP, const NR_single_precision::DP, const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2)
{
	NRquad3d::nrfunc=func;
	return qgaus(NRquad3d::f1,x1,x2);
}


void NR_single_precision::quadct(const NR_single_precision::DP x, const NR_single_precision::DP y, NR_single_precision::Vec_I_DP &xx, NR_single_precision::Vec_I_DP &yy, NR_single_precision::DP &fa,
	NR_single_precision::DP &fb, NR_single_precision::DP &fc, NR_single_precision::DP &fd)
{
	int k,na,nb,nc,nd;
	NR_single_precision::DP ff;

	int nn=xx.size();
	na=nb=nc=nd=0;
	for (k=0;k<nn;k++) {
		if (yy(k) > y)
			xx(k) > x ? ++na : ++nb;
		else
			xx(k) > x ? ++nd : ++nc;
	}
	ff=1.0/nn;
	fa=ff*na;
	fb=ff*nb;
	fc=ff*nc;
	fd=ff*nd;
}




NR_single_precision::DP x;

void NR_single_precision::quadmx(NR_single_precision::Mat_O_DP &a)
{
	const NR_single_precision::DP PI=3.14159263589793238;
	int j,k;
	NR_single_precision::DP h,xx,cx;

	int n=a.rows();
	NR_single_precision::Vec_DP wt(n);
	h=PI/(n-1);
	for (j=0;j<n;j++) {
		x=xx=j*h;
		wwghts(wt,h,kermom);
		cx=cos(xx);
		for (k=0;k<n;k++)
			a(j,k)=wt(k)*cx*cos(k*h);
		++a(j,j);
	}
}


void NR_single_precision::quadvl(const NR_single_precision::DP x, const NR_single_precision::DP y, NR_single_precision::DP &fa, NR_single_precision::DP &fb, NR_single_precision::DP &fc, NR_single_precision::DP &fd)
{
	NR_single_precision::DP qa,qb,qc,qd;

	qa=MIN(2.0,MAX(0.0,1.0-x));
	qb=MIN(2.0,MAX(0.0,1.0-y));
	qc=MIN(2.0,MAX(0.0,x+1.0));
	qd=MIN(2.0,MAX(0.0,y+1.0));
	fa=0.25*qa*qb;
	fb=0.25*qb*qc;
	fc=0.25*qc*qd;
	fd=0.25*qd*qa;
}


NR_single_precision::DP NR_single_precision::ran0(int &idum)
{
	const int IA=16807,IM=2147483647,IQ=127773;
	const int IR=2836,MASK=123459876;
	const NR_single_precision::DP AM=1.0/NR_single_precision::DP(IM);
	int k;
	NR_single_precision::DP ans;

	idum ^= MASK;
	k=idum/IQ;
	idum=IA*(idum-k*IQ)-IR*k;
	if (idum < 0) idum += IM;
	ans=AM*idum;
	idum ^= MASK;
	return ans;
}


NR_single_precision::DP NR_single_precision::ran1(int &idum)
{
	const int IA=16807,IM=2147483647,IQ=127773,IR=2836,NTAB=32;
	const int NDIV=(1+(IM-1)/NTAB);
	const NR_single_precision::DP EPS=3.0e-16,AM=1.0/IM,RNMX=(1.0-EPS);
	static int iy=0;
	static NR_single_precision::Vec_INT iv(NTAB);
	int j,k;
	NR_single_precision::DP temp;

	if (idum <= 0 || !iy) {
		if (-idum < 1) idum=1;
		else idum = -idum;
		for (j=NTAB+7;j>=0;j--) {
			k=idum/IQ;
			idum=IA*(idum-k*IQ)-IR*k;
			if (idum < 0) idum += IM;
			if (j < NTAB) iv(j) = idum;
		}
		iy=iv(0);
	}
	k=idum/IQ;
	idum=IA*(idum-k*IQ)-IR*k;
	if (idum < 0) idum += IM;
	j=iy/NDIV;
	iy=iv(j);
	iv(j) = idum;
	if ((temp=AM*iy) > RNMX) return RNMX;
	else return temp;
}


NR_single_precision::DP NR_single_precision::ran2(int &idum)
{
	const int IM1=2147483563,IM2=2147483399;
	const int IA1=40014,IA2=40692,IQ1=53668,IQ2=52774;
	const int IR1=12211,IR2=3791,NTAB=32,IMM1=IM1-1;
	const int NDIV=1+IMM1/NTAB;
	const NR_single_precision::DP EPS=3.0e-16,RNMX=1.0-EPS,AM=1.0/NR_single_precision::DP(IM1);
	static int idum2=123456789,iy=0;
	static NR_single_precision::Vec_INT iv(NTAB);
	int j,k;
	NR_single_precision::DP temp;

	if (idum <= 0) {
		idum=(idum==0 ? 1 : -idum);
		idum2=idum;
		for (j=NTAB+7;j>=0;j--) {
			k=idum/IQ1;
			idum=IA1*(idum-k*IQ1)-k*IR1;
			if (idum < 0) idum += IM1;
			if (j < NTAB) iv(j) = idum;
		}
		iy=iv(0);
	}
	k=idum/IQ1;
	idum=IA1*(idum-k*IQ1)-k*IR1;
	if (idum < 0) idum += IM1;
	k=idum2/IQ2;
	idum2=IA2*(idum2-k*IQ2)-k*IR2;
	if (idum2 < 0) idum2 += IM2;
	j=iy/NDIV;
	iy=iv(j)-idum2;
	iv(j) = idum;
	if (iy < 1) iy += IMM1;
	if ((temp=AM*iy) > RNMX) return RNMX;
	else return temp;
}
#include <cstdlib>



NR_single_precision::DP NR_single_precision::ran3(int &idum)
{
	static int inext,inextp;
	static int iff=0;
	const int MBIG=1000000000,MSEED=161803398,MZ=0;
	const NR_single_precision::DP FAC=(1.0/MBIG);
	static NR_single_precision::Vec_INT ma(56);
	int i,ii,k,mj,mk;

	if (idum < 0 || iff == 0) {
		iff=1;
		mj=labs(MSEED-labs(idum));
		mj %= MBIG;
		ma(55)=mj;
		mk=1;
		for (i=1;i<=54;i++) {
			ii=(21*i) % 55;
			ma(ii)=mk;
			mk=mj-mk;
			if (mk < int(MZ)) mk += MBIG;
			mj=ma(ii);
		}
		for (k=0;k<4;k++)
			for (i=1;i<=55;i++) {
				ma(i) -= ma(1+(i+30) % 55);
				if (ma(i) < int(MZ)) ma(i) += MBIG;
			}
		inext=0;
		inextp=31;
		idum=1;
	}
	if (++inext == 56) inext=1;
	if (++inextp == 56) inextp=1;
	mj=ma(inext)-ma(inextp);
	if (mj < int(MZ)) mj += MBIG;
	ma(inext)=mj;
	return mj*FAC;
}


NR_single_precision::DP NR_single_precision::ran4(int &idum)
{
#if defined(vax) || defined(_vax_) || defined(__vax__) || defined(VAX)
	static const unsigned long jflone = 0x00004080;
	static const unsigned long jflmsk = 0xffff007f;
#else
	static const unsigned long jflone = 0x3f800000;
	static const unsigned long jflmsk = 0x007fffff;
#endif
	unsigned long irword,itemp,lword;
	static int idums = 0;

	if (idum < 0) {
		idums = -idum;
		idum=1;
	}
	irword=idum;
	lword=idums;
	psdes(lword,irword);
	itemp=jflone | (jflmsk & irword);
	++idum;
	return (*(float *)&itemp)-1.0;
}


void NR_single_precision::rank(NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_O_INT &irank)
{
	int j;

	int n=indx.size();
	for (j=0;j<n;j++) irank(indx(j))=j;
}


extern int idum;

void NR_single_precision::ranpt(NR_single_precision::Vec_O_DP &pt, NR_single_precision::Vec_I_DP &regn)
{
	int j;

	int n=pt.size();
	for (j=0;j<n;j++)
		pt(j)=regn(j)+(regn(n+j)-regn(j))*ran1(idum);
}




void NR_single_precision::ratint(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, const NR_single_precision::DP x, NR_single_precision::DP &y, NR_single_precision::DP &dy)
{
	const NR_single_precision::DP TINY=1.0e-25;
	int m,i,ns=0;
	NR_single_precision::DP w,t,hh,h,dd;

	int n=xa.size();
	NR_single_precision::Vec_DP c(n),d(n);
	hh=fabs(x-xa(0));
	for (i=0;i<n;i++) {
		h=fabs(x-xa(i));
		if (h == 0.0) {
			y=ya(i);
			dy=0.0;
			return;
		} else if (h < hh) {
			ns=i;
			hh=h;
		}
		c(i)=ya(i);
		d(i)=ya(i)+TINY;
	}
	y=ya(ns--);
	for (m=1;m<n;m++) {
		for (i=0;i<n-m;i++) {
			w=c(i+1)-d(i);
			h=xa(i+m)-x;
			t=(xa(i)-x)*d(i)/h;
			dd=t-c(i+1);
			if (dd == 0.0) NR::nrerror("Error in routine ratint");
			dd=w/dd;
			d(i)=c(i+1)*dd;
			c(i)=t*dd;
		}
		y += (dy=(2*(ns+1) < (n-m) ? c(ns+1) : d(ns--)));
	}
}
#include <iostream>
#include <iomanip>




void NR_single_precision::ratlsq(NR_single_precision::DP fn(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const int mm,
	const int kk, NR_single_precision::Vec_O_DP &cof, NR_single_precision::DP &dev)
{
	const int NPFAC=8,MAXIT=5;
	const NR_single_precision::DP BIG=1.0e30,PIO2=1.570796326794896619;
	int i,it,j,ncof,npt;
	NR_single_precision::DP devmax,e,hth,power,sum;

	ncof=mm+kk+1;
	npt=NPFAC*ncof;
	NR_single_precision::Vec_DP bb(npt),coff(ncof),ee(npt),fs(npt),w(ncof),wt(npt),xs(npt);
	NR_single_precision::Mat_DP u(npt,ncof),v(ncof,ncof);
	dev=BIG;
	for (i=0;i<npt;i++) {
		if (i < (npt/2)-1) {
			hth=PIO2*i/(npt-1.0);
			xs(i)=a+(b-a)*SQR(sin(hth));
		} else {
			hth=PIO2*(npt-i)/(npt-1.0);
			xs(i)=b-(b-a)*SQR(sin(hth));
		}
		fs(i)=fn(xs(i));
		wt(i)=1.0;
		ee(i)=1.0;
	}
	e=0.0;
	for (it=0;it<MAXIT;it++) {
		for (i=0;i<npt;i++) {
			power=wt(i);
			bb(i)=power*(fs(i)+SIGN(e,ee(i)));
			for (j=0;j<mm+1;j++) {
				u(i,j)=power;
				power *= xs(i);
			}
			power = -bb(i);
			for (j=mm+1;j<ncof;j++) {
				power *= xs(i);
				u(i,j)=power;
			}
		}
		svdcmp(u,w,v);
		svbksb(u,w,v,bb,coff);
		devmax=sum=0.0;
		for (j=0;j<npt;j++) {
			ee(j)=ratval(xs(j),coff,mm,kk)-fs(j);
			wt(j)=fabs(ee(j));
			sum += wt(j);
			if (wt(j) > devmax) devmax=wt(j);
		}
		e=sum/npt;
		if (devmax <= dev) {
			for (j=0;j<ncof;j++) cof(j)=coff(j);
			dev=devmax;
		}
		cout << " ratlsq iteration= " << it;
		cout << "  max error= " << devmax << endl;
	}
}


NR_single_precision::DP NR_single_precision::ratval(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &cof, const int mm, const int kk)
{
	int j;
	NR_single_precision::DP sumd,sumn;

	for (sumn=cof(mm),j=mm-1;j>=0;j--) sumn=sumn*x+cof(j);
	for (sumd=0.0,j=mm+kk;j>mm;j--) sumd=(sumd+cof(j))*x;
	return sumn/(1.0+sumd);
}






NR_single_precision::DP NR_single_precision::rc(const NR_single_precision::DP x, const NR_single_precision::DP y)
{
	const NR_single_precision::DP ERRTOL=0.0012, TINY=1.69e-38, SQRTNY=1.3e-19, BIG=3.0e37;
	const NR_single_precision::DP TNBG=TINY*BIG, COMP1=2.236/SQRTNY, COMP2=TNBG*TNBG/25.0;
	const NR_single_precision::DP THIRD=1.0/3.0, C1=0.3, C2=1.0/7.0, C3=0.375, C4=9.0/22.0;
	NR_single_precision::DP alamb,ave,s,w,xt,yt;

	if (x < 0.0 || y == 0.0 || (x+fabs(y)) < TINY || (x+fabs(y)) > BIG ||
		(y<-COMP1 && x > 0.0 && x < COMP2))
			NR::nrerror("invalid arguments in rc");
	if (y > 0.0) {
		xt=x;
		yt=y;
		w=1.0;
	} else {
		xt=x-y;
		yt= -y;
		w=sqrt(x)/sqrt(xt);
	}
	do {
		alamb=2.0*sqrt(xt)*sqrt(yt)+yt;
		xt=0.25*(xt+alamb);
		yt=0.25*(yt+alamb);
		ave=THIRD*(xt+yt+yt);
		s=(yt-ave)/ave;
	} while (fabs(s) > ERRTOL);
	return w*(1.0+s*s*(C1+s*(C2+s*(C3+s*C4))))/sqrt(ave);
}






NR_single_precision::DP NR_single_precision::rd(const NR_single_precision::DP x, const NR_single_precision::DP y, const NR_single_precision::DP z)
{
	const NR_single_precision::DP ERRTOL=0.0015, TINY=1.0e-25, BIG=4.5e21;
	const NR_single_precision::DP C1=3.0/14.0, C2=1.0/6.0, C3=9.0/22.0;
	const NR_single_precision::DP C4=3.0/26.0, C5=0.25*C3, C6=1.5*C4;
	NR_single_precision::DP alamb,ave,delx,dely,delz,ea,eb,ec,ed,ee,fac,sqrtx,sqrty,
		sqrtz,sum,xt,yt,zt;

	if (MIN(x,y) < 0.0 || MIN(x+y,z) < TINY || MAX(MAX(x,y),z) > BIG)
		NR::nrerror("invalid arguments in rd");
	xt=x;
	yt=y;
	zt=z;
	sum=0.0;
	fac=1.0;
	do {
		sqrtx=sqrt(xt);
		sqrty=sqrt(yt);
		sqrtz=sqrt(zt);
		alamb=sqrtx*(sqrty+sqrtz)+sqrty*sqrtz;
		sum += fac/(sqrtz*(zt+alamb));
		fac=0.25*fac;
		xt=0.25*(xt+alamb);
		yt=0.25*(yt+alamb);
		zt=0.25*(zt+alamb);
		ave=0.2*(xt+yt+3.0*zt);
		delx=(ave-xt)/ave;
		dely=(ave-yt)/ave;
		delz=(ave-zt)/ave;
	} while (MAX(MAX(fabs(delx),fabs(dely)),fabs(delz)) > ERRTOL);
	ea=delx*dely;
	eb=delz*delz;
	ec=ea-eb;
	ed=ea-6.0*eb;
	ee=ed+ec+ec;
	return 3.0*sum+fac*(1.0+ed*(-C1+C5*ed-C6*delz*ee)
		+delz*(C2*ee+delz*(-C3*ec+delz*C4*ea)))/(ave*sqrt(ave));
}






void NR_single_precision::realft(NR_single_precision::Vec_IO_DP &data, const int isign)
{
	int i,i1,i2,i3,i4;
	NR_single_precision::DP c1=0.5,c2,h1r,h1i,h2r,h2i,wr,wi,wpr,wpi,wtemp,theta;

	int n=data.size();
	theta=3.141592653589793238/NR_single_precision::DP(n>>1);
	if (isign == 1) {
		c2 = -0.5;
		four1(data,1);
	} else {
		c2=0.5;
		theta = -theta;
	}
	wtemp=sin(0.5*theta);
	wpr = -2.0*wtemp*wtemp;
	wpi=sin(theta);
	wr=1.0+wpr;
	wi=wpi;
	for (i=1;i<(n>>2);i++) {
		i2=1+(i1=i+i);
		i4=1+(i3=n-i1);
		h1r=c1*(data(i1)+data(i3));
		h1i=c1*(data(i2)-data(i4));
		h2r= -c2*(data(i2)+data(i4));
		h2i=c2*(data(i1)-data(i3));
		data(i1)=h1r+wr*h2r-wi*h2i;
		data(i2)=h1i+wr*h2i+wi*h2r;
		data(i3)=h1r-wr*h2r+wi*h2i;
		data(i4)= -h1i+wr*h2i+wi*h2r;
		wr=(wtemp=wr)*wpr-wi*wpi+wr;
		wi=wi*wpr+wtemp*wpi+wi;
	}
	if (isign == 1) {
		data(0) = (h1r=data(0))+data(1);
		data(1) = h1r-data(1);
	} else {
		data(0)=c1*((h1r=data(0))+data(1));
		data(1)=c1*(h1r-data(1));
		four1(data,-1);
	}
}




void NR_single_precision::rebin(const NR_single_precision::DP rc, const int nd, NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &xin,
	NR_single_precision::Mat_IO_DP &xi, const int j)
{
	int i,k=0;
	NR_single_precision::DP dr=0.0,xn=0.0,xo=0.0;

	for (i=0;i<nd-1;i++) {
		while (rc > dr)
			dr += r((++k)-1);
		if (k > 1) xo=xi(j,k-2);
		xn=xi(j,k-1);
		dr -= rc;
		xin(i)=xn-(xn-xo)*dr/r(k-1);
	}
	for (i=0;i<nd-1;i++) xi(j,i)=xin(i);
	xi(j,nd-1)=1.0;
}




void NR_single_precision::red(const int iz1, const int iz2, const int jz1, const int jz2,
	const int jm1, const int jm2, const int jmf, const int ic1,
	const int jc1, const int jcf, const int kc, Mat3D_I_DP &c,
	NR_single_precision::Mat_IO_DP &s)
{
	int loff,l,j,ic,i;
	NR_single_precision::DP vx;

	loff=jc1-jm1;
	ic=ic1;
	for (j=jz1;j<jz2;j++) {
		for (l=jm1;l<jm2;l++) {
			vx=c(ic,l+loff,kc-1);
			for (i=iz1;i<iz2;i++) s(i,l) -= s(i,j)*vx;
		}
		vx=c(ic,jcf,kc-1);
		for (i=iz1;i<iz2;i++) s(i,jmf) -= s(i,j)*vx;
		ic += 1;
	}
}




void NR_single_precision::relax(NR_single_precision::Mat_IO_DP &u, NR_single_precision::Mat_I_DP &rhs)
{
	int i,ipass,isw,j,jsw=1;
	NR_single_precision::DP h,h2;

	int n=u.rows();
	h=1.0/(n-1);
	h2=h*h;
	for (ipass=0;ipass<2;ipass++,jsw=3-jsw) {
		isw=jsw;
		for (j=1;j<n-1;j++,isw=3-isw)
			for (i=isw;i<n-1;i+=2)
				u(i,j)=0.25*(u(i+1,j)+u(i-1,j)+u(i,j+1)
					+u(i,j-1)-h2*rhs(i,j));
	}
}




void NR_single_precision::relax2(NR_single_precision::Mat_IO_DP &u, NR_single_precision::Mat_I_DP &rhs)
{
	int i,ipass,isw,j,jsw=1;
	NR_single_precision::DP foh2,h,h2i,res;

	int n=u.rows();
	h=1.0/(n-1);
	h2i=1.0/(h*h);
	foh2 = -4.0*h2i;
	for (ipass=0;ipass<2;ipass++,jsw=3-jsw) {
		isw=jsw;
		for (j=1;j<n-1;j++,isw=3-isw) {
			for (i=isw;i<n-1;i+=2) {
				res=h2i*(u(i+1,j)+u(i-1,j)+u(i,j+1)+u(i,j-1)-
					4.0*u(i,j))+u(i,j)*u(i,j)-rhs(i,j);
				u(i,j) -= res/(foh2+2.0*u(i,j));
			}
		}
	}
}




void NR_single_precision::resid(NR_single_precision::Mat_O_DP &res, NR_single_precision::Mat_I_DP &u, NR_single_precision::Mat_I_DP &rhs)
{
	int i,j;
	NR_single_precision::DP h,h2i;

	int n=u.rows();
	h=1.0/(n-1);
	h2i=1.0/(h*h);
	for (j=1;j<n-1;j++)
		for (i=1;i<n-1;i++)
			res(i,j) = -h2i*(u(i+1,j)+u(i-1,j)+u(i,j+1)
				+u(i,j-1)-4.0*u(i,j))+rhs(i,j);
	for (i=0;i<n;i++)
		res(i,0)=res(i,n-1)=res(0,i)=res(n-1,i)=0.0;
}






NR_single_precision::DP NR_single_precision::revcst(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_INT &iorder, NR_single_precision::Vec_IO_INT &n)
{
	int j,ii;
	NR_single_precision::DP de;
	NR_single_precision::Vec_DP xx(4),yy(4);

	int ncity=x.size();
	n(2)=(n(0)+ncity-1) % ncity;
	n(3)=(n(1)+1) % ncity;
	for (j=0;j<4;j++) {
		ii=iorder(n(j));
		xx(j)=x(ii);
		yy(j)=y(ii);
	}
	de = -alen(xx(0),xx(2),yy(0),yy(2));
	de -= alen(xx(1),xx(3),yy(1),yy(3));
	de += alen(xx(0),xx(3),yy(0),yy(3));
	de += alen(xx(1),xx(2),yy(1),yy(2));
	return de;
}



void NR_single_precision::reverse(NR_single_precision::Vec_IO_INT &iorder, NR_single_precision::Vec_I_INT &n)
{
	int nn,j,k,l,itmp;

	int ncity=iorder.size();
	nn=(1+((n(1)-n(0)+ncity) % ncity))/2;
	for (j=0;j<nn;j++) {
		k=(n(0)+j) % ncity;
		l=(n(1)-j+ncity) % ncity;
		itmp=iorder(k);
		iorder(k)=iorder(l);
		iorder(l)=itmp;
	}
}





NR_single_precision::DP NR_single_precision::rf(const NR_single_precision::DP x, const NR_single_precision::DP y, const NR_single_precision::DP z)
{
	const NR_single_precision::DP ERRTOL=0.0025, TINY=1.5e-38, BIG=3.0e37, THIRD=1.0/3.0;
	const NR_single_precision::DP C1=1.0/24.0, C2=0.1, C3=3.0/44.0, C4=1.0/14.0;
	NR_single_precision::DP alamb,ave,delx,dely,delz,e2,e3,sqrtx,sqrty,sqrtz,xt,yt,zt;

	if (MIN(MIN(x,y),z) < 0.0 || MIN(MIN(x+y,x+z),y+z) < TINY ||
		MAX(MAX(x,y),z) > BIG)
			NR::nrerror("invalid arguments in rf");
	xt=x;
	yt=y;
	zt=z;
	do {
		sqrtx=sqrt(xt);
		sqrty=sqrt(yt);
		sqrtz=sqrt(zt);
		alamb=sqrtx*(sqrty+sqrtz)+sqrty*sqrtz;
		xt=0.25*(xt+alamb);
		yt=0.25*(yt+alamb);
		zt=0.25*(zt+alamb);
		ave=THIRD*(xt+yt+zt);
		delx=(ave-xt)/ave;
		dely=(ave-yt)/ave;
		delz=(ave-zt)/ave;
	} while (MAX(MAX(fabs(delx),fabs(dely)),fabs(delz)) > ERRTOL);
	e2=delx*dely-delz*delz;
	e3=delx*dely*delz;
	return (1.0+(C1*e2-C2-C3*e3)*e2+C4*e3)/sqrt(ave);
}





NR_single_precision::DP NR_single_precision::rj(const NR_single_precision::DP x, const NR_single_precision::DP y, const NR_single_precision::DP z, const NR_single_precision::DP p)
{
	const NR_single_precision::DP ERRTOL=0.0015, TINY=2.5e-13, BIG=9.0e11;
	const NR_single_precision::DP C1=3.0/14.0, C2=1.0/3.0, C3=3.0/22.0, C4=3.0/26.0;
	const NR_single_precision::DP C5=0.75*C3, C6=1.5*C4, C7=0.5*C2, C8=C3+C3;
	NR_single_precision::DP a,alamb,alpha,ans,ave,b,beta,delp,delx,dely,delz,ea,eb,ec,ed,ee,
		fac,pt,rcx,rho,sqrtx,sqrty,sqrtz,sum,tau,xt,yt,zt;

	if (MIN(MIN(x,y),z) < 0.0 || MIN(MIN(x+y,x+z),MIN(y+z,fabs(p))) < TINY
		|| MAX(MAX(x,y),MAX(z,fabs(p))) > BIG)
			NR::nrerror("invalid arguments in rj");
	sum=0.0;
	fac=1.0;
	if (p > 0.0) {
		xt=x;
		yt=y;
		zt=z;
		pt=p;
	} else {
		xt=MIN(MIN(x,y),z);
		zt=MAX(MAX(x,y),z);
		yt=x+y+z-xt-zt;
		a=1.0/(yt-p);
		b=a*(zt-yt)*(yt-xt);
		pt=yt+b;
		rho=xt*zt/yt;
		tau=p*pt/yt;
		rcx=rc(rho,tau);
	}
	do {
		sqrtx=sqrt(xt);
		sqrty=sqrt(yt);
		sqrtz=sqrt(zt);
		alamb=sqrtx*(sqrty+sqrtz)+sqrty*sqrtz;
		alpha=SQR(pt*(sqrtx+sqrty+sqrtz)+sqrtx*sqrty*sqrtz);
		beta=pt*SQR(pt+alamb);
		sum += fac*rc(alpha,beta);
		fac=0.25*fac;
		xt=0.25*(xt+alamb);
		yt=0.25*(yt+alamb);
		zt=0.25*(zt+alamb);
		pt=0.25*(pt+alamb);
		ave=0.2*(xt+yt+zt+pt+pt);
		delx=(ave-xt)/ave;
		dely=(ave-yt)/ave;
		delz=(ave-zt)/ave;
		delp=(ave-pt)/ave;
	} while (MAX(MAX(fabs(delx),fabs(dely)),
		MAX(fabs(delz),fabs(delp))) > ERRTOL);
	ea=delx*(dely+delz)+dely*delz;
	eb=delx*dely*delz;
	ec=delp*delp;
	ed=ea-3.0*ec;
	ee=eb+2.0*delp*(ea-ec);
	ans=3.0*sum+fac*(1.0+ed*(-C1+C5*ed-C6*ee)+eb*(C7+delp*(-C8+delp*C4))
		+delp*ea*(C2-delp*C3)-C2*delp*ec)/(ave*sqrt(ave));
	if (p <= 0.0) ans=a*(b*ans+3.0*(rcx-rf(xt,yt,zt)));
	return ans;
}



void NR_single_precision::rk4(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, const NR_single_precision::DP x, const NR_single_precision::DP h,
	NR_single_precision::Vec_O_DP &yout, void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	int i;
	NR_single_precision::DP xh,hh,h6;

	int n=y.size();
	NR_single_precision::Vec_DP dym(n),dyt(n),yt(n);
	hh=h*0.5;
	h6=h/6.0;
	xh=x+hh;
	for (i=0;i<n;i++) yt(i)=y(i)+hh*dydx(i);
	derivs(xh,yt,dyt);
	for (i=0;i<n;i++) yt(i)=y(i)+hh*dyt(i);
	derivs(xh,yt,dym);
	for (i=0;i<n;i++) {
		yt(i)=y(i)+h*dym(i);
		dym(i) += dyt(i);
	}
	derivs(x+h,yt,dyt);
	for (i=0;i<n;i++)
		yout(i)=y(i)+h6*(dydx(i)+dyt(i)+2.0*dym(i));
}



void NR_single_precision::rkck(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, const NR_single_precision::DP x,
	const NR_single_precision::DP h, NR_single_precision::Vec_O_DP &yout, NR_single_precision::Vec_O_DP &yerr,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	static const NR_single_precision::DP a2=0.2, a3=0.3, a4=0.6, a5=1.0, a6=0.875,
		b21=0.2, b31=3.0/40.0, b32=9.0/40.0, b41=0.3, b42 = -0.9,
		b43=1.2, b51 = -11.0/54.0, b52=2.5, b53 = -70.0/27.0,
		b54=35.0/27.0, b61=1631.0/55296.0, b62=175.0/512.0,
		b63=575.0/13824.0, b64=44275.0/110592.0, b65=253.0/4096.0,
		c1=37.0/378.0, c3=250.0/621.0, c4=125.0/594.0, c6=512.0/1771.0,
		dc1=c1-2825.0/27648.0, dc3=c3-18575.0/48384.0,
		dc4=c4-13525.0/55296.0, dc5 = -277.00/14336.0, dc6=c6-0.25;
	int i;

	int n=y.size();
	NR_single_precision::Vec_DP ak2(n),ak3(n),ak4(n),ak5(n),ak6(n),ytemp(n);
	for (i=0;i<n;i++)
		ytemp(i)=y(i)+b21*h*dydx(i);
	derivs(x+a2*h,ytemp,ak2);
	for (i=0;i<n;i++)
		ytemp(i)=y(i)+h*(b31*dydx(i)+b32*ak2(i));
	derivs(x+a3*h,ytemp,ak3);
	for (i=0;i<n;i++)
		ytemp(i)=y(i)+h*(b41*dydx(i)+b42*ak2(i)+b43*ak3(i));
	derivs(x+a4*h,ytemp,ak4);
	for (i=0;i<n;i++)
		ytemp(i)=y(i)+h*(b51*dydx(i)+b52*ak2(i)+b53*ak3(i)+b54*ak4(i));
	derivs(x+a5*h,ytemp,ak5);
	for (i=0;i<n;i++)
		ytemp(i)=y(i)+h*(b61*dydx(i)+b62*ak2(i)+b63*ak3(i)+b64*ak4(i)+b65*ak5(i));
	derivs(x+a6*h,ytemp,ak6);
	for (i=0;i<n;i++)
		yout(i)=y(i)+h*(c1*dydx(i)+c3*ak3(i)+c4*ak4(i)+c6*ak6(i));
	for (i=0;i<n;i++)
		yerr(i)=h*(dc1*dydx(i)+dc3*ak3(i)+dc4*ak4(i)+dc5*ak5(i)+dc6*ak6(i));
}



extern NR_single_precision::Vec_DP *xx_p;
extern NR_single_precision::Mat_DP *y_p;

void NR_single_precision::rkdumb(NR_single_precision::Vec_I_DP &vstart, const NR_single_precision::DP x1, const NR_single_precision::DP x2,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	int i,k;
	NR_single_precision::DP x,h;

	NR_single_precision::Vec_DP &xx=*xx_p;
	NR_single_precision::Mat_DP &y=*y_p;
	int nvar=y.rows();
	int nstep=y.cols()-1;
	NR_single_precision::Vec_DP v(nvar),vout(nvar),dv(nvar);
	for (i=0;i<nvar;i++) {
		v(i)=vstart(i);
		y(i,0)=v(i);
	}
	xx(0)=x1;
	x=x1;
	h=(x2-x1)/nstep;
	for (k=0;k<nstep;k++) {
		derivs(x,v,dv);
		rk4(v,dv,x,h,vout,derivs);
		if (x+h == x)
			NR::nrerror("Step size too small in routine rkdumb");
		x += h;
		xx(k+1)=x;
		for (i=0;i<nvar;i++) {
			v(i)=vout(i);
			y(i,k+1)=v(i);
		}
	}
}





void NR_single_precision::rkqs(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &x, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const NR_single_precision::DP SAFETY=0.9, PGROW=-0.2, PSHRNK=-0.25, ERRCON=1.89e-4;
	int i;
	NR_single_precision::DP errmax,h,htemp,xnew;

	int n=y.size();
	h=htry;
	NR_single_precision::Vec_DP yerr(n),ytemp(n);
	for (;;) {
		rkck(y,dydx,x,h,ytemp,yerr,derivs);
		errmax=0.0;
		for (i=0;i<n;i++) errmax=MAX(errmax,fabs(yerr(i)/yscal(i)));
		errmax /= eps;
		if (errmax <= 1.0) break;
		htemp=SAFETY*h*pow(errmax,PSHRNK);
		h=(h >= 0.0 ? MAX(htemp,0.1*h) : MIN(htemp,0.1*h));
		xnew=x+h;
		if (xnew == x) NR::nrerror("stepsize underflow in rkqs");
	}
	if (errmax > ERRCON) hnext=SAFETY*h*pow(errmax,PGROW);
	else hnext=5.0*h;
	x += (hdid=h);
	for (i=0;i<n;i++) y(i)=ytemp(i);
}





void NR_single_precision::rlft3(Mat3D_IO_DP &data, NR_single_precision::Mat_IO_DP &speq, const int isign)
{
	ASSERT(0);
	/*
	int i1,i2,i3,j1,j2,j3,ii3,k1,k2,k3,k4;
	NR_single_precision::DP theta,wi,wpi,wpr,wr,wtemp;
	NR_single_precision::DP c1,c2,h1r,h1i,h2r,h2i;
	NR_single_precision::Vec_INT nn(3);

	int nn1=data.dim1();
	int nn2=data.dim2();
	int nn3=data.dim3();
	c1=0.5;
	c2= -0.5*isign;
	theta=isign*(6.28318530717959/nn3);
	wtemp=sin(0.5*theta);
	wpr= -2.0*wtemp*wtemp;
	wpi=sin(theta);
	nn(0)=nn1;
	nn(1)=nn2;
	nn(2)=nn3 >> 1;
	NR_single_precision::Vec_DP data_v(&data(0,0,0),nn1*nn2*nn3);
	if (isign == 1) {
		fourn(data_v,nn,isign);
		k1=0;
		for (i1=0;i1<nn1;i1++)
			for (i2=0,j2=0;i2<nn2;i2++,k1+=nn3) {
				speq(i1,j2++)=data_v(k1);
				speq(i1,j2++)=data_v(k1+1);
			}
	}
	for (i1=0;i1<nn1;i1++) {
		j1=(i1 != 0 ? nn1-i1 : 0);
		wr=1.0;
		wi=0.0;
		for (ii3=0;ii3<=(nn3>>1);ii3+=2) {
			k1=i1*nn2*nn3;
			k3=j1*nn2*nn3;
			for (i2=0;i2<nn2;i2++,k1+=nn3) {
				if (ii3 == 0) {
					j2=(i2 != 0 ? ((nn2-i2)<<1) : 0);
					h1r=c1*(data_v(k1)+speq(j1,j2));
					h1i=c1*(data_v(k1+1)-speq(j1,j2+1));
					h2i=c2*(data_v(k1)-speq(j1,j2));
					h2r= -c2*(data_v(k1+1)+speq(j1,j2+1));
					data_v(k1)=h1r+h2r;
					data_v(k1+1)=h1i+h2i;
					speq(j1,j2)=h1r-h2r;
					speq(j1,j2+1)=h2i-h1i;
				} else {
					j2=(i2 != 0 ? nn2-i2 : 0);
					j3=nn3-ii3;
					k2=k1+ii3;
					k4=k3+j2*nn3+j3;
					h1r=c1*(data_v(k2)+data_v(k4));
					h1i=c1*(data_v(k2+1)-data_v(k4+1));
					h2i=c2*(data_v(k2)-data_v(k4));
					h2r= -c2*(data_v(k2+1)+data_v(k4+1));
					data_v(k2)=h1r+wr*h2r-wi*h2i;
					data_v(k2+1)=h1i+wr*h2i+wi*h2r;
					data_v(k4)=h1r-wr*h2r+wi*h2i;
					data_v(k4+1)= -h1i+wr*h2i+wi*h2r;
				}
			}
			wr=(wtemp=wr)*wpr-wi*wpi+wr;
			wi=wi*wpr+wtemp*wpi+wi;
		}
	}
	if (isign == -1) fourn(data_v,nn,isign);
	k1=0;
	for (i1=0;i1<nn1;i1++)
		for (i2=0;i2<nn2;i2++)
			for (i3=0;i3<nn3;i3++) data(i1,i2,i3)=data_v(k1++);*/
}


#include <limits>



extern NR_single_precision::DP aa,abdevt;
extern const NR_single_precision::Vec_I_DP *xt_p,*yt_p;

NR_single_precision::DP NR_single_precision::rofunc(const NR_single_precision::DP b)
{
	const NR_single_precision::DP EPS=DBL_EPSILON;
	int j;
	NR_single_precision::DP d,sum=0.0;

	const NR_single_precision::Vec_DP &xt=*xt_p,&yt=*yt_p;
	int ndatat=xt.size();
	NR_single_precision::Vec_DP arr(ndatat);
	for (j=0;j<ndatat;j++) arr(j)=yt(j)-b*xt(j);
	if (ndatat & 1 == 1) {
		aa=select((ndatat-1)>>1,arr);
	} else {
		j=ndatat >> 1;
		aa=0.5*(select(j-1,arr)+select(j,arr));
	}
	abdevt=0.0;
	for (j=0;j<ndatat;j++) {
		d=yt(j)-(b*xt(j)+aa);
		abdevt += fabs(d);
		if (yt(j) != 0.0) d /= fabs(yt(j));
		if (fabs(d) > EPS) sum += (d >= 0.0 ? xt(j) : -xt(j));
	}
	return sum;
}





void NR_single_precision::rotate(NR_single_precision::Mat_IO_DP &r, NR_single_precision::Mat_IO_DP &qt, const int i, const NR_single_precision::DP a,
	const NR_single_precision::DP b)
{
	int j;
	NR_single_precision::DP c,fact,s,w,y;

	int n=r.rows();
	if (a == 0.0) {
		c=0.0;
		s=(b >= 0.0 ? 1.0 : -1.0);
	} else if (fabs(a) > fabs(b)) {
		fact=b/a;
		c=SIGN(1.0/sqrt(1.0+(fact*fact)),a);
		s=fact*c;
	} else {
		fact=a/b;
		s=SIGN(1.0/sqrt(1.0+(fact*fact)),b);
		c=fact*s;
	}
	for (j=i;j<n;j++) {
		y=r(i,j);
		w=r(i+1,j);
		r(i,j)=c*y-s*w;
		r(i+1,j)=s*y+c*w;
	}
	for (j=0;j<n;j++) {
		y=qt(i,j);
		w=qt(i+1,j);
		qt(i,j)=c*y-s*w;
		qt(i+1,j)=s*y+c*w;
	}
}



void NR_single_precision::rsolv(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_IO_DP &b)
{
	int i,j;
	NR_single_precision::DP sum;

	int n=a.rows();
	b(n-1) /= d(n-1);
	for (i=n-2;i>=0;i--) {
		for (sum=0.0,j=i+1;j<n;j++) sum += a(i,j)*b(j);
		b(i)=(b(i)-sum)/d(i);
	}
}



void NR_single_precision::rstrct(NR_single_precision::Mat_O_DP &uc, NR_single_precision::Mat_I_DP &uf)
{
	int ic,iif,jc,jf,ncc;

	int nc=uc.rows();
	ncc=2*nc-2;
	for (jf=2,jc=1;jc<nc-1;jc++,jf+=2) {
		for (iif=2,ic=1;ic<nc-1;ic++,iif+=2) {
			uc(ic,jc)=0.5*uf(iif,jf)+0.125*(uf(iif+1,jf)+uf(iif-1,jf)
				+uf(iif,jf+1)+uf(iif,jf-1));
		}
	}
	for (jc=0,ic=0;ic<nc;ic++,jc+=2) {
		uc(ic,0)=uf(jc,0);
		uc(ic,nc-1)=uf(jc,ncc);
	}
	for (jc=0,ic=0;ic<nc;ic++,jc+=2) {
		uc(0,ic)=uf(0,jc);
		uc(nc-1,ic)=uf(ncc,jc);
	}
}





NR_single_precision::DP NR_single_precision::rtbis(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc)
{
	const int JMAX=40;
	int j;
	NR_single_precision::DP dx,f,fmid,xmid,rtb;

	f=func(x1);
	fmid=func(x2);
	if (f*fmid >= 0.0) NR::nrerror("Root must be bracketed for bisection in rtbis");
	rtb = f < 0.0 ? (dx=x2-x1,x1) : (dx=x1-x2,x2);
	for (j=0;j<JMAX;j++) {
		fmid=func(xmid=rtb+(dx *= 0.5));
		if (fmid <= 0.0) rtb=xmid;
		if (fabs(dx) < xacc || fmid == 0.0) return rtb;
	}
	NR::nrerror("Too many bisections in rtbis");
	return 0.0;
}





NR_single_precision::DP NR_single_precision::rtflsp(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc)
{
	const int MAXIT=30;
	int j;
	NR_single_precision::DP fl,fh,xl,xh,dx,del,f,rtf;

	fl=func(x1);
	fh=func(x2);
	if (fl*fh > 0.0) NR::nrerror("Root must be bracketed in rtflsp");
	if (fl < 0.0) {
		xl=x1;
		xh=x2;
	} else {
		xl=x2;
		xh=x1;
		SWAP(fl,fh);
	}
	dx=xh-xl;
	for (j=0;j<MAXIT;j++) {
		rtf=xl+dx*fl/(fl-fh);
		f=func(rtf);
		if (f < 0.0) {
			del=xl-rtf;
			xl=rtf;
			fl=f;
		} else {
			del=xh-rtf;
			xh=rtf;
			fh=f;
		}
		dx=xh-xl;
		if (fabs(del) < xacc || f == 0.0) return rtf;
	}
	NR::nrerror("Maximum number of iterations exceeded in rtflsp");
	return 0.0;
}





NR_single_precision::DP NR_single_precision::rtnewt(void funcd(const NR_single_precision::DP, NR_single_precision::DP &, NR_single_precision::DP &), const NR_single_precision::DP x1, const NR_single_precision::DP x2,
	const NR_single_precision::DP xacc)
{
	const int JMAX=20;
	int j;
	NR_single_precision::DP df,dx,f,rtn;

	rtn=0.5*(x1+x2);
	for (j=0;j<JMAX;j++) {
		funcd(rtn,f,df);
		dx=f/df;
		rtn -= dx;
		if ((x1-rtn)*(rtn-x2) < 0.0)
			NR::nrerror("Jumped out of brackets in rtnewt");
		if (fabs(dx) < xacc) return rtn;
	}
	NR::nrerror("Maximum number of iterations exceeded in rtnewt");
	return 0.0;
}





NR_single_precision::DP NR_single_precision::rtsafe(void funcd(const NR_single_precision::DP, NR_single_precision::DP &, NR_single_precision::DP &), const NR_single_precision::DP x1, const NR_single_precision::DP x2,
	const NR_single_precision::DP xacc)
{
	const int MAXIT=100;
	int j;
	NR_single_precision::DP df,dx,dxold,f,fh,fl,temp,xh,xl,rts;

	funcd(x1,fl,df);
	funcd(x2,fh,df);
	if ((fl > 0.0 && fh > 0.0) || (fl < 0.0 && fh < 0.0))
		NR::nrerror("Root must be bracketed in rtsafe");
	if (fl == 0.0) return x1;
	if (fh == 0.0) return x2;
	if (fl < 0.0) {
		xl=x1;
		xh=x2;
	} else {
		xh=x1;
		xl=x2;
	}
	rts=0.5*(x1+x2);
	dxold=fabs(x2-x1);
	dx=dxold;
	funcd(rts,f,df);
	for (j=0;j<MAXIT;j++) {
		if ((((rts-xh)*df-f)*((rts-xl)*df-f) > 0.0)
			|| (fabs(2.0*f) > fabs(dxold*df))) {
			dxold=dx;
			dx=0.5*(xh-xl);
			rts=xl+dx;
			if (xl == rts) return rts;
		} else {
			dxold=dx;
			dx=f/df;
			temp=rts;
			rts -= dx;
			if (temp == rts) return rts;
		}
		if (fabs(dx) < xacc) return rts;
		funcd(rts,f,df);
		if (f < 0.0)
			xl=rts;
		else
			xh=rts;
	}
	NR::nrerror("Maximum number of iterations exceeded in rtsafe");
	return 0.0;
}





NR_single_precision::DP NR_single_precision::rtsec(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc)
{
	const int MAXIT=30;
	int j;
	NR_single_precision::DP fl,f,dx,xl,rts;

	fl=func(x1);
	f=func(x2);
	if (fabs(fl) < fabs(f)) {
		rts=x1;
		xl=x2;
		SWAP(fl,f);
	} else {
		xl=x1;
		rts=x2;
	}
	for (j=0;j<MAXIT;j++) {
		dx=(xl-rts)*f/(f-fl);
		xl=rts;
		fl=f;
		rts += dx;
		f=func(rts);
		if (fabs(dx) < xacc || f == 0.0) return rts;
	}
	NR::nrerror("Maximum number of iterations exceeded in rtsec");
	return 0.0;
}



extern NR_single_precision::Vec_DP *x_p;
extern NR_single_precision::Mat_DP *d_p;

void NR_single_precision::rzextr(const int iest, const NR_single_precision::DP xest, NR_single_precision::Vec_I_DP &yest, NR_single_precision::Vec_O_DP &yz,
	NR_single_precision::Vec_O_DP &dy)
{
	int j,k,nv;
	NR_single_precision::DP yy,v,ddy,c,b1,b;

	nv=yz.size();
	NR_single_precision::Vec_DP fx(iest+1);
	NR_single_precision::Vec_DP &x=*x_p;
	NR_single_precision::Mat_DP &d=*d_p;
	x(iest)=xest;
	if (iest == 0)
		for (j=0;j<nv;j++) {
			yz(j)=yest(j);
			d(j,0)=yest(j);
			dy(j)=yest(j);
		}
	else {
		for (k=0;k<iest;k++)
			fx(k+1)=x(iest-(k+1))/xest;
		for (j=0;j<nv;j++) {
			v=d(j,0);
			d(j,0)=yy=c=yest(j);
			for (k=1;k<=iest;k++) {
				b1=fx(k)*v;
				b=b1-c;
				if (b != 0.0) {
					b=(c-v)/b;
					ddy=c*b;
					c=b1*b;
				} else
					ddy=v;
				if (k != iest) v=d(j,k);
				d(j,k)=ddy;
				yy += ddy;
			}
			dy(j)=ddy;
			yz(j)=yy;
		}
	}
}


void NR_single_precision::savgol(NR_single_precision::Vec_O_DP &c, const int np, const int nl, const int nr,
	const int ld, const int m)
{
	int j,k,imj,ipj,kk,mm;
	NR_single_precision::DP d,fac,sum;

	if (np < nl+nr+1 || nl < 0 || nr < 0 || ld > m || nl+nr < m)
		NR::nrerror("bad args in savgol");
	NR_single_precision::Vec_INT indx(m+1);
	NR_single_precision::Mat_DP a(m+1,m+1);
	NR_single_precision::Vec_DP b(m+1);
	for (ipj=0;ipj<=(m << 1);ipj++) {
		sum=(ipj ? 0.0 : 1.0);
		for (k=1;k<=nr;k++) sum += pow(NR_single_precision::DP(k),NR_single_precision::DP(ipj));
		for (k=1;k<=nl;k++) sum += pow(NR_single_precision::DP(-k),NR_single_precision::DP(ipj));
		mm=MIN(ipj,2*m-ipj);
		for (imj = -mm;imj<=mm;imj+=2) a((ipj+imj)/2,(ipj-imj)/2)=sum;
	}
	ludcmp(a,indx,d);
	for (j=0;j<m+1;j++) b(j)=0.0;
	b(ld)=1.0;
	lubksb(a,indx,b);
	for (kk=0;kk<np;kk++) c(kk)=0.0;
	for (k = -nl;k<=nr;k++) {
		sum=b(0);
		fac=1.0;
		for (mm=1;mm<=m;mm++) sum += b(mm)*(fac *= k);
		kk=(np-k) % np;
		c(kk)=sum;
	}
}

#include <string>
#include <iostream>
#include <iomanip>



void NR_single_precision::scrsho(NR_single_precision::DP fx(const NR_single_precision::DP))
{
	Msg::error("scrsho");
	/*
	const int ISCR=60, JSCR=21;
	const char BLANK=' ', ZERO='-', YY='l', XX='-', FF='x';
	int jz,j,i;
	NR_single_precision::DP ysml,ybig,x2,x1,x,dyj,dx;
	NR_single_precision::Vec_DP y(ISCR);
	string scr(JSCR);

	for (;;) {
		cout << endl << "Enter x1 x2 (x1=x2 to stop):" << endl;
		cin >> x1 >> x2;
		if (x1 == x2) break;
		scr(0)=YY;
		for (i=1;i<(ISCR-1);i++)
			scr(0) += XX;
		scr(0) += YY;
		for (j=1;j<(JSCR-1);j++) {
			scr(j)=YY;
			for (i=1;i<(ISCR-1);i++)
				scr(j) += BLANK;
			scr(j) += YY;
		}
		scr(JSCR-1)=scr(0);
		dx=(x2-x1)/(ISCR-1);
		x=x1;
		ysml=ybig=0.0;
		for (i=0;i<ISCR;i++) {
			y(i)=fx(x);
			if (y(i) < ysml) ysml=y(i);
			if (y(i) > ybig) ybig=y(i);
			x += dx;
		}
		if (ybig == ysml) ybig=ysml+1.0;
		dyj=(JSCR-1)/(ybig-ysml);
		jz=int(-ysml*dyj);
		for (i=0;i<ISCR;i++) {
			scr(jz,i)=ZERO;
			j=int((y(i)-ysml)*dyj);
			scr(j,i)=FF;
		}
		cout << fixed ;
		cout << setw(11) << ybig << " " << scr(JSCR-1) << endl;
		for (j=JSCR-2;j>=1;j--)
			cout << "            " << scr(j) << endl;
		cout << setw(11) << ysml << " " << scr(0) << endl;
		cout << setw(19) << x1 << setw(55) << x2;
	}*/
}



NR_single_precision::DP NR_single_precision::select(const int k, NR_single_precision::Vec_IO_DP &arr)
{
	int i,ir,j,l,mid;
	NR_single_precision::DP a;

	int n=arr.size();
	l=0;
	ir=n-1;
	for (;;) {
		if (ir <= l+1) {
			if (ir == l+1 && arr(ir) < arr(l))
				SWAP(arr(l),arr(ir));
			return arr(k);
		} else {
			mid=(l+ir) >> 1;
			SWAP(arr(mid),arr(l+1));
			if (arr(l) > arr(ir))
				SWAP(arr(l),arr(ir));
			if (arr(l+1) > arr(ir))
				SWAP(arr(l+1),arr(ir));
			if (arr(l) > arr(l+1))
				SWAP(arr(l),arr(l+1));
			i=l+1;
			j=ir;
			a=arr(l+1);
			for (;;) {
				do i++; while (arr(i) < a);
				do j--; while (arr(j) > a);
				if (j < i) break;
				SWAP(arr(i),arr(j));
			}
			arr(l+1)=arr(j);
			arr(j)=a;
			if (j >= k) ir=j-1;
			if (j <= k) l=i;
		}
	}
}



NR_single_precision::DP NR_single_precision::selip(const int k, NR_single_precision::Vec_I_DP &arr)
{
	const int M=64;
	const NR_single_precision::DP BIG=1.0e30;
	int i,j,jl,jm,ju,kk,mm,nlo,nxtmm;
	NR_single_precision::DP ahi,alo,sum;
	NR_single_precision::Vec_INT isel(M+2);
	NR_single_precision::Vec_DP sel(M+2);

	int n=arr.size();
	if (k < 0 || k > n-1) NR::nrerror("bad input to selip");
	kk=k;
	ahi=BIG;
	alo = -BIG;
	for (;;) {
		mm=nlo=0;
		sum=0.0;
		nxtmm=M+1;
		for (i=0;i<n;i++) {
			if (arr(i) >= alo && arr(i) <= ahi) {
				mm++;
				if (arr(i) == alo) nlo++;
				if (mm <= M) sel(mm-1)=arr(i);
				else if (mm == nxtmm) {
					nxtmm=mm+mm/M;
					sel((i+2+mm+kk) % M)=arr(i);
				}
				sum += arr(i);
			}
		}
		if (kk < nlo) {
			return alo;
		}
		else if (mm < M+1) {
			shell(mm,sel);
			ahi = sel(kk);
			return ahi;
		}
		sel(M)=sum/mm;
		shell(M+1,sel);
		sel(M+1)=ahi;
		for (j=0;j<M+2;j++) isel(j)=0;
		for (i=0;i<n;i++) {
			if (arr(i) >= alo && arr(i) <= ahi) {
				jl=0;
				ju=M+2;
				while (ju-jl > 1) {
					jm=(ju+jl)/2;
					if (arr(i) >= sel(jm-1)) jl=jm;
					else ju=jm;
				}
				isel(ju-1)++;
			}
		}
		j=0;
		while (kk >= isel(j)) {
			alo=sel(j);
			kk -= isel(j++);
		}
		ahi=sel(j);
	}
}

#include <iostream>
#include <iomanip>



namespace sfroid
{
const int M=40;
int mm,n,mpt=M+1;
NR_single_precision::DP h,c2=0.0,anorm;
NR_single_precision::Vec_DP *x_p;

int main(void)	// Program sfroid
{
	const int NE=3,NB=1,NYJ=NE,NYK=M+1;
	int i,itmax,k;
	NR_single_precision::DP conv,deriv,fac1,fac2,q1,slowc;
	NR_single_precision::Vec_INT indexv(NE);
	NR_single_precision::Vec_DP scalv(NE);
	NR_single_precision::Mat_DP y(NYJ,NYK);

	x_p=new NR_single_precision::Vec_DP(M+1);
	NR_single_precision::Vec_DP &x=*x_p;
	itmax=100;
	conv=1.0e-14;
	slowc=1.0;
	h=1.0/M;
	cout << endl << "Enter m n" << endl;
	cin >> mm >> n;
	if ((n+mm & 1) != 0) {
		indexv(0)=0;
		indexv(1)=1;
		indexv(2)=2;
	} else {
		indexv(0)=1;
		indexv(1)=0;
		indexv(2)=2;
	}
	anorm=1.0;
	if (mm != 0) {
		q1=n;
		for (i=1;i<=mm;i++) anorm = -0.5*anorm*(n+i)*(q1--/i);
	}
	for (k=0;k<M;k++) {
		x(k)=k*h;
		fac1=1.0-x(k)*x(k);
		fac2=exp((-mm/2.0)*log(fac1));
		y(0,k)=NR_single_precision::plgndr(n,mm,x(k))*fac2;
		deriv = -((n-mm+1)*NR_single_precision::plgndr(n+1,mm,x(k))-
			(n+1)*x(k)*NR_single_precision::plgndr(n,mm,x(k)))/fac1;
		y(1,k)=mm*x(k)*y(0,k)/fac1+deriv*fac2;
		y(2,k)=n*(n+1)-mm*(mm+1);
	}
	x(M)=1.0;
	y(0,M)=anorm;
	y(2,M)=n*(n+1)-mm*(mm+1);
	y(1,M)=(y(2,M)-c2)*y(0,M)/(2.0*(mm+1.0));
	scalv(0)=fabs(anorm);
	scalv(1)=(y(1,M) > scalv(0) ? y(1,M) : scalv(0));
	scalv(2)=(y(2,M) > 1.0 ? y(2,M) : 1.0);
	for (;;) {
		cout << endl << "Enter c**2 or 999 to end" << endl;
		cin >> c2;
		if (c2 == 999) {
			delete x_p;
			return 0;
		}
		NR_single_precision::solvde(itmax,conv,slowc,scalv,indexv,NB,y);
		cout << endl << " m = " << mm;
		cout << "  n = " << n << "  c**2 = ";
		cout << fixed << c2;
		cout << " lamda = " << (y(2,0)+mm*(mm+1));
		cout << endl;
	}
}

}
extern int sfroid::mm,sfroid::n,sfroid::mpt;
extern NR_single_precision::DP sfroid::h,sfroid::c2,sfroid::anorm;
extern NR_single_precision::Vec_DP *sfroid::x_p;

void NR_single_precision::difeq(const int k, const int k1, const int k2, const int jsf,
	const int is1, const int isf, NR_single_precision::Vec_I_INT &indexv, NR_single_precision::Mat_O_DP &s,
	NR_single_precision::Mat_I_DP &y)
{
	NR_single_precision::DP temp,temp1,temp2;

	NR_single_precision::Vec_DP &x=*sfroid::x_p;
	if (k == k1) {
		if ((sfroid::n+sfroid::mm & 1) != 0) {
			s(2,3+indexv(0))=1.0;
			s(2,3+indexv(1))=0.0;
			s(2,3+indexv(2))=0.0;
			s(2,jsf)=y(0,0);
		} else {
			s(2,3+indexv(0))=0.0;
			s(2,3+indexv(1))=1.0;
			s(2,3+indexv(2))=0.0;
			s(2,jsf)=y(1,0);
		}
	} else if (k > k2-1) {
		s(0,3+indexv(0)) = -(y(2,sfroid::mpt-1)-sfroid::c2)/(2.0*(sfroid::mm+1.0));
		s(0,3+indexv(1))=1.0;
		s(0,3+indexv(2)) = -y(0,sfroid::mpt-1)/(2.0*(sfroid::mm+1.0));
		s(0,jsf)=y(1,sfroid::mpt-1)-(y(2,sfroid::mpt-1)-sfroid::c2)*y(0,sfroid::mpt-1)/
			(2.0*(sfroid::mm+1.0));
		s(1,3+indexv(0))=1.0;
		s(1,3+indexv(1))=0.0;
		s(1,3+indexv(2))=0.0;
		s(1,jsf)=y(0,sfroid::mpt-1)-sfroid::anorm;
	} else {
		s(0,indexv(0)) = -1.0;
		s(0,indexv(1)) = -0.5*sfroid::h;
		s(0,indexv(2))=0.0;
		s(0,3+indexv(0))=1.0;
		s(0,3+indexv(1)) = -0.5*sfroid::h;
		s(0,3+indexv(2))=0.0;
		temp1=x(k)+x(k-1);
		temp=sfroid::h/(1.0-temp1*temp1*0.25);
		temp2=0.5*(y(2,k)+y(2,k-1))-sfroid::c2*0.25*temp1*temp1;
		s(1,indexv(0))=temp*temp2*0.5;
		s(1,indexv(1)) = -1.0-0.5*temp*(sfroid::mm+1.0)*temp1;
		s(1,indexv(2))=0.25*temp*(y(0,k)+y(0,k-1));
		s(1,3+indexv(0))=s(1,indexv(0));
		s(1,3+indexv(1))=2.0+s(1,indexv(1));
		s(1,3+indexv(2))=s(1,indexv(2));
		s(2,indexv(0))=0.0;
		s(2,indexv(1))=0.0;
		s(2,indexv(2)) = -1.0;
		s(2,3+indexv(0))=0.0;
		s(2,3+indexv(1))=0.0;
		s(2,3+indexv(2))=1.0;
		s(0,jsf)=y(0,k)-y(0,k-1)-0.5*sfroid::h*(y(1,k)+y(1,k-1));
		s(1,jsf)=y(1,k)-y(1,k-1)-temp*((x(k)+x(k-1))
			*0.5*(sfroid::mm+1.0)*(y(1,k)+y(1,k-1))-temp2
			*0.5*(y(0,k)+y(0,k-1)));
		s(2,jsf)=y(2,k)-y(2,k-1);
	}
}

void NR_single_precision::shell(const int m, NR_single_precision::Vec_IO_DP &a)
{
	int i,j,inc;
	NR_single_precision::DP v;

	inc=1;
	do {
		inc *= 3;
		inc++;
	} while (inc <= m);
	do {
		inc /= 3;
		for (i=inc;i<m;i++) {
			v=a(i);
			j=i;
			while (a(j-inc) > v) {
				a(j)=a(j-inc);
				j -= inc;
				if (j < inc) break;
			}
			a(j)=v;
		}
	} while (inc > 1);
}







void NR_single_precision::simp1(NR_single_precision::Mat_I_DP &a, const int mm, NR_single_precision::Vec_I_INT &ll, const int nll,
	const int iabf, int &kp, NR_single_precision::DP &bmax)
{
	int k;
	NR_single_precision::DP test;

	if (nll <= 0)
		bmax=0.0;
	else {
		kp=ll(0);
		bmax=a(mm,kp);
		for (k=1;k<nll;k++) {
			if (iabf == 0)
				test=a(mm,ll(k))-bmax;
			else
				test=fabs(a(mm,ll(k)))-fabs(bmax);
			if (test > 0.0) {
				bmax=a(mm,ll(k));
				kp=ll(k);
			}
		}
	}
}



void NR_single_precision::simp2(NR_single_precision::Mat_I_DP &a, const int m, const int n, int &ip, const int kp)
{
	const NR_single_precision::DP EPS=1.0e-14;
	int k,i;
	NR_single_precision::DP qp,q0,q,q1;

	ip=0;
	for (i=0;i<m;i++)
		if (a(i+1,kp) < -EPS) break;
	if (i+1>m) return;
	q1 = -a(i+1,0)/a(i+1,kp);
	ip=i+1;
	for (i=ip;i<m;i++) {
		if (a(i+1,kp) < -EPS) {
			q = -a(i+1,0)/a(i+1,kp);
			if (q < q1) {
				ip=i+1;
				q1=q;
			} else if (q == q1) {
				for (k=0;k<n;k++) {
					qp = -a(ip,k+1)/a(ip,kp);
					q0 = -a(i,k+1)/a(i,kp);
					if (q0 != qp) break;
				}
				if (q0 < qp) ip=i+1;
			}
		}
	}
}



void NR_single_precision::simp3(NR_single_precision::Mat_IO_DP &a, const int i1, const int k1, const int ip,
	const int kp)
{
	int ii,kk;
	NR_single_precision::DP piv;

	piv=1.0/a(ip,kp);
	for (ii=0;ii<i1+1;ii++)
		if (ii != ip) {
			a(ii,kp) *= piv;
			for (kk=0;kk<k1+1;kk++)
				if (kk != kp)
					a(ii,kk) -= a(ip,kk)*a(ii,kp);
		}
	for (kk=0;kk<k1+1;kk++)
		if (kk != kp) a(ip,kk) *= -piv;
	a(ip,kp)=piv;
}



void NR_single_precision::simplx(NR_single_precision::Mat_IO_DP &a, const int m1, const int m2, const int m3,
	int &icase, NR_single_precision::Vec_O_INT &izrov, NR_single_precision::Vec_O_INT &iposv)
{
	const NR_single_precision::DP EPS=1.0e-14;
	int i,k,ip,is,kh,kp,nl1;
	NR_single_precision::DP q1,bmax;

	int m=a.rows()-2;
	int n=a.cols()-1;
	if (m != (m1+m2+m3)) NR::nrerror("Bad input constraint counts in simplx");
	NR_single_precision::Vec_INT l1(n+1),l3(m);
	nl1=n;
	for (k=0;k<n;k++) {
		l1(k)=k+1;
		izrov(k)=k;
	}
	for (i=1;i<=m;i++) {
		if (a(i,0) < 0.0) NR::nrerror("Bad input tableau in simplx");
		iposv(i-1)=n+i-1;
	}
	if (m2+m3 != 0) {
		for (i=0;i<m2;i++) l3(i)=1;
		for (k=0;k<(n+1);k++) {
			q1=0.0;
			for (i=m1+1;i<m+1;i++) q1 += a(i,k);
			a(m+1,k) = -q1;
		}
		for (;;) {
			simp1(a,m+1,l1,nl1,0,kp,bmax);
			if (bmax <= EPS && a(m+1,0) < -EPS) {
				icase = -1;
				return;
			} else if (bmax <= EPS && a(m+1,0) <= EPS) {
				for (ip=m1+m2+1;ip<m+1;ip++) {
					if (iposv(ip-1) == (ip+n-1)) {
						simp1(a,ip,l1,nl1,1,kp,bmax);
						if (bmax > EPS)
							goto one;
					}
				}
				for (i=m1+1;i<=m1+m2;i++)
					if (l3(i-m1-1) == 1)
						for (k=0;k<n+1;k++)
							a(i,k)= -a(i,k);
				break;
			}
			simp2(a,m,n,ip,kp);
			if (ip == 0) {
				icase = -1;
				return;
			}
	one:	simp3(a,m+1,n,ip,kp);
			if (iposv(ip-1) >= (n+m1+m2)) {
				for (k=0;k<nl1;k++)
					if (l1(k) == kp) break;
				--nl1;
				for (is=k;is<nl1;is++) l1(is)=l1(is+1);
			} else {
				kh=iposv(ip-1)-m1-n+1;
				if (kh >= 1 && l3(kh-1)) {
					l3(kh-1)=0;
					++a(m+1,kp);
					for (i=0;i<m+2;i++)
						a(i,kp)= -a(i,kp);
				}
			}
			SWAP(izrov(kp-1),iposv(ip-1));
		}
	}
	for (;;) {
		simp1(a,0,l1,nl1,0,kp,bmax);
		if (bmax <= EPS) {
			icase=0;
			return;
		}
		simp2(a,m,n,ip,kp);
		if (ip == 0) {
			icase=1;
			return;
		}
		simp3(a,m,n,ip,kp);
		SWAP(izrov(kp-1),iposv(ip-1));
	}
}



void NR_single_precision::simpr(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, NR_single_precision::Vec_I_DP &dfdx, NR_single_precision::Mat_I_DP &dfdy,
	const NR_single_precision::DP xs, const NR_single_precision::DP htot, const int nstep, NR_single_precision::Vec_O_DP &yout,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	int i,j,nn;
	NR_single_precision::DP d,h,x;

	int n=y.size();
	NR_single_precision::Mat_DP a(n,n);
	NR_single_precision::Vec_INT indx(n);
	NR_single_precision::Vec_DP del(n),ytemp(n);
	h=htot/nstep;
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) a(i,j) = -h*dfdy(i,j);
		++a(i,i);
	}
	ludcmp(a,indx,d);
	for (i=0;i<n;i++)
		yout(i)=h*(dydx(i)+h*dfdx(i));
	lubksb(a,indx,yout);
	for (i=0;i<n;i++)
		ytemp(i)=y(i)+(del(i)=yout(i));
	x=xs+h;
	derivs(x,ytemp,yout);
	for (nn=2;nn<=nstep;nn++) {
		for (i=0;i<n;i++)
			yout(i)=h*yout(i)-del(i);
		lubksb(a,indx,yout);
		for (i=0;i<n;i++) ytemp(i) += (del(i) += 2.0*yout(i));
		x += h;
		derivs(x,ytemp,yout);
	}
	for (i=0;i<n;i++)
		yout(i)=h*yout(i)-del(i);
	lubksb(a,indx,yout);
	for (i=0;i<n;i++)
		yout(i) += ytemp(i);
}





void NR_single_precision::sinft(NR_single_precision::Vec_IO_DP &y)
{
	int j;
	NR_single_precision::DP sum,y1,y2,theta,wi=0.0,wr=1.0,wpi,wpr,wtemp;

	int n=y.size();
	theta=3.141592653589793238/NR_single_precision::DP(n);
	wtemp=sin(0.5*theta);
	wpr= -2.0*wtemp*wtemp;
	wpi=sin(theta);
	y(0)=0.0;
	for (j=1;j<(n>>1)+1;j++) {
		wr=(wtemp=wr)*wpr-wi*wpi+wr;
		wi=wi*wpr+wtemp*wpi+wi;
		y1=wi*(y(j)+y(n-j));
		y2=0.5*(y(j)-y(n-j));
		y(j)=y1+y2;
		y(n-j)=y1-y2;
	}
	realft(y,1);
	y(0)*=0.5;
	sum=y(1)=0.0;
	for (j=0;j<n-1;j+=2) {
		sum += y(j);
		y(j)=y(j+1);
		y(j+1)=sum;
	}
}





void NR_single_precision::slvsm2(NR_single_precision::Mat_O_DP &u, NR_single_precision::Mat_I_DP &rhs)
{
	int i,j;
	NR_single_precision::DP disc,fact,h=0.5;

	for (i=0;i<3;i++)
		for (j=0;j<3;j++)
			u(i,j)=0.0;
	fact=2.0/(h*h);
	disc=sqrt(fact*fact+rhs(1,1));
	u(1,1)= -rhs(1,1)/(fact+disc);
}



void NR_single_precision::slvsml(NR_single_precision::Mat_O_DP &u, NR_single_precision::Mat_I_DP &rhs)
{
	int i,j;
	NR_single_precision::DP h=0.5;

	for (i=0;i<3;i++)
		for (j=0;j<3;j++)
			u(i,j)=0.0;
	u(1,1) = -h*h*rhs(1,1)/4.0;
}





void NR_single_precision::sncndn(const NR_single_precision::DP uu, const NR_single_precision::DP emmc, NR_single_precision::DP &sn, NR_single_precision::DP &cn, NR_single_precision::DP &dn)
{
	const NR_single_precision::DP CA=1.0e-8;
	bool bo;
	int i,ii,l;
	NR_single_precision::DP a,b,c,d,emc,u;
	NR_single_precision::Vec_DP em(13),en(13);

	emc=emmc;
	u=uu;
	if (emc != 0.0) {
		bo=(emc < 0.0);
		if (bo) {
			d=1.0-emc;
			emc /= -1.0/d;
			u *= (d=sqrt(d));
		}
		a=1.0;
		dn=1.0;
		for (i=0;i<13;i++) {
			l=i;
			em(i)=a;
			en(i)=(emc=sqrt(emc));
			c=0.5*(a+emc);
			if (fabs(a-emc) <= CA*a) break;
			emc *= a;
			a=c;
		}
		u *= c;
		sn=sin(u);
		cn=cos(u);
		if (sn != 0.0) {
			a=cn/sn;
			c *= a;
			for (ii=l;ii>=0;ii--) {
				b=em(ii);
				a *= c;
				c *= dn;
				dn=(en(ii)+a)/(b+a);
				a=c/b;
			}
			a=1.0/sqrt(c*c+1.0);
			sn=(sn >= 0.0 ? a : -a);
			cn=c*sn;
		}
		if (bo) {
			a=dn;
			dn=cn;
			cn=a;
			sn /= d;
		}
	} else {
		cn=1.0/cosh(u);
		dn=cn;
		sn=tanh(u);
	}
}





NR_single_precision::DP NR_single_precision::snrm(NR_single_precision::Vec_I_DP &sx, const int itol)
{
	int i,isamax;
	NR_single_precision::DP ans;

	int n=sx.size();
	if (itol <= 3) {
		ans = 0.0;
		for (i=0;i<n;i++) ans += sx(i)*sx(i);
		return sqrt(ans);
	} else {
		isamax=0;
		for (i=0;i<n;i++) {
			if (fabs(sx(i)) > fabs(sx(isamax))) isamax=i;
		}
		return fabs(sx(isamax));
	}
}



void NR_single_precision::sobseq(const int n, NR_single_precision::Vec_O_DP &x)
{
	Msg::error("sobseq");
	/*
	const int MAXBIT=30,MAXDIM=6;
	int j,k,l;
	unsigned long i,im,ipp;
	static int mdeg(MAXDIM)={1,2,3,3,4,4};
	static unsigned long in;
	static Vec_ULNG ix(MAXDIM);
	static Vec_ULNG_p iu(MAXBIT);
	static unsigned long ip(MAXDIM)={0,1,1,2,1,4};
	static unsigned long iv(MAXDIM*MAXBIT)=
		{1,1,1,1,1,1,3,1,3,3,1,1,5,7,7,3,3,5,15,11,5,15,13,9};
	static NR_single_precision::DP fac;

	if (n < 0) {
		for (k=0;k<MAXDIM;k++) ix(k)=0;
		in=0;
		if (iv(0) != 1) return;
		fac=1.0/(1 << MAXBIT);
		for (j=0,k=0;j<MAXBIT;j++,k+=MAXDIM) iu(j) = &iv(k);
		for (k=0;k<MAXDIM;k++) {
			for (j=0;j<mdeg(k);j++) iu(j,k) <<= (MAXBIT-1-j);
			for (j=mdeg(k);j<MAXBIT;j++) {
				ipp=ip(k);
				i=iu(j-mdeg(k),k);
				i ^= (i >> mdeg(k));
				for (l=mdeg(k)-1;l>=1;l--) {
					if (ipp & 1) i ^= iu(j-l,k);
					ipp >>= 1;
				}
				iu(j,k)=i;
			}
		}
	} else {
		im=in++;
		for (j=0;j<MAXBIT;j++) {
			if (!(im & 1)) break;
			im >>= 1;
		}
		if (j >= MAXBIT) NR::nrerror("MAXBIT too small in sobseq");
		im=j*MAXDIM;
		for (k=0;k<MIN(n,MAXDIM);k++) {
			ix(k) ^= iv(im+k);
			x(k)=ix(k)*fac;
		}
	}*/
}

#include <iostream>
#include <iomanip>




void NR_single_precision::solvde(const int itmax, const NR_single_precision::DP conv, const NR_single_precision::DP slowc,
	NR_single_precision::Vec_I_DP &scalv, NR_single_precision::Vec_I_INT &indexv, const int nb, NR_single_precision::Mat_IO_DP &y)
{
	int ic1,ic2,ic3,ic4,it,j,j1,j2,j3,j4,j5,j6,j7,j8,j9;
	int jc1,jcf,jv,k,k1,k2,km,kp,nvars;
	NR_single_precision::DP err,errj,fac,vmax,vz;

	int ne=y.rows();
	int m=y.cols();
	NR_single_precision::Vec_INT kmax(ne);
	NR_single_precision::Vec_DP ermax(ne);
	Mat3D_DP c(ne,ne-nb+1,m+1);
	NR_single_precision::Mat_DP s(ne,2*ne+1);
	k1=0; k2=m;
	nvars=ne*m;
	j1=0,j2=nb,j3=nb,j4=ne,j5=j4+j1;
	j6=j4+j2,j7=j4+j3,j8=j4+j4,j9=j8+j1;
	ic1=0,ic2=ne-nb,ic3=ic2,ic4=ne;
	jc1=0,jcf=ic3;
	for (it=0;it<itmax;it++) {
		k=k1;
		difeq(k,k1,k2,j9,ic3,ic4,indexv,s,y);
		pinvs(ic3,ic4,j5,j9,jc1,k1,c,s);
		for (k=k1+1;k<k2;k++) {
			kp=k;
			difeq(k,k1,k2,j9,ic1,ic4,indexv,s,y);
			red(ic1,ic4,j1,j2,j3,j4,j9,ic3,jc1,jcf,kp,c,s);
			pinvs(ic1,ic4,j3,j9,jc1,k,c,s);
		}
		k=k2;
		difeq(k,k1,k2,j9,ic1,ic2,indexv,s,y);
		red(ic1,ic2,j5,j6,j7,j8,j9,ic3,jc1,jcf,k2,c,s);
		pinvs(ic1,ic2,j7,j9,jcf,k2,c,s);
		bksub(ne,nb,jcf,k1,k2,c);
		err=0.0;
		for (j=0;j<ne;j++) {
			jv=indexv(j);
			errj=vmax=0.0;
			km=0;
			for (k=k1;k<k2;k++) {
				vz=fabs(c(jv,0,k));
				if (vz > vmax) {
					vmax=vz;
					km=k+1;
				}
				errj += vz;
			}
			err += errj/scalv(j);
			ermax(j)=c(jv,0,km-1)/scalv(j);
			kmax(j)=km;
		}
		err /= nvars;
		fac=(err > slowc ? slowc/err : 1.0);
		for (j=0;j<ne;j++) {
			jv=indexv(j);
			for (k=k1;k<k2;k++)
			y(j,k) -= fac*c(jv,0,k);
		}
		cout << "Iter.";
		cout << "Error" <<  "FAC" << endl;
		cout << it;
		cout << fixed << err;
		cout << fac << endl;
		if (err < conv) return;
	}
	NR::nrerror("Too many iterations in solvde");
}





void NR_single_precision::sor(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &b, NR_single_precision::Mat_I_DP &c, NR_single_precision::Mat_I_DP &d, NR_single_precision::Mat_I_DP &e,
	NR_single_precision::Mat_I_DP &f, NR_single_precision::Mat_IO_DP &u, const NR_single_precision::DP rjac)
{
	const int MAXITS=1000;
	const NR_single_precision::DP EPS=1.0e-13;
	int j,l,n,ipass,jsw,lsw;
	NR_single_precision::DP anorm,anormf=0.0,omega=1.0,resid;

	int jmax=a.rows();
	for (j=1;j<jmax-1;j++)
		for (l=1;l<jmax-1;l++)
			anormf += fabs(f(j,l));
	for (n=0;n<MAXITS;n++) {
		anorm=0.0;
		jsw=1;
		for (ipass=0;ipass<2;ipass++) {
			lsw=jsw;
			for (j=1;j<jmax-1;j++) {
				for (l=lsw;l<jmax-1;l+=2) {
					resid=a(j,l)*u(j+1,l)+b(j,l)*u(j-1,l)
						+c(j,l)*u(j,l+1)+d(j,l)*u(j,l-1)
						+e(j,l)*u(j,l)-f(j,l);
					anorm += fabs(resid);
					u(j,l) -= omega*resid/e(j,l);
				}
				lsw=3-lsw;
			}
			jsw=3-jsw;
			omega=(n == 0 && ipass == 0 ? 1.0/(1.0-0.5*rjac*rjac) :
				1.0/(1.0-0.25*rjac*rjac*omega));
		}
		if (anorm < EPS*anormf) return;
	}
	NR::nrerror("MAXITS exceeded");
}



void NR_single_precision::sort(NR_single_precision::Vec_IO_DP &arr)
{
	const int M=7,NSTACK=50;
	int i,ir,j,k,jstack=-1,l=0;
	NR_single_precision::DP a;
	NR_single_precision::Vec_INT istack(NSTACK);

	int n=arr.size();
	ir=n-1;
	for (;;) {
		if (ir-l < M) {
			for (j=l+1;j<=ir;j++) {
				a=arr(j);
				for (i=j-1;i>=l;i--) {
					if (arr(i) <= a) break;
					arr(i+1)=arr(i);
				}
				arr(i+1)=a;
			}
			if (jstack < 0) break;
			ir=istack(jstack--);
			l=istack(jstack--);
		} else {
			k=(l+ir) >> 1;
			SWAP(arr(k),arr(l+1));
			if (arr(l) > arr(ir)) {
				SWAP(arr(l),arr(ir));
			}
			if (arr(l+1) > arr(ir)) {
				SWAP(arr(l+1),arr(ir));
			}
			if (arr(l) > arr(l+1)) {
				SWAP(arr(l),arr(l+1));
			}
			i=l+1;
			j=ir;
			a=arr(l+1);
			for (;;) {
				do i++; while (arr(i) < a);
				do j--; while (arr(j) > a);
				if (j < i) break;
				SWAP(arr(i),arr(j));
			}
			arr(l+1)=arr(j);
			arr(j)=a;
			jstack += 2;
			if (jstack >= NSTACK) NR::nrerror("NSTACK too small in sort.");
			if (ir-i+1 >= j-l) {
				istack(jstack)=ir;
				istack(jstack-1)=i;
				ir=j-1;
			} else {
				istack(jstack)=j-1;
				istack(jstack-1)=l;
				l=i;
			}
		}
	}
}



void NR_single_precision::sort2(NR_single_precision::Vec_IO_DP &arr, NR_single_precision::Vec_IO_DP &brr)
{
	const int M=7,NSTACK=50;
	int i,ir,j,k,jstack=-1,l=0;
	NR_single_precision::DP a,b;
	NR_single_precision::Vec_INT istack(NSTACK);

	int n=arr.size();
	ir=n-1;
	for (;;) {
		if (ir-l < M) {
			for (j=l+1;j<=ir;j++) {
				a=arr(j);
				b=brr(j);
				for (i=j-1;i>=l;i--) {
					if (arr(i) <= a) break;
					arr(i+1)=arr(i);
					brr(i+1)=brr(i);
				}
				arr(i+1)=a;
				brr(i+1)=b;
			}
			if (jstack < 0) break;
			ir=istack(jstack--);
			l=istack(jstack--);
		} else {
			k=(l+ir) >> 1;
			SWAP(arr(k),arr(l+1));
			SWAP(brr(k),brr(l+1));
			if (arr(l) > arr(ir)) {
				SWAP(arr(l),arr(ir));
				SWAP(brr(l),brr(ir));
			}
			if (arr(l+1) > arr(ir)) {
				SWAP(arr(l+1),arr(ir));
				SWAP(brr(l+1),brr(ir));
			}
			if (arr(l) > arr(l+1)) {
				SWAP(arr(l),arr(l+1));
				SWAP(brr(l),brr(l+1));
			}
			i=l+1;
			j=ir;
			a=arr(l+1);
			b=brr(l+1);
			for (;;) {
				do i++; while (arr(i) < a);
				do j--; while (arr(j) > a);
				if (j < i) break;
				SWAP(arr(i),arr(j));
				SWAP(brr(i),brr(j));
			}
			arr(l+1)=arr(j);
			arr(j)=a;
			brr(l+1)=brr(j);
			brr(j)=b;
			jstack += 2;
			if (jstack >= NSTACK) NR::nrerror("NSTACK too small in sort2.");
			if (ir-i+1 >= j-l) {
				istack(jstack)=ir;
				istack(jstack-1)=i;
				ir=j-1;
			} else {
				istack(jstack)=j-1;
				istack(jstack-1)=l;
				l=i;
			}
		}
	}
}



void NR_single_precision::sort3(NR_single_precision::Vec_IO_DP &ra, NR_single_precision::Vec_IO_DP &rb, NR_single_precision::Vec_IO_DP &rc)
{
	int j;

	int n=ra.size();
	NR_single_precision::Vec_INT iwksp(n);
	NR_single_precision::Vec_DP wksp(n);
	indexx(ra,iwksp);
	for (j=0;j<n;j++) wksp(j)=ra(j);
	for (j=0;j<n;j++) ra(j)=wksp(iwksp(j));
	for (j=0;j<n;j++) wksp(j)=rb(j);
	for (j=0;j<n;j++) rb(j)=wksp(iwksp(j));
	for (j=0;j<n;j++) wksp(j)=rc(j);
	for (j=0;j<n;j++) rc(j)=wksp(iwksp(j));
}

#include <fstream>




namespace {
	inline NR_single_precision::DP window(const int j, const NR_single_precision::DP a, const NR_single_precision::DP b)
	{
		return 1.0-fabs((j-a)*b);     // Bartlett
		// return 1.0;                // Square
		// return 1.0-SQR((j-a)*b);   // Welch
	}
}

void NR_single_precision::spctrm(ifstream &fp, NR_single_precision::Vec_O_DP &p, const int k, const bool ovrlap)
{
	int mm,m4,kk,joffn,joff,j2,j;
	NR_single_precision::DP w,facp,facm,sumw=0.0,den=0.0;

	int m=p.size();
	mm=m << 1;
	m4=mm << 1;
	NR_single_precision::Vec_DP w1(m4),w2(m);
	facm=m;
	facp=1.0/m;
	for (j=0;j<mm;j++)
		sumw += SQR(window(j,facm,facp));
	for (j=0;j<m;j++) p(j)=0.0;
	if (ovrlap)
		for (j=0;j<m;j++)
			fp >> w2(j);
	for (kk=0;kk<k;kk++) {
		for (joff=0;joff<2;joff++) {
			if (ovrlap) {
				for (j=0;j<m;j++) w1(joff+j+j)=w2(j);
				for (j=0;j<m;j++)
					fp >> w2(j);
				joffn=joff+mm-1;
				for (j=0;j<m;j++) w1(joffn+j+j+1)=w2(j);
			} else {
				for (j=joff;j<m4;j+=2)
					fp >> w1(j);
			}
		}
		for (j=0;j<mm;j++) {
			j2=j+j;
			w=window(j,facm,facp);
			w1(j2+1) *= w;
			w1(j2) *= w;
		}
		four1(w1,1);
		p(0) += (SQR(w1(0))+SQR(w1(1)));
		for (j=1;j<m;j++) {
			j2=j+j;
			p(j) += (SQR(w1(j2+1))+SQR(w1(j2))
				+SQR(w1(m4-j2+1))+SQR(w1(m4-j2)));
		}
		den += sumw;
	}
	den *= m4;
	for (j=0;j<m;j++) p(j) /= den;
}





void NR_single_precision::spear(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &d, NR_single_precision::DP &zd, NR_single_precision::DP &probd,
	NR_single_precision::DP &rs, NR_single_precision::DP &probrs)
{
	int j;
	NR_single_precision::DP vard,t,sg,sf,fac,en3n,en,df,aved;

	int n=data1.size();
	NR_single_precision::Vec_DP wksp1(n),wksp2(n);
	for (j=0;j<n;j++) {
		wksp1(j)=data1(j);
		wksp2(j)=data2(j);
	}
	sort2(wksp1,wksp2);
	crank(wksp1,sf);
	sort2(wksp2,wksp1);
	crank(wksp2,sg);
	d=0.0;
	for (j=0;j<n;j++)
		d += SQR(wksp1(j)-wksp2(j));
	en=n;
	en3n=en*en*en-en;
	aved=en3n/6.0-(sf+sg)/12.0;
	fac=(1.0-sf/en3n)*(1.0-sg/en3n);
	vard=((en-1.0)*en*en*SQR(en+1.0)/36.0)*fac;
	zd=(d-aved)/sqrt(vard);
	probd=erfcc(fabs(zd)/1.4142136);
	rs=(1.0-(6.0/en3n)*(d+(sf+sg)/12.0))/sqrt(fac);
	fac=(rs+1.0)*(1.0-rs);
	if (fac > 0.0) {
		t=rs*sqrt((en-2.0)/fac);
		df=en-2.0;
		probrs=betai(0.5*df,0.5,df/(df+t*t));
	} else
		probrs=0.0;
}





void NR_single_precision::sphbes(const int n, const NR_single_precision::DP x, NR_single_precision::DP &sj, NR_single_precision::DP &sy, NR_single_precision::DP &sjp, NR_single_precision::DP &syp)
{
	const NR_single_precision::DP RTPIO2=1.253314137315500251;
	NR_single_precision::DP factor,order,rj,rjp,ry,ryp;

	if (n < 0 || x <= 0.0) NR::nrerror("bad arguments in sphbes");
	order=n+0.5;
	bessjy(x,order,rj,ry,rjp,ryp);
	factor=RTPIO2/sqrt(x);
	sj=factor*rj;
	sy=factor*ry;
	sjp=factor*rjp-sj/(2.0*x);
	syp=factor*ryp-sy/(2.0*x);
}

#include <iostream>
#include <iomanip>




namespace sphfpt
{

int m,n;
NR_single_precision::DP c2,dx,gmma;

int n2;
NR_single_precision::DP x1,x2,xf;

int main(void)	// Program sphfpt
{
	const int N1=2,N2=1,NTOT=N1+N2;
	const NR_single_precision::DP DXX=1.0e-8;
	bool check;
	int i;
	NR_single_precision::DP q1;
	NR_single_precision::Vec_DP v(NTOT);

	n2=N2;
	dx=DXX;
	for (;;) {
		cout << endl << "input m,n,c-squared (n >= m, c=999 to end)" << endl;
		cin >> m >> n >> c2;
		if (c2 == 999) break;
		if (n < m || m < 0) continue;
		gmma=1.0;
		q1=n;
		for (i=1;i<=m;i++) gmma *= -0.5*(n+i)*(q1--/i);
		v(0)=n*(n+1)-m*(m+1)+c2/2.0;
		v(2)=v(0);
		v(1)=gmma*(1.0-(v(2)-c2)*dx/(2*(m+1)));
		x1= -1.0+dx;
		x2=1.0-dx;
		xf=0.0;
		NR_single_precision::newt(v,check,NR_single_precision::shootf);
		if (check) {
			cout << "shootf failed; bad initial guess" << endl;
		} else {
			cout << "    " << "mu(m,n)" << endl;
			cout << fixed ;
			cout << v(0) << endl;
		}
	}
	return 0;
}

void load1(const NR_single_precision::DP x1, NR_single_precision::Vec_I_DP &v1, NR_single_precision::Vec_O_DP &y)
{
	NR_single_precision::DP y1 = ((n-m & 1) != 0 ? -gmma : gmma);
	y(2)=v1(0);
	y(1) = -(y(2)-c2)*y1/(2*(m+1));
	y(0)=y1+y(1)*dx;
}

void load2(const NR_single_precision::DP x2, NR_single_precision::Vec_I_DP &v2, NR_single_precision::Vec_O_DP &y)
{
	y(2)=v2(1);
	y(0)=v2(0);
	y(1)=(y(2)-c2)*y(0)/(2*(m+1));
}

void score(const NR_single_precision::DP xf, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &f)
{
	int i;

	for (i=0;i<3;i++) f(i)=y(i);
}

void derivs(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dydx)
{
	dydx(0)=y(1);
	dydx(1)=(2.0*x*(m+1.0)*y(1)-(y(2)-c2*x*x)*y(0))/(1.0-x*x);
	dydx(2)=0.0;
}
}

extern int sphfpt::n2;
extern NR_single_precision::DP sphfpt::x1,sphfpt::x2,sphfpt::xf;


void sphfpt::derivs(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dydx);
void sphfpt::load1(const NR_single_precision::DP x1, NR_single_precision::Vec_I_DP &v1, NR_single_precision::Vec_O_DP &y);
void sphfpt::load2(const NR_single_precision::DP x2, NR_single_precision::Vec_I_DP &v2, NR_single_precision::Vec_O_DP &y);
void sphfpt::score(const NR_single_precision::DP xf, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &f);

void NR_single_precision::shootf(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f)
{
	ASSERT(0);
	/*
	const NR_single_precision::DP EPS=1.0e-14;
	int i,nbad,nok;
	NR_single_precision::DP h1,hmin=0.0;

	int nvar=v.size();
	NR_single_precision::Vec_DP f1(nvar),f2(nvar),y(nvar);
	NR_single_precision::Vec_DP v2(&v(n2),nvar-n2);
	kmax=0;
	h1=(x2-x1)/100.0;
	sphfpt::load1(x1,v,y);
	odeint(y,x1,xf,EPS,h1,hmin,nok,nbad,derivs,rkqs);
	sphfpt::score(xf,y,f1);
	sphfpt::load2(x2,v2,y);
	odeint(y,x2,xf,EPS,h1,hmin,nok,nbad,derivs,rkqs);
	sphfpt::score(xf,y,f2);
	for (i=0;i<nvar;i++) f(i)=f1(i)-f2(i);*/
}
#include <iostream>
#include <iomanip>



namespace sphoot
{

	int m,n;
	NR_single_precision::DP c2,dx,gmma;

	int nvar;
	NR_single_precision::DP x1,x2;

	int main(void)	// Program sphoot
	{
		const int N2=1;
		bool check;
		int i;
		NR_single_precision::DP q1;
		NR_single_precision::Vec_DP v(N2);

		dx=1.0e-8;
		nvar=3;
		for (;;) {
			cout << endl << "input m,n,c-squared (999 to end)" << endl;
			cin >> m >> n >> c2;
			if (c2 == 999) break;
			if (n < m || m < 0) continue;
			gmma=1.0;
			q1=n;
			for (i=1;i<=m;i++) gmma *= -0.5*(n+i)*(q1--/i);
			v(0)=n*(n+1)-m*(m+1)+c2/2.0;
			x1= -1.0+dx;
			x2=0.0;
			NR_single_precision::newt(v,check,NR_single_precision::shoot);
			if (check) {
				cout << "shoot failed; bad initial guess" << endl;
			} else {
				cout << "    " << "mu(m,n)" << endl;
				cout << fixed ;
				cout << v(0) << endl;
			}
		}
		return 0;
	}

	void load(const NR_single_precision::DP x1, NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &y)
	{
		NR_single_precision::DP y1 = ((n-m & 1) != 0 ? -gmma : gmma);
		y(2)=v(0);
		y(1) = -(y(2)-c2)*y1/(2*(m+1));
		y(0)=y1+y(1)*dx;
	}

	void score(const NR_single_precision::DP xf, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &f)
	{
		f(0)=((n-m & 1) != 0 ? y(0) : y(1));
	}

	void derivs(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dydx)
	{
		dydx(0)=y(1);
		dydx(1)=(2.0*x*(m+1.0)*y(1)-(y(2)-c2*x*x)*y(0))/(1.0-x*x);
		dydx(2)=0.0;
	}
}

extern int sphoot::nvar;
extern NR_single_precision::DP sphoot::x1,sphoot::x2;


void sphoot::derivs(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dydx);
void sphoot::load(const NR_single_precision::DP x1, NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &y);
void sphoot::score(const NR_single_precision::DP xf, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &f);

void NR_single_precision::shoot(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f)
{
	const NR_single_precision::DP EPS=1.0e-14;
	int nbad,nok;
	NR_single_precision::DP h1,hmin=0.0;

	NR_single_precision::Vec_DP y(sphoot::nvar);
	kmax=0;
	h1=(sphoot::x2-sphoot::x1)/100.0;
	sphoot::load(sphoot::x1,v,y);
	odeint(y,sphoot::x1,sphoot::x2,EPS,h1,hmin,nok,nbad,sphoot::derivs,rkqs);
	sphoot::score(sphoot::x2,y,f);
}






void NR_single_precision::splie2(NR_single_precision::Vec_I_DP &x1a, NR_single_precision::Vec_I_DP &x2a, NR_single_precision::Mat_I_DP &ya, NR_single_precision::Mat_O_DP &y2a)
{
	int m,n,j,k;

	m=x1a.size();
	n=x2a.size();
	NR_single_precision::Vec_DP ya_t(n),y2a_t(n);
	for (j=0;j<m;j++) {
		for (k=0;k<n;k++) ya_t(k)=ya(j,k);
		spline(x2a,ya_t,1.0e30,1.0e30,y2a_t);
		for (k=0;k<n;k++) y2a(j,k)=y2a_t(k);
	}
}



void NR_single_precision::splin2(NR_single_precision::Vec_I_DP &x1a, NR_single_precision::Vec_I_DP &x2a, NR_single_precision::Mat_I_DP &ya, NR_single_precision::Mat_I_DP &y2a,
	const NR_single_precision::DP x1, const NR_single_precision::DP x2, NR_single_precision::DP &y)
{
	int j,k;

	int m=x1a.size();
	int n=x2a.size();
	NR_single_precision::Vec_DP ya_t(n),y2a_t(n),yytmp(m),ytmp(m);
	for (j=0;j<m;j++) {
		for (k=0;k<n;k++) {
			ya_t(k)=ya(j,k);
			y2a_t(k)=y2a(j,k);
		}
		splint(x2a,ya_t,y2a_t,x2,yytmp(j));
	}
	spline(x1a,yytmp,1.0e30,1.0e30,ytmp);
	splint(x1a,yytmp,ytmp,x1,y);
}



void NR_single_precision::spline(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, const NR_single_precision::DP yp1, const NR_single_precision::DP ypn,
	NR_single_precision::Vec_O_DP &y2)
{
	int i,k;
	NR_single_precision::DP p,qn,sig,un;

	int n=y2.size();
	NR_single_precision::Vec_DP u(n-1);
	if (yp1 > 0.99e30)
		y2(0)=u(0)=0.0;
	else {
		y2(0) = -0.5;
		u(0)=(3.0/(x(1)-x(0)))*((y(1)-y(0))/(x(1)-x(0))-yp1);
	}
	for (i=1;i<n-1;i++) {
		sig=(x(i)-x(i-1))/(x(i+1)-x(i-1));
		p=sig*y2(i-1)+2.0;
		y2(i)=(sig-1.0)/p;
		u(i)=(y(i+1)-y(i))/(x(i+1)-x(i)) - (y(i)-y(i-1))/(x(i)-x(i-1));
		u(i)=(6.0*u(i)/(x(i+1)-x(i-1))-sig*u(i-1))/p;
	}
	if (ypn > 0.99e30)
		qn=un=0.0;
	else {
		qn=0.5;
		un=(3.0/(x(n-1)-x(n-2)))*(ypn-(y(n-1)-y(n-2))/(x(n-1)-x(n-2)));
	}
	y2(n-1)=(un-qn*u(n-2))/(qn*y2(n-2)+1.0);
	for (k=n-2;k>=0;k--)
		y2(k)=y2(k)*y2(k+1)+u(k);
}



void NR_single_precision::splint(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, NR_single_precision::Vec_I_DP &y2a, const NR_single_precision::DP x, NR_single_precision::DP &y)
{
	int k;
	NR_single_precision::DP h,b,a;

	int n=xa.size();
	int klo=0;
	int khi=n-1;
	while (khi-klo > 1) {
		k=(khi+klo) >> 1;
		if (xa(k) > x) khi=k;
		else klo=k;
	}
	h=xa(khi)-xa(klo);
	if (h == 0.0) NR::nrerror("Bad xa input to routine splint");
	a=(xa(khi)-x)/h;
	b=(x-xa(klo))/h;
	y=a*ya(klo)+b*ya(khi)+((a*a*a-a)*y2a(klo)
		+(b*b*b-b)*y2a(khi))*(h*h)/6.0;
}



void NR_single_precision::spread(const NR_single_precision::DP y, NR_single_precision::Vec_IO_DP &yy, const NR_single_precision::DP x, const int m)
{
	Msg::error("spread");
	/*
	static int nfac(11)={0,1,1,2,6,24,120,720,5040,40320,362880};
	int ihi,ilo,ix,j,nden;
	NR_single_precision::DP fac;

	int n=yy.size();
	if (m > 10) NR::nrerror("factorial table too small in spread");
	ix=int(x);
	if (x == NR_single_precision::DP(ix)) yy(ix-1) += y;
	else {
		ilo=MIN(MAX(int(x-0.5*m),0),int(n-m));
		ihi=ilo+m;
		nden=nfac(m);
		fac=x-ilo-1;
		for (j=ilo+1;j<ihi;j++) fac *= (x-j-1);
		yy(ihi-1) += y*fac/(nden*(x-ihi));
		for (j=ihi-1;j>ilo;j--) {
			nden=(nden/(j-ilo))*(j-ihi);
			yy(j-1) += y*fac/(nden*(x-j));
		}
	}*/
}



void NR_single_precision::sprsax(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &b)
{
	int i,k;

	int n=x.size();
	if (ija(0) != n+1)
		NR::nrerror("sprsax: mismatched vector and matrix");
	for (i=0;i<n;i++) {
		b(i)=sa(i)*x(i);
		for (k=ija(i);k<ija(i+1);k++) {
			b(i) += sa(k)*x(ija(k));
		}
	}
}





void NR_single_precision::sprsin(NR_single_precision::Mat_I_DP &a, const NR_single_precision::DP thresh, NR_single_precision::Vec_O_DP &sa, NR_single_precision::Vec_O_INT &ija)
{
	int i,j,k;

	int n=a.rows();
	int nmax=sa.size();
	for (j=0;j<n;j++) sa(j)=a(j,j);
	ija(0)=n+1;
	k=n;
	for (i=0;i<n;i++) {
		for (j=0;j<n;j++) {
			if (fabs(a(i,j)) >= thresh && i != j) {
				if (++k > nmax) NR::nrerror("sprsin: sa and ija too small");
				sa(k)=a(i,j);
				ija(k)=j;
			}
		}
		ija(i+1)=k+1;
	}
}



void NR_single_precision::sprspm(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &sb, NR_single_precision::Vec_I_INT &ijb,
	NR_single_precision::Vec_O_DP &sc, NR_single_precision::Vec_I_INT &ijc)
{
	int i,ijma,ijmb,j,m,ma,mb,mbb,mn;
	NR_single_precision::DP sum;

	if (ija(0) != ijb(0) || ija(0) != ijc(0))
		NR::nrerror("sprspm: sizes do not match");
	for (i=0;i<ijc(0)-1;i++) {
		m=i+1;
		j=m-1;
		mn=ijc(i);
		sum=sa(i)*sb(i);
		for (;;) {
			mb=ijb(j);
			for (ma=ija(i);ma<ija(i+1);ma++) {
				ijma=ija(ma);
				if (ijma == j) sum += sa(ma)*sb(j);
				else {
					while (mb < ijb(j+1)) {
						ijmb=ijb(mb);
						if (ijmb == i) {
							sum += sa(i)*sb(mb++);
							continue;
						} else if (ijmb < ijma) {
							mb++;
							continue;
						} else if (ijmb == ijma) {
							sum += sa(ma)*sb(mb++);
							continue;
						}
						break;
					}
				}
			}
			for (mbb=mb;mbb<ijb(j+1);mbb++) {
				if (ijb(mbb) == i) sum += sa(i)*sb(mbb);
			}
			sc(m-1)=sum;
			sum=0.0;
			if (mn >= ijc(i+1)) break;
			j=ijc((m= ++mn)-1);
		}
	}
}





void NR_single_precision::sprstm(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &sb, NR_single_precision::Vec_I_INT &ijb,
	const NR_single_precision::DP thresh, NR_single_precision::Vec_O_DP &sc, NR_single_precision::Vec_O_INT &ijc)
{
	int i,ijma,ijmb,j,k,ma,mb,mbb;
	NR_single_precision::DP sum;

	if (ija(0) != ijb(0)) NR::nrerror("sprstm: sizes do not match");
	int nmax=sc.size();
	ijc(0)=k=ija(0);
	for (i=0;i<ija(0)-1;i++) {
		for (j=0;j<ijb(0)-1;j++) {
			if (i == j) sum=sa(i)*sb(j); else sum=0.0e0;
			mb=ijb(j);
			for (ma=ija(i);ma<ija(i+1);ma++) {
				ijma=ija(ma);
				if (ijma == j) sum += sa(ma)*sb(j);
				else {
					while (mb < ijb(j+1)) {
						ijmb=ijb(mb);
						if (ijmb == i) {
							sum += sa(i)*sb(mb++);
							continue;
						} else if (ijmb < ijma) {
							mb++;
							continue;
						} else if (ijmb == ijma) {
							sum += sa(ma)*sb(mb++);
							continue;
						}
						break;
					}
				}
			}
			for (mbb=mb;mbb<ijb(j+1);mbb++) {
				if (ijb(mbb) == i) sum += sa(i)*sb(mbb);
			}
			if (i == j) sc(i)=sum;
			else if (fabs(sum) > thresh) {
				if (k > nmax-1) NR::nrerror("sprstm: sc and ijc too small");
				sc(k)=sum;
				ijc(k++)=j;
			}
		}
		ijc(i+1)=k;
	}
}



void NR_single_precision::sprstp(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_O_DP &sb, NR_single_precision::Vec_O_INT &ijb)
{
	int j,jl,jm,jp,ju,k,m,n2,noff,inc,iv;
	NR_single_precision::DP v;

	n2=ija(0);
	for (j=0;j<n2-1;j++) sb(j)=sa(j);
	NR_single_precision::Vec_INT ija_vec((int *) &ija(n2),ija(n2-1)-ija(0));
	NR_single_precision::Vec_INT ijb_vec(&ijb(n2),ija(n2-1)-ija(0));
	indexx(ija_vec,ijb_vec);
	for (j=n2,k=0;j < ija(n2-1);j++,k++) {
		ijb(j)=ijb_vec(k);
	}
	jp=0;
	for (k=ija(0);k<ija(n2-1);k++) {
		m=ijb(k)+n2;
		sb(k)=sa(m);
		for (j=jp;j<ija(m)+1;j++)
			ijb(j)=k;
		jp=ija(m)+1;
		jl=0;
		ju=n2-1;
		while (ju-jl > 1) {
			jm=(ju+jl)/2;
			if (ija(jm) > m) ju=jm; else jl=jm;
		}
		ijb(k)=jl;
	}
	for (j=jp;j<n2;j++) ijb(j)=ija(n2-1);
	for (j=0;j<n2-1;j++) {
		jl=ijb(j+1)-ijb(j);
		noff=ijb(j);
		inc=1;
		do {
			inc *= 3;
			inc++;
		} while (inc <= jl);
		do {
			inc /= 3;
			for (k=noff+inc;k<noff+jl;k++) {
				iv=ijb(k);
				v=sb(k);
				m=k;
				while (ijb(m-inc) > iv) {
					ijb(m)=ijb(m-inc);
					sb(m)=sb(m-inc);
					m -= inc;
					if (m-noff+1 <= inc) break;
				}
				ijb(m)=iv;
				sb(m)=v;
			}
		} while (inc > 1);
	}
}



void NR_single_precision::sprstx(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &b)
{
	int i,j,k;

	int n=x.size();
	if (ija(0) != (n+1))
		NR::nrerror("mismatched vector and matrix in sprstx");
	for (i=0;i<n;i++) b(i)=sa(i)*x(i);
	for (i=0;i<n;i++) {
		for (k=ija(i);k<ija(i+1);k++) {
			j=ija(k);
			b(j) += sa(k)*x(i);
		}
	}
}





//NR_single_precision::Mat_DP *d_p;

void NR_single_precision::stifbs(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &xx, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	Msg::error("stifbs");
	/*
	const int KMAXX=7,IMAXX=KMAXX+1;
	const NR_single_precision::DP SAFE1=0.25,SAFE2=0.7,REDMAX=1.0e-5,REDMIN=0.7;
	const NR_single_precision::DP TINY=1.0e-30,SCALMX=0.1;
	bool exitflag=false;
	int i,iq,k,kk,km,reduct;
	static int first=1,kmax,kopt,nvold = -1;
	NR_single_precision::DP eps1,errmax,fact,h,red,scale,work,wrkmin,xest;
	static NR_single_precision::DP epsold = -1.0,xnew;
	static NR_single_precision::Vec_DP a(IMAXX);
	static NR_single_precision::Mat_DP alf(KMAXX,KMAXX);
	static int nseq_d(IMAXX)={2,6,10,14,22,34,50,70};
	NR_single_precision::Vec_INT nseq(nseq_d,IMAXX);

	int nv=y.size();
	d_p=new NR_single_precision::Mat_DP(nv,KMAXX);
	x_p=new NR_single_precision::Vec_DP(KMAXX);
	NR_single_precision::Vec_DP dfdx(nv),err(KMAXX),yerr(nv),ysav(nv),yseq(nv);
	NR_single_precision::Mat_DP dfdy(nv,nv);
	if (eps != epsold || nv != nvold) {
		hnext = xnew = -1.0e29;
		eps1=SAFE1*eps;
		a(0)=nseq(0)+1;
		for (k=0;k<KMAXX;k++) a(k+1)=a(k)+nseq(k+1);
		for (iq=1;iq<KMAXX;iq++) {
			for (k=0;k<iq;k++)
				alf(k,iq)=pow(eps1,(a(k+1)-a(iq+1))/
					((a(iq+1)-a(0)+1.0)*(2*k+3)));
		}
		epsold=eps;
		nvold=nv;
		a(0) += nv;
		for (k=0;k<KMAXX;k++) a(k+1)=a(k)+nseq(k+1);
		for (kopt=1;kopt<KMAXX-1;kopt++)
			if (a(kopt+1) > a(kopt)*alf(kopt-1,kopt)) break;
		kmax=kopt;
	}
	h=htry;
	for (i=0;i<nv;i++) ysav(i)=y(i);
	jacobn_s(xx,y,dfdx,dfdy);
	if (xx != xnew || h != hnext) {
		first=1;
		kopt=kmax;
	}
	reduct=0;
	for (;;) {
		for (k=0;k<=kmax;k++) {
			xnew=xx+h;
			if (xnew == xx) NR::nrerror("step size underflow in stifbs");
			simpr(ysav,dydx,dfdx,dfdy,xx,h,nseq(k),yseq,derivs);
			xest=SQR(h/nseq(k));
			pzextr(k,xest,yseq,y,yerr);
			if (k != 0) {
				errmax=TINY;
				for (i=0;i<nv;i++) errmax=MAX(errmax,fabs(yerr(i)/yscal(i)));
				errmax /= eps;
				km=k-1;
				err(km)=pow(errmax/SAFE1,1.0/(2*km+3));
			}
			if (k != 0 && (k >= kopt-1 || first)) {
				if (errmax < 1.0) {
					exitflag=true;
					break;
				}
				if (k == kmax || k == kopt+1) {
					red=SAFE2/err(km);
					break;
				}
				else if (k == kopt && alf(kopt-1,kopt) < err(km)) {
					red=1.0/err(km);
					break;
				}
				else if (kopt == kmax && alf(km,kmax-1) < err(km)) {
					red=alf(km,kmax-1)*SAFE2/err(km);
					break;
				}
				else if (alf(km,kopt) < err(km)) {
					red=alf(km,kopt-1)/err(km);
					break;
				}
			}
		}
		if (exitflag) break;
		red=MIN(red,REDMIN);
		red=MAX(red,REDMAX);
		h *= red;
		reduct=1;
	}
	xx=xnew;
	hdid=h;
	first=0;
	wrkmin=1.0e35;
	for (kk=0;kk<=km;kk++) {
		fact=MAX(err(kk),SCALMX);
		work=fact*a(kk+1);
		if (work < wrkmin) {
			scale=fact;
			wrkmin=work;
			kopt=kk+1;
		}
	}
	hnext=h/scale;
	if (kopt >= k && kopt != kmax && !reduct) {
		fact=MAX(scale/alf(kopt-1,kopt),SCALMX);
		if (a(kopt+1)*fact <= wrkmin) {
			hnext=h/fact;
			kopt++;
		}
	}
	delete d_p;
	delete x_p;*/
}





void NR_single_precision::stiff(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &x, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	const NR_single_precision::DP SAFETY=0.9,GROW=1.5,PGROW= -0.25,SHRNK=0.5;
	const NR_single_precision::DP PSHRNK=(-1.0/3.0),ERRCON=0.1296;
	const int MAXTRY=40;
	const NR_single_precision::DP GAM=1.0/2.0,A21=2.0,A31=48.0/25.0,A32=6.0/25.0,C21= -8.0,
		C31=372.0/25.0,C32=12.0/5.0,C41=(-112.0/125.0),
		C42=(-54.0/125.0),C43=(-2.0/5.0),B1=19.0/9.0,B2=1.0/2.0,
		B3=25.0/108.0,B4=125.0/108.0,E1=17.0/54.0,E2=7.0/36.0,E3=0.0,
		E4=125.0/108.0,C1X=1.0/2.0,C2X=(-3.0/2.0),C3X=(121.0/50.0),
		C4X=(29.0/250.0),A2X=1.0,A3X=3.0/5.0;
	int i,j,jtry;
	NR_single_precision::DP d,errmax,h,xsav;

	int n=y.size();
	NR_single_precision::Mat_DP a(n,n),dfdy(n,n);
	NR_single_precision::Vec_INT indx(n);
	NR_single_precision::Vec_DP dfdx(n),dysav(n),err(n),ysav(n),g1(n),g2(n),g3(n),g4(n);
	xsav=x;
	for (i=0;i<n;i++) {
		ysav(i)=y(i);
		dysav(i)=dydx(i);
	}
	jacobn_s(xsav,ysav,dfdx,dfdy);
	h=htry;
	for (jtry=0;jtry<MAXTRY;jtry++) {
		for (i=0;i<n;i++) {
			for (j=0;j<n;j++) a(i,j) = -dfdy(i,j);
			a(i,i) += 1.0/(GAM*h);
		}
		ludcmp(a,indx,d);
		for (i=0;i<n;i++)
			g1(i)=dysav(i)+h*C1X*dfdx(i);
		lubksb(a,indx,g1);
		for (i=0;i<n;i++)
			y(i)=ysav(i)+A21*g1(i);
		x=xsav+A2X*h;
		derivs(x,y,dydx);
		for (i=0;i<n;i++)
			g2(i)=dydx(i)+h*C2X*dfdx(i)+C21*g1(i)/h;
		lubksb(a,indx,g2);
		for (i=0;i<n;i++)
			y(i)=ysav(i)+A31*g1(i)+A32*g2(i);
		x=xsav+A3X*h;
		derivs(x,y,dydx);
		for (i=0;i<n;i++)
			g3(i)=dydx(i)+h*C3X*dfdx(i)+(C31*g1(i)+C32*g2(i))/h;
		lubksb(a,indx,g3);
		for (i=0;i<n;i++)
			g4(i)=dydx(i)+h*C4X*dfdx(i)+(C41*g1(i)+C42*g2(i)+C43*g3(i))/h;
		lubksb(a,indx,g4);
		for (i=0;i<n;i++) {
			y(i)=ysav(i)+B1*g1(i)+B2*g2(i)+B3*g3(i)+B4*g4(i);
			err(i)=E1*g1(i)+E2*g2(i)+E3*g3(i)+E4*g4(i);
		}
		x=xsav+h;
		if (x == xsav) NR::nrerror("stepsize not significant in stiff");
		errmax=0.0;
		for (i=0;i<n;i++) errmax=MAX(errmax,fabs(err(i)/yscal(i)));
		errmax /= eps;
		if (errmax <= 1.0) {
			hdid=h;
			hnext=(errmax > ERRCON ? SAFETY*h*pow(errmax,PGROW) : GROW*h);
			return;
		} else {
			hnext=SAFETY*h*pow(errmax,PSHRNK);
			h=(h >= 0.0 ? MAX(hnext,SHRNK*h) : MIN(hnext,SHRNK*h));
		}
	}
	NR::nrerror("exceeded MAXTRY in stiff");
}



void NR_single_precision::stoerm(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &d2y, const NR_single_precision::DP xs,
	const NR_single_precision::DP htot, const int nstep, NR_single_precision::Vec_O_DP &yout,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &))
{
	int i,nn,n,neqns;
	NR_single_precision::DP h,h2,halfh,x;

	int nv=y.size();
	NR_single_precision::Vec_DP ytemp(nv);
	h=htot/nstep;
	halfh=0.5*h;
	neqns=nv/2;
	for (i=0;i<neqns;i++) {
		n=neqns+i;
		ytemp(i)=y(i)+(ytemp(n)=h*(y(n)+halfh*d2y(i)));
	}
	x=xs+h;
	derivs(x,ytemp,yout);
	h2=h*h;
	for (nn=1;nn<nstep;nn++) {
		for (i=0;i<neqns;i++)
			ytemp(i) += (ytemp(neqns+i) += h2*yout(i));
		x += h;
		derivs(x,ytemp,yout);
	}
	for (i=0;i<neqns;i++) {
		n=neqns+i;
		yout(n)=ytemp(n)/h+halfh*yout(i);
		yout(i)=ytemp(i);
	}
}



void NR_single_precision::svbksb(NR_single_precision::Mat_I_DP &u, NR_single_precision::Vec_I_DP &w, NR_single_precision::Mat_I_DP &v, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_O_DP &x)
{
	int jj,j,i;
	NR_single_precision::DP s;

	int m=u.rows();
	int n=u.cols();
	NR_single_precision::Vec_DP tmp(n);
	for (j=0;j<n;j++) {
		s=0.0;
		if (w(j) != 0.0) {
			for (i=0;i<m;i++) s += u(i,j)*b(i);
			s /= w(j);
		}
		tmp(j)=s;
	}
	for (j=0;j<n;j++) {
		s=0.0;
		for (jj=0;jj<n;jj++) s += v(j,jj)*tmp(jj);
		x(j)=s;
	}
}





void NR_single_precision::svdcmp(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &w, NR_single_precision::Mat_O_DP &v)
{
	bool flag;
	int i,its,j,jj,k,l,nm;
	NR_single_precision::DP anorm,c,f,g,h,s,scale,x,y,z;

	int m=a.rows();
	int n=a.cols();
	NR_single_precision::Vec_DP rv1(n);
	g=scale=anorm=0.0;
	for (i=0;i<n;i++) {
		l=i+2;
		rv1(i)=scale*g;
		g=s=scale=0.0;
		if (i < m) {
			for (k=i;k<m;k++) scale += fabs(a(k,i));
			if (scale != 0.0) {
				for (k=i;k<m;k++) {
					a(k,i) /= scale;
					s += a(k,i)*a(k,i);
				}
				f=a(i,i);
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a(i,i)=f-g;
				for (j=l-1;j<n;j++) {
					for (s=0.0,k=i;k<m;k++) s += a(k,i)*a(k,j);
					f=s/h;
					for (k=i;k<m;k++) a(k,j) += f*a(k,i);
				}
				for (k=i;k<m;k++) a(k,i) *= scale;
			}
		}
		w(i)=scale *g;
		g=s=scale=0.0;
		if (i+1 <= m && i != n) {
			for (k=l-1;k<n;k++) scale += fabs(a(i,k));
			if (scale != 0.0) {
				for (k=l-1;k<n;k++) {
					a(i,k) /= scale;
					s += a(i,k)*a(i,k);
				}
				f=a(i,l-1);
				g = -SIGN(sqrt(s),f);
				h=f*g-s;
				a(i,l-1)=f-g;
				for (k=l-1;k<n;k++) rv1(k)=a(i,k)/h;
				for (j=l-1;j<m;j++) {
					for (s=0.0,k=l-1;k<n;k++) s += a(j,k)*a(i,k);
					for (k=l-1;k<n;k++) a(j,k) += s*rv1(k);
				}
				for (k=l-1;k<n;k++) a(i,k) *= scale;
			}
		}
		anorm=MAX(anorm,(fabs(w(i))+fabs(rv1(i))));
	}
	for (i=n-1;i>=0;i--) {
		if (i < n-1) {
			if (g != 0.0) {
				for (j=l;j<n;j++)
					v(j,i)=(a(i,j)/a(i,l))/g;
				for (j=l;j<n;j++) {
					for (s=0.0,k=l;k<n;k++) s += a(i,k)*v(k,j);
					for (k=l;k<n;k++) v(k,j) += s*v(k,i);
				}
			}
			for (j=l;j<n;j++) v(i,j)=v(j,i)=0.0;
		}
		v(i,i)=1.0;
		g=rv1(i);
		l=i;
	}
	for (i=MIN(m,n)-1;i>=0;i--) {
		l=i+1;
		g=w(i);
		for (j=l;j<n;j++) a(i,j)=0.0;
		if (g != 0.0) {
			g=1.0/g;
			for (j=l;j<n;j++) {
				for (s=0.0,k=l;k<m;k++) s += a(k,i)*a(k,j);
				f=(s/a(i,i))*g;
				for (k=i;k<m;k++) a(k,j) += f*a(k,i);
			}
			for (j=i;j<m;j++) a(j,i) *= g;
		} else for (j=i;j<m;j++) a(j,i)=0.0;
		++a(i,i);
	}
	for (k=n-1;k>=0;k--) {
		for (its=0;its<30;its++) {
			flag=true;
			for (l=k;l>=0;l--) {
				nm=l-1;
				if (fabs(rv1(l))+anorm == anorm) {
					flag=false;
					break;
				}
				if (fabs(w(nm))+anorm == anorm) break;
			}
			if (flag) {
				c=0.0;
				s=1.0;
				for (i=l-1;i<k+1;i++) {
					f=s*rv1(i);
					rv1(i)=c*rv1(i);
					if (fabs(f)+anorm == anorm) break;
					g=w(i);
					h=pythag(f,g);
					w(i)=h;
					h=1.0/h;
					c=g*h;
					s = -f*h;
					for (j=0;j<m;j++) {
						y=a(j,nm);
						z=a(j,i);
						a(j,nm)=y*c+z*s;
						a(j,i)=z*c-y*s;
					}
				}
			}
			z=w(k);
			if (l == k) {
				if (z < 0.0) {
					w(k) = -z;
					for (j=0;j<n;j++) v(j,k) = -v(j,k);
				}
				break;
			}
			if (its == 29) NR::nrerror("no convergence in 30 svdcmp iterations");
			x=w(l);
			nm=k-1;
			y=w(nm);
			g=rv1(nm);
			h=rv1(k);
			f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
			g=pythag(f,1.0);
			f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
			c=s=1.0;
			for (j=l;j<=nm;j++) {
				i=j+1;
				g=rv1(i);
				y=w(i);
				h=s*g;
				g=c*g;
				z=pythag(f,h);
				rv1(j)=z;
				c=f/z;
				s=h/z;
				f=x*c+g*s;
				g=g*c-x*s;
				h=y*s;
				y *= c;
				for (jj=0;jj<n;jj++) {
					x=v(jj,j);
					z=v(jj,i);
					v(jj,j)=x*c+z*s;
					v(jj,i)=z*c-x*s;
				}
				z=pythag(f,h);
				w(j)=z;
				if (z) {
					z=1.0/z;
					c=f*z;
					s=h*z;
				}
				f=c*g+s*y;
				x=c*y-s*g;
				for (jj=0;jj<m;jj++) {
					y=a(jj,j);
					z=a(jj,i);
					a(jj,j)=y*c+z*s;
					a(jj,i)=z*c-y*s;
				}
			}
			rv1(l)=0.0;
			rv1(k)=f;
			w(k)=x;
		}
	}
}



void NR_single_precision::svdfit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_O_DP &a,
	NR_single_precision::Mat_O_DP &u, NR_single_precision::Mat_O_DP &v, NR_single_precision::Vec_O_DP &w, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_O_DP &))
{
	int i,j;
	const NR_single_precision::DP TOL=1.0e-13;
	NR_single_precision::DP wmax,tmp,thresh,sum;

	int ndata=x.size();
	int ma=a.size();
	NR_single_precision::Vec_DP b(ndata),afunc(ma);
	for (i=0;i<ndata;i++) {
		funcs(x(i),afunc);
		tmp=1.0/sig(i);
		for (j=0;j<ma;j++) u(i,j)=afunc(j)*tmp;
		b(i)=y(i)*tmp;
	}
	svdcmp(u,w,v);
	wmax=0.0;
	for (j=0;j<ma;j++)
		if (w(j) > wmax) wmax=w(j);
	thresh=TOL*wmax;
	for (j=0;j<ma;j++)
		if (w(j) < thresh) w(j)=0.0;
	svbksb(u,w,v,b,a);
	chisq=0.0;
	for (i=0;i<ndata;i++) {
		funcs(x(i),afunc);
		sum=0.0;
		for (j=0;j<ma;j++) sum += a(j)*afunc(j);
		chisq += (tmp=(y(i)-sum)/sig(i),tmp*tmp);
	}
}



void NR_single_precision::svdvar(NR_single_precision::Mat_I_DP &v, NR_single_precision::Vec_I_DP &w, NR_single_precision::Mat_O_DP &cvm)
{
	int i,j,k;
	NR_single_precision::DP sum;

	int ma=w.size();
	NR_single_precision::Vec_DP wti(ma);
	for (i=0;i<ma;i++) {
		wti(i)=0.0;
		if (w(i) != 0.0) wti(i)=1.0/(w(i)*w(i));
	}
	for (i=0;i<ma;i++) {
		for (j=0;j<i+1;j++) {
			sum=0.0;
			for (k=0;k<ma;k++)
				sum += v(i,k)*v(j,k)*wti(k);
			cvm(j,i)=cvm(i,j)=sum;
		}
	}
}



void NR_single_precision::toeplz(NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_I_DP &y)
{
	int j,k,m,m1,m2,n,n1;
	NR_single_precision::DP pp,pt1,pt2,qq,qt1,qt2,sd,sgd,sgn,shn,sxn;

	n=y.size();
	n1=n-1;
	if (r(n1) == 0.0) NR::nrerror("toeplz-1 singular principal minor");
	x(0)=y(0)/r(n1);
	if (n1 == 0) return;
	NR_single_precision::Vec_DP g(n1),h(n1);
	g(0)=r(n1-1)/r(n1);
	h(0)=r(n1+1)/r(n1);
	for (m=0;m<n;m++) {
		m1=m+1;
		sxn = -y(m1);
		sd = -r(n1);
		for (j=0;j<m+1;j++) {
			sxn += r(n1+m1-j)*x(j);
			sd += r(n1+m1-j)*g(m-j);
		}
		if (sd == 0.0) NR::nrerror("toeplz-2 singular principal minor");
		x(m1)=sxn/sd;
		for (j=0;j<m+1;j++)
			x(j) -= x(m1)*g(m-j);
		if (m1 == n1) return;
		sgn = -r(n1-m1-1);
		shn = -r(n1+m1+1);
		sgd = -r(n1);
		for (j=0;j<m+1;j++) {
			sgn += r(n1+j-m1)*g(j);
			shn += r(n1+m1-j)*h(j);
			sgd += r(n1+j-m1)*h(m-j);
		}
		if (sgd == 0.0) NR::nrerror("toeplz-3 singular principal minor");
		g(m1)=sgn/sgd;
		h(m1)=shn/sd;
		k=m;
		m2=(m+2) >> 1;
		pp=g(m1);
		qq=h(m1);
		for (j=0;j<m2;j++) {
			pt1=g(j);
			pt2=g(k);
			qt1=h(j);
			qt2=h(k);
			g(j)=pt1-pp*qt2;
			g(k)=pt2-pp*qt1;
			h(j)=qt1-qq*pt2;
			h(k--)=qt2-qq*pt1;
		}
	}
	NR::nrerror("toeplz - should not arrive here!");
}





void NR_single_precision::tptest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &t, NR_single_precision::DP &prob)
{
	int j;
	NR_single_precision::DP var1,var2,ave1,ave2,sd,df,cov=0.0;

	int n=data1.size();
	avevar(data1,ave1,var1);
	avevar(data2,ave2,var2);
	for (j=0;j<n;j++)
		cov += (data1(j)-ave1)*(data2(j)-ave2);
	cov /= df=n-1;
	sd=sqrt((var1+var2-2.0*cov)/n);
	t=(ave1-ave2)/sd;
	prob=betai(0.5*df,0.5,df/(df+t*t));
}





void NR_single_precision::tqli(NR_single_precision::Vec_IO_DP &d, NR_single_precision::Vec_IO_DP &e, NR_single_precision::Mat_IO_DP &z)
{
	int m,l,iter,i,k;
	NR_single_precision::DP s,r,p,g,f,dd,c,b;

	int n=d.size();
	for (i=1;i<n;i++) e(i-1)=e(i);
	e(n-1)=0.0;
	for (l=0;l<n;l++) {
		iter=0;
		do {
			for (m=l;m<n-1;m++) {
				dd=fabs(d(m))+fabs(d(m+1));
				if (fabs(e(m))+dd == dd) break;
			}
			if (m != l) {
				if (iter++ == 30) NR::nrerror("Too many iterations in tqli");
				g=(d(l+1)-d(l))/(2.0*e(l));
				r=pythag(g,1.0);
				g=d(m)-d(l)+e(l)/(g+SIGN(r,g));
				s=c=1.0;
				p=0.0;
				for (i=m-1;i>=l;i--) {
					f=s*e(i);
					b=c*e(i);
					e(i+1)=(r=pythag(f,g));
					if (r == 0.0) {
						d(i+1) -= p;
						e(m)=0.0;
						break;
					}
					s=f/r;
					c=g/r;
					g=d(i+1)-p;
					r=(d(i)-g)*s+2.0*c*b;
					d(i+1)=g+(p=s*r);
					g=c*r-b;
					// Next loop can be omitted if eigenvectors not wanted
					for (k=0;k<n;k++) {
						f=z(k,i+1);
						z(k,i+1)=s*z(k,i)+c*f;
						z(k,i)=c*z(k,i)-s*f;
					}
				}
				if (r == 0.0 && i >= l) continue;
				d(l) -= p;
				e(l)=g;
				e(m)=0.0;
			}
		} while (m != l);
	}
}



NR_single_precision::DP NR_single_precision::trapzd(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const int n)
{
	NR_single_precision::DP x,tnm,sum,del;
	static NR_single_precision::DP s;
	int it,j;

	if (n == 1) {
		return (s=0.5*(b-a)*(func(a)+func(b)));
	} else {
		for (it=1,j=1;j<n-1;j++) it <<= 1;
		tnm=it;
		del=(b-a)/tnm;
		x=a+0.5*del;
		for (sum=0.0,j=0;j<it;j++,x+=del) sum += func(x);
		s=0.5*(s+(b-a)*sum/tnm);
		return s;
	}
}





void NR_single_precision::tred2(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &d, NR_single_precision::Vec_O_DP &e)
{
	int l,k,j,i;
	NR_single_precision::DP scale,hh,h,g,f;

	int n=d.size();
	for (i=n-1;i>0;i--) {
		l=i-1;
		h=scale=0.0;
		if (l > 0) {
			for (k=0;k<l+1;k++)
				scale += fabs(a(i,k));
			if (scale == 0.0)
				e(i)=a(i,l);
			else {
				for (k=0;k<l+1;k++) {
					a(i,k) /= scale;
					h += a(i,k)*a(i,k);
				}
				f=a(i,l);
				g=(f >= 0.0 ? -sqrt(h) : sqrt(h));
				e(i)=scale*g;
				h -= f*g;
				a(i,l)=f-g;
				f=0.0;
				for (j=0;j<l+1;j++) {
				// Next statement can be omitted if eigenvectors not wanted
					a(j,i)=a(i,j)/h;
					g=0.0;
					for (k=0;k<j+1;k++)
						g += a(j,k)*a(i,k);
					for (k=j+1;k<l+1;k++)
						g += a(k,j)*a(i,k);
					e(j)=g/h;
					f += e(j)*a(i,j);
				}
				hh=f/(h+h);
				for (j=0;j<l+1;j++) {
					f=a(i,j);
					e(j)=g=e(j)-hh*f;
					for (k=0;k<j+1;k++)
						a(j,k) -= (f*e(k)+g*a(i,k));
				}
			}
		} else
			e(i)=a(i,l);
		d(i)=h;
	}
	// Next statement can be omitted if eigenvectors not wanted
	d(0)=0.0;
	e(0)=0.0;
	// Contents of this loop can be omitted if eigenvectors not
	//	wanted except for statement d(i)=a(i,i);
	for (i=0;i<n;i++) {
		l=i;
		if (d(i) != 0.0) {
			for (j=0;j<l;j++) {
				g=0.0;
				for (k=0;k<l;k++)
					g += a(i,k)*a(k,j);
				for (k=0;k<l;k++)
					a(k,j) -= g*a(k,i);
			}
		}
		d(i)=a(i,i);
		a(i,i)=1.0;
		for (j=0;j<l;j++) a(j,i)=a(i,j)=0.0;
	}
}



void NR_single_precision::tridag(NR_single_precision::Vec_I_DP &a, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &u)
{
	int j;
	NR_single_precision::DP bet;

	int n=a.size();
	NR_single_precision::Vec_DP gam(n);
	if (b(0) == 0.0) NR::nrerror("Error 1 in tridag");
	u(0)=r(0)/(bet=b(0));
	for (j=1;j<n;j++) {
		gam(j)=c(j-1)/bet;
		bet=b(j)-a(j)*gam(j);
		if (bet == 0.0) NR::nrerror("Error 2 in tridag");
		u(j)=(r(j)-a(j)*u(j-1))/bet;
	}
	for (j=(n-2);j>=0;j--)
		u(j) -= gam(j+1)*u(j+1);
}






NR_single_precision::DP NR_single_precision::trncst(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_INT &iorder, NR_single_precision::Vec_IO_INT &n)
{
	int j,ii;
	NR_single_precision::DP de;
	NR_single_precision::Vec_DP xx(6),yy(6);

	int ncity=x.size();
	n(3)=(n(2)+1) % ncity;
	n(4)=(n(0)+ncity-1) % ncity;
	n(5)=(n(1)+1) % ncity;
	for (j=0;j<6;j++) {
		ii=iorder(n(j));
		xx(j)=x(ii);
		yy(j)=y(ii);
	}
	de = -alen(xx(1),xx(5),yy(1),yy(5));
	de -= alen(xx(0),xx(4),yy(0),yy(4));
	de -= alen(xx(2),xx(3),yy(2),yy(3));
	de += alen(xx(0),xx(2),yy(0),yy(2));
	de += alen(xx(1),xx(3),yy(1),yy(3));
	de += alen(xx(4),xx(5),yy(4),yy(5));
	return de;
}



void NR_single_precision::trnspt(NR_single_precision::Vec_IO_INT &iorder, NR_single_precision::Vec_I_INT &n)
{
	int m1,m2,m3,nn,j,jj;

	int ncity=iorder.size();
	NR_single_precision::Vec_INT jorder(ncity);
	m1=(n(1)-n(0)+ncity) % ncity;
	m2=(n(4)-n(3)+ncity) % ncity;
	m3=(n(2)-n(5)+ncity) % ncity;
	nn=0;
	for (j=0;j<=m1;j++) {
		jj=(j+n(0)) % ncity;
		jorder(nn++)=iorder(jj);
	}
	for (j=0;j<=m2;j++) {
		jj=(j+n(3)) % ncity;
		jorder(nn++)=iorder(jj);
	}
	for (j=0;j<=m3;j++) {
		jj=(j+n(5)) % ncity;
		jorder(nn++)=iorder(jj);
	}
	for (j=0;j<ncity;j++)
		iorder(j)=jorder(j);
}





void NR_single_precision::ttest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &t, NR_single_precision::DP &prob)
{
	NR_single_precision::DP var1,var2,svar,df,ave1,ave2;

	int n1=data1.size();
	int n2=data2.size();
	avevar(data1,ave1,var1);
	avevar(data2,ave2,var2);
	df=n1+n2-2;
	svar=((n1-1)*var1+(n2-1)*var2)/df;
	t=(ave1-ave2)/sqrt(svar*(1.0/n1+1.0/n2));
	prob=betai(0.5*df,0.5,df/(df+t*t));
}





void NR_single_precision::tutest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &t, NR_single_precision::DP &prob)
{
	NR_single_precision::DP var1,var2,df,ave1,ave2;

	int n1=data1.size();
	int n2=data2.size();
	avevar(data1,ave1,var1);
	avevar(data2,ave2,var2);
	t=(ave1-ave2)/sqrt(var1/n1+var2/n2);
	df=SQR(var1/n1+var2/n2)/(SQR(var1/n1)/(n1-1)+SQR(var2/n2)/(n2-1));
	prob=betai(0.5*df,0.5,df/(df+SQR(t)));
}



void NR_single_precision::twofft(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::Vec_O_DP &fft1,
	NR_single_precision::Vec_O_DP &fft2)
{
	int nn3,nn2,jj,j;
	NR_single_precision::DP rep,rem,aip,aim;

	int n=data1.size();
	nn3=1+(nn2=n+n);
	for (j=0,jj=0;j<n;j++,jj+=2) {
		fft1(jj)=data1(j);
		fft1(jj+1)=data2(j);
	}
	four1(fft1,1);
	fft2(0)=fft1(1);
	fft1(1)=fft2(1)=0.0;
	for (j=2;j<n+1;j+=2) {
		rep=0.5*(fft1(j)+fft1(nn2-j));
		rem=0.5*(fft1(j)-fft1(nn2-j));
		aip=0.5*(fft1(j+1)+fft1(nn3-j));
		aim=0.5*(fft1(j+1)-fft1(nn3-j));
		fft1(j)=rep;
		fft1(j+1)=aim;
		fft1(nn2-j)=rep;
		fft1(nn3-j)= -aim;
		fft2(j)=aip;
		fft2(j+1)= -rem;
		fft2(nn2-j)=aip;
		fft2(nn3-j)=rem;
	}
}



void NR_single_precision::vander(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &w, NR_single_precision::Vec_I_DP &q)
{
	int i,j,k;
	NR_single_precision::DP b,s,t,xx;

	int n=q.size();
	NR_single_precision::Vec_DP c(n);
	if (n == 1) w(0)=q(0);
	else {
		for (i=0;i<n;i++) c(i)=0.0;
		c(n-1) = -x(0);
		for (i=1;i<n;i++) {
			xx = -x(i);
			for (j=(n-1-i);j<(n-1);j++) c(j) += xx*c(j+1);
			c(n-1) += xx;
		}
		for (i=0;i<n;i++) {
			xx=x(i);
			t=b=1.0;
			s=q(n-1);
			for (k=n-1;k>0;k--) {
				b=c(k)+xx*b;
				s += q(k-1)*b;
				t=xx*t+b;
			}
			w(i)=s/t;
		}
	}
}

#include <iostream>
#include <iomanip>




extern int idum;

void NR_single_precision::vegas(NR_single_precision::Vec_I_DP &regn, NR_single_precision::DP fxn(NR_single_precision::Vec_I_DP &, const NR_single_precision::DP), const int init,
	const int ncall, const int itmx, const int nprn, NR_single_precision::DP &tgral, NR_single_precision::DP &sd,
	NR_single_precision::DP &chi2a)
{
	const int NDMX=50, MXDIM=10;
	const NR_single_precision::DP ALPH=1.5, TINY=1.0e-30;
	static int i,it,j,k,mds,nd,ndo,ng,npg;
	static NR_single_precision::DP calls,dv2g,dxg,f,f2,f2b,fb,rc,ti;
	static NR_single_precision::DP tsi,wgt,xjac,xn,xnd,xo,schi,si,swgt;
	static NR_single_precision::Vec_INT ia(MXDIM),kg(MXDIM);
	static NR_single_precision::Vec_DP dt(MXDIM),dx(MXDIM),r(NDMX),x(MXDIM),xin(NDMX);
	static NR_single_precision::Mat_DP d(NDMX,MXDIM),di(NDMX,MXDIM),xi(MXDIM,NDMX);

	int ndim=regn.size()/2;
	if (init <= 0) {
		mds=ndo=1;
		for (j=0;j<ndim;j++) xi(j,0)=1.0;
	}
	if (init <= 1) si=swgt=schi=0.0;
	if (init <= 2) {
		nd=NDMX;
		ng=1;
		if (mds != 0) {
			ng=int(pow(ncall/2.0+0.25,1.0/ndim));
			mds=1;
			if ((2*ng-NDMX) >= 0) {
				mds = -1;
				npg=ng/NDMX+1;
				nd=ng/npg;
				ng=npg*nd;
			}
		}
		for (k=1,i=0;i<ndim;i++) k *= ng;
		npg=MAX(int(ncall/k),2);
		calls=NR_single_precision::DP(npg)*NR_single_precision::DP(k);
		dxg=1.0/ng;
		for (dv2g=1,i=0;i<ndim;i++) dv2g *= dxg;
		dv2g=SQR(calls*dv2g)/npg/npg/(npg-1.0);
		xnd=nd;
		dxg *= xnd;
		xjac=1.0/calls;
		for (j=0;j<ndim;j++) {
			dx(j)=regn(j+ndim)-regn(j);
			xjac *= dx(j);
		}
		if (nd != ndo) {
			for (i=0;i<MAX(nd,ndo);i++) r(i)=1.0;
			for (j=0;j<ndim;j++)
				rebin(ndo/xnd,nd,r,xin,xi,j);
			ndo=nd;
		}
		if (nprn >= 0) {
			cout << " Input parameters for vegas";
			cout << "  ndim= " << ndim;
			cout << "  ncall= " << calls << endl;
			cout << "  it=" << it;
			cout << "  itmx=" << itmx << endl;
			cout << "  nprn=" << nprn;
			cout << "  ALPH=" << ALPH << endl;
			cout << "  mds=" << mds;
			cout << "  nd=" << nd << endl;
			for (j=0;j<ndim;j++) {
				cout << " x1(" << j;
				cout << ")= " << regn(j) << " xu(";
				cout << j << ")= ";
				cout << regn(j+ndim) << endl;
			}
		}
	}
	for (it=0;it<itmx;it++) {
		ti=tsi=0.0;
		for (j=0;j<ndim;j++) {
			kg(j)=1;
			for (i=0;i<nd;i++) d(i,j)=di(i,j)=0.0;
		}
		for (;;) {
			fb=f2b=0.0;
			for (k=0;k<npg;k++) {
				wgt=xjac;
				for (j=0;j<ndim;j++) {
					xn=(kg(j)-ran2(idum))*dxg+1.0;
					ia(j)=MAX(MIN(int(xn),NDMX),1);
					if (ia(j) > 1) {
						xo=xi(j,ia(j)-1)-xi(j,ia(j)-2);
						rc=xi(j,ia(j)-2)+(xn-ia(j))*xo;
					} else {
						xo=xi(j,ia(j)-1);
						rc=(xn-ia(j))*xo;
					}
					x(j)=regn(j)+rc*dx(j);
					wgt *= xo*xnd;
				}
				f=wgt*fxn(x,wgt);
				f2=f*f;
				fb += f;
				f2b += f2;
				for (j=0;j<ndim;j++) {
					di(ia(j)-1,j) += f;
					if (mds >= 0) d(ia(j)-1,j) += f2;
				}
			}
			f2b=sqrt(f2b*npg);
			f2b=(f2b-fb)*(f2b+fb);
			if (f2b <= 0.0) f2b=TINY;
			ti += fb;
			tsi += f2b;
			if (mds < 0) {
				for (j=0;j<ndim;j++) d(ia(j)-1,j) += f2b;
			}
			for (k=ndim-1;k>=0;k--) {
				kg(k) %= ng;
				if (++kg(k) != 1) break;
			}
			if (k < 0) break;
		}
		tsi *= dv2g;
		wgt=1.0/tsi;
		si += wgt*ti;
		schi += wgt*ti*ti;
		swgt += wgt;
		tgral=si/swgt;
		chi2a=(schi-si*tgral)/(it+0.0001);
		if (chi2a < 0.0) chi2a = 0.0;
		sd=sqrt(1.0/swgt);
		tsi=sqrt(tsi);
		if (nprn >= 0) {
			cout << " iteration no. " << (it+1);
			cout << " : integral = " << ti;
			cout << " +/- " << tsi << endl;
			cout << " all iterations:  " << " integral =";
			cout << tgral << "+-" << sd;
			cout << " chi**2/IT n =" << chi2a << endl;
			if (nprn != 0) {
				for (j=0;j<ndim;j++) {
					cout << " DATA FOR axis  " << j << endl;
					cout << "     X      delta i          X      delta i";
					cout << "          X       deltai" << endl;
					for (i=nprn/2;i<nd;i += nprn+2) {
						cout << xi(j,i) << di(i,j);
						cout << xi(j,i+1) << di(i+1,j);
						cout << xi(j,i+2) << di(i+2,j);
						cout << endl;
					}
				}
			}
		}
		for (j=0;j<ndim;j++) {
			xo=d(0,j);
			xn=d(1,j);
			d(0,j)=(xo+xn)/2.0;
			dt(j)=d(0,j);
			for (i=2;i<nd;i++) {
				rc=xo+xn;
				xo=xn;
				xn=d(i,j);
				d(i-1,j) = (rc+xn)/3.0;
				dt(j) += d(i-1,j);
			}
			d(nd-1,j)=(xo+xn)/2.0;
			dt(j) += d(nd-1,j);
		}
		for (j=0;j<ndim;j++) {
			rc=0.0;
			for (i=0;i<nd;i++) {
				if (d(i,j) < TINY) d(i,j)=TINY;
				r(i)=pow((1.0-d(i,j)/dt(j))/
					(log(dt(j))-log(d(i,j))),ALPH);
				rc += r(i);
			}
			rebin(rc/xnd,nd,r,xin,xi,j);
		}
	}
}



void NR_single_precision::voltra(const NR_single_precision::DP t0, const NR_single_precision::DP h, NR_single_precision::Vec_O_DP &t, NR_single_precision::Mat_O_DP &f,
	NR_single_precision::DP g(const int, const NR_single_precision::DP),
	NR_single_precision::DP ak(const int, const int, const NR_single_precision::DP, const NR_single_precision::DP))
{
	int i,j,k,l;
	NR_single_precision::DP d,sum;

	int m=f.rows();
	int n=f.cols();
	NR_single_precision::Vec_INT indx(m);
	NR_single_precision::Vec_DP b(m);
	NR_single_precision::Mat_DP a(m,m);
	t(0)=t0;
	for (k=0;k<m;k++) f(k,0)=g(k,t(0));
	for (i=1;i<n;i++) {
		t(i)=t(i-1)+h;
		for (k=0;k<m;k++) {
			sum=g(k,t(i));
			for (l=0;l<m;l++) {
				sum += 0.5*h*ak(k,l,t(i),t(0))*f(l,0);
				for (j=1;j<i;j++)
					sum += h*ak(k,l,t(i),t(j))*f(l,j);
				if (k == l)
					a(k,l)=1.0-0.5*h*ak(k,l,t(i),t(i));
				else
					a(k,l) = -0.5*h*ak(k,l,t(i),t(i));
			}
			b(k)=sum;
		}
		ludcmp(a,indx,d);
		lubksb(a,indx,b);
		for (k=0;k<m;k++) f(k,i)=b(k);
	}
}



void NR_single_precision::wt1(NR_single_precision::Vec_IO_DP &a, const int isign,
	void wtstep(NR_single_precision::Vec_IO_DP &, const int, const int))
{
	int nn;

	int n=a.size();
	if (n < 4) return;
	if (isign >= 0) {
		for (nn=n;nn>=4;nn>>=1) wtstep(a,nn,isign);
	} else {
		for (nn=4;nn<=n;nn<<=1) wtstep(a,nn,isign);
	}
}



void NR_single_precision::wtn(NR_single_precision::Vec_IO_DP &a, NR_single_precision::Vec_I_INT &nn, const int isign,
	void wtstep(NR_single_precision::Vec_IO_DP &, const int, const int))
{
	int idim,i1,i2,i3,k,n,nnew,nprev=1,nt,ntot=1;

	int ndim=nn.size();
	for (idim=0;idim<ndim;idim++) ntot *= nn(idim);
	for (idim=0;idim<ndim;idim++) {
		n=nn(idim);
		NR_single_precision::Vec_DP wksp(n);
		nnew=n*nprev;
		if (n > 4) {
			for (i2=0;i2<ntot;i2+=nnew) {
				for (i1=0;i1<nprev;i1++) {
					for (i3=i1+i2,k=0;k<n;k++,i3+=nprev) wksp(k)=a(i3);
					if (isign >= 0) {
						for(nt=n;nt>=4;nt >>= 1)
							wtstep(wksp,nt,isign);
					} else {
						for(nt=4;nt<=n;nt <<= 1)
							wtstep(wksp,nt,isign);
					}
					for (i3=i1+i2,k=0;k<n;k++,i3+=nprev) a(i3)=wksp(k);
				}
			}
		}
		nprev=nnew;
	}
}



void NR_single_precision::wwghts(NR_single_precision::Vec_O_DP &wghts, const NR_single_precision::DP h,
	void kermom(NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP y))
{
	int j,k;
	NR_single_precision::Vec_DP wold(4);
	NR_single_precision::DP hh,hi,c,fac,a,b;

	int n=wghts.size();
	hh=h;
	hi=1.0/hh;
	for (j=0;j<n;j++) wghts(j)=0.0;
	if (n >= 4) {
		NR_single_precision::Vec_DP wold(4),wnew(4),w(4);
		kermom(wold,0.0);
		b=0.0;
		for (j=0;j<n-3;j++) {
			c=j;
			a=b;
			b=a+hh;
			if (j == n-4) b=(n-1)*hh;
			kermom(wnew,b);
			for (fac=1.0,k=0;k<4;k++,fac*=hi)
				w(k)=(wnew(k)-wold(k))*fac;
			wghts(j) += (((c+1.0)*(c+2.0)*(c+3.0)*w(0)
				-(11.0+c*(12.0+c*3.0))*w(1)+3.0*(c+2.0)*w(2)-w(3))/6.0);
			wghts(j+1) += ((-c*(c+2.0)*(c+3.0)*w(0)
				+(6.0+c*(10.0+c*3.0))*w(1)-(3.0*c+5.0)*w(2)+w(3))*0.5);
			wghts(j+2) += ((c*(c+1.0)*(c+3.0)*w(0)
				-(3.0+c*(8.0+c*3.0))*w(1)+(3.0*c+4.0)*w(2)-w(3))*0.5);
			wghts(j+3) += ((-c*(c+1.0)*(c+2.0)*w(0)
				+(2.0+c*(6.0+c*3.0))*w(1)-3.0*(c+1.0)*w(2)+w(3))/6.0);
			for (k=0;k<4;k++) wold(k)=wnew(k);
		}
	} else if (n == 3) {
		NR_single_precision::Vec_DP wold(3),wnew(3),w(3);
		kermom(wold,0.0);
		kermom(wnew,hh+hh);
		w(0)=wnew(0)-wold(0);
		w(1)=hi*(wnew(1)-wold(1));
		w(2)=hi*hi*(wnew(2)-wold(2));
		wghts(0)=w(0)-1.5*w(1)+0.5*w(2);
		wghts(1)=2.0*w(1)-w(2);
		wghts(2)=0.5*(w(2)-w(1));
	} else if (n == 2) {
		NR_single_precision::Vec_DP wold(2),wnew(2),w(2);
		kermom(wold,0.0);
		kermom(wnew,hh);
		wghts(0)=wnew(0)-wold(0)-(wghts(1)=hi*(wnew(1)-wold(1)));
	}
}





bool NR_single_precision::zbrac(NR_single_precision::DP func(const NR_single_precision::DP), NR_single_precision::DP &x1, NR_single_precision::DP &x2)
{
	const int NTRY=50;
	const NR_single_precision::DP FACTOR=1.6;
	int j;
	NR_single_precision::DP f1,f2;

	if (x1 == x2) NR::nrerror("Bad initial range in zbrac");
	f1=func(x1);
	f2=func(x2);
	for (j=0;j<NTRY;j++) {
		if (f1*f2 < 0.0) return true;
		if (fabs(f1) < fabs(f2))
			f1=func(x1 += FACTOR*(x1-x2));
		else
			f2=func(x2 += FACTOR*(x2-x1));
	}
	return false;
}



void NR_single_precision::zbrak(NR_single_precision::DP fx(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const int n,
	NR_single_precision::Vec_O_DP &xb1, NR_single_precision::Vec_O_DP &xb2, int &nroot)
{
	int i;
	NR_single_precision::DP x,fp,fc,dx;

	int nb=xb1.size();
	nroot=0;
	dx=(x2-x1)/n;
	fp=fx(x=x1);
	for (i=0;i<n;i++) {
		fc=fx(x += dx);
		if (fc*fp <= 0.0) {
			xb1(nroot)=x-dx;
			xb2(nroot++)=x;
			if(nroot == nb) return;
		}
		fp=fc;
	}
}


#include <limits>



NR_single_precision::DP NR_single_precision::zbrent(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP tol)
{
	const int ITMAX=100;
	const NR_single_precision::DP EPS=DBL_EPSILON;
	int iter;
	NR_single_precision::DP a=x1,b=x2,c=x2,d,e,min1,min2;
	NR_single_precision::DP fa=func(a),fb=func(b),fc,p,q,r,s,tol1,xm;

	if ((fa > 0.0 && fb > 0.0) || (fa < 0.0 && fb < 0.0))
		NR::nrerror("Root must be bracketed in zbrent");
	fc=fb;
	for (iter=0;iter<ITMAX;iter++) {
		if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0)) {
			c=a;
			fc=fa;
			e=d=b-a;
		}
		if (fabs(fc) < fabs(fb)) {
			a=b;
			b=c;
			c=a;
			fa=fb;
			fb=fc;
			fc=fa;
		}
		tol1=2.0*EPS*fabs(b)+0.5*tol;
		xm=0.5*(c-b);
		if (fabs(xm) <= tol1 || fb == 0.0) return b;
		if (fabs(e) >= tol1 && fabs(fa) > fabs(fb)) {
			s=fb/fa;
			if (a == c) {
				p=2.0*xm*s;
				q=1.0-s;
			} else {
				q=fa/fc;
				r=fb/fc;
				p=s*(2.0*xm*q*(q-r)-(b-a)*(r-1.0));
				q=(q-1.0)*(r-1.0)*(s-1.0);
			}
			if (p > 0.0) q = -q;
			p=fabs(p);
			min1=3.0*xm*q-fabs(tol1*q);
			min2=fabs(e*q);
			if (2.0*p < (min1 < min2 ? min1 : min2)) {
				e=d;
				d=p/q;
			} else {
				d=xm;
				e=d;
			}
		} else {
			d=xm;
			e=d;
		}
		a=b;
		fa=fb;
		if (fabs(d) > tol1)
			b += d;
		else
			b += SIGN(tol1,xm);
			fb=func(b);
	}
	NR::nrerror("Maximum number of iterations exceeded in zbrent");
	return 0.0;
}

#include <complex>



void NR_single_precision::zrhqr(NR_single_precision::Vec_I_DP &a, NR_single_precision::Vec_O_CPLX_DP &rt)
{
	int j,k;
	complex<NR_single_precision::DP> x;

	int m=a.size()-1;
	NR_single_precision::Mat_DP hess(m,m);
	for (k=0;k<m;k++) {
		hess(0,k) = -a(m-k-1)/a(m);
		for (j=1;j<m;j++) hess(j,k)=0.0;
		if (k != m-1) hess(k+1,k)=1.0;
	}
	balanc(hess);
	hqr(hess,rt);
	for (j=1;j<m;j++) {
		x=rt(j);
		for (k=j-1;k>=0;k--) {
			if (real(rt(k)) <= real(x)) break;
			rt(k+1)=rt(k);
		}
		rt(k+1)=x;
	}
}





NR_single_precision::DP NR_single_precision::zriddr(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc)
{
	const int MAXIT=60;
	const NR_single_precision::DP UNUSED=-1.11e30;
	int j;
	NR_single_precision::DP ans,fh,fl,fm,fnew,s,xh,xl,xm,xnew;

	fl=func(x1);
	fh=func(x2);
	if ((fl > 0.0 && fh < 0.0) || (fl < 0.0 && fh > 0.0)) {
		xl=x1;
		xh=x2;
		ans=UNUSED;
		for (j=0;j<MAXIT;j++) {
			xm=0.5*(xl+xh);
			fm=func(xm);
			s=sqrt(fm*fm-fl*fh);
			if (s == 0.0) return ans;
			xnew=xm+(xm-xl)*((fl >= fh ? 1.0 : -1.0)*fm/s);
			if (fabs(xnew-ans) <= xacc) return ans;
			ans=xnew;
			fnew=func(ans);
			if (fnew == 0.0) return ans;
			if (SIGN(fm,fnew) != fm) {
				xl=xm;
				fl=fm;
				xh=ans;
				fh=fnew;
			} else if (SIGN(fl,fnew) != fl) {
				xh=ans;
				fh=fnew;
			} else if (SIGN(fh,fnew) != fh) {
				xl=ans;
				fl=fnew;
			} else NR::nrerror("never get here.");
			if (fabs(xh-xl) <= xacc) return ans;
		}
		NR::nrerror("zriddr exceed maximum iterations");
	}
	else {
		if (fl == 0.0) return x1;
		if (fh == 0.0) return x2;
		NR::nrerror("root must be bracketed in zriddr.");
	}
	return 0.0;
}


#include <complex>



void NR_single_precision::zroots(NR_single_precision::Vec_I_CPLX_DP &a, NR_single_precision::Vec_O_CPLX_DP &roots, const bool &polish)
{
	const NR_single_precision::DP EPS=1.0e-14;
	int i,its,j,jj;
	complex<NR_single_precision::DP> x,b,c;

	int m=a.size()-1;
	Vec_CPLX_DP ad(m+1);
	for (j=0;j<=m;j++) ad(j)=a(j);
	for (j=m-1;j>=0;j--) {
		x=0.0;
		Vec_CPLX_DP ad_v(j+2);
		for (jj=0;jj<j+2;jj++) ad_v(jj)=ad(jj);
		laguer(ad_v,x,its);
		if (fabs(imag(x)) <= 2.0*EPS*fabs(real(x)))
			x=complex<NR_single_precision::DP>(real(x),0.0);
		roots(j)=x;
		b=ad(j+1);
		for (jj=j;jj>=0;jj--) {
			c=ad(jj);
			ad(jj)=b;
			b=x*b+c;
		}
	}
	if (polish)
		for (j=0;j<m;j++)
			laguer(a,roots(j),its);
	for (j=1;j<m;j++) {
		x=roots(j);
		for (i=j-1;i>=0;i--) {
			if (real(roots(i)) <= real(x)) break;
			roots(i+1)=roots(i);
		}
		roots(i+1)=x;
	}
}
}