#ifndef _NR_SINGLE_PRECISION_H_
#define _NR_SINGLE_PRECISION_H_
#include <fstream>
#include <complex>
#include "NR_single_precision_util.h"
#include "NR_single_precision_types.h"
#ifdef EPS
#undef EPS
#endif
using namespace std;

namespace NR_single_precision {

void addint(NR_single_precision::Mat_O_DP &uf, NR_single_precision::Mat_I_DP &uc, NR_single_precision::Mat_O_DP &res);
void airy(const NR_single_precision::DP x, NR_single_precision::DP &ai, NR_single_precision::DP &bi, NR_single_precision::DP &aip, NR_single_precision::DP &bip);
void amebsa(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_O_DP &pb, NR_single_precision::DP &yb, const NR_single_precision::DP ftol,
	NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &), int &iter, const NR_single_precision::DP temptr);
void amoeba(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_IO_DP &y, const NR_single_precision::DP ftol, NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &),
	int &nfunk);
NR_single_precision::DP amotry(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_O_DP &y, NR_single_precision::Vec_IO_DP &psum, NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &),
	const int ihi, const NR_single_precision::DP fac);
NR_single_precision::DP amotsa(NR_single_precision::Mat_IO_DP &p, NR_single_precision::Vec_O_DP &y, NR_single_precision::Vec_IO_DP &psum, NR_single_precision::Vec_O_DP &pb, NR_single_precision::DP &yb,
	NR_single_precision::DP funk(NR_single_precision::Vec_I_DP &), const int ihi, NR_single_precision::DP &yhi, const NR_single_precision::DP fac);
void anneal(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_IO_INT &iorder);
NR_single_precision::DP anorm2(NR_single_precision::Mat_I_DP &a);
void arcmak(NR_single_precision::Vec_I_ULNG &nfreq, unsigned long nchh, unsigned long nradd,
	arithcode &acode);
void arcode(unsigned long &ich, string &code, unsigned long &lcd,
	const int isign, arithcode &acode);
void arcsum(NR_single_precision::Vec_I_ULNG &iin, NR_single_precision::Vec_O_ULNG &iout, unsigned long ja,
	const int nwk, const unsigned long nrad, const unsigned long nc);
void asolve(NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_O_DP &x, const int itrnsp);
void atimes(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &r, const int itrnsp);
void avevar(NR_single_precision::Vec_I_DP &data, NR_single_precision::DP &ave, NR_single_precision::DP &var);
void balanc(NR_single_precision::Mat_IO_DP &a);
void banbks(NR_single_precision::Mat_I_DP &a, const int m1, const int m2, NR_single_precision::Mat_I_DP &al,
	NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_IO_DP &b);
void bandec(NR_single_precision::Mat_IO_DP &a, const int m1, const int m2, NR_single_precision::Mat_O_DP &al,
	NR_single_precision::Vec_O_INT &indx, NR_single_precision::DP &d);
void banmul(NR_single_precision::Mat_I_DP &a, const int m1, const int m2, NR_single_precision::Vec_I_DP &x,
	NR_single_precision::Vec_O_DP &b);
void bcucof(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &y1, NR_single_precision::Vec_I_DP &y2, NR_single_precision::Vec_I_DP &y12,
	const NR_single_precision::DP d1, const NR_single_precision::DP d2, NR_single_precision::Mat_O_DP &c);
void bcuint(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &y1, NR_single_precision::Vec_I_DP &y2, NR_single_precision::Vec_I_DP &y12,
	const NR_single_precision::DP x1l, const NR_single_precision::DP x1u, const NR_single_precision::DP x2l, const NR_single_precision::DP x2u,
	const NR_single_precision::DP x1, const NR_single_precision::DP x2, NR_single_precision::DP &ansy, NR_single_precision::DP &ansy1, NR_single_precision::DP &ansy2);
void beschb(const NR_single_precision::DP x, NR_single_precision::DP &gam1, NR_single_precision::DP &gam2, NR_single_precision::DP &gampl, NR_single_precision::DP &gammi);
NR_single_precision::DP bessi(const int n, const NR_single_precision::DP x);
NR_single_precision::DP bessi0(const NR_single_precision::DP x);
NR_single_precision::DP bessi1(const NR_single_precision::DP x);
void bessik(const NR_single_precision::DP x, const NR_single_precision::DP xnu, NR_single_precision::DP &ri, NR_single_precision::DP &rk, NR_single_precision::DP &rip, NR_single_precision::DP &rkp);
NR_single_precision::DP bessj(const int n, const NR_single_precision::DP x);
NR_single_precision::DP bessj0(const NR_single_precision::DP x);
NR_single_precision::DP bessj1(const NR_single_precision::DP x);
void bessjy(const NR_single_precision::DP x, const NR_single_precision::DP xnu, NR_single_precision::DP &rj, NR_single_precision::DP &ry, NR_single_precision::DP &rjp, NR_single_precision::DP &ryp);
NR_single_precision::DP bessk(const int n, const NR_single_precision::DP x);
NR_single_precision::DP bessk0(const NR_single_precision::DP x);
NR_single_precision::DP bessk1(const NR_single_precision::DP x);
NR_single_precision::DP bessy(const int n, const NR_single_precision::DP x);
NR_single_precision::DP bessy0(const NR_single_precision::DP x);
NR_single_precision::DP bessy1(const NR_single_precision::DP x);
NR_single_precision::DP beta(const NR_single_precision::DP z, const NR_single_precision::DP w);
NR_single_precision::DP betacf(const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP x);
NR_single_precision::DP betai(const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP x);
NR_single_precision::DP bico(const int n, const int k);
void bksub(const int ne, const int nb, const int jf, const int k1,
	const int k2, Mat3D_IO_DP &c);
NR_single_precision::DP bnldev(const NR_single_precision::DP pp, const int n, int &idum);
NR_single_precision::DP brent(const NR_single_precision::DP ax, const NR_single_precision::DP bx, const NR_single_precision::DP cx, NR_single_precision::DP f(const NR_single_precision::DP),
	const NR_single_precision::DP tol, NR_single_precision::DP &xmin);
void broydn(NR_single_precision::Vec_IO_DP &x, bool &check, void vecfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void bsstep(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &xx, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void caldat(const int julian, int &mm, int &id, int &iyyy);
void chder(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_O_DP &cder, const int n);
NR_single_precision::DP chebev(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &c, const int m, const NR_single_precision::DP x);
void chebft(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_O_DP &c, NR_single_precision::DP func(const NR_single_precision::DP));
void chebpc(NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_O_DP &d);
void chint(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_O_DP &cint, const int n);
NR_single_precision::DP chixy(const NR_single_precision::DP bang);
void choldc(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &p);
void cholsl(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_DP &p, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_O_DP &x);
void chsone(NR_single_precision::Vec_I_DP &bins, NR_single_precision::Vec_I_DP &ebins, const int knstrn, NR_single_precision::DP &df,
	NR_single_precision::DP &chsq, NR_single_precision::DP &prob);
void chstwo(NR_single_precision::Vec_I_DP &bins1, NR_single_precision::Vec_I_DP &bins2, const int knstrn, NR_single_precision::DP &df,
	NR_single_precision::DP &chsq, NR_single_precision::DP &prob);
void cisi(const NR_single_precision::DP x, complex<NR_single_precision::DP> &cs);
void cntab1(NR_single_precision::Mat_I_INT &nn, NR_single_precision::DP &chisq, NR_single_precision::DP &df, NR_single_precision::DP &prob, NR_single_precision::DP &cramrv, NR_single_precision::DP &ccc);
void cntab2(NR_single_precision::Mat_I_INT &nn, NR_single_precision::DP &h, NR_single_precision::DP &hx, NR_single_precision::DP &hy, NR_single_precision::DP &hygx, NR_single_precision::DP &hxgy,
	NR_single_precision::DP &uygx, NR_single_precision::DP &uxgy, NR_single_precision::DP &uxy);
void convlv(NR_single_precision::Vec_I_DP &data, NR_single_precision::Vec_I_DP &respns, const int isign,
	NR_single_precision::Vec_O_DP &ans);
void copy(NR_single_precision::Mat_O_DP &aout, NR_single_precision::Mat_I_DP &ain);
void correl(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::Vec_O_DP &ans);
void cosft1(NR_single_precision::Vec_IO_DP &y);
void cosft2(NR_single_precision::Vec_IO_DP &y, const int isign);
void covsrt(NR_single_precision::Mat_IO_DP &covar, NR_single_precision::Vec_I_BOOL &ia, const int mfit);
void crank(NR_single_precision::Vec_IO_DP &w, NR_single_precision::DP &s);
void cyclic(NR_single_precision::Vec_I_DP &a, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_I_DP &c, const NR_single_precision::DP alpha,
	const NR_single_precision::DP beta, NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &x);
void daub4(NR_single_precision::Vec_IO_DP &a, const int n, const int isign);
NR_single_precision::DP dawson(const NR_single_precision::DP x);
NR_single_precision::DP dbrent(const NR_single_precision::DP ax, const NR_single_precision::DP bx, const NR_single_precision::DP cx, NR_single_precision::DP f(const NR_single_precision::DP),
	NR_single_precision::DP df(const NR_single_precision::DP), const NR_single_precision::DP tol, NR_single_precision::DP &xmin);
void ddpoly(NR_single_precision::Vec_I_DP &c, const NR_single_precision::DP x, NR_single_precision::Vec_O_DP &pd);
bool decchk(string str, char &ch);
void derivs_s(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dydx);
NR_single_precision::DP df1dim(const NR_single_precision::DP x);
void dfpmin(NR_single_precision::Vec_IO_DP &p, const NR_single_precision::DP gtol, int &iter, NR_single_precision::DP &fret,
	NR_single_precision::DP func(NR_single_precision::Vec_I_DP &), void dfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
NR_single_precision::DP dfridr(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x, const NR_single_precision::DP h, NR_single_precision::DP &err);
void dftcor(const NR_single_precision::DP w, const NR_single_precision::DP delta, const NR_single_precision::DP a, const NR_single_precision::DP b,
	NR_single_precision::Vec_I_DP &endpts, NR_single_precision::DP &corre, NR_single_precision::DP &corim, NR_single_precision::DP &corfac);
void dftint(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const NR_single_precision::DP w,
	NR_single_precision::DP &cosint, NR_single_precision::DP &sinint);
void difeq(const int k, const int k1, const int k2, const int jsf,
	const int is1, const int isf, NR_single_precision::Vec_I_INT &indexv, NR_single_precision::Mat_O_DP &s,
	NR_single_precision::Mat_I_DP &y);
void dlinmin(NR_single_precision::Vec_IO_DP &p, NR_single_precision::Vec_IO_DP &xi, NR_single_precision::DP &fret, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &),
	void dfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void eclass(NR_single_precision::Vec_O_INT &nf, NR_single_precision::Vec_I_INT &lista, NR_single_precision::Vec_I_INT &listb);
void eclazz(NR_single_precision::Vec_O_INT &nf, bool equiv(const int, const int));
NR_single_precision::DP ei(const NR_single_precision::DP x);
void eigsrt(NR_single_precision::Vec_IO_DP &d, NR_single_precision::Mat_IO_DP &v);
NR_single_precision::DP elle(const NR_single_precision::DP phi, const NR_single_precision::DP ak);
NR_single_precision::DP ellf(const NR_single_precision::DP phi, const NR_single_precision::DP ak);
NR_single_precision::DP ellpi(const NR_single_precision::DP phi, const NR_single_precision::DP en, const NR_single_precision::DP ak);
void elmhes(NR_single_precision::Mat_IO_DP &a);
NR_single_precision::DP erfcc(const NR_single_precision::DP x);
NR_single_precision::DP erff(const NR_single_precision::DP x);
NR_single_precision::DP erffc(const NR_single_precision::DP x);
void eulsum(NR_single_precision::DP &sum, const NR_single_precision::DP term, const int jterm, NR_single_precision::Vec_IO_DP &wksp);
NR_single_precision::DP evlmem(const NR_single_precision::DP fdt, NR_single_precision::Vec_I_DP &d, const NR_single_precision::DP xms);
NR_single_precision::DP expdev(int &idum);
NR_single_precision::DP expint(const int n, const NR_single_precision::DP x);
NR_single_precision::DP f1dim(const NR_single_precision::DP x);
NR_single_precision::DP factln(const int n);
NR_single_precision::DP factrl(const int n);
void fasper(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, const NR_single_precision::DP ofac, const NR_single_precision::DP hifac,
	NR_single_precision::Vec_O_DP &wk1, NR_single_precision::Vec_O_DP &wk2, int &nout, int &jmax, NR_single_precision::DP &prob);
void fdjac(NR_single_precision::Vec_IO_DP &x, NR_single_precision::Vec_I_DP &fvec, NR_single_precision::Mat_O_DP &df,
	void vecfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void fgauss(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &a, NR_single_precision::DP &y, NR_single_precision::Vec_O_DP &dyda);
void fit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, const bool mwt, NR_single_precision::DP &a,
	NR_single_precision::DP &b, NR_single_precision::DP &siga, NR_single_precision::DP &sigb, NR_single_precision::DP &chi2, NR_single_precision::DP &q);
void fitexy(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sigx, NR_single_precision::Vec_I_DP &sigy,
	NR_single_precision::DP &a, NR_single_precision::DP &b, NR_single_precision::DP &siga, NR_single_precision::DP &sigb, NR_single_precision::DP &chi2, NR_single_precision::DP &q);
void fixrts(NR_single_precision::Vec_IO_DP &d);
void fleg(const NR_single_precision::DP x, NR_single_precision::Vec_O_DP &pl);
void flmoon(const int n, const int nph, int &jd, NR_single_precision::DP &frac);
NR_single_precision::DP fmin(NR_single_precision::Vec_I_DP &x);
void four1(NR_single_precision::Vec_IO_DP &data, const int isign);
void fourew(NR_single_precision::Vec_FSTREAM_p &file, int &na, int &nb, int &nc, int &nd);
void fourfs(NR_single_precision::Vec_FSTREAM_p &file, NR_single_precision::Vec_I_INT &nn, const int isign);
void fourn(NR_single_precision::Vec_IO_DP &data, NR_single_precision::Vec_I_INT &nn, const int isign);
void fpoly(const NR_single_precision::DP x, NR_single_precision::Vec_O_DP &p);
void fred2(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_O_DP &t, NR_single_precision::Vec_O_DP &f, NR_single_precision::Vec_O_DP &w,
	NR_single_precision::DP g(const NR_single_precision::DP), NR_single_precision::DP ak(const NR_single_precision::DP, const NR_single_precision::DP));
NR_single_precision::DP fredin(const NR_single_precision::DP x, const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_I_DP &t, NR_single_precision::Vec_I_DP &f,
	NR_single_precision::Vec_I_DP &w, NR_single_precision::DP g(const NR_single_precision::DP), NR_single_precision::DP ak(const NR_single_precision::DP, const NR_single_precision::DP));
void frenel(const NR_single_precision::DP x, complex<NR_single_precision::DP> &cs);
void frprmn(NR_single_precision::Vec_IO_DP &p, const NR_single_precision::DP ftol, int &iter, NR_single_precision::DP &fret,
	NR_single_precision::DP func(NR_single_precision::Vec_I_DP &), void dfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void ftest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &f, NR_single_precision::DP &prob);
NR_single_precision::DP gamdev(const int ia, int &idum);
NR_single_precision::DP gammln(const NR_single_precision::DP xx);
NR_single_precision::DP gammp(const NR_single_precision::DP a, const NR_single_precision::DP x);
NR_single_precision::DP gammq(const NR_single_precision::DP a, const NR_single_precision::DP x);
NR_single_precision::DP gasdev(int &idum);
void gaucof(NR_single_precision::Vec_IO_DP &a, NR_single_precision::Vec_IO_DP &b, const NR_single_precision::DP amu0, NR_single_precision::Vec_O_DP &x,
	NR_single_precision::Vec_O_DP &w);
void gauher(NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w);
void gaujac(NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP alf, const NR_single_precision::DP bet);
void gaulag(NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP alf);
void gauleg(const NR_single_precision::DP x1, const NR_single_precision::DP x2, NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_O_DP &w);
void gaussj(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Mat_IO_DP &b);
void gcf(NR_single_precision::DP &gammcf, const NR_single_precision::DP a, const NR_single_precision::DP x, NR_single_precision::DP &gln);
NR_single_precision::DP golden(const NR_single_precision::DP ax, const NR_single_precision::DP bx, const NR_single_precision::DP cx, NR_single_precision::DP f(const NR_single_precision::DP),
	const NR_single_precision::DP tol, NR_single_precision::DP &xmin);
void gser(NR_single_precision::DP &gamser, const NR_single_precision::DP a, const NR_single_precision::DP x, NR_single_precision::DP &gln);
void hpsel(NR_single_precision::Vec_I_DP &arr, NR_single_precision::Vec_O_DP &heap);
void hpsort(NR_single_precision::Vec_IO_DP &ra);
void hqr(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_CPLX_DP &wri);
void hufapp(NR_single_precision::Vec_IO_ULNG &index, NR_single_precision::Vec_I_ULNG &nprob, const unsigned long n,
	const unsigned long m);
void hufdec(unsigned long &ich, string &code, const unsigned long lcode,
	unsigned long &nb, huffcode &hcode);
void hufenc(const unsigned long ich, string &code, unsigned long &nb,
	huffcode &hcode);
void hufmak(NR_single_precision::Vec_I_ULNG &nfreq, const unsigned long nchin,
	unsigned long &ilong, unsigned long &nlong, huffcode &hcode);
void hunt(NR_single_precision::Vec_I_DP &xx, const NR_single_precision::DP x, int &jlo);
void hypdrv(const NR_single_precision::DP s, NR_single_precision::Vec_I_DP &yy, NR_single_precision::Vec_O_DP &dyyds);
complex<NR_single_precision::DP> hypgeo(const complex<NR_single_precision::DP> &a, const complex<NR_single_precision::DP> &b,
	const complex<NR_single_precision::DP> &c, const complex<NR_single_precision::DP> &z);
void hypser(const complex<NR_single_precision::DP> &a, const complex<NR_single_precision::DP> &b,
	const complex<NR_single_precision::DP> &c, const complex<NR_single_precision::DP> &z,
	complex<NR_single_precision::DP> &series, complex<NR_single_precision::DP> &deriv);
unsigned short icrc(const unsigned short crc, const string &bufptr,
	const short jinit, const int jrev);
unsigned short icrc1(const unsigned short crc, const unsigned char onech);
unsigned long igray(const unsigned long n, const int is);
void indexx(NR_single_precision::Vec_I_DP &arr, NR_single_precision::Vec_O_INT &indx);
void indexx(NR_single_precision::Vec_I_INT &arr, NR_single_precision::Vec_O_INT &indx);
void interp(NR_single_precision::Mat_O_DP &uf, NR_single_precision::Mat_I_DP &uc);
int irbit1(unsigned long &iseed);
int irbit2(unsigned long &iseed);
void jacobi(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &d, NR_single_precision::Mat_O_DP &v, int &nrot);
void jacobn_s(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &dfdx, NR_single_precision::Mat_O_DP &dfdy);
int julday(const int mm, const int id, const int iyyy);
void kendl1(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &tau, NR_single_precision::DP &z, NR_single_precision::DP &prob);
void kendl2(NR_single_precision::Mat_I_DP &tab, NR_single_precision::DP &tau, NR_single_precision::DP &z, NR_single_precision::DP &prob);
void kermom(NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP y);
void ks2d1s(NR_single_precision::Vec_I_DP &x1, NR_single_precision::Vec_I_DP &y1, void quadvl(const NR_single_precision::DP, const NR_single_precision::DP,
	NR_single_precision::DP &, NR_single_precision::DP &, NR_single_precision::DP &, NR_single_precision::DP &), NR_single_precision::DP &d1, NR_single_precision::DP &prob);
void ks2d2s(NR_single_precision::Vec_I_DP &x1, NR_single_precision::Vec_I_DP &y1, NR_single_precision::Vec_I_DP &x2, NR_single_precision::Vec_I_DP &y2, NR_single_precision::DP &d,
	NR_single_precision::DP &prob);
void ksone(NR_single_precision::Vec_IO_DP &data, NR_single_precision::DP func(const NR_single_precision::DP), NR_single_precision::DP &d, NR_single_precision::DP &prob);
void kstwo(NR_single_precision::Vec_IO_DP &data1, NR_single_precision::Vec_IO_DP &data2, NR_single_precision::DP &d, NR_single_precision::DP &prob);
void laguer(NR_single_precision::Vec_I_CPLX_DP &a, complex<NR_single_precision::DP> &x, int &its);
void lfit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_IO_DP &a,
	NR_single_precision::Vec_I_BOOL &ia, NR_single_precision::Mat_O_DP &covar, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_O_DP &));
void linbcg(NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_IO_DP &x, const int itol, const NR_single_precision::DP tol,
	const int itmax, int &iter, NR_single_precision::DP &err);
void linmin(NR_single_precision::Vec_IO_DP &p, NR_single_precision::Vec_IO_DP &xi, NR_single_precision::DP &fret, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &));
void lnsrch(NR_single_precision::Vec_I_DP &xold, const NR_single_precision::DP fold, NR_single_precision::Vec_I_DP &g, NR_single_precision::Vec_IO_DP &p,
	NR_single_precision::Vec_O_DP &x, NR_single_precision::DP &f, const NR_single_precision::DP stpmax, bool &check, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &));
void locate(NR_single_precision::Vec_I_DP &xx, const NR_single_precision::DP x, int &j);
void lop(NR_single_precision::Mat_O_DP &out, NR_single_precision::Mat_I_DP &u);
void lubksb(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_IO_DP &b);
void ludcmp(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_INT &indx, NR_single_precision::DP &d);
void machar(int &ibeta, int &it, int &irnd, int &ngrd, int &machep,
	int &negep, int &iexp, int &minexp, int &maxexp, NR_single_precision::DP &eps, NR_single_precision::DP &epsneg,
	NR_single_precision::DP &xmin, NR_single_precision::DP &xmax);
void matadd(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &b, NR_single_precision::Mat_O_DP &c);
void matsub(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &b, NR_single_precision::Mat_O_DP &c);
void medfit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::DP &a, NR_single_precision::DP &b, NR_single_precision::DP &abdev);
void memcof(NR_single_precision::Vec_I_DP &data, NR_single_precision::DP &xms, NR_single_precision::Vec_O_DP &d);
bool metrop(const NR_single_precision::DP de, const NR_single_precision::DP t);
void mgfas(NR_single_precision::Mat_IO_DP &u, const int maxcyc);
void mglin(NR_single_precision::Mat_IO_DP &u, const int ncycle);
NR_single_precision::DP midexp(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n);
NR_single_precision::DP midinf(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n);
NR_single_precision::DP midpnt(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const int n);
NR_single_precision::DP midsql(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n);
NR_single_precision::DP midsqu(NR_single_precision::DP funk(const NR_single_precision::DP), const NR_single_precision::DP aa, const NR_single_precision::DP bb, const int n);
void miser(NR_single_precision::DP func(NR_single_precision::Vec_I_DP &), NR_single_precision::Vec_I_DP &regn, const int npts,
	const NR_single_precision::DP dith, NR_single_precision::DP &ave, NR_single_precision::DP &var);
void mmid(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, const NR_single_precision::DP xs, const NR_single_precision::DP htot,
	const int nstep, NR_single_precision::Vec_O_DP &yout,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void mnbrak(NR_single_precision::DP &ax, NR_single_precision::DP &bx, NR_single_precision::DP &cx, NR_single_precision::DP &fa, NR_single_precision::DP &fb, NR_single_precision::DP &fc,
	NR_single_precision::DP func(const NR_single_precision::DP));

void mnewt(const int ntrial, NR_single_precision::Vec_IO_DP &x, const NR_single_precision::DP tolx, const NR_single_precision::DP tolf, void (*usrfun)(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &fvec, NR_single_precision::Mat_O_DP &fjac));
//void mnewt(const int ntrial, NR_single_precision::Vec_IO_DP &x, const NR_single_precision::DP tolx, const NR_single_precision::DP tolf);
void moment(NR_single_precision::Vec_I_DP &data, NR_single_precision::DP &ave, NR_single_precision::DP &adev, NR_single_precision::DP &sdev, NR_single_precision::DP &var, NR_single_precision::DP &skew,
	NR_single_precision::DP &curt);
void mp2dfr(NR_single_precision::Vec_IO_UCHR &a, string &s);
void mpadd(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mpdiv(NR_single_precision::Vec_O_UCHR &q, NR_single_precision::Vec_O_UCHR &r, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mpinv(NR_single_precision::Vec_O_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mplsh(NR_single_precision::Vec_IO_UCHR &u);
void mpmov(NR_single_precision::Vec_O_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mpmul(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mpneg(NR_single_precision::Vec_IO_UCHR &u);
void mppi(const int np);
void mprove(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &alud, NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_I_DP &b,
	NR_single_precision::Vec_IO_DP &x);
void mpsad(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, const int iv);
void mpsdv(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, const int iv, int &ir);
void mpsmu(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, const int iv);
void mpsqrt(NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_O_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mpsub(int &is, NR_single_precision::Vec_O_UCHR &w, NR_single_precision::Vec_I_UCHR &u, NR_single_precision::Vec_I_UCHR &v);
void mrqcof(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_I_DP &a,
	NR_single_precision::Vec_I_BOOL &ia, NR_single_precision::Mat_O_DP &alpha, NR_single_precision::Vec_O_DP &beta, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &,NR_single_precision::DP &, NR_single_precision::Vec_O_DP &));
void mrqmin(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_IO_DP &a,
	NR_single_precision::Vec_I_BOOL &ia, NR_single_precision::Mat_O_DP &covar, NR_single_precision::Mat_O_DP &alpha, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::DP &, NR_single_precision::Vec_O_DP &), NR_single_precision::DP &alamda);
void newt(NR_single_precision::Vec_IO_DP &x, bool &check, void vecfunc(NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void odeint(NR_single_precision::Vec_IO_DP &ystart, const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP eps,
	const NR_single_precision::DP h1, const NR_single_precision::DP hmin, int &nok, int &nbad,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &),
	void rkqs(NR_single_precision::Vec_IO_DP &, NR_single_precision::Vec_IO_DP &, NR_single_precision::DP &, const NR_single_precision::DP, const NR_single_precision::DP,
	NR_single_precision::Vec_I_DP &, NR_single_precision::DP &, NR_single_precision::DP &, void (*)(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &)));
void orthog(NR_single_precision::Vec_I_DP &anu, NR_single_precision::Vec_I_DP &alpha, NR_single_precision::Vec_I_DP &beta, NR_single_precision::Vec_O_DP &a,
	NR_single_precision::Vec_O_DP &b);
void pade(NR_single_precision::Vec_IO_DP &cof, NR_single_precision::DP &resid);
void pccheb(NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_O_DP &c);
void pcshft(const NR_single_precision::DP a, const NR_single_precision::DP b, NR_single_precision::Vec_IO_DP &d);
void pearsn(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::DP &r, NR_single_precision::DP &prob, NR_single_precision::DP &z);
void period(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, const NR_single_precision::DP ofac, const NR_single_precision::DP hifac,
	NR_single_precision::Vec_O_DP &px, NR_single_precision::Vec_O_DP &py, int &nout, int &jmax, NR_single_precision::DP &prob);
void piksr2(NR_single_precision::Vec_IO_DP &arr, NR_single_precision::Vec_IO_DP &brr);
void piksrt(NR_single_precision::Vec_IO_DP &arr);
void pinvs(const int ie1, const int ie2, const int je1, const int jsf,
	const int jc1, const int k, Mat3D_O_DP &c, NR_single_precision::Mat_IO_DP &s);
NR_single_precision::DP plgndr(const int l, const int m, const NR_single_precision::DP x);
NR_single_precision::DP poidev(const NR_single_precision::DP xm, int &idum);
void polcoe(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_O_DP &cof);
void polcof(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, NR_single_precision::Vec_O_DP &cof);
void poldiv(NR_single_precision::Vec_I_DP &u, NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &q, NR_single_precision::Vec_O_DP &r);
void polin2(NR_single_precision::Vec_I_DP &x1a, NR_single_precision::Vec_I_DP &x2a, NR_single_precision::Mat_I_DP &ya, const NR_single_precision::DP x1,
	const NR_single_precision::DP x2, NR_single_precision::DP &y, NR_single_precision::DP &dy);
void polint(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, const NR_single_precision::DP x, NR_single_precision::DP &y, NR_single_precision::DP &dy);
void powell(NR_single_precision::Vec_IO_DP &p, NR_single_precision::Mat_IO_DP &xi, const NR_single_precision::DP ftol, int &iter,
	NR_single_precision::DP &fret, NR_single_precision::DP func(NR_single_precision::Vec_I_DP &));
void predic(NR_single_precision::Vec_I_DP &data, NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_O_DP &future);
NR_single_precision::DP probks(const NR_single_precision::DP alam);
void psdes(unsigned long &lword, unsigned long &irword);
void pwt(NR_single_precision::Vec_IO_DP &a, const int n, const int isign);
void pwtset(const int n);
NR_single_precision::DP pythag(const NR_single_precision::DP a, const NR_single_precision::DP b);
void pzextr(const int iest, const NR_single_precision::DP xest, NR_single_precision::Vec_I_DP &yest, NR_single_precision::Vec_O_DP &yz,
	NR_single_precision::Vec_O_DP &dy);
NR_single_precision::DP qgaus(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b);
void qrdcmp(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &c, NR_single_precision::Vec_O_DP &d, bool &sing);
NR_single_precision::DP qromb(NR_single_precision::DP func(const NR_single_precision::DP), NR_single_precision::DP a, NR_single_precision::DP b);
NR_single_precision::DP qromo(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b,
	NR_single_precision::DP choose(NR_single_precision::DP (*)(const NR_single_precision::DP), const NR_single_precision::DP, const NR_single_precision::DP, const int));
void qroot(NR_single_precision::Vec_I_DP &p, NR_single_precision::DP &b, NR_single_precision::DP &c, const NR_single_precision::DP eps);
void qrsolv(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_IO_DP &b);
void qrupdt(NR_single_precision::Mat_IO_DP &r, NR_single_precision::Mat_IO_DP &qt, NR_single_precision::Vec_IO_DP &u, NR_single_precision::Vec_I_DP &v);
NR_single_precision::DP qsimp(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b);
NR_single_precision::DP qtrap(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b);
NR_single_precision::DP quad3d(NR_single_precision::DP func(const NR_single_precision::DP, const NR_single_precision::DP, const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2);
void quadct(const NR_single_precision::DP x, const NR_single_precision::DP y, NR_single_precision::Vec_I_DP &xx, NR_single_precision::Vec_I_DP &yy, NR_single_precision::DP &fa,
	NR_single_precision::DP &fb, NR_single_precision::DP &fc, NR_single_precision::DP &fd);
void quadmx(NR_single_precision::Mat_O_DP &a);
void quadvl(const NR_single_precision::DP x, const NR_single_precision::DP y, NR_single_precision::DP &fa, NR_single_precision::DP &fb, NR_single_precision::DP &fc, NR_single_precision::DP &fd);
NR_single_precision::DP ran0(int &idum);
NR_single_precision::DP ran1(int &idum);
NR_single_precision::DP ran2(int &idum);
NR_single_precision::DP ran3(int &idum);
NR_single_precision::DP ran4(int &idum);
void rank(NR_single_precision::Vec_I_INT &indx, NR_single_precision::Vec_O_INT &irank);
void ranpt(NR_single_precision::Vec_O_DP &pt, NR_single_precision::Vec_I_DP &regn);
void ratint(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, const NR_single_precision::DP x, NR_single_precision::DP &y, NR_single_precision::DP &dy);
void ratlsq(NR_single_precision::DP fn(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const int mm,
	const int kk, NR_single_precision::Vec_O_DP &cof, NR_single_precision::DP &dev);
NR_single_precision::DP ratval(const NR_single_precision::DP x, NR_single_precision::Vec_I_DP &cof, const int mm, const int kk);
NR_single_precision::DP rc(const NR_single_precision::DP x, const NR_single_precision::DP y);
NR_single_precision::DP rd(const NR_single_precision::DP x, const NR_single_precision::DP y, const NR_single_precision::DP z);
void realft(NR_single_precision::Vec_IO_DP &data, const int isign);
void rebin(const NR_single_precision::DP rc, const int nd, NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &xin,
	NR_single_precision::Mat_IO_DP &xi, const int j);
void red(const int iz1, const int iz2, const int jz1, const int jz2,
	const int jm1, const int jm2, const int jmf, const int ic1,
	const int jc1, const int jcf, const int kc, Mat3D_I_DP &c,
	NR_single_precision::Mat_IO_DP &s);
void relax(NR_single_precision::Mat_IO_DP &u, NR_single_precision::Mat_I_DP &rhs);
void relax2(NR_single_precision::Mat_IO_DP &u, NR_single_precision::Mat_I_DP &rhs);
void resid(NR_single_precision::Mat_O_DP &res, NR_single_precision::Mat_I_DP &u, NR_single_precision::Mat_I_DP &rhs);
NR_single_precision::DP revcst(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_INT &iorder, NR_single_precision::Vec_IO_INT &n);
void reverse(NR_single_precision::Vec_IO_INT &iorder, NR_single_precision::Vec_I_INT &n);
NR_single_precision::DP rf(const NR_single_precision::DP x, const NR_single_precision::DP y, const NR_single_precision::DP z);
NR_single_precision::DP rj(const NR_single_precision::DP x, const NR_single_precision::DP y, const NR_single_precision::DP z, const NR_single_precision::DP p);
void rk4(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, const NR_single_precision::DP x, const NR_single_precision::DP h,
	NR_single_precision::Vec_O_DP &yout, void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void rkck(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, const NR_single_precision::DP x,
	const NR_single_precision::DP h, NR_single_precision::Vec_O_DP &yout, NR_single_precision::Vec_O_DP &yerr,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void rkdumb(NR_single_precision::Vec_I_DP &vstart, const NR_single_precision::DP x1, const NR_single_precision::DP x2,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void rkqs(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &x, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void rlft3(Mat3D_IO_DP &data, NR_single_precision::Mat_IO_DP &speq, const int isign);
NR_single_precision::DP rofunc(const NR_single_precision::DP b);
void rotate(NR_single_precision::Mat_IO_DP &r, NR_single_precision::Mat_IO_DP &qt, const int i, const NR_single_precision::DP a,
	const NR_single_precision::DP b);
void rsolv(NR_single_precision::Mat_I_DP &a, NR_single_precision::Vec_I_DP &d, NR_single_precision::Vec_IO_DP &b);
void rstrct(NR_single_precision::Mat_O_DP &uc, NR_single_precision::Mat_I_DP &uf);
NR_single_precision::DP rtbis(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc);
NR_single_precision::DP rtflsp(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc);
NR_single_precision::DP rtnewt(void funcd(const NR_single_precision::DP, NR_single_precision::DP &, NR_single_precision::DP &), const NR_single_precision::DP x1, const NR_single_precision::DP x2,
	const NR_single_precision::DP xacc);
NR_single_precision::DP rtsafe(void funcd(const NR_single_precision::DP, NR_single_precision::DP &, NR_single_precision::DP &), const NR_single_precision::DP x1, const NR_single_precision::DP x2,
	const NR_single_precision::DP xacc);
NR_single_precision::DP rtsec(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc);
void rzextr(const int iest, const NR_single_precision::DP xest, NR_single_precision::Vec_I_DP &yest, NR_single_precision::Vec_O_DP &yz,
	NR_single_precision::Vec_O_DP &dy);
void savgol(NR_single_precision::Vec_O_DP &c, const int np, const int nl, const int nr,
	const int ld, const int m);
void scrsho(NR_single_precision::DP fx(const NR_single_precision::DP));
NR_single_precision::DP select(const int k, NR_single_precision::Vec_IO_DP &arr);
NR_single_precision::DP selip(const int k, NR_single_precision::Vec_I_DP &arr);
void shell(const int n, NR_single_precision::Vec_IO_DP &a);
void shoot(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f);
void shootf(NR_single_precision::Vec_I_DP &v, NR_single_precision::Vec_O_DP &f);
void simp1(NR_single_precision::Mat_I_DP &a, const int mm, NR_single_precision::Vec_I_INT &ll, const int nll,
	const int iabf, int &kp, NR_single_precision::DP &bmax);
void simp2(NR_single_precision::Mat_I_DP &a, const int m, const int n, int &ip, const int kp);
void simp3(NR_single_precision::Mat_IO_DP &a, const int i1, const int k1, const int ip,
	const int kp);
void simplx(NR_single_precision::Mat_IO_DP &a, const int m1, const int m2, const int m3,
	int &icase, NR_single_precision::Vec_O_INT &izrov, NR_single_precision::Vec_O_INT &iposv);
void simpr(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &dydx, NR_single_precision::Vec_I_DP &dfdx, NR_single_precision::Mat_I_DP &dfdy,
	const NR_single_precision::DP xs, const NR_single_precision::DP htot, const int nstep, NR_single_precision::Vec_O_DP &yout,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void sinft(NR_single_precision::Vec_IO_DP &y);
void slvsm2(NR_single_precision::Mat_O_DP &u, NR_single_precision::Mat_I_DP &rhs);
void slvsml(NR_single_precision::Mat_O_DP &u, NR_single_precision::Mat_I_DP &rhs);
void sncndn(const NR_single_precision::DP uu, const NR_single_precision::DP emmc, NR_single_precision::DP &sn, NR_single_precision::DP &cn, NR_single_precision::DP &dn);
NR_single_precision::DP snrm(NR_single_precision::Vec_I_DP &sx, const int itol);
void sobseq(const int n, NR_single_precision::Vec_O_DP &x);
void solvde(const int itmax, const NR_single_precision::DP conv, const NR_single_precision::DP slowc,
	NR_single_precision::Vec_I_DP &scalv, NR_single_precision::Vec_I_INT &indexv, const int nb, NR_single_precision::Mat_IO_DP &y);
void sor(NR_single_precision::Mat_I_DP &a, NR_single_precision::Mat_I_DP &b, NR_single_precision::Mat_I_DP &c, NR_single_precision::Mat_I_DP &d, NR_single_precision::Mat_I_DP &e,
	NR_single_precision::Mat_I_DP &f, NR_single_precision::Mat_IO_DP &u, const NR_single_precision::DP rjac);
void sort(NR_single_precision::Vec_IO_DP &arr);
void sort2(NR_single_precision::Vec_IO_DP &arr, NR_single_precision::Vec_IO_DP &brr);
void sort3(NR_single_precision::Vec_IO_DP &ra, NR_single_precision::Vec_IO_DP &rb, NR_single_precision::Vec_IO_DP &rc);
void spctrm(ifstream &fp, NR_single_precision::Vec_O_DP &p, const int k, const bool ovrlap);
void spear(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &d, NR_single_precision::DP &zd, NR_single_precision::DP &probd,
	NR_single_precision::DP &rs, NR_single_precision::DP &probrs);
void sphbes(const int n, const NR_single_precision::DP x, NR_single_precision::DP &sj, NR_single_precision::DP &sy, NR_single_precision::DP &sjp, NR_single_precision::DP &syp);
void splie2(NR_single_precision::Vec_I_DP &x1a, NR_single_precision::Vec_I_DP &x2a, NR_single_precision::Mat_I_DP &ya, NR_single_precision::Mat_O_DP &y2a);
void splin2(NR_single_precision::Vec_I_DP &x1a, NR_single_precision::Vec_I_DP &x2a, NR_single_precision::Mat_I_DP &ya, NR_single_precision::Mat_I_DP &y2a,
	const NR_single_precision::DP x1, const NR_single_precision::DP x2, NR_single_precision::DP &y);
void spline(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, const NR_single_precision::DP yp1, const NR_single_precision::DP ypn,
	NR_single_precision::Vec_O_DP &y2);
void splint(NR_single_precision::Vec_I_DP &xa, NR_single_precision::Vec_I_DP &ya, NR_single_precision::Vec_I_DP &y2a, const NR_single_precision::DP x, NR_single_precision::DP &y);
void spread(const NR_single_precision::DP y, NR_single_precision::Vec_IO_DP &yy, const NR_single_precision::DP x, const int m);
void sprsax(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &b);
void sprsin(NR_single_precision::Mat_I_DP &a, const NR_single_precision::DP thresh, NR_single_precision::Vec_O_DP &sa, NR_single_precision::Vec_O_INT &ija);
void sprspm(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &sb, NR_single_precision::Vec_I_INT &ijb,
	NR_single_precision::Vec_O_DP &sc, NR_single_precision::Vec_I_INT &ijc);
void sprstm(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &sb, NR_single_precision::Vec_I_INT &ijb,
	const NR_single_precision::DP thresh, NR_single_precision::Vec_O_DP &sc, NR_single_precision::Vec_O_INT &ijc);
void sprstp(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_O_DP &sb, NR_single_precision::Vec_O_INT &ijb);
void sprstx(NR_single_precision::Vec_I_DP &sa, NR_single_precision::Vec_I_INT &ija, NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &b);
void stifbs(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &xx, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void stiff(NR_single_precision::Vec_IO_DP &y, NR_single_precision::Vec_IO_DP &dydx, NR_single_precision::DP &x, const NR_single_precision::DP htry,
	const NR_single_precision::DP eps, NR_single_precision::Vec_I_DP &yscal, NR_single_precision::DP &hdid, NR_single_precision::DP &hnext,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void stoerm(NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &d2y, const NR_single_precision::DP xs,
	const NR_single_precision::DP htot, const int nstep, NR_single_precision::Vec_O_DP &yout,
	void derivs(const NR_single_precision::DP, NR_single_precision::Vec_I_DP &, NR_single_precision::Vec_O_DP &));
void svbksb(NR_single_precision::Mat_I_DP &u, NR_single_precision::Vec_I_DP &w, NR_single_precision::Mat_I_DP &v, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_O_DP &x);
void svdcmp(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &w, NR_single_precision::Mat_O_DP &v);
void svdfit(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_DP &sig, NR_single_precision::Vec_O_DP &a,
	NR_single_precision::Mat_O_DP &u, NR_single_precision::Mat_O_DP &v, NR_single_precision::Vec_O_DP &w, NR_single_precision::DP &chisq,
	void funcs(const NR_single_precision::DP, NR_single_precision::Vec_O_DP &));
void svdvar(NR_single_precision::Mat_I_DP &v, NR_single_precision::Vec_I_DP &w, NR_single_precision::Mat_O_DP &cvm);
void toeplz(NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &x, NR_single_precision::Vec_I_DP &y);
void tptest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &t, NR_single_precision::DP &prob);
void tqli(NR_single_precision::Vec_IO_DP &d, NR_single_precision::Vec_IO_DP &e, NR_single_precision::Mat_IO_DP &z);
NR_single_precision::DP trapzd(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP a, const NR_single_precision::DP b, const int n);
void tred2(NR_single_precision::Mat_IO_DP &a, NR_single_precision::Vec_O_DP &d, NR_single_precision::Vec_O_DP &e);
void tridag(NR_single_precision::Vec_I_DP &a, NR_single_precision::Vec_I_DP &b, NR_single_precision::Vec_I_DP &c, NR_single_precision::Vec_I_DP &r, NR_single_precision::Vec_O_DP &u);
NR_single_precision::DP trncst(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_I_DP &y, NR_single_precision::Vec_I_INT &iorder, NR_single_precision::Vec_IO_INT &n);
void trnspt(NR_single_precision::Vec_IO_INT &iorder, NR_single_precision::Vec_I_INT &n);
void ttest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &t, NR_single_precision::DP &prob);
void tutest(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::DP &t, NR_single_precision::DP &prob);
void twofft(NR_single_precision::Vec_I_DP &data1, NR_single_precision::Vec_I_DP &data2, NR_single_precision::Vec_O_DP &fft1,
	NR_single_precision::Vec_O_DP &fft2);
void vander(NR_single_precision::Vec_I_DP &x, NR_single_precision::Vec_O_DP &w, NR_single_precision::Vec_I_DP &q);
void vegas(NR_single_precision::Vec_I_DP &regn, NR_single_precision::DP fxn(NR_single_precision::Vec_I_DP &, const NR_single_precision::DP), const int init,
	const int ncall, const int itmx, const int nprn, NR_single_precision::DP &tgral, NR_single_precision::DP &sd,
	NR_single_precision::DP &chi2a);
void voltra(const NR_single_precision::DP t0, const NR_single_precision::DP h, NR_single_precision::Vec_O_DP &t, NR_single_precision::Mat_O_DP &f,
	NR_single_precision::DP g(const int, const NR_single_precision::DP),
	NR_single_precision::DP ak(const int, const int, const NR_single_precision::DP, const NR_single_precision::DP));
void wt1(NR_single_precision::Vec_IO_DP &a, const int isign,
	void wtstep(NR_single_precision::Vec_IO_DP &, const int, const int));
void wtn(NR_single_precision::Vec_IO_DP &a, NR_single_precision::Vec_I_INT &nn, const int isign,
	void wtstep(NR_single_precision::Vec_IO_DP &, const int, const int));
void wwghts(NR_single_precision::Vec_O_DP &wghts, const NR_single_precision::DP h,
	void kermom(NR_single_precision::Vec_O_DP &w, const NR_single_precision::DP y));
bool zbrac(NR_single_precision::DP func(const NR_single_precision::DP), NR_single_precision::DP &x1, NR_single_precision::DP &x2);
void zbrak(NR_single_precision::DP fx(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const int n,
	NR_single_precision::Vec_O_DP &xb1, NR_single_precision::Vec_O_DP &xb2, int &nroot);
NR_single_precision::DP zbrent(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP tol);
void zrhqr(NR_single_precision::Vec_I_DP &a, NR_single_precision::Vec_O_CPLX_DP &rt);
NR_single_precision::DP zriddr(NR_single_precision::DP func(const NR_single_precision::DP), const NR_single_precision::DP x1, const NR_single_precision::DP x2, const NR_single_precision::DP xacc);
void zroots(NR_single_precision::Vec_I_CPLX_DP &a, NR_single_precision::Vec_O_CPLX_DP &roots, const bool &polish);
}
#endif /* _NR_SINGLE_PRECISION_H_ */
