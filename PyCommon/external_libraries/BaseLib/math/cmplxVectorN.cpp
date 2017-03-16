#include "stdafx.h"
#include "mathclass.h"
#include "cmplxvectorn.h"
#include    <math.h>
#include    <stdlib.h>
#include    <stdio.h>

cmplxvectorn::cmplxvectorn(int size)
{
	n=on=0;
	setSize(size);
}

cmplxvectorn::cmplxvectorn()
{
	n=on=0;
}

cmplxvectorn::~cmplxvectorn()
{
	if(on>0) delete[] v;	
}

void cmplxvectorn::setSize( int x )
{
	if(n==x)
		return;

	if ( on<x )
	{
		ASSERT(on>=n);
		
		if ( on>0 ) delete[] v;
		v = new complex_old[x];
		on = x;
	}
	n = x;
}

void cmplxvectorn::resize(int nsize)
{
	if(nsize<=n)
		setSize(nsize);
	else if(n<=on)
	{
		int prev_size=n;
		setSize(nsize);
		//this->setValue(prev_size, nsize, 0, 0);
	}
	else
	{
		int capacity=MAX(on,10);
		// capacity가 nsize를 포함할때까지 doubling
		while(capacity<nsize)	capacity*=2;

		int prev_size=n;
		cmplxvectorn backup;
		backup.assign(*this);
		setSize(capacity);
		setSize(nsize);
		this->assign(backup, 0, prev_size);
		//this->setValue(prev_size, nsize, 0);
	}
}

void cmplxvectorn::assign(const cmplxvectorn& other)
{
    setSize(other.size());
	for( int i=0; i<n; i++ )
		v[i] = other[i];	
}

void cmplxvectorn::assign( const cmplxvectorn& value, int start, int end)
{
	ASSERT(value.size()==end-start);
	for(int i=start; i<end; i++)
	{
		(*this)[i]=value[i-start];
	}
}

/***********************************************************************

dft - Discrete Fourier Transform
  
This function performs a straight DFT of N points on an array of
complex_old numbers whose first member is pointed to by Datain.  The
output is placed in an array pointed to by Dataout.

void dft(COMPLEX *Datain, COMPLEX *Dataout, int N)

*************************************************************************/

void cmplxvectorn::dft(const cmplxvectorn& Datain)
{
	int N=Datain.size();
	cmplxvectorn& Dataout=*this;
    int i,k,n,p;
    static int nstore = 0;      /* store N for future use */
    static cmplxvectorn cf;     /* coefficient storage */
    
    double arg;

/* Create the coefficients if N has changed */

    if(N != nstore) {
		cf.setSize(N);

        arg = M_PI*2/N;
        for (i=0 ; i<N ; i++) {
            cf[i].real = cos(arg*i);
            cf[i].imag = -sin(arg*i);
        }
    }

/* Perform the DFT calculation */

    printf("\n");
	int inIndex=0;
	int outIndex=0;
    for (k=0 ; k<N ; k++) 
	{
		inIndex=0;
		complex_old& Din = Datain[inIndex];                          
        Dataout[outIndex].real = Din.real;   //if n =0 then cos=1,sin=0
        Dataout[outIndex].imag = Din.imag;   //don't exit imag number in datain
        inIndex++;

        for (n=1; n<N; n++) {

			p = (int)((long)n*k % N);
            complex_old& cfelt = cf[p];         /* pointer to cf modulo N */

            Dataout[outIndex].real += Din.real * cfelt.real
                             - Din.imag * cfelt.imag;

            Dataout[outIndex].imag += Din.real * cfelt.imag
                             + Din.imag * cfelt.real;
            inIndex++;
        }
        if (k % 32 == 31) printf("*");
        outIndex++;          /* next output */
    }
    printf("\n");
}

/***********************************************************************

idft - Inverse Discrete Fourier Transform
  
This function performs an inverse DFT of N points on an array of
complex_old numbers whose first member is pointed to by Datain.  The
output is placed in an array pointed to by Dataout.
It returns nothing.

void idft(COMPLEX *Datain, COMPLEX *Dataout, int N)

*************************************************************************/

void cmplxvectorn::idft(const cmplxvectorn& Datain)
{
	cmplxvectorn& Dataout=*this;
	int N=Datain.size();

    int i,k,n,p;
    static int nstore = 0;      // store N for future use 
    static cmplxvectorn cf;         // coefficient storage 
    double arg;

// Create the coefficients if N has changed 

    if(N != nstore) {
		cf.setSize(N);

// scale stored values by 1/N 
        arg = 8.0*atan(1.0)/N;
        for (i=0 ; i<N ; i++) {
            cf[i].real = (double)(cos(arg*i)/(double)N);
            cf[i].imag = (double)(sin(arg*i)/(double)N);
        }
    }

// Perform the DFT calculation 

    printf("\n");
	int inIndex=0;
	int outIndex=0;
    for (k=0 ; k<N ; k++) {

        inIndex=0;
		complex_old& Din = Datain[inIndex];
        Dataout[outIndex].real = Din.real * cf[0].real;
        Dataout[outIndex].imag = Din.imag * cf[0].real;
        inIndex++;
        for (n=1; n<N; n++) {

			p = (int)((long)n*k % N);
			complex_old& cfelt = cf[p];         /* pointer to cf modulo N */

            Dataout[outIndex].real += Din.real * cfelt.real
                             - Din.imag * cfelt.imag;

            Dataout[outIndex].imag += Din.real * cfelt.imag
                             + Din.imag * cfelt.real;
            inIndex++;
        }
        if (k % 32 == 31) printf("*");
        outIndex++;          // next output 
    }
    printf("\n");
}

/**************************************************************************

fft - In-place radix 2 decimation in frequency FFT

Requires pointer to complex_old array, x and power of 2 size of FFT, m
(size of FFT = 2^m).  Places FFT output on top of input COMPLEX array.

void fft(COMPLEX *x, int m)

*************************************************************************/

void cmplxvectorn::fft()
{
	complex_old* x=v;

	int m=log2(size());

	static cmplxvectorn wvec;
 	static complex_old *w;           // used to store the w complex_old array 
    static int mstore = 0;       // stores m for future reference 
    static int n = 1;            // length of fft stored for future 

    complex_old u,temp,tm;
    complex_old *xi,*xip,*xj,*wptr;

    int i,j,k,l,le,windex;

    double arg,w_real,w_imag,wrecur_real,wrecur_imag,wtemp_real;

    if(m != mstore) {

// free previously allocated storage and set new m 

        mstore = m;
        if(m == 0) return;       // if m=0 then done 

// n = 2^m = fft length 

        n = 1 << m;   
        le = n/2;  //difference between the upper and lower leg indices

// allocate the storage for w 

		wvec.setSize(le-1);
        w = &wvec[0];

        if(!w) {
            printf("\nUnable to allocate complex_old W array\n");
            exit(1);
        }

// calculate the w values recursively 

        arg = M_PI/le;         //  PI/le calculation 
        wrecur_real = w_real = cos(arg);
        wrecur_imag = w_imag = -sin(arg);
        xj = w;
        for (j = 1 ; j < le ; j++) {
            xj->real = (double)wrecur_real;
            xj->imag = (double)wrecur_imag;
            xj++;
            wtemp_real = wrecur_real*w_real - wrecur_imag*w_imag;
            wrecur_imag = wrecur_real*w_imag + wrecur_imag*w_real;
            wrecur_real = wtemp_real;
        }
    }

// start fft 

    le = n;
    windex = 1;
    for (l = 0 ; l < m ; l++) {
        le = le/2;

// first iteration with no multiplies 

        for(i = 0 ; i < n ; i = i + 2*le) {
            xi = x + i;
            xip = xi + le;
            temp.real = xi->real + xip->real;
            temp.imag = xi->imag + xip->imag;
            xip->real = xi->real - xip->real;
            xip->imag = xi->imag - xip->imag;
            *xi = temp;
        }

// remaining iterations use stored w 

        wptr = w + windex - 1;
        for (j = 1 ; j < le ; j++) {
            u = *wptr;
            for (i = j ; i < n ; i = i + 2*le) {
                xi = x + i;
                xip = xi + le;
                temp.real = xi->real + xip->real;
                temp.imag = xi->imag + xip->imag;
                tm.real = xi->real - xip->real;
                tm.imag = xi->imag - xip->imag;             
                xip->real = tm.real*u.real - tm.imag*u.imag;
                xip->imag = tm.real*u.imag + tm.imag*u.real;
                *xi = temp;
            }
            wptr = wptr + windex;
        }
        windex = 2*windex;
    }            

// rearrange data by bit reversing 

    j = 0;
    for (i = 1 ; i < (n-1) ; i++) {
        k = n/2;
        while(k <= j) {
            j = j - k;
            k = k/2;
        }
        j = j + k;
        if (i < j) {
            xi = x + i;
            xj = x + j;
            temp = *xj;
            *xj = *xi;
            *xi = temp;
        }
    }
}

/**************************************************************************

ifft - In-place radix 2 decimation in time inverse FFT

Requires pointer to complex_old array, x and power of 2 size of FFT, m
(size of FFT = 2^m).  Places inverse FFT output on top of input
frequency domain COMPLEX array.

void ifft(COMPLEX *x, int m)

*************************************************************************/

void cmplxvectorn::ifft()
{
	complex_old* x=v;
	int m=log2(size());
	
	static cmplxvectorn wvec;
    static complex_old *w;           // used to store the w complex_old array 
    static int mstore = 0;       // stores m for future reference 
    static int n = 1;            // length of ifft stored for future 

    complex_old u,temp,tm;
    complex_old *xi,*xip,*xj,*wptr;

    int i,j,k,l,le,windex;

    double arg,w_real,w_imag,wrecur_real,wrecur_imag,wtemp_real;
    double scale;

    if(m != mstore) {

        mstore = m;
        if(m == 0) return;       // if m=0 then done 

// n = 2^m = inverse fft length 

        n = 1 << m;
        le = n/2;

// allocate the storage for w 

		wvec.setSize(le -1);
        w = &wvec[0];
        if(!w) {
            printf("\nUnable to allocate complex_old W array\n");
            exit(1);
        }

// calculate the w values recursively 

        arg = 4.0*atan(1.0)/le;         // PI/le calculation 
        wrecur_real = w_real = cos(arg);
        wrecur_imag = w_imag = sin(arg);  // opposite sign from fft 
        xj = w;
        for (j = 1 ; j < le ; j++) {
            xj->real = (double)wrecur_real;
            xj->imag = (double)wrecur_imag;
            xj++;
            wtemp_real = wrecur_real*w_real - wrecur_imag*w_imag;
            wrecur_imag = wrecur_real*w_imag + wrecur_imag*w_real;
            wrecur_real = wtemp_real;
        }
    }

// start inverse fft 

    le = n;
    windex = 1;
    for (l = 0 ; l < m ; l++) {
        le = le/2;

// first iteration with no multiplies 

        for(i = 0 ; i < n ; i = i + 2*le) {
            xi = x + i;
            xip = xi + le;
            temp.real = xi->real + xip->real;
            temp.imag = xi->imag + xip->imag;
            xip->real = xi->real - xip->real;
            xip->imag = xi->imag - xip->imag;
            *xi = temp;
        }

// remaining iterations use stored w 

        wptr = w + windex - 1;
        for (j = 1 ; j < le ; j++) {
            u = *wptr;
            for (i = j ; i < n ; i = i + 2*le) {
                xi = x + i;
                xip = xi + le;
                temp.real = xi->real + xip->real;
                temp.imag = xi->imag + xip->imag;
                tm.real = xi->real - xip->real;
                tm.imag = xi->imag - xip->imag;             
                xip->real = tm.real*u.real - tm.imag*u.imag;
                xip->imag = tm.real*u.imag + tm.imag*u.real;
                *xi = temp;
            }
            wptr = wptr + windex;
        }
        windex = 2*windex;
    }            

// rearrange data by bit reversing 

    j = 0;
    for (i = 1 ; i < (n-1) ; i++) {
        k = n/2;
        while(k <= j) {
            j = j - k;
            k = k/2;
        }
        j = j + k;
        if (i < j) {
            xi = x + i;
            xj = x + j;
            temp = *xj;
            *xj = *xi;
            *xi = temp;
        }
    }

// scale all results by 1/n 
    scale = (double)(1.0/n);
    for(i = 0 ; i < n ; i++) {
        x->real = scale*x->real;
        x->imag = scale*x->imag;
        x++;
    }
}

void cmplxvectorn::fromTrigvectorn(const vectorn& vecx)
{
	setSize(vecx.size()/2);

	for(int i=0; i<size(); i++)
	{
		v[i].real=vecx[i*2];
		v[i].imag=vecx[i*2+1];
	}
}

void cmplxvectorn::fromvectorn(const vectorn& vecx)
{
	setSize(vecx.size());

	for(int i=0; i<size(); i++)
	{
		v[i].real=vecx[i];
		v[i].imag=0.f;
	}
}

void cmplxvectorn::rfft(const vectorn& vecx)
{
	fromvectorn(vecx);
	fft();
}

/************************************************************

rfft - trig recombination real input FFT

Requires real array pointed to by x, pointer to complex_old
output array, y and the size of real FFT in power of
2 notation, m (size of input array and FFT, N = 2**m).
On completion, the COMPLEX array pointed to by y 
contains the lower N/2 + 1 elements of the spectrum.

void rfft(double *x, COMPLEX *y, int m)

***************************************************************/

void cmplxvectorn::rtrigfft(const vectorn& vecx)
{
	m_real* x=&vecx[0];
	int m=log2(vecx.size());
	setSize(vecx.size()/2);
	complex_old* y=v;
	static	cmplxvectorn cfvec;
    static    complex_old *cf;
    static    int      mstore = 0;
    int       p,num,k,index;
    double     Realsum, Realdif, Imagsum, Imagdif;
    double    factor, arg;
    complex_old *ck, *xk, *xnk, *cx;

// First call the fft routine using the x array but with
//  half the size of the real fft 

    p = m - 1;
	cmplxvectorn cxvec;
	cxvec.fromvectorn(vecx);
	cxvec.fft();
	
    cx = &cxvec[0];

// Next create the coefficients for recombination, if required 

    num = 1 << p;    // num is half the real sequence length.  

    if (m!=mstore){

		cfvec.setSize(num-1);
		cf=&cfvec[0];

      factor = 4.0*atan(1.0)/num;
      for (k = 1; k < num; k++){
        arg = factor*k;
        cf[k-1].real = (double)cos(arg);
        cf[k-1].imag = (double)sin(arg);
      }
    }  

// DC component, no multiplies 
    y[0].real = cx[0].real + cx[0].imag;
    y[0].imag = 0.0;

// other frequencies by trig recombination 
    ck = cf;
    xk = cx + 1;
    xnk = cx + num - 1;
    for (k = 1; k < num; k++){
      Realsum = ( xk->real + xnk->real ) / 2;
      Imagsum = ( xk->imag + xnk->imag ) / 2;
      Realdif = ( xk->real - xnk->real ) / 2;
      Imagdif = ( xk->imag - xnk->imag ) / 2;

      y[k].real = Realsum + ck->real * Imagsum
                          - ck->imag * Realdif ;

      y[k].imag = Imagdif - ck->imag * Imagsum
                          - ck->real * Realdif ;
      ck++;
      xk++;
      xnk--;
    }
}

/*************************************************************************

ham - Hamming window

Scales both real and imaginary parts of input array in-place.
Requires COMPLEX pointer and length of input array.

void ham(COMPLEX *x, int n)

*************************************************************************/
/*
void ham(x, n)
  COMPLEX    *x;
  int        n;
{
  int    i;
  double ham,factor;

  factor = 8.0*atan(1.0)/(n-1);
  for (i = 0 ; i < n ; i++){
    ham = 0.54 - 0.46*cos(factor*i);
    x->real *= ham;
    x->imag *= ham;
    x++;
  }
}

/*************************************************************************

han - Hanning window

Scales both real and imaginary parts of input array in-place.
Requires COMPLEX pointer and length of input array.

void han(COMPLEX *x, int n)

*************************************************************************/

void cmplxvectorn::han()
{
  int    i;
  double factor,han;

  factor = 8.0*atan(1.0)/(size()-1);
  for (i = 0 ; i < size() ; i++){
    han = 0.5 - 0.5*cos(factor*i);
    data(i).real*= han;
	data(i).imag*= han;
  }
}

/*************************************************************************

triang - triangle window

Scales both real and imaginary parts of input array in-place.
Requires COMPLEX pointer and length of input array.

void triang(COMPLEX *,int n)

*************************************************************************/
/*
void triang(x, n)
  COMPLEX    *x;
  int        n;
{
  int    i;
  double tri,a;
  a = 2.0/(n-1);

  for (i = 0 ; i <= (n-1)/2 ; i++) {
    tri = i*a;
    x->real *= tri;
    x->imag *= tri;
    x++;
  }
  for ( ; i < n ; i++) {
    tri = 2.0 - i*a;
    x->real *= tri;
    x->imag *= tri;
    x++;
  }
}

/*************************************************************************

black - Blackman window

Scales both real and imaginary parts of input array in-place.
Requires COMPLEX pointer and length of input array.

void black(COMPLEX *x, int n)

*************************************************************************/

/*
void black(x, n)
  COMPLEX    *x;
  int        n;
{
  int    i;
  double black,factor;

  factor = 8.0*atan(1.0)/(n-1);
  for (i=0; i<n; ++i){
    black = 0.42 - 0.5*cos(factor*i) + 0.08*cos(2*factor*i);
    x->real *= black;
    x->imag *= black;
    x++;
  }
}

/*************************************************************************

harris - 4 term Blackman-Harris window

Scales both real and imaginary parts of input array in-place.
Requires COMPLEX pointer and length of input array.

void harris(COMPLEX *x, int n)

*************************************************************************/
/*
void harris(x, n)
  COMPLEX    *x;
  int        n;
{
  int    i;
  double harris,factor,arg;

  factor = 8.0*atan(1.0)/n;
  for (i=0; i<n; ++i){
    arg = factor * i;
    harris = 0.35875 - 0.48829*cos(arg) + 0.14128*cos(2*arg)
               - 0.01168*cos(3*arg);
    x->real *= harris;
    x->imag *= harris;
    x++;
  }
}

/**************************************************************************

log2 - base 2 logarithm

Returns base 2 log such that i = 2^ans where ans = log2(i).
if log2(i) is between two values, the larger is returned.

int log2(unsigned int x)

*************************************************************************/
int cmplxvectorn::log2(int x)
{
	int m, estSize;
	for(m=1, estSize=2; estSize<size();  m++,estSize*=2);
	ASSERT(estSize==size());
	return m;
}

/* backup (rfft numerical recipe versions)
	void four1(int isign);	// used in fourier transform
		//! fourier transform
	 Calculates the Fourier transform of a set of n real-valued data points. Replaces this data ( which is stored in array data) 
	by the positive frequency half of its complex_old fourier transform. The real-valued first and last components of the complex_old transform are 
	returend as elements data[0] and data[1], respectively. n must be a power of 2. This routine also calculates the inverse transform
	of a complex_old data array if it is the transform of real data. (Result in this case must be multiplied by 2/n)
	참고사항: 입력은 real, 결과는 complex_old형이다. 즉 real과 imagenary가 번갈아 나오는 포맷. 
	vectorn& fourierTransform(bool bForwardTransform);
	void fourierDecompose(vectorn& real, vectorn& imag);
	vectorn& fourierMagnitude(const vectorn& freq); 

	vectorn& vectorn::fourierTransform(bool bForwardTransform)
{
	int i,i1,i2,i3,i4;
	vectorn& data=*this;
	m_real c1=0.5,c2,h1r,h1i,h2r,h2i,wr,wi,wpr,wpi,wtemp,theta;

	int n=size();
	theta=3.141592653589793238/m_real(n>>1);
	if (bForwardTransform) {
		c2 = -0.5;
		data.four1(1);
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
		h1r=c1*(data[i1]+data[i3]);
		h1i=c1*(data[i2]-data[i4]);
		h2r= -c2*(data[i2]+data[i4]);
		h2i=c2*(data[i1]-data[i3]);
		data[i1]=h1r+wr*h2r-wi*h2i;
		data[i2]=h1i+wr*h2i+wi*h2r;
		data[i3]=h1r-wr*h2r+wi*h2i;
		data[i4]= -h1i+wr*h2i+wi*h2r;
		wr=(wtemp=wr)*wpr-wi*wpi+wr;
		wi=wi*wpr+wtemp*wpi+wi;
	}
	if (bForwardTransform) {
		data[0] = (h1r=data[0])+data[1];
		data[1] = h1r-data[1];
	} else {
		data[0]=c1*((h1r=data[0])+data[1]);
		data[1]=c1*(h1r-data[1]);
		data.four1(-1);
	}
	return data;
}

void vectorn::four1(int isign)	// used in fourier transform
{
	vectorn& data=*this;
	int n,mmax,m,j,istep,i;
	m_real wtemp,wr,wpr,wpi,wi,theta,tempr,tempi;

	int nn=size()/2;
	n=nn << 1;
	j=1;
	for (i=1;i<n;i+=2) {
		if (j > i) {
			data.swap(j-1,i-1);
			data.swap(j,i);
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
				tempr=wr*data[j-1]-wi*data[j];
				tempi=wr*data[j]+wi*data[j-1];
				data[j-1]=data[i-1]-tempr;
				data[j]=data[i]-tempi;
				data[i-1] += tempr;
				data[i] += tempi;
			}
			wr=(wtemp=wr)*wpr-wi*wpi+wr;
			wi=wi*wpr+wtemp*wpi+wi;
		}
		mmax=istep;
	}
}

void vectorn::fourierDecompose(vectorn& real, vectorn& imag)
{
	real.setSize(size()/2);
	imag.setSize(size()/2);
	for(int i=0; i<size()/2; i++)
	{
		real[i]=(*this)[2*i];
		imag[i]=(*this)[2*i+1];
	}
}

vectorn& vectorn::fourierMagnitude(const vectorn& freq)
{
	setSize(freq.size()/2);
	for(int i=0; i<size(); i++)
	{
		(*this)[i]=sqrt(SQR(freq[2*i])+SQR(freq[2*i+1]));
	}
	return *this;
}

*/

void cmplxvectorn::real(vectorn& real)
{
	real.setSize(size());

	for(int i=0; i<size();i++)
		real[i]=data(i).real;
}

void cmplxvectorn::imag(vectorn& imag)
{
	imag.setSize(size());

	for(int i=0; i<size();i++)
		imag[i]=data(i).imag;
}

void cmplxvectorn::mag(vectorn& mag)
{
	mag.setSize(size());

	for(int i=0; i<size();i++)
		mag[i]=data(i).length();
}

void cmplxvectorn::fromvectorn(const vectorn& vecx, int windowSize, int offset)
{
	setSize(windowSize);
	for(int i=0; i<windowSize; i++)
	{
		int cur=i+offset-windowSize/2;
		if(cur>=0 && cur<vecx.size())
			data(i).real=vecx[cur];
		else 
			data(i).real=0;		// zero padding. its safe because i already applied hanned window
        
		data(i).imag=0;
	}
}
