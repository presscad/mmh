#pragma once
class vectorn;
/**
 * \ingroup group_math
 *
 * 
 * \todo 
 *
 * \bug 
 *
 */
class cmplxvectorn
{
private:
    int     n,
			on;	// size of memory allocated. if memory is allocated in matrixn class, on is 0!!.
    complex_old *v;

public:

	cmplxvectorn(int size);
	cmplxvectorn();
	~cmplxvectorn();

	int size() const		{return n;}
	void setSize( int x );
	void resize(int nsize);
	void assign(const cmplxvectorn& other);
	void assign( const cmplxvectorn& value, int start, int end);
	complex_old&   operator[](int i) const { assert(i>=0 && i<n); return v[i]; }
	complex_old&   data(int i) const { assert(i>=0 && i<n); return v[i]; }

	void fft();								//!< In-place radix 2 decimation in time FFT
	void ifft();							//!< In-place radix 2 decimation in time inverse FFT
	void dft(const cmplxvectorn& other);	//!< Discrete Fourier Transform
	void idft(const cmplxvectorn& other);	//!< Inverse Discrete Fourier Transform
	void rtrigfft(const vectorn& other);	//!< Trig recombination real input FFT
	void rfft(const vectorn& other);		//!< real input FFT
	void ham(int);							//!< Hamming window
	void han();								//!< Hanning window in place
	void triang(int);						//!< Triangle window
	void black(int);						//!< Blackman window
	void harris(int);						//!< 4 term Blackman-Harris window
	int log2(int);							//!< Base 2 logarithm
	
	void real(vectorn& real);
	void imag(vectorn& imag);
	void mag(vectorn& mag);

	void fromTrigvectorn(const vectorn& vecx);// real, imag alternating vectorn
	void fromvectorn(const vectorn& vecx);
	void fromvectorn(const vectorn& vecx, int windowSize, int offset);
};