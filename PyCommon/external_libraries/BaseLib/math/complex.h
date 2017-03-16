#pragma once
class complex_old
{
public:
    m_real real, imag;

    // negation
    friend complex_old    operator-( complex_old const& );

    // addtion
    friend complex_old    operator+( complex_old const&, complex_old const& );

    // subtraction
    friend complex_old    operator-( complex_old const&, complex_old const& );

    // dot product
    friend m_real    operator%( complex_old const&, complex_old const& );

    // Multiplication
    friend complex_old    operator*( complex_old const&, complex_old const& );

    // scalar Multiplication
    friend complex_old    operator*( complex_old const&, m_real );
    friend complex_old    operator*( m_real, complex_old const& );

    // scalar Division
    friend complex_old    operator/( complex_old const&, m_real );

    // functions
    friend complex_old		c_exp( m_real );
	friend m_real		c_ln( complex_old const& );
    friend complex_old		inverse( complex_old const& );
    friend m_real		len( complex_old const& );
    friend complex_old		slerp( complex_old const&, complex_old const&, m_real );
    friend complex_old		interpolate( m_real, complex_old const&, complex_old const& );
    friend m_real		distance( complex_old const&, complex_old const& );
    friend m_real		difference( complex_old const&, complex_old const& );

    // stream
//    friend ostream& operator<<( ostream&, complex_old const& );
  //  friend istream& operator>>( istream&, complex_old& );

  public:
	//
    // constructors
	//
    complex_old() {};
    complex_old( m_real x, m_real y)
                          { real=x; imag=y; }
    complex_old( m_real a[2] ) { real=a[0]; imag=a[1]; }

	//
    // inquiry functions
	//
    complex_old  inverse() const { return complex_old( real, -1.f*imag); }
	complex_old  normalize() const;
	m_real  length() const;

    void getValue( m_real d[2] ) { d[0]=real; d[1]=imag; }
    void setValue( m_real d[2] ) { real=d[0]; imag=d[1]; }

    m_real x() const { return real; }
    m_real y() const { return imag; }

	m_real&   operator[](int i)	 { if(i==0) return real; return imag;};
	//
    // Set parameters
	//
    void set_x( m_real rr) { real=rr; }
    void set_y( m_real ii) { imag=ii; }
};

