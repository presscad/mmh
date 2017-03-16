
#ifndef POINT2_H
#define POINT2_H

namespace jhm
{

class point2
{
  private:
    int p[2];

    // negation
    friend point2    operator-( point2 const& );
    
    // addtion
    friend point2&   operator+=( point2&, point2 const& );
    friend point2    operator+( point2 const&, point2 const& );
    
    // subtraction
    friend point2    operator-( point2 const&, point2 const& );
    friend point2&   operator-=( point2&, point2 const& );

  public:
    // constructors
    point2() {}
    point2( int x, int y ) { p[0]=x; p[1]=y; }
    point2( int a[2] ) { p[0]=a[0]; p[1]=a[1]; }

    // inquiry functions
    int& operator[](int i) { return p[i]; }

	int x() const { return p[0]; };
	int y() const { return p[1]; };
    void   getValue( int d[2] ) { d[0]=p[0]; d[1]=p[1]; }
    void   setValue( int d[2] ) { p[0]=d[0]; p[1]=d[1]; }
	int    getValue( int n ) const { return p[n]; }
	point2 setValue( int x, int y )
								   { p[0]=x, p[1]=y; return *this; }
	

	m_real length() const;


    // change functions
    void set_x( int x ) { p[0]=x; };
    void set_y( int x ) { p[1]=x; };
};

}

#endif
