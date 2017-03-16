
#ifndef transf_H
#define transf_H

// rigid transformation
class transf
{
  public:
    quater rotation;
    vector3 translation;

	// unary operations
	void leftMult(const transf& a);	//!< this=a*this;
	void operator*=(const transf& a);	//!< this=this*a;
	friend vector3&      operator*=( vector3& v, transf const& f);	//!< v=f*v

    // binary operations
    void interpolate( m_real, transf const&, transf const& );
	void mult(transf const&, transf const&);
    friend transf       operator* ( transf const&, transf const& );
    friend vector3       operator* ( transf const& , vector3 const&);

    // constructors
    transf() {};
    transf( quater const& a, vector3 const& b ) { rotation=a; translation=b; }
	transf(matrix4 const& a)	{ rotation.setRotation(a); translation.translation(a);}
    transf			inverse() const;

	void identity();

	void operator=(matrix4 const& a);
};

#endif
