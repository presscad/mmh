
#ifndef __PM_VECTOR_ARRAY_H
#define __PM_VECTOR_ARRAY_H



class PmLinearMotion;

class PmVectorArray
{
  private:
	int				oSize;
	int				size;

  public:
	PmVector*		vectors;

  public:
	PmVectorArray();
	~PmVectorArray();

	PmMaskType		getMask() const;

	int				getSize() const { return size; }
	void			setSize( int );

	PmVector&		getVector( int i ) const { return vectors[i]; }
	void			setVector( int i, PmVector const& v ) { vectors[i] = v; }

	void			getLinearVectors( jhm::vector* ) const;
	void			setLinearVectors( jhm::vector* );

	void			getAngularVectors( int, jhm::vector* ) const;
	void			setAngularVectors( int, jhm::vector* );

	void			amplify( m_real, PmMaskType );

	//
	// Operators
	//
    PmVectorArray&	operator=( PmVectorArray const& );
    PmVectorArray&	operator+=( PmVectorArray const& );
	PmVectorArray&	operator*=( m_real );
	PmVectorArray&	operator*=( PmVectorArray const& );
	PmVectorArray&	operator/=( m_real );

	PmVectorArray&	difference( PmLinearMotion const&, PmLinearMotion const& );
	PmVectorArray&	difference( PmVectorArray const&, PmVectorArray const& );
};

#endif

