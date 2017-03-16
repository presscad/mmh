
#ifndef __QM_COMPLEX_H
#define __QM_COMPLEX_H

#include "qmConicHS.h"
#include "qmAxialHS.h"
#include "qmSphericalHS.h"



class QmComplex
{
  private:
	int				num;
	QmHalfspace*	root;

    //
    // stream
    //
    friend std::ostream& operator<<( std::ostream&, QmComplex const& );
    friend std::istream& operator>>( std::istream&, QmComplex& );

  public:
    //
    // Constructors
    //
	QmComplex() { num=0; root=NULL; }

	//
	// Manipulate List
	//
	QmHalfspace* addHalfspace( QmHalfspace* );
	QmHalfspace* removeHalfspace( QmHalfspace* );

    //
    // Inquiry functions
    //
    int     inclusion( jhm::quater const& );
    int     intersection( QmGeodesic const&, jhm::quater* );

	m_real	distance(jhm::quater const& );
	jhm::vector 	gradient( jhm::quater const& );

	jhm::quater	project( jhm::quater const& );
	jhm::matrix	project( jhm::matrix const& );
	jhm::transf	project( jhm::transf const& );
};

#endif

