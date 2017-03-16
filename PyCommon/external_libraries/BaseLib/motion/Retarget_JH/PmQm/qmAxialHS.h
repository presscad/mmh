
#ifndef __QM_AXIAL_HS_H
#define __QM_AXIAL_HS_H

#include "qmHalfspace.h"



class QmAxialHS : public QmHalfspace
{
  private:
	jhm::quater		orientation;
	jhm::unit_vector	axis;
	jhm::interval	angle_bound;

	//
    // stream
    //
    friend std::ostream& operator<<( std::ostream&, QmAxialHS const& );
    friend std::istream& operator>>( std::istream&, QmAxialHS& );

  public:
    //
    // Constructors
    //
	QmAxialHS() {}
	QmAxialHS( jhm::quater const& q, jhm::unit_vector const& v, jhm::interval const& i )
		{ orientation=q; axis=v; angle_bound=i; }
	

    //
    // Inquiry functions
    //
    int     inclusion( jhm::quater const& );
    int     intersection( QmGeodesic const&, jhm::quater* );
    m_real	distance( jhm::quater const& );
	jhm::quater	nearest( jhm::quater const& );
	jhm::vector	gradient( jhm::quater const& );

    //
    // Set and get parameters
    //
    void        setOrientation( jhm::quater const& q ) { orientation=q; }
    void        setAxis( jhm::unit_vector const& v )   { axis=v; }
    void        setAngleBound( jhm::interval const& i ){ angle_bound=i; }
    jhm::quater      getOrientation() const { return orientation; }
    jhm::unit_vector getAxis()        const { return axis; }
    jhm::interval	getAngleBound()  const { return angle_bound; }

	//
	// File ouput
	//
	void		write( std::ostream& );
};

#endif

