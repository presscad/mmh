
#ifndef __QM_HALFSPACE_H
#define __QM_HALFSPACE_H

#include "qmGeodesic.h"



class QmHalfspace
{
  private:
	QmHalfspace* next;
	QmHalfspace* prev;

  public:
	enum { EMPTY, INCLUDED, ONE_POINT, TWO_POINT };

	//
	// Constructors
	//
	QmHalfspace() { next=prev=NULL; }

    //
    // Set and get parameters
    //
	QmHalfspace*	getNext() { return next; }
	QmHalfspace*	getPrev() { return prev; }
	QmHalfspace*	setNext( QmHalfspace* h ) { next=h; return next; }
	QmHalfspace*	setPrev( QmHalfspace* h ) { prev=h; return prev; }

	//
	// Inquiry functions
	//
	virtual int		inclusion( jhm::quater const& ) = 0;
	virtual int		intersection( QmGeodesic const&, jhm::quater* ) = 0;
	virtual m_real	distance( jhm::quater const& ) =0;
	virtual	jhm::quater	nearest( jhm::quater const& ) = 0;
	virtual	jhm::vector	gradient( jhm::quater const& ) = 0;

	//
	// File ouput
	//
	virtual void	write( std::ostream& ) = 0;
};

#endif

