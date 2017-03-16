
#ifndef __PM_MOTION_H
#define __PM_MOTION_H



class PmMotion
{
  private:
	PmHuman*			body;
	int					size;

  protected:
	int					oSize;

  public:
	PmConstraint*		constraints;

  public:
	//
	// Constructors
	//
	PmMotion();
	~PmMotion();

	//
	// Set and get parameters
	//
	virtual PmMaskType	getMask() const=0;
	virtual int			getDOF() const=0;

	virtual void		setBody( PmHuman* h ) { body=h; }
	PmHuman*			getBody() const { return body; }

	int					getSize() const { return size; }
	virtual void		setSize( int );
	virtual void		extend( int );

	PmConstraint&		getConstraint( int i ) const { return constraints[i]; }
	void				setConstraint( int i, PmConstraint const& c )
							{ constraints[i] = c; }

	PmMotion&			operator=( PmMotion const& );
};

#endif
