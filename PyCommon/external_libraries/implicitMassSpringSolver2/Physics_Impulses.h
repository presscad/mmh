// Physics_GravityForce.h: interface for the Physics_GravityForce class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHYSICS_impulse_H__D6C2DAAB_5C83_4E63_B1F6_23F8C3BC2BCB__INCLUDED_)
#define AFX_PHYSICS_impulse_H__D6C2DAAB_5C83_4E63_B1F6_23F8C3BC2BCB__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class Physics_Contacts //: public Physics_Force  
{
	
public:
	Physics_LargeVector relativeVelocity;
	Physics_LargeVector deltaPosition;
	Physics_LargeVector normal;
	

	enum {begin_contact, continued_contact, no_contact};
	
	std::vector<short> contactStatus;

	// informations of previous frame
	std::vector<short> prevContactStatus;
	std::vector<void*> contactInfo;

	Physics_LargeVector prevRelativeVelocity;
	Physics_LargeVector prevNormal;

	Physics_Contacts(int nparticles);
	virtual ~Physics_Contacts();

	//virtual void Apply( Physics_t fTime, Physics_LargeVector &masses, bool bDerivs,
	//					Physics_LargeVector &p, Physics_LargeVector &v, 
	//					Physics_LargeVector &f_int, Physics_LargeVector &f_ext,
	//					Physics_SparseSymmetricMatrix &dp, Physics_SparseSymmetricMatrix &dv);

	void setContacts(int iparticle, double dvx, double dvy, double dvz, double dx, double dy, double dz, double nx, double ny, double nz, void* info)
	{
		relativeVelocity[iparticle].x=dvx;
		relativeVelocity[iparticle].y=dvy;
		relativeVelocity[iparticle].z=dvz;
		deltaPosition[iparticle].x=dx;
		deltaPosition[iparticle].y=dy;
		deltaPosition[iparticle].z=dz;
		normal[iparticle].x=nx;
		normal[iparticle].y=ny;
		normal[iparticle].z=nz;
		switch(contactStatus[iparticle])
		{
		case no_contact:
			contactStatus[iparticle]=begin_contact;
			break;
		case begin_contact:
			contactStatus[iparticle]=continued_contact;
			break;
		}
		contactInfo[iparticle]=info;
	}


	void saveContactsInformation(int iparticle)
	{
		prevContactStatus[iparticle]=contactStatus[iparticle];
		if(contactStatus[iparticle]!=no_contact)
		{
			prevRelativeVelocity[iparticle]=relativeVelocity[iparticle];
			prevNormal[iparticle]=normal[iparticle];
		}
	}

	void setNoContact(int i)	
	{
		if(contactStatus[i]!=no_contact)
		{
			deltaPosition[i].Zero();
			relativeVelocity[i].Zero();
			normal[i].Zero();
			contactInfo[i]=NULL;

		}
		contactStatus[i]=no_contact;
	}
};

#endif // !defined(AFX_PHYSICS_impulse_H__D6C2DAAB_5C83_4E63_B1F6_23F8C3BC2BCB__INCLUDED_)
