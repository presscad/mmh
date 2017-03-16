/*
VirtualPhysics v0.9

VirtualPhysics was developed as part of the project entitled "Development of 
Real-time Physics Simulation Engine for e-Entertainments" which was financially
supported by the grant from the strategic technology development program
(Project No. 2008-F-033-02) of both the MKE(Ministry of Knowledge Economy) and
MCST(Ministry of Culture, Sports and Tourism) of Korea.

Copyright (c) 2008-2010, Jinwook Kim, Korea Institute of Science and Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.

   3. Only research partners of the project "Development of Real-time Physics 
      Simulation Engine for e-Entertainments" can modify this list of 
	  conditions and the following disclaimer with the consent of the author, 
	  where the research partners refer to all principal investigators 
	  involved in the project. 

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifndef VP_SYSTEM
#define VP_SYSTEM

#include <VP/vpDataType.h>
#include <VP/vpBody.h>

class vpSystem
{
	friend class			 vpJoint;
	friend class			 vpBody;
	friend class			 vpWorld;

public:
	/*!
		get a number of joints in the system.
	*/
	virtual int				 GetNumJoint(void) const;

	/*!
		get a number of bodies in the system.
	*/
	virtual int				 GetNumBody(void) const;

	/*!
		get a kinetic energy of the system.
	*/
	virtual scalar			 GetKineticEnergy(void) const;

	/*!
		get a potential energy of the system.
	*/
	virtual scalar			 GetPotentialEnergy(void) const;

	virtual void			 BackupState(void);
	virtual void			 RollbackState(void);

	virtual void			 ForwardDynamics(void);
	virtual void			 ForwardDynamics2(void);
	virtual void			 InverseDynamics(void);
	virtual void			 HybridDynamics(void);

protected:
							 vpSystem();

	friend ostream			&operator << (ostream &, const vpWorld &);

	virtual void			 Initialize(bool init_dynamics = true);
	virtual vpBody			*GetRoot(void);
	virtual void			 SetCollisionID(int);
	virtual void			 SetContactID(int);
	virtual void			 Reparameterize(void);
	virtual void			 BuildKinematics(void);
	virtual void			 BuildDynamics(void);
	virtual void			 UpdateFrame(bool = true);
	virtual void			 FDIteration1(void);
	virtual void			 FDIteration2(void);
	virtual void			 FDIteration2s(void);
	virtual void			 FDIteration2s(int);
	virtual void			 FDIteration2s(vpBody *);
	virtual void			 FDIteration3(void);
	virtual void			 FDIteration3s(void);
	virtual void			 IDIteration1(void);
	virtual void			 IDIteration2(void);
	virtual void			 HDIteration2(void);
	virtual void			 HDIteration3(void);

	virtual void			 Register2BrokenJoints(vpJoint *);
	virtual void			 IntegrateDynamicsEuler(scalar);
	virtual void			 IntegrateDynamicsRK4(scalar);
	virtual void			 IntegrateDynamicsBackwardEuler(scalar);
	virtual void			 IntegrateDynamicsBackwardEulerFast(scalar);

	AInertia				 m_sRootInertia;
	AInertia				 m_sRootInvInertia;
	dse3					 m_sRootBias;
	int						 m_iNumTotalDOF;
	vpWorld					*m_pWorld;
	vpBody					*m_pRoot;
	vpBodyPtrArray	 		 m_pBody;
	vpJointPtrArray	 		 m_pJoint;
	set<vpJoint *>			 m_pBrokenJoint;
	SE3Array		 		 m_sT;
	vpStateArray	 		 m_sState;
	vpSpringPtrArray		 m_pSpring;

	scalarArray		 		 m_sStateDisplacement;
	scalarArray		 		 m_sStateVelocity;

	RMatrix					 _Jv;
	bool					 _JvInitialized;
};

#ifndef VP_PROTECT_SRC
	#include "vpSystem.inl"
#endif

#endif
