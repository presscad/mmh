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

#ifndef VP_JOINT
#define VP_JOINT

#include <VP/vpDataType.h>

/*!
	\class vpJoint
	\brief Abstract class for joint
	
	vpJoint is an abstract class to connect two adjacent bodies.
	\sa vpRJoint vpPJoint vpUJoint vpSJoint vpWJoint
*/

class vpJoint
{
	friend class			 vpState;
	friend class			 vpBody;
	friend class			 vpSystem;
	friend class			 vpWorld;

public:

							 vpJoint();

	/*!
		break the joint.
	*/
	void					 Break(void);

	/*!
		return the degrees of freedom.
	*/
	virtual int				 GetDOF(void) const = 0;

	/*!
		get the maximum magnitude of normal force that can break the joint.
	*/
	const scalar			&GetMaxNormalForce(void) const;

	/*!
		set the maximum magnitude of normal force that can break the joint.
	*/
	void					 SetMaxNormalForce(const scalar &);

	/*!
		get the maximum magnitude of normal torque that can break the joint.
	*/
	const scalar			&GetMaxNormalTorque(void) const;

	/*!
		set the maximum magnitude of normal torque that can break the joint.
	*/
	void					 SetMaxNormalTorque(const scalar &);

	/*!
		get a magnitude of normal force applied to the joint.
	*/
	virtual scalar			 GetNormalForce(void) const = 0;

	/*!
		get a magnitude of normal torque applied to the joint.
	*/
	virtual	scalar			 GetNormalTorque(void) const = 0;

	/*!
		set whether hybrid dynamics respects acceleration or torque
	*/
	void					 SetHybridDynamicsType(VP::HD_TYPE);
	VP::HD_TYPE				 GetHybridDynamicsType(void) const;

	virtual void			 streamOut(ostream &) const;

	string					 m_szName;

protected:

	void					 Initialize(void);
	void					 UpdateForce(void);
	bool					 IsOverMaxNormalForce(void) const;
	void					 SetBody(VP::SIDE, vpBody *, const SE3 &);
	vpStateArray			&GetState(void) const;
	virtual void			 SwapBody(void);
	virtual	bool			 Reparameterize(void);
	virtual void			 BuildKinematics(void) = 0;
	virtual SE3				 Transform(void) const = 0;
	virtual void			 UpdateSpringDamperTorque(void) = 0;
	virtual scalar			 GetPotentialEnergy(void) const = 0;
	
	virtual const scalar	&GetDisplacement_(int) const = 0;
	virtual void			 SetDisplacement_(int, const scalar &) = 0;
	virtual const scalar	&GetVelocity_(int) const = 0;
	virtual void			 SetVelocity_(int, const scalar &) = 0;
	virtual const scalar	&GetAcceleration_(int) const = 0;
	virtual void			 SetAcceleration_(int, const scalar &) = 0;
	virtual const scalar	&GetImpulsiveTorque_(int) const = 0;
	virtual void			 SetImpulsiveTorque_(int, const scalar &) = 0;
	virtual scalar			 GetTorque_(int) const = 0;
	virtual void			 SetTorque_(int, const scalar &) = 0;
	virtual void			 SetSpringDamperTorque_(int, const scalar &) = 0;
	virtual	const scalar	&GetRestitution_(int) const = 0;
	virtual bool			 ViolateUpperLimit_(int) const = 0;
	virtual bool			 ViolateLowerLimit_(int) const = 0;

	virtual void			 UpdateTorqueID(void) = 0;
	virtual void			 UpdateTorqueHD(void) = 0;
	virtual void			 UpdateVelocity(const se3 &) = 0;
	virtual void			 UpdateAccelerationID(const se3 &) = 0;
	virtual void			 UpdateAccelerationFD(const se3 &) = 0;
	virtual void			 UpdateAInertia(AInertia &) = 0;
	virtual void			 UpdateLOTP(void) = 0;
	virtual void			 UpdateTP(void) = 0;
	virtual void			 UpdateLP(void) = 0;
	virtual dse3			 GetLP(void) = 0;
	virtual void			 ClearTP(void) = 0;
	
	virtual void			 IntegrateDisplacement(const scalar &) = 0;
	virtual void			 IntegrateVelocity(const scalar &) = 0;

	Inertia					 m_sI;
	AInertia				 m_sJ;
	SE3						 m_sM;
	SE3						 m_sRelativeFrame;
	SE3						 m_sLeftBodyFrame;
	SE3						 m_sRightBodyFrame;
	se3						 m_sV;
	se3						 m_sDV;
	se3						 m_sW;					// ad(m_sV, Vl) + dSdq
	dse3					 m_sF;
	dse3					 m_sB;
	dse3					 m_sC;
	scalar					 m_rMaxNormalForce;
	scalar					 m_rMaxNormalTorque;
	int						 m_iIdx;				// cross reference index: m_pSystem->m_pJoint[m_iIdx] = this
	VP::SIGN				 m_eSign;
	bool					 m_bBreakable;
	VP::HD_TYPE				 m_sHDType;
	vpSystem				*m_pSystem;
	vpWorld					*m_pWorld;
	vpBody					*m_pLeftBody;
	vpBody					*m_pRightBody;
	vpJoint					*m_pParentJoint;
	vpJointPtrArray			 m_pChildJoints;
};

class vpState
{
public:
							 vpState();
							 vpState(vpJoint *, int);
	void					 SetDisplacement(const scalar &);
	const scalar			&GetDisplacement(void) const;
	void					 SetVelocity(const scalar &);
	const scalar			&GetVelocity(void) const;
	void					 SetAcceleration(const scalar &);
	const scalar			&GetAcceleration(void) const;
	void					 SetImpulsiveTorque(const scalar &);
	const scalar			&GetImpulsiveTorque(void) const;
	void					 SetSpringDamperTorque(const scalar &);
	scalar					 GetTorque(void) const;
	void					 SetTorque(const scalar &);
	const scalar			&GetRestitution(void) const;
	bool					 ViolateUpperLimit(void) const;
	bool					 ViolateLowerLimit(void) const;

	int						 m_iIdx;
	vpJoint					*m_pJoint;
};

void Putse3ToRMatrix(RMatrix &, const se3 &, int);

#ifndef VP_PROTECT_SRC
	#include "vpJoint.inl"
#endif

#endif
