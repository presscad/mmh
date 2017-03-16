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

#ifndef VP_PJOINT
#define VP_PJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class vpPJoint
	\brief Prismatic joint
	
	vpPJoint is a class to model a prismatic joint.
	Connected bodies can slide relatively about a given direction. 
	<img src="\vp_site/pjoint.gif">
*/
class vpPJoint : public vpJoint
{
public:

							 vpPJoint();

	/*!
		set a sliding direction of the joint.
	*/
	void					 SetDirection(const Vec3 &);

	/*!
		set a displacement of the joint variable.
	*/
	void					 SetDisplacement(const scalar &);

	/*!
		set a velocity of the joint variable.
	*/
	void					 SetVelocity(const scalar &);

	/*!
		set an acceleration of the joint variable.
	*/
	void					 SetAcceleration(const scalar &);

	/*!
		set a force of the joint.
		The force will be reset after every simulation step
	*/
	void					 SetForce(const scalar &);

	/*!
		add a force to the joint.
		The force will be reset after every simulation step
	*/
	void					 AddForce(const scalar &);

	/*!
		set an initial distance of the joint variable.
	*/
	void					 SetInitialDisplacement(const scalar &);

	/*!
		set an elasticity of the joint variable.
	*/
	void					 SetElasticity(const scalar &);

	/*!
		set a damping parameter of the joint variable.
	*/
	void					 SetDamping(const scalar &);

	/*!
		set an upper joint limit.
	*/
	void					 SetUpperLimit(const scalar &);

	/*!
		set an upper joint limit.
	*/
	void					 SetLowerLimit(const scalar &);

	/*!
		set a restitution.
	*/
	void					 SetRestitution(const scalar &);

	/*!
		get a sliding direction of the joint.
	*/
	const Vec3				&GetDirection(void) const;

	/*!
		get a displacement of the joint variable.
	*/
	scalar					 GetDisplacement(void) const;

	/*!
		get a velocity of the joint variable.
	*/
	scalar					 GetVelocity(void) const;

	/*!
		get an acceleration of the joint variable.
	*/
	scalar					 GetAcceleration(void) const;

	/*!
		get a force of the joint.
	*/
	scalar					 GetForce(void) const;

	/*!
		get an intial displacement of the joint variable.
	*/
	scalar					 GetInitialDisplacement(void) const;

	/*!
		get an elasticity of the joint variable.
	*/
	scalar					 GetElasticity(void) const;

	/*!
		get a damping parameterof the joint variable.
	*/
	scalar					 GetDamping(void) const;

	/*!
		get a upper joint limit.
	*/
	scalar					 GetUpperLimit(void) const;

	/*!
		get an lower joint limit.
	*/
	scalar					 GetLowerLimit(void) const;

	/*!
		disable upper joint limit.
	*/
	void					 DisableUpperLimit(void);

	/*!
		disable lower joint limit.
	*/
	void					 DisableLowerLimit(void);

	/*!
		return whether this joint is set to have upper limit
	*/
	bool					 IsEnabledUpperLimit(void) const;

	/*!
		return whether this joint is set to have lower limit
	*/
	bool					 IsEnabledLowerLimit(void) const;

	virtual int				 GetDOF(void) const;
	virtual scalar			 GetNormalForce(void) const;
	virtual scalar			 GetNormalTorque(void) const;

	virtual void			 streamOut(ostream &) const;

protected:

	virtual void			 SwapBody(void);
	virtual void			 BuildKinematics(void);
	virtual SE3				 Transform(void) const;
	virtual void			 UpdateSpringDamperTorque(void);
	virtual scalar			 GetPotentialEnergy(void) const;
	virtual const scalar	&GetDisplacement_(int) const;
	virtual void			 SetDisplacement_(int, const scalar &);
	virtual const scalar	&GetVelocity_(int) const;
	virtual void			 SetVelocity_(int, const scalar &);
	virtual const scalar	&GetAcceleration_(int) const;
	virtual void			 SetAcceleration_(int, const scalar &);
	virtual const scalar	&GetImpulsiveTorque_(int) const;
	virtual void			 SetImpulsiveTorque_(int, const scalar &);
	virtual scalar			 GetTorque_(int) const;
	virtual void			 SetTorque_(int, const scalar &);
	virtual void			 SetSpringDamperTorque_(int, const scalar &);
	virtual	const scalar	&GetRestitution_(int) const;
	virtual bool			 ViolateUpperLimit_(int) const;
	virtual bool			 ViolateLowerLimit_(int) const;

	virtual void			 UpdateTorqueID(void);
	virtual void			 UpdateTorqueHD(void);
	virtual void			 UpdateVelocity(const se3 &);
	virtual void			 UpdateAccelerationID(const se3 &);
	virtual void			 UpdateAccelerationFD(const se3 &);
	virtual void			 UpdateAInertia(AInertia &);
	virtual void			 UpdateLOTP(void);
	virtual void			 UpdateTP(void);
	virtual void			 UpdateLP(void);
	virtual dse3			 GetLP(void);
	virtual void			 ClearTP(void);

	virtual void			 IntegrateDisplacement(const scalar &);
	virtual void			 IntegrateVelocity(const scalar &);

	dse3					 m_sL;
	Vec3					 m_sDir;
	scalar					 m_rQ;
	scalar					 m_rDq;
	scalar					 m_rDdq;
	scalar					 m_rActuationTau;
	scalar					 m_rSpringDamperTau;
	scalar					 m_rImpulsiveTau;
	scalar					 m_rQi;
	scalar					 m_rQul;
	scalar					 m_rQll;
	scalar					 m_rRestitution;
	scalar					 m_rK;
	scalar					 m_rC;
	bool					 m_bHasUpperLimit;
	bool					 m_bHasLowerLimit;

	se3						 m_sVl;
	scalar					 m_sO;
	scalar					 m_sT;
	scalar					 m_sP;
};

#ifndef VP_PROTECT_SRC
	#include "vpPJoint.inl"
#endif

#endif
