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

#ifndef VP_RJOINT
#define VP_RJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class vpRJoint
	\brief Revolute joint
	
	vpRJoint is a class to model a revolute joint.
	Connected bodies can rotate relatively about a rotational axis.
	<img src="\vp_site/rjoint.gif">
*/
class vpRJoint : public vpJoint
{
public:

							 vpRJoint();

	/*!
		set a rotational axis.
		The axis is Vec3(0, 0, 1) by default.
	*/
	void					 SetAxis(const Vec3 &);

	/*!
		set an angle of the joint variable.
	*/
	void					 SetAngle(const scalar &);

	/*!
		set an angular velocity of the joint variable.
	*/
	void					 SetVelocity(const scalar &);

	/*!
		set an angular acceleration of the joint variable.
	*/
	void					 SetAcceleration(const scalar &);

	/*!
		set a torque of the joint.
		The torque will be reset after every simulation step
	*/
	void					 SetTorque(const scalar &);

	/*!
		add a torque to the joint.
		The torque will be reset after every simulation step
	*/
	void					 AddTorque(const scalar &);

	/*!
		set an initial angle of the joint variable.
	*/
	void					 SetInitialAngle(const scalar &);

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
		get a rotational axis.
	*/
	Vec3					 GetAxis(void) const;

	/*!
		get an angle of the joint variable.
	*/
	scalar					 GetAngle(void) const;

	/*!
		get an anglular velocity of the joint variable.
	*/
	scalar					 GetVelocity(void) const;

	/*!
		get an anglular acceleration of the joint variable.
	*/
	scalar					 GetAcceleration(void) const;

	/*!
		get a torque of the joint.
	*/
	scalar					 GetTorque(void) const;

	/*!
		get an initial angle of the joint variable.
	*/
	scalar					 GetInitialAngle(void) const;

	/*!
		get an elasticity of the joint variable.
	*/
	const scalar			&GetElasticity(void) const;

	/*!
		get a damping paramter of the joint variable.
	*/
	const scalar			&GetDamping(void) const;

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
	bool					 m_bUsingDefaultAxis;
	bool					 m_bHasUpperLimit;
	bool					 m_bHasLowerLimit;

	Axis					 m_sS;					// local jacobian
	Axis					 m_sVl;					// local velocity
	scalar					 m_sO;
	scalar					 m_sT;
	scalar					 m_sP;
};

#ifndef VP_PROTECT_SRC
	#include "vpRJoint.inl"
#endif

#endif
