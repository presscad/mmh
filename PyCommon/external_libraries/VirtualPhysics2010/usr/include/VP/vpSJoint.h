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

#ifndef VP_SJOINT
#define VP_SJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class vpSJoint
	\brief Sliding joint class
	
	vpSJoint is a class to model a sliding joint.	
*/
class vpSJoint : public vpJoint
{
public:

							 vpSJoint();

	/*!
		set a sliding axis
	*/
	void					 SetAxis(const Vec3 &axis);

	/*!
		set a rotating angle
	*/
	void					 SetAngle(const scalar &theta);

	/*!
		set a sliding displacement
	*/
	void					 SetDisplacement(const scalar &x);

	/*!
		set a rotating angular velocity
	*/
	void					 SetAngularVelocity(const scalar &w);

	/*!
		set a sliding velocity
	*/
	void					 SetVelocity(const scalar &v);

	/*!
		set a rotating angular acceleration
	*/
	void					 SetAngularAcceleration(const scalar &dw);

	/*!
		set a sliding acceleration
	*/
	void					 SetAcceleration(const scalar &a);

	/*!
		set a rotating torque
	*/
	void					 SetTorque(const scalar &tau);

	/*!
		set a sliding force
	*/
	void					 SetSlidingForce(const scalar &f);

	/*!
		add a rotating torque
	*/
	void					 AddTorque(const scalar &tau);

	/*!
		add a sliding force
	*/
	void					 AddSlidingForce(const scalar &f);

	/*!
		set an intial angle
	*/
	void					 SetInitialAngle(const scalar &theta_i);

	/*!
		set an intial displacement
	*/
	void					 SetInitialDisplacement(const scalar &x_i);

	/*!
		set a rotational elasticity
	*/
	void					 SetRotationalElasticity(const scalar &k);

	/*!
		set a linear elasticity
	*/
	void					 SetSlidingElasticity(const scalar &k);

	/*!
		set a rotational damping
	*/
	void					 SetRotationalDamping(const scalar &c);

	/*!
		set a linear damping
	*/
	void					 SetSlidingDamping(const scalar &c);

	/*!
		set an upper joint limit for rotation
	*/
	void					 SetRotationalUpperLimit(const scalar &);

	/*!
		set an upper joint limit for sliding
	*/
	void					 SetSlidingUpperLimit(const scalar &);

	/*!
		set an lower joint limit for rotation
	*/
	void					 SetRotationalLowerLimit(const scalar &);
	/*!
		set an lower joint limit for sliding
	*/
	void					 SetSlidingLowerLimit(const scalar &);

	/*!
		set a restitution for rotation
	*/
	void					 SetRotationalRestitution(const scalar &);

	/*!
		set a restitution for sliding
	*/
	void					 SetSlidingRestitution(const scalar &);

	/*!
		get a rotating axis
	*/
	const Vec3				&GetAxis(void) const;
	
	/*!
		get a rotating angle
	*/
	scalar					 GetAngle(void) const;

	/*!
		get a sliding displacement
	*/
	scalar					 GetDisplacement(void) const;

	/*!
		get an angular velocity
	*/
	scalar					 GetAngularVelocity(void) const;

	/*!
		get an sliding velocity
	*/
	scalar					 GetVelocity(void) const;

	/*!
		get an angular acceleration
	*/
	scalar					 GetAngularAcceleration(void) const;

	/*!
		get an sliding acceleration
	*/
	scalar					 GetAcceleration(void) const;

	/*!
		get a rotating torque
	*/
	scalar					 GetTorque(void) const;

	/*!
		get a sliding force
	*/
	scalar					 GetSlidingForce(void) const;

	/*!
		get an intial rotating angle
	*/
	scalar					 GetInitialAngle(void) const;

	/*!
		get an intial displacement
	*/
	scalar					 GetInitialDisplacement(void) const;

	/*!
		get an rotational elasticity
	*/
	scalar					 GetRotationalElasticity(void) const;

	/*!
		get an sliding elasticity
	*/
	scalar					 GetSlidingElasticity(void) const;

	/*!
		get a rotational damping coefficient
	*/
	scalar					 GetRotationalDamping(void) const;

	/*!
		get a sliding damping coefficient
	*/
	scalar					 GetSlidingDamping(void) const;

	/*!
		get a rotational upper limit
	*/
	scalar					 GetRotationalUpperLimit(void) const;

	/*!
		get a sliding upper limit
	*/
	scalar					 GetSlidingUpperLimit(void) const;

	/*!
		get a rotational lower limit
	*/
	scalar					 GetRotationalLowerLimit(void) const;

	/*!
		get a sliding lower limit
	*/
	scalar					 GetSlidingLowerLimit(void) const;

	/*!
		disable upper limit for rotation
	*/
	void					 DisableRotationalUpperLimit(void);

	/*!
		disable upper limit for sliding
	*/
	void					 DisableSlidingUpperLimit(void);

	/*!
		disable lower limit for rotation
	*/
	void					 DisableRotationalLowerLimit(void);

	/*!
		disable lower limit for rotation
	*/
	void					 DisableSlidingLowerLimit(void);

	/*!
		return whether this joint is set to have rotational upper limit
	*/
	bool					 IsEnabledRotationalUpperLimit(void) const;

	/*!
		return whether this joint is set to have sliding upper limit
	*/
	bool					 IsEnabledSlidingUpperLimit(void) const;

	/*!
		return whether this joint is set to have rotational lower limit
	*/
	bool					 IsEnabledRotationalLowerLimit(void) const;

	/*!
		return whether this joint is set to have sliding lower limit
	*/
	bool					 IsEnabledSlidingLowerLimit(void) const;

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

	dse3					 m_sL[2];
	scalar					 m_rQ[2];
	scalar					 m_rQi[2];
	scalar					 m_rDq[2];
	scalar					 m_rDdq[2];
	scalar					 m_rQul[2];
	scalar					 m_rQll[2];
	scalar					 m_rActuationTau[2];
	scalar					 m_rSpringDamperTau[2];
	scalar					 m_rImpulsiveTau[2];
	scalar					 m_rRestitution[2];
	scalar					 m_rK[2];
	scalar					 m_rC[2];
	bool					 m_bHasUpperLimit[2];
	bool					 m_bHasLowerLimit[2];

	Vec3					 m_sDir;
	Axis					 m_sAxis;
	se3						 m_sVl;
	scalar					 m_sO[3];
	scalar					 m_sT[2];
	scalar					 m_sP[2];
};

#ifndef VP_PROTECT_SRC
	#include "vpSJoint.inl"
#endif

#endif
