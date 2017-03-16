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

#ifndef VP_UJOINT
#define VP_UJOINT

#include <VP/vpDataType.h>
#include <VP/vpJoint.h>

/*!
	\class vpUJoint
	\brief Universal joint class
	
	vpUJoint is a class to model a universal joint.
	<img src="\vp_site/ujoint.gif">
*/
class vpUJoint : public vpJoint
{
public:

							 vpUJoint();

	/*!
		set a rotational axis of the idx-th joint variable.
		\param idx joint variable index. Universal joint is a 2 dof joint. Hence the index should be 0 or 1.
				The axis is Vec3(1, 0, 0) for the 0-th joint variable and (0, 1, 0) for the 1-th joint variable by default.
	*/
	void					 SetAxis(int idx, const Vec3 &axis);

	/*!
		set a joint angle of the idx-th joint variable.
	*/
	void					 SetAngle(int idx, const scalar &theta);

	/*!
		set an angular velocity of the idx-th joint variable.
	*/
	void					 SetVelocity(int idx, const scalar &omega);

	/*!
		set an angualr acceleration of the idx-th joint variable.
	*/
	void					 SetAcceleration(int idx, const scalar &alpha);

	/*!
		set a joint torque of the idx-th joint variable.
	*/
	void					 SetTorque(int idx, const scalar &tau);

	/*!
		add a joint torque to the idx-th joint variable.
	*/
	void					 AddTorque(int idx, const scalar &tau);

	/*!
		set an intial angle of the idx-th joint variable.
	*/
	void					 SetInitialAngle(int idx, const scalar &theta_i);

	/*!
		set an elasticity of the idx-th joint variable.
	*/
	void					 SetElasticity(int idx, const scalar &k);

	/*!
		set an upper joint limit.
	*/
	void					 SetUpperLimit(int idx, const scalar &);

	/*!
		set an lower joint limit.
	*/
	void					 SetLowerLimit(int idx, const scalar &);

	/*!
		set a damping parameter of the idx-th joint variable.
	*/
	void					 SetDamping(int idx, const scalar &c);

	/*!
		set a restitution.
	*/
	void					 SetRestitution(int idx, const scalar &);

	/*!
		get a rotation axis of the idx-th joint variable.
	*/
	Vec3					 GetAxis(int idx) const;
	
	/*!
		get a joint angle of the idx-th joint variable.
	*/
	scalar					 GetAngle(int idx) const;

	/*!
		get an angular velocity of the idx-th joint variable.
	*/
	scalar					 GetVelocity(int idx) const;

	/*!
		get an angular acceleration of the idx-th joint variable.
	*/
	scalar					 GetAcceleration(int idx) const;

	/*!
		get a joint torque of the idx-th joint variable.
	*/
	scalar					 GetTorque(int idx) const;

	/*!
		get an intial joint angle of the idx-th joint variable.
	*/
	scalar					 GetInitialAngle(int idx) const;

	/*!
		get an elasticity of the idx-th joint variable.
	*/
	scalar					 GetElasticity(int idx) const;

	/*!
		get a damping parameter of the idx-th joint variable.
	*/
	scalar					 GetDamping(int idx) const;

	/*!
		get a upper limit
	*/
	scalar					 GetUpperLimit(int idx) const;

	/*!
		get a lower limit
	*/
	scalar					 GetLowerLimit(int idx) const;

	/*!
		disable upper limit of the idx th joint angle.
	*/
	void					 DisableUpperLimit(int idx);

	/*!
		disable lower limit of the idx th joint angle.
	*/
	void					 DisableLowerLimit(int idx);

	/*!
		return whether this joint is set to have upper limit
	*/
	bool					 IsEnabledUpperLimit(int) const;

	/*!
		return whether this joint is set to have lower limit
	*/
	bool					 IsEnabledLowerLimit(int) const;

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

	Axis					 m_sAxis[2];
	Axis					 m_sS[2];
	Axis					 m_sDSdq;
	Axis					 m_sVl;
	scalar					 m_sO[3];
	scalar					 m_sT[2];
	scalar					 m_sP[2];
};

#ifndef VP_PROTECT_SRC
	#include "vpUJoint.inl"
#endif

#endif
