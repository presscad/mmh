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

VP_INLINE vpWJoint::vpWJoint()
{
}

VP_INLINE int vpWJoint::GetDOF(void) const
{
	return 0;
}

VP_INLINE scalar vpWJoint::GetPotentialEnergy(void) const
{
	return SCALAR_0;
}

VP_INLINE scalar vpWJoint::GetNormalForce(void) const
{
	return sqrt(m_sF[3] * m_sF[3] + m_sF[4] * m_sF[4] + m_sF[5] * m_sF[5]);
}

VP_INLINE scalar vpWJoint::GetNormalTorque(void) const
{
	return sqrt(m_sF[0] * m_sF[0] + m_sF[1] * m_sF[1] + m_sF[2] * m_sF[2]);
}

VP_INLINE SE3 vpWJoint::Transform(void) const
{
	return SE3();
}

VP_INLINE void	vpWJoint::BuildKinematics(void)
{
}

VP_INLINE void	vpWJoint::UpdateSpringDamperTorque(void)
{
}

VP_INLINE const scalar &vpWJoint::GetDisplacement_(int) const
{
	assert(0 && "vpWJoint::GetDisplacement_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetDisplacement_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetDisplacement_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetVelocity_(int) const
{
	assert(0 && "vpWJoint::GetVelocity_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetVelocity_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetVelocity_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetAcceleration_(int) const
{
	assert(0 && "vpWJoint::GetAcceleration_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetAcceleration_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetAcceleration_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetImpulsiveTorque_(int) const
{
	assert(0 && "vpWJoint::GetImpulsiveTorque_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetImpulsiveTorque_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetImpulsiveTorque_() -> cannot reach here");
}

VP_INLINE void vpWJoint::SetTorque_(int idx, const scalar &)
{
	assert(0 && "vpWJoint::SetTorque_() -> cannot reach here");
}

VP_INLINE scalar vpWJoint::GetTorque_(int) const
{
	assert(0 && "vpWJoint::GetTorque_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE void vpWJoint::SetSpringDamperTorque_(int, const scalar &)
{
	assert(0 && "vpWJoint::SetSpringDamperTorque_() -> cannot reach here");
}

VP_INLINE const scalar &vpWJoint::GetRestitution_(int) const
{
	assert(0 && "vpWJoint::GetRestitution_() -> cannot reach here");
	return m_rNothing;
}

VP_INLINE bool vpWJoint::ViolateUpperLimit_(int) const
{
	return false;
}

VP_INLINE bool	vpWJoint::ViolateLowerLimit_(int) const
{
	return false;
}

VP_INLINE void vpWJoint::UpdateTorqueID(void)
{
}

VP_INLINE void vpWJoint::UpdateTorqueHD(void)
{
}

VP_INLINE void	vpWJoint::UpdateVelocity(const se3 &)
{
}

VP_INLINE void	vpWJoint::UpdateAccelerationID(const se3 &)
{
}

VP_INLINE void	vpWJoint::UpdateAccelerationFD(const se3 &)
{
}

VP_INLINE void vpWJoint::UpdateAInertia(AInertia &)
{
}

VP_INLINE void vpWJoint::UpdateLOTP(void)
{
}

VP_INLINE void vpWJoint::UpdateTP(void)
{
}

VP_INLINE void vpWJoint::UpdateLP(void)
{
}

VP_INLINE dse3 vpWJoint::GetLP(void)
{
	return dse3(SCALAR_0);
}

VP_INLINE void vpWJoint::ClearTP(void)
{
}

VP_INLINE void vpWJoint::IntegrateDisplacement(const scalar &)
{
}

VP_INLINE void vpWJoint::IntegrateVelocity(const scalar &)
{
}
