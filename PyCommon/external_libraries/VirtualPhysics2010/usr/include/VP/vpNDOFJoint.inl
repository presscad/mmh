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

VP_INLINE void vpNDOFJoint::SetTransformFunc(TransformNDOF *fun)
{
	assert(m_iDOF == fun->m_iDOF && "vpNDOFJoint::SetTransformFunc(TransformNDOF *) -> inconsistent DOF");
	m_pTransform = fun;
}

VP_INLINE void	vpNDOFJoint::SetPosition(int i, const scalar &x)
{
	m_rQ[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void	vpNDOFJoint::SetVelocity(int i, const scalar &x)
{
	m_rDq[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetAcceleration(int i, const scalar &x)
{
	m_rDdq[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetTorque(int i, const scalar &x)
{
	m_rActuationTau[i] = (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vpNDOFJoint::AddTorque(int i, const scalar &x)
{
	m_rActuationTau[i] += (m_eSign == VP::PLUS ? x : -x); 
}

VP_INLINE void vpNDOFJoint::SetInitialPosition(int i, const scalar &x)
{
	m_rQi[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetElasticity(int i, const scalar &x)
{
	assert(x >= SCALAR_0 && "vpNDOFJoint::SetElasticity(scalar) -> can not be negative");
	m_rK[i] = x;
}

VP_INLINE void vpNDOFJoint::SetDamping(int i, const scalar &x)
{
	assert(x >= SCALAR_0 && "vpNDOFJoint::SetDamping(scalar) -> can not be negative");
	m_rC[i] = x;
}

VP_INLINE void vpNDOFJoint::SetUpperLimit(int i, const scalar &x)
{
	m_bHasUpperLimit[i] = true;
	m_rQul[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::SetLowerLimit(int i, const scalar &x)
{
	m_bHasLowerLimit[i] = true;
	m_rQll[i] = (m_eSign == VP::PLUS ? x : -x);
}

VP_INLINE void vpNDOFJoint::DisableUpperLimit(int i)
{
	m_bHasUpperLimit[i] = false;
}

VP_INLINE void vpNDOFJoint::DisableLowerLimit(int i)
{
	m_bHasLowerLimit[i] = false;
}

VP_INLINE void vpNDOFJoint::SetRestitution(int i, const scalar &e)
{
	m_rRestitution[i] = e;
}

VP_INLINE scalar vpNDOFJoint::GetPosition(int i) const
{
	return (m_eSign == VP::PLUS ? m_rQ[i]: -m_rQ[i]);
}

VP_INLINE scalar vpNDOFJoint::GetVelocity(int i) const
{
	return (m_eSign == VP::PLUS ? m_rDq[i]: -m_rDq[i]);
}

VP_INLINE scalar vpNDOFJoint::GetAcceleration(int i) const
{
	return (m_eSign == VP::PLUS ? m_rDdq[i]: -m_rDdq[i]);
}

VP_INLINE scalar vpNDOFJoint::GetTorque(int i) const
{
	return (m_eSign == VP::PLUS ? m_rActuationTau[i]: -m_rActuationTau[i]);
}

VP_INLINE scalar vpNDOFJoint::GetInitialPosition(int i) const
{
	return (m_eSign == VP::PLUS ? m_rQi[i]: -m_rQi[i]);
}

VP_INLINE const scalar &vpNDOFJoint::GetElasticity(int i) const
{
	return m_rK[i];
}

VP_INLINE const scalar &vpNDOFJoint::GetDamping(int i) const
{
	return m_rC[i];
}

VP_INLINE int vpNDOFJoint::GetDOF(void) const
{
	return m_iDOF;
}

VP_INLINE SE3 vpNDOFJoint::Transform(void) const
{
	assert(m_pTransform && "vpNDOFJoint::Transform -> transform function is not defined");
	return m_pTransform->GetTransform(m_rQ);
}

VP_INLINE void vpNDOFJoint::UpdateSpringDamperTorque(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rSpringDamperTau[i] = m_rK[i] * (m_rQ[i] - m_rQi[i]) + m_rC[i] * m_rDq[i];
}

VP_INLINE scalar vpNDOFJoint::GetPotentialEnergy(void) const
{
	scalar sum = SCALAR_0;
	for ( int i = 0; i < m_iDOF; i++ ) sum += m_rK[i] * (m_rQ[i] - m_rQi[i]) * (m_rQ[i] - m_rQi[i]);
	return SCALAR_1_2 * sum;
}

VP_INLINE scalar vpNDOFJoint::GetNormalForce(void) const
{
	return SCALAR_0;
}

VP_INLINE scalar vpNDOFJoint::GetNormalTorque(void) const
{
	return SCALAR_0;
}

VP_INLINE void vpNDOFJoint::SetDisplacement_(int i, const scalar &x)
{
	m_rQ[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetDisplacement_(int i) const
{
	return m_rQ[i];
}

VP_INLINE void vpNDOFJoint::SetVelocity_(int i, const scalar &x)
{
	m_rDq[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetVelocity_(int i) const
{
	return m_rDq[i];
}

VP_INLINE void vpNDOFJoint::SetAcceleration_(int i, const scalar &x)
{
	m_rDdq[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetAcceleration_(int i) const
{
	return m_rDdq[i];
}

VP_INLINE void vpNDOFJoint::SetImpulsiveTorque_(int i, const scalar &x)
{
	m_rImpulsiveTau[i] = x;
}

VP_INLINE const scalar &vpNDOFJoint::GetImpulsiveTorque_(int i) const
{
	return m_rImpulsiveTau[i];
}

VP_INLINE void vpNDOFJoint::SetSpringDamperTorque_(int i, const scalar &x)
{
	m_rSpringDamperTau[i] = x;
}

VP_INLINE void vpNDOFJoint::SetTorque_(int i, const scalar &x)
{
	m_rActuationTau[i] = x;
}

VP_INLINE scalar vpNDOFJoint::GetTorque_(int i) const
{
	return (m_rActuationTau[i] - m_rSpringDamperTau[i]);
}

VP_INLINE const scalar &vpNDOFJoint::GetRestitution_(int i) const
{
	return m_rRestitution[i];
}

VP_INLINE bool vpNDOFJoint::ViolateUpperLimit_(int i) const
{
	return m_bHasUpperLimit[i] && m_rQ[i] >= m_rQul[i];
}

VP_INLINE bool	vpNDOFJoint::ViolateLowerLimit_(int i) const
{
	return m_bHasLowerLimit[i] && m_rQ[i] <= m_rQll[i];
}

VP_INLINE TransformNDOF::TransformNDOF(int dof)
{
	m_iDOF = dof;
	m_rEPS = LIE_EPS;
}

VP_INLINE bool vpNDOFJoint::IsEnabledUpperLimit(int idx) const
{
	return m_bHasUpperLimit[idx];
}

VP_INLINE bool vpNDOFJoint::IsEnabledLowerLimit(int idx) const
{
	return m_bHasLowerLimit[idx];
}

VP_INLINE void vpNDOFJoint::IntegrateDisplacement(const scalar &h)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rQ[i] += h * m_rDq[i];
}

VP_INLINE void vpNDOFJoint::IntegrateVelocity(const scalar &h)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rDq[i] += h * m_rDdq[i];
}
