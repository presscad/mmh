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

VP_INLINE int vpSingleSystem::GetNumJoint(void) const
{
	return 0;
}

VP_INLINE int vpSingleSystem::GetNumLoop(void) const
{
	return 0;
}

VP_INLINE int vpSingleSystem::GetNumBody(void) const
{
	return 0;
}

VP_INLINE vpSingleSystem::vpSingleSystem()
{
}


VP_INLINE vpBody *vpSingleSystem::GetRoot(void)
{
	return m_pRoot;
}

VP_INLINE void vpSingleSystem::UpdateFrame(bool)
{
}

VP_INLINE void vpSingleSystem::SetCollisionID(int k)
{
	m_pRoot->m_iCollisionID = k;
}

VP_INLINE void vpSingleSystem::SetContactID(int k)
{
	m_pRoot->m_iContactID = k;
}

VP_INLINE void vpSingleSystem::FDIteration2(void)
{
	m_sRootBias.dad(m_pRoot->m_sV, m_pRoot->m_sI * m_pRoot->m_sV);
	m_sRootBias += m_pRoot->GetForce();
}

VP_INLINE void vpSingleSystem::FDIteration2s(void)
{
	m_sRootBias = m_pRoot->GetImpulse();
}

VP_INLINE void vpSingleSystem::FDIteration2s(int)
{
	m_sRootBias = m_pRoot->GetImpulse();
}

VP_INLINE void vpSingleSystem::FDIteration2s(vpBody *)
{
	m_sRootBias = m_pRoot->GetImpulse();
}

VP_INLINE void vpSingleSystem::FDIteration3(void)
{
	m_pRoot->m_sDV = m_sRootInvInertia * m_sRootBias;
}

VP_INLINE void vpSingleSystem::FDIteration3s(void)
{
	m_pRoot->m_sDV = m_sRootInvInertia * m_sRootBias;
}

VP_INLINE void vpSingleSystem::IntegrateDynamicsEuler(scalar time_step)
{
	ForwardDynamics();
	
	m_pRoot->m_sFrame *= Exp(time_step * m_pRoot->m_sV);
	
	m_pRoot->m_sV += time_step * m_pRoot->m_sDV;
}

VP_INLINE int vpSingleGroundSystem::GetNumJoint(void) const
{
	return 0;
}

VP_INLINE int vpSingleGroundSystem::GetNumLoop(void) const
{
	return 0;
}

VP_INLINE int vpSingleGroundSystem::GetNumBody(void) const
{
	return 0;
}

VP_INLINE scalar vpSingleGroundSystem::GetKineticEnergy(void) const
{
	return SCALAR_0;
}

VP_INLINE scalar vpSingleGroundSystem::GetPotentialEnergy(void) const
{
	return SCALAR_0;
}

VP_INLINE void vpSingleGroundSystem::BackupState(void)
{
}

VP_INLINE void vpSingleGroundSystem::RollbackState(void)
{
}

VP_INLINE vpSingleGroundSystem::vpSingleGroundSystem()
{
}

VP_INLINE vpBody *vpSingleGroundSystem::GetRoot(void)
{
	return m_pRoot;
}

VP_INLINE void vpSingleGroundSystem::UpdateFrame(bool)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2(void)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2s(void)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2s(int)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration2s(vpBody *)
{
}

VP_INLINE void vpSingleGroundSystem::FDIteration3(void)
{	
}

VP_INLINE void vpSingleGroundSystem::FDIteration3s(void)
{
}

VP_INLINE void vpSingleGroundSystem::ForwardDynamics(void)
{
}

VP_INLINE void vpSingleGroundSystem::IntegrateDynamicsEuler(scalar time_step)
{
}

VP_INLINE void vpSingleGroundSystem::IntegrateDynamicsRK4(scalar time_step)
{
}

VP_INLINE void vpSingleGroundSystem::SetCollisionID(int k)
{
	m_pRoot->m_iCollisionID = k;
}

VP_INLINE void vpSingleGroundSystem::SetContactID(int k)
{
	m_pRoot->m_iContactID = k;
}

