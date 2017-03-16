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

VP_INLINE void vpWorld::SetTimeStep(scalar t)
{
	m_rTimeStep = t;
	SetContactEPS();	
}

VP_INLINE scalar vpWorld::GetSimulationTime() const
{
	return m_rTime;
}

VP_INLINE scalar vpWorld::GetTimeStep(void) const
{
	return m_rTimeStep;
}

VP_INLINE void vpWorld::EnableCollision(bool flag)
{
	m_bDetectCollision = flag;
}

VP_INLINE scalar vpWorld::GetTotalEnergy() const
{
	return GetKineticEnergy() + GetPotentialEnergy();
}

VP_INLINE void vpWorld::SetGravity(const Vec3 &g)
{
	m_sGravity = g;	
	SetContactEPS();	
}

VP_INLINE const Vec3 &vpWorld::GetGravity(void) const
{
	return m_sGravity;
}

VP_INLINE int vpWorld::GetNumBody(void) const
{
	return m_pBody.size();
}

VP_INLINE const vpBody *vpWorld::GetBody(int idx) const
{
	if ( idx < 0 || idx >= m_pBody.size() ) return NULL;
	return m_pBody[idx];
}

VP_INLINE vpBody *vpWorld::GetBody(int idx)
{
	if ( idx < 0 || idx >= m_pBody.size() ) return NULL;
	return m_pBody[idx];
}

VP_INLINE int vpWorld::GetNumMaterial(void) const
{
	return m_pMaterial.size();
}

VP_INLINE const vpMaterial *vpWorld::GetMaterial(int idx) const
{
	if ( idx < 0 || idx >= m_pMaterial.size() ) return NULL;
	return m_pMaterial[idx];
}

VP_INLINE int vpWorld::GetNumJoint(void) const
{
	return m_pJoint.size();
}

VP_INLINE const vpJoint *vpWorld::GetJoint(int idx) const
{
	if ( idx < 0 || idx >= m_pJoint.size() ) return NULL;
	return m_pJoint[idx];
}

VP_INLINE void vpWorld::SetGlobalFrame(const SE3 &T)
{
	m_sGlobalFrame = T;
}

VP_INLINE const SE3 &vpWorld::GetGlobalFrame(void) const
{
	return m_sGlobalFrame;
}

VP_INLINE int vpWorld::GetNumThreads(void)
{
	return m_iNumThreads;
}

VP_INLINE void vpWorld::SetGlobalDamping(scalar d)
{
	m_rGlobalDamping = d;
}

VP_INLINE scalar vpWorld::GetGlobalDampling(void)
{
	return m_rGlobalDamping;
}

VP_INLINE int vpWorld::GetFrameCount(void) const
{
	return m_iFrameCount;
}