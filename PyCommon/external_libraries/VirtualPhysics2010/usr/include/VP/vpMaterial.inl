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

VP_INLINE scalar vpMaterial::GetDensity(void) const
{
	return m_rDensity;
}

VP_INLINE void vpMaterial::SetDensity(scalar d)
{
	m_rDensity = d;
}

VP_INLINE scalar vpMaterial::GetRestitution(void) const
{
	return m_rRestitution;
}

VP_INLINE void vpMaterial::SetRestitution(scalar r)
{
	m_rRestitution = r;
}

VP_INLINE scalar vpMaterial::GetStaticFriction(void) const
{
	return m_rStaticFriction;
}

VP_INLINE void vpMaterial::SetStaticFriction(scalar f)
{
	m_rStaticFriction = f;
}

VP_INLINE scalar vpMaterial::GetDynamicFriction(void) const
{
	return m_rDynamicFriction;
}

VP_INLINE void vpMaterial::SetDynamicFriction(scalar f)
{
	m_rDynamicFriction = f;
}

VP_INLINE scalar vpMaterial::GetSpinningFriction(void) const
{
	return m_rSpinningFriction;
}

VP_INLINE void vpMaterial::SetSpinningFriction(scalar f)
{
	m_rSpinningFriction = f;
}
