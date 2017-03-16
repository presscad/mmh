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

#include <VP/vpSpring.h>
#include <VP/vpBody.h>

vpSpring::vpSpring() :	m_rSpringCoef(SCALAR_0),
						m_rDampingCoef(SCALAR_0),
						m_rInitialDistance(SCALAR_0)
{
	m_pLeftBody = m_pRightBody = NULL;
}

void vpSpring::Connect(vpBody *LB, vpBody *RB, const Vec3 &LP, const Vec3 &RP)
{
	assert(LB && RB && "vpSpring::Connect() -> invalid bodies");

	m_pLeftBody = LB;
	m_pRightBody = RB;
	m_sLeftBodyPosition = LP;
	m_sRightBodyPosition = RP;

	m_pLeftBody->AddSpring(this);
	m_pRightBody->AddSpring(this);

	m_rInitialDistance = Norm(LB->GetFrame() * LP - RB->GetFrame() * RP);
}

void vpSpring::Remove(void)
{
	//assert(m_pLeftBody && m_pRightBody && "vpSpring::Remove() -> invalid bodies");

	if ( m_pLeftBody ) m_pLeftBody->RemoveSpring(this);
	if ( m_pRightBody ) m_pRightBody->RemoveSpring(this);
}

void vpSpring::UpdateForce(void)
{
	Vec3 d  = m_pLeftBody->GetFrame() * m_sLeftBodyPosition;
	d -= m_pRightBody->GetFrame() * m_sRightBodyPosition;
	
	scalar dx = d.Normalize() - m_rInitialDistance;

	Vec3 V	= m_pLeftBody->GetLinVelocity(m_sLeftBodyPosition);
	V -= m_pRightBody->GetLinVelocity(m_sRightBodyPosition);
	
	scalar dV = Inner(d, V);
	
	Vec3 f = (m_rSpringCoef * dx + m_rDampingCoef * dV) * d;
	
	m_pLeftBody->ApplyGlobalForce(-f, m_sLeftBodyPosition);
	m_pRightBody->ApplyGlobalForce(f, m_sRightBodyPosition);


	m_rPotentialEnergy = SCALAR_1_2 * m_rSpringCoef * dx * dx;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpSpring.inl>
#endif
