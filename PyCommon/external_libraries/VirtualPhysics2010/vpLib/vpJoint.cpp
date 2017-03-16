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

#include <VP/vpJoint.h>
#include <VP/vpBody.h>
#include <VP/vpSystem.h>
#include <VP/vpWorld.h>

vpJoint::vpJoint() : 	m_pLeftBody(NULL),
						m_pRightBody(NULL),
						m_eSign(VP::PLUS),
						m_sV(SCALAR_0),
						m_sDV(SCALAR_0),
						m_sW(SCALAR_0),
						m_sF(SCALAR_0),
						m_sI(SCALAR_0),
						m_bBreakable(false),
						m_rMaxNormalForce(SCALAR_MAX),
						m_rMaxNormalTorque(SCALAR_MAX)
{
	Initialize();
}

void vpJoint::SetBody(VP::SIDE side, vpBody *pB, const SE3 &T)
{
	if ( side == VP::LEFT )
	{
		m_pLeftBody = pB;
		m_sLeftBodyFrame = T;
	} else
	{
		m_pRightBody = pB;
		m_sRightBodyFrame = T;
	}
}

void vpJoint::SwapBody()
{
	// swap body pointers
	vpBody *tmp = m_pLeftBody;
	m_pLeftBody = m_pRightBody;
	m_pRightBody = tmp;
	
	// swap body frames
	SE3 T = m_sLeftBodyFrame;
	m_sLeftBodyFrame = m_sRightBodyFrame;
	m_sRightBodyFrame = T;
	
	// swap m_eSign
	m_eSign = (m_eSign == VP::PLUS ? VP::MINUS : VP::PLUS);
}

vpStateArray &vpJoint::GetState(void) const
{
	return m_pSystem->m_sState;
}

void vpJoint::Break(void)
{
    if ( m_pSystem ) m_pSystem->Register2BrokenJoints(this);
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpJoint.inl>
#endif
