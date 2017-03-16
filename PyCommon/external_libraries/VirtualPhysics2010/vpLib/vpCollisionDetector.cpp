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

#include <VP/vpCollisionDetector.h>
#include <VP/vpWorld.h>
#include <VP/vpBody.h>

vpCollisionDetector::vpCollisionDetector(void)
{
	m_iMaxNumSimulContact = 10;
}

void vpCollisionDetector::initialize(void)
{
	vpBodyPairSet collidablePair;

	for ( int i = 0; i < m_pWorld->GetNumBody(); i++ )
	{
		if ( !m_pWorld->GetBody(i)->IsCollidable() ) continue;
		for ( int j = i + 1; j < m_pWorld->GetNumBody(); j++ )
		{
			if ( !m_pWorld->GetBody(j)->IsCollidable() ) continue;

			vpBodyPair BPair(m_pWorld->GetBody(i), m_pWorld->GetBody(j));
			if ( (!BPair.pLeftBody->IsGround() || !BPair.pRightBody->IsGround()) && (m_sNonCollidablePair.find(BPair) == m_sNonCollidablePair.end()) )
				collidablePair.insert(BPair);
		}
	}

	m_sCollisionLUT.resize((int)collidablePair.size());

	int idx = 0;
	for ( vpBodyPairSet::const_iterator itor = collidablePair.begin(); itor != collidablePair.end(); itor++, idx++ )
	{
		m_sCollisionLUT[idx].pLeftBody = itor->pLeftBody;
		m_sCollisionLUT[idx].pRightBody = itor->pRightBody;
		m_sCollisionLUT[idx].colInfo.resize(m_iMaxNumSimulContact);
		for ( int j = 0; j < m_sCollisionLUT[idx].colInfo.size(); j++ ) m_sCollisionLUT[idx].colInfo[j].contactForce = SCALAR_0;
	}
}

void vpCollisionDetector::IgnoreCollision(vpBody *pB0, vpBody *pB1)
{
	m_sNonCollidablePair.insert(vpBodyPair(pB0, pB1));
}

void vpCollisionDetector::Attach(vpWorld *pWorld)
{
	m_pWorld = pWorld;
}

void vpCollisionDetector::SetSkinDepth(const scalar &d)
{
	m_rSkinDepth = d;
}

scalar vpCollisionDetector::GetSkinDepth(void) const
{
	return m_rSkinDepth;
}

void vpCollisionDetector::SetMaxNumSimulContact(int num)
{
	m_iMaxNumSimulContact = num;
}

/*
const _array<vpCollisionList> &vpCollisionDetector::GetCollisionList(void) const
{
	return m_sCollisionList;
}
*/
