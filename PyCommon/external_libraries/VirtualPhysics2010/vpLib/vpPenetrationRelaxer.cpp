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

#include <VP/vpWorld.h>
#include <VP/vpGeom.h>
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpMaterial.h>
#include <VP/vpSystem.h>
#include <VP/vpSpring.h>
#include <VP/vpCollisionDetector.h>
/*
struct ColPairCmp
{
	bool operator()(const vpCollisionDSC *s1, const vpCollisionDSC *s2) const
	{
		return Inner(s1->pLeftBody->GetFrame() * s1->leftPosition, s1->pLeftBody->GetWorld()->GetGravity()) >
			Inner(s2->pLeftBody->GetFrame() * s2->leftPosition, s2->pLeftBody->GetWorld()->GetGravity());
	}
};
*/
void vpWorld::RelaxPenetration(void)
{
	/*
	for ( int i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->m_iRelaxationCount = !m_pBody[i]->IsGround();
	
	// sort collision pairs in gravity direction order
	for ( int i = 0; i < m_sCollisionPair.size(); i++ )
	{
		set<vpCollisionDSC *, ColPairCmp> col_body_pair;
		for ( int j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			if ( m_sCollisionPair[i][j].pLeftBody->m_pSystem->GetNumJoint() == 0 && m_sCollisionPair[i][j].pRightBody->m_pSystem->GetNumJoint() == 0 )
				col_body_pair.insert(&m_sCollisionPair[i][j]);
		}

		set <vpCollisionDSC *, ColPairCmp>::iterator itor = col_body_pair.begin();
		while ( itor != col_body_pair.end() )
		{
			vpCollisionDSC *x = *itor;
			if ( (*itor)->pLeftBody->m_pSystem->m_pRoot->m_iRelaxationCount )
			//if ( !(*itor)->pLeftBody->IsGround() )
			{
				SE3 T = (*itor)->pLeftBody->m_pSystem->m_pRoot->GetFrame();
				T.SetPosition(T.GetPosition() - (*itor)->penetration * (*itor)->normal);
				(*itor)->pLeftBody->m_pSystem->m_pRoot->SetFrame(T);
				(*itor)->pLeftBody->m_pSystem->m_pRoot->m_iRelaxationCount--;
			} else if ( (*itor)->pRightBody->m_pSystem->m_pRoot->m_iRelaxationCount )
			//} else if ( !(*itor)->pRightBody->IsGround() )
			{
				SE3 T = (*itor)->pRightBody->m_pSystem->m_pRoot->GetFrame();
				T.SetPosition(T.GetPosition() + (*itor)->penetration * (*itor)->normal);
				(*itor)->pRightBody->m_pSystem->m_pRoot->SetFrame(T);
				(*itor)->pRightBody->m_pSystem->m_pRoot->m_iRelaxationCount--;
			}
			itor++;
		}
	}
	*/
}
