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

#include <VP/vpPrimitiveCollisionDetector.h>
#include <VP/vpWorld.h>
#include <VP/vpBody.h>
#include <VP/vpGeom.h>

int	vpPrimitiveCollisionDetector::m_iMaxNumContact4Box = 3;

void vpPrimitiveCollisionDetector::Initialize(void)
{
}

void vpPrimitiveCollisionDetector::DetectCollision(void)
{
	for ( int i = 0; i < m_sCollisionList.size(); i++ ) m_sCollisionList[i].clear();

	m_sCollisionList.resize(m_pWorld->GetNumThreads());
	
	int total = 0;
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for reduction(+ : total)
#endif
	for ( int i = 0; i < m_sCollisionLUT.size(); i++ )
	{
		vpCollisionDSC &pair = m_sCollisionLUT[i];

		pair.colInfo.resize(0);

		if ( !pair.pLeftBody->DetectCollisionApprox(pair.pRightBody) ) continue;
				
		for ( int j = 0; j < pair.pLeftBody->GetNumGeometry(); j++ )
		for ( int k = 0; k < pair.pRightBody->GetNumGeometry(); k++ )
			pair.pLeftBody->GetGeometry(j)->DetectCollision(pair.pRightBody->GetGeometry(k), pair.colInfo);
		
		if ( pair.colInfo.size() > 0 )
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
			m_sCollisionList[omp_get_thread_num()].push_back(i);
#else
			m_sCollisionList[0].push_back(i);
#endif
			
		
		total += pair.colInfo.size();
	}
	m_iNumConllision = total;
}

void vpPrimitiveCollisionDetector::SetMaxNumContact4Box(int n)
{
	m_iMaxNumContact4Box = n;
}

int vpPrimitiveCollisionDetector::GetMaxNumContact4Box(void)
{
	return m_iMaxNumContact4Box;
}
