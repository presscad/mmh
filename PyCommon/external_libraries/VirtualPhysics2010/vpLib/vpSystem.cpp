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

#include <VP/vpSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpJoint.h>
#include <VP/vpBody.h>

vpSystem::vpSystem()
{
	m_pRoot = NULL;
}

void vpSystem::Initialize(bool init_dynamics)
{
	int i;

	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->m_pSystem = this;
	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->m_pSystem = this;
	
	for ( i = 0; i < m_pBody.size(); i++ ) 
		for ( int j = 0; j < m_pBody[i]->m_pSpring.size(); j++ ) m_pSpring.check_push_back(m_pBody[i]->m_pSpring[j]);

	BuildKinematics();

	if ( init_dynamics )
	{
		for ( i = 0; i < m_pBody.size(); i++ )
		{
			m_pBody[i]->m_pWorld = m_pWorld;
			m_pBody[i]->Initialize();
		}
		BuildDynamics();
		FDIteration1();
	}

	_JvInitialized = false;
}

void vpSystem::BackupState(void)
{
	for ( int i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->BackupState();

	m_sStateDisplacement.resize(m_sState.size());
	m_sStateVelocity.resize(m_sState.size());
	for ( int i = 0; i < m_sState.size(); i++ )
	{
		m_sStateDisplacement[i] = m_sState[i].GetDisplacement();
		m_sStateVelocity[i] = m_sState[i].GetVelocity();
	}
}

void vpSystem::RollbackState(void)
{
	for ( int i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->RollbackState();

	if ( m_sStateDisplacement.size() == m_sState.size() )
	{
		for ( int i = 0; i < m_sState.size(); i++ )
		{
			m_sState[i].SetDisplacement(m_sStateDisplacement[i]);
			m_sState[i].SetVelocity(m_sStateVelocity[i]);
		}
	}
}

void vpSystem::Reparameterize(void)
{
	for ( int i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->Reparameterize();
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpSystem.inl>
#endif

