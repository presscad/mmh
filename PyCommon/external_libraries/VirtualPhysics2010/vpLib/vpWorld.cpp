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
#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpSystem.h>
#include <VP/vpSingleSystem.h>
#include <VP/vpSpring.h>
#include <VP/vpMaterial.h>
#include <VP/vpPrimitiveCollisionDetector.h>
#include <VP/vpIntegrator.h>

#include <stdio.h>
#include <stdarg.h>

#ifndef max
	#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

vpWorld::vpWorld()
{
	m_bDetectCollision = true;
	m_bIsInitialized = false;
	m_rTime = SCALAR_0;
	m_iFrameCount = 0;
	m_rTimeStep = (scalar)0.01;
	m_sGravity = SCALAR_0;
	m_rGlobalDamping = SCALAR_1;
	m_iNumThreads = 1;
	m_iMaxIterSolver = 20;
	IntegrateDynamics = &vpSystem::IntegrateDynamicsRK4;

	SetContactEPS();
	m_pCollisionDetector = new vpPrimitiveCollisionDetector;
	
#ifdef VP_PROFILING_STATISTICS
	m_bFirstReportStatistics = true;
	m_iResetFrameCount = 0;
	m_iMaxCollisionSize = 0;
	m_iNumCallSolveAxEqualB = 0;
	m_iSuccessfulSolveAxEqualB = 0;
	m_iNumCallSecondSolveAxEqualB = 0;	
	m_iNumCollisionGroup = 0;
	m_iMaxContactSize = 0;
	m_iNumCallSORSolveLCP = 0;
	m_iSuccessfulSORSolveLCP = 0;
	m_iNumContactGroup = 0;
#endif
}

void vpWorld::SetNumThreads(int n)
{
	int available_cores;
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel 
		available_cores = omp_get_num_threads();

	m_iNumThreads = 0 < n && n <= available_cores ? n : available_cores;
#else
	m_iNumThreads = 1;
#endif
}

void vpWorld::SetContactEPS(void)
{
	m_rContactEPS = 1.01 * m_rTimeStep * sqrt(SquareSum(m_sGravity));
}

vpWorld::~vpWorld()
{
	for ( int i = 0; i < m_pSystem.size(); i++ ) delete m_pSystem[i];
	delete m_pCollisionDetector;
}

scalar vpWorld::GetBoundingSphere(Vec3 &center) const
{
	scalar radius = SCALAR_0;
	center = SCALAR_0;
	
	if ( m_pBody.size() )
	{	
		center = m_pBody[0]->GetFrame().GetPosition();
		radius = m_pBody[0]->GetBoundingSphereRadius();

		for ( int i = 1; i < m_pBody.size(); i++ )
		{
			Vec3 dir = m_pBody[i]->GetFrame().GetPosition() - center;
			scalar dist = Norm(dir);

			if ( dist + m_pBody[i]->GetBoundingSphereRadius() > radius )
			{
				if ( dist + radius < m_pBody[i]->GetBoundingSphereRadius() )
				{
					center = m_pBody[i]->GetFrame().GetPosition();
					radius = m_pBody[i]->GetBoundingSphereRadius();
				} else
				{
					center += ((dist + m_pBody[i]->GetBoundingSphereRadius() - radius) / (SCALAR_2 * dist)) * dir;
					radius = SCALAR_1_2 * (radius  + m_pBody[i]->GetBoundingSphereRadius() + dist);
				}
			}
		}
	}

	return radius;
}

scalar vpWorld::GetKineticEnergy() const
{
	scalar sum = SCALAR_0;
	for ( int i = 0; i < m_pSystem.size(); i++ ) sum += m_pSystem[i]->GetKineticEnergy();
	return sum;
}

scalar vpWorld::GetPotentialEnergy() const
{
	scalar sum = SCALAR_0;
	for ( int i = 0; i < m_pSystem.size(); i++ ) sum += m_pSystem[i]->GetPotentialEnergy();
	for ( int i = 0; i < m_pSpring.size(); i++ ) sum += m_pSpring[i]->GetPotentialEnergy();
	return sum;
}

void vpWorld::FindAdjacentBodies(vpJoint *pJointPrev, vpBody *pBodyCurrent, vpBodyPtrArray &pBody, vpJointPtrArray &pJoint, vpBodyPtrSet &pGround)
{
	pBodyCurrent->m_iIdx = pBody.size();
	pBody.push_back(pBodyCurrent);
	
	for ( int i = 0; i < pBodyCurrent->m_pJoint.size(); i++ )
	{
		vpJoint *pJointCurrent = pBodyCurrent->m_pJoint[i];
		pJoint.check_push_back(pJointCurrent);
		pJointCurrent->m_iIdx = pJoint.find(pJointCurrent);
		if ( pJointCurrent != pJointPrev )
		{
			if ( pJointCurrent->m_pRightBody == pBodyCurrent ) pJointCurrent->SwapBody();
			pJointCurrent->m_pParentJoint = pJointPrev;
			if ( pJointPrev ) pJointPrev->m_pChildJoints.push_back(pJointCurrent);
			
			if ( pBody.find(pJointCurrent->m_pRightBody) == -1 )
			{
				if ( !pJointCurrent->m_pRightBody->m_bIsGround ) FindAdjacentBodies(pJointCurrent, pJointCurrent->m_pRightBody, pBody, pJoint, pGround);
				else pGround.insert(pJointCurrent->m_pRightBody);
			} else 
				assert(0 && "A closed loop chain is not allowed!");
		}
	}
}

void vpWorld::Initialize(void)
{
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	omp_set_num_threads(m_iNumThreads);
#endif

	int i, j, k;
	vpSystem *pSystem;
	vpBody *pBody;

	for ( i = 0; i < m_pJoint.size(); i++ ) m_pJoint[i]->Initialize();
	m_pBody.clear();
	m_pJoint.clear();
	for ( i = 0; i < m_pSystem.size(); i++ ) delete m_pSystem[i];
	m_pSystem.clear();

	vpBodyPtrSet registeredBody = m_pRegisteredBody;

	while ( registeredBody.size() )
	{
		pBody = *registeredBody.begin();

		int idx = m_pBody.find(pBody);
		if ( idx != -1 )
		{
			registeredBody.erase(pBody);
			continue;
		}

		if ( pBody->m_pJoint.empty() )
		{
			if ( pBody->m_bIsGround )	pSystem = new vpSingleGroundSystem;
			else						pSystem = new vpSingleSystem;

			m_pBody.push_back(pBody);
			pSystem->m_pRoot = pBody;
			pSystem->m_pBody.push_back(pBody);
			pSystem->m_pWorld = this;
			m_pSystem.push_back(pSystem);
		} else if ( pBody->m_bIsGround )
		{
			for ( j = 0; j < pBody->m_pJoint.size(); j++ )
			{
				vpJoint *pJthJoint = pBody->m_pJoint[j];
				if ( m_pJoint.find(pJthJoint) >= 0 ) continue;

				pSystem = new vpSystem;
				
				if ( pJthJoint->m_pRightBody == pBody ) pJthJoint->SwapBody();
				pJthJoint->m_pParentJoint = NULL;
	
				vpBody *pNextBody = pJthJoint->m_pRightBody;

				vpBodyPtrSet pGounds;
				FindAdjacentBodies(pJthJoint, pNextBody, pSystem->m_pBody, pSystem->m_pJoint, pGounds);
				pSystem->m_pBody.push_back(pBody);

				if ( pGounds.size() )
				{
					// found loop
					pSystem->m_pBody.push_back(*pGounds.begin());
				}
				
				for ( k = 0; k < pSystem->m_pBody.size(); k++ )
					if ( pSystem->m_pBody[k] != pBody ) m_pBody.push_back(pSystem->m_pBody[k]);

				pSystem->m_pRoot = pBody;
				pSystem->m_pWorld = this;
				m_pSystem.push_back(pSystem);
				m_pJoint.push_back(pSystem->m_pJoint);
			}
			m_pBody.push_back(pBody);
		} else
		{
			pSystem = new vpSystem;
			vpBodyPtrSet pGround;
			FindAdjacentBodies(NULL, pBody, pSystem->m_pBody, pSystem->m_pJoint, pGround);
			if ( pGround.size() )
			{
				for ( j = 0; j < pSystem->m_pJoint.size(); j++ ) pSystem->m_pJoint[j]->Initialize();
				registeredBody.erase(pBody);
				registeredBody.insert(pGround.begin(), pGround.end());
				delete pSystem;
				continue;
			}
			
			m_pBody.push_back(pSystem->m_pBody);
			pSystem->m_pRoot = pSystem->m_pBody[0];
			pSystem->m_pWorld = this;
			m_pSystem.push_back(pSystem);
			m_pJoint.push_back(pSystem->m_pJoint);
		}
	}

	for ( i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->Initialize(true);

	m_pMaterial.clear();
	for ( i = 0; i < m_pBody.size(); i++ ) 
	{
		for ( j = 0; j < m_pBody[i]->m_pSpring.size(); j++ ) 
			m_pSpring.check_push_back(m_pBody[i]->m_pSpring[j]);

		m_pMaterial.check_push_back(m_pBody[i]->GetMaterial());

		if ( !m_pBody[i]->m_szName.empty() ) m_sBodyNameTable[m_pBody[i]->m_szName] = m_pBody[i];

		m_pBody[i]->SetFrame(m_sGlobalFrame * m_pBody[i]->GetFrame());
	}

	for ( i = 0; i < m_pJoint.size(); i++ )
		if ( !m_pJoint[i]->m_szName.empty() ) m_sJointNameTable[m_pJoint[i]->m_szName] = m_pJoint[i];

	for ( i = 0; i < m_pMaterial.size(); i++ )
		if ( !m_pMaterial[i]->m_szName.empty() ) m_sMaterialNameTable[m_pMaterial[i]->m_szName] = m_pMaterial[i];

	m_pCollisionDetector->Attach(this);
	m_pCollisionDetector->initialize();
}

const vpBody *vpWorld::GetBodyByName(const string &name) const
{
	map<string, const vpBody *>::const_iterator itor = m_sBodyNameTable.begin();
	while ( itor != m_sBodyNameTable.end() )
	{
		if ( itor->first == name ) return itor->second;
		itor++;
	}
	return NULL;
}

const vpJoint *vpWorld::GetJointByName(const string &name) const
{
	map<string, const vpJoint  *>::const_iterator itor = m_sJointNameTable.begin();
	while ( itor != m_sJointNameTable.end() )
	{
		if ( itor->first == name ) return itor->second;
		itor++;
	}
	return NULL;
}

const vpMaterial *vpWorld::GetMaterialByName(const string &name) const
{
	map<string, const vpMaterial *>::const_iterator itor = m_sMaterialNameTable.begin();
	while ( itor != m_sMaterialNameTable.end() )
	{
		if ( itor->first == name ) return itor->second;
		itor++;
	}
	return NULL;
}

void vpWorld::AddBody(vpBody *pBody)
{
	if ( pBody ) m_pRegisteredBody.insert(pBody);
}

void vpWorld::SetIntegrator(VP::INTEGRATOR_TYPE type)
{
	switch ( type )
	{
	case VP::RK4:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsRK4;
		break;
	case VP::EULER:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsEuler;
		break;
	case VP::IMPLICIT_EULER:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsBackwardEuler;
		break;
	case VP::IMPLICIT_EULER_FAST:
		IntegrateDynamics = &vpSystem::IntegrateDynamicsBackwardEulerFast;
		break;
	}
}

void vpWorld::StepAhead(void)
{
	BreakJoints();

	VP_TIMER_ACTION(m_sStepAheadTimer, Resume);

	m_rTime += m_rTimeStep;
	m_iFrameCount++;

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_pSystem.size(); i++ )
	{
		//if ( m_pSystem[i]->m_pRoot->m_sContactBody.size() ) 
			m_pSystem[i]->m_pRoot->m_sV *= m_rGlobalDamping;
		
		for ( int j = 0; j < m_pSystem[i]->m_sState.size(); j++ ) m_pSystem[i]->m_sState[j].SetVelocity(m_rGlobalDamping * m_pSystem[i]->m_sState[j].GetVelocity());
	}

	VP_TIMER_ACTION(m_sDynamicsTimer, Resume);
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_pSystem.size(); i++ ) (m_pSystem[i]->*IntegrateDynamics)(m_rTimeStep);
	VP_TIMER_ACTION(m_sDynamicsTimer, Halt);

	for ( int i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->ResetForce();
	
	for ( int i = 0; i < m_pSystem.size(); i++ )
		for ( int j = 0; j < m_pSystem[i]->m_sState.size(); j++ ) m_pSystem[i]->m_sState[j].SetTorque(SCALAR_0);

	if ( m_bDetectCollision )
	{
		for ( int i = 0; i < m_pBody.size(); i++ )
			m_pBody[i]->UpdateGeomFrame();
	
		VP_TIMER_ACTION(m_sColDetTimer, Resume);
		m_pCollisionDetector->DetectCollision();
		VP_TIMER_ACTION(m_sColDetTimer, Halt);

		VP_TIMER_ACTION(m_sColGraphTimer, Resume);
		BuildCollisionGraph();
		VP_TIMER_ACTION(m_sColGraphTimer, Halt);

		VP_TIMER_ACTION(m_sColResTimer, Resume);
		ResolveCollision();
		VP_TIMER_ACTION(m_sColResTimer, Halt);

		VP_TIMER_ACTION(m_sConResTimer, Resume);
		ResolveContact();
		VP_TIMER_ACTION(m_sConResTimer, Halt);
		
		VP_TIMER_ACTION(m_sRelaxationTimer, Resume);
		//RelaxPenetration();
		VP_TIMER_ACTION(m_sRelaxationTimer, Halt);
	}

	VP_TIMER_ACTION(m_sStepAheadTimer, Halt);
}

void vpWorld::IgnoreCollision(vpBody *pB1, vpBody *pB2)
{
	if ( pB1 != pB2 ) m_pCollisionDetector->IgnoreCollision(pB1, pB2);
}

void vpWorld::BreakJoints(void)
{
	for ( int i = 0; i < m_pSystem.size(); i++ )
	{
		while ( m_pSystem[i]->m_pBrokenJoint.size() )
		{
			vpJoint *pJoint = *m_pSystem[i]->m_pBrokenJoint.begin();
			
			pJoint->m_pLeftBody->RemoveJoint(pJoint);
			pJoint->m_pRightBody->RemoveJoint(pJoint);
			
			AddBody(pJoint->m_pRightBody);
			Initialize();

			/*vpSystem *pSystem = new vpSystem;
			pSystem->m_pRoot = pJoint->m_pRightBody;
			pSystem->m_pWorld = this;
			m_pSystem.push_back(pSystem);

			pJoint->m_pLeftBody->RemoveJoint(pJoint);
			pJoint->m_pRightBody->RemoveJoint(pJoint);
			pJoint->m_pSystem = NULL;

			AddBody(pJoint->m_pRightBody);
			Initialize();
			*/

			m_pSystem[i]->m_pBrokenJoint.erase(pJoint);
		}
	}
}

static char _vpLogFileName[1024] = "vp.log";

void SetLogFileName(const char *name)
{
	strcpy(_vpLogFileName, name);
}

void VP::LogInfo(const char *str, ...)
{
	char buf[4096];
	va_list va;

	if ( !str ) return;

	va_start(va, str);
	vsprintf(buf, str, va);
	va_end(va);

	FILE *fout = fopen(_vpLogFileName, "a");
	fprintf(fout, buf);
	fclose(fout);
}

void vpWorld::BackupState(void)
{
	for ( int i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->BackupState();
}

void vpWorld::RollbackState(void)
{
	int i, j;
	
	for ( i = 0; i < m_pBody.size(); i++ ) m_pBody[i]->ResetForce();
	
	for ( i = 0; i < m_pSystem.size(); i++ )
		for ( j = 0; j < m_pSystem[i]->m_sState.size(); j++ ) m_pSystem[i]->m_sState[j].SetTorque(SCALAR_0);

	for ( i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->RollbackState();

	//Initialize();
}

void vpWorld::Clear(void)
{
}

void vpWorld::UpdateFrame(void)
{
	for ( int i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->UpdateFrame();
}

void vpWorld::AddWorld(vpWorld *pWorld)
{
	if ( pWorld )
	{
		for ( vpBodyPtrSet::const_iterator itor = pWorld->m_pRegisteredBody.begin(); itor != pWorld->m_pRegisteredBody.end(); itor++ )
			AddBody(*itor);
	}
}

int vpWorld::GetNumGeometry(void) const
{
	int num = 0;
	for ( int i = 0; i < m_pBody.size(); i++ ) num += m_pBody[i]->GetNumGeometry();
	return num;
}

int vpWorld::GetNumCollision(void) const
{
	int num = 0;
	for ( int i = 0; i < m_sCollisionPair.size(); i++ ) num += (int)m_sCollisionPair[i].size();
	return num;
}

int vpWorld::GetNumContact(void) const
{
	int num = 0;
	for ( int i = 0; i < m_sContactPair.size(); i++ ) num += (int)m_sContactPair[i].size();
	return num;
}

void vpWorld::ReportStatistics(void)
{
#ifdef VP_PROFILING_STATISTICS
	if ( m_bFirstReportStatistics )
	{
		m_bFirstReportStatistics = false;
	char timebuf[26];
	time_t ltime;
	time(&ltime);
    ctime_s(timebuf, 26, &ltime);
    VP::LogInfo("\nVP run on %s", timebuf);
    VP::LogInfo("------------------------------------------------------------------------\n");

	VP::LogInfo("#body                             = %i\n", m_pBody.size());
	VP::LogInfo("#joint                            = %i\n", m_pJoint.size());
	VP::LogInfo("#cores used                       = %i\n", m_iNumThreads);	
	VP::LogInfo("simulation time step              = %f\n", m_rTimeStep);
	VP::LogInfo("max# iteration for solvers        = %i\n", m_iMaxIterSolver);
	VP::LogInfo("#collision lookup table           = %i\n", m_pCollisionDetector->m_sCollisionLUT.size());
	VP::LogInfo("max# points for box-box collision = %i\n", vpPrimitiveCollisionDetector::GetMaxNumContact4Box());	
	}

	VP::LogInfo("\n");
	VP::LogInfo("#frames simulated                   = [%i, %i]\n", m_iResetFrameCount, m_iFrameCount);
	VP::LogInfo("  total computation time            = %4.3f sec\n", m_sStepAheadTimer.Read());
	VP::LogInfo("    solving dynamics                = %4.3f sec(%3.2f percent)\n", m_sDynamicsTimer.Read(), 100 * m_sDynamicsTimer.Read() / m_sStepAheadTimer.Read());
	VP::LogInfo("    detecting collision             = %4.3f sec(%3.2f percent)\n", m_sColDetTimer.Read(), 100 * m_sColDetTimer.Read() / m_sStepAheadTimer.Read());
	VP::LogInfo("    building collision graph        = %4.3f sec(%3.2f percent)\n", m_sColGraphTimer.Read(), 100 * m_sColGraphTimer.Read() / m_sStepAheadTimer.Read());
	VP::LogInfo("    resolving collision             = %4.3f sec(%3.2f percent)\n", m_sColResTimer.Read(), 100 * m_sColResTimer.Read() / m_sStepAheadTimer.Read());
	VP::LogInfo("    resolving contact               = %4.3f sec(%3.2f percent)\n", m_sConResTimer.Read(), 100 * m_sConResTimer.Read() / m_sStepAheadTimer.Read());
	
	VP::LogInfo("\n");
	VP::LogInfo("  #collision group                  = %i\n", m_iNumCollisionGroup);
	VP::LogInfo("  max size of collision matrix      = %i\n", m_iMaxCollisionSize);
	VP::LogInfo("  collision matrix size histogram   = ");
	for ( unsigned int i = 0; i < m_iCollisionSizeHistogram.size(); i++ ) VP::LogInfo("%i ", m_iCollisionSizeHistogram[i]);
	VP::LogInfo("\n");
	VP::LogInfo("  #call(SolveAxEqualB)              = %i\n", m_iNumCallSolveAxEqualB);
	VP::LogInfo("    success rate                    = %3.2f percent\n", 100.0 * (scalar)m_iSuccessfulSolveAxEqualB / (scalar)m_iNumCallSolveAxEqualB);
	//VP::LogInfo("    #call(SecondSolveAxEqualB)      = %3.2f percent\n", 100.0 * (scalar)m_iNumCallSecondSolveAxEqualB / (scalar)m_iNumCallSolveAxEqualB);
	
	VP::LogInfo("\n");
	VP::LogInfo("  #contact group                    = %i\n", m_iNumCollisionGroup);
	VP::LogInfo("  max size of contact matrix        = %i\n", m_iMaxContactSize);
	VP::LogInfo("  contact matrix size histogram     = ");
	for ( unsigned int i = 0; i < m_iContactSizeHistogram.size(); i++ ) VP::LogInfo("%i ", m_iContactSizeHistogram[i]);
	VP::LogInfo("\n");
	VP::LogInfo("  #call(SORSolveLCP)                = %i\n", m_iNumCallSORSolveLCP);
	VP::LogInfo("    success rate                    = %3.2f percent\n", 100.0 * (scalar)m_iSuccessfulSORSolveLCP / (scalar)m_iNumCallSORSolveLCP);

#else
	VP::LogInfo("cannot report profiling statistics because VP_PROFILING_STATISTICS is not defined\n");
#endif
}

void vpWorld::ResetStatistics(void)
{
#ifdef VP_PROFILING_STATISTICS
	m_iResetFrameCount = m_iFrameCount;
	m_sStepAheadTimer.Reset();
	m_sDynamicsTimer.Reset();
	m_sColGraphTimer.Reset();
	m_sColDetTimer.Reset();
	m_sColResTimer.Reset();
	m_sConResTimer.Reset();
	m_sRelaxationTimer.Reset();
	for ( int i = 0; i < 10; i++ ) m_sProfilingTimer[i].Reset();

	m_iMaxCollisionSize = 0;
	m_iNumCallSolveAxEqualB = 0;
	m_iSuccessfulSolveAxEqualB = 0;
	m_iNumCallSecondSolveAxEqualB = 0;	
	m_iNumCollisionGroup = 0;
	m_iMaxContactSize = 0;
	m_iNumCallSORSolveLCP = 0;
	m_iSuccessfulSORSolveLCP = 0;
	m_iNumContactGroup = 0;
	m_iCollisionSizeHistogram.clear();
	m_iContactSizeHistogram.clear();
#endif
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpWorld.inl>
#endif
