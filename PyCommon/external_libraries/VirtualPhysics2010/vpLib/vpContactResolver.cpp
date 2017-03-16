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

// verify if a = Af + b, 0 < a \perf f > 0
void verifyLCP(const _rmatrix<scalar> &A, const RMatrix &f, const RMatrix &b)
{
	RMatrix a = A * f + b;
	scalar err = SCALAR_0;
	for ( int i = 0; i < a.RowSize(); i++ )
	{
		if ( a[i] < -1e-2 )
		{
			cout << "SORSolveLCP wrong solution a = " << ~a;
			return;
		}
		if ( f[i] < -1e-2 )
		{
			cout << "SORSolveLCP wrong solution f = " << ~f;
			return;
		}
		err += abs(a[i] * f[i]);
	}

	if ( err > 1e-2 )
		cout << "SolveLCP wrong solution sum(abs(a[i] * f[i])) = " << err << "  " << b.RowSize() << endl;
}

void vpWorld::ResolveContact(void)
{
	int n = m_sContactPair.size();
	if ( !n ) return;

	m_sContactK.resize(m_sContactPair.size());

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < n; i++ )
	{
		if ( m_sContactPair[i].size() <= 0 ) continue;

		vpBody *pLeftBody, *pRightBody;
		vpState *pState;
		vpSystem *pSystem;

		Vec3 dvl, dvr;

		m_sContactK[i].clear((int)m_sContactPair[i].size(), (int)m_sContactPair[i].size());

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp parallel for
#endif
		for ( int j = 0; j < m_sContactPair[i].size(); j++ )
		{
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[m_sContactPair[i][j].first];
			vpCollisionInfo &colInfoJ = testPairJ.colInfo[m_sContactPair[i][j].second];

			if ( testPairJ.pState )
			{
				pState = testPairJ.pState;
				pSystem = pState->m_pJoint->m_pSystem;
				pState->SetImpulsiveTorque(SCALAR_1);
				pSystem->FDIteration2s(pState->m_pJoint->m_iIdx);
				pSystem->FDIteration3s();

				for ( int k = j; k < m_sContactPair[i].size(); k++ )
				{
					vpCollisionDSC &testPairK = m_pCollisionDetector->m_sCollisionLUT[m_sContactPair[i][k].first];
					vpCollisionInfo &colInfoK = testPairK.colInfo[m_sContactPair[i][k].second];

					if ( testPairK.pState )
					{
						if ( pSystem == testPairK.pState->m_pJoint->m_pSystem )
						{
							m_sContactK[i].setValue(k, j, testPairK.pState->GetAcceleration());
						}
					} else
					{
						if ( pSystem == testPairK.pLeftBody->m_pSystem )
							dvl = testPairK.pLeftBody->GetLinAccWithZeroVel(colInfoK.leftPosition);
						else
							dvl = SCALAR_0;
						
						if ( pSystem == testPairK.pRightBody->m_pSystem )
							dvr = testPairK.pRightBody->GetLinAccWithZeroVel(colInfoK.rightPosition);
						else
							dvr = SCALAR_0;

						m_sContactK[i].setValue(k, j, Inner(dvr - dvl, colInfoK.normal));
					}
				}

				pState->SetImpulsiveTorque(SCALAR_0);
			} else
			{
				pLeftBody = testPairJ.pLeftBody;
				pRightBody = testPairJ.pRightBody;
				
				pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), -colInfoJ.normal), colInfoJ.leftPosition);
				pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), colInfoJ.normal), colInfoJ.rightPosition);
				
				if ( pLeftBody->m_pSystem == pRightBody->m_pSystem )
				{
					if ( pLeftBody->m_iIdx > pRightBody->m_iIdx )
						pLeftBody->m_pSystem->FDIteration2s(pLeftBody);
					else
						pLeftBody->m_pSystem->FDIteration2s(pRightBody);
					pLeftBody->m_pSystem->FDIteration3s();
				} else
				{
					pLeftBody->m_pSystem->FDIteration2s(pLeftBody);
					pLeftBody->m_pSystem->FDIteration3s();
					pRightBody->m_pSystem->FDIteration2s(pRightBody);
					pRightBody->m_pSystem->FDIteration3s();
				}

				//for ( int k = j; k < m_sContactPair[i].size(); k++ )
				for ( set<int>::iterator ktor = m_sContactPair[i][j].impactSet.begin(); ktor != m_sContactPair[i][j].impactSet.end(); ktor++ )
				{
					int k = *ktor;
					if ( k < j ) continue;

					vpCollisionDSC &testPairK = m_pCollisionDetector->m_sCollisionLUT[m_sContactPair[i][k].first];
					vpCollisionInfo &colInfoK = testPairK.colInfo[m_sContactPair[i][k].second];

					if ( testPairK.pState )
					{
						if ( pLeftBody->m_pSystem == testPairK.pState->m_pJoint->m_pSystem ||
							pRightBody->m_pSystem == testPairK.pState->m_pJoint->m_pSystem )
						{
							m_sContactK[i].setValue(k, j, testPairK.pState->GetAcceleration());
						}
					} else
					{
						if ( pLeftBody->m_pSystem == testPairK.pLeftBody->m_pSystem ||
							pRightBody->m_pSystem == testPairK.pLeftBody->m_pSystem )
							dvl = testPairK.pLeftBody->GetLinAccWithZeroVel(colInfoK.leftPosition);
						else
							dvl = SCALAR_0;
						
						if ( pLeftBody->m_pSystem == testPairK.pRightBody->m_pSystem ||
							pRightBody->m_pSystem == testPairK.pRightBody->m_pSystem )
							dvr = testPairK.pRightBody->GetLinAccWithZeroVel(colInfoK.rightPosition);
						else
							dvr = SCALAR_0;

						m_sContactK[i].setValue(k, j, Inner(dvr - dvl, colInfoK.normal));
					}
				}
		
				pLeftBody->ResetImpulse();
				pRightBody->ResetImpulse();
			}
		}
	}
	
	m_sB.resize(m_sContactPair.size());
	
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_pSystem.size(); i++ ) m_pSystem[i]->ForwardDynamics();
	
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_sContactPair.size(); i++ )
	{
		if ( m_sContactPair[i].size() <= 0 ) continue;

		vpBody *pLeftBody, *pRightBody;
		m_sB[i].ReNew((int)m_sContactPair[i].size(), 1);

		for ( int j = 0; j < m_sContactPair[i].size(); j++ )
		{
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[m_sContactPair[i][j].first];
			vpCollisionInfo &colInfoJ = testPairJ.colInfo[m_sContactPair[i][j].second];

			if ( testPairJ.pState )
			{
				m_sB[i][j] = testPairJ.pState->GetAcceleration();
			} else
			{
				pLeftBody = testPairJ.pLeftBody;
				pRightBody = testPairJ.pRightBody;

				// m_sB[i][j] < 0 indicates that the bodies are accelerated to interpenetrate.
				m_sB[i][j] = Inner(pRightBody->GetLinAcceleration(colInfoJ.rightPosition) - pLeftBody->GetLinAcceleration(colInfoJ.leftPosition), colInfoJ.normal);
			}
		}
	}

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_sContactPair.size(); i++ )
	{
		if ( m_sContactPair[i].size() <= 0 ) continue;
		
		vpBody *pLeftBody, *pRightBody;

		RMatrix f((int)m_sContactPair[i].size(), 1);
		
		for ( int j = 0; j < m_sContactPair[i].size(); j++ )
		{
			vpCollisionInfo &colInfoJ = m_pCollisionDetector->m_sCollisionLUT[m_sContactPair[i][j].first].colInfo[m_sContactPair[i][j].second];
			f[j] =  colInfoJ.contactForce;
		}
		
		VP_TIMER_ACTION(m_sProfilingTimer[5], Resume);

		int iter = SORSolveLCP(m_sContactK[i], f, m_sB[i], 1.0, m_iMaxIterSolver, 1e-2);
		
		//int iter = SORSolveLCP(m_sContactK[i], f, m_sB[i], 1.5, 100, 1e-2);
		//int iter = CGSolveLCP(m_sContactK[i], f, m_sB[i], m_sB[i].RowSize(), 1e-3);

		VP_TIMER_ACTION(m_sProfilingTimer[5], Halt);

#ifdef VP_PROFILING_STATISTICS
		if ( 0 < iter && iter < m_iMaxIterSolver ) 
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
			#pragma omp atomic
#endif
			m_iSuccessfulSORSolveLCP++;

		int slot = (int)log10((scalar)m_sB[i].RowSize());
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp critical
#endif
		{
			if ( (int)m_iContactSizeHistogram.size() < slot + 1 ) m_iContactSizeHistogram.resize(slot+1);
			m_iContactSizeHistogram[slot]++;
			m_iNumCallSORSolveLCP++;
		}
#endif

		//cout << ~f << ~(m_sContactK[i] * f + m_sB[i]);
		//verifyLCP(convert(m_sContactK[i]), f, m_sB[i]);

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp parallel for
#endif
		for ( int j = 0; j < m_sContactPair[i].size(); j++ )
		{
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[m_sContactPair[i][j].first];
			vpCollisionInfo &colInfoJ = testPairJ.colInfo[m_sContactPair[i][j].second];

			colInfoJ.contactForce = f[j];

			if ( testPairJ.pState )
			{
				testPairJ.pState->SetTorque(f[j]);
			} else
			{
				pLeftBody = testPairJ.pLeftBody;
				pRightBody = testPairJ.pRightBody;
				
				pLeftBody->ApplyLocalForce(InvRotate(pLeftBody->GetFrame(), -f[j] * colInfoJ.normal), colInfoJ.leftPosition);
				pRightBody->ApplyLocalForce(InvRotate(pRightBody->GetFrame(), f[j] * colInfoJ.normal), colInfoJ.rightPosition);
				
				scalar frictional_coeff = max(pLeftBody->GetMaterial()->GetDynamicFriction(), pRightBody->GetMaterial()->GetDynamicFriction());

				if ( frictional_coeff != SCALAR_0 )
				{
					Vec3 Vt = colInfoJ.tangentialVelocity;
					scalar mag = Vt.Normalize();
					if ( mag > LIE_EPS )
					{
						pLeftBody->ApplyLocalForce(InvRotate(pLeftBody->GetFrame(), frictional_coeff * f[j] * Vt), colInfoJ.leftPosition);
						pRightBody->ApplyLocalForce(InvRotate(pRightBody->GetFrame(), -frictional_coeff * f[j] * Vt), colInfoJ.rightPosition);
					}
				}
			}			
		}
	}
}
