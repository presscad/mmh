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

void vpWorld::ResolveCollision(void)
{
	int n = m_sCollisionPair.size();
	if ( !n ) return;

	m_sCollisionK.resize(n);
	m_sDelV.resize(n);
	m_sP.resize(n);

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < n; i++ )
	{
		if ( m_sCollisionPair[i].size() <= 0 ) continue;

		vpBody *pLeftBody, *pRightBody;
		vpState *pState;
		vpSystem *pSystem;
		
		se3 DelV;
		Vec3 dvl, dvr;

		// m_sCollisionK[i] : corresponding the inverse of the Mass matrix for the i th collision group
		//
		// Impulse can be desribed as P = M \Delta V or \Delta V = K P --- (1)
		// , where \Delta V_k is the difference of the relative velocity of the k th collision pair and
		// P_j is the impulse applied to the j th collision pair.
		// Then K_{k,j} is \Delta V_k when P_j is unit and the others are zero.
		// Now we apply the unit impulse only at the j th collision pair and calculate \Delta V to construct the j th column of K.
		// Calculation of \Delta V can be done by solving forward dynamics ignoring velocity terms(vpSystem::FDIteration2s and vpSystem::FDIteration3s).
		// Note that the result is not the acceleration but the difference of the velocity.
		// All we keep the difference of the velocity of all the states at m_sUnitDelV and m_sUnitDelDq, which will be used later.
		// Then using a coefficient of restitution, we construct \Delta V = V^ - V^- = -(1 + e) * V^ and solve P from (1).
		//
		// Finally, we can update the velocities of all the states.
		// Note that imuplses on a dynamics system can be superposed.
		// We already have the difference of the velocity of states for each of unit impluses (m_sUnitDelV and m_sUnitDelDq).
		// Hence updated velocities of the k the state can be given by adding \sum_j m_sUnitDelV_{k,j} * P_j
	
		m_sCollisionK[i].clear((int)m_sCollisionPair[i].size(), (int)m_sCollisionPair[i].size());
		m_sDelV[i].ReNew((int)m_sCollisionPair[i].size(), 1);
		
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp parallel for
#endif
		for ( int j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[m_sCollisionPair[i][j].first];
			vpCollisionInfo &colInfoJ = testPairJ.colInfo[m_sCollisionPair[i][j].second];

			if ( testPairJ.pState )
			{
				pState = testPairJ.pState;
				pSystem = pState->m_pJoint->m_pSystem;

				pState->SetImpulsiveTorque(SCALAR_1);

				pSystem->FDIteration2s(pState->m_pJoint->m_iIdx);
				pSystem->FDIteration3s();

				for ( int k = j; k < m_sCollisionPair[i].size(); k++ )
				{
					vpCollisionDSC &testPairK = m_pCollisionDetector->m_sCollisionLUT[m_sCollisionPair[i][k].first];
					vpCollisionInfo &colInfoK = testPairK.colInfo[m_sCollisionPair[i][k].second];

					if ( testPairK.pState )
					{
						if ( pSystem == testPairK.pState->m_pJoint->m_pSystem )
						{
							m_sCollisionK[i].setValue(k, j, testPairK.pState->GetAcceleration());
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

						m_sCollisionK[i].setValue(k, j, Inner(dvr - dvl, colInfoK.normal));
					}
				}
		
				if ( pState->ViolateUpperLimit() )
				{
					if ( colInfoJ.collidingVelocity <= 0.0 )
						m_sDelV[i][j] = SCALAR_0;
					else
						m_sDelV[i][j] = -(SCALAR_1 + pState->GetRestitution()) * colInfoJ.collidingVelocity;
				} else // if ( pState->ViolateLowerLimit() )
				{
					if ( colInfoJ.collidingVelocity >= 0.0 )
						m_sDelV[i][j] = SCALAR_0;
					else
						m_sDelV[i][j] = -(SCALAR_1 + pState->GetRestitution()) * colInfoJ.collidingVelocity;
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

				//for ( int k = j; k < m_sCollisionPair[i].size(); k++ )
				for ( set<int>::iterator ktor = m_sCollisionPair[i][j].impactSet.begin(); ktor != m_sCollisionPair[i][j].impactSet.end(); ktor++ )				
				{
					int k = *ktor;
					if ( k < j ) continue;
					
					vpCollisionDSC &testPairK = m_pCollisionDetector->m_sCollisionLUT[m_sCollisionPair[i][k].first];
					vpCollisionInfo &colInfoK = testPairK.colInfo[m_sCollisionPair[i][k].second];

					if ( testPairK.pState )
					{
						if ( pLeftBody->m_pSystem == testPairK.pState->m_pJoint->m_pSystem ||
							 pRightBody->m_pSystem == testPairK.pState->m_pJoint->m_pSystem )
						{
							m_sCollisionK[i].setValue(k, j, testPairK.pState->GetAcceleration());
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

						m_sCollisionK[i].setValue(k, j, Inner(dvr - dvl, colInfoK.normal));
					}
				}

				if ( colInfoJ.collidingVelocity >= SCALAR_0 )
					m_sDelV[i][j] = SCALAR_0;
				else
					m_sDelV[i][j] = -(SCALAR_1 + min(pLeftBody->GetMaterial()->GetRestitution(), pRightBody->GetMaterial()->GetRestitution())) * colInfoJ.collidingVelocity;

				pLeftBody->ResetImpulse();
				pRightBody->ResetImpulse();
			}
		}
		
		VP_TIMER_ACTION(m_sProfilingTimer[4], Resume);

		m_sP[i] = Zeros<scalar>(m_sDelV[i].RowSize(), 1);
		//int iter = CGSolveAxEqualB(m_sCollisionK[i], m_sP[i], m_sDelV[i], m_iMaxIterSolver, 1e-3);	// faster than SOR but m_sCollisionK[i] becomes singular sometimes
		int iter = SORSolveAxEqualB(m_sCollisionK[i], m_sP[i], m_sDelV[i], 1.0, m_iMaxIterSolver, 1e-3);

#ifdef VP_PROFILING_STATISTICS
		if ( 0 < iter && iter < m_iMaxIterSolver ) 
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
			#pragma omp atomic
#endif
			m_iSuccessfulSolveAxEqualB++;

		int slot = (int)log10((scalar)m_sP[i].RowSize());
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp critical
#endif
		{
			if ( (int)m_iCollisionSizeHistogram.size() < slot + 1 ) m_iCollisionSizeHistogram.resize(slot+1);
			m_iCollisionSizeHistogram[slot]++;
			m_iNumCallSolveAxEqualB++;
		}
#endif
		if ( SquareSum(m_sP[i]) / (scalar)m_sP[i].RowSize() > 10000.0 )
			SVDSolveAxEqualB(convert(m_sCollisionK[i]), m_sP[i], m_sDelV[i], 0.01);
		else
		{
			if ( iter == -1 )
			{
				m_sP[i] = Zeros<scalar>(m_sDelV[i].RowSize(), 1);
				SORSolveAxEqualB(m_sCollisionK[i], m_sP[i], m_sDelV[i], 1.0, m_iMaxIterSolver, 1e-3);

	#ifdef VP_PROFILING_STATISTICS
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
				#pragma omp atomic
#endif
				m_iNumCallSecondSolveAxEqualB++;
	#endif
			}
		}
		VP_TIMER_ACTION(m_sProfilingTimer[4], Halt);

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp parallel for
#endif
		for ( int j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[m_sCollisionPair[i][j].first];
			vpCollisionInfo &colInfoJ = testPairJ.colInfo[m_sCollisionPair[i][j].second];

			if ( testPairJ.pState )
			{
				testPairJ.pState->SetImpulsiveTorque(m_sP[i][j]);
			} else
			{
				pLeftBody = testPairJ.pLeftBody;
				pRightBody = testPairJ.pRightBody;
				
				pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), -m_sP[i][j] * colInfoJ.normal), colInfoJ.leftPosition);
				pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), m_sP[i][j] * colInfoJ.normal), colInfoJ.rightPosition);

				//scalar dynamic_friction = min(pLeftBody->GetMaterial()->GetDynamicFriction(), pRightBody->GetMaterial()->GetDynamicFriction());

				//if ( dynamic_friction > LIE_EPS )
				//{
				//	Vec3 Vt = pPair_ij->tangentialVelocity;
				//	scalar mag = Vt.Normalize();
				//	if ( mag > LIE_EPS )
				//	{
				//		//if ( !pLeftBody->m_pJoint.size() ) 
				//			pLeftBody->ApplyLocalImpulse(InvRotate(pLeftBody->GetFrame(), (dynamic_friction * m_sP[j]) * Vt), pPair_ij->leftPosition);
				//		//if ( !pRightBody->m_pJoint.size() ) 
				//			pRightBody->ApplyLocalImpulse(InvRotate(pRightBody->GetFrame(), (-dynamic_friction * m_sP[j]) * Vt), pPair_ij->rightPosition);
				//	}
				//}

				//scalar spinning_friction = pPair_ij->spinningVelocity * min(pLeftBody->GetMaterial()->GetSpinningFriction(), pRightBody->GetMaterial()->GetSpinningFriction());

				//if ( abs(spinning_friction) > LIE_EPS )
				//{
				//	if ( !pLeftBody->m_pJoint.size() )
				//		pLeftBody->ApplyLocalImpulse((-spinning_friction * m_sP[j]) * (Axis)InvRotate(pLeftBody->GetFrame(), pPair_ij->normal));
				//	if ( !pRightBody->m_pJoint.size() )
				//		pRightBody->ApplyLocalImpulse((spinning_friction * m_sP[j]) * (Axis)InvRotate(pRightBody->GetFrame(), pPair_ij->normal));
				//}
			}
		}

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp parallel for
#endif
		for ( int j = 0; j < m_pCollisionSystem[i].size(); j++ )
		{
			m_pCollisionSystem[i][j]->FDIteration2s();
			m_pCollisionSystem[i][j]->FDIteration3s();

			for ( int k = 0; k < m_pCollisionSystem[i][j]->m_sState.size(); k++ )
				m_pCollisionSystem[i][j]->m_sState[k].SetVelocity(m_pCollisionSystem[i][j]->m_sState[k].GetVelocity() + m_pCollisionSystem[i][j]->m_sState[k].GetAcceleration());

			m_pCollisionSystem[i][j]->m_pRoot->m_sV += m_pCollisionSystem[i][j]->m_pRoot->m_sDV;
		}

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
		#pragma omp parallel for
#endif
		for ( int j = 0; j < m_sCollisionPair[i].size(); j++ )
		{
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[m_sCollisionPair[i][j].first];

			if ( testPairJ.pState )
			{
				testPairJ.pState->SetImpulsiveTorque(SCALAR_0);
			} else
			{
				testPairJ.pLeftBody->ResetImpulse();
				testPairJ.pRightBody->ResetImpulse();
			}
		}
	}
}

