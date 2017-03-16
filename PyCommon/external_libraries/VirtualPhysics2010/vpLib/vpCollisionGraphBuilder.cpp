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

inline LUTIndex makeLUTIndex(int f, int s)
{
	LUTIndex i;
	i.first = f;
	i.second = s;
	return i;
}

void vpWorld::BuildCollisionGraph(void)
{
	VP_TIMER_ACTION(m_sProfilingTimer[0], Resume);

	for ( int i = 0; i < m_pBody.size(); i++ )
	{
		m_pBody[i]->m_iCollisionID = -1;
		m_pBody[i]->m_iContactID = -1;
		//m_pBody[i]->UpdateGeomFrame();
	
		//m_pBody[i]->m_sCollisionBody.clear();
		//m_pBody[i]->m_sContactBody.clear();
		m_pBody[i]->m_sCollisionPairIdx.clear();
		m_pBody[i]->m_sContactPairIdx.clear();
	}

	for ( int i = 0; i < m_sCollisionPairL.size(); i++ ) m_sCollisionPairL[i].clear();
	for ( int i = 0; i < m_sContactPairL.size(); i++ ) m_sContactPairL[i].clear();
	m_sCollisionPairL.clear();
	m_sContactPairL.clear();
	
	VP_TIMER_ACTION(m_sProfilingTimer[0], Halt);
	VP_TIMER_ACTION(m_sProfilingTimer[1], Resume);

	for ( int i = 0; i < m_pCollisionDetector->m_sCollisionList.size(); i++ )
	{
		for ( intList::const_iterator itor = m_pCollisionDetector->m_sCollisionList[i].begin(); itor != m_pCollisionDetector->m_sCollisionList[i].end(); itor++ )
		{
			vpCollisionDSC &testPair = m_pCollisionDetector->m_sCollisionLUT[*itor];
			testPair.pState = NULL;

			for ( int j = 0; j < testPair.colInfo.size(); j++ )
			{
				vpCollisionInfo &colInfo = testPair.colInfo[j];
				
				colInfo.leftPosition = testPair.pLeftBody->GetFrame() % colInfo.point;
				colInfo.rightPosition = testPair.pRightBody->GetFrame() % colInfo.point;
				Vec3 Vr = testPair.pRightBody->GetLinVelocity(colInfo.rightPosition) - testPair.pLeftBody->GetLinVelocity(colInfo.leftPosition);
				colInfo.collidingVelocity = Inner(Vr, colInfo.normal);
				
				if ( colInfo.collidingVelocity < m_rContactEPS && colInfo.penetration > SCALAR_0 )
				{
					colInfo.tangentialVelocity = Vr - colInfo.collidingVelocity * colInfo.normal;
					colInfo.spinningVelocity = Inner(testPair.pLeftBody->GetAngVelocity() - testPair.pRightBody->GetAngVelocity(), colInfo.normal);
					
					int LEFTID = testPair.pLeftBody->m_iCollisionID;
					int RIGHTID = testPair.pRightBody->m_iCollisionID;

					//testPair.pLeftBody->m_sCollisionBody.insert(testPair.pRightBody);
					//testPair.pRightBody->m_sCollisionBody.insert(testPair.pLeftBody);

					if ( LEFTID == -1 && RIGHTID == -1 )
					{
						// Two bodys are not registered to m_sCollisionPairL.
						// Regitering the bodys to m_sCollisionPairL(m_sCollisionPairL[m_iCollisionID of the bodys]).
						// Later, we can find the body in m_sCollisionPairL by refering vpBody::m_iCollisionID.
						int n = m_sCollisionPairL.size();
						testPair.pLeftBody->m_pSystem->SetCollisionID(n);
						testPair.pRightBody->m_pSystem->SetCollisionID(n);
						m_sCollisionPairL.resize(n + 1, true);
						m_sCollisionPairL[n].push_back(makeLUTIndex(*itor, j));

						testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(n, (int)m_sCollisionPairL[n].size()-1));
						testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(n, (int)m_sCollisionPairL[n].size()-1));
					} else if ( LEFTID != -1 && RIGHTID != -1 )
					{
						// Two bodys are already registered to m_sCollisionPairL
						// All the bodys in m_sCollisionPairL[testPair.pLeftBody->m_iCollisionID] and
						// m_sCollisionPairL[testPair.pRightBody->m_iCollisionID] should have same m_iCollisionID
						// , except that one of the two bodys is not a ground.
						// In case that one of the two bodys is a ground, the bodys will be managed separately.
						// This is because collision of two groups need(should!) not be computed simulataneously.
				
						if ( LEFTID != RIGHTID )
						{
							if ( !testPair.pLeftBody->m_bIsGround && !testPair.pRightBody->m_bIsGround )
							{
								// Copy all the bodypairs that are stored in testPair.pRightBody->m_iCollisionID'th slot
								// of m_sCollisionPairL to testPair.pLeftBody->m_iCollisionID'th slot, and make empty the 
								// testPair.pRightBody->m_iCollisionID'th slot.
								for ( LUTIndexList::iterator jtor = m_sCollisionPairL[RIGHTID].begin(); jtor != m_sCollisionPairL[RIGHTID].end(); jtor++ )
								{
									vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[jtor->first];
									testPairJ.pLeftBody->m_pSystem->SetCollisionID(LEFTID);
									testPairJ.pRightBody->m_pSystem->SetCollisionID(LEFTID);

									for ( list<pair<int, int> >::iterator ktor = testPairJ.pLeftBody->m_sCollisionPairIdx.begin(); ktor != testPairJ.pLeftBody->m_sCollisionPairIdx.end(); ktor++ )
									{
										if ( ktor->first == RIGHTID )
										{
											ktor->first = LEFTID;
											ktor->second += (int)m_sCollisionPairL[LEFTID].size();
										}
									}
								
									for ( list<pair<int, int> >::iterator ktor = testPairJ.pRightBody->m_sCollisionPairIdx.begin(); ktor != testPairJ.pRightBody->m_sCollisionPairIdx.end(); ktor++ )
									{
										if ( ktor->first == RIGHTID )
										{
											ktor->first = LEFTID;
											ktor->second += (int)m_sCollisionPairL[LEFTID].size();
										}
									}
								}
								m_sCollisionPairL[LEFTID].splice(m_sCollisionPairL[LEFTID].end(), m_sCollisionPairL[RIGHTID]);
								m_sCollisionPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
							} else if ( testPair.pLeftBody->m_bIsGround )
							{
								m_sCollisionPairL[RIGHTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(RIGHTID, (int)m_sCollisionPairL[RIGHTID].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(RIGHTID, (int)m_sCollisionPairL[RIGHTID].size()-1));
							} else
							{
								m_sCollisionPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
							}
						} else
						{
							m_sCollisionPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

							testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
							testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
						}
					} else
					{
						// One of two bodys is registered to m_sCollisionPairL.
						// If the body is not a ground, non registered body will be registered to same m_iCollisionID slot in m_sCollisionPairL.
						// Else, non registered body should be separately managed.
						if ( LEFTID == -1 )
						{
							if ( testPair.pRightBody->m_bIsGround )
							{
								int n = m_sCollisionPairL.size();
								testPair.pLeftBody->m_pSystem->SetCollisionID(n);
								m_sCollisionPairL.resize(n + 1, true);
								m_sCollisionPairL[n].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(n, (int)m_sCollisionPairL[n].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(n, (int)m_sCollisionPairL[n].size()-1));	
							} else
							{
								testPair.pLeftBody->m_pSystem->SetCollisionID(RIGHTID);
								m_sCollisionPairL[RIGHTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(RIGHTID, (int)m_sCollisionPairL[RIGHTID].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(RIGHTID, (int)m_sCollisionPairL[RIGHTID].size()-1));
							}
						} else // RIGHTID == -1
						{
							if ( testPair.pLeftBody->m_bIsGround )
							{
								int n = m_sCollisionPairL.size();
								testPair.pRightBody->m_pSystem->SetCollisionID(n);
								m_sCollisionPairL.resize(n + 1, true);
								m_sCollisionPairL[n].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(n, (int)m_sCollisionPairL[n].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(n, (int)m_sCollisionPairL[n].size()-1));
							} else
							{
								testPair.pRightBody->m_pSystem->SetCollisionID(LEFTID);
								m_sCollisionPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
								testPair.pRightBody->m_sCollisionPairIdx.push_back(make_pair(LEFTID, (int)m_sCollisionPairL[LEFTID].size()-1));
							}
						}
					}
				}

				if ( abs(colInfo.collidingVelocity) < m_rContactEPS )
				{
					int LEFTID = testPair.pLeftBody->m_iContactID;
					int RIGHTID = testPair.pRightBody->m_iContactID;

					//testPair.pLeftBody->m_sContactBody.insert(testPair.pRightBody);
					//testPair.pRightBody->m_sContactBody.insert(testPair.pLeftBody);

					if ( LEFTID == -1 && RIGHTID == -1 )
					{
						int n = m_sContactPairL.size();
						testPair.pLeftBody->m_pSystem->SetContactID(n);
						testPair.pRightBody->m_pSystem->SetContactID(n);
						m_sContactPairL.resize(n + 1, true);
						m_sContactPairL[n].push_back(makeLUTIndex(*itor, j));

						testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(n, (int)m_sContactPairL[n].size()-1));
						testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(n, (int)m_sContactPairL[n].size()-1));
					} else if ( LEFTID != -1 && RIGHTID != -1 )
					{
						if ( LEFTID != RIGHTID )
						{
							if ( !testPair.pLeftBody->m_bIsGround && !testPair.pRightBody->m_bIsGround )
							{
								for ( LUTIndexList::iterator jtor = m_sContactPairL[RIGHTID].begin(); jtor != m_sContactPairL[RIGHTID].end(); jtor++ )
								{
									vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[jtor->first];

									testPairJ.pLeftBody->m_pSystem->SetContactID(LEFTID);
									testPairJ.pRightBody->m_pSystem->SetContactID(LEFTID);

									for ( list<pair<int, int> >::iterator ktor = testPairJ.pLeftBody->m_sContactPairIdx.begin(); ktor != testPairJ.pLeftBody->m_sContactPairIdx.end(); ktor++ )
									{
										if ( ktor->first == RIGHTID )
										{
											ktor->first = LEFTID;
											ktor->second += (int)m_sContactPairL[LEFTID].size();
										}
									}
								
									for ( list<pair<int, int> >::iterator ktor = testPairJ.pRightBody->m_sContactPairIdx.begin(); ktor != testPairJ.pRightBody->m_sContactPairIdx.end(); ktor++ )
									{
										if ( ktor->first == RIGHTID )
										{
											ktor->first = LEFTID;
											ktor->second += (int)m_sContactPairL[LEFTID].size();
										}
									}
								}
								m_sContactPairL[LEFTID].splice(m_sContactPairL[LEFTID].end(), m_sContactPairL[RIGHTID]);
								m_sContactPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
							} else if ( testPair.pLeftBody->m_bIsGround )
							{
								m_sContactPairL[RIGHTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(RIGHTID, (int)m_sContactPairL[RIGHTID].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(RIGHTID, (int)m_sContactPairL[RIGHTID].size()-1));
							} else
							{
								m_sContactPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
							}
						} else
						{
							m_sContactPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

							testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
							testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
						}
					} else
					{
						if ( LEFTID == -1 )
						{
							if ( testPair.pRightBody->m_bIsGround )
							{
								int n = m_sContactPairL.size();
								testPair.pLeftBody->m_pSystem->SetContactID(n);
								m_sContactPairL.resize(n + 1, true);
								m_sContactPairL[n].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(n, (int)m_sContactPairL[n].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(n, (int)m_sContactPairL[n].size()-1));
							} else
							{
								testPair.pLeftBody->m_pSystem->SetContactID(RIGHTID);
								m_sContactPairL[RIGHTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(RIGHTID, (int)m_sContactPairL[RIGHTID].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(RIGHTID, (int)m_sContactPairL[RIGHTID].size()-1));
							}
						} else
						{
							if ( testPair.pLeftBody->m_bIsGround )
							{
								int n = m_sContactPairL.size();
								testPair.pRightBody->m_pSystem->SetContactID(n);
								m_sContactPairL.resize(n + 1, true);
								m_sContactPairL[n].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(n, (int)m_sContactPairL[n].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(n, (int)m_sContactPairL[n].size()-1));
							} else
							{
								testPair.pRightBody->m_pSystem->SetContactID(LEFTID);
								m_sContactPairL[LEFTID].push_back(makeLUTIndex(*itor, j));

								testPair.pLeftBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
								testPair.pRightBody->m_sContactPairIdx.push_back(make_pair(LEFTID, (int)m_sContactPairL[LEFTID].size()-1));
							}
						}
					}
				}
			}
		}
	}
	/*
	// joint limit check
	for ( int i = 0; i < m_pSystem.size(); i++ )
	{
		for ( int j = 0; j < m_pSystem[i]->m_sState.size(); j++ )
		{
			vpState &state = m_pSystem[i]->m_sState[j];

			if ( state.ViolateUpperLimit() || state.ViolateLowerLimit() )
			{
				testPair.collidingVelocity = testPair.pState->GetVelocity();
			
				testPair.pLeftBody = testPair.pRightBody = NULL;
			
				if ( (testPair.pState->ViolateUpperLimit() && testPair.collidingVelocity > -m_rContactEPS) ||
					 (testPair.pState->ViolateLowerLimit() && testPair.collidingVelocity < m_rContactEPS) )
				{
					if ( testPair.pState->m_pJoint->m_pLeftBody->m_iCollisionID == -1 )
					{
						int n = m_sCollisionPairL.size();
						testPair.pState->m_pJoint->m_pSystem->SetCollisionID(n);
						m_sCollisionPairL.resize(n + 1, true);
						m_sCollisionPairL[n].push_back(testPair);
					} else
						m_sCollisionPairL[testPair.pState->m_pJoint->m_pLeftBody->m_iCollisionID].push_back(testPair);
				}
			
				if ( abs(testPair.collidingVelocity) < m_rContactEPS )
				{
					if ( testPair.pState->m_pJoint->m_pLeftBody->m_iContactID == -1 )
					{
						int n = m_sContactPairL.size();
						testPair.pState->m_pJoint->m_pSystem->SetContactID(n);
						m_sContactPairL.resize(n + 1, true);
						m_sContactPairL[n].push_back(testPair);
					} else
						m_sContactPairL[testPair.pState->m_pJoint->m_pLeftBody->m_iContactID].push_back(testPair);
				}
			}
		}
	}
	// end of joint limit check
	*/
	
	VP_TIMER_ACTION(m_sProfilingTimer[1], Halt);
	VP_TIMER_ACTION(m_sProfilingTimer[2], Resume);

	for ( int i = 0; i < m_pCollisionSystem.size(); i++ ) m_pCollisionSystem[i].clear();
	m_pCollisionSystem.resize(m_sCollisionPairL.size());
	
	for ( int i = 0; i < m_pCollisionSystem.size(); i++ ) 
	{
		for ( LUTIndexList::iterator jtor = m_sCollisionPairL[i].begin(); jtor != m_sCollisionPairL[i].end(); jtor++ )
		{
			vpCollisionDSC &testPair = m_pCollisionDetector->m_sCollisionLUT[jtor->first];

			if ( testPair.pState )
				m_pCollisionSystem[i].check_push_back(testPair.pState->m_pJoint->m_pSystem);
			else
			{
				m_pCollisionSystem[i].check_push_back(testPair.pLeftBody->m_pSystem);
				m_pCollisionSystem[i].check_push_back(testPair.pRightBody->m_pSystem);
			}
		}
	}

	for ( int i = 0; i < m_sCollisionPair.size(); i++ ) m_sCollisionPair[i].clear();
	m_sCollisionPair.resize(m_sCollisionPairL.size());

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_sCollisionPairL.size(); i++ )
	{
		m_sCollisionPair[i].resize((int)m_sCollisionPairL[i].size());

		int j = 0;
		for ( LUTIndexList::iterator jtor = m_sCollisionPairL[i].begin(); jtor != m_sCollisionPairL[i].end(); jtor++, j++ )
		{
			m_sCollisionPair[i][j] = *jtor;
			
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[jtor->first];
			m_sCollisionPair[i][j].impactSet.clear();
			m_sCollisionPair[i][j].impactSet.insert(j);

			if ( !testPairJ.pLeftBody->IsGround() )
				for ( list<pair<int, int> >::iterator ktor = testPairJ.pLeftBody->m_sCollisionPairIdx.begin(); ktor != testPairJ.pLeftBody->m_sCollisionPairIdx.end(); ktor++ )
					m_sCollisionPair[i][j].impactSet.insert(ktor->second);
				
			if ( !testPairJ.pRightBody->IsGround() )
				for ( list<pair<int, int> >::iterator ktor = testPairJ.pRightBody->m_sCollisionPairIdx.begin(); ktor != testPairJ.pRightBody->m_sCollisionPairIdx.end(); ktor++ )
		 			m_sCollisionPair[i][j].impactSet.insert(ktor->second);
		} 
	}
	
	VP_TIMER_ACTION(m_sProfilingTimer[2], Halt);
	VP_TIMER_ACTION(m_sProfilingTimer[3], Resume);

	for ( int i = 0; i < m_pContactSystem.size(); i++ ) m_pContactSystem[i].clear();
	m_pContactSystem.resize(m_sContactPairL.size());

	for ( int i = 0; i < m_pContactSystem.size(); i++ ) 
	{
		m_pContactSystem[i].clear();

		for ( LUTIndexList::iterator jtor = m_sContactPairL[i].begin(); jtor != m_sContactPairL[i].end(); jtor++ )
		{
			vpCollisionDSC &testPair = m_pCollisionDetector->m_sCollisionLUT[jtor->first];

			if ( testPair.pState )
				m_pContactSystem[i].check_push_back(testPair.pState->m_pJoint->m_pSystem);
			else
			{
				m_pContactSystem[i].check_push_back(testPair.pLeftBody->m_pSystem);
				m_pContactSystem[i].check_push_back(testPair.pRightBody->m_pSystem);
			}
		}
	}

	for ( int i = 0; i < m_sContactPair.size(); i++ ) m_sContactPair[i].clear();
	m_sContactPair.resize(m_sContactPairL.size());

#if !defined(__APPLE__) || defined(__APPLE_OMP__)
	#pragma omp parallel for
#endif
	for ( int i = 0; i < m_sContactPair.size(); i++ )
	{
		m_sContactPair[i].resize((int)m_sContactPairL[i].size());
		int j = 0;
		for ( LUTIndexList::iterator jtor = m_sContactPairL[i].begin(); jtor != m_sContactPairL[i].end(); jtor++, j++ )
		{
			m_sContactPair[i][j] = *jtor;
			
			vpCollisionDSC &testPairJ = m_pCollisionDetector->m_sCollisionLUT[jtor->first];
			m_sContactPair[i][j].impactSet.clear();
			m_sContactPair[i][j].impactSet.insert(j);

			if ( !testPairJ.pLeftBody->IsGround() )
				for ( list<pair<int, int> >::iterator ktor = testPairJ.pLeftBody->m_sContactPairIdx.begin(); ktor != testPairJ.pLeftBody->m_sContactPairIdx.end(); ktor++ )
					m_sContactPair[i][j].impactSet.insert(ktor->second);
			
			if ( !testPairJ.pRightBody->IsGround() )
				for ( list<pair<int, int> >::iterator ktor = testPairJ.pRightBody->m_sContactPairIdx.begin(); ktor != testPairJ.pRightBody->m_sContactPairIdx.end(); ktor++ )
					m_sContactPair[i][j].impactSet.insert(ktor->second);
		}
	}

#ifdef VP_PROFILING_STATISTICS
	for ( int i = 0; i < m_sCollisionPair.size(); i++ )
	{
		m_iNumCollisionGroup++;
		m_iMaxCollisionSize = max(m_iMaxCollisionSize, m_sCollisionPair[i].size());
	}
	
	for ( int i = 0; i < m_sContactPair.size(); i++ )
	{
		m_iNumContactGroup++;
		m_iMaxContactSize = max(m_iMaxContactSize, m_sContactPair[i].size());
	}
	
	VP_TIMER_ACTION(m_sProfilingTimer[3], Halt);
#endif
}
