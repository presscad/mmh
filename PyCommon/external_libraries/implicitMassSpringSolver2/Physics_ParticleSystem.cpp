// Physics_Physics_ParticleSystem.cpp: implementation of the Physics_Physics_ParticleSystem class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"
#include "Physics_Impulses.h"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define FLOOR_Y	((Physics_t)-0.99)
#define FLOOR_Y	((Physics_t)30)

Physics_ParticleSystem::Physics_ParticleSystem( int iParticles ) :
	m_Positions( iParticles ), m_Velocities( iParticles ),
	m_dv( iParticles ), m_dp( iParticles ), 
	m_vTemp1( iParticles ), m_vTemp2( iParticles ), m_PosTemp( iParticles ),
	m_TotalForces_int( iParticles ), m_TotalForces_ext( iParticles ),
	m_W( iParticles ), m_H( iParticles ),
	m_Masses( iParticles ), m_MassInv( iParticles ),
	m_MxMasses( iParticles, iParticles ),
	m_TotalForces_dp( iParticles, iParticles ),
	m_TotalForces_dv( iParticles, iParticles ),
	m_A( iParticles, iParticles ),
	m_P( iParticles ),
	m_PInv( iParticles ),
	m_MxTemp1( iParticles, iParticles ), m_MxTemp2( iParticles, iParticles ),
	m_z( iParticles ), m_b( iParticles ), m_r( iParticles ), 
	m_c( iParticles ), m_q( iParticles ), m_s( iParticles ), m_y( iParticles ),
	mContacts(iParticles),
	m_vContactForce(iParticles)
{
	m_iIntegrationMethod = INTEGRATE_SEMI_IMPLICIT;

	m_cfg.m_dynamicFrictionCoef.Resize(iParticles);
	m_cfg.m_dynamicFrictionCoef.Zero();
	m_cfg.DFthresholdVel=0.1;

	for( int i=0; i<iParticles; i++ )
	{
		m_Masses.m_pData[i] = Physics_Vector3( 1,1,1 );
		m_MassInv.m_pData[i] = Physics_Vector3( 1,1,1 );
		m_MxMasses(i,i).SetIdentity();
	}

	// ys
	m_cfg.m_staticFrictionCoef.Resize(iParticles);
	m_cfg.m_staticFrictionCoef.Zero();
	m_ConstraintsYS.resize(iParticles);
	for(int i=0; i<iParticles; ++i)
		m_ConstraintsYS[i] = new Physics_Constraint(i,3);

	m_S = new Physics_Matrix3x3[iParticles];
	m_iParticles = iParticles;


	m_vContactForce.Zero();
	m_TotalForces_int.Zero();
	m_TotalForces_ext.Zero();
	m_LastStep = 1.0;
}

Physics_ParticleSystem::~Physics_ParticleSystem()
{
	SAFE_DELETE_ARRAY( m_S );

	// ys
	for(int i=0; i<m_iParticles; ++i)
		delete m_ConstraintsYS[i];

	for(int i=0; i<m_Forces.size(); i++)
		delete m_Forces[i];
}

void Physics_ParticleSystem::Update( float fTime )
{
	switch( m_iIntegrationMethod )
	{
		case INTEGRATE_EXPLICIT:
			Update_Explicit( fTime );
			break;
		case INTEGRATE_IMPLICIT:
			Update_Implicit( fTime );
			break;
		case INTEGRATE_SEMI_IMPLICIT:
			Update_SemiImplicit( fTime );
			break;
	}
}

void Physics_ParticleSystem::SetupMatrices()
{
	
	Physics_Force *pForce ;
	//DWORD dwTimeIn = GetTickCount(), dwTimeOut;

	if( m_iIntegrationMethod != INTEGRATE_EXPLICIT )
	{
		m_H.Zero();

		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{			
			m_Forces[i]->PrepareMatrices(m_H, m_TotalForces_dp);
		}	
	}

	//
	// Setup the implicit matrices
	//
	if( m_iIntegrationMethod == INTEGRATE_IMPLICIT )
	{
		m_TotalForces_dv.Copy( m_TotalForces_dp );
		m_A.Copy( m_TotalForces_dp );
		m_MxTemp1.Copy( m_TotalForces_dp );
		m_MxTemp2.Copy( m_TotalForces_dp );
	}

	//
	// Setup the semi-implicit matrices
	//
	if( m_iIntegrationMethod == INTEGRATE_SEMI_IMPLICIT )
	{
		m_W.SetIdentity();
		m_H.Scale( (Physics_t)1.0/m_ParticleMass, m_H );
		m_W.Subtract( m_H, m_W );
		m_W.Invert();
	}


	//dwTimeOut = GetTickCount();
	char szTemp[50];
	//sprintf( szTemp, "Initialize: %lu msec\r\n", dwTimeOut - dwTimeIn );
	//OutputDebugString( szTemp );

}

//
// Use Explicit integration with Deformation Constraints
//
void Physics_ParticleSystem::Update_Explicit( float fTime )
{
	Physics_t fTotal = 0.0f, fStep = (Physics_t)fTime;
	Physics_Vector3 vCOG, dTorque, tmp, tmp2;
	int i;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.Zero();
		m_TotalForces_ext.Zero();


		//
		// Apply the forces
		//
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{			
			m_Forces[i]->Apply(fStep, m_Masses, false, m_Positions, m_Velocities,
						  m_TotalForces_int, m_TotalForces_ext,
						  m_TotalForces_dp, m_TotalForces_dv );
		}


		//
		// Compute the new velocities and positions
		//
		m_TotalForces_ext.Add( m_TotalForces_int, m_vTemp1 );
		m_vTemp1.ElementMultiply( m_MassInv, m_dv );
		m_dv.Scale( fStep, m_dv );
		m_Velocities.Add( m_dv, m_Velocities );
		m_Velocities.Scale( fStep, m_vTemp1 );
		m_Positions.Add( m_vTemp1, m_PosTemp );

		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
			for(int i=0, ni=m_Forces.size(); i<ni; i++)
			{
				m_Forces[i]->Fixup( m_Masses, m_PosTemp );
			}
		}
		//
		// Update velocity and position
		//
		m_PosTemp.Subtract( m_Positions, m_vTemp1 );
		m_vTemp1.Scale( (Physics_t)1.0 / fStep, m_Velocities );
		/*for( i=0; i<m_iParticles; i++ )
			if( m_PosTemp.m_pData[i].y < FLOOR_Y )
			{
				m_PosTemp.m_pData[i].y = FLOOR_Y;
				m_Velocities.m_pData[i].y = 0;
			}*/
		m_Positions = m_PosTemp;

		fTotal += (Physics_t)fabs( fStep );
	}
}


//#define TS_FRICTION
//#define YS_FRICTION
//#define YS_FRICTION2
#define YS_FRICTION3

#ifdef TS_FRICTION
void Physics_ParticleSystem::Update_Implicit( float fTime )
{
	int i, iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3;
	Physics_t fTotal = (Physics_t)0.0f, fStep = (Physics_t)fTime;
	Physics_t alpha, Delta_0, Delta_old, Delta_new;
	Physics_t Eps_Sq = (Physics_t)1e-22;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.Zero();
		m_TotalForces_ext.Zero();
		m_TotalForces_dp.Zero();
		m_TotalForces_dv.Zero();
		m_MxTemp1.Zero();
		m_MxTemp2.Zero();

		// Setting m_S and m_y
		Reset();


		//
		// Apply the forces
		//
		void *Pos = NULL;
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply( fStep, m_Masses, true, m_Positions, m_Velocities, 
				m_TotalForces_int, m_TotalForces_ext, 
				m_TotalForces_dp, m_TotalForces_dv );
		}

		//printf("ext\n");
		//m_TotalForces_ext.Dump();
		//printf("int\n"); 
		//m_TotalForces_int.Dump();

		for(int i=0; i<m_iParticles; i++)
		{
			bool bApplyFrictionForce=false;
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				//cout << VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])<< endl;
				if(VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])<0.0001)
					//if(0)
				{
					// 땅으로 부터 잡아당기는 힘이 작용하는 경우, constraint을 풀어준다.
					m_y[i]=mContacts.deltaPosition[i];
					VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);
					mContacts.contactStatus[i]=Physics_Contacts::no_contact;
					continue;
				}
				else
					bApplyFrictionForce=true;
			}

			if(mContacts.contactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 dir;
				Physics_Vector3 p=mContacts.normal[i];
				p.Normalize();

				// dynamic friction
				Physics_Matrix3x3 pp, I;
				pp.SetFromOuterProduct(p, p);
				I.SetIdentity();
				I.Subtract(pp, m_S[i]);

				m_y[i]=mContacts.deltaPosition[i];

				// correct positions and velocities - 꺼내서 다음 프레임에 충돌 체크가 되지 않도록 한다.

				VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);

				//VECTOR3_ADD(m_Velocities[i], mImpulse.desiredDeltaVelocity[i], m_Velocities[i]);

				// 수직방향 속도.
				double dn=VECTOR3_DP(m_Velocities[i], p);

				// 수평방향 속도.				
				//VECTOR3_SUB(m_Velocities[i], p*dn, m_Velocities[i]);
				//	m_Velocities[i].y=0;

				/*if(mContacts.relativeVelocity[i].Length() <=m_cfg.DFthresholdVel && m_cfg.m_dynamicFrictionCoef[i].x >0.0)
				{
				// static friction
				m_S[i].Zero();
				bApplyFrictionForce=false;
				}*/

			}

			mContacts.saveContactsInformation(i);

			if(bApplyFrictionForce && VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])>0.0001)
			{
				// Apply friction force
				Physics_Vector3 frictionForce(0,0,0);

				Physics_Vector3 vRelVel=mContacts.prevRelativeVelocity[i];
				Physics_Vector3 p=mContacts.prevNormal[i];

				double relveldotp=VECTOR3_DP(vRelVel,p);
				double relVel=vRelVel.Length();

				if(relVel>m_cfg.DFthresholdVel)
				{
					// calc frictionForce
					Physics_Vector3 dir, temp;
					VECTOR3_CROSSPRODUCT(vRelVel, p, temp);
					temp.Normalize();
					VECTOR3_CROSSPRODUCT(p, temp, dir);
					dir.Normalize();

					double normalForce=VECTOR3_DP(p, m_vContactForce[i]);

					double mu=m_cfg.m_dynamicFrictionCoef[i].x;
					VECTOR3_SCALE(dir, mu*normalForce*-1, frictionForce);
				}

				// apply external forces
				VECTOR3_ADD(m_TotalForces_ext[i], frictionForce, m_TotalForces_ext[i]); 
			}
		}


		//
		// Form the symmetric matrix A = M - h * df/dv - h^2 * df/dx (Baraff 논문의 equation 6의 양변에 M을 곱한식)
		// We regroup this as A = M - h * (df/dv + h * df/dx)
		//
		m_TotalForces_int.Add( m_TotalForces_ext, m_TotalForces_int );
		m_TotalForces_dp.Scale( fStep, m_MxTemp1 );
		m_TotalForces_dv.Add( m_MxTemp1, m_MxTemp2 );
		m_MxTemp2.Scale( fStep, m_MxTemp1 );
		m_MxMasses.Subtract( m_MxTemp1, m_A );

		//
		// Compute b = h * ( f(0) + h * df/dx * v(0) + df/dx * y )
		//
		m_Velocities.Scale( fStep, m_vTemp2 );
		m_vTemp2.Add( m_y, m_vTemp2 );
		m_TotalForces_dp.PostMultiply( m_vTemp2, m_vTemp1 );
		//		m_vTemp1.Scale( fStep, m_vTemp2 );
		m_TotalForces_int.Add( m_vTemp1, m_vTemp1 );
		m_vTemp1.Scale( fStep, m_b );

		//
		// Setup the inverse of preconditioner -- We use a vector for memory efficiency.  
		// Technically it's the diagonal of a matrix
		//
		for( i=0; i<m_iParticles; i++ )
		{
			m_PInv.m_pData[i].x = (Physics_t)m_A(i,i).m_Mx[0];
			m_PInv.m_pData[i].y = (Physics_t)m_A(i,i).m_Mx[4];
			m_PInv.m_pData[i].z = (Physics_t)m_A(i,i).m_Mx[8];
		}
		m_PInv.Invert( m_P );

		//
		// Modified Preconditioned Conjugate Gradient method
		//

		m_dv = m_z;

		// delta_0 = DotProduct( filter( b ), P * filter( b ) );
		m_b.ElementMultiply( m_S, m_vTemp1 );
		m_P.ElementMultiply( m_vTemp1, m_vTemp2 );
		Delta_0 = m_vTemp2.DotProduct( m_vTemp1 );
		if( Delta_0 < 0 )
		{
			m_b.Dump( "b:\r\n" );
			m_P.Dump( "P:\r\n" );
			//OutputDebugString( "Negative Delta_0 most likely caused by a non-Positive Definite matrix\r\n" );
		}

		// r = filter( b - A * dv )
		m_A.PostMultiply( m_dv, m_vTemp1 );
		m_b.Subtract( m_vTemp1, m_vTemp2 );
		m_vTemp2.ElementMultiply( m_S, m_r );

		// c = filter( Pinv * r )
		m_PInv.ElementMultiply( m_r, m_vTemp1 );
		m_vTemp1.ElementMultiply( m_S, m_c );

		Delta_new = m_r.DotProduct( m_c );

		if( Delta_new < Eps_Sq * Delta_0 )
		{
			m_b.Dump( "b: \r\n" );
			m_P.Dump( "P: \r\n" );
			//OutputDebugString( "This isn't good!  Probably a non-Positive Definite matrix\r\n" );
		}

		while( (Delta_new > Eps_Sq * Delta_0) && (iIterations < iMaxIterations) )
		{
			m_A.PostMultiply( m_c, m_vTemp1 );

			m_vTemp1.ElementMultiply( m_S, m_q );

			alpha = Delta_new / (m_c.DotProduct( m_q ) );
			m_c.Scale( alpha, m_vTemp1 );
			m_dv.Add( m_vTemp1, m_dv );

			m_q.Scale( alpha, m_vTemp1 );
			m_r.Subtract( m_vTemp1, m_r );

			m_PInv.ElementMultiply( m_r, m_s );
			Delta_old = Delta_new;
			Delta_new = m_r.DotProduct( m_s );

			m_c.Scale( Delta_new / Delta_old, m_vTemp1 );
			m_s.Add( m_vTemp1, m_vTemp2 );
			m_vTemp2.ElementMultiply( m_S, m_c );

			iIterations++;
		}

		/*
		#ifdef _DEBUG
		for(int i=0; i<m_iParticles; i++)
		{
		if(mContacts.impulseExist[i])
		{
		printf("%d ", i);
		VECTOR3_PRINT(m_dv.m_pData[i]);
		printf("==");
		VECTOR3_PRINT(mContacts.desiredDeltaVelocity.m_pData[i]);
		printf("\n");
		}
		}
		#endif*/

		m_A.PostMultiply( m_dv, m_vTemp1 );

		m_vTemp1.Subtract( m_b, m_vContactForce);
		m_vContactForce.Dump();

		m_Velocities.Add( m_dv, m_Velocities );

		// 여기서 velocity clipping이 추가되야함.
		for(int i=0; i<m_iParticles; i++)
		{
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 const& p=mContacts.prevNormal[i];
				// 수직방향 속도
				double dn=VECTOR3_DP(m_Velocities[i], p);

				// 수평방향 속도만 남기기.
				VECTOR3_SUB(m_Velocities[i], p*dn, m_Velocities[i]);
			}
		}




		m_Velocities.Scale( fStep, m_vTemp1 );		

		m_Positions.Add( m_vTemp1, m_Positions );

		/*
		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
		for(int i=0, ni=m_Forces.size(); i<ni; i++)	
		{
		m_Forces[i]->Fixup( m_Masses, m_Positions);
		}
		}
		*/

		//mImpulse.clear();

		fTotal += (Physics_t)fabs( fStep );
	}
}
#endif
#ifdef YS_FRICTION
void Physics_ParticleSystem::Update_Implicit( float fTime )
{
	int i, iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3;
	Physics_t fTotal = (Physics_t)0.0f, fStep = (Physics_t)fTime;
	Physics_t alpha, Delta_0, Delta_old, Delta_new;
	Physics_t Eps_Sq = (Physics_t)1e-22;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.Zero();
		m_TotalForces_ext.Zero();
		m_TotalForces_dp.Zero();
		m_TotalForces_dv.Zero();
		m_MxTemp1.Zero();
		m_MxTemp2.Zero();

		// Setting m_S and m_y
		Reset();


		//
		// Apply the forces
		//
		void *Pos = NULL;
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply( fStep, m_Masses, true, m_Positions, m_Velocities, 
				m_TotalForces_int, m_TotalForces_ext, 
				m_TotalForces_dp, m_TotalForces_dv );
		}

		//printf("ext\n");
		//m_TotalForces_ext.Dump();
		//printf("int\n"); 
		//m_TotalForces_int.Dump();

#ifndef YS_FRICTION
		for(int i=0; i<m_iParticles; i++)
		{
			bool bApplyFrictionForce=false;
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				//cout << VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])<< endl;
				if(VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])<0.0001)
					//if(0)
				{
					// 땅으로 부터 잡아당기는 힘이 작용하는 경우, constraint을 풀어준다.
					m_y[i]=mContacts.deltaPosition[i];
					VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);
					mContacts.contactStatus[i]=Physics_Contacts::no_contact;
					continue;
				}
				else
					bApplyFrictionForce=true;
			}

			if(mContacts.contactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 dir;
				Physics_Vector3 p=mContacts.normal[i];
				p.Normalize();

				// dynamic friction
				Physics_Matrix3x3 pp, I;
				pp.SetFromOuterProduct(p, p);
				I.SetIdentity();
				I.Subtract(pp, m_S[i]);

				m_y[i]=mContacts.deltaPosition[i];

				// correct positions and velocities - 꺼내서 다음 프레임에 충돌 체크가 되지 않도록 한다.
				VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);
				//VECTOR3_ADD(m_Velocities[i], mImpulse.desiredDeltaVelocity[i], m_Velocities[i]);

				// 수직방향 속도.
				double dn=VECTOR3_DP(m_Velocities[i], p);

				// 수평방향 속도.				
				//VECTOR3_SUB(m_Velocities[i], p*dn, m_Velocities[i]);
				//	m_Velocities[i].y=0;

				/*if(mContacts.relativeVelocity[i].Length() <=m_cfg.DFthresholdVel && m_cfg.m_dynamicFrictionCoef[i].x >0.0)
				{
				// static friction
				m_S[i].Zero();
				bApplyFrictionForce=false;
				}*/

			}

			mContacts.saveContactsInformation(i);

			if(bApplyFrictionForce && VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])>0.0001)
			{
				// Apply friction force
				Physics_Vector3 frictionForce(0,0,0);

				Physics_Vector3 vRelVel=mContacts.prevRelativeVelocity[i];
				Physics_Vector3 p=mContacts.prevNormal[i];

				double relveldotp=VECTOR3_DP(vRelVel,p);
				double relVel=vRelVel.Length();

				if(relVel>m_cfg.DFthresholdVel)
				{
					// calc frictionForce
					Physics_Vector3 dir, temp;
					VECTOR3_CROSSPRODUCT(vRelVel, p, temp);
					temp.Normalize();
					VECTOR3_CROSSPRODUCT(p, temp, dir);
					dir.Normalize();

					double normalForce=VECTOR3_DP(p, m_vContactForce[i]);

					// ys
					normalForce -= VECTOR3_DP(p, m_TotalForces_int[i]);
					normalForce -= VECTOR3_DP(p, m_TotalForces_ext[i]);

					double mu=m_cfg.m_dynamicFrictionCoef[i].x;
					VECTOR3_SCALE(dir, mu*normalForce*-1, frictionForce);
				}

				// apply external forces
				VECTOR3_ADD(m_TotalForces_ext[i], frictionForce, m_TotalForces_ext[i]); 
			}
		}
#endif






		// ys
#ifdef YS_FRICTION
#define DBL_EPSILON     2.2204460492503131e-016 /* smallest such that 1.0+DBL_EPSILON != 1.0 */
		double epsilon = .01;
		double r = 0.;

		for(int i=0; i<m_iParticles; ++i)
		{
			if(Position(i).y < 0.)
			{
				//Physics_Vector3 normal(0,1,0);
				//if(VECTOR3_DP(Velocity(i), normal) < 0)
				if(Velocity(i).y < 0)
				{
					//VECTOR3_SUB(Velocity(i), normal * (1+r) * VECTOR3_DP(Velocity(i), normal), Velocity(i));
					Velocity(i).y = Velocity(i).y - (1+r) * Velocity(i).y;
				}

				Physics_Vector3 force;
				VECTOR3_ADD(m_TotalForces_ext[i], m_TotalForces_int[i], force);
				//VECTOR3_ADD(m_TotalForces_int[i], Physics_Vector3(0,0,0), force);

				//if(VECTOR3_DP(Velocity(i), normal) < DBL_EPSILON && VECTOR3_DP(Velocity(i), normal) > -DBL_EPSILON)
				//&& VECTOR3_DP(force, normal) < 0)
				if(Position(i).y < 0. + epsilon)
				{
					//Physics_t normalForceNorm = VECTOR3_DP(force, normal);
					//VECTOR3_ADD(normal * -normalForceNorm, m_TotalForces_ext[i], m_TotalForces_ext[i]);

					if(force.y < 0)
						m_TotalForces_ext[i].y += (-force.y);


					// drag force
					//if (Vector2.Dot(p.velocity, tangent) == 0) // 정지상태
					//{
					//	// 최대정지마찰계수가 적용될 때의 마찰력
					//	Vector2 frictionForce = Vector2.Normalize(p.force) * (normalForceNorm * MaxStaticDragCoefficient);
					//	if (Vector2.Dot(frictionForce, p.velocity) > 0)     // 마찰력의 방향은 속도의 반대
					//		frictionForce = -frictionForce;

					//	if (p.force.Length() < frictionForce.Length())    // 정지마찰계수가 최대값에 도달하지 않았을 때
					//	{
					//		p.force.X = 0;
					//		p.force.Y = 0;
					//	}
					//	else
					//	{
					//		//Debug.WriteLine("p.force:{0}", p.force.ToString());
					//		//Debug.WriteLine("frictionForce:{0}", frictionForce.ToString());
					//		//Debug.WriteLine("");
					//		p.force += frictionForce;
					//	}
					//}
					//else  // 운동상태

					//if(force.y < 0)
					{
						//if(Velocity(i).Length() < .5)
						//{
						//	Velocity(i).x = 0.;Velocity(i).y = 0.;Velocity(i).z = 0.;
						//}
						//else
						{
							// 마찰력의 방향은 속도의 반대
							double DynamicDragCoefficient = m_cfg.m_dynamicFrictionCoef[i].x;

							Physics_Vector3 normalizedVelocity(Velocity(i).x, 0, Velocity(i).z);
							normalizedVelocity.Normalize();
							Physics_Vector3 frictionForce = (normalizedVelocity * -1) * fabs(force.y) * DynamicDragCoefficient;
							//Physics_Vector3 frictionForce = (normalizedVelocity * -1) * fabs(51.*9.8+force.y) * DynamicDragCoefficient;

							VECTOR3_ADD(frictionForce, m_TotalForces_ext[i], m_TotalForces_ext[i]);
						}

						// velocity 방향 체크 - 방향 바뀌면 정지상태로 만들기 위해 velocity=0로 해준다.
						//Vector2 nextAcceleration = p.force * (1.0f / p.mass);
						//Vector2 nextVelocity = p.velocity + nextAcceleration * TimeStep;
						//if (Vector2.Dot(p.velocity, nextVelocity) < 0)
						//{
						//	p.velocity.X = 0;
						//	p.velocity.Y = 0;
						//	p.force.X = 0;
						//	p.force.Y = 0;
						//}
					}
				}
			}
		}
#endif


		//
		// Form the symmetric matrix A = M - h * df/dv - h^2 * df/dx (Baraff 논문의 equation 6의 양변에 M을 곱한식)
		// We regroup this as A = M - h * (df/dv + h * df/dx)
		//
		m_TotalForces_int.Add( m_TotalForces_ext, m_TotalForces_int );
		m_TotalForces_dp.Scale( fStep, m_MxTemp1 );
		m_TotalForces_dv.Add( m_MxTemp1, m_MxTemp2 );
		m_MxTemp2.Scale( fStep, m_MxTemp1 );
		m_MxMasses.Subtract( m_MxTemp1, m_A );

		//
		// Compute b = h * ( f(0) + h * df/dx * v(0) + df/dx * y )
		//
		m_Velocities.Scale( fStep, m_vTemp2 );
		m_vTemp2.Add( m_y, m_vTemp2 );
		m_TotalForces_dp.PostMultiply( m_vTemp2, m_vTemp1 );
		//		m_vTemp1.Scale( fStep, m_vTemp2 );
		m_TotalForces_int.Add( m_vTemp1, m_vTemp1 );
		m_vTemp1.Scale( fStep, m_b );

		//
		// Setup the inverse of preconditioner -- We use a vector for memory efficiency.  
		// Technically it's the diagonal of a matrix
		//
		for( i=0; i<m_iParticles; i++ )
		{
			m_PInv.m_pData[i].x = (Physics_t)m_A(i,i).m_Mx[0];
			m_PInv.m_pData[i].y = (Physics_t)m_A(i,i).m_Mx[4];
			m_PInv.m_pData[i].z = (Physics_t)m_A(i,i).m_Mx[8];
		}
		m_PInv.Invert( m_P );

		//
		// Modified Preconditioned Conjugate Gradient method
		//

		m_dv = m_z;

		// delta_0 = DotProduct( filter( b ), P * filter( b ) );
		m_b.ElementMultiply( m_S, m_vTemp1 );
		m_P.ElementMultiply( m_vTemp1, m_vTemp2 );
		Delta_0 = m_vTemp2.DotProduct( m_vTemp1 );
		if( Delta_0 < 0 )
		{
			m_b.Dump( "b:\r\n" );
			m_P.Dump( "P:\r\n" );
			//OutputDebugString( "Negative Delta_0 most likely caused by a non-Positive Definite matrix\r\n" );
		}

		// r = filter( b - A * dv )
		m_A.PostMultiply( m_dv, m_vTemp1 );
		m_b.Subtract( m_vTemp1, m_vTemp2 );
		m_vTemp2.ElementMultiply( m_S, m_r );

		// c = filter( Pinv * r )
		m_PInv.ElementMultiply( m_r, m_vTemp1 );
		m_vTemp1.ElementMultiply( m_S, m_c );

		Delta_new = m_r.DotProduct( m_c );

		if( Delta_new < Eps_Sq * Delta_0 )
		{
			m_b.Dump( "b: \r\n" );
			m_P.Dump( "P: \r\n" );
			//OutputDebugString( "This isn't good!  Probably a non-Positive Definite matrix\r\n" );
		}

		while( (Delta_new > Eps_Sq * Delta_0) && (iIterations < iMaxIterations) )
		{
			m_A.PostMultiply( m_c, m_vTemp1 );

			m_vTemp1.ElementMultiply( m_S, m_q );

			alpha = Delta_new / (m_c.DotProduct( m_q ) );
			m_c.Scale( alpha, m_vTemp1 );
			m_dv.Add( m_vTemp1, m_dv );

			m_q.Scale( alpha, m_vTemp1 );
			m_r.Subtract( m_vTemp1, m_r );

			m_PInv.ElementMultiply( m_r, m_s );
			Delta_old = Delta_new;
			Delta_new = m_r.DotProduct( m_s );

			m_c.Scale( Delta_new / Delta_old, m_vTemp1 );
			m_s.Add( m_vTemp1, m_vTemp2 );
			m_vTemp2.ElementMultiply( m_S, m_c );

			iIterations++;
		}

		/*
		#ifdef _DEBUG
		for(int i=0; i<m_iParticles; i++)
		{
		if(mContacts.impulseExist[i])
		{
		printf("%d ", i);
		VECTOR3_PRINT(m_dv.m_pData[i]);
		printf("==");
		VECTOR3_PRINT(mContacts.desiredDeltaVelocity.m_pData[i]);
		printf("\n");
		}
		}
		#endif*/

		m_A.PostMultiply( m_dv, m_vTemp1 );

#ifndef YS_FRICTION
		m_vTemp1.Subtract( m_b, m_vContactForce);
		//m_vContactForce.Dump();
#endif

		m_Velocities.Add( m_dv, m_Velocities );

#ifndef YS_FRICTION
		// 여기서 velocity clipping이 추가되야함.
		for(int i=0; i<m_iParticles; i++)
		{
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 const& p=mContacts.prevNormal[i];
				// 수직방향 속도
				double dn=VECTOR3_DP(m_Velocities[i], p);

				// 수평방향 속도만 남기기.
				VECTOR3_SUB(m_Velocities[i], p*dn, m_Velocities[i]);
			}
		}
#endif





		m_Velocities.Scale( fStep, m_vTemp1 );		

		m_Positions.Add( m_vTemp1, m_Positions );

		/*
		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
		for(int i=0, ni=m_Forces.size(); i<ni; i++)	
		{
		m_Forces[i]->Fixup( m_Masses, m_Positions);
		}
		}
		*/

		//mImpulse.clear();

		fTotal += (Physics_t)fabs( fStep );
	}
}}
#endif
#ifdef YS_FRICTION2
void Physics_ParticleSystem::Update_Implicit( float fTime )
{
	int i, iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3;
	Physics_t fTotal = (Physics_t)0.0f, fStep = (Physics_t)fTime;
	Physics_t alpha, Delta_0, Delta_old, Delta_new;
	Physics_t Eps_Sq = (Physics_t)1e-22;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.Zero();
		m_TotalForces_ext.Zero();
		m_TotalForces_dp.Zero();
		m_TotalForces_dv.Zero();
		m_MxTemp1.Zero();
		m_MxTemp2.Zero();

		// Setting m_S and m_y
		Reset();


		//
		// Apply the forces
		//
		void *Pos = NULL;
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply( fStep, m_Masses, true, m_Positions, m_Velocities, 
				m_TotalForces_int, m_TotalForces_ext, 
				m_TotalForces_dp, m_TotalForces_dv );
		}

		//printf("ext\n");
		//m_TotalForces_ext.Dump();
		//printf("int\n"); 
		//m_TotalForces_int.Dump();

		for(int i=0; i<m_iParticles; i++)
		{
			bool bApplyFrictionForce=false;
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				//cout << VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])<< endl;
				if(VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])<0.0001)
					//if(0)
				{
					// 땅으로 부터 잡아당기는 힘이 작용하는 경우, constraint을 풀어준다.
					m_y[i]=mContacts.deltaPosition[i];
					VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);
					mContacts.contactStatus[i]=Physics_Contacts::no_contact;
					continue;
				}
				else
					bApplyFrictionForce=true;
			}

			if(mContacts.contactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 dir;
				Physics_Vector3 p=mContacts.normal[i];
				p.Normalize();

				// dynamic friction
				Physics_Matrix3x3 pp, I;
				pp.SetFromOuterProduct(p, p);
				I.SetIdentity();
				I.Subtract(pp, m_S[i]);

				m_y[i]=mContacts.deltaPosition[i];

				// correct positions and velocities - 꺼내서 다음 프레임에 충돌 체크가 되지 않도록 한다.

				VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);

				//VECTOR3_ADD(m_Velocities[i], mImpulse.desiredDeltaVelocity[i], m_Velocities[i]);

				// 수직방향 속도.
				double dn=VECTOR3_DP(m_Velocities[i], p);

				// 수평방향 속도.				
				//VECTOR3_SUB(m_Velocities[i], p*dn, m_Velocities[i]);
				//	m_Velocities[i].y=0;

				/*if(mContacts.relativeVelocity[i].Length() <=m_cfg.DFthresholdVel && m_cfg.m_dynamicFrictionCoef[i].x >0.0)
				{
				// static friction
				m_S[i].Zero();
				bApplyFrictionForce=false;
				}*/

			}

			mContacts.saveContactsInformation(i);

			if(bApplyFrictionForce && VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i])>0.0001)
			{
				// Apply friction force
				Physics_Vector3 frictionForce(0,0,0);

				Physics_Vector3 vRelVel=mContacts.prevRelativeVelocity[i];
				Physics_Vector3 p=mContacts.prevNormal[i];

				double relveldotp=VECTOR3_DP(vRelVel,p);
				double relVel=vRelVel.Length();

				if(relVel>m_cfg.DFthresholdVel)
				{
					// calc frictionForce
					Physics_Vector3 dir, temp;
					VECTOR3_CROSSPRODUCT(vRelVel, p, temp);
					temp.Normalize();
					VECTOR3_CROSSPRODUCT(p, temp, dir);
					dir.Normalize();

					double normalForce=VECTOR3_DP(p, m_vContactForce[i]);

					// ys
					normalForce -= VECTOR3_DP(p, m_TotalForces_int[i]);
					normalForce -= VECTOR3_DP(p, m_TotalForces_ext[i]);

					double mu=m_cfg.m_dynamicFrictionCoef[i].x;
					VECTOR3_SCALE(dir, mu*normalForce*-1, frictionForce);
				}

				// apply external forces
				VECTOR3_ADD(m_TotalForces_ext[i], frictionForce, m_TotalForces_ext[i]); 
			}
		}


		//
		// Form the symmetric matrix A = M - h * df/dv - h^2 * df/dx (Baraff 논문의 equation 6의 양변에 M을 곱한식)
		// We regroup this as A = M - h * (df/dv + h * df/dx)
		//
		m_TotalForces_int.Add( m_TotalForces_ext, m_TotalForces_int );
		m_TotalForces_dp.Scale( fStep, m_MxTemp1 );
		m_TotalForces_dv.Add( m_MxTemp1, m_MxTemp2 );
		m_MxTemp2.Scale( fStep, m_MxTemp1 );
		m_MxMasses.Subtract( m_MxTemp1, m_A );

		//
		// Compute b = h * ( f(0) + h * df/dx * v(0) + df/dx * y )
		//
		m_Velocities.Scale( fStep, m_vTemp2 );
		m_vTemp2.Add( m_y, m_vTemp2 );
		m_TotalForces_dp.PostMultiply( m_vTemp2, m_vTemp1 );
		//		m_vTemp1.Scale( fStep, m_vTemp2 );
		m_TotalForces_int.Add( m_vTemp1, m_vTemp1 );
		m_vTemp1.Scale( fStep, m_b );

		//
		// Setup the inverse of preconditioner -- We use a vector for memory efficiency.  
		// Technically it's the diagonal of a matrix
		//
		for( i=0; i<m_iParticles; i++ )
		{
			m_PInv.m_pData[i].x = (Physics_t)m_A(i,i).m_Mx[0];
			m_PInv.m_pData[i].y = (Physics_t)m_A(i,i).m_Mx[4];
			m_PInv.m_pData[i].z = (Physics_t)m_A(i,i).m_Mx[8];
		}
		m_PInv.Invert( m_P );

		//
		// Modified Preconditioned Conjugate Gradient method
		//

		m_dv = m_z;

		// delta_0 = DotProduct( filter( b ), P * filter( b ) );
		m_b.ElementMultiply( m_S, m_vTemp1 );
		m_P.ElementMultiply( m_vTemp1, m_vTemp2 );
		Delta_0 = m_vTemp2.DotProduct( m_vTemp1 );
		if( Delta_0 < 0 )
		{
			m_b.Dump( "b:\r\n" );
			m_P.Dump( "P:\r\n" );
			//OutputDebugString( "Negative Delta_0 most likely caused by a non-Positive Definite matrix\r\n" );
		}

		// r = filter( b - A * dv )
		m_A.PostMultiply( m_dv, m_vTemp1 );
		m_b.Subtract( m_vTemp1, m_vTemp2 );
		m_vTemp2.ElementMultiply( m_S, m_r );

		// c = filter( Pinv * r )
		m_PInv.ElementMultiply( m_r, m_vTemp1 );
		m_vTemp1.ElementMultiply( m_S, m_c );

		Delta_new = m_r.DotProduct( m_c );

		if( Delta_new < Eps_Sq * Delta_0 )
		{
			m_b.Dump( "b: \r\n" );
			m_P.Dump( "P: \r\n" );
			//OutputDebugString( "This isn't good!  Probably a non-Positive Definite matrix\r\n" );
		}

		while( (Delta_new > Eps_Sq * Delta_0) && (iIterations < iMaxIterations) )
		{
			m_A.PostMultiply( m_c, m_vTemp1 );

			m_vTemp1.ElementMultiply( m_S, m_q );

			alpha = Delta_new / (m_c.DotProduct( m_q ) );
			m_c.Scale( alpha, m_vTemp1 );
			m_dv.Add( m_vTemp1, m_dv );

			m_q.Scale( alpha, m_vTemp1 );
			m_r.Subtract( m_vTemp1, m_r );

			m_PInv.ElementMultiply( m_r, m_s );
			Delta_old = Delta_new;
			Delta_new = m_r.DotProduct( m_s );

			m_c.Scale( Delta_new / Delta_old, m_vTemp1 );
			m_s.Add( m_vTemp1, m_vTemp2 );
			m_vTemp2.ElementMultiply( m_S, m_c );

			iIterations++;
		}

		/*
		#ifdef _DEBUG
		for(int i=0; i<m_iParticles; i++)
		{
		if(mContacts.impulseExist[i])
		{
		printf("%d ", i);
		VECTOR3_PRINT(m_dv.m_pData[i]);
		printf("==");
		VECTOR3_PRINT(mContacts.desiredDeltaVelocity.m_pData[i]);
		printf("\n");
		}
		}
		#endif*/

		m_A.PostMultiply( m_dv, m_vTemp1 );

		m_vTemp1.Subtract( m_b, m_vContactForce);
		//m_vContactForce.Dump();

		m_Velocities.Add( m_dv, m_Velocities );

		// 여기서 velocity clipping이 추가되야함.
		for(int i=0; i<m_iParticles; i++)
		{
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 const& p=mContacts.prevNormal[i];
				// 수직방향 속도
				double dn=VECTOR3_DP(m_Velocities[i], p);

				// 수평방향 속도만 남기기.
				VECTOR3_SUB(m_Velocities[i], p*dn, m_Velocities[i]);
			}
		}




		m_Velocities.Scale( fStep, m_vTemp1 );		

		m_Positions.Add( m_vTemp1, m_Positions );

		/*
		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
		for(int i=0, ni=m_Forces.size(); i<ni; i++)	
		{
		m_Forces[i]->Fixup( m_Masses, m_Positions);
		}
		}
		*/

		//mImpulse.clear();

		fTotal += (Physics_t)fabs( fStep );
	}
}
#endif
#ifdef YS_FRICTION3
void Physics_ParticleSystem::Update_Implicit( float fTime )
{
	int i, iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3;
	Physics_t fTotal = (Physics_t)0.0f, fStep = (Physics_t)fTime;
	Physics_t alpha, Delta_0, Delta_old, Delta_new;
	Physics_t Eps_Sq = (Physics_t)1e-22;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.Zero();
		m_TotalForces_ext.Zero();
		m_TotalForces_dp.Zero();
		m_TotalForces_dv.Zero();
		m_MxTemp1.Zero();
		m_MxTemp2.Zero();

		// Setting m_S and m_y
		Reset();


		//
		// Apply the forces
		//
		void *Pos = NULL;
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply( fStep, m_Masses, true, m_Positions, m_Velocities, 
				m_TotalForces_int, m_TotalForces_ext, 
				m_TotalForces_dp, m_TotalForces_dv );
		}

		//printf("ext\n");
		//m_TotalForces_ext.Dump();
		//printf("int\n"); 
		//m_TotalForces_int.Dump();

		// ys
		for(int i=0; i<m_iParticles; i++)
		{
			if(mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
			{
				if(VECTOR3_DP(m_vContactForce[i], mContacts.prevNormal[i]) < 0.)
				{
					mContacts.contactStatus[i]=Physics_Contacts::no_contact;
					ReleaseConstraint(i);
				}
				else if (m_ConstraintsYS[i]->m_iDegreesOfFreedom == 0)	// if locked
				{
					Physics_Vector3 vNormal = mContacts.prevNormal[i];

					Physics_t normalForce = VECTOR3_DP(vNormal, m_vContactForce[i]);
					Physics_Vector3 vNormalForce;
					VECTOR3_SCALE(vNormal, normalForce, vNormalForce);

					Physics_Vector3 vTangentialForce;
					VECTOR3_SUB(m_vContactForce[i], vNormalForce, vTangentialForce);
					Physics_t tangentialForce = vTangentialForce.Length();

					if( tangentialForce > m_cfg.m_staticFrictionCoef[i].x * normalForce)
						ReleaseLock(i, 2, vNormal, Physics_Vector3(0,0,0));
				}
			}

			//printf("%d contact status:%d\n", i, mContacts.contactStatus[i]);
			if(mContacts.contactStatus[i]!=Physics_Contacts::no_contact)
			{
				Physics_Vector3 vRelVel = mContacts.relativeVelocity[i];
				Physics_Vector3 vNormal = mContacts.normal[i];

				Physics_t normalRelVel = VECTOR3_DP(vRelVel, vNormal);
				Physics_Vector3 vNormalRelVel;
				VECTOR3_SCALE(vNormal, normalRelVel, vNormalRelVel);

				Physics_Vector3 vTangentialRelVel;
				VECTOR3_SUB(vRelVel, vNormalRelVel, vTangentialRelVel);
				Physics_t tangentialRelVel = vTangentialRelVel.Length();


				// add constraint (constrain velocity)

				// static friction
				if(m_cfg.m_staticFrictionCoef[i].x > 0. && 
					tangentialRelVel < m_cfg.tangentialLockingThresholdVel)
				{
					SetLock(i, Physics_Vector3(-vRelVel.x, -vRelVel.y, -vRelVel.z));
				}
				// dynamic friction
				else
				{
					SetConstraint(i, 2, vNormal, Physics_Vector3(0,0,0), -VECTOR3_DP(vNormal, vRelVel));

					// apply friction forces
					Physics_Vector3 frictionForce;

					//if(relVel>m_cfg.DFthresholdVel)
					{
						Physics_Vector3 dir;
						dir = vTangentialRelVel;
						dir.Normalize();

						double normalForce = VECTOR3_DP(vNormal, m_vContactForce[i]);
						normalForce -= VECTOR3_DP(vNormal, m_TotalForces_int[i]);
						normalForce -= VECTOR3_DP(vNormal, m_TotalForces_ext[i]);
						//printf("%d %f\n", i, normalForce);

						double mu=m_cfg.m_dynamicFrictionCoef[i].x;
						VECTOR3_SCALE(dir, mu*normalForce*-1, frictionForce);
					}
					VECTOR3_ADD(m_TotalForces_ext[i], frictionForce, m_TotalForces_ext[i]); 
				}

				// position alternation
				m_y[i]=mContacts.deltaPosition[i];
				VECTOR3_ADD(m_Positions[i], mContacts.deltaPosition[i], m_Positions[i]);
			}

			mContacts.saveContactsInformation(i);
		}


		//
		// Form the symmetric matrix A = M - h * df/dv - h^2 * df/dx (Baraff 논문의 equation 6의 양변에 M을 곱한식)
		// We regroup this as A = M - h * (df/dv + h * df/dx)
		//
		m_TotalForces_int.Add( m_TotalForces_ext, m_TotalForces_int );
		m_TotalForces_dp.Scale( fStep, m_MxTemp1 );
		m_TotalForces_dv.Add( m_MxTemp1, m_MxTemp2 );
		m_MxTemp2.Scale( fStep, m_MxTemp1 );
		m_MxMasses.Subtract( m_MxTemp1, m_A );

		//
		// Compute b = h * ( f(0) + h * df/dx * v(0) + df/dx * y )
		//
		m_Velocities.Scale( fStep, m_vTemp2 );
		m_vTemp2.Add( m_y, m_vTemp2 );
		m_TotalForces_dp.PostMultiply( m_vTemp2, m_vTemp1 );
		//		m_vTemp1.Scale( fStep, m_vTemp2 );
		m_TotalForces_int.Add( m_vTemp1, m_vTemp1 );
		m_vTemp1.Scale( fStep, m_b );

		//
		// Setup the inverse of preconditioner -- We use a vector for memory efficiency.  
		// Technically it's the diagonal of a matrix
		//
		for( i=0; i<m_iParticles; i++ )
		{
			m_PInv.m_pData[i].x = (Physics_t)m_A(i,i).m_Mx[0];
			m_PInv.m_pData[i].y = (Physics_t)m_A(i,i).m_Mx[4];
			m_PInv.m_pData[i].z = (Physics_t)m_A(i,i).m_Mx[8];
		}
		m_PInv.Invert( m_P );

		//
		// Modified Preconditioned Conjugate Gradient method
		//

		m_dv = m_z;

		// delta_0 = DotProduct( filter( b ), P * filter( b ) );
		m_b.ElementMultiply( m_S, m_vTemp1 );
		m_P.ElementMultiply( m_vTemp1, m_vTemp2 );
		Delta_0 = m_vTemp2.DotProduct( m_vTemp1 );
		if( Delta_0 < 0 )
		{
			m_b.Dump( "b:\r\n" );
			m_P.Dump( "P:\r\n" );
			//OutputDebugString( "Negative Delta_0 most likely caused by a non-Positive Definite matrix\r\n" );
		}

		// r = filter( b - A * dv )
		m_A.PostMultiply( m_dv, m_vTemp1 );
		m_b.Subtract( m_vTemp1, m_vTemp2 );
		m_vTemp2.ElementMultiply( m_S, m_r );

		// c = filter( Pinv * r )
		m_PInv.ElementMultiply( m_r, m_vTemp1 );
		m_vTemp1.ElementMultiply( m_S, m_c );

		Delta_new = m_r.DotProduct( m_c );

		if( Delta_new < Eps_Sq * Delta_0 )
		{
			m_b.Dump( "b: \r\n" );
			m_P.Dump( "P: \r\n" );
			//OutputDebugString( "This isn't good!  Probably a non-Positive Definite matrix\r\n" );
		}

		while( (Delta_new > Eps_Sq * Delta_0) && (iIterations < iMaxIterations) )
		{
			m_A.PostMultiply( m_c, m_vTemp1 );

			m_vTemp1.ElementMultiply( m_S, m_q );

			alpha = Delta_new / (m_c.DotProduct( m_q ) );
			m_c.Scale( alpha, m_vTemp1 );
			m_dv.Add( m_vTemp1, m_dv );

			m_q.Scale( alpha, m_vTemp1 );
			m_r.Subtract( m_vTemp1, m_r );

			m_PInv.ElementMultiply( m_r, m_s );
			Delta_old = Delta_new;
			Delta_new = m_r.DotProduct( m_s );

			m_c.Scale( Delta_new / Delta_old, m_vTemp1 );
			m_s.Add( m_vTemp1, m_vTemp2 );
			m_vTemp2.ElementMultiply( m_S, m_c );

			iIterations++;
		}

		m_A.PostMultiply( m_dv, m_vTemp1 );

		m_vTemp1.Subtract( m_b, m_vContactForce);
		//m_vContactForce.Dump();

		m_Velocities.Add( m_dv, m_Velocities );



		m_Velocities.Scale( fStep, m_vTemp1 );		

		m_Positions.Add( m_vTemp1, m_Positions );

		/*
		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
		for(int i=0, ni=m_Forces.size(); i<ni; i++)	
		{
		m_Forces[i]->Fixup( m_Masses, m_Positions);
		}
		}
		*/

		fTotal += (Physics_t)fabs( fStep );
	}
}
#endif

void Physics_ParticleSystem::Update_SemiImplicit( float fTime )
{
	Physics_t fTotal = 0.0f, fStep = (Physics_t)fTime;
	Physics_Vector3 vCOG, dTorque, tmp, tmp2;
	int i;

	while( fTotal < fTime )
	{
		//
		// Calculate the center of gravity
		//
		vCOG.x = vCOG.y = vCOG.z = 0;
		for( i=0; i<m_iParticles; i++ )
			VECTOR3_ADD( vCOG, m_Positions.m_pData[i], vCOG );
		vCOG.x /= m_iParticles;
		vCOG.y /= m_iParticles;
		vCOG.z /= m_iParticles;

		dTorque.x = dTorque.y = dTorque.z = 0.0f;

		//
		// Update the W matrix if necessary
		//
		if( fStep != m_LastStep )
		{
			m_W.SetIdentity();
			m_H.Scale( fStep * fStep / m_LastStep / m_LastStep, m_H );
			m_W.Subtract( m_H, m_W );
			m_W.Invert();
			m_LastStep = fStep;
		}

		// 
		// Zero out everything
		//
		m_TotalForces_int.Zero();
		m_TotalForces_ext.Zero();


		//
		// Apply the forces
		//
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply(fStep, m_Masses, false, m_Positions, m_Velocities,
				m_TotalForces_int, m_TotalForces_ext,
				m_TotalForces_dp, m_TotalForces_dv );
		}

		//
		// Filter the internal forces
		//
		m_W.PreMultiply( m_TotalForces_int, m_vTemp1 );

		//
		// Update the torque
		//
		for( i=0; i<m_iParticles; i++ )
		{
			VECTOR3_CROSSPRODUCT( m_vTemp1.m_pData[i], m_Positions.m_pData[i], tmp );
			VECTOR3_ADD( dTorque, tmp, dTorque );
		}


		//
		// Compute the new velocities and positions
		//
		m_vTemp1.Add( m_TotalForces_ext, m_dv );
		m_dv.Scale( fStep, m_dv );
		m_dv.ElementMultiply( m_MassInv, m_dv );
		m_Velocities.Add( m_dv, m_Velocities );



		m_Velocities.Scale( fStep, m_vTemp1 );
		m_Positions.Add( m_vTemp1, m_PosTemp );

		/*//
		// Keep us above the floor -- cheesey collision detection
		//
		for( i=0; i<m_iParticles; i++ )
		if( m_PosTemp.m_pData[i].y < FLOOR_Y )
		{
		m_PosTemp.m_pData[i].y = FLOOR_Y;
		m_Velocities.m_pData[i].y = 0;
		}
		*/
		//
		// Post correct for angular momentum
		//

		for( i=0; i<m_iParticles; i++ )
		{
			if( m_Masses.m_pData[i].x )
			{
				VECTOR3_SUB( vCOG, m_Positions.m_pData[i], tmp2 );
				VECTOR3_CROSSPRODUCT( tmp2, dTorque, tmp );
				VECTOR3_SCALE( tmp, fStep * fStep * fStep * m_MassInv.m_pData[i].x, tmp );
				VECTOR3_ADD( m_PosTemp.m_pData[i], tmp, m_PosTemp.m_pData[i] );
				/*				if( m_PosTemp.m_pData[i].y < FLOOR_Y )
				{
				m_PosTemp.m_pData[i].y = FLOOR_Y;
				m_Velocities.m_pData[i].y = 0;
				}*/
			}
		}

		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
			for(int i=0, ni=m_Forces.size(); i<ni; i++)	
			{
				m_Forces[i]->Fixup( m_Masses, m_PosTemp );
			}
		}
		//
		// Update velocityp and position
		//
		/*
		for( i=0; i<m_iParticles; i++ )
		if( m_PosTemp.m_pData[i].y < FLOOR_Y )
		{
		m_PosTemp.m_pData[i].y = FLOOR_Y;
		m_Velocities.m_pData[i].y = 0;
		}*/

		m_PosTemp.Subtract( m_Positions, m_vTemp1 );
		m_vTemp1.Scale( (Physics_t)1.0 / fStep, m_Velocities );
		m_Positions = m_PosTemp;

		fTotal += (Physics_t)fabs( fStep );
	}
}


// ys
void Physics_ParticleSystem::SetConstraint( int iParticle, int iDegreesOfFreedom, Physics_Vector3 axis1 , Physics_Vector3 axis2 , Physics_Vector3 ConstrainedVelocity )
{
	m_ConstraintsYS[iParticle]->m_iParticle = iParticle;
	m_ConstraintsYS[iParticle]->m_iDegreesOfFreedom = iDegreesOfFreedom;
	m_ConstraintsYS[iParticle]->m_p = axis1;
	m_ConstraintsYS[iParticle]->m_q = axis2;
	m_ConstraintsYS[iParticle]->m_ConstrainedVelocity = ConstrainedVelocity;
	m_ConstraintsYS[iParticle]->m_op1.SetFromOuterProduct(axis1, axis1);
	m_ConstraintsYS[iParticle]->m_op2.SetFromOuterProduct(axis2, axis2);

	m_ConstraintsYS[iParticle]->Apply(m_S, m_z);
}
// ys
void Physics_ParticleSystem::ReleaseConstraint( int iParticle )
{
	m_ConstraintsYS[iParticle]->m_iDegreesOfFreedom = 3;
	m_S[iParticle].SetIdentity();
	m_z.m_pData[iParticle] = 0;
}
// ys
void Physics_ParticleSystem::SetLock( int iParticle, Physics_Vector3 ConstrainedVelocity )
{
	SetConstraint(iParticle, 0, Physics_Vector3(0,0,0), Physics_Vector3(0,0,0), ConstrainedVelocity);
}
// ys
void Physics_ParticleSystem::ReleaseLock( int iParticle, int iDegreesOfFreedom, Physics_Vector3 axis1 , Physics_Vector3 axis2 )
{
	ReleaseConstraint(iParticle);
	SetConstraint(iParticle, iDegreesOfFreedom, axis1, axis2, Physics_Vector3(0,0,0));
}

void Physics_ParticleSystem::AddConstraint(Physics_Constraint &Constraint )
{
	m_Constraints.AddItem( &Constraint );

	//
	// Apply the constraints
	//
	for( int i=0; i<m_iParticles; i++ )
	{
		m_S[i].SetIdentity();
	}
	m_z.Zero();


	void *Pos = NULL;
	Physics_Constraint *pConstraint = m_Constraints.NextItem( &Pos );
	while( pConstraint )
	{
		pConstraint->Apply(m_S, m_z);
		pConstraint = m_Constraints.NextItem( &Pos );
	}
}

bool Physics_ParticleSystem::DeleteConstraint( Physics_Constraint &Constraint )
{
	bool bResult = m_Constraints.DeleteItem( &Constraint );

	if( bResult )
	{
		//
		// Apply the constraints
		//
		for( int i=0; i<m_iParticles; i++ )
		{
			m_S[i].SetIdentity();
		}
		m_z.Zero();


		void *Pos = NULL;
		Physics_Constraint *pConstraint = m_Constraints.NextItem( &Pos );
		while( pConstraint )
		{
			pConstraint->Apply(m_S, m_z);
			pConstraint = m_Constraints.NextItem( &Pos );
		}
	}
	return bResult;
}

void Physics_ParticleSystem::Reset()
{
	//
	// Update the constraints for the Implicit integration scheme
	//
	if( m_iIntegrationMethod == INTEGRATE_IMPLICIT )
	{
		for( int i=0; i<m_iParticles; i++ )
		{
			m_S[i].SetIdentity();
		}
		m_z.Zero();
		m_y.Zero();

		void *Pos = NULL;
		Physics_Constraint *pConstraint = m_Constraints.NextItem( &Pos );
		while( pConstraint )
		{
			pConstraint->Apply(m_S, m_z);
			pConstraint = m_Constraints.NextItem( &Pos );
		}
	}
}
