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

#include <VP/vpBody.h>
#include <VP/vpJoint.h>
#include <VP/vpSystem.h>
#include <VP/vpWorld.h>
#include <VP/vpMaterial.h>
#include <VP/vpGeom.h>

vpBody::vpBody() :	m_sV(SCALAR_0),
					m_sDV(SCALAR_0),
					m_sForce(SCALAR_0),
					m_sImpulse(SCALAR_0),
					m_bIsCollidable(true),
					m_pSystem(NULL),
					m_pWorld(NULL),
					m_bSetInertia(false),
					m_rBoundingSphereRadius(SCALAR_0),
					m_bIsGround(false)
{
	m_pMaterial = vpMaterial::GetDefaultMaterial();
	m_sI = Inertia(SCALAR_1);
	m_sHDType = VP::DYNAMIC;
}

void vpBody::SetInertia(const Inertia &J)
{
	//assert(m_sI.GetMass() != SCALAR_0 && "vpBody::SetInertia -> zero mass");
	m_sI = J;
	m_sCenterOfMass = (SCALAR_1 / m_sI.GetMass()) * m_sI.GetOffset();
	m_bSetInertia = true;
}

void vpBody::SetJoint(vpJoint *pJ, const SE3 &T)
{
	if ( m_pJoint.find(pJ) == -1 )
	{
		m_pJoint.push_back(pJ);
		if ( !pJ->m_pLeftBody ) pJ->SetBody(VP::LEFT, this, T);
		else pJ->SetBody(VP::RIGHT, this, T);
	} else
	{
		if ( pJ->m_pLeftBody == this ) pJ->SetBody(VP::LEFT, this, T);
		else pJ->SetBody(VP::RIGHT, this, T);
	}
}

void vpBody::RemoveJoint(vpJoint *pJ)
{
	m_pJoint.remove(m_pJoint.find(pJ));
}

void vpBody::RemoveSpring(vpSpring *pSpring)
{
	m_pSpring.remove(m_pSpring.find(pSpring));
}

const SE3 &vpBody::GetJointFrame(const vpJoint *pJ) const
{
	assert((pJ->m_pLeftBody == this || pJ->m_pRightBody == this) && "vpBody::GetJointFrame");
	if ( pJ->m_pLeftBody == this ) return pJ->m_sLeftBodyFrame;
	return pJ->m_sRightBodyFrame;	
}

void vpBody::SetJointFrame(vpJoint *pJ, const SE3 &T)
{
	assert((pJ->m_pLeftBody == this || pJ->m_pRightBody == this) && "vpBody::SetJointFrame");
	if ( pJ->m_pLeftBody == this ) pJ->m_sLeftBodyFrame = T;
	pJ->m_sRightBodyFrame = T;
}

dse3 vpBody::GetForce(void) const
{
	assert(m_pWorld && "vpBody::GetForce -> not regitered to the world");
	
	dse3 f;
	SE3 T;

	f = m_sForce;
	
	T.Set(m_sFrame, m_sCenterOfMass);
	f += m_sI * InvAd(T, m_pWorld->GetGravity());

	return f;
}

dse3 vpBody::GetGravityForce(void) const
{
	assert(m_pWorld && "vpBody::GetForce -> not regitered to the world");
	
	SE3 T;
	T.Set(m_sFrame, m_sCenterOfMass);
	return m_sI * InvAd(T, m_pWorld->GetGravity());
}

const dse3 &vpBody::GetNetForce(void) const
{
	return m_sForce;
}

void vpBody::AddGeometry(vpGeom *pGeom, const SE3 &T)
{
	assert(pGeom && "vpBody::AddGeometry -> invalid geometry");
	m_pGeom.check_push_back(pGeom);
	pGeom->m_pBody = this;
	pGeom->m_sLocalFrame = T;
}

void vpBody::Initialize(void)
{
	int i;

	if ( !m_bSetInertia )
	{		
		m_sI = Inertia(SCALAR_0);
		for ( i = 0; i < m_pGeom.size(); i++ )
			m_sI = m_sI + m_pGeom[i]->GetInertia(m_pMaterial->GetDensity()).Transform(Inv(m_pGeom[i]->m_sLocalFrame));
		
		m_sCenterOfMass = (SCALAR_1 / m_sI.GetMass()) * m_sI.GetOffset();		

		if ( m_pGeom.empty() ) SetInertia(Inertia(SCALAR_1));
	}

	m_rBoundingSphereRadius = SCALAR_0;
	for ( i = 0; i < m_pGeom.size(); i++ )
		m_rBoundingSphereRadius = max(m_rBoundingSphereRadius, Norm(m_pGeom[i]->m_sLocalFrame.GetPosition()) + m_pGeom[i]->GetBoundingSphereRadius());
}

void vpBody::UpdateGeomFrame(void)
{
	for ( int i = 0; i < m_pGeom.size(); i++ ) m_pGeom[i]->UpdateGlobalFrame();
}

int vpBody::GetID(void) const
{
	if ( m_pWorld )
		for ( int i = 0; i < m_pWorld->m_pBody.size(); i++ )
			if ( m_pWorld->m_pBody[i] == this ) 
				return i;
	
	return -1;
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpBody.inl>
#endif

