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

#include <VP/vpGeom.h>
#include <VP/vpBody.h>
#include <VP/PrimColDet.h>
#include <VP/vpPrimitiveCollisionDetector.h>

vpGeom::vpGeom() : m_pBody(NULL)
{
}

void vpGeom::UpdateGlobalFrame()
{
	assert(m_pBody && "vpGeom::UpdateGlobalFrame -> there is no related body.");
	m_sGlobalFrame  = m_pBody->GetFrame();
	m_sGlobalFrame *= m_sLocalFrame;
}

vpBox::vpBox()
{
	m_sHalfSize = Vec3(SCALAR_1_2, SCALAR_1_2, SCALAR_1_2);
}

vpBox::vpBox(const Vec3 &size)
{
	m_sHalfSize = SCALAR_1_2 * size;
}

bool vpBox::DetectCollision(const vpGeom *pGeom, vpCollisionInfoArray &info) const
{
	return pGeom->DetectCollision(this, info);
}

bool vpBox::DetectCollision(const vpBox *box, vpCollisionInfoArray &info) const
{
	return ColDetBoxBox(m_sHalfSize, m_sGlobalFrame, box->m_sHalfSize, box->m_sGlobalFrame, info, vpPrimitiveCollisionDetector::GetMaxNumContact4Box());
}

bool vpBox::DetectCollision(const vpSphere *sphere, vpCollisionInfoArray &info) const
{
	bool re = sphere->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpBox::DetectCollision(const vpCapsule *pCapsule, vpCollisionInfoArray &info) const
{
	bool re = pCapsule->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpBox::DetectCollision(const vpPlane *pPlane, vpCollisionInfoArray &info) const
{
	return ColDetBoxPlane(m_sHalfSize, m_sGlobalFrame, pPlane->GetNormal(), pPlane->GetGlobalFrame(), info);
}

bool vpBox::DetectCollision(const vpCylinder *pCylinder, vpCollisionInfoArray &info) const
{
	bool re = pCylinder->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpBox::DetectCollision(const vpTorus *pTorus, vpCollisionInfoArray &info) const
{
	bool re = pTorus->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

Inertia vpBox::GetInertia(scalar density) const 
{
	return BoxInertia(density, m_sHalfSize); 
}

vpSphere::vpSphere()
{
	m_rRadius = SCALAR_1_2;
}

vpSphere::vpSphere(scalar rad)
{
	m_rRadius = rad;
}

bool vpSphere::DetectCollision(const vpGeom *pGeom, vpCollisionInfoArray &info) const
{
	return pGeom->DetectCollision(this, info);
}

bool vpSphere::DetectCollision(const vpBox *box, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetSphereBox(m_rRadius, m_sGlobalFrame, box->GetHalfSize(), box->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpSphere::DetectCollision(const vpSphere *sphere, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetSphereSphere(m_rRadius, m_sGlobalFrame, sphere->m_rRadius, sphere->m_sGlobalFrame, info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpSphere::DetectCollision(const vpCapsule *pCapsule, vpCollisionInfoArray &info) const
{
	bool re = pCapsule->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpSphere::DetectCollision(const vpPlane *pPlane, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetSpherePlane(m_rRadius, m_sGlobalFrame.GetPosition(), pPlane->GetNormal(), pPlane->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpSphere::DetectCollision(const vpCylinder *pCylinder, vpCollisionInfoArray &info) const
{
	bool re = pCylinder->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpSphere::DetectCollision(const vpTorus *pTorus, vpCollisionInfoArray &info) const
{
	bool re = pTorus->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

Inertia vpSphere::GetInertia(scalar density) const
{
	return SphereInertia(density, m_rRadius);
}

vpCapsule::vpCapsule()
{
	m_rRadius = SCALAR_1_2;
	m_rHalfHeight = SCALAR_1_2;
}

vpCapsule::vpCapsule(scalar radius, scalar height)
{
	m_rRadius = radius;
	m_rHalfHeight = SCALAR_1_2 * height - radius;
	if ( m_rHalfHeight < SCALAR_0 ) m_rHalfHeight = SCALAR_0;
}

bool vpCapsule::DetectCollision(const vpGeom *pGeom, vpCollisionInfoArray &info) const
{
	return pGeom->DetectCollision(this, info);
}

bool vpCapsule::DetectCollision(const vpBox *pBox, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetCapsuleBox(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pBox->GetHalfSize(), pBox->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpCapsule::DetectCollision(const vpSphere *pSphere, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetCapsuleSphere(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pSphere->GetRadius(), pSphere->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpCapsule::DetectCollision(const vpCapsule *pCapsule, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetCapsuleCapsule(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pCapsule->m_rRadius, pCapsule->m_rHalfHeight, pCapsule->m_sGlobalFrame, info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpCapsule::DetectCollision(const vpPlane *pPlane, vpCollisionInfoArray &info) const
{
	bool re = pPlane->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpCapsule::DetectCollision(const vpCylinder *pCylinder, vpCollisionInfoArray &info) const
{
	bool re = pCylinder->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpCapsule::DetectCollision(const vpTorus *pTorus, vpCollisionInfoArray &info) const
{
	bool re = pTorus->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

Inertia vpCapsule::GetInertia(scalar density) const
{
	return CylinderInertia(density, m_rRadius, SCALAR_2 * m_rHalfHeight) + SphereInertia(SCALAR_1_2 * density, m_rRadius).Transform(Vec3(SCALAR_0, SCALAR_0, m_rHalfHeight)) + SphereInertia(SCALAR_1_2 * density, m_rRadius).Transform(Vec3(SCALAR_0, SCALAR_0, -m_rHalfHeight));
}

vpPlane::vpPlane()
{
	m_sNormal = Vec3(SCALAR_0, SCALAR_0, SCALAR_1);
}

vpPlane::vpPlane(const Vec3 &normal)
{
	m_sNormal = normal;
	scalar mag = m_sNormal.Normalize();
	assert(mag > SCALAR_0 && "vpPlane::vpPlane(const Vec3 &normal) -> zero normal");
}

bool vpPlane::DetectCollision(const vpGeom *pGeom, vpCollisionInfoArray &info) const
{
	return pGeom->DetectCollision(this, info);
}

bool vpPlane::DetectCollision(const vpSphere *pSphere, vpCollisionInfoArray &info) const
{
	bool re = pSphere->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpPlane::DetectCollision(const vpBox *pBox, vpCollisionInfoArray &info) const
{
	bool re = pBox->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpPlane::DetectCollision(const vpCapsule *pCapsule, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpPlane::DetectCollision(const vpPlane *pPlane, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpPlane::DetectCollision(const vpCylinder *pCylinder, vpCollisionInfoArray &info) const
{
	bool re = pCylinder->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

bool vpPlane::DetectCollision(const vpTorus *pTorus, vpCollisionInfoArray &info) const
{
	bool re = pTorus->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

Inertia vpPlane::GetInertia(scalar density) const
{
	return Inertia(SCALAR_1);
}

vpCylinder::vpCylinder()
{
	m_rRadius = SCALAR_1_2;
	m_rHalfHeight = SCALAR_1;
}

vpCylinder::vpCylinder(scalar radius, scalar height)
{
	m_rRadius = radius;
	m_rHalfHeight = SCALAR_1_2 * height;
}

bool vpCylinder::DetectCollision(const vpGeom *pGeom, vpCollisionInfoArray &info) const
{
	return pGeom->DetectCollision(this, info);
}

bool vpCylinder::DetectCollision(const vpBox *pBox, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpCylinder::DetectCollision(const vpSphere *pSphere, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetCylinderSphere(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pSphere->GetRadius(), pSphere->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpCylinder::DetectCollision(const vpCapsule *pCapsule, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpCylinder::DetectCollision(const vpPlane *pPlane, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetCylinderPlane(m_rRadius, m_rHalfHeight, m_sGlobalFrame, pPlane->GetNormal(), pPlane->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpCylinder::DetectCollision(const vpCylinder *pCylinder, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpCylinder::DetectCollision(const vpTorus *pTorus, vpCollisionInfoArray &info) const
{
	bool re = pTorus->DetectCollision(this, info);
	if ( re ) info[info.size() - 1].normal *= -SCALAR_1;
	return re;
}

Inertia vpCylinder::GetInertia(scalar density) const
{
	return CylinderInertia(density, m_rRadius, SCALAR_2 * m_rHalfHeight);
}

vpTorus::vpTorus()
{
	m_rRingRadius = SCALAR_1;
	m_rTubeRadius = SCALAR_1_2;
}

vpTorus::vpTorus(scalar ring_radius, scalar tube_radius)
{
	m_rRingRadius = ring_radius;
	m_rTubeRadius = tube_radius;
}

bool vpTorus::DetectCollision(const vpGeom *pGeom, vpCollisionInfoArray &info) const
{
	return pGeom->DetectCollision(this, info);
}

bool vpTorus::DetectCollision(const vpBox *pBox, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpTorus::DetectCollision(const vpSphere *pSphere, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpTorus::DetectCollision(const vpCapsule *pCapsule, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpTorus::DetectCollision(const vpPlane *pPlane, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetTorusPlane(m_rRingRadius, m_rTubeRadius, m_sGlobalFrame, pPlane->GetNormal(), pPlane->GetGlobalFrame(), info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

bool vpTorus::DetectCollision(const vpCylinder *pCylinder, vpCollisionInfoArray &info) const
{
	return false;
}

bool vpTorus::DetectCollision(const vpTorus *pTorus, vpCollisionInfoArray &info) const
{
	int n = info.size();
	info.resize(n + 1);
	bool re = ColDetTorusTorus(m_rRingRadius, m_rTubeRadius, m_sGlobalFrame, pTorus->m_rRingRadius, pTorus->m_rTubeRadius, pTorus->m_sGlobalFrame, info[n].normal, info[n].point, info[n].penetration);
	if ( !re ) info.resize(n);
	return re;
}

Inertia vpTorus::GetInertia(scalar density) const
{
	return TorusInertia(density, m_rRingRadius, m_rTubeRadius);
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpGeom.inl>
#endif
