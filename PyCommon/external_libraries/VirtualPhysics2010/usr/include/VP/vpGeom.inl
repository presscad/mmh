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

VP_INLINE const SE3 &vpGeom::GetLocalFrame(void) const
{
	return m_sLocalFrame;
}

VP_INLINE const SE3 &vpGeom::GetGlobalFrame(void) const
{
	return m_sGlobalFrame;
}

VP_INLINE void vpBox::SetSize(const Vec3 &size)
{
	m_sHalfSize = SCALAR_1_2 * size;
}

VP_INLINE Vec3 vpBox::GetSize(void) const
{
	return SCALAR_2 * m_sHalfSize;
}

VP_INLINE const Vec3 &vpBox::GetHalfSize(void) const
{
	return m_sHalfSize;
}

VP_INLINE scalar vpBox::GetBoundingSphereRadius(void) const
{
	return Norm(m_sHalfSize);
}

VP_INLINE void vpBox::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'B';
	if ( data ) 
	{
		data[0] = SCALAR_2 * m_sHalfSize[0];
		data[1] = SCALAR_2 * m_sHalfSize[1];
		data[2] = SCALAR_2 * m_sHalfSize[2];
	}
}

VP_INLINE void vpSphere::SetRadius(scalar rad)
{
	m_rRadius = rad;
}

VP_INLINE scalar vpSphere::GetRadius(void) const
{
	return m_rRadius;
}

VP_INLINE scalar vpSphere::GetBoundingSphereRadius(void) const
{
	return m_rRadius;
}

VP_INLINE void vpSphere::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'S';
	if ( data ) data[0] = m_rRadius;
}

VP_INLINE scalar vpCapsule::GetRadius(void) const
{
	return m_rRadius;
}

VP_INLINE scalar vpCapsule::GetHeight(void) const
{
	return SCALAR_2 * (m_rHalfHeight + m_rRadius);
}

VP_INLINE void vpCapsule::SetSize(scalar radius, scalar height)
{
	m_rRadius = radius;
	m_rHalfHeight = SCALAR_1_2 * height - radius;
	if ( m_rHalfHeight < SCALAR_0 ) m_rHalfHeight = SCALAR_0;
}

VP_INLINE scalar vpCapsule::GetBoundingSphereRadius(void) const
{
	return m_rHalfHeight + m_rRadius;
}

VP_INLINE void vpCapsule::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'C';
	if ( data ) 
	{
		data[0] = m_rRadius;
		data[1] = SCALAR_2 * (m_rHalfHeight + m_rRadius);
	}
}

VP_INLINE const Vec3 &vpPlane::GetNormal(void) const
{
	return m_sNormal;
}

VP_INLINE void vpPlane::SetNormal(const Vec3 &normal)
{
	m_sNormal = normal;
}

VP_INLINE scalar vpPlane::GetBoundingSphereRadius(void) const
{
	return 10;//SCALAR_MAX;
}

VP_INLINE void vpPlane::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'P';
	if ( data ) 
	{
		data[0] = m_sNormal[0];
		data[1] = m_sNormal[1];
		data[2] = m_sNormal[2];
	}
}

VP_INLINE scalar vpCylinder::GetRadius(void) const
{
	return m_rRadius;
}

VP_INLINE scalar vpCylinder::GetHeight(void) const
{
	return SCALAR_2 * m_rHalfHeight;
}

VP_INLINE void vpCylinder::SetSize(scalar radius, scalar height)
{
	m_rRadius = radius;
	m_rHalfHeight = SCALAR_1_2 * height;
}

VP_INLINE scalar vpCylinder::GetBoundingSphereRadius(void) const
{
	return m_rHalfHeight + m_rRadius;
}

VP_INLINE void vpCylinder::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'L';
	if ( data ) 
	{
		data[0] = m_rRadius;
		data[1] = SCALAR_2 * m_rHalfHeight;
	}
}

VP_INLINE scalar vpTorus::GetRingRadius(void) const
{
	return m_rRingRadius;
}

VP_INLINE scalar vpTorus::GetTubeRadius(void) const
{
	return m_rTubeRadius;
}

VP_INLINE void vpTorus::SetSize(scalar ringRad, scalar tubeRad)
{
	m_rRingRadius = ringRad;
	m_rTubeRadius = tubeRad;
}

VP_INLINE scalar vpTorus::GetBoundingSphereRadius(void) const
{
	return m_rRingRadius + m_rTubeRadius;
}

VP_INLINE void vpTorus::GetShape(char *Type, scalar *data) const
{
	Type[0] = 'T';
	if ( data ) 
	{
		data[0] = m_rRingRadius;
		data[1] = m_rTubeRadius;
	}
}
