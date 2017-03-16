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

#define _USE_MATH_DEFINES
#include <math.h>

#include "vpRenderer.h"
#include <VP/vpCollisionDetector.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <algorithm>

#define _SLICE_SIZE		12

void _draw_sphere(const scalar &rad);
void _draw_box(const scalar sz[3]);
void _draw_capsule(const scalar &rad, const scalar &height);
void _draw_cylinder(const scalar &rad, const scalar &height);
void _draw_plane(const scalar normal[3]);
void _draw_torus(const scalar &ring_rad, const scalar &tube_rad);

#ifdef SCALAR_AS_DOUBLE
#define glMultMatrix glMultMatrixd
#else if
#define glMultMatrix glMultMatrixf
#endif

static scalar _T[16];
static GLUquadricObj *_sqobj = NULL;

vpRenderer::vpRenderer()
{
	m_bDrawContactPoints = false;
	m_bInitialize = false;
	m_pWorld = NULL;
	
	if ( !_sqobj )
	{
		_sqobj = gluNewQuadric();
		gluQuadricTexture(_sqobj, GL_TRUE);
	}
}

vpRenderer::~vpRenderer()
{
	if ( m_pWorld && m_bInitialize )
		for ( int i = 0; i < m_pWorld->GetNumBody(); i++ ) glDeleteLists(m_sDisplayListMap[m_pWorld->GetBody(i)], 1);
}

void vpRenderer::SetTarget(const vpWorld *pWorld)
{
	m_pWorld = pWorld;
	m_bInitialize = false;
}

void vpRenderer::_Initialize(void)
{
	if ( m_pWorld && m_bInitialize )
		for ( int i = 0; i < m_pWorld->GetNumBody(); i++ ) glDeleteLists(m_sDisplayListMap[m_pWorld->GetBody(i)], 1);

	m_bInitialize = true;

	if ( !m_pWorld ) return;

	const vpBody *pBody;
	const vpGeom *pGeom;
	char type;
	scalar data[3];

	for ( unsigned int i = 0; i < m_sBodyGroup.size(); i++ ) m_sBodyGroup[i].bodyPair.resize(0);

	m_sBodyGroup.resize(1);
	m_sBodyGroup.back().pMaterial = NULL;
	for ( int i = 0; i < m_pWorld->GetNumBody(); i++ )
	{
		pBody = m_pWorld->GetBody(i);
		unsigned int displaylist = glGenLists(1);
		glNewList(displaylist, GL_COMPILE);
		for ( int j = 0; j < pBody->GetNumGeometry(); j++ )
		{
			pGeom = pBody->GetGeometry(j);
			glPushMatrix();
			pGeom->GetLocalFrame().ToArray(_T);
			glMultMatrix(_T);
			
			pGeom->GetShape(&type, data);
			switch ( type )
			{
			case 'B':
				data[0] *= SCALAR_1_2;
				data[1] *= SCALAR_1_2;
				data[2] *= SCALAR_1_2;
				_draw_box(data);
				break;
			case 'C':
				data[1] -= SCALAR_2 * data[0];
				_draw_capsule(data[0], data[1]);
				break;
			case 'S':
				_draw_sphere(data[0]);
				break;
			case 'P':
				_draw_plane(data);
				break;
			case 'L':
				_draw_cylinder(data[0], data[1]);
				break;
			case 'T':
				_draw_torus(data[0], data[1]);
				break;
			}
			glPopMatrix();
		}
		glEndList();
		m_sDisplayListMap[pBody] = displaylist;
		
		BODY_PAIR bpair = { pBody, displaylist };
		m_sBodyGroup.back().bodyPair.push_back(bpair);	

		m_sBodyGroupMap[pBody] = 0;
	}
}

void vpRenderer::SetMaterial(const vpBody *pBody, glMaterial *pMaterial)
{
	if ( !m_pWorld ) return;
	if ( ! m_bInitialize ) _Initialize();

	bool found = false;
	for ( unsigned int i = 0; i < m_sBodyGroup.size(); i++ )
	{
		if ( m_sBodyGroup[i].pMaterial == pMaterial )
		{
			int idx = m_sBodyGroupMap[pBody];
			vector<BODY_PAIR>::iterator itor = m_sBodyGroup[idx].bodyPair.begin();
			while ( itor != m_sBodyGroup[idx].bodyPair.end() )
			{
				if ( itor->pBody == pBody ) itor = m_sBodyGroup[idx].bodyPair.erase(itor);
				else itor++;
			}			

			BODY_PAIR bpair = { pBody,  m_sDisplayListMap[pBody] };
			m_sBodyGroup[i].bodyPair.push_back(bpair);
			
			m_sBodyGroupMap[pBody] = i;

			found = true;
		}
	}
	if ( !found )
	{
		int idx = m_sBodyGroupMap[pBody];
		vector<BODY_PAIR>::iterator itor = m_sBodyGroup[idx].bodyPair.begin();
		while ( itor != m_sBodyGroup[idx].bodyPair.end() )
		{
			if ( itor->pBody == pBody ) itor = m_sBodyGroup[idx].bodyPair.erase(itor);
			else itor++;
		}			

		BODY_PAIR bpair = { pBody,  m_sDisplayListMap[pBody] };
		m_sBodyGroup.resize(m_sBodyGroup.size() + 1);
		m_sBodyGroup.back().bodyPair.push_back(bpair);
		m_sBodyGroup.back().pMaterial = pMaterial;

		m_sBodyGroupMap[pBody] = (int)m_sBodyGroup.size() - 1;
	}
}

void vpRenderer::DrawContacPoints(bool flag)
{
	m_bDrawContactPoints = flag;
}

void vpRenderer::Render(bool apply_material)
{
	if ( !m_pWorld ) return;

	if ( ! m_bInitialize ) _Initialize();

	for ( unsigned int i = 0; i < m_sBodyGroup.size(); i++ )
	{
		if ( apply_material && m_sBodyGroup[i].pMaterial ) m_sBodyGroup[i].pMaterial->enable();
		for ( unsigned int j = 0; j < m_sBodyGroup[i].bodyPair.size(); j++ )
		{
			glPushMatrix();
			m_sBodyGroup[i].bodyPair[j].pBody->GetFrame().ToArray(_T);
			glMultMatrix(_T);
				glCallList(m_sBodyGroup[i].bodyPair[j].iDisplayList);
			glPopMatrix();
		}
		if ( apply_material && m_sBodyGroup[i].pMaterial ) m_sBodyGroup[i].pMaterial->disable();
	}
	
	if ( m_bDrawContactPoints )
	{
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
/*		for ( int i = 0; i < m_pWorld->m_pCollisionDetector->GetCollisionList().size(); i++ )
			for ( intPairList::iterator jtor = m_pWorld->m_pCollisionDetector->GetCollisionList()[i].begin(); jtor != m_pWorld->m_pCollisionDetector->GetCollisionList()[i].end(); jtor++ )
				glVertex3d(jtor->point[0], jtor->point[1], jtor->point[2]);		
	*/	glEnd();
		glEnable(GL_LIGHTING);
	}
	
	glDisable(GL_DEPTH_TEST);
	glText::print(10, 10, m_szCaption);
	glEnable(GL_DEPTH_TEST);
}

void vpRenderer::SetCaption(const char buf[])
{
	strcpy(m_szCaption, buf);
}

void _draw_box(const scalar _sz[3])
{
	float sz[3]		= { (float)_sz[0], (float)_sz[1], (float)_sz[2] };

	float data[][8] = {	{  0.0f,  0.0f,  0.0f,  0.0f,  1.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,  sz[0], -sz[1],  sz[2] },
						{  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  sz[0],  sz[1],  sz[2] },
						{  0.0f,  1.0f,  0.0f,  0.0f,  1.0f, -sz[0],  sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  0.0f, -1.0f, -sz[0], -sz[1], -sz[2] },
						{  1.0f,  1.0f,  0.0f,  0.0f, -1.0f, -sz[0],  sz[1], -sz[2] },
						{  0.0f,  1.0f,  0.0f,  0.0f, -1.0f,  sz[0],  sz[1], -sz[2] },
						{  0.0f,  0.0f,  0.0f,  0.0f, -1.0f,  sz[0], -sz[1], -sz[2] },
						{  0.0f,  1.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1], -sz[2] },
						{  0.0f,  0.0f,  0.0f,  1.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
						{  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
						{  1.0f,  1.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
						{  0.0f,  1.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
						{  0.0f,  0.0f,  0.0f, -1.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f, -1.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1], -sz[2] },
						{  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1], -sz[2] },
						{  0.0f,  1.0f,  1.0f,  0.0f,  0.0f,  sz[0],  sz[1],  sz[2] },
						{  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  sz[0], -sz[1],  sz[2] },
						{  0.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1], -sz[2] },
						{  1.0f,  0.0f, -1.0f,  0.0f,  0.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1],  sz[2] },
						{  0.0f,  1.0f, -1.0f,  0.0f,  0.0f, -sz[0],  sz[1], -sz[2] }	};

	glInterleavedArrays(GL_T2F_N3F_V3F, 0, data);
	glDrawArrays(GL_QUADS, 0, 24);
}

void _draw_plane(const scalar _normal[3])
{
	Axis Z(SCALAR_0, SCALAR_0, SCALAR_1);
	Axis N = Normalize(Axis(_normal));
	
	SE3 T = Exp(Cross(Z, N), acos(Inner(Z, N)));
	T.ToArray(_T);
	glPushMatrix();
	glMultMatrix(_T);
	
	float normal[3]		= { (float)_normal[0], (float)_normal[1], (float)_normal[2] };
	float sz[] = { 100.0f, 100.0f, 0.0f };
	float data[][8] = {	{  0.0f,  0.0f,  0.0f,  0.0f,  1.0f, -sz[0], -sz[1],  sz[2] },
						{  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,  sz[0], -sz[1],  sz[2] },
						{  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  sz[0],  sz[1],  sz[2] },
						{  0.0f,  1.0f,  0.0f,  0.0f,  1.0f, -sz[0],  sz[1],  sz[2] } };

	glInterleavedArrays(GL_T2F_N3F_V3F, 0, data);
	glDrawArrays(GL_QUADS, 0, 4);
	glPopMatrix();
}

void _draw_sphere(const scalar &rad)
{
/*	glBegin(GL_QUAD_STRIP);
	for ( int i = 0; i < _SLICE_SIZE - 1; i++ )
	{
		scalar t = (scalar)i / (scalar)(_SLICE_SIZE - 1);
		scalar t2 = (scalar)(i + 1) / (scalar)(_SLICE_SIZE - 1);
		scalar cp = cos((scalar)M_PI * t - (scalar)M_PI_2);
		scalar sp = sin((scalar)M_PI * t - (scalar)M_PI_2);
		scalar cp2 = cos((scalar)M_PI * t2 - (scalar)M_PI_2);
		scalar sp2 = sin((scalar)M_PI * t2 - (scalar)M_PI_2);

		for ( int j = 0; j < _SLICE_SIZE; j++ )
		{
			scalar s = (scalar)j / (scalar)(_SLICE_SIZE - 1);
			scalar ct = cos(M_2PI * s);
			scalar st = sin(M_2PI * s);

			glTexCoord2d(s, t);
			glNormal3d(cp * ct, sp,  -cp * st);
			glVertex3d(rad * cp * ct, rad * sp,  -rad * cp * st);

			glTexCoord2d(s, t2);
			glNormal3d(cp2 * ct, sp2,  -cp2 * st);
			glVertex3d(rad * cp2 * ct, rad * sp2,  -rad * cp2 * st);
		}
	}
	glEnd();
*/
	static GLUquadricObj *_sqobj = NULL;
	if ( !_sqobj )
	{
		_sqobj = gluNewQuadric();
		gluQuadricTexture(_sqobj, GL_TRUE);
	}

	gluSphere(_sqobj, rad, _SLICE_SIZE, _SLICE_SIZE);
}

void _draw_capsule(const scalar &rad, const scalar &height)
{
	int i, j;
	scalar ct_i, st_i, ct_im1 = SCALAR_1, st_im1 = SCALAR_0, cp_i, sp_i, cp_im1, sp_im1;

	glBegin(GL_QUADS);
	for ( i = 1; i < _SLICE_SIZE + 1; i++ )
	{
		ct_i = cos(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);
		st_i = sin(M_2PI * (scalar)i / (scalar)_SLICE_SIZE);

		glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)SCALAR_0);
		glNormal3d(ct_im1, st_im1, SCALAR_0);
		glVertex3d(rad * ct_im1, rad * st_im1, -SCALAR_1_2 * height);
		glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_0);
		glNormal3d(ct_i, st_i, SCALAR_0);
		glVertex3d(rad * ct_i, rad * st_i, -SCALAR_1_2 * height);
		glTexCoord2d((double)i / (double)_SLICE_SIZE, SCALAR_1);
		glNormal3d(ct_i, st_i, SCALAR_0);
		glVertex3d(rad * ct_i, rad * st_i, SCALAR_1_2 * height);
		glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1);
		glNormal3d(ct_im1, st_im1, SCALAR_0);
		glVertex3d(rad * ct_im1, rad * st_im1, SCALAR_1_2 * height);

		cp_im1 = SCALAR_1;
		sp_im1 = SCALAR_0;
		for ( j = 1; j < _SLICE_SIZE + 1; j++ )
		{
			cp_i = cos(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);
			sp_i = sin(SCALAR_1_2 * (scalar)M_PI * (scalar)j / (scalar)_SLICE_SIZE);

			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  sp_im1);
			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  sp_im1);
			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, rad * sp_im1 + SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_i, st_i * cp_i,  sp_i);
			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, rad * sp_i + SCALAR_1_2 * height);
			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, SCALAR_1 - (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  sp_i);
			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, rad * sp_i + SCALAR_1_2 * height);

			glTexCoord2d((double)(i - 1) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_im1, st_im1 * cp_im1,  -sp_im1);
			glVertex3d(rad * ct_im1 * cp_im1, rad * st_im1 * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j - 1) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_im1, st_i * cp_im1,  -sp_im1);
			glVertex3d(rad * ct_i * cp_im1, rad * st_i * cp_im1, -rad * sp_im1 - SCALAR_1_2 * height);
			glTexCoord2d((double)(i) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_i * cp_i, st_i * cp_i,  -sp_i);
			glVertex3d(rad * ct_i * cp_i, rad * st_i * cp_i, -rad * sp_i - SCALAR_1_2 * height);
			glTexCoord2d((double)(i-1) / (double)_SLICE_SIZE, (double)(j) / (double)_SLICE_SIZE);
			glNormal3d(ct_im1 * cp_i, st_im1 * cp_i,  -sp_i);
			glVertex3d(rad * ct_im1 * cp_i, rad * st_im1 * cp_i, -rad * sp_i - SCALAR_1_2 * height);
			
			cp_im1 = cp_i;
			sp_im1 = sp_i;
		}

		ct_im1 = ct_i;
		st_im1 = st_i;
	}
	glEnd();
}

void _draw_cylinder(const scalar &rad, const scalar &height)
{
	glPushMatrix();
		glTranslated(0.0, 0.0, -0.5 * height);
		gluCylinder(_sqobj, rad, rad, height, _SLICE_SIZE, 1); 
		glTranslated(0.0, 0.0, height);
		gluDisk(_sqobj, 0.0, rad, _SLICE_SIZE, 1);
		glTranslated(0.0, 0.0, -height);
		glRotated(180.0, 1.0, 0.0, 0.0);
		gluDisk(_sqobj, 0.0, rad, _SLICE_SIZE, 1);
	glPopMatrix();
}

void _draw_torus(const scalar &ring_rad, const scalar &tube_rad)
{
	glBegin(GL_QUAD_STRIP);
	for ( int i = 0; i < _SLICE_SIZE - 1; i++ )
	{
		scalar v = (scalar)i / (scalar)(_SLICE_SIZE - 1);
		scalar v2 = (scalar)(i + 1) / (scalar)(_SLICE_SIZE - 1);
		scalar cv = cos(M_2PI * v);
		scalar sv = sin(M_2PI * v);
		scalar cv2 = cos(M_2PI * v2);
		scalar sv2 = sin(M_2PI * v2);

		for ( int j = 0; j < _SLICE_SIZE; j++ )
		{
			scalar u = (scalar)j / (scalar)(_SLICE_SIZE - 1);
			scalar cu = cos(M_2PI * u);
			scalar su = sin(M_2PI * u);

			glTexCoord2d(u, v);
			glNormal3d(cv * cu, cv * su, sv);
			glVertex3d((ring_rad + tube_rad * cv) * cu, (ring_rad + tube_rad * cv) * su,  tube_rad * sv);

			glTexCoord2d(u, v2);
			glNormal3d(cv2 * cu, cv2 * su, sv2);
			glVertex3d((ring_rad + tube_rad * cv2) * cu, (ring_rad + tube_rad * cv2) * su,  tube_rad * sv2);
		}
	}
	glEnd();
}