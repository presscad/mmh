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

#ifndef _VP_BASIC_RENDERER_
#define _VP_BASIC_RENDERER_

#ifdef _WIN64
	#include <GL/freeglut.h>
#else
	#include <GL/glut.h>
#endif

#include <stdarg.h>

#define VP_BASIC_RENDERER_WORLD(world) { _pWorld = &world; }

void initialize(void);
void frame(void);
void keyboard(unsigned char key, int x, int y);

static float _view_matrix[] = { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f }; 
static float _mat_col[] = { 0.8f, 0.8f, 0.8f, 1.0f };
static int _mouse_ctrl = 0;
static int _x_i, _y_i;
static float _v[3];
static float _init_view_matrix[16];
static GLdouble world_radius;
static Vec3 world_center;
static vpWorld *_pWorld = NULL;

void printf(int x, int y, char *string, ...)
{
	if ( !string ) return;

	static char text[1024], *c;
	va_list va;
	va_start(va, string);
	vsprintf(text, string, va);
	va_end(va);

	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0.0, (double)glutGet(GLUT_WINDOW_WIDTH), 0.0, (double)glutGet(GLUT_WINDOW_HEIGHT), -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRasterPos2i(x, y);
	for ( c = text; *c != '\0'; c++ ) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *c);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);	
	glEnable(GL_LIGHTING);
}


inline float square(const float p)
{
	return p * p;
}

void resetPerspective(void)
{
	float dist = sqrt(	square(_view_matrix[0] * (float)world_center[0] + _view_matrix[4] * (float)world_center[1] + _view_matrix[ 8] * (float)world_center[2] + _view_matrix[12]) +
						square(_view_matrix[1] * (float)world_center[0] + _view_matrix[5] * (float)world_center[1] + _view_matrix[ 9] * (float)world_center[2] + _view_matrix[13]) +
						square(_view_matrix[2] * (float)world_center[0] + _view_matrix[6] * (float)world_center[1] + _view_matrix[10] * (float)world_center[2] + _view_matrix[14]) );

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (double)glutGet(GLUT_WINDOW_WIDTH) / (double)glutGet(GLUT_WINDOW_HEIGHT), max(1.0f, dist - world_radius), dist + world_radius);
	glMatrixMode(GL_MODELVIEW);	
}

void reshapeFunc(int w, int h)
{
	resetPerspective();
	glViewport(0, 0, w, h);
}

float dot(const float v[3], const float w[3])
{
	return v[0] * w[0] + v[1] * w[1] + v[2] * w[2];
}

void p2v(float v[3], int x, int y)
{
	float p[3];

	p[0] = 2.0f * (float)x / (float)glutGet(GLUT_WINDOW_WIDTH) - 1.0f;
	p[0] = min(max(p[0], -1.0f), 1.0f);
	p[1] = 1.0f - 2.0f * (float)y / (float)glutGet(GLUT_WINDOW_HEIGHT);
	p[1] = min(max(p[1], -1.0f), 1.0f);
	p[2] = p[0] * p[0] + p[1] * p[1];

	if ( p[2] > 1.0f ) 
	{
		p[2] = sqrt(p[2]);
		p[0] /= p[2];
		p[1] /= p[2];
		p[2] = 0.0f;
	} else
		p[2] = sqrt(1.0f - p[2]);

	v[0] = dot(_init_view_matrix + 0, p);
	v[1] = dot(_init_view_matrix + 4, p);
	v[2] = dot(_init_view_matrix + 8, p);
}

void mouseFunc(int button, int state, int x, int y)
{
	if ( state == GLUT_DOWN )
	{
		_x_i = x;
		_y_i = y;		
		memcpy(_init_view_matrix, _view_matrix, sizeof(_view_matrix));		

		switch ( button )
		{
		case GLUT_LEFT_BUTTON:
			p2v(_v, x, y);
			_mouse_ctrl += 1;
			break;
		case GLUT_RIGHT_BUTTON:
			_mouse_ctrl += 2;
			break;
		}
	} else
		_mouse_ctrl = 0;
}

void motionFunc(int x, int y)
{
	if ( _mouse_ctrl == 1 )
	{
		float _w[3];
		p2v(_w, x, y);		
		glPushMatrix();
		glLoadMatrixf(_init_view_matrix);
		glRotatef(180.0f / (float)M_PI * acos(max(-1.0f, min(1.0f, dot(_v, _w)))), _v[1] * _w[2] - _v[2] * _w[1], _v[2] * _w[0] - _v[0] * _w[2], _v[0] * _w[1] - _v[1] * _w[0]);
		glGetFloatv(GL_MODELVIEW_MATRIX, _view_matrix);
		glPopMatrix();
	} else if ( _mouse_ctrl == 2 )
	{
		float dist = sqrt(dot(_view_matrix + 12, _view_matrix + 12));
		_view_matrix[12] = _init_view_matrix[12] + dist * (float)(x - _x_i) / (float)glutGet(GLUT_WINDOW_WIDTH);
		_view_matrix[13] = _init_view_matrix[13] - dist * (float)(y - _y_i) / (float)glutGet(GLUT_WINDOW_HEIGHT);
	} else // _mouse_ctrl == 3
	{
		float z = 1.0f + (float)(y - _y_i) / (float)glutGet(GLUT_WINDOW_HEIGHT);
		_view_matrix[12] = z * _init_view_matrix[12];
		_view_matrix[13] = z * _init_view_matrix[13];
		_view_matrix[14] = z * _init_view_matrix[14];
	}

	resetPerspective();
}

#define _SLICE_SIZE 32
void draw_torus(const scalar &ring_rad, const scalar &tube_rad)
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

void _init(void)
{
	initialize();

	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	
	world_radius = 1.0f * _pWorld->GetBoundingSphere(world_center);

	if ( _view_matrix[14] == 0.0f ) _view_matrix[14] = -2.0f * ((float)world_radius + 1.0f);

	GLdouble _T[16], _val[3];
	char type;

	GLUquadricObj *qobj = gluNewQuadric();

	for ( int i = 0; i < _pWorld->GetNumBody(); i++ )
	{
		_pWorld->GetBody(i)->m_iDisplayList = glGenLists(1);
		glNewList(_pWorld->GetBody(i)->m_iDisplayList, GL_COMPILE);
		for ( int j = 0; j < 3; j++ ) _mat_col[j] = 0.3f + 0.5f * (float)rand() / (float)RAND_MAX;
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, _mat_col);
		for ( int j = 0; j < _pWorld->GetBody(i)->GetNumGeometry(); j++ )
		{
			_pWorld->GetBody(i)->GetGeometry(j)->GetShape(&type, _val);
			_pWorld->GetBody(i)->GetGeometry(j)->GetLocalFrame().ToArray(_T);

			glPushMatrix();
			glMultMatrixd(_T);
			switch ( type )
			{
			case 'S':
				gluSphere(qobj, _val[0], 8, 8);
				break;
			case 'B':
				glScaled(_val[0], _val[1], _val[2]);
				glutSolidCube(1.0);
				break;
			case 'C':
				_val[1] -= 2.0 * _val[0];
				glTranslated(0.0, 0.0, -SCALAR_1_2 * _val[1]);
				gluSphere(qobj, _val[0], 12, 12);
				gluCylinder(qobj, _val[0], _val[0], _val[1], 12, 1);
				glTranslated(0.0, 0.0, _val[1]);
				gluSphere(qobj, _val[0], 12, 12);
				break;
			case 'P':
				{
				Axis Z(SCALAR_0, SCALAR_0, SCALAR_1);
				Axis N = Normalize(Axis(_val[0], _val[1], _val[2]));
				
				SE3 T = Exp(Cross(Z, N), acos(Inner(Z, N)));
				T.ToArray(_T);
				glPushMatrix();
				glMultMatrixd(_T);
				
				float sz[] = { 10.0f, 10.0f, 0.0f };
				float data[][8] = {	{  0.0f,  0.0f,  0.0f,  0.0f,  1.0f, -sz[0], -sz[1],  sz[2] },
									{  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,  sz[0], -sz[1],  sz[2] },
									{  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,  sz[0],  sz[1],  sz[2] },
									{  0.0f,  1.0f,  0.0f,  0.0f,  1.0f, -sz[0],  sz[1],  sz[2] } };

				glInterleavedArrays(GL_T2F_N3F_V3F, 0, data);
				glDrawArrays(GL_QUADS, 0, 4);
				glPopMatrix();
				}
				break;
			case 'L':
				glPushMatrix();
					glTranslated(0.0, 0.0, -0.5 * _val[1]);
					gluCylinder(qobj, _val[0], _val[0], _val[1], 32, 1); 
					glTranslated(0.0, 0.0, _val[1]);
					gluDisk(qobj, 0.0, _val[0], 32, 1);
					glTranslated(0.0, 0.0, -_val[1]);
					glRotated(180.0, 1.0, 0.0, 0.0);
					gluDisk(qobj, 0.0, _val[0], 32, 1);
				glPopMatrix();
				break;
			case 'T':
				draw_torus(_val[0], _val[1]);
				break;
			}
			glPopMatrix();
		}
		glEndList();
	}
}

void _displayFunc(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	glLoadMatrixf(_view_matrix);

	GLfloat _T[16];
	
	frame();

	for ( int i = 0; i < _pWorld->GetNumBody(); i++ )
	{
		_pWorld->GetBody(i)->GetFrame().ToArray(_T);
		glPushMatrix();
		glMultMatrixf(_T);
		glCallList(_pWorld->GetBody(i)->m_iDisplayList);
		glPopMatrix();
	}

	glutSwapBuffers();	
}

void idleFunc(void)
{
	glutPostRedisplay();
}

void specialFunc(int key, int x, int y)
{
	switch ( key )
	{
	case GLUT_KEY_LEFT:
		break;
	case GLUT_KEY_RIGHT:
		break;
	case GLUT_KEY_DOWN:
		break;
	case GLUT_KEY_UP:
		break;
	case GLUT_KEY_PAGE_DOWN:
		break;
	case GLUT_KEY_PAGE_UP:
		break;
	}
}

void keyboardFunc(unsigned char key, int x, int y)
{
	switch ( key )
	{
	case 27:
		exit(0);
		break;
	case 'V':
		printf("%3.2f", _view_matrix[0]);
		for ( int i = 1; i < 16; i++ ) printf(", %3.2f", _view_matrix[i]);
		printf("\n");
		break;
	default:
		keyboard(key, x, y);
	}
}

void set_view(double v0, double v1, double v2, double v3, double v4, double v5, double v6, double v7, double v8, double v9, double v10, double v11, double v12, double v13, double v14, double v15)
{
	_view_matrix[0] = (float)v0;
	_view_matrix[1] = (float)v1;
	_view_matrix[2] = (float)v2;
	_view_matrix[3] = (float)v3;
	_view_matrix[4] = (float)v4;
	_view_matrix[5] = (float)v5;
	_view_matrix[6] = (float)v6;
	_view_matrix[7] = (float)v7;
	_view_matrix[8] = (float)v8;
	_view_matrix[9] = (float)v9;
	_view_matrix[10] = (float)v10;
	_view_matrix[11] = (float)v11;
	_view_matrix[12] = (float)v12;
	_view_matrix[13] = (float)v13;
	_view_matrix[14] = (float)v14;
	_view_matrix[15] = (float)v15;
}

int main(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);

	glutInitWindowSize(512, 512);
	glutCreateWindow("VirtualPhysics");

	glutKeyboardFunc(keyboardFunc);
	glutSpecialFunc(specialFunc);
	glutDisplayFunc(_displayFunc);
	glutReshapeFunc(reshapeFunc);
	glutIdleFunc(idleFunc);
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);

	_init();

	glutMainLoop();

	return 0;
}

#endif
