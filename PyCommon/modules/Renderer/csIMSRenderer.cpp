#pragma once

#include "stdafx.h"

//#include <windows.h> 
#include <GL/gl.h>
#include <GL/glu.h>
//#include <gl/glut.h>
 
#include "../../common_sources/bputil.h"
#include "../../external_libraries/implicitMassSpringSolver2/Physics.h"

#include "csIMSRenderer.h"
#include "../Simulator/csIMSModel.h"

BOOST_PYTHON_MODULE(csIMSRenderer)
{
	class_<IMSModelRenderer>("IMSModelRenderer", init<IMSModel*>())
		.def(init<IMSModel*, const tuple&>())
		.def(init<IMSModel*, const tuple&, bool>())
		.def(init<IMSModel*, const tuple&, const dict&>())
		.def("render", &IMSModelRenderer::render)
		;
}

IMSModelRenderer::IMSModelRenderer(IMSModel* pModel)
{
	_IMSModelRenderer(pModel, make_tuple(255,255,255), false, dict());
}

IMSModelRenderer::IMSModelRenderer( IMSModel* pModel, const tuple& color )
{
	_IMSModelRenderer(pModel, color, false, dict());
}

IMSModelRenderer::IMSModelRenderer( IMSModel* pModel, const tuple& color, bool drawParticles )
{
	_IMSModelRenderer(pModel, color, drawParticles, dict());
}
IMSModelRenderer::IMSModelRenderer( IMSModel* pModel, const tuple& color, const dict& colorMap )
{
	_IMSModelRenderer(pModel, color, false, colorMap);
}
void IMSModelRenderer::_IMSModelRenderer(IMSModel* pModel, const tuple& color, bool drawParticles, const dict& colorMap)
{
	_pModel = pModel;

	_color[0] = XD(color[0]);
	_color[1] = XD(color[1]);
	_color[2] = XD(color[2]);

	_drawParticles = drawParticles;

	_colorMap = colorMap;
}
void IMSModelRenderer::render()
{
	// particle
	if(_drawParticles)
	{
		glColor3ubv(_color);
		glPointSize(3.);
		glBegin(GL_POINTS);
		for(int i=0; i<_pModel->_pSystem->m_iParticles; ++i)
		{
			Physics_Vector3& pos = _pModel->_pSystem->Position(i);
			glVertex3d(pos.x, pos.y, pos.z);
		}
		glEnd();
	}

	// spring
	glBegin(GL_LINES);
	list items = _pModel->_subspringsMap.items();
	for(int i=0; i<len(items); ++i)
	{
		string subspringName = XS(items[i][0]);
		list springIndices = (list)items[i][1];

		GLubyte color[3];
		if(_colorMap.has_key(subspringName))
		{
			color[0] = XI(_colorMap[subspringName][0]);
			color[1] = XI(_colorMap[subspringName][1]);
			color[2] = XI(_colorMap[subspringName][2]);
		}
		else
		{
			color[0] = _color[0];
			color[1] = _color[1];
			color[2] = _color[2];
		}

		if(subspringName == "__MUSCLE__")
		{
		}
		else
		{
			glColor3ubv(color);
			for(int j=0; j<len(springIndices); ++j)
			{
				int si = XI(springIndices[j]);

				Physics_Vector3& pos1 = _pModel->_pSystem->Position(_pModel->_springs[si]->m_iParticle[0]);
				Physics_Vector3& pos2 = _pModel->_pSystem->Position(_pModel->_springs[si]->m_iParticle[1]);
				glVertex3d(pos1.x, pos1.y, pos1.z);
				glVertex3d(pos2.x, pos2.y, pos2.z);
			}
		}
	}
	glEnd();
}

