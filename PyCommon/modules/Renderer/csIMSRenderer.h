#pragma once
//#include <windows.h>
#include <GL/gl.h>

class IMSModel;
class IMSModelRenderer
{
public:
	IMSModel* _pModel;
	GLubyte _color[3];
	bool _drawParticles;
	dict _colorMap;
	void _IMSModelRenderer(IMSModel* pModel, const tuple& color, bool drawParticles, const dict& colorMap);

public:	// expose to python
	IMSModelRenderer(IMSModel* pModel);
	IMSModelRenderer(IMSModel* pModel, const tuple& color);
	IMSModelRenderer(IMSModel* pModel, const tuple& color, bool drawParticles);
	IMSModelRenderer(IMSModel* pModel, const tuple& color, const dict& colorMap);
	void render();
};
