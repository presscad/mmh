#pragma once
//#include <windows.h>
#include <GL/gl.h>


const int POLYGON_LINE = 0;
const int POLYGON_FILL = 1;

const int RENDER_OBJECT = 0;
const int RENDER_SHADOW = 1;
const int RENDER_REFLECTION = 2;

class VpModel;

class VpModelRenderer
{
private:
	VpModel* _pModel;
	GLubyte _color[3];
	int _polygonStyle;
	double _lineWidth;

public:	// expose to python
	VpModelRenderer(VpModel* pModel, const tuple& color, int polygonStyle=POLYGON_FILL, double lineWidth=1.);
	void render(int renderType=RENDER_OBJECT);
};
