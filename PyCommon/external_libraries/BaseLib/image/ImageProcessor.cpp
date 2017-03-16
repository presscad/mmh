
// ImageProcessor.cpp: implementation of the CImageProcessor class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdimage.h"
#include "ImageProcessor.h"
#include "Image.h"
#include "ImagePixel.h"
#include "ImagePixelPtr.h"
#include "../math/mathclass.h"
#include "../utility/util.h"

int Imp::_private::g_nChartPrecision=256;

void Imp::ChangeChartPrecision(int precision)
{
	Imp::_private::g_nChartPrecision=precision;
}

void Imp::DefaultPrecision()
{
	Imp::_private::g_nChartPrecision=256;
}

namespace Imp
{

void SafeDelete(CImage* pImage, const char* filename)	
{ 
	if(pImage==NULL) 
		return; 
	pImage->Save(filename); 
	delete pImage;
}

CImage* RotateLeft(CImage *pInput)
{
	int width=pInput->GetWidth();
	int height=pInput->GetHeight();
	
	CImage* pOutput=new CImage();
	pOutput->Create(height,width, pInput->GetBitCount());

	if( pInput->GetBitCount() == 8 )
	{
		CPixelPtr inputptr(*pInput);
		CPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[width-i-1][j]=inputptr[j][i];
			}
		}
	}
	else 
	{
		CColorPixelPtr inputptr(*pInput);
		CColorPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[width-i-1][j]=inputptr[j][i];
			}
		}
	}
	return pOutput;
}

CImage* RotateRight(CImage *pInput)
{
	int width=pInput->GetWidth();
	int height=pInput->GetHeight();
	
	CImage* pOutput=new CImage();
	pOutput->Create(height,width, pInput->GetBitCount());

	if( pInput->GetBitCount() == 8 )
	{
		CPixelPtr inputptr(*pInput);
		CPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[i][height-j-1]=inputptr[j][i];
			}
		}
	}
	else 
	{
		CColorPixelPtr inputptr(*pInput);
		CColorPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[i][height-j-1]=inputptr[j][i];
			}
		}
	}
	return pOutput;
}

CImage* RotateHalf(CImage *pInput)
{
	int width=pInput->GetWidth();
	int height=pInput->GetHeight();
	
	CImage* pOutput=new CImage();
	pOutput->Create(width,height, pInput->GetBitCount());

	if( pInput->GetBitCount() == 8 )
	{
		CPixelPtr inputptr(*pInput);
		CPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[height-j-1][width-i-1]=inputptr[j][i];
			}
		}
	}
	else 
	{
		CColorPixelPtr inputptr(*pInput);
		CColorPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[height-j-1][width-i-1]=inputptr[j][i];
			}
		}
	}
	return pOutput;
}

void Crop(CImage* pOutput, CImage *pInput, const TRect &rect)
{
	ASSERT(pOutput);
	int width=rect.Width()+1;
	int height=rect.Height()+1;
	
	ASSERT(pOutput->GetWidth()==width);
	ASSERT(pOutput->GetHeight()==height);

	TRect inputRect(0,0, pInput->GetWidth(), pInput->GetHeight());
	POINT point;
	if( pInput->GetBitCount() == 8 )
	{
		CPixelPtr inputptr(*pInput);
		CPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				point.x=i+rect.left;
				point.y=j+rect.top;
				if(inputRect.PtInRect(point))
					outputptr[j][i]=inputptr[j+rect.top][i+rect.left];
				else
					outputptr[j][i]=0;
			}
		}
	}
	else 
	{
		CColorPixelPtr inputptr(*pInput);
		CColorPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				point.x=i+rect.left;
				point.y=j+rect.top;

				if(inputRect.PtInRect(point))
					outputptr[j][i]=inputptr[j+rect.top][i+rect.left];
				else
				{
					outputptr[j][i].R=0;
					outputptr[j][i].G=0;
					outputptr[j][i].B=0;
				}
				
			}
		}
	}
	
}

CImage* CropCentered(CImage *pInput, int width, int height)
{
	int hw=width/2;
	int hh=height/2;
	int hiw=pInput->GetWidth()/2;
	int hih=pInput->GetHeight()/2;
	return Crop(pInput,TRect(hiw-hw,hih-hh, hiw+hw, hih+hh));
}

CImage* Crop(CImage *pInput, const TRect &rect)
{
	CImage* pOutput=new CImage();

	int width=rect.Width()+1;
	int height=rect.Height()+1;

	pOutput->Create(width,height, pInput->GetBitCount());

	if( pInput->GetBitCount() == 8 )
	{
		CPixelPtr inputptr(*pInput);
		CPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[j][i]=inputptr[j+rect.top][i+rect.left];
			}
		}
	}
	else 
	{
		CColorPixelPtr inputptr(*pInput);
		CColorPixelPtr outputptr(*pOutput);
		
		for(int i=0; i<width; i++)
		{
			for(int j=0; j<height; j++)
			{
				outputptr[j][i]=inputptr[j+rect.top][i+rect.left];
			}
		}
	}
	return pOutput;
}

CImage* Clone(CImage* pInput)
{
	CImage* pOutput=Crop(pInput, TRect(0,0,pInput->GetWidth()-1, pInput->GetHeight()-1));
	return pOutput;
}


CImage* StitchHoriz(CImage* pLeft, CImage* pRight)
{
	CImage* pOut=new CImage();
	ASSERT(pLeft->GetHeight()==pRight->GetHeight());
	ASSERT(pLeft->GetBitCount()==pRight->GetBitCount());
	pOut->Create(pLeft->GetWidth()+pRight->GetWidth(), pLeft->GetHeight(), pLeft->GetBitCount());


	CImagePixel out(pOut);
    CImagePixel left(pLeft);
	CImagePixel right(pRight);

	out.DrawPattern(0,0, left);
	out.DrawPattern(pLeft->GetWidth(), 0, right);
	return pOut;
}


void DrawGraph(CImage* pInput, int numVertex, std::list<int>& listEdge)
{
	// vertex:0 to n , edge : integer 2개, 즉 list의 길이는 2의배수여야 함.
	ASSERT(pInput->GetWidth() > numVertex*2);
	ASSERT(pInput->GetHeight() > numVertex*2);
	ASSERT(pInput->GetBitCount()==8);

	CImagePixel cIP;

	int i;
	for(i=0; i<numVertex*2; i+=2)
	{
		// draw vertices
		cIP.SetPixel(i,i,RGB(255,255,255));
	}

	std::list<int>::iterator itr;


	for(itr=listEdge.begin(); itr!=listEdge.end(); )
	{
		int from=*itr;
		itr++;
		int to=*itr;
		itr++;

		if(from<to)
		{
			// 아래쪽으로 edge를 그린다.
			int length=to-from;

			// parameters are x, y, length
			cIP.DrawVertLine(from*2, from*2, length*2, RGB(255,255,255));
			cIP.DrawHorizLine(from*2, to*2, length*2, RGB(255,255,255));
		}
		else
		{
			// 위쪽으로 edge를 그린다.
			int length=from-to;

			// parameters are x, y, length
			cIP.DrawHorizLine(to*2, to*2, length*2, RGB(255,255,255));
			cIP.DrawVertLine(from*2, to*2, length*2, RGB(255,255,255));			
		}
	}	
}

int ToImageY(m_real y, m_real min, m_real max, int yoffset)
{
	#define M_ToImageY(x) (int(((x)-min)/(max-min)*((m_real)_private::g_nChartPrecision-1.f)+0.5f))
	int j=M_ToImageY(y);
	if(j<0) j=0;
	else if(j>(_private::g_nChartPrecision-1)) j=(_private::g_nChartPrecision-1);
	j=(_private::g_nChartPrecision-1)-j;
	j+=yoffset;
	return j;
}

void _private::DrawChart(CImage* pInput, int numFrame, m_real* aValue,float min, float max, COLORREF color, int xoffset, int yoffset, int xdelta,int chart_type)
{
	ASSERT(pInput->GetWidth() >= numFrame);
	ASSERT(pInput->GetHeight() >= _private::g_nChartPrecision+yoffset);
	ASSERT(pInput->GetBitCount()==24);

	// min이하는 pixel (_private::g_nChartPrecision-1)에 그려지고, max이상은 0에 그려진다. 즉 아래쪽이 min값

	CImagePixel inputptr(pInput);

	if(min<=0 && max>=0)
	{
		// draw y=0 line.
		int zero_y=ToImageY(0, min, max, yoffset);
		inputptr.DrawLine(0+xoffset, zero_y, xdelta*(numFrame-1)+xoffset,zero_y,RGB(128,128,128));
	}
	// draw top line
	inputptr.DrawLine( 0+xoffset, (_private::g_nChartPrecision-1)+yoffset, xdelta*(numFrame-1)+xoffset, (_private::g_nChartPrecision-1)+yoffset, RGB(128,128,128));

	char temp[100];
    sprintf(temp,"%f", min);
	inputptr.DrawText(0+xoffset,240+yoffset, temp);
	sprintf(temp,"%f", max);
	inputptr.DrawText(0+xoffset,yoffset, temp);

	

	vector3 vcolor;
	vcolor.x=GetRValue(color);
	vcolor.y=GetBValue(color);
	vcolor.z=GetBValue(color);

	vector3 white(255,255,255);
	vector3 hcolor;
	hcolor.interpolate(0.7, vcolor, white);
	
	COLORREF halfcolor=RGB(int(hcolor.x), int(hcolor.y), int(hcolor.z));


	int prev_x=-1, prev_y;
	m_real* itr;
	for(int i=0; i<numFrame; i++)
	{
		itr=&aValue[i];

		int j=ToImageY(*itr, min, max, yoffset);
		if(chart_type==LINE_CHART)
		{
			if(prev_x!=-1)
				inputptr.DrawLine( xdelta*prev_x+xoffset, prev_y, xdelta*i+xoffset, j, halfcolor);
			else
				inputptr.SetPixel( xdelta*i+xoffset, j, color);
		}
		else if(chart_type==BAR_CHART)
		{
			inputptr.DrawLine( xdelta*i+xoffset, (_private::g_nChartPrecision-1)+yoffset, xdelta*i+xoffset, j, color);
		}

		prev_x=i;
		prev_y=j;
	}

	if(chart_type==LINE_CHART)
	{
		int prev_x=-1, prev_y;
		m_real* itr;

		for(int i=0; i<numFrame; i++)
		{
			itr=&aValue[i];

			int j=ToImageY(*itr, min, max, yoffset);
			inputptr.SetPixel( xdelta*i+xoffset, j, color);

			prev_x=i;
			prev_y=j;
		}
	}
}

#define C0 0
#define C1 128
#define C2 255

COLORREF GetColor(int i)
{
	switch(i%26)
	{
	case 0:	return RGB(C0,C0,C1);
	case 1: return RGB(C0,C0,C2);	
	case 2: return RGB(C0,C1,C0);
	case 3: return RGB(C0,C1,C1);
	case 4: return RGB(C0,C1,C2);
	case 5: return RGB(C0,C2,C0);
	case 6: return RGB(C0,C2,C1);
	case 7: return RGB(C0,C2,C2);
	
	case 8: return RGB(C1,C0,C0);	
	case 9: return RGB(C1,C0,C1);
	case 10: return RGB(C1,C0,C2);
	case 11: return RGB(C1,C1,C0);
	case 12: return RGB(C1,C1,C1);	
	case 13: return RGB(C1,C1,C2);	
	case 14: return RGB(C1,C2,C0);
	case 15: return RGB(C1,C2,C1);	
	case 16: return RGB(C1,C2,C2);	
	
	case 17: return RGB(C2,C0,C0);
	case 18: return RGB(C2,C0,C1);	
	case 19: return RGB(C2,C0,C2);	
	case 20: return RGB(C2,C1,C0);
	case 21: return RGB(C2,C1,C1);	
	case 22: return RGB(C2,C1,C2);	
	case 23: return RGB(C2,C2,C0);
	case 24: return RGB(C2,C2,C1);	
	case 25: return RGB(C2,C2,C2);	
	}	
	return RGB(0,0,0);
}

CImage* Resize(CImage* pInput, int width, int height)
{
	CImagePixel cIP(pInput);
	CImage* pOutput=new CImage();
	pOutput->Create(width, height, pInput->GetBitCount());
	CImagePixel cOP(pOutput);
	float x,y;
	float x_ratio=(float)pInput->GetWidth()/(float)width;
	float y_ratio=(float)pInput->GetHeight()/(float)height;
	for(int i=0; i<width; i++)
	{
		for(int j=0; j<height; j++)
		{
			x=(float)i+0.5f;
			y=(float)j+0.5f;
			int count;
			cOP.SetPixel(i,j,cIP.GetPixel(x*x_ratio,y*y_ratio, count));
		}
	}
	return pOutput;
}

#include "../math/mathclass.h"

void SaveMatrix(matrixn& matrix, const char* filename)
{
	CImage* temp=DrawMatrix(matrix);
	temp->Save(filename);
	delete temp;
}

void LoadMatrix(matrixn& matrix, const char* filename)
{
	CImage temp;
	temp.Load(filename);
	CImagePixel cIP(&temp);

	matrix.setSize(temp.GetHeight(), temp.GetWidth());
	for(int i=0; i<temp.GetHeight(); i++)
	{
		for(int j=0; j<temp.GetWidth(); j++)
		{
			matrix[i][j]=GetRValue(cIP.GetPixel(j,i));
		}
	}
}

void SaveMatrixAndInfo(matrixn& matrix, const char* filename_prefix)
{
	m_real min, max;
	CImage* temp=DrawMatrix(matrix, min, max);
	TString filename;
	filename.format("%s_%f_%f.bmp",filename_prefix, min, max);
	temp->Save(filename);
	delete temp;
}


CImage* DrawMatrix(matrixn& matrix)
{
	m_real min,max;
	return DrawMatrix(matrix, min,max);
}

CImage* DrawMatrix(matrixn& matrix, m_real& min, m_real& max)
{
	min=matrix[0][0];
	max=matrix[0][0];
	
	int nrow=matrix.rows();
	int ncolumn=matrix.cols();
	for(int i=0; i<nrow; i++)
		for(int j=0; j<ncolumn;j++)
		{
			if(matrix[i][j]>max) max=matrix[i][j];
			if(matrix[i][j]<min) min=matrix[i][j];
		}

	TRACE("Draw2D min:%f max:%f\n", min, max);
	CImage *output=new CImage();
	output->Create(ncolumn, nrow, 24);

	CImagePixel outputptr(output);
	
	for(int i=0; i<nrow; i++)
		for(int j=0; j<ncolumn;j++)
		{
			int color=(int)((matrix[i][j]-min)*255.f/(max-min));
			outputptr.SetPixel(j,i, RGB(color, color, color));
		}

	return output;
}

CImage* DrawMatrix2(matrixn& matrix, m_real min, m_real max)
{
	int nrow=matrix.rows();
	int ncolumn=matrix.cols();
	
	printf("Draw2D min:%f max:%f\n", min, max);
	CImage *output=new CImage();
	output->Create(ncolumn, nrow, 24);
	
	CImagePixel outputptr(output);
	
	for(int i=0; i<nrow; i++)
		for(int j=0; j<ncolumn;j++)
		{
			int color=CLAMP((int)((matrix[i][j]-min)*255.f/(max-min)),0,255);
			outputptr.SetPixel(j,i, RGB(color, color, color));
		}

	return output;
}

CImage* Plot(const matrixn& samples, const vectorn& min, const vectorn& max)
{
	CImage* pInput=new CImage();

	int dim=samples.cols();

	int numPage=dim*(dim-1)/2;
	pInput->Create(512, 512*numPage,24);
	CImagePixel imagepixel(pInput);

	int curPage=0;
	for(int dim1=0; dim1<dim; dim1++)
		for(int dim2=dim1+1; dim2<dim; dim2++)
		{
			imagepixel.DrawHorizLine(0, curPage*512, 512, RGB(0,0,128));

			float rx=max[dim1]-min[dim1];
			float ry=max[dim2]-min[dim2];
			float minx=min[dim1];
			float miny=min[dim2];

			for(int i=0; i<samples.rows(); i++)
			{
				float x,y;
				x=(samples[i][dim1]-minx)/rx;
				y=(samples[i][dim2]-miny)/ry;

				if(x<0) x=0;
				if(x>1) x=1;
				if(y<0) y=0;
				if(y>1) y=1;

				imagepixel.SetPixel(x,(y+(float)curPage)/(float)numPage, RGB(255,255,255));
			}

			curPage++;
		}

	return pInput;
}


/// pInput을 2x2 matrix pTransf를 가지고 transform한 이미지를 return한다.
/*!
	(source_x,source_y)*matrix=(target_x,target_y)
*/
CImage* Transform(matrixn& transf, CImage* pInput)
{
	vectorn point[4];
	matrixn invTransf;
	//invTransf.SVinverse(transf);	이함수 및 SVinverse함수 전혀 동작 안함.-.-
	
	invTransf.transpose(transf);
	// ad-bc로 나눠준다.
	invTransf/=transf[0][0]*transf[1][1]-transf[0][1]*transf[1][0];

	int width=pInput->GetWidth();
	int height=pInput->GetHeight();

	point[0].multmat(vectorn(2, 0.5,0.5),transf);
	point[1].multmat(vectorn(2, width-0.5,0.5),transf);
	point[2].multmat(vectorn(2, 0.5,height-0.5),transf);
	point[3].multmat(vectorn(2, width-0.5,height-0.5),transf);

	matrixn points(4,2);
	for(int i=0; i<4; i++)
		points.row(i)=point[i];

	vectorn min,max,range;
	min.aggregateColumn(CAggregate::MINIMUM,points);
	max.aggregateColumn(CAggregate::MAXIMUM,points);
	range.sub(max,min);
	
	int nwidth,nheight;
	nwidth=int(range.x()+1.f);
	nheight=int(range.y()+1.f);

	CImage* pOutput=new CImage();
	pOutput->Create(nwidth,nheight,pInput->GetBitCount());

	CImagePixel cIIP(pInput);
	CImagePixel cOIP(pOutput);

	vectorn targetPoint(2);
	vectorn sourcePoint(2);
	int count;
	for(int i=0; i<nwidth; i++)
		for(int j=0; j<nheight; j++)
		{
			sourcePoint.x()=min.x()+i+0.5;
			sourcePoint.y()=min.y()+j+0.5;
			targetPoint.multmat(sourcePoint,invTransf);
			
			cOIP.SetPixel( i, j, cIIP.GetPixel(targetPoint.x(), targetPoint.y(), count));
		}

	return pOutput;
}

CImage* Rotate(float radian, CImage* pInput)
{
	matrixn temp;
	temp.setSize(2,2);
	temp[0][0]=cos(radian);
	temp[0][1]=sin(radian);
	temp[1][0]=-sin(radian);
	temp[1][1]=cos(radian);
	return Transform(temp, pInput);
}

CImage* DrawChart(const matrixn& matrix, int chart_type, float min, float max , float horizLine)	//!< 0 frame부터 n-1프레임까지의 여러 signal들을 그린다.)
{
	if(min==max)
	{
		vectorn temp;
		temp.minimum(matrix);
		min=temp.minimum();
		temp.maximum(matrix);
		max=temp.maximum();
	}

	interval range(min, max);
	range.expand(0.0001);

	vectorn aMin, aMax;
	aMin.setSize(matrix.rows());
	aMax.setSize(matrix.rows());
	aMin.setAllValue(range.start());
	aMax.setAllValue(range.end());

	if(horizLine!=FLT_MAX)
	{
		vectorn aLine;
		aLine.setSize(matrix.rows());
		aLine.setAllValue(horizLine);
		return DrawChart(matrix, chart_type, aMin, aMax, aLine.dataPtr());
	}

	return DrawChart(matrix, chart_type, aMin, aMax);
}

CImage* DrawChart(const matrixn& matrix, float min, float max)
{
	CImage* pImage=new CImage();
	pImage->Create(matrix.rows(), _private::g_nChartPrecision, 24);

	CImagePixel cip(pImage);
	cip.Clear(RGB(255,255,255));

	COLORREF color[4];
	color[0]=RGB(255,0,0);
	color[1]=RGB(0,128,0);
	color[2]=RGB(0,0,255);
	color[3]=RGB(64,64,0);

	/* acceleration graph*/

	if(min==max)
	{
		min=matrix.minimum();
		max=matrix.maximum();
	}
	
	vectorn aColumn;
	for(int i=0; i<matrix.cols(); i++)
	{
		matrix.getColumn(i, aColumn);
		_private::DrawChart(pImage, matrix.rows(), aColumn.dataPtr(), min, max, color[i], 0, 0, 1, LINE_CHART);
	}
	
	return pImage;
}

CImage* DrawChart(const matrixn& matrix, int chart_type, vectorn& aMin, vectorn& aMax, m_real* aY)
{
	CImage* pImage=new CImage();
	pImage->Create(MAX(100, matrix.cols()), _private::g_nChartPrecision*matrix.rows(), 24);

	CImagePixel cip(pImage);
	cip.Clear(RGB(255,255,255));

	COLORREF color[2];
	color[0]=RGB(0,0,128);
	color[1]=RGB(0,128,0);
	/* acceleration graph*/

	if(aMin.size()==0)
		aMin.aggregate(CAggregate::MINIMUM, matrix);
	
	if(aMax.size()==0)
		aMax.aggregate(CAggregate::MAXIMUM, matrix);

	for(int i=0; i<matrix.rows(); i++)
	{
		_private::DrawChart(pImage, matrix.cols(), matrix[i], aMin[i], aMax[i], color[i%2], 0, _private::g_nChartPrecision*i, 1, chart_type);
		if(aY)
		{
			CImagePixel cip(pImage);
			int iy=ToImageY(aY[i], aMin[i], aMax[i], _private::g_nChartPrecision*i);
            cip.DrawHorizLine(0, iy, matrix.cols(), color[i%2+1]);
		}
	}
	
	return pImage;
}

CImage* DrawChart(const vectorn& vector, int chart_type, float min, float max)
{
	CImage* pImage=new CImage();
	pImage->Create(vector.size(), _private::g_nChartPrecision, 24);

	CImagePixel cip(pImage);
	cip.Clear(RGB(255,255,255));

	if(min==max)
	{
/*		intvectorn sorted;
		sorted.sortedOrder(vector);
		min=vector[sorted[sorted.size()/5]];
		max=vector[sorted[sorted.size()*4/5]];*/

		min=vector.minimum();
		max=vector.maximum();
	}

	interval range(min, max);
	range.expand(0.0001);

	_private::DrawChart(pImage, vector.size(), vector.dataPtr(), range.start(), range.end(), RGB(0,0,0), 0, 0, 1,chart_type);
	
	return pImage;
}

CImage* Plot(const vectorn& x, const vectorn& y)
{
	ASSERT(x.size()==y.size());

	CImage* pImage=new CImage();
#define SIZE 1024
	pImage->Create(SIZE ,SIZE , 24);
	CImagePixel ip(pImage);

	m_real xmin,xwidth;
	m_real ymin,ywidth;
	xmin=x.minimum();
	xwidth=x.maximum()-xmin;
	ymin=y.minimum();
	ywidth=y.maximum()-ymin;
	
#define XX(iii) ((x[iii]-xmin)/xwidth*(m_real)0.8+(m_real)0.1)
#define YY(iii) ((y[iii]-ymin)/(ywidth)*(m_real)0.8+(m_real)0.1)

	m_real prev_x=XX(0);
	m_real prev_y=YY(0);
    for(int i=1; i<x.size(); i++)
	{
		ip.DrawLine(prev_x*SIZE, prev_y*SIZE, XX(i)*SIZE, YY(i)*SIZE, RGB(255,255,255));		
		prev_x=XX(i);
		prev_y=YY(i);
	}
	return pImage;
}

CImage* DrawSpectrum(vectorn& signal, int signalWindow, int offset)
{

#define START(isignal)	((isignal)*offset)
#define END(isignal)	((isignal)*offset+signalWindow)

	int numSignal;
	for( numSignal=0; END(numSignal)< signal.size(); numSignal++) ;

	CImage* pOutput=NULL;

	if(numSignal>0)
	{
		intvectorn aIndex;
		matrixn spectrum(numSignal, signalWindow);

		for(int i=0; i<numSignal; i++)
		{
			aIndex.colon(START(i), END(i));
			spectrum.row(i).extract(signal, aIndex);
		}

		CImage *pIm2=DrawChart(spectrum, LINE_CHART);
		matrixn fmag;
		fmag.setSize(spectrum.rows(), spectrum.cols()/2);
		cmplxvectorn *spectrum2;
		spectrum2=new cmplxvectorn[numSignal];
		for(int i=0; i<numSignal; i++)
		{
			spectrum2[i].setSize(signalWindow);
			spectrum2[i].rfft(spectrum.row(i));
			fmag.row(i).lengths(spectrum2[i]);
			fmag.row(i).save("test.txt",false);
		}
		delete[] spectrum2;
/*		matrixn logSpectrum;
		logSpectrum.setSameSize(fmag);
		for(i=0; i<numSignal; i++)	logSpectrum[i].op(OP_LOG,fmag[i]);
		*/
		CImage *pIm1=DrawChart(fmag, BAR_CHART);
		pOutput=StitchHoriz(pIm1, pIm2);

		CImagePixel cip(pOutput);
		int argMax;
		char temp[100];

		aIndex.colon(1,fmag.row(0).size());
		for(int i=0; i<numSignal; i++)
		{
			argMax=fmag.row(0).argMax();
			sprintf(temp,"freq: %d interval: %f", argMax, (float)signalWindow/(float)argMax);
			cip.DrawText(10,i*_private::g_nChartPrecision+10, temp);
		}

		delete pIm1;
		delete pIm2;
	}
	return pOutput;
}

CImage* DrawChart(const bitvectorn& ab, COLORREF color)
{
	CImage* pImage=new CImage();
	pImage->Create(ab.size(), 20, 24);

	CImagePixel cip(pImage);
	for(int i=0; i<ab.size(); i++)
	{
		if(ab[i])
			cip.DrawVertLine(i, 0, 20, color);
	}

	return pImage;
}

CImage* DrawChart(const intvectorn& ab, const char* colormapfile)
{
	CImage* pImage=new CImage();
	pImage->Create(ab.size(), 20, 24);

	CImagePixel cip(pImage);

	CImage icolormap;
	icolormap.Load(colormapfile);
	CImagePixel colormap(&icolormap);

	int min=ab.minimum();
	int max=ab.maximum();

	intvectorn colormapIndex;
	colormapIndex.makeSamplingIndex2(icolormap.GetWidth(), max-min+1);

	for(int i=0; i<ab.size(); i++)
	{
		cip.DrawVertLine(i, 0, 20, colormap.GetPixel(colormapIndex[ab[i]-min],0));
	}

	return pImage;
}

CImage* DrawChartText(const ::intvectorn& ab, TArray<TString>* translationTable)
{
	int upperBound=ab.maximum()+1;
	if(translationTable==NULL)
	{
		translationTable=new TArray<TString>(upperBound);
		for(int i=0; i<upperBound; i++)
		{
			translationTable->data(i).format("%d",i);
		}		
	}
	else
	{
		ASSERT(translationTable->size()>=upperBound);
	}

	int maxLen=0;
	for(int i=0; i<upperBound; i++)
	{
		if(translationTable->data(i).length()>maxLen)
			maxLen=translationTable->data(i).length();		
	}
	
#define FONT_HEIGHT 16
	CImage* pImage=new CImage();
	pImage->Create(ab.size(), FONT_HEIGHT*maxLen, 24);

	CImagePixel cip(pImage);


	::intvectorn encoding;
	encoding.runLengthEncode(ab);

	int numGrp=encoding.size()/2;

	//OutputToFile("encoding.txt", TString("encoding %s\n", encoding.output().ptr()));

	for(int grp=0; grp<numGrp; grp++)
	{
		int start=encoding[grp*2];
		int end=encoding[(grp+1)*2];

		TString& text=translationTable->data(ab[start]);
		for(int i=0; i<text.length(); i++)
            cip.DrawText(start,FONT_HEIGHT*i, text.subString(i,i+1));
	}

	return pImage;
}
};