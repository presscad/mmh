// ImageProcessor.h: interface for the CImageProcessor class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IMAGEPROCESSOR_H__44C4DFF4_5928_4C00_938C_11993EA77A65__INCLUDED_)
#define AFX_IMAGEPROCESSOR_H__44C4DFF4_5928_4C00_938C_11993EA77A65__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "image.h"
#include "../math/mathclass.h"

#include <list>

// for backward compatibility
#define CImageProcessor Imp


namespace Imp
{
	CImage* Clone(CImage* pInput);
	void Crop(CImage* pOutput, CImage *pInput, const TRect &rect);	//!< 이미 생성된 output이미지에 크롭할 영역 저장
	CImage* Crop(CImage* pInput, const TRect& rect);	//!< crop된 image생성
	CImage* CropCentered(CImage *pInput, int width, int height);
	CImage* RotateRight(CImage* pInput);
	CImage* RotateLeft(CImage* pInput);
	CImage* RotateHalf(CImage* pInput);
	CImage* Resize(CImage* pInput, int width, int height);
	CImage* StitchHoriz(CImage* pLeft, CImage* pRight);	
	//!< vertex:0 to n-1 , edge : integer 2개, 즉 list의 길이는 2의배수여야 함. motion graph등을 debug할때 사용 가능
	void DrawGraph(CImage* pInput, int numVertex, std::list<int>& listEdge);	
	
	COLORREF GetColor(int i);

	void SafeDelete(CImage* pImage, const char* filename);

	enum {LINE_CHART, BAR_CHART};

	CImage* Plot(const vectorn& x, const vectorn& y);
	CImage* Plot(const matrixn& samples, const vectorn& min, const vectorn& max);
	CImage* DrawChart(const vectorn& vector, int chart_type, float min=0, float max=0); 	//!< 0 frame부터 n-1프레임까지의 float value를 그린다.
	CImage* DrawChart(const matrixn& matrix, int chart_type, vectorn& aMin, vectorn& aMax, m_real* aY=NULL);	//!< 0 frame부터 n-1프레임까지의 여러 signal들을 그린다.
	CImage* DrawChart(const matrixn& matrix, int chart_type, float min=0, float max=0, float horizLine=FLT_MAX);		//!< 0 frame부터 n-1프레임까지의 여러 signal들을 그린다.
	CImage* DrawChart(const matrixn& matrix, float min=0, float max=0);						//!< 0 frame부터 n-1프레임까지의 멀티 dimensional signal을 그린다.
	CImage* DrawChart(const bitvectorn& ab, COLORREF color);
	CImage* DrawChart(const intvectorn& ab, const char* colormapfile="../resource/default/colormap.bmp");
	CImage* DrawChartText(const intvectorn& ab, TArray<TString>* translationTable=NULL);
	CImage* DrawMatrix(matrixn& matrix);
	CImage* DrawMatrix(matrixn& matrix, m_real& min, m_real& max);
	CImage* DrawMatrix2(matrixn& matrix, m_real min=0, m_real max=0);
	CImage* DrawSpectrum(vectorn& signal, int window, int offset);	// draw chart
	
	void LoadMatrix(matrixn& matrix, const char* filename);
	void SaveMatrix(matrixn& matrix, const char* filename);
	// filename을 "filename_prefix_minvalue_maxvalue.bmp"로 한다.
	void SaveMatrixAndInfo(matrixn& matrix, const char* filename_prefix);	
	
	
	/// pInput을 2x2 matrix pTransf를 가지고 transform한 이미지를 return한다.
	/*!
		(source_x,source_y)*matrix=(target_x,target_y)
	*/
	CImage* Transform(matrixn& transf, CImage* pInput);
	CImage* Rotate(float radian, CImage* pInput);	
	
	void ChangeChartPrecision(int precision);
	void DefaultPrecision();

	// private
	namespace _private
	{
		extern int g_nChartPrecision;
		void DrawChart(CImage* pInput, int numFrame, m_real* aValue,float min, float max, COLORREF color, int xoffset, int yoffset, int xdelta,int chart_type);
	}
}


#endif // !defined(AFX_IMAGEPROCESSOR_H__44C4DFF4_5928_4C00_938C_11993EA77A65__INCLUDED_)
