#pragma once

#include "image.h"
#include "imagepixelptr.h"

inline void SetPixel(CPixelPtr& inputptr, int x, int y, int value)				{	inputptr[y][x]=value;}
inline void SetPixel(CColorPixelPtr& inputptr, int x, int y, COLORREF color)	{	inputptr[y][x].R=GetRValue(color);		inputptr[y][x].G=GetGValue(color);		inputptr[y][x].B=GetBValue(color);}

class CImagePixel
{
public:
	CImagePixel();
	CImagePixel(CImage* pInput);
	~CImagePixel();
	void Init(CImage* pInput);

	/// image좌표계에서 정의된 점을 그린다. -> 느리다는 점에 주의. 좀더 빠르게 점을 찍고 싶은 경우 CColorPixelPtr클래스의 사용. 사용법은 ImagePixel.cpp의 SetPixel함수 참고
	inline void SetPixel(int x, int y, COLORREF color)
	{
#ifndef COLOR_IMAGE_ONLY
		if(m_pPP)
			::SetPixel(*m_pPP, x, y, GetRValue(color));
		else
#endif
			::SetPixel(*m_pCPP, x, y, color);
	}

	inline COLORREF GetPixel(int x, int y) const
	{
#ifndef COLOR_IMAGE_ONLY
		if(m_pPP)
			return RGB((*m_pPP)[y][x],(*m_pPP)[y][x],(*m_pPP)[y][x]);
		else 
#endif
			return RGB((*m_pCPP)[y][x].R,(*m_pCPP)[y][x].G,(*m_pCPP)[y][x].B);
	}

	inline tagCOLOR& Pixel(int x, int y) const	{ASSERT(m_pCPP);return (*m_pCPP)[y][x];};
	static inline void ToTagColor(COLORREF color, tagCOLOR& tagCOLOR)	{ tagCOLOR.R=GetRValue(color); tagCOLOR.G=GetGValue(color); tagCOLOR.B=GetBValue(color);}
	
	void SetPixel(float x, float y, COLORREF color); ///< (0,1)좌표계에서 정의된 점을 그린다. -> 느리다는 점에 주의. 좀더 빠르게 점을 찍고 싶은 경우 CColorPixelPtr클래스 사용. 사용법은 ImagePixel.cpp의 SetPixel함수 참고
	
	
	COLORREF GetPixel(float x, float y,int& count);	///< bilinear filtering getpixel
	
	void DrawHorizLine(int x, int y, int width, COLORREF color);
	void DrawVertLine(int x, int y, int height, COLORREF color,bool bDotted=false);
	void DrawLine(int x1, int y1, int x2, int y2, COLORREF color);
	void DrawBox(const TRect& rect, COLORREF color);
	void DrawPattern(int x, int y, const CImagePixel& patternPixel, bool bUseColorKey=false, COLORREF colorkey=0, bool bOverideColor=false, COLORREF overrideColor=0);
	void DrawPattern(int x, int y, CImage* pPattern, bool bUseColorKey=false, COLORREF colorkey=0, bool bOverideColor=false, COLORREF overrideColor=0);
	void DrawSubPattern(int x, int y, const CImagePixel& patternPixel, const TRect& patternRect, bool bUseColorKey=false, COLORREF colorkey=0);

	void Clear(COLORREF color);
	void DrawText(int x, int y, const char* str, bool bUserColorKey=false, COLORREF colorkey=0);

	inline int Width() const	{ return m_pInput->GetWidth();}
	inline int Height() const	{ return m_pInput->GetHeight();}
private:
	CImage* m_pInput;
#ifndef COLOR_IMAGE_ONLY
	CPixelPtr* m_pPP;
#endif
	CColorPixelPtr* m_pCPP;
};


/// SetPixel(i,j, Color(c1)+Color(c2));...
class Color
{
public:
	Color()										{}
	Color(COLORREF color)						{ m_color.m_COLORREF=color;}
	
	Color operator+( const Color & c)			{ Color oc; for(int i=0; i<NUM_CHANNEL; i++)	oc[i]=(c[i]+(*this)[i])/2;  return oc;}	
	operator COLORREF()	const					{ return m_color.m_COLORREF;}
	
	inline unsigned char& operator[] (int i) const	{ return m_color.m_c[i];}
	
	enum { ALPHA=0, BLUE, GREEN, RED, NUM_CHANNEL};

private:
	union
	{
		mutable unsigned char m_c[NUM_CHANNEL];
		COLORREF m_COLORREF;
	} m_color;
};
