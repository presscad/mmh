// ColorConversion.h: interface for the CColorConversion class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_COLORCONVERSION_H__94047EBC_E8E5_45AB_9492_92AEC1D1F874__INCLUDED_)
#define AFX_COLORCONVERSION_H__94047EBC_E8E5_45AB_9492_92AEC1D1F874__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#ifdef USE_MATHCLASS
struct HSVType;
//-- RGB, each 0 to 255 
struct RGBType 
{
	RGBType()							{ R=0.f; G=0.f; B=0.f;};
	RGBType(float r, float g, float b)	{R=r; G=g; B=b;};
	RGBType(COLORREF color)				{ R=(float)GetRValue(color); G=(float)GetGValue(color); B=(float)GetBValue(color);}
	COLORREF GetColor()					{ return RGB(R,G,B);};
	RGBType& fromHSVType(const HSVType &hsv);
	float R, G, B;
}; 

//-- H = 0.0 to 360.0 (corresponding to 0..360.0 degrees around hexcone) 
//-- S = 0.0 (shade of gray) to 1.0 (pure color) 
//-- V = 0.0 (black) to 1.0 (white)
struct HSVType
{
	HSVType()							{;};
	HSVType(float h, float s, float v)	{H=h; S=s; V=v;};
	HSVType& fromRGBType(const RGBType& rgb);
	float H, S, V;
}; 

#endif
#endif // !defined(AFX_COLORCONVERSION_H__94047EBC_E8E5_45AB_9492_92AEC1D1F874__INCLUDED_)
