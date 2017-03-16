// ColorConversion.cpp: implementation of the CColorConversion class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdimage.h"
#include "../math/mathclass.h"
#include "ColorConversion.h"
#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define UNDEFINED -1 

#define UNDEFINED 0
// Theoretically, hue 0 (pure red) is identical to hue 6 in these transforms. Pure 

// red always maps to 6 in this implementation. Therefore UNDEFINED can be 

// defined as 0 in situations where only unsigned numbers are desired. 

#define RETURN_RGB(r, g, b) { R = r*255.f; G = g*255.f; B = b*255.f; return *this;} 
#ifdef USE_MATHCLASS
RGBType& RGBType::fromHSVType(const HSVType &HSV)
{
	float H,S,V;
	H=HSV.H;
	S=HSV.S;
	V=HSV.V;
	if (S==0.0 )//then   -- color is on black-and-white center line 
	{
		//  -- achromatic: shades of gray 
		//    -- supposedly invalid for h=0 but who cares 
		RETURN_RGB(V,V,V);
	}
	else// -- chromatic color 
	{
		float hTemp,i, p,q,t,f;
		if( H==360.0 )//then  -- 360 degrees same as 0 degrees 
			hTemp=0.0;
		else 
			hTemp=H;

		hTemp=hTemp/60.0;  // -- h is now in [0,6) 
		i=floor(hTemp);  //-- largest integer <= htemp
		f=hTemp-i;          //-- fractional part of htemp

		p=V*(1.0-S) ;
		q=V*(1.0-(S*f)) ;
		t=V*(1.0-(S*(1.0-f))) ;

		switch((int)i)
		{
			case 0: 
				RETURN_RGB(V,t,p)
			case 1:
				RETURN_RGB(q,V,p)
			case 2:
				RETURN_RGB(p,V,t)
			case 3: 
				RETURN_RGB(p,q,V)
			case 4:
				RETURN_RGB(t,p,V)
			case 5:
				RETURN_RGB(V,p,q)
		}
	}
	return *this;
}

 
HSVType& HSVType::fromRGBType(const RGBType& RGB)
{
	float minVal,Delta;
	float R=RGB.R;
	float G=RGB.G;
	float B=RGB.B;
	minVal=MIN3(R, G, B); 
	V=MAX3(R, G, B) ;

	Delta=V-minVal ;

	//-- Calculate saturation: saturation is 0 if r, g and b are all 0 
	if( V==0.0)	S=0.0;
	else S=Delta / V ;
	
	if( S==0.0)
		H=0.0;  //  -- Achromatic: When s = 0, h is undefined but who cares 
	else
	{			//  -- Chromatic 
		if( R==V)// then -- between yellow and magenta [degrees] 
		  H=60.0*(G-B)/Delta ;
		else 
		{
		  if(G==V)// then -- between cyan and yellow 
			H=120.0+60.0*(B-R)/Delta ;
		  else //-- between magenta and cyan 
			H=240.0+60.0*(R-G)/Delta ;
		}
	}
	if (H<0.0) H=H+360.0 ;
	//-- return a list of values as an rgb object would not be sensible 
	V/=255.0;
	return *this;
}

#endif