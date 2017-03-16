// ImagePixel.cpp: implementation of the CImagePixel class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdimage.h"
#include "ImagePixel.h"
#include "ImagePixelPtr.h"
#include "Image.h"

CImagePixel::CImagePixel()
{
	m_pInput=NULL;
#ifndef COLOR_IMAGE_ONLY
	m_pPP=NULL;
#endif
	m_pCPP=NULL;
}

CImagePixel::CImagePixel(CImage* pInput)
{
	m_pInput=pInput;
#ifndef COLOR_IMAGE_ONLY
	m_pPP=NULL;
#endif
	m_pCPP=NULL;
	Init(pInput);
}

CImagePixel::~CImagePixel()
{
#ifndef COLOR_IMAGE_ONLY
	if(m_pPP) delete m_pPP;
#endif
	if(m_pCPP) delete m_pCPP;
}

void CImagePixel::Init(CImage* pInput)
{
	m_pInput=pInput;
#ifndef COLOR_IMAGE_ONLY
	if(m_pPP) {delete m_pPP; m_pPP=NULL;}
#endif
	if(m_pCPP) { delete m_pCPP; m_pCPP=NULL;}
	
	if(pInput->GetBitCount()==8)
	{
#ifndef COLOR_IMAGE_ONLY
		m_pPP=new CPixelPtr(*pInput);
#endif
	}
	else
	{
		m_pCPP=new CColorPixelPtr(*pInput);
	}
}

COLORREF CImagePixel::GetPixel(float x, float y, int& count)
{
	///< bilinear filtering getpixel
	COLORREF color;
	int x1,y1,x2,y2;
	x1=(int)x;
	x2=x1+1;
	y1=(int)y;
	y2=y1+1;

	count=1;
	// 4개 픽셀중 하나라도 밖에 있는 경우 
	if(x1<0 || x2>=m_pInput->GetWidth() || y1<0 || y2>=m_pInput->GetHeight())
	{
		count=0;
		color=RGB(0,0,0);
	}
	// 모두 안에 있는경우 
	else
	{
		COLORREF c1,c2,c3,c4;

		float errx=x-x1;
		float erry=y-y1;
		float ex1=(1.f-errx)*(1.f-errx);
		float ex2=errx*errx;
		float ey1=(1.f-erry)*(1.f-erry);
		float ey2=erry*erry;
		
		// p1: x1,y1
		// p2: x1,y2
		// p3: x2,y1
		// p4: y2,y2
		float w1,w2,w3,w4;
		w1=ex1+ey1;
		w2=ex1+ey2;
		w3=ex2+ey1;
		w4=ex2+ey2;
		float sum=w1+w2+w3+w4;
		w1/=sum;
		w2/=sum;
		w3/=sum;
		w4/=sum;

		count=4;
		c1=GetPixel(x1,y1);
		c2=GetPixel(x1,y2);
		c3=GetPixel(x2,y1);
		c4=GetPixel(x2,y2);
		color=RGB(int(GetRValue(c1)*w1+GetRValue(c2)*w2+GetRValue(c3)*w3+GetRValue(c4)*w4),
				  int(GetGValue(c1)*w1+GetGValue(c2)*w2+GetGValue(c3)*w3+GetGValue(c4)*w4),
				  int(GetBValue(c1)*w1+GetBValue(c2)*w2+GetBValue(c3)*w3+GetBValue(c4)*w4));
	}
	return color;
}
void CImagePixel::SetPixel(float fx, float fy, COLORREF color)
{
	int width=m_pInput->GetWidth();
	int height=m_pInput->GetHeight();
	
	int x,y;
	x=int(fx*width);
	y=int(fy*height);

	if(x<0) x=0;
	if(y<0) y=0;
	if(x>=width) x=width-1;
	if(y>=height) y=height-1;

	SetPixel(x,y,color);
}

void CImagePixel::DrawHorizLine(int x, int y, int width, COLORREF color)
{
#ifndef COLOR_IMAGE_ONLY
	if(m_pPP)
	{
		CPixelPtr& inputptr=*(m_pPP);
		for(int i=x; i<x+width; i++)
		{
			inputptr[y][i]=GetRValue(color);
		}
	}
	else 
#endif
	{
		CColorPixelPtr& inputptr=*(m_pCPP);
	
		for(int i=x; i<x+width; i++)
		{
			inputptr[y][i].R=GetRValue(color);
			inputptr[y][i].G=GetGValue(color);
			inputptr[y][i].B=GetBValue(color);
		}
	}
}

void CImagePixel::DrawVertLine(int x, int y, int height, COLORREF color,bool bDotted)
{
	int step=1;
	if(bDotted) step=3;
#ifndef COLOR_IMAGE_ONLY
	if(m_pPP)
	{
		CPixelPtr& inputptr=*(m_pPP);
		
		for(int j=y; j<y+height; j+=step)
		{
			inputptr[j][x]=GetRValue(color);
		}
	}
	else 
#endif
	{
		CColorPixelPtr& inputptr=*(m_pCPP);
	
		for(int j=y; j<y+height; j+=step)
		{
			inputptr[j][x].R=GetRValue(color);
			inputptr[j][x].G=GetGValue(color);
			inputptr[j][x].B=GetBValue(color);
		}
	}
}


void CImagePixel::DrawBox(const TRect& rect, COLORREF color)
{
#ifndef COLOR_IMAGE_ONLY
	if(m_pPP)
	{
		CPixelPtr& inputptr=*m_pPP;
		
		for(int i=rect.left; i<rect.right; i++)
		{
			for(int j=rect.top; j<rect.bottom; j++)
			{
				inputptr[j][i]=GetRValue(color);
			}
		}
	}
	else 
#endif
	{
		tagCOLOR sColor;
		ToTagColor(color,sColor);

		CColorPixelPtr& inputptr=*m_pCPP;

		/*
		// easy to read version
		for(int j=rect.top; j<rect.bottom; j++)
		{
			LPCOLOR ptr=inputptr[j];
			for(int i=rect.left; i<rect.right; i++)
			{
				memcpy(&ptr[i],&sColor,sizeof(tagCOLOR));
			}
		}
		*/
		// fast version
		tagCOLOR* aBuffer;
		int width=rect.right-rect.left;
		aBuffer=new tagCOLOR[width];
		for(int i=0; i<width; i++)
			aBuffer[i]=sColor;
		for(int j=rect.top; j<rect.bottom; j++)
		{
			LPCOLOR ptr=inputptr[j];

			memcpy(&ptr[rect.left],aBuffer, sizeof(tagCOLOR)*(width));
		}
		delete[] aBuffer;
	}	
}

void CImagePixel::Clear(COLORREF color)
{
	int width=m_pInput->GetWidth();
	int height=m_pInput->GetHeight();

	DrawBox(TRect(0,0, width, height), color);
}

void CImagePixel::DrawPattern(int x, int y, const CImagePixel& patternPixel, bool bUseColorKey, COLORREF colorkey, bool bOverideColor, COLORREF overrideColor)
{
	int imageWidth=m_pInput->GetWidth();
	int imageHeight=m_pInput->GetHeight();
	int patternWidth=patternPixel.m_pInput->GetWidth();
	int patternHeight=patternPixel.m_pInput->GetHeight();

	int imagex, imagey;
	ASSERT(m_pCPP);

	tagCOLOR sColorkey;
	ToTagColor(colorkey,sColorkey);

	if(bUseColorKey)
	{
		if(bOverideColor)
		{
			float ovR=float(GetRValue(overrideColor))/255.f;
			float ovG=float(GetGValue(overrideColor))/255.f;
			float ovB=float(GetBValue(overrideColor))/255.f;
			for(int j=0; j<patternHeight; j++)
				for(int i=0; i<patternWidth; i++)
				{
					imagex=x+i; imagey=y+j;
					if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
					{
						if(memcmp(&patternPixel.Pixel(i,j),&sColorkey, sizeof(tagCOLOR))!=0)
						{
						//	SetPixel( imagex, imagey, overrideColor);
							tagCOLOR& c=Pixel(imagex, imagey);
							tagCOLOR& cc=patternPixel.Pixel(i,j);
							c.R=cc.R*ovR;
							c.G=cc.G*ovG;
							c.B=cc.B*ovB;							
						}
					}
				}
		}
		else
		{
			for(int j=0; j<patternHeight; j++)
				for(int i=0; i<patternWidth; i++)			
				{
					imagex=x+i; imagey=y+j;
					if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
					{
						if(memcmp(&patternPixel.Pixel(i,j),&sColorkey, sizeof(tagCOLOR))!=0)
							Pixel(imagex,imagey)=patternPixel.Pixel(i,j);
					}
				}
		}
	}
	else
	{
		ASSERT(!bOverideColor);
		for(int j=0; j<patternHeight; j++)
		{
			LPCOLOR target=&Pixel(x,y+j);
			LPCOLOR source=&patternPixel.Pixel(0,j);
			memcpy(target,source, sizeof(tagCOLOR)*patternWidth);
		}
	}
}

void CImagePixel::DrawPattern(int x, int y, CImage* pPattern, bool bUseColorkey, COLORREF colorkey, bool bOverideColor, COLORREF overrideColor)
{
	CImagePixel patternPixel(pPattern);
	DrawPattern(x,y,patternPixel,bUseColorkey,colorkey,bOverideColor,overrideColor);
}

void CImagePixel::DrawLine(int x1, int y1, int x2, int y2, COLORREF color)	//!< inputptr, inputptr2중 하나는 NULL로 줄것.
{
	int dx,dy,x,y,x_end,p,const1,const2,y_end;
	int delta;
	dx=abs(x1-x2);
	dy=abs(y1-y2);
	
	if (((y1-y2)>0 && (x1-x2)>0 ) || (y1-y2)<0 && (x1-x2)<0)
	{
		delta=1;							//기울기 >0
	}
	else
	{
		delta=-1;							//기울기 <0
	}
	
	if(dx>dy)								//기울기 0 < |m| <=1
	{
		p=2*dy-dx;
		const1=2*dy;
		const2=2*(dy-dx);
		if(x1>x2)
		{
			x=x2;y=y2;
			x_end=x1;
		}
		else
		{
			x=x1;y=y1;
			x_end=x2;
		}
	
		SetPixel( x,y, color);
		while(x<x_end)
		{
			x=x+1;
			if(p<0)
			{
				p=p+const1;
			}
			else
			{
				y=y+delta;
				p=p+const2;
			}
			SetPixel( x,y, color);
		}									//기울기 |m| > 1
	}
	else
	{
		p=2*dx-dy;
		const1=2*dx;
		const2=2*(dx-dy);
		if(y1>y2)
		{
			y=y2;x=x2;
			y_end=y1;
		}
		else
		{
			y=y1;x=x1;
			y_end=y2;
		}
		
		SetPixel( x,y, color);
		while(y<y_end)
		{
			y=y+1;
			if(p<0)
			{
				p=p+const1;
			}
			else
			{
				x=x+delta;
				p=p+const2;
			}

			SetPixel( x,y, color);
		}
	}
}

void CImagePixel::DrawSubPattern(int x, int y, const CImagePixel& patternPixel, const TRect& patternRect, bool bUseColorKey, COLORREF colorkey)
{
	int imageWidth=m_pInput->GetWidth();
	int imageHeight=m_pInput->GetHeight();
	int patternWidth=patternPixel.m_pInput->GetWidth();
	int patternHeight=patternPixel.m_pInput->GetHeight();

	ASSERT(patternRect.right<=patternWidth);
	ASSERT(patternRect.top<=patternHeight);

	int imagex, imagey;
	ASSERT(m_pCPP);

	tagCOLOR sColorkey;
	ToTagColor(colorkey,sColorkey);

	if(bUseColorKey)
	{
		for(int j=patternRect.top; j<patternRect.bottom; j++)
			for(int i=patternRect.left; i<patternRect.right; i++)			
			{
				imagex=x+i-patternRect.left; imagey=y+j-patternRect.top;
				if(imagex>=0 && imagex<imageWidth && imagey>=0 && imagey <imageHeight)
				{
					if(memcmp(&patternPixel.Pixel(i,j),&sColorkey, sizeof(tagCOLOR))!=0)
						Pixel(imagex,imagey)=patternPixel.Pixel(i,j);
				}
			}
	}
	else
	{
		TRect rect=patternRect;
		if(x<0)
		{
			rect.left-=x;
			x-=x;
		}
		if(x+rect.Width()>imageWidth)
		{
			int delta=x+rect.Width()-imageWidth;
			rect.right-=delta;			
		}
		if(rect.Width()>0)
		{
			for(int j=rect.top; j<rect.bottom; j++)
			{
				imagey=y+j-rect.top;
				if(imagey>=0 && imagey <imageHeight)
				{
					LPCOLOR target=&Pixel(x,imagey);
					LPCOLOR source=&patternPixel.Pixel(rect.left,j);
					memcpy(target,source, sizeof(tagCOLOR)*rect.Width());
				}
			}
		}
	}
}

void CImagePixel::DrawText(int x, int y, const char* str, bool bUserColorKey, COLORREF colorkey)
{
	static CImage* pText=NULL;
	if(!pText)
	{
		pText=new CImage();
		pText->Load("../resource/default/ascii.bmp");
	}
	CImage& cText=*pText;
    
	CImagePixel patternPixel(&cText);
#define FONT_HEIGHT 16
#define FONT_WIDTH 8
	int len=strlen(str);
	for(int i=0; i<len; i++)
	{
		char c=str[i];
		int code=(c-' ');
		ASSERT(code>=0 && code<32*3);
		int left=code%32*FONT_WIDTH ;
		int top=code/32*FONT_HEIGHT;
		DrawSubPattern(x+i*FONT_WIDTH , y, patternPixel, TRect(left,top,left+FONT_WIDTH , top+FONT_HEIGHT), bUserColorKey, colorkey);
	}
}