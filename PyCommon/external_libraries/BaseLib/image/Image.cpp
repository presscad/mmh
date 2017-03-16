// Image.cpp: implementation of the CImage class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdimage.h"
#include "Image.h"
#include "ImagePixel.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CImage::CImage()
{
	m_Size		 = TSize(1,1);
	m_hImage	 = NULL;
//	m_pPal		 = NULL;

}

CImage::~CImage()
{
	Free();
}

int CImage::GetBitCount()
{
	if (m_hImage == NULL) return -1;
	LPBITMAPINFOHEADER lpbi;
	lpbi = (LPBITMAPINFOHEADER) ::GlobalLock((HGLOBAL) m_hImage );
	return lpbi->biBitCount;
}

int CImage::GetWidth()
{
	return m_Size.cx;
}

int CImage::GetHeight()
{
	return m_Size.cy;	
}

bool CImage::Create(int width, int height, int depth)
{
	LPBITMAPINFOHEADER lpbi ;
	BYTE		*lpPal;
    DWORD       dwSizeImage;
    int         i;

    dwSizeImage = height*(DWORD)((width*depth/8+3)&~3);

	if(depth == 24)
		m_hImage= (HDIB)GlobalAlloc(GHND,sizeof(BITMAPINFOHEADER)+dwSizeImage);
    else
		m_hImage= (HDIB)GlobalAlloc(GHND,sizeof(BITMAPINFOHEADER)+dwSizeImage + 1024);

    if (m_hImage == NULL)
        return false;

	lpbi = (LPBITMAPINFOHEADER)GlobalLock(m_hImage);
	lpbi->biSize            = sizeof(BITMAPINFOHEADER) ;
    lpbi->biWidth           = width;
    lpbi->biHeight          = height;
    lpbi->biPlanes          = 1;
    lpbi->biBitCount        = depth;
    lpbi->biCompression     = BI_RGB ;
    lpbi->biSizeImage       = dwSizeImage;
    lpbi->biXPelsPerMeter   = 0 ;
    lpbi->biYPelsPerMeter   = 0 ;
    lpbi->biClrUsed         = 0 ;
    lpbi->biClrImportant    = 0 ;

	lpPal = (BYTE *) lpbi;
	if (depth == 8)
	{
		lpbi->biClrUsed = 256;

		DWORD offDest = sizeof(BITMAPINFOHEADER);
		for(i = 0; i < 256; i++)
		{
			lpPal[offDest++] = (BYTE)i;
			lpPal[offDest++] = (BYTE)i;
			lpPal[offDest++] = (BYTE)i;
			lpPal[offDest++] = 0x00;
		}                  
	}

	InitDIB(FALSE);
	return true;
}

bool CImage::InitDIB(bool bCreatePalette)
{
	// 이미지의 가로, 세로 크기 설정
	LPSTR pDIB = (LPSTR)::GlobalLock((HGLOBAL) m_hImage);
	m_Size = TSize((int) ::DIBWidth(pDIB), (int) ::DIBHeight(pDIB));
	::GlobalUnlock((HGLOBAL) m_hImage);

	if(bCreatePalette)
	{
		ASSERT(0);
		/*
		if(m_pPal != NULL) delete m_pPal;
		// 팔레트 생성
		m_pPal = new CPalette;
		if (CreateDIBPalette() == NULL)
		{
			// 팔레트를 가지지 않는 경우
			delete m_pPal;
			m_pPal = NULL;
			return false;
		}*/
	}
	return true;
}

bool CImage::CreateDIBPalette()
{
	LPLOGPALETTE lpPal;      // pointer to a logical palette
	HANDLE hLogPal;          // handle to a logical palette
	HPALETTE hPal = NULL;    // handle to a palette
	int i;                   // loop index
	WORD wNumColors;         // number of colors in color table
	LPSTR lpbi;              // pointer to packed-DIB
	LPBITMAPINFO lpbmi;      // pointer to BITMAPINFO structure (Win3.0)
	LPBITMAPCOREINFO lpbmc;  // pointer to BITMAPCOREINFO structure (old)
	BOOL bWinStyleDIB;       // flag which signifies whether this is a Win3.0 DIB
	bool bResult = false;

	/* if handle to DIB is invalid, return FALSE */

	if (m_hImage == NULL)
	  return false;

   lpbi = (LPSTR) ::GlobalLock((HGLOBAL) m_hImage);

   /* get pointer to BITMAPINFO (Win 3.0) */
   lpbmi = (LPBITMAPINFO)lpbi;

   /* get pointer to BITMAPCOREINFO (old 1.x) */
   lpbmc = (LPBITMAPCOREINFO)lpbi;

   /* get the number of colors in the DIB */
   wNumColors = ::DIBNumColors(lpbi);

   if (wNumColors != 0)
   {
	   ASSERT(0);
	   /*
		// allocate memory block for logical palette 
		hLogPal = ::GlobalAlloc(GHND, sizeof(LOGPALETTE)
									+ sizeof(PALETTEENTRY)
									* wNumColors);

		// if not enough memory, clean up and return NULL 
		if (hLogPal == 0)
		{
			::GlobalUnlock((HGLOBAL) m_hImage);
			return false;
		}

		lpPal = (LPLOGPALETTE) ::GlobalLock((HGLOBAL) hLogPal);

		// set version and number of palette entries 
		lpPal->palVersion = PALVERSION;
		lpPal->palNumEntries = (WORD)wNumColors;

		// is this a Win 3.0 DIB? 
		bWinStyleDIB = IS_WIN30_DIB(lpbi);
		for (i = 0; i < (int)wNumColors; i++)
		{
			if (bWinStyleDIB)
			{
				lpPal->palPalEntry[i].peRed = lpbmi->bmiColors[i].rgbRed;
				lpPal->palPalEntry[i].peGreen = lpbmi->bmiColors[i].rgbGreen;
				lpPal->palPalEntry[i].peBlue = lpbmi->bmiColors[i].rgbBlue;
				lpPal->palPalEntry[i].peFlags = 0;
			}
			else
			{
				lpPal->palPalEntry[i].peRed = lpbmc->bmciColors[i].rgbtRed;
				lpPal->palPalEntry[i].peGreen = lpbmc->bmciColors[i].rgbtGreen;
				lpPal->palPalEntry[i].peBlue = lpbmc->bmciColors[i].rgbtBlue;
				lpPal->palPalEntry[i].peFlags = 0;
			}
		}

		// create the palette and get handle to it 
		if(m_pPal->CreatePalette(lpPal))
			bResult=true;
		::GlobalUnlock((HGLOBAL) hLogPal);
		::GlobalFree((HGLOBAL) hLogPal);
		*/
	}

	::GlobalUnlock((HGLOBAL) m_hImage);

	return bResult;
}

//int CImage::LoadJpg(const char* filename) 이함수는 ImageFIleJpg.cpp에 있음.

bool CImage::IsDataNull()
{
	return (m_hImage == NULL);
}

void CImage::Free()
{
	if( m_hImage )
	{
		if( GlobalFree( m_hImage ) != NULL)
		{
			TRACE("Can't free handle in CImage::Free()");
		}
		m_hImage = NULL;
	}
	/*if(m_pPal != NULL)
	{
		delete m_pPal;
		m_pPal = NULL;
	}*/
}

/*
int CImage::Draw(CDC* pDC, const CRect& sourceRect, CRect& destRect)
{
	return Draw(pDC->m_hDC, sourceRect, destRect);
}
*/

int CImage::Draw(HDC hDC, const TRect& sourceRect, TRect& destRect)
{
			/*
			본 draw함수가 못 미덥거나 무슨 내용인지 궁금하면, 아래 내용을 실행해도 같은 내용이 그려진다는 사실을 참고할 것.
			CRect rectCrop=sourceRect;
			static CImage* pCrop=NULL;
			if(pCrop==NULL)
			{
				pCrop=new CImage();
				pCrop->Create(rectCrop.Width(), rectCrop.Height(), 24);
			}
			else if(pCrop->GetHeight()!=rectCrop.Height()+1 || pCrop->GetWidth()!=rectCrop.Width()+1)
			{
				delete pCrop;
				pCrop=new CImage();
				pCrop->Create(rectCrop.Width(), rectCrop.Height(), 24);
			}
			CImageProcessor::Crop(pCrop, m_pSignal, rectCrop);
			return pCrop->Draw(pDC, CRect(0,0, rectCrop.Width(), rectCrop.Height()), destRect);*/


	/* CImage클래스는 왼쪽위가 0,0인 좌표계를 쓰지만 DIB는 내부적으로 반대로 되어 있다. 따라서 sourceRect를 아래위로 뒤집어준다.*/
	TRect sourceRect2=sourceRect;
	sourceRect2.bottom=GetHeight()-sourceRect.top;
	sourceRect2.top=GetHeight()-sourceRect.bottom;


	LPRECT lpDIBRect=(LPRECT)sourceRect2;
	LPRECT lpDCRect=(LPRECT)destRect;
	LPSTR	lpDIBHdr;	// BITMAPINFOHEADER를 가리킬 포인터
	LPSTR	lpDIBBits;	// DIB 비트를 가리킬 포인터
	BOOL		bSuccess=FALSE;	 // Success/fail 플래그
	HPALETTE 	hPal=NULL;		 // DIB 팔레트
	HPALETTE 	hOldPal=NULL;	 // 이전 팔레트

	// 메모리 고정
	lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) m_hImage);
	// DIB 비트가 저장되어 있는 곳의 주소를 얻음
	lpDIBBits = ::FindDIBBits(lpDIBHdr);

	// 팔레트를 얻어 DC에 선택
	/*
	if(m_pPal != NULL)
	{

		hPal = (HPALETTE) m_pPal->m_hObject;
		hOldPal = ::SelectPalette(hDC, hPal, TRUE);
	}
	*/

	::SetStretchBltMode(hDC, COLORONCOLOR);

	if ((RECTWIDTH(lpDCRect)  == RECTWIDTH(lpDIBRect)) &&
	   (RECTHEIGHT(lpDCRect) == RECTHEIGHT(lpDIBRect)))
		// 원래 크기로 그대로 출력하는 경우
		bSuccess = ::SetDIBitsToDevice(hDC, // hDC
			lpDCRect->left,		 			// DestX
			lpDCRect->top,		 			// DestY
			RECTWIDTH(lpDCRect),	 		// nDestWidth
			RECTHEIGHT(lpDCRect),			// nDestHeight
			lpDIBRect->left,		 		// SrcX
			lpDIBRect->top,					// SrcY
			0,                          	// nStartScan
			(WORD)DIBHeight(lpDIBHdr),  	// nNumScans
			lpDIBBits,                  	// lpBits
			(LPBITMAPINFO)lpDIBHdr,			// lpBitsInfo
			DIB_RGB_COLORS);				// wUsage
	else	// 확대 또는 축소하여 출력하는 경우
		bSuccess = ::StretchDIBits(hDC, 	// hDC
			lpDCRect->left,					// DestX
			lpDCRect->top,					// DestY
			RECTWIDTH(lpDCRect),			// nDestWidth
			RECTHEIGHT(lpDCRect),			// nDestHeight
			lpDIBRect->left,				// SrcX
			lpDIBRect->top,					// SrcY
			RECTWIDTH(lpDIBRect),			// wSrcWidth
			RECTHEIGHT(lpDIBRect),			// wSrcHeight
			lpDIBBits,						// lpBits
			(LPBITMAPINFO)lpDIBHdr,			// lpBitsInfo
			DIB_RGB_COLORS,					// wUsage
			SRCCOPY);						// dwROP

	// 메모리 놓아줌
   ::GlobalUnlock((HGLOBAL) m_hImage);
	// DC 복원
	if (hOldPal != NULL) ::SelectPalette(hDC, hOldPal, TRUE);
	return bSuccess;
}

bool CImage::Save(const char *lpszFileName)
{
	TString filetype;
	filetype = lpszFileName;
	filetype.makeUpper();
	BOOL retval;
	if(filetype.find(".BMP") > -1) retval=SaveBMP(lpszFileName);
#ifdef TIFLIB
	else if(filetype.find(".TIF") > -1) retval=SaveTIF(lpszFileName);
#endif
//	else if(filetype.find(".GIF") > -1) retval=SaveGIF(lpszFileName);
#ifdef JPGLIB
	else if(filetype.find(".JPG") > -1) retval=SaveJPG(lpszFileName);
#endif
	else retval=FALSE;

	if(retval) return true;
	return false;
}

bool CImage::Load(const char* lpszFileName)
{
	TString filetype;
	filetype = lpszFileName;
	filetype.makeUpper();
	BOOL retval;
	if(filetype.find(".BMP") > -1) retval=LoadBMP(lpszFileName);
#ifdef TIFLIB
	else if(filetype.find(".TIF") > -1) retval=LoadTIF(lpszFileName);
#endif
//	else if(filetype.find(".GIF") > -1) retval=LoadGIF(lpszFileName);
#ifdef JPGLIB
	else if(filetype.find(".JPG") > -1) retval=LoadJPG(lpszFileName);
#endif
	else retval=FALSE;

	if(retval) return true;
	return false;
}

/////////////////////////////////////////////////////////////////////////////
/******************************************************
				DIB와 관련된 전역 함수
******************************************************/

LPSTR WINAPI FindDIBBits(LPSTR lpbi)
{
	return (lpbi + *(LPDWORD)lpbi + ::PaletteSize(lpbi));
}


DWORD WINAPI DIBWidth(LPSTR lpDIB)
{
	LPBITMAPINFOHEADER lpbmi;  // pointer to a Win 3.0-style DIB
	LPBITMAPCOREHEADER lpbmc;  // pointer to an other-style DIB

	/* point to the header (whether Win 3.0 and old) */

	lpbmi = (LPBITMAPINFOHEADER)lpDIB;
	lpbmc = (LPBITMAPCOREHEADER)lpDIB;

	/* return the DIB width if it is a Win 3.0 DIB */
	if (IS_WIN30_DIB(lpDIB))
		return lpbmi->biWidth;
	else  /* it is an other-style DIB, so return its width */
		return (DWORD)lpbmc->bcWidth;
}


DWORD WINAPI DIBHeight(LPSTR lpDIB)
{
	LPBITMAPINFOHEADER lpbmi;  // pointer to a Win 3.0-style DIB
	LPBITMAPCOREHEADER lpbmc;  // pointer to an other-style DIB

	/* point to the header (whether old or Win 3.0 */

	lpbmi = (LPBITMAPINFOHEADER)lpDIB;
	lpbmc = (LPBITMAPCOREHEADER)lpDIB;

	/* return the DIB height if it is a Win 3.0 DIB */
	if (IS_WIN30_DIB(lpDIB))
		return lpbmi->biHeight;
	else  /* it is an other-style DIB, so return its height */
		return (DWORD)lpbmc->bcHeight;
}



WORD WINAPI PaletteSize(LPSTR lpbi)
{
   /* calculate the size required by the palette */
   if (IS_WIN30_DIB (lpbi))
	  return (WORD)(::DIBNumColors(lpbi) * sizeof(RGBQUAD));
   else
	  return (WORD)(::DIBNumColors(lpbi) * sizeof(RGBTRIPLE));
}



WORD WINAPI DIBNumColors(LPSTR lpbi)
{
	WORD wBitCount;  // DIB bit count

	/*  If this is a Windows-style DIB, the number of colors in the
	 *  color table can be less than the number of bits per pixel
	 *  allows for (i.e. lpbi->biClrUsed can be set to some value).
	 *  If this is the case, return the appropriate value.
	 */

	if (IS_WIN30_DIB(lpbi))
	{
		DWORD dwClrUsed;

		dwClrUsed = ((LPBITMAPINFOHEADER)lpbi)->biClrUsed;
		if (dwClrUsed != 0)
			return (WORD)dwClrUsed;
	}

	/*  Calculate the number of colors in the color table based on
	 *  the number of bits per pixel for the DIB.
	 */
	if (IS_WIN30_DIB(lpbi))
		wBitCount = ((LPBITMAPINFOHEADER)lpbi)->biBitCount;
	else
		wBitCount = ((LPBITMAPCOREHEADER)lpbi)->bcBitCount;

	/* return number of colors based on bits per pixel */
	switch (wBitCount)
	{
		case 1:
			return 2;

		case 4:
			return 16;

		case 8:
			return 256;

		default:
			return 0;
	}
}


/******************************************************
				클립보드를 위한 전역 함수
******************************************************/

HGLOBAL WINAPI CopyHandle (HGLOBAL h)
{
	if (h == NULL)
		return NULL;

	DWORD dwLen = ::GlobalSize((HGLOBAL) h);
	HGLOBAL hCopy = ::GlobalAlloc(GHND, dwLen);

	if (hCopy != NULL)
	{
		void* lpCopy = ::GlobalLock((HGLOBAL) hCopy);
		void* lp     = ::GlobalLock((HGLOBAL) h);
		memcpy(lpCopy, lp, dwLen);
		::GlobalUnlock(hCopy);
		::GlobalUnlock(h);
	}

	return hCopy;
}
