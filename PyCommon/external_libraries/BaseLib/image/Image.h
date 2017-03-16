// Image.h: interface for the CImage class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IMAGE_H__819723AE_3198_48ED_AFA2_7EA5BE04AF17__INCLUDED_)
#define AFX_IMAGE_H__819723AE_3198_48ED_AFA2_7EA5BE04AF17__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define HDIB HGLOBAL

// DIB constants
#define PALVERSION   0x300

// DIB Macros
#define IS_WIN30_DIB(lpbi) ((*(LPDWORD)(lpbi)) == sizeof(BITMAPINFOHEADER))
#define RECTWIDTH(lpRect)  (((lpRect)->right>(lpRect)->left)?((lpRect)->right - (lpRect)->left):((lpRect)->right- (lpRect)->left))
#define RECTHEIGHT(lpRect)  (((lpRect)->bottom>(lpRect)->top)?((lpRect)->bottom- (lpRect)->top):((lpRect)->top- (lpRect)->bottom))
//#define RECTHEIGHT(lpRect) ((lpRect)->bottom - (lpRect)->top)
#define WIDTHBYTES(bits)   (((bits) + 31) / 32 * 4)

/// 화면에서 봤을때 왼쪽 위가 0,0인 이미지
/**
 * \ingroup group_image
 *
 *
 * \par requirements
 * 이 클래스 사용하려면 link옵션에 아래 내용 추가할 것.
 * /nodefaultlib:"libcmt" /nodefaultlib:"LIBC" /nodefaultlib:"libci" 
 * 
 * 
 * \todo 
 *
 * \bug 
 *
 */
struct TSize
{
	TSize(){}
	TSize(long sx, long sy):cx(sx), cy(sy){}
	long cx;
	long cy;
};

struct TRect : RECT
{
 //   long left, top, right, bottom;

    TRect()
    {
    }
    TRect( long l, long t, long r, long b )
    {
        left = l;
        top = t;   
        right = r;
        bottom = b;                
    }
    TRect& operator = ( const TRect& other )
    {
        left = other.left;
        top = other.top;
        right = other.right;
        bottom = other.bottom;       

        return *this;
    }

	int Width() const				{ return right-left;}
	int Height() const				{ return bottom-top;}
	bool PtInRect(POINT pt)	const	{ if(pt.x>=left && pt.y>=top && pt.x<right && pt.y<bottom) return true; return false;}

	operator RECT*() const	{ return (RECT*)this;}
};

class CImage  
{
private:
	HDIB m_hImage;
	TSize m_Size;
	//CPalette* m_pPal;

	// utility funcitions
	bool CreateDIBPalette();
	bool InitDIB(bool bCreatePalette = true);
	HDIB GetHandle()			{return m_hImage;}
	int GetRealWidth()			{return WIDTHBYTES((GetWidth()*GetBitCount()));}	
	//CPalette *GetPalette()		{return m_pPal;}

	// BMP 파일 읽어오기
	BOOL LoadBMP(LPCTSTR lpszFileName);
	
	// BMP 파일 저장하기
	BOOL SaveBMP(LPCTSTR lpszFileName);
public:
	CImage();
	virtual ~CImage();
	int GetBitCount();
	int GetWidth();
	int GetHeight();
	bool Create(int width, int height, int bitcount);
	
	bool Load(const char* filename);
	bool Save(const char* filename);
	
	bool IsDataNull();
	void Free();
	int Draw(HDC hDC, const TRect& sourceRect, TRect& destRect);
/*	int Draw(CDC* pDC, const CRect& sourceRect, CRect& destRect);	
	int Draw(CDC* pDC, CRect& destRect)	{ return Draw(pDC, CRect(0,0, GetWidth(), GetHeight()),destRect);};
	int Draw(CDC* pDC, int x, int y, float scale=1.f)	{ return Draw(pDC, CRect(0,0, GetWidth(), GetHeight()),CRect(x,y,x+(int)(GetWidth()*scale), y+int(GetHeight()*scale)));};*/

	friend class CPixelPtr;
	friend class CColorPixelPtr;
};

/******************************************************
						전역 함수
******************************************************/
LPSTR WINAPI FindDIBBits(LPSTR lpbi);
DWORD WINAPI DIBWidth(LPSTR lpDIB);
DWORD WINAPI DIBHeight(LPSTR lpDIB);
WORD WINAPI PaletteSize(LPSTR lpbi);
WORD WINAPI DIBNumColors(LPSTR lpbi);
HGLOBAL WINAPI CopyHandle (HGLOBAL h);

#endif // !defined(AFX_IMAGE_H__819723AE_3198_48ED_AFA2_7EA5BE04AF17__INCLUDED_)

