/////////////////////////////////////////////////////////////////////////////
//	PixelPointer.cpp

#include "stdafx.h"
#include "stdimage.h"
#include "ImagePixelPtr.h"

/////////////////////////////////////////////////////////////////
// CColorPixel
CColorPixel::CColorPixel()
{

}

CColorPixel::~CColorPixel()
{

}


/////////////////////////////////////////////////////////////////
// CColorPixelPtr
CColorPixelPtr::CColorPixelPtr(CImage &Im)
{
	m_nHeight = Im.GetHeight();		// 이미지의 가로 길이
	int nWidth = Im.GetRealWidth();	// 이미지의 세로 길이

	// 이미지의 세로 길이 만큼 포인터 배열 할당
	m_pPtr = new LPCOLOR [m_nHeight];

	// 메모리 블록을 고정
	m_hHandle = Im.GetHandle();
	LPSTR lpDIBHdr  = (LPSTR)::GlobalLock((HGLOBAL) m_hHandle);

	// 헤더 다음에 나오는 첫번째 픽셀의 포인터를 얻음
	LPSTR lpDIBBits = (LPSTR)::FindDIBBits(lpDIBHdr);

	// 가로 길이 만큼씩 진행하며 m_pPtr에 이미지의 
	// 각 줄의 첫번째 픽셀의 주소를 뒤집어서 저장
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = (LPCOLOR)lpDIBBits;
		lpDIBBits += nWidth;
	}
}

CColorPixelPtr::CColorPixelPtr(HDIB hHandle)
{
	// 메모리 블록을 고정
	LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) hHandle);
	m_hHandle = hHandle;

	m_nHeight = ::DIBHeight(lpDIBHdr);	// 이미지의 세로 길이
	int nWidth = WIDTHBYTES(::DIBWidth(lpDIBHdr)*24);	// 이미지의 가로 길이

	// 이미지의 세로 길이 만큼 포인터 배열 할당
	m_pPtr = new LPCOLOR [m_nHeight];

	// 헤더 다음에 나오는 첫번째 픽셀의 포인터를 얻음
	LPSTR lpDIBBits = (LPSTR)::FindDIBBits(lpDIBHdr);

	// 가로 길이 만큼씩 진행하며 m_pPtr에 이미지의 
	// 각 줄의 첫번째 픽셀의 주소를 뒤집어서 저장
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = (LPCOLOR)lpDIBBits;
		lpDIBBits += nWidth;
	}
} 


CColorPixelPtr::CColorPixelPtr(LPSTR lpDIBHdr)
{
	m_hHandle = NULL;

	m_nHeight = ::DIBHeight(lpDIBHdr);	// 이미지의 세로 길이
	m_nWidth = WIDTHBYTES(::DIBWidth(lpDIBHdr)*24);	// 이미지의 가로 길이

	// 이미지의 세로 길이 만큼 포인터 배열 할당
	m_pPtr = new LPCOLOR [m_nHeight];

	// 헤더 다음에 나오는 첫번째 픽셀의 포인터를 얻음
	LPSTR lpDIBBits = (LPSTR)::FindDIBBits(lpDIBHdr);

	// 가로 길이 만큼씩 진행하며 m_pPtr에 이미지의 
	// 각 줄의 첫번째 픽셀의 주소를 뒤집어서 저장
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = (LPCOLOR)lpDIBBits;
		lpDIBBits += m_nWidth;
	}
} 

CColorPixelPtr::~CColorPixelPtr()
{ 
	// 메모리 블록을 풀어줌
	if(m_hHandle )
		// 할당 받았던 포인터 배열을 해제
		GlobalUnlock((HGLOBAL) m_hHandle);
	
	delete [] m_pPtr;	
}

/////////////////////////////////////////////////////////////////
// CPixel
CPixel::CPixel()
{

}

CPixel::~CPixel()
{

}

CPixel::CPixel(int i)
{
	I = i;
}

/////////////////////////////////////////////////////////////////
// CColorPixelPtr

CPixelPtr::CPixelPtr(CImage &Im)
{
	m_nHeight = Im.GetHeight();		// 이미지의 가로 길이
	int nWidth = Im.GetRealWidth();	// 이미지의 세로 길이

	// 이미지의 세로 길이 만큼 포인터 배열 할당
	m_pPtr = new BYTE *[m_nHeight]; 
	
	// 메모리 블록을 고정
	m_hHandle = Im.GetHandle();
	LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) m_hHandle);

	// 헤더 다음에 나오는 첫번째 픽셀의 포인터를 얻음
	BYTE *lpDIBBits = (BYTE *) ::FindDIBBits(lpDIBHdr);

	// 가로 길이 만큼씩 진행하며 m_pPtr에 이미지의 
	// 각 줄의 첫번째 픽셀의 주소를 뒤집어서 저장
	for(int i=m_nHeight -1 ; i>=0 ; i--)
	{
		m_pPtr[i] = lpDIBBits;
		lpDIBBits += nWidth;
	}
}

 
CPixelPtr::CPixelPtr(HDIB hHandle)
{
	// 메모리 블록을 고정
	LPSTR lpDIBHdr  = (LPSTR) ::GlobalLock((HGLOBAL) hHandle);
	m_hHandle = hHandle;

	m_nHeight = ::DIBHeight(lpDIBHdr);	// 이미지의 세로 길이
	int nWidth = WIDTHBYTES(::DIBWidth(lpDIBHdr)*8);	// 이미지의 가로 길이

	// 이미지의 세로 길이 만큼 포인터 배열 할당
	m_pPtr = new BYTE *[m_nHeight];

	// 헤더 다음에 나오는 첫번째 픽셀의 포인터를 얻음
	BYTE *lpDIBBits = (BYTE *)::FindDIBBits(lpDIBHdr);

	// 가로 길이 만큼씩 진행하며 m_pPtr에 이미지의 
	// 각 줄의 첫번째 픽셀의 주소를 뒤집어서 저장
	for(int i=m_nHeight-1 ; i>=0 ; i--)
	{
		m_pPtr[i] = lpDIBBits;
		lpDIBBits += nWidth;
	}
}

CPixelPtr::~CPixelPtr()
{
	// 메모리 블록을 풀어줌
	GlobalUnlock((HGLOBAL) m_hHandle);
	// 할당 받았던 포인터 배열을 해제
	delete [] m_pPtr;
}

