// TFile.h: interface for the TFile class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
 
#include "TextFile.h"
#include "TypeString.h"
#include "TArray.h"
class vectorn;
class intvectorn;
class matrixn;
class intmatrixn;
class bitvectorn;
class vector3N;
class quaterN;
class matrix4;
class vector3;
class quater;
class BitArray;
class hypermatrixn;

/// Type을 저장한다.
class BinaryFile
{
public:
	BinaryFile();
	BinaryFile(bool bWrite, const char* filename);
	virtual ~BinaryFile();
	bool openWrite(const char *fileName);
	bool openRead(const char *fileName);
	void close();

	void packInt(int num);
	void packFloat(double num);
	void packArray(void *buffer, int count, size_t size);
	void pack(const char *str);
	void pack(const vectorn& vec);
	void pack(const vector3& vec);
	void pack(const quater& vec);
	void pack(const intvectorn& vec);
	void pack(const matrixn& mat);
	void pack(const intmatrixn& mat);
	void pack(const vector3N& mat);
	void pack(const quaterN& mat);
	void pack(const TArray<TString>& aSz);
	void pack(const TStrings& aSz);
	void pack(const bitvectorn& vec);
	void pack(const matrix4& mat);
	void pack(const BitArray& bits);
	void pack(const hypermatrixn& mat3d);


	void unpack(BitArray& bits);
	void unpackInt(int& num);
	void unpackFloat(double& num);
	int	unpackInt()					{ int num; unpackInt(num); return num; }
	double unpackFloat()				{ double num; unpackFloat(num); return num; }
	void unpackStr(char *str);	//!< 할당 되어있음을 가정한다.
	TString unpackStr();
	void unpackArray(void *buffer, int count, size_t size);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void unpackArrayMalloc(void **pbuffer, int count, size_t size);
	void unpack(TString& str);	
	void unpack(vectorn& vec);
	void unpack(vector3& vec);
	void unpack(quater& vec);
	void unpack(intvectorn& vec);
	void unpack(matrixn& mat);
	void unpack(intmatrixn& mat);
	void unpack(TArray<TString>& aSz);
	void unpack(TStrings& aSz);
	void unpack(bitvectorn& vec);
	void unpack(quaterN& mat);
	void unpack(vector3N& mat);
	void unpack(matrix4& mat);
	void unpack(hypermatrixn& mat3d);

	
private:
	// without type checking
	void _packInt(int num);
	void _unpackInt(int& num);
	void _packFloat(double num);
	void _unpackFloat(double& num);
	int _unpackInt()	{ int num; _unpackInt(num); return num;}
	double _unpackFloat()	{ double num; _unpackFloat(num); return num;}
	void _packArray(void *buffer, int count, size_t size);
	void _unpackArray(void *buffer, int count, size_t size);

	enum { TYPE_INT, TYPE_FLOAT, TYPE_FLOATN, TYPE_INTN, TYPE_BITN, TYPE_FLOATMN, TYPE_INTMN, TYPE_BITMN, TYPE_STRING, TYPE_STRINGN, TYPE_ARRAY , TYPE_EOF};
	FILE *m_pFile;	
	char *m_pBuffer;
	char* m_pBufferPointer;
};

/// type checking을 하지 않는다. 디버그가 어려워서 더이상 쓰지 않는다.
class TFile  
{
public:
	/// bTextMode인 경우, text file형태로 저장한다.
	TFile(bool bTextMode=false);
	bool OpenWriteFile(const char *fileName);
	bool OpenReadFile(const char *fileName);
	void PackInt(int num);
	void PackFloat(float num);
	void PackStr(const char *str);
	void CloseFile();
	int UnpackInt();
	float UnpackFloat();
	void UnpackStr(char *str);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void UnpackStrMalloc(char **pstr);
	void PackArray(void *buffer, size_t count, size_t size);
	void UnpackArray(void *buffer, size_t count, size_t size);
	//! 이 함수는 malloc을 해서 unpack을 한다. 반드시 나중에 free해주어야 한다.
	void UnpackArrayMalloc(void **pbuffer, size_t count, size_t size);
	
	virtual ~TFile();
private:
	FILE *m_pFile;
	char* buffer;
	bool m_bTextMode;	
};

