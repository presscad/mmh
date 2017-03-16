// TTable.h: interface for the CTTable class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_TTABLE_H__187A1BA8_0894_4B7F_8FB6_F4B1ED49C6C6__INCLUDED_)
#define AFX_TTABLE_H__187A1BA8_0894_4B7F_8FB6_F4B1ED49C6C6__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TArray.h"
#include "NameTable.h"
#include <vector>
#include <map>
#include <list>

class CTableElt
{
public:
	enum { TE_INTEGER, TE_FLOAT, TE_STRING, TE_NO_DATA };	

	CTableElt(char* szData)								{m_eType=TE_STRING; m_szData=new char[strlen(szData)+1]; strcpy(m_szData,szData);};
	CTableElt(int nData)								{m_eType=TE_INTEGER; m_nData=nData;};
	CTableElt(float fData)								{m_eType=TE_FLOAT; m_fData=fData;};
	CTableElt()											{ m_eType=TE_NO_DATA;};
	~CTableElt()										{ Clear(); };

	void Clear()										{ if(m_eType==TE_STRING) delete[] m_szData; m_eType=TE_NO_DATA;};
	
	// data query
	int Type() const									{ return m_eType;};
	void GetData(char*& sz)	const						{ ASSERT(m_eType==TE_STRING); sz=m_szData;};									
	void GetData(int& n) const							{ ASSERT(m_eType==TE_INTEGER); n=m_nData;};
	void GetData(float& f) const						{ ASSERT(m_eType==TE_FLOAT); f=m_fData;};
	char* GetString() const								{ ASSERT(m_eType==TE_STRING); return m_szData;};
	int GetInt() const									{ ASSERT(m_eType==TE_INTEGER); return m_nData;};
	float GetFloat() const								{ ASSERT(m_eType==TE_FLOAT); return m_fData;};
	
	// data setup
	void SetData(char *szData)							{ Clear(); m_eType=TE_STRING; m_szData=new char[strlen(szData)+1]; strcpy(m_szData,szData);};
	void SetData(int nData)								{ Clear(); m_eType=TE_INTEGER; m_nData=nData;};
	void SetData(float fData)							{ Clear(); m_eType=TE_FLOAT; m_fData=fData;};
	bool operator==(const CTableElt& other) const;

private:
	int m_eType;
	union
	{
		char * m_szData;
		int m_nData;
		float m_fData;
	};

};

typedef CTArray<CTableElt> columnVector;

// 추가기능을 구현하고 싶으면 왠만하면 상속해서 하시오.
class CTTable  
{
public:
	CTTable();
	virtual ~CTTable();

	bool InitTable(const char* columfilename);
	bool InitTable(char* columfilename, int numRow);
	bool ReadFromFile(const char* columnfilename, const char* datafilename, const char* seps=NULL);
	bool SaveIntoFile(char* datafilename);

	int ColumnIndex(char* str)							{ return m_aColumns[str];};
	
	int NumConstraints(int nColumn)						{ return m_aaValues[nColumn].Size();};
	char* ConstraintName(int nColumn, int nHandle)		{ return m_aaValues[nColumn][nHandle];};	
	int ConstraintHandle(int nColumn, char* str)		{ return m_aaValues[nColumn][str];};	

	int NumColumns()									{ return m_aColumns.Size();};
	int NumRows()										{ return m_aaData.size();};
	char* ColumnName(int nColumn)						{ return m_aColumns[nColumn];};
	const CTableElt& Data(int row, int column)			{ return m_aaData[row]->operator[](column);};
	const char* StringData(int row, int column);
	int IntData(int row, int column)					{ return Data(row, column).GetInt();};
	float FloatData(int row, int column)				{ return Data(row, column).GetFloat();};

	// n번째 column의 값이 data와 같은 row의 index list를 return 
	void FindRow(int nColumn, const CTableElt &data, std::list<int>& listIndex);
	// source index에 해당하는 row중 n번째 column의 값이 data와 같은 row의 index list를 return 
	void FindRow(int nColumn, const CTableElt &data, std::list<int>& sourceIndex, std::list<int>& resultIndex);

protected:
	void InitTable(int nRow, int nColumn);
	void ReleaseTable();
	
	void SetData(int row, int column, char* data);
	void SetData(int row, int column, int data);
	void SetData(int row, int column, float data);


	std::vector<columnVector*> m_aaData;
	CTArray<NameTable> m_aaValues;	//!< column이 string constraint를 갖는 경우 이를 저장한다.
	NameTable m_aColumns;			//!< column name
	int *m_aeColumnType;			//!< column type
};


#endif // !defined(AFX_TTABLE_H__187A1BA8_0894_4B7F_8FB6_F4B1ED49C6C6__INCLUDED_)
