// TTable.cpp: implementation of the CTTable class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "TTable.h"
#include <list>

bool CTableElt::operator==(const CTableElt& other) const
{
	if(m_eType!=other.m_eType) return false;

	switch(m_eType)
	{
	case TE_INTEGER:
		return m_nData==other.m_nData;
	case TE_FLOAT:
		return m_fData==other.m_fData;
	case TE_STRING:
		return strcmp(m_szData, other.m_szData)==0;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CTTable::CTTable()
{
	m_aeColumnType=NULL;
}

CTTable::~CTTable()
{
	ReleaseTable();
}

bool CTTable::InitTable(const char* columnfilename)
{
	ReleaseTable();

	FILE* file;
	char buff[4096];
	char seps[]   = ",\n";

	if(!(file=fopen(columnfilename,"rt"))) return false;
	m_aColumns.Clear();
	char *token;
	
	fgets(buff, 4096, file);
	if(buff[strlen(buff)-1]=='\n')
		buff[strlen(buff)-1]=0;

	int numColumn=atoi(buff);
	m_aaValues.Init(numColumn);
	m_aeColumnType=new int[numColumn];
	int cur_column=0;

	while(fgets(buff, 4096, file))
	{
		if(buff[0]=='#') continue;
		token=strtok(buff, seps);

		// type 읽기 
		if(strcmp(token,"INTEGER")==0)
			m_aeColumnType[cur_column]=CTableElt::TE_INTEGER;
		else if(strcmp(token,"FLOAT")==0)
			m_aeColumnType[cur_column]=CTableElt::TE_FLOAT;
		else if(strcmp(token,"STRING")==0)
			m_aeColumnType[cur_column]=CTableElt::TE_STRING;
		else ASSERT(0);

		token=strtok(NULL, seps);

		// column 이름 읽기 
		m_aColumns.Insert(token);

		// constraint 읽기
		while(token=strtok(NULL,seps))
			m_aaValues[cur_column].Insert(token);
		
		cur_column++;
	}
	
	fclose(file);
	return true;
}

const char* CTTable::StringData(int row, int column)
{
	const CTableElt& data=Data(row, column);
	if(data.Type()==CTableElt::TE_STRING)
		return data.GetString();
	else if(data.Type()==CTableElt::TE_INTEGER && m_aaValues[column].Size()!=0)
		// constraint이 있는경우, 해당 이름으로 바꾼다.
		return m_aaValues[column][data.GetInt()];
	
	ASSERT(0);
	return 0;
}

bool CTTable::InitTable(char* columnfilename, int numRow)
{
	InitTable(columnfilename);

	for(int i=0; i<numRow; i++)
	{
		columnVector* pColumn=new columnVector();
		pColumn->Init(NumColumns());
		m_aaData.push_back(pColumn);
	}
	return true;
}

void CTTable::SetData(int row, int column, char* data)
{
	if(m_aaValues[column].Size()!=0)
	{
		// constraint이 있는경우, 해당 숫자(Handle)로 바꾼다.
		m_aaData[row]->operator[](column).SetData((int)(m_aaValues[column][data]));
	}
	else
		m_aaData[row]->operator[](column).SetData(data);
}

void CTTable::SetData(int row, int column, int data)
{
	m_aaData[row]->operator[](column).SetData(data);
}

void CTTable::SetData(int row, int column, float data)
{
	m_aaData[row]->operator[](column).SetData(data);
}

bool CTTable::ReadFromFile(const char* columnfilename, const char* filename, const char* iseps)
{
	InitTable(columnfilename);
	FILE* file;
	int numColumn=NumColumns();

	if(!(file=fopen(filename,"rt"))) return false;
	
	char buff[4096];
	char seps[10];
	if(iseps==NULL)
		strcpy(seps,",\n");
	else
		strcpy(seps,iseps);
	char* token;
	int cur_column;
	int cur_row=-1;
	while(fgets(buff, 4096, file))
	{
		columnVector* pColumn=new columnVector();
		pColumn->Init(numColumn);
		m_aaData.push_back(pColumn);
		cur_row++;

		token=strtok(buff, seps);
		cur_column=0;
		while(token!=NULL)
		{
			switch(m_aeColumnType[cur_column])
			{
			case CTableElt::TE_INTEGER:
				SetData(cur_row, cur_column, atoi(token));
				break;
			case CTableElt::TE_FLOAT:
				SetData(cur_row, cur_column, (float)atof(token));
				break;
			case CTableElt::TE_STRING:
				SetData(cur_row, cur_column, token);
				break;
			}
			token=strtok(NULL,seps);
			cur_column++;
		}		

		if(cur_column!=numColumn)	
		{
			ReleaseTable();
			return false;
		}
	}
	//TRACE("%d column %d row\n", numColumn, m_aaData.size());
	fclose(file);
	return true;
}	

bool CTTable::SaveIntoFile(char* datafilename)
{
	FILE* file;
	if(!(file=fopen(datafilename,"w"))) return false;

	for(int i=0; i<NumRows(); i++)
	{
		for(int j=0; j<NumColumns(); j++)
		{
			if(j==0)
				fprintf(file,"%s", StringData(i,j));
			else
				fprintf(file,",%s", StringData(i,j));
		}
		fprintf(file,"\n");
	}

	fclose(file);
	return true;
}

void CTTable::ReleaseTable()
{
	m_aColumns.Clear();
	if(m_aeColumnType)
	{	
		delete[] m_aeColumnType;
		m_aeColumnType=NULL;
	}

	std::vector<columnVector*>::iterator i;
	for(i=m_aaData.begin(); i!=m_aaData.end(); i++)
	{
		delete (*i);
	}
	m_aaData.clear();	

	for(int j=0; j<m_aaValues.Size(); j++)
	{
		m_aaValues[j].Clear();
	}
	m_aaValues.Release();
}

void CTTable::FindRow(int nColumn, const CTableElt &data, std::list<int>& listIndex)
{
	int nRow=m_aaData.size();
	listIndex.clear();

	for(int row=0; row<nRow; row++)
	{
		if(m_aaData[row]->operator[](nColumn)==data)
			listIndex.push_back(row);
	}
}

void CTTable::FindRow(int nColumn, const CTableElt &data, std::list<int>& sourceIndex, std::list<int>& listIndex)
{
	int nRow=m_aaData.size();
	listIndex.clear();

	std::list<int>::iterator i;
	for(i=sourceIndex.begin(); i!=sourceIndex.end(); i++)
	{
		ASSERT((*i)<nRow);
		if(m_aaData[*i]->operator[](nColumn)==data)
			listIndex.push_back(*i);
	}
}

