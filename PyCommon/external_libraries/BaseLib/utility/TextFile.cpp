// TextFile.cpp: implementation of the CTextFile class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "TextFile.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CTextFile::CTextFile()
{
	m_strSeps= " ,\t\n";
	m_bToUpper=false;
	Init();
}

void CTextFile::Init()
{
	m_pFile=NULL;

	for(int i=0; i<NUM_UNDO; i++)
		m_aStates[i].Init();
	
	m_nCurrState=0;
	m_nEmptyLine=0;
}

CTextFile::~CTextFile()
{
	if(m_pFile) fclose(m_pFile);
}

bool ReadFromFile(FILE*& file, char* buff)
{
	// #으로 시작하는 line은 제거
	bool bEOF;
	if(file==NULL) return true;
	while(1)
	{
		bEOF=(fgets(buff, 4095, file)==NULL);
		if(bEOF)
		{
			fclose(file);
			file=NULL;
			return bEOF;
		}
		if(buff[0] != '#')
			return bEOF;
	}
}

bool CTextFile::OpenReadFile(const char *fileName)
{
	Init();
	ASSERT(m_pFile==NULL);
	m_pFile=fopen(fileName,"rt");
	if(!m_pFile) return false;
	ReadOneLine();
	return true;
}

void CTextFile::CloseFile()
{
	if(m_pFile)
		fclose(m_pFile);
	Init();
}

bool IsOneOf(char c, const char* seps)
{
	for(int i=0; seps[i]; i++)
		if(c==seps[i]) return true;
	return false;
}

char* copy(int left, int right, char* buff, char* output)
{
	int i;
	for(i=left; i<right; i++)
		output[i-left]=buff[i];
	output[i-left]=0;
	return output;
}

char* CTextFile::Strtok()
{
	int& cur_index=m_aStates[m_nCurrState].nIndex;
	int cur_line=m_aStates[m_nCurrState].nLine;
		
	int i;
	for(i=cur_index; m_aszLine[cur_line][i] && IsOneOf(m_aszLine[cur_line][i], m_strSeps); i++);
	int left=i;
	ASSERT(left<4096);
	for(; m_aszLine[cur_line][i] && !IsOneOf(m_aszLine[cur_line][i], m_strSeps); i++);
	int right=i;
	ASSERT(right<4096);
	cur_index=right;
	if(left==right) return NULL;
	return copy(left, right, m_aszLine[cur_line], m_szOutput);
}

bool CTextFile::ReadOneLine()
{
	m_aStates[m_nCurrState].nLine=(m_aStates[m_nCurrState].nLine+1)%NUM_LINES;
	m_aStates[m_nCurrState].nIndex=0;
	int curline=m_aStates[m_nCurrState].nLine;
	
	if(curline==m_nEmptyLine)
	{
		m_abEOF[curline]=ReadFromFile(m_pFile, m_aszLine[curline]);
		m_nEmptyLine=(m_nEmptyLine+1)%NUM_LINES;
	}
	return m_abEOF[curline];
}

void CTextFile::SaveUndoState()
{
	m_aStates[(m_nCurrState+1)%NUM_UNDO].Clone(m_aStates[m_nCurrState]);
	m_nCurrState=(m_nCurrState+1)%NUM_UNDO;	
}

char* CTextFile::GetToken(bool& bLineChange)
{
	bLineChange=false;
	SaveUndoState();

	if(m_abEOF[m_aStates[m_nCurrState].nLine]) return NULL;

	char* token;
	token=Strtok();
	
	while(!token)
		if(!ReadOneLine())
		{
			bLineChange=true;
			token = Strtok();
		}
		else 
			return NULL;

	if(m_bToUpper)
		for(int i = 0; i < strlen(token); i++)
			token[i] = toupper(token[i]);

	return token;
}

char* CTextFile::GetLine()
{
	SaveUndoState();
	int curline=m_aStates[m_nCurrState].nLine;
	if(m_abEOF[curline]) return NULL;
	char* token;	
	int len=strlen(m_aszLine[curline]);
	token=copy(m_aStates[m_nCurrState].nIndex, len, m_aszLine[curline], m_szOutput);
	ReadOneLine();
	return token;
}

void CTextFile::Undo()
{
	m_nCurrState=(m_nCurrState+NUM_UNDO-1)%NUM_UNDO;
}