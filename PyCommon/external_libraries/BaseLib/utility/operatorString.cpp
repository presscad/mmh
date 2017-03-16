// TypeString.cpp: implementation of the TypeString class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "stdtemplate.h"
#include "util.h"
#include "TypeString.h"
#include "operatorstring.h"
#include <stdio.h>
#include <string.h>

void sz0::filename ::calc(TString& c) const
{
	int index1, index2;
	index1=c.findCharRight('\\');
	index2=c.findCharRight('/');
	int index=(index1>index2)?index1:index2;
	if(index!=-1)
		c=c.right(-1*index-1);
}

void sz0::Operator::calc(TString& c) const	{ Msg::error("v1::%s::calc(vr) not implemented!!!\n", typeid( *this).name());}
void sz0::zero::calc(TString& c) const
{
	m_op.calc(c);
	c.replace(' ','0');
}

sz0::format::format(const char* pszFormat,...)
{
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
}

sz0::format0::format0(const char* pszFormat,...)
{
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
	m_str.replace(' ','0');
}

void sz0::format::calc(TString& c) const
{
	c=m_str;
}

void sz0::format0::calc(TString& c) const
{
	c=m_str;
}

TString sz1::format(const char* pszFormat, ...) 
{
	TString m_str;
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
	return m_str;
}

TString sz1::format0(const char* pszFormat, ...) 
{
	TString m_str;
	va_list argptr ;
	va_start(argptr, pszFormat);
	m_str._format(pszFormat, argptr);
	m_str.replace(' ','0');
	return m_str;
}

// extract filename: eg) fn="c:\a\b.bmp" -> return "b.bmp", where dir="c:\a"
TString sz1::filename(TString const& fn, TString& dir)
{
	TString lfn;
	// split fn into dir+lfn
	int currDir;
	if((currDir=fn.findCharRight('/'))!=-1)
	{
		lfn=fn.right(-1*currDir-1);
		dir=fn.left(currDir);
	}
	else
	{
		dir.empty();
		lfn=fn;
	}

	return lfn;
}

TString sz1::extension(TString const& fn)
{
	int pos;
	if((pos=fn.findCharRight('.'))!=-1)
	{
		return fn.right(-1*pos-1);
	}
	TString temp;
	temp.empty();
	return temp;
}