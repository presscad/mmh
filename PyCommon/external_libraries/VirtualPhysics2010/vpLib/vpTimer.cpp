/*
VirtualPhysics v0.9

VirtualPhysics was developed as part of the project entitled "Development of 
Real-time Physics Simulation Engine for e-Entertainments" which was financially
supported by the grant from the strategic technology development program
(Project No. 2008-F-033-02) of both the MKE(Ministry of Knowledge Economy) and
MCST(Ministry of Culture, Sports and Tourism) of Korea.

Copyright (c) 2008-2010, Jinwook Kim, Korea Institute of Science and Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.

   3. Only research partners of the project "Development of Real-time Physics 
      Simulation Engine for e-Entertainments" can modify this list of 
	  conditions and the following disclaimer with the consent of the author, 
	  where the research partners refer to all principal investigators 
	  involved in the project. 

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include <VP/vpTimer.h>

#ifdef _WIN32
	double	vpTimer::m_fResolution = 0;
	int		vpTimer::m_nLowshift = 0;

	inline LONGLONG __clock()
	{
		LARGE_INTEGER Count;
		QueryPerformanceCounter(&Count);
		return Count.QuadPart;
	}
#else
	inline clock_t __clock() { return clock(); }
#endif


vpTimer::vpTimer()
{
#ifdef _WIN32
	if ( m_fResolution == 0 || m_nLowshift == 0 )
	{
		LARGE_INTEGER m_Frequency;
		QueryPerformanceFrequency(&m_Frequency);
		LONGLONG nShift = m_Frequency.QuadPart;
		m_nLowshift = 0;
		while ( nShift > 1000000 )
		{
			m_nLowshift++;
			nShift >>= 1;
		}
		m_fResolution = 1.0 / (double)nShift;
	}
#endif

	m_sEllapsedTime = 0;
	m_bHalt = true;
}

void vpTimer::Halt(void)
{
	if ( !m_bHalt )
	{
		m_sEllapsedTime += (__clock() - m_sResumeTime);
		m_bHalt = true;
	}
}

void vpTimer::Resume(void)
{
	if ( m_bHalt ) 
	{
		m_sResumeTime = __clock();		
		m_bHalt = false;
	}
}

void vpTimer::Reset(void)
{
	m_sEllapsedTime = 0;
}

double vpTimer::Read(void)
{
	if ( !m_bHalt ) Halt();
	//return (double)(m_sEllapsedTime >> m_nLowshift) * m_fResolution;
	return (double)m_sEllapsedTime; 
}
