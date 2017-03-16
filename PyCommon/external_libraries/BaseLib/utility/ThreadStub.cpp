// ThreadStub.cpp: implementation of the ThreadStub class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "ThreadStub.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ThreadStub::ThreadStub()
{
	m_bDebug=false;
	m_eThreadState=NO_THREAD;
}

ThreadStub::~ThreadStub()
{
	endRunningThread();
}

UINT ThreadStub::threadControllingFunction( LPVOID pParam )
{
	ThreadStub* pThis=(ThreadStub*)pParam;

	if(pThis->m_eThreadState==THREAD_GO)
		pThis->lock();

	pThis->threadMain();
	ASSERT(pThis->m_eThreadState==THREAD_PAUSE || pThis->m_eThreadState==THREAD_GO);
	pThis->m_eThreadState=NO_THREAD;

	pThis->unlock();
	return 0;// Successfully ended.
}

bool ThreadStub::wait()
{	
	if(m_eThreadState==THREAD_ONE_STEP_START)
	{
		// infinite wait
		while(m_eThreadState==THREAD_ONE_STEP_START)
		{
			printf("sleepping...\n");
			Sleep(10);
		}
	}
	
	if(m_eThreadState==THREAD_ONE_STEP_RUNNING)
	{
		lock();
		// do nothing;
		unlock();
	}

	if(m_eThreadState==THREAD_PAUSE) return 1;
	else return 0;
}

void ThreadStub::runOneStep()
{
	if(m_bDebug)
	{
		threadOneLoop();
		return;
	}

	if(m_eThreadState==NO_THREAD)
	{
		m_eThreadState=THREAD_ONE_STEP_START;		
		AfxBeginThread( threadControllingFunction, this);	// Make worker thread 
	}
	else if(m_eThreadState==THREAD_PAUSE)
	{
		m_eThreadState=THREAD_ONE_STEP_START;		
	}
}

void ThreadStub::pause()
{
	if(m_eThreadState==THREAD_GO)
	{
		m_eThreadState=THREAD_ONE_STEP_RUNNING;
	}
}

void ThreadStub::run()
{
	if(m_eThreadState==NO_THREAD)
	{
		m_eThreadState=THREAD_GO;
		if(m_bDebug)
			threadControllingFunction(this);
		else
			AfxBeginThread( threadControllingFunction, this);	// Make worker thread 		
	}
	else if(m_eThreadState==THREAD_PAUSE)
	{
		m_eThreadState=THREAD_GO;
	}
}

void ThreadStub::threadBarrier()
{
	// I implemented most simple but inefficient method because the performance of ThreadBarrier function is not important at all.
	while(1)
	{
		switch(m_eThreadState)
		{
		case NO_THREAD:
			ASSERT(0);
			break;
		case THREAD_ONE_STEP_START:
			lock();
			m_eThreadState=THREAD_ONE_STEP_RUNNING;
			return;
		
		case THREAD_ONE_STEP_RUNNING:
			m_eThreadState=THREAD_PAUSE;
			unlock();
			break;
		
		case THREAD_PAUSE:	// infinite pause
			Sleep(10);	// sleep 0.01s and continue loop.
			break;

		case THREAD_END:
			m_eThreadState=NO_THREAD;			
			AfxEndThread(1);	// terminate by user interrupt.
			break;
		case THREAD_GO:
			// do nothing
			return;
		}
	}
}

void ThreadStub::threadMain()
{
	// In inherited class, reimplement this function if you want StepSolve functionality.
	while(1)
	{
		threadOneLoop();

		// wait until next runOneStep() is called.
		threadBarrier();
	}
}

void ThreadStub::threadOneLoop()
{
}

void ThreadStub::endRunningThread()
{
	if(m_eThreadState!=NO_THREAD)
	{
		pause();
		wait();
		m_eThreadState=THREAD_END;
		while(m_eThreadState!=NO_THREAD) Sleep(100);		
	}
}