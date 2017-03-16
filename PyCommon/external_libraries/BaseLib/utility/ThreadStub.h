// ThreadStub.h: interface for the CThreadStub class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ThreadStub_H__5811B20E_9B3D_48EF_AE21_6EA7BD82463F__INCLUDED_)
#define AFX_ThreadStub_H__5811B20E_9B3D_48EF_AE21_6EA7BD82463F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "afxmt.h"	// critical section


class ThreadStub  
{
	// Inherit this and add your own information.
	// You can use member variable in the Main function.
	// The instance of this class corresponds to one running thread.

public:
	ThreadStub();
	virtual ~ThreadStub();

	// following functions are called outside this thread. 
	// I assumed there exist only one derived class(thread). If not, more synchronization is needed.

	void runOneStep();			//!< stop at each ThreadBarrier call in the Main function. If you want to continue, you must call do one step again.
	void run();					//!< non-stop run.
	void pause();				//!< pause at closest ThreadBarrier call in the Main function.
	bool wait();				//!< wait until this thread actually pauses. This function can only be used after eiter runOneStep(); or pause();. Otherwise it will return error code 0.
	void endRunningThread();	//!< you can kill a thread from outside using this function.
	
protected:
	/** locking. thread에서 돌아가는 코드와 동시에 실행 되면 안되는 코드를 lock(), unlock()으로 감싸준다.
	/*  내부적으로 thread가 pause되어 있는 기간만 제외하고 항상 lock이 되어 있다. 
	 */
	void lock()		{ASSERT(0);}// m_cCriticalSection.Lock();}
	void unlock()	{ASSERT(0);}// m_cCriticalSection.Unlock();}

	bool m_bDebug;				//!< set this true to debug.(multi-threading off)
    void threadBarrier();		//!< In the Main function, use ThreadBarrier function if you want to control the execution of the thread.
	virtual void threadMain();		//!< inherit this and implement your program.
	virtual void threadOneLoop();	//!< inherit this and implement your program.

private:
//	CCriticalSection m_cCriticalSection;	//!< use as you want	
	static UINT threadControllingFunction( LPVOID pParam );
	enum { NO_THREAD, THREAD_GO, THREAD_ONE_STEP_START, THREAD_ONE_STEP_RUNNING, THREAD_PAUSE, THREAD_END};
	int m_eThreadState;
};



#endif // !defined(AFX_ThreadStub_H__5811B20E_9B3D_48EF_AE21_6EA7BD82463F__INCLUDED_)
