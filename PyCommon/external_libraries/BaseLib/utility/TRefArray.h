#pragma once

template <class T>
/**
 * TArray와 유사. 다른점은, init시 new를 하지 않는다는 점. 즉 factory가 없고, 따라서 원소도 NULL일 수 있다.
 * delete도 하지 않는다.
 */
class TRefArray
{
public:
	TRefArray(int n);
	TRefArray();
	virtual ~TRefArray();

	void init(int nsize);	
	void resize(int nsize); 
	void release();	
	void empty()	{ for(int i=0; i<size(); i++) m_apElement[i]=NULL;}	// 각 원소를 delete는 하지 않고 NULL로 만든다.
	int size() const;
	bool isEmpty(int nIndex)		{ return m_apElement[nIndex]==NULL;}
	T** ptrArray() const			{ return m_apElement;}
	T*  ptr(int i) const			{ return m_apElement[i];}
	T& operator[](int nIndex) const;	
	
	void swap(int i, int j);
	T* replace(int i, T* pElement);
	void pushBack(T* pElement)		{ resize(size()+1); VERIFY(replace(size()-1, pElement)==NULL);}
	//! compare func는 T의 **를 입력으로 받는다. 출력은 element1과 element2의 크기를 비교해서 1,0,-1을 각각 클때, 같을때, 작을때 return한다.
	void sort(int start, int end, int (*compareFunc)(const void** ppElement1, const void** ppElement2));
	
protected:
	T **m_apElement;
	// m_apElement[0]부터 m_apElement[m_nSize-1]까지는 NULL일수도 아닐수도 있다.
	// m_apElement[m_nSize]부터 m_apElement[m_nCapacity]까지는 NULL이다.
	// 그 이후를 사용하려하면 m_apElement자체가 doubling된다.
	int m_nSize;
	int m_nCapacity;
};

//#include "stdafx.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

template <class T>
TRefArray<T>::TRefArray(int n)
{
	m_nSize=0;
	m_nCapacity=0;
	Init(n);
}

template <class T>
TRefArray<T>::TRefArray()
{
	m_nSize=0;
	m_nCapacity=0;
}


template <class T>
TRefArray<T>::~TRefArray()
{
	release();
}

template <class T>
int TRefArray<T>::size() const
{
	return m_nSize;
}

template <class T>
void TRefArray<T>::swap(int i, int j)
{
	T* temp;
	temp=m_apElement[i];
	m_apElement[i]=m_apElement[j];
	m_apElement[j]=temp;
}

template <class T>
T* TRefArray<T>::replace(int i, T* pElement)
{
	T* prevElt;
	ASSERT(i<m_nSize);
	prevElt=m_apElement[i];
	m_apElement[i]=pElement;
	return prevElt;
}

template <class T>
void TRefArray<T>::resize(int nsize)
{
	ASSERT(m_nSize<=m_nCapacity);
	if(nsize<=m_nSize)
	{
		for(int i=nsize; i<m_nSize; i++)
		{
			m_apElement[i]=NULL;
		}
		m_nSize=nsize;
	}
	else if(nsize<=m_nCapacity)
	{
		for(int i=m_nSize; i<nsize; i++)
		{
			m_apElement[i]=NULL; // means empty
		}
		m_nSize=nsize;
	}
	else
	{
		ASSERT(nsize>m_nCapacity && nsize>m_nSize);
		if(m_nCapacity==0)
		{
			init(nsize);
			return;
		}

		// m_nCapacity가 nsize를 포함할때까지 doubling
		for(;m_nCapacity<nsize;)	m_nCapacity*=2;

		T** apTempElement=new T*[m_nCapacity];
		for(int i=0; i<m_nSize; i++)
			// copy
			apTempElement[i]=m_apElement[i];
		for(int i=m_nSize; i<m_nCapacity; i++)
			apTempElement[i]=NULL;

		delete[] m_apElement;
		m_apElement=apTempElement;

		m_nSize=nsize;
	}
}

template <class T>
void TRefArray<T>::init(int nsize)
{
	release();

	m_nSize=nsize;
	m_nCapacity=nsize;
	m_apElement=new T*[nsize];
	for(int i=0; i<nsize; i++) m_apElement[i]=NULL;
}

template <class T>
void TRefArray<T>::release()
{
	if(m_nSize==0) return;

	delete[] m_apElement;

	m_nSize=0;
	m_nCapacity=0;
}

template <class T>
T& TRefArray<T>::operator[](int nIndex) const
{
	return *m_apElement[nIndex];
}

template <class T>
void TRefArray<T>::sort(int start, int end, int (*compareFunc)(const void** ppElement1, const void** ppElement2))
{
	qsort((void*)&(m_apElement[start]), end-start, sizeof(T*), (int (*)(const void*, const void*))compareFunc);
}
