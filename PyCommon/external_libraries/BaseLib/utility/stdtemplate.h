#pragma once
#include <stdio.h>
//#include <tchar.h>
#include <vector>

template <class T>
bool vector_remove(std::vector<T>& vec, const T& elt)
{
	for(int i=0; i<vec.size(); i++)
	{
		if(vec[i]==elt)
		{
			for(int j=i; i<vec.size()-1; i++)
				std::swap(vec[i],vec[i+1]);

			vec.resize(vec.size()-1);
			return true;
		}
	}
	return false;
}








//////////////////////
// 아래는 deprecated.
// use boost::shared_ptr..
/// reference를 저장한다. 즉 new나 delete를 담당하지 않고 pointer를 저장할 뿐이다.
template <class T>
class TFactory
{
public:
	
	TFactory(){ }		virtual ~TFactory(){}
	virtual T* create(int index=-1) { return NULL; }	// do nothing
	virtual T* copy(const T& elt)	{ return NULL;}	// do nothing
	virtual void release(T* pElt)	{ }	// do nothing
	virtual TFactory<T>* clone() const	{ return new TFactory<T>;}
};

// factory 를 어떤거를 쓰는가에 따라 메모리 관리 방식이 달라진다. 메모리 관리는 스스로 책임지기 바람.
// factory는 list, array등에서 쓰임.


/// 스스로 생성하고 스스로 지운다.
template <class T>
class TDefaultFactory : public TFactory<T>
{
public:
	TDefaultFactory(){}		virtual ~TDefaultFactory(){}
	virtual T* create(int index=-1) { return new T(); }
	virtual T* copy(const T& elt)	{ T* ptr=create(); *ptr=elt; return ptr;}	// copy value
	virtual void release(T* pElt)	{ ASSERT(pElt); delete pElt;}
	virtual TFactory<T>* clone() const	{ return new TDefaultFactory<T>;}
};

template <class T, class T2>
class TDefaultFactoryDerived : public TDefaultFactory<T>
{
public:
	TDefaultFactoryDerived (){}
	virtual ~TDefaultFactoryDerived(){}

	virtual T* create(int index=-1) { return (T*)new T2(); }
	virtual TFactory<T>* clone() const	{ return new TDefaultFactoryDerived<T,T2>;}
};

/// 생성은 스스로 하지 않지만, 스스로 지운다.
template <class T>
class TReleaseFactory : public TFactory<T>
{
public:
	TReleaseFactory(){}		virtual ~TReleaseFactory(){}
	virtual T* create(int index=-1) { return NULL; }	// do nothing
	virtual T* copy(const T& elt)	{ ASSERT(0); return NULL;} // do nothing
	virtual void release(T* pElt)	{ if(pElt) delete pElt; }	// release
	virtual TFactory<T>* clone() const	{ return new TReleaseFactory<T>;}
};

/// 생성은 하지만, 지우지 않는다.
template <class T>
class TCreateFactory : public TFactory<T>
{
public:
	TCreateFactory(){}		virtual ~TCreateFactory(){}
	virtual T* create(int index=-1)		{ return new T(); }	// create
	virtual T* copy(const T& elt)		{ return &elt;}		// copy reference
	virtual void release(T* pElt)		{ } // do nothing
	virtual TFactory<T>* clone() const	{ return new TCreateFactory<T>;}
};

