#include "stdafx.h"
#include "mathclass.h"
#include "intVectorN.h"
#include "intMatrixN.h"
#include "../utility/TextFile.h"
intVectorN::intVectorN()
{
	on = n = 0; m_pParent=NULL;
}

intVectorN::intVectorN( int x )
{
    on = n = 0; m_pParent=NULL;
    setSize(x);
}

intVectorN::intVectorN(const intVectorN& other)
{
	on = n = 0; m_pParent=NULL;
    assign(other);
}

intVectorN::intVectorN( int x, int y)
{
	//!< 2D vector
	on=n=0; m_pParent=NULL;
	setSize(2);
	v[0]=x;
	v[1]=y;
}

intVectorN::intVectorN( int x, int y, int z)
{
	//!< 3D vector
	on=n=0; m_pParent=NULL;
	setSize(3);
	v[0]=x;
	v[1]=y;
	v[2]=z;
}

intVectorN::intVectorN( int x, int y, int z, int w)
{
	//!< 4D vector
	on=n=0; m_pParent=NULL;
	setSize(4);
	v[0]=x;
	v[1]=y;
	v[2]=z;
	v[3]=w;
}

intVectorN::intVectorN(const v0::Operator& op)
{
	on = n = 0; m_pParent=NULL;
	op0(op);
}

intVectorN::~intVectorN()
{
    if ( on>0 ) delete[] v;
}

void
intVectorN::setValue( int *d )
{
	for( int i=0; i<n; i++ )
		v[i] = d[i];
}

void intVectorN::setValue( int start, int end, intVectorN const& value )
{
	ASSERT(value.size()==end-start);
	for(int i=start; i<end; i++)
	{
		(*this)[i]=value[i-start];
	}	
}

void
intVectorN::getValue( int *d )
{
	for( int i=0; i<n; i++ )
		d[i] = v[i];
}

void
intVectorN::setSize( int x )
{
	if(n==x)
		return;

	if(m_pParent)
		return resize(x);

	if ( on<x )
	{
		// 아래 assert가 fail이 날수 있는 경우는 on==0인데 n!=x인 경우이다.
		ASSERT(on>=n);
		
		if ( on>0 ) delete[] v;
		v = new int[x];
		on = x;
	}
	n = x;
}

void intVectorN::resize(int nsize)
{
	if(m_pParent)
	{
		// 이는 매트릭스에서 할당한 벡터를 다른 크기로 setSize한 경우에 일어난다.
		// 즉 매트릭스의 한 row만 크기를 바꾼 것이다. 이때는 matrix를 resize한다.

		ASSERT(on==0);
		ASSERT(m_pParent);
		ASSERT(m_pParent->column()==n);
		//안전하게 크기를 줄이지는 않고 늘리기만 한다.
		m_pParent->resize(m_pParent->row(), MAX(n,nsize));
		return;
	}

	if(nsize<=n)
		setSize(nsize);
	else if(nsize<=on)
	{
		int prev_size=n;
		setSize(nsize);
		this->setValue(prev_size, nsize, 0);
	}
	else
	{
		int capacity=MAX(on,10);
		// capacity가 nsize를 포함할때까지 doubling
		while(capacity<nsize)	capacity*=2;

		int prev_size=n;
		intVectorN backup;
		backup.assign(*this);
		setSize(capacity);
		setSize(nsize);
		this->setValue(0, prev_size, backup);
		this->setValue(prev_size, nsize, 0);
	}
}

intVectorN&
intVectorN::operator=( intVectorN const& a )
{
    intVectorN &c = (*this);
    c.setSize( a.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] = a[i];
    return c;
}

intVectorN&
intVectorN::assign( intVectorN const& a )
{
    intVectorN &c = (*this);
    c.setSize( a.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] = a[i];
    return c;
}

intVectorN&
intVectorN::assign( bitVectorN const& a )
{
    intVectorN &c = (*this);
    c.setSize( a.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] = (int)a[i];
    return c;
}

intVectorN&  
intVectorN::assign( vectorN const& a)
{
    intVectorN &c = (*this);
    c.setSize( a.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] = (int)(floor(a[i]+0.5));
    return c;
}

intVectorN&
intVectorN::negate()
{
    intVectorN &c = (*this);
    for( int i=0; i<c.size(); i++ )
        c[i] = -c[i];
    return c;
}
intVectorN operator-( intVectorN const& a)
{
	intVectorN neg=a;
	neg.negate();
	return neg;
}

intVectorN&
intVectorN::add( intVectorN const& a, intVectorN const& b )
{
    intVectorN &c = (*this);
    assert( a.size()==b.size() );
    c.setSize( a.size() );

    for( int i=0; i<a.size(); i++ )
        c[i] = a[i] + b[i];
    return c;
}

intVectorN&
intVectorN::operator+=( intVectorN const& a )
{
    intVectorN &c = (*this);
    assert( c.size()==a.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] += a[i];
    return c;
}


intVectorN& intVectorN::operator+=( int n )
{
	intVectorN &c = (*this);

    for( int i=0; i<c.size(); i++ )
        c[i] += n;
    return c;
}

intVectorN operator+( intVectorN const& a, intVectorN const& b)
{
	intVectorN c;
	c.setSize(a.size());

    for(int i=0; i<a.size(); i++)
	{
		c[i]=a[i]+b[i];
	}

    return c;
}

intVectorN&
intVectorN::sub( intVectorN const& a, intVectorN const& b )
{
    intVectorN &c = (*this);
    assert( a.size()==b.size() );
    c.setSize( a.size() );

    for( int i=0; i<a.size(); i++ )
        c[i] = a[i] - b[i];
    return c;
}

intVectorN&
intVectorN::operator-=( intVectorN const& a )
{
    intVectorN &c = (*this);
    assert( c.size()==a.size() );

    for( int i=0; i<a.size(); i++ )
        c[i] -= a[i];
    return c;
}

intVectorN operator-( intVectorN const& a, intVectorN const& b)
{
	intVectorN c;
	c.setSize(a.size());

    for(int i=0; i<a.size(); i++)
	{
		c[i]=a[i]-b[i];
	}

    return c;
}

int
operator%( intVectorN const& a, intVectorN const& b )
{
    assert( a.size()==b.size() );

    int c=0;
    for( int i=0; i<a.size(); i++ )
        c += a[i] * b[i];
    return c;
}

intVectorN cross(intVectorN const& a, intVectorN const& b)
{
	intVectorN c(3);
	ASSERT(a.size()==3);
	ASSERT(a.size()==b.size());
	
	int ax=a[0];
	int ay=a[1];
	int az=a[2];
	int bx=b[0];
	int by=b[1];
	int bz=b[2];


	c.x()=ay*bz-az*by;
	c.y()=az*bx-ax*bz;
	c.z()=ax*by-ay*bx;

	return c;
}
intVectorN&
intVectorN::mult( intVectorN const& b, int a )
{
    intVectorN &c = (*this);
    c.setSize( b.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] = b[i]*a;
    return c;
}

intVectorN&
intVectorN::operator*=( int a )
{
    intVectorN &c = (*this);

    for( int i=0; i<c.size(); i++ )
        c[i] *= a;
    return c;
}

intVectorN operator*( intVectorN const& a, int b )
{
	intVectorN c=a;
	c*=b;
    return c;
}

intVectorN&
intVectorN::div( intVectorN const& b, int a )
{
    intVectorN &c = (*this);
    c.setSize( b.size() );

    for( int i=0; i<c.size(); i++ )
        c[i] = b[i]/a;
    return c;
}

intVectorN&
intVectorN::operator/=( int a )
{
    intVectorN &c = (*this);

    for( int i=0; i<c.size(); i++ )
        c[i] /= a;
    return c;
}

intVectorN  operator/( intVectorN const& a, int b)
{
	intVectorN c=a;
	c/=b;
    return c;
}

bool operator<(intVectorN const& a, intVectorN const& b)
{
	for(int i=0; i<a.size(); i++)
	{
		if(a[i]>=b[i]) return false;
	}
	return true;
}

bool operator>(intVectorN const& a, intVectorN const& b)
{
	for(int i=0; i<a.size(); i++)
	{
		if(a[i]<=b[i]) return false;
	}
	return true;
}

bool intVectorN::operator==(intVectorN const& other) const
{
	for(int i=0; i<size(); i++)
	{
		if((*this)[i]!=other[i]) return false;
	}
	return true;
}

intVectorN&
intVectorN::mult( intMatrixN const& a, intVectorN const& b )
{
    intVectorN &c = (*this);
    assert( a.column()==b.size() );
    c.setSize( a.row() );

    for( int i=0; i<a.row(); i++ )
    {
        c[i] = 0;
        for( int k=0; k<b.size(); k++ )
            c[i] += a[i][k] * b[k];
    }

    return c;
}

intVectorN&
intVectorN::mult( intVectorN const& b, intMatrixN const& a )
{
    intVectorN &c = (*this);
    assert( a.row()==b.size() );
    c.setSize( a.column() );

    for( int i=0; i<a.column(); i++ )
    {
        c[i] = 0;
        for( int k=0; k<b.size(); k++ )
            c[i] += b[k] * a[k][i];
    }

    return c;
}

intVectorN&  intVectorN::mult( intVectorN const& a, intVectorN const& b)
{
	intVectorN &c=(*this);
	assert( a.size()==b.size() );
    c.setSize( a.size() );
	for ( int i=0; i<c.getSize(); i++ )
		c[i]=a[i]*b[i];

	return c;
}

m_real
intVectorN::length() const
{
    int c=0;
    for( int i=0; i<n; i++ )
        c += this->v[i]*this->v[i];
    return sqrt((m_real)c);
}

m_real
intVectorN::len() const
{
	return this->length();
}
/*
ostream& operator<<( ostream& os, intVectorN const& a )
{
    os << "( ";
    for( int i=0; i< a.size()-1; i++ )
        os << a.v[i] << " , ";
    os << a.v[a.size()-1] << " )";
    return os;
}

istream& operator>>( istream& is, intVectorN& a )
{
	static char	buf[256];
    //is >> "(";
	is >> buf;
    for( int i=0; i< a.size()-1; i++ )
	{
		//is >> a.v[i] >> ",";
		is >> a.v[i] >> buf;
	}
	//is >> a.v[a.size()-1] >> ")";
	is >> a.v[a.size()-1] >> buf;
    return is;
}
*/

intVectorN& intVectorN::concaten(intVectorN const& a, intVectorN const& b)
{
	intVectorN &c = (*this);
	c.setSize(a.size()+b.size());

	for(int i=0; i<a.size(); i++)
		c[i]=a[i];

	for(i=0; i<b.size(); i++)
		c[i+a.size()]=b[i];

	return c;
}

void intVectorN::concaten(intVectorN &a)
{
	int prev_size=size();
	resize(size()+a.size());
	assign(a, prev_size, prev_size+a.size());	
}

int intVectorN::minimum() const
{
	int min=v[0];
	for(int i=1; i<n; i++)
	{
		if(v[i]<min) min=v[i];
	}
	return min;
}

int intVectorN::maximum() const
{
	int max=v[0];
	for(int i=1; i<n; i++)
	{
		if(v[i]>max) max=v[i];
	}
	return max;
}

int intVectorN::sum() const
{
	int sum=0;
	for(int i=0; i<n; i++)
	{
		sum+=v[i];
	}
	return sum;	
}

intVectorN& intVectorN::minimum(intVectorN const& a, intVectorN const& b)
{
	intVectorN &c = (*this);
	*this=a;
	for(int i=0; i<b.size(); i++)
		if(b[i]<v[i]) v[i]=b[i];

	return c;
}

intVectorN& intVectorN::maximum(intVectorN const& a, intVectorN const& b)
{
	intVectorN &c = (*this);
	*this=a;
	for(int i=0; i<b.size(); i++)
		if(b[i]>v[i]) v[i]=b[i];

	return c;
}

intVectorN& intVectorN::minimum(intMatrixN const&mat)
{
	intVectorN &c = (*this);

	intVectorN min=mat[0];

	for(int i=1; i<mat.row(); i++)
	{
		this->minimum(min,mat[i]);
		min=*this;
	}

	return c;
}

intVectorN& intVectorN::maximum(intMatrixN const&mat)
{
	intVectorN &c = (*this);

	intVectorN max=mat[0];

	for(int i=1; i<mat.row(); i++)
	{
		this->maximum(max,mat[i]);
		max=*this;
	}

	return c;
}

intVectorN&  intVectorN::makeSamplingIndex(int nLen, int numSample)
{
	setSize(numSample);

	// simple sampling
	float len=(float)nLen;

	float factor=1.f/(float)numSample;
	for(int i=0; i<numSample; i++)
	{
		float position=((float)i+0.5f)*factor;
		(*this)[i]=(int)(position*len);
	}
	return *this;
}

intVectorN&  intVectorN::makeSamplingIndex2(int nLen, int numSample)
{
	// 첫프레임과 마지막 프레임은 반드시 포함하고 나머지는 그 사이에서 uniform sampling
	if(numSample<3 || nLen<3)
		return makeSamplingIndex(nLen, numSample);
	
	setSize(numSample);
	(*this)[0]=0;
	(*this)[numSample-1]=nLen-1;
	
	intVectorN colon;
	colon.colon(1,numSample-1);

	intVectorN samplingIndex;
	samplingIndex.makeSamplingIndex(nLen-2, numSample-2);
	samplingIndex+=1;
	this->assign(samplingIndex, colon);

	return *this;
}

intVectorN&  intVectorN::assign( intVectorN const& value, intVectorN const& columnIndex)
{
	ASSERT(value.size()==columnIndex.size());
	for(int i=0; i<columnIndex.size(); i++)
	{
		(*this)[columnIndex[i]]=value[i];
	}
	return *this;
}

intVectorN& intVectorN::assign( intVectorN const& value, int start, int end)
{
	ASSERT(value.size()==end-start);
	for(int i=start; i<end; i++)
	{
		(*this)[i]=value[i-start];
	}
	return *this;
}

intVectorN& intVectorN::findIndex(intVectorN const& source, int value)
{
	int count=0;
	for(int i=0; i<source.size(); i++)
	{
		if(source[i]==value)
			count++;
	}
	setSize(count);
	count=0;
	for(i=0; i<source.size(); i++)
	{
		if(source[i]==value)
		{
			(*this)[count]=i;
			count++;
		}
	}
	return *this;
}

intVectorN& intVectorN::findIndex(bitVectorN const& source, bool value, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	int count=0;
	for(int i=start; i<end; i++)
	{
		if(source[i]==value)
			count++;
	}
	setSize(count);
	count=0;
	for(i=start; i<end; i++)
	{
		if(source[i]==value)
		{
			(*this)[count]=i;
			count++;
		}
	}
	return *this;
}

intVectorN&  intVectorN::colon(int start, int end, int stepSize)
{
	int nSize;
	
	if(stepSize==1)
		nSize=(end-start);
	else
		nSize=(end-start+1)/stepSize;

	setSize(nSize);
	int currPos=0;
	for(int i=start; i<end; i+=stepSize)
	{
		(*this)[currPos]=i;
		currPos++;
	}
	ASSERT(currPos==nSize);
	return *this;
}

/*
void intVectorN::output(CString& id)
{
	id+="[";
	char a[50];
	for(int i=0; i<size(); i++)
	{
		id+=itoa((*this)[i],a,10);
		id+=",";
	}
	id+="]";
}*/

TString intVectorN::output(const char* left, const char* typeString, const char* seperator, const char* right)
{
	TString id;
	id+=left;
	for(int i=0; i<size(); i++)
	{
		id.add(typeString, (*this)[i]);

		if(i!=size()-1)
			id.add("%s", seperator);
	}
	id+=right;

	return id;
}

void intVectorN::trace(const char * id)
{
	TString out=output();
	TRACE("%s=%s\n", id, out);
}


intVectorN& intVectorN::op2(const v2::Operator& op, intVectorN const& a, intVectorN const& b)
{
	op.calcInt(*this, a, b);
	return *this;
}

intVectorN& intVectorN::op2(const v2::Operator& op, intVectorN const& a, int b)
{
	op.calcInt(*this, a, b);
	return* this;
}

int intVectorN::findFirstIndex(int value) const
{
	for(int i=0; i<size(); i++)
		if((*this)[i]==value)
			return i;
	return -1;
}

void intVectorN::push_back(int elt)
{
	resize(size()+1);
	(*this)[size()-1]=elt;
}

int intVectorN::popBack()
{
	int out=(*this)[size()-1];
	resize(size()-1);
	return out;
}

int intVectorN::popFront()
{
	int out=(*this)[0];

	for(int i=0; i<size()-1; i++)
	{
		(*this)[i]=(*this)[i+1];
	}
	resize(size()-1);
	return out;
}


/*
intVectorN& intVectorN::op1(unaryOP uop, const intVectorN &a)
{
	switch(uop)
	{
		case OP_RMULT:
		case OP_RADD:
		case OP_RSUB:
		case OP_RDIV:
			{
				CBinaryOP cOP(CUnaryOP::ToBinaryOP(uop));
				ASSERT(size()==a.size());
				for(int i=0; i<a.size(); i++)
					(*this)[i]=cOP.CalcInt((*this)[i],a[i]);
				return *this;
			}
		default:
			{
				CUnaryOP cOP(uop);
				setSize(a.size());
				for(int i=0; i<a.size(); i++)
					(*this)[i]=cOP.CalcInt(a[i]);
			}
	}
	return *this;			
}
*/

intVectorN& intVectorN::op1(const v1::Operator& c, const intVectorN & a)
{
	c.calcInt(*this, a);
	return *this;
}

intVectorN& intVectorN::op1(const v1::Operator& cOP, int a)
{
	cOP.calcInt(*this, a);
	return *this;
}

int intVectorN::count(const s2::Operator& op, int value, int start, int end)
{
	int count=0;
	if(start<0) start=0;
	if(end>size()) end=size();

	for(int i=start; i<end; i++)
		if(op.CalcInt((*this)[i], value)) count++;

	return count;
}

class info
{
public:
	info(){}
	info(const vectorN* pIn, int index){m_pInput=pIn; m_index=index;};
	~info(){}
	const vectorN * m_pInput;
	int m_index;
	static int compareInfo(const void** ppA, const void** ppB)
	{
		info& a=*((info*)*ppA);
		info& b=*((info*)*ppB);
		m_real valA=a.m_pInput->getValue(a.m_index);
		m_real valB=b.m_pInput->getValue(b.m_index);
		if(valA<valB)
			return -1;
		if(valA==valB)
			return 0;
		return 1;
	}
	static info* factory(void* param, int index)	{ return new info((const vectorN*) param, index);};
};



intVectorN&  intVectorN::sortedOrder(vectorN const & input)
{
	//!< input[0]<input[2]<input[1]<input[3]인경우 결과는 [0213]

	CTArray<info> aSort;
	aSort.ChangeDefaultParam((void*)(&input));
	aSort.ChangeFactory(info::factory);
	aSort.Init(input.size());

	aSort.Sort(0, input.size(), info::compareInfo);

	setSize(input.size());
	for(int i=0; i<input.size(); i++)
	{
		(*this)[i]=aSort[i].m_index;
	}
	return *this;
}

void intVectorN::runLengthEncode(const vectorN& source)
{
	/**
	 * RunLength encoding을 구한다.
	 * [1 1 1 3 3 3 3 4 4] -> [0 1 3 3 7 4 9]
	 * 0 부터 1이 3까지 나오고, 3이 7까지 나오고 4가 9까지 나온다는 뜻.
	 * \param source 
	 */

	setSize(0);
	push_back(0);
	int curValue=source[0];

	for(int i=1; i<source.size(); i++)
	{
		if(curValue!=source[i])
		{
			push_back(curValue);
			push_back(i);
			curValue=i;
		}
	}	
}

void intVectorN::runLengthEncode(const intVectorN& source)
{
	/**
	 * RunLength encoding을 구한다.
	 * [1 1 1 3 3 3 3 4 4] -> [0 1 3 3 7 4 9]
	 * 0 부터 1이 3까지 나오고, 3이 7까지 나오고 4가 9까지 나온다는 뜻.
	 * \param source 
	 */

	setSize(0);
	push_back(0);
	int curValue=source[0];

	for(int i=1; i<source.size(); i++)
	{
		if(curValue!=source[i])
		{
			push_back(curValue);
			push_back(i);
			curValue=source[i];
		}
	}
	push_back(curValue);
	push_back(i);
}

void intVectorN::runLengthEncodeCut(const bitVectorN& cutState, int start, int end)
{
	if(start<0) start=0;
	if(end>cutState.size()) end=cutState.size();

	setSize(0);

	push_back(start);  // start
	for(int i=start+1; i<end; i++)
	{
        if(cutState[i])
		{
			push_back(i);	// end
			push_back(i);   // start
		}
	}

	push_back(i);	// end
}

void intVectorN::runLengthEncode(const bitVectorN& source, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	setSize(0);

	bool bFindTrue=false;
	for(int i=start; i<end; i++)
	{
		if(bFindTrue)
		{
			if(!source[i])
			{
				push_back(i);	// end
				bFindTrue=false;
			}
		}
		else
		{
            if(source[i])
			{
				push_back(i);	// start
				bFindTrue=true;
			}
		}
	}

	if(bFindTrue)
		push_back(i);	// end
}

void intVectorN::runLengthDecode(bitVectorN& out, int size)
{
	out.setSize(size);
	out.clearAll();

	for(int i=0; i< this->size()/2; i++)
	{
		int start=(*this)[i*2];
		int end=(*this)[i*2+1];

		out.setValue(start, end, true);
	}	
}
	
intVectorN& intVectorN::op0(const v0::Operator& op)
{
	op.calcInt(*this);
	return *this;
}

intVectorN&  intVectorN::add( int const& a, intVectorN const& b)	{ return op2(v2::add(),b,a);}
intVectorN&  intVectorN::add( intVectorN const& a, int const& b)	{ return op2(v2::add(),a,b);}
intVectorN&  intVectorN::sub( int const& a, intVectorN const& b)	{ op2(v2::sub(),b,a); (*this)*=-1; return *this; }
intVectorN&  intVectorN::sub( intVectorN const& a, int const& b)	{ return op2(v2::sub(),a,b);}	
intVectorN&  intVectorN::operator-=( int value)						{ return op1(v1::rsub(), value);}


void intVectorN::encode(TString& out)
{
	out=output("","%d",",","");
}

void intVectorN::decode(const TString& input)
{
	int i=0;
	TString token;
	setSize(0);
	while(i!=input.length())
	{
		input.token(i, TString(" ,\n"), token);
		push_back(atoi(token));
	}
}

void intIntervals::load(const char* filename)
{
	CTextFile intervalMarking;

	int currSize=0;
	if(intervalMarking.OpenReadFile(filename))
	{
		char* token;
		while(token=intervalMarking.GetToken())
		{
			currSize++;
			if(currSize>size())
				resize(currSize);

            int index=atoi(token);
			start(currSize-1)=index;


			token=intervalMarking.GetToken();
			if(!token)
			{
				Msg::error("Error! cannot load %s", filename);
			}
			else
			{
				int index=atoi(token);
				end(currSize-1)=index;
				if(end(currSize-1)<start(currSize-1))
					Msg::error("Error while loading %s", filename);
			}				
		}
	}
	else
	{
		Msg::print("file open error %s", filename);
	}

	resize(currSize);
}

int intIntervals::findOverlap(int startf, int endf, int startInterval)
{
	for(int i=startInterval; i<size(); i++)
	{
		// case 1:  --=__ or --==---
		if(startf<=start(i) && endf>start(i))
			return i;
		// case 2:  __=-- or __==___
		if(startf>start(i) && startf<end(i))
			return i;
	}

	return -1;
}

void intIntervals::removeInterval(int i)
{
	if(i<size())
	{
		for(int j=i; j<size()-1; j++)
		{
			m_vStart[j]=m_vStart[j+1];
			m_vEnd[j]=m_vEnd[j+1];
		}

		resize(size()-1);
	}
}


void intIntervals::runLengthEncode(const bitVectorN& source, int start, int end)
{
	if(start<0) start=0;
	if(end>source.size()) end=source.size();

	setSize(0);

	bool bFindTrue=false;
	for(int i=start; i<end; i++)
	{
		if(bFindTrue)
		{
			if(!source[i])
			{
				m_vEnd.push_back(i);	// end
				bFindTrue=false;
			}
		}
		else
		{
			if(source[i])
			{
				m_vStart.push_back(i);	// start
				bFindTrue=true;
			}
		}
	}

	if(bFindTrue)
		m_vEnd.push_back(i);	// end

	assert(m_vEnd.size()==m_vStart.size());
}

void intIntervals::encodeIntoVector(intVectorN& out)
{
	out.setSize(size()*2);

	//* start=encoding[grp*2];
	//* end=encoding[grp*2+1];
	for(int grp=0; grp<size(); grp++)
	{
		out[grp*2]=start(grp);
		out[grp*2+1]=end(grp);
	}
}

void intIntervals::decodeFromVector(const intVectorN& in)
{
	setSize(in.size()/2);

	for(int grp=0; grp<size(); grp++)
	{
		start(grp)=in[grp*2];
		end(grp)=in[grp*2+1];
	}
}

void intIntervals::offset(int offset)
{
	m_vStart+=offset;
	m_vEnd+=offset;
}

void intIntervals::toBitVectorN(bitVectorN& bitVector)
{
	bitVector.resize(0);
	for(int i=0; i<size(); i++)
	{
		bitVector.resize(MAX(bitVector.size(), end(i)));

		bitVector.setValue(start(i),end(i),true);
	}
}

