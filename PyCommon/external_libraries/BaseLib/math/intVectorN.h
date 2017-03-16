#pragma once
class intMatrixN;
#include "../utility/Typestring.h"

//  ToPtr<Type>, ToInt<Type> 함수를 쓰면void pointer를 저장하기위해 쓸수도 있다.



/**
 * \ingroup group_math
 *
 * integer형식의 n dimensional vector
 * 
 * \todo 
 *
 * \bug 
 *
 */
class intVectorN
{
  private:
    int     n,
			on;
    int *v;
	intMatrixN* m_pParent;

    // stream
//    friend ostream&  operator<<( ostream&, intVectorN const& );
  //  friend istream&  operator>>( istream&, intVectorN& );

  public:

	intVectorN(const intVectorN& other);
	intVectorN();
	intVectorN( int );
	intVectorN( int x, int y);						//!< 2D vector
	intVectorN( int x, int y, int z);			//!< 3D vector
	intVectorN( int x, int y, int z, int w);  //!< 4D vector
	intVectorN(const v0::Operator&);
	~intVectorN();

	inline int&	x()		{ assert(n>=1); return v[0];};
	inline int&	y()		{ assert(n>=2); return v[1];};
	inline int&	z()		{ assert(n>=3); return v[2];};
	inline int&	w()		{ assert(n>=4); return v[3];};

	void	  release()	{ if(on>0) delete v; n=on=0;};
	void	  getValue( int* );
	int	  getValue( int i ) const { assert(i>=0 && i<n); return v[i]; }
	void	  setValue( int* );
	void	  setValue( int i, int d ) { assert(i>=0 && i<n); v[i] = d; }
	void	  setValue( intVectorN const& columnIndex, int d)	{ for(int i=0; i<columnIndex.size(); i++) (*this)[columnIndex[i]]=d;}
	void	  setValue( int start, int end, int d)				{ for(int i=start; i<end; i++) (*this)[i]=d;}
	void	  setValue( int start, int end, intVectorN const& value );
	void	  setAllValue(int d)	{ for(int i=0; i<n; i++) v[i]=d;};

	int&   operator[](int i) const { assert(i>=0 && i<n); return v[i]; }
	int       size() const { return n; }
	int       getSize() const { return n; }
	void      setSize( int );
	void resize(int nsize);

	m_real   len() const ;
	m_real   length() const ;
	
	// binary operations
	// e.g > a.add(b,c) , c.cross(d,e) ....
	// binaryOP={ OP_ADD, OP_SUB, OP_MULT, OP_DIV, OP_POW, OP_CROSS 등등 ... math_macro.h참고 }
	intVectorN& op2(const v2::Operator&, intVectorN const&, intVectorN const&);
	intVectorN& op2(const v2::Operator&, intVectorN const&, int);
	intVectorN&  add( intVectorN const& a, intVectorN const& b);
	intVectorN&  sub( intVectorN const&, intVectorN const& );
	intVectorN&  add( int const& a, intVectorN const& b);
	intVectorN&  add( intVectorN const& a, int const& b);
	intVectorN&  sub( int const& a, intVectorN const& b);
	intVectorN&  sub( intVectorN const& a, int const& b);
	intVectorN&  mult( intVectorN const&, int );
	intVectorN&  mult( intVectorN const&, intMatrixN const& );
	intVectorN&  mult( intVectorN const&, intVectorN const& );
	intVectorN&  mult( intMatrixN const& a, intVectorN const& b );
	intVectorN&  div( intVectorN const&, int );
	intVectorN& cross(intVectorN const& a, intVectorN const& b);// cross product for 3D vector only		
	intVectorN& minimum(intVectorN const&, intVectorN const&);
	intVectorN& maximum(intVectorN const&, intVectorN const&);
	intVectorN& concaten(intVectorN const& a, intVectorN const& b);

	// source[index]==value인 index들을 찾는다.
	intVectorN& findIndex(intVectorN const& source, int value);
	intVectorN& findIndex(bitVectorN const& source, bool value, int start=0, int end=INT_MAX);
	int count(const s2::Operator&, int value, int start=0, int end=INT_MAX);

	friend int	Angle2D(intVectorN const& a, intVectorN const& b); // calc angle between 0 to 2pi
	friend int	Angle(intVectorN const& a, intVectorN const& b); // calc angle between 0 to pi
	friend int    operator%( intVectorN const&, intVectorN const& );    // dot product
	friend bool operator<(intVectorN const& a, intVectorN const& b);
	friend bool operator>(intVectorN const& a, intVectorN const& b);
	friend bool operator<=(intVectorN const& a, intVectorN const& b) { return !(a>b);};
	friend bool operator>=(intVectorN const& a, intVectorN const& b) { return !(a<b);};
	bool operator==(intVectorN const& other) const;
	// slow binary operations
	friend intVectorN operator+( intVectorN const&, intVectorN const& );
	friend intVectorN operator-( intVectorN const&, intVectorN const& );
	friend intVectorN operator*( intVectorN const& a, int b );
	friend intVectorN operator/( intVectorN const& a, int b);

	// unary operations
	intVectorN& op1(const v1::Operator&, const intVectorN &);
	intVectorN& op1(const v1::Operator&, int);
	void concaten(intVectorN & a);
	intVectorN&  assign( vectorN const& );
	intVectorN&  assign( bitVectorN const& );
	intVectorN&  assign( intVectorN const& );
	intVectorN&  assign( intVectorN const& value, intVectorN const& columnIndex);
	intVectorN&  assign( intVectorN const& value, int start, int end);
	intVectorN&  minimum(intMatrixN const&);	//!< matrix의 각 column별로 minimum value를 뽑는다.
	intVectorN&  maximum(intMatrixN const&);	//!< matrix의 각 column별로 maximum value를 뽑는다.
	intVectorN&  operator=( intVectorN const& );
	intVectorN&  operator+=( intVectorN const& );
	intVectorN&  operator+=( int n );
	intVectorN&  operator-=( intVectorN const& );
	intVectorN&  operator-=( int value);//!<	{ return op1(v1::rsub(), value);};
	intVectorN&  operator*=( int );
	intVectorN&  operator/=( int );
	intVectorN&  sortedOrder(vectorN const & input);	//!< input[0]<input[2]<input[1]<input[3]인경우 결과는 [0213]

	// slow unary operations (negation)
	friend intVectorN operator-( intVectorN const& );

	// void operations
	intVectorN& op0(const v0::Operator&);
	intVectorN&  negate(); // negation
	intVectorN&  makeSamplingIndex(int nLen, int nSample);	// uniform sampling ex. Index(3,5)=(0,1,1,2,2)
	intVectorN&  makeSamplingIndex2(int nLen, int nSample);	// 첫프레임과 마지막 프레임은 반드시 포함하고 나머지는 그 사이에서 uniform sampling
	intVectorN&  colon(int start, int end, int stepSize=1);	// start포함 end포함 안함. (start, start+stepSize, start+stepSize*2,...)
	

	/**
	 * RunLength encoding을 구한다.
	 * [1 1 1 3 3 3 3 4 4] -> [0 1 3 3 7 4 9]
	 * 0 부터 1이 나오고, 3부터 3이나오고, 7부터 4가 나온다.
	 * numGrp=encoding.size()/2;
	 * start=encoding[grp*2];
	 * value=encoding[grp*2+1];
	 * end=encoding[(grp+1)*2];
	 */
	void runLengthEncode(const intVectorN& source);

	/**
	 * RunLength encoding을 구한다.
	 * depricated. -> moved into intIntervals class.
	 * [1 1 1 0 0 0 0 1 0] -> [0 3 7 8]
	 * 0 부터 3까지 1이 나오고, 7부터 8까지 나오고 1이 나온다는 뜻.	 
	 * numGrp=encoding.size()/2;
	 * start=encoding[grp*2];
	 * end=encoding[grp*2+1];
	 */
	void runLengthEncode(const bitVectorN& source, int start=0, int end=INT_MAX);
	void runLengthDecode(bitVectorN& out, int size);
	
	/**
	 * CutState의 RunLength encoding을 구한다.
	 * [1 0 0 1 0 0 0 1 0] -> [0 3, 3 7, 7 9]
	 * 0 부터 3까지 가 첫번째 구간, 3~7이 두번째 구간, 7~9가 세번째 구간.
	int numSeg=encoding.size()/2;
	int start=encoding[seg*2];
	int end=encoding[seg*2+1];
	 */
	void runLengthEncodeCut(const bitVectorN& cutState, int start=0, int end=INT_MAX);

	int minimum() const ;
	int maximum() const;
	int sum() const;
	int findFirstIndex(int value) const;
	
	
	TString output(const char* left="[", const char* typeString="%d", const char* seperator=",", const char* right="]");
	
	void encode(TString& output);
	void decode(const TString& input);
	
	operator int*() const	{ return v;};
	void push_back(int elt);
	void pushBack(int x) { push_back(x);}
	int popBack();
	int popFront();

	friend class intMatrixN;

	// deprecated functions. do not use this
	void runLengthEncode(const vectorN& source);
	void trace(const char * id);	

};


class intIntervals
{
public:
	intIntervals(){}
	~intIntervals(){}
	int numInterval() const			{ return m_vStart.size();}
	int size() const				{ return numInterval();}

	void setSize(int n)				{ m_vStart.setSize(n); m_vEnd.setSize(n);}
	void resize(int n)				{ m_vStart.resize(n); m_vEnd.resize(n);}
	void removeInterval(int i);
	int& start(int iInterval) const	{ return m_vStart[iInterval];}
	int& end(int iInterval) const	{ return m_vEnd[iInterval];}
	void load(const char* filename);

	// [start, end) 과 겹치는 interval의 index를 return. 없으면 -1
	int findOverlap(int start, int end, int startInterval=0);

	/**
	* RunLength encoding을 구한다.
	* depricated. -> moved into intIntervals class.
	* [1 1 1 0 0 0 0 1 0] -> [0 3], [7 8]
	* 0 부터 3까지 1이 나오고, 7부터 8까지 나오고 1이 나온다는 뜻.	 
	*/
	void runLengthEncode(const bitVectorN& source, int start=0, int end=INT_MAX);
	void encodeIntoVector(intVectorN& out);
	void decodeFromVector(const intVectorN& in);
	void offset(int offset);

	void toBitVectorN(bitVectorN& bitVector);
private:
	intVectorN m_vStart;
	intVectorN m_vEnd;
};