#pragma once

// class matrixnTransposeView 
// :: for O(1) time matrix-transpose operation.
//        (I seperated the class matrixnTransposedView from matrixn mainly for performance: 
//            I didn't want to use over-sophisticated matrix type such as blitz::matrix (of which element-accesses are way slower),
//        and I didn't want to use dynamic polymorphism either) 

// More detailed explanation:

// blitz::matrix maintains two strides (one for column and another for row).
// Thus, for accessing an element at (i,j):   "value= ptr+stride1*i+stride2*j" should be performed
// which is a little too much computation (in my benchmark, a matrix inversion was about 1.5 times slower.)

// However, gsl_matrix and matrixn maintains one stride for column (a column-major matrix)
// which enables accessing an element at (i,j) using "value= ptr+stride1*i+j".
// This is good for most linear algebra operations except the transpose operation.

// Thus I designed another class for maintaining transposed matrix (a row-major matrix)
// which has the same interface with matrixn.
class matrixnTransposedView
{
protected:
	int n, m, stride;	// n denotes the # of columns (unlike in matrixnView)
	m_real* buffer;	//always reference
public:
	matrixnTransposedView(matrixnTransposedView const& other);	// this = other			: O(1) 
	matrixnTransposedView(matrixn const& other);				// this = other^T		: O(1) 
	~matrixnTransposedView(){}

	void	setDiagonal(m_real value);
	void	setDiagonal(const vectorn& );
	int	rows()  const			{ return m; }
	int	nrows()  const			{ return m; }
	int	cols() const			{ return n; }
	int	ncols() const			{ return n; }

	inline m_real&  value(int i, int j) const		
	{ASSERT(i<rows() && j<cols()); return *((m_real*)(buffer+j*stride)+i);}
	m_real&	 operator()(int i, int j) const		{ return value(i,j);}

	template <class T> 
	void operator=(T const& x);

	TString output(const char* formatString="%f") const;

	// an O(1) operation.
	matrixnView transpose() const;

	// L-value로 사용될수 있는 reference vector 를 return한다.
	vectornView		row(int i)const						
	{ assert(i>=0 && rows()>i); return vectornView (&value(i,0), cols(), stride); }
	
	vectornView		column(int i)const				
	{ assert(i>=0 && cols()>i); return vectornView (&value(0,i), rows(), 1); }

	matrixnTransposedView range(int startRow, int endRow, int startColumn=0, int endColumn=INT_MAX);
	inline const matrixnTransposedView range(int startRow, int endRow, int startColumn=0, int endColumn=INT_MAX) const;	
};


template <class T> 
void matrixnTransposedView::operator=(T const& x)
{
	ASSERT(rows()==x.rows());
	ASSERT(cols()==x.cols());
	for(int i=0; i<rows(); i++)
		for(int j=0; j<rows(); j++)
			value(i,j)=x(i,j);
}