#pragma once
#include "intvectorn.h"

/**
 * \ingroup group_math
 *
 * integer형식의 multi-dimensional matrix 
 * \todo 
 *
 * \bug 
 *
 */
class intmatrixn
{
  private:
    int      n,
             m,
			on;
	int* buffer;
    TArray<intvectorn> v;	// a set of column vectors


  public:
    // constructors
     intmatrixn();
     intmatrixn( int nrow, int ncol);
     intmatrixn( int, int, intvectorn* );
	 intmatrixn( int, int, int*);
    ~intmatrixn();

    // inquiry functions
    void		getValue( int** ) const;
    int			getValue( int, int ) const;
    void		setValue( int** );
    void		setValue( int, int, int );
	void	    setValue(int rowstart, int rowend, int columnstart, int columnend, int value);
	void		setAllValue(int);

	intvectorn& row(int i) const		{ assert(i>=0 && n>i); return v[i]; };
    intvectorn& operator[](int i) const { assert(i>=0 && n>i); return v[i]; };
    int			row()    const { return n; }
    int			column() const { return m; }
    void		setSize( int, int );
	void		resize( int nRow, int nColumn );

	void		setRow( int, const intvectorn& );
	void		setColumn( int, const intvectorn& );

	void	  deleteRows(int start, int end);	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다
	void	deleteCols(int start, int end);// end이하는 end-start만큼 왼쪽으로 당겨진다. 즉 matrix크기가 end-start만큼 가로로 작아진다
	void bubbles(int nrow, int nbubbles);// nrow이하는 nbubble만큼 아래로 내린다. 즉 matrix크기가 nbubble만큼 세로로 커지고, 빈칸이 생긴다.
	void bubbleColumns(int ncolumn, int nbubbles);

    intmatrixn& transpose( intmatrixn const& );
    int			det() const;

    intmatrixn& assign( intmatrixn const& );
    intmatrixn& mult( intmatrixn const&, intmatrixn const& );

    intmatrixn& operator*=( int );
    intmatrixn& operator/=( int );
    intmatrixn& operator+=( intmatrixn const& );
    intmatrixn& operator-=( intmatrixn const& );
	int			findFirstRow(const intvectorn& rowVec) const;
	void		push_back(const intvectorn& rowVec);

	void		toVector(intvectorn& vec);	// concat all column vector of this matrix into one large vector.
	void		makeVectorIndex(int row, int col);	// (*this)[i][j]=j*col+i;
	void		makeMatrixIndex(int row, int col); // (*this)[j*col+i][0]=i, (*this)[j*col+i][1]=j

	
	// stream
//    friend ostream& operator<<( ostream&, intmatrixn const& );
  //  friend istream& operator>>( istream&, intmatrixn& );
};

