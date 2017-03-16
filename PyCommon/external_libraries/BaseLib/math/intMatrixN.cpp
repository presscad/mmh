#include "stdafx.h"
#include "mathclass.h"
#include "intmatrixn.h"
intmatrixn::intmatrixn()
{
    on = n = m = 0;
}

intmatrixn::intmatrixn( int x, int y )
{
    on = n = m = 0;
    setSize(x,y);
}

intmatrixn::intmatrixn( int x, int y, intvectorn *a )
{
    on = n = m = 0;
	setSize(x,y);

	for( int i=0; i<n; i++ )
	{
		assert(a[i].size()==m);
		v[i] = a[i];
	}
}

intmatrixn::intmatrixn( int x, int y, int* array)
{
    on = n = m = 0;
	setSize(x,y);

	int count=0;
	for(int i=0; i<n; i++)
	{
		for(int j=0; j<m; j++)
		{
			v[i][j]=array[count];
			count++;
		}
	}
}

intmatrixn::~intmatrixn()
{
	if( on>0) delete[] buffer;	
}

void
intmatrixn::getValue( int **d ) const
{
	for( int i=0; i<n; i++ )
	for( int j=0; i<m; j++ )
		d[i][j] = v[i][j];
}

int
intmatrixn::getValue( int row, int column ) const
{
	return v[row][column];
}

void
intmatrixn::setValue( int **d )
{
	for( int i=0; i<n; i++ )
	for( int j=0; i<m; j++ )
		v[i][j] = d[i][j];
}

void
intmatrixn::setValue( int row, int column, int value )
{
	this->v[row].setValue( column, value );
}

void intmatrixn::setValue(int rowstart, int rowend, int columnstart, int columnend, int d)
{
	for(int i=rowstart; i<rowend; i++)
		for(int j=columnstart; j<columnend; j++)
			v[i][j]=d;
}

void intmatrixn::setAllValue(int value)
{
	for(int i=0; i<n; i++)
		v[i].setAllValue(value);
}

void
intmatrixn::setRow( int x, const intvectorn& vec )
{
	for( int i=0; i<m; i++ )
		v[x][i] = vec[i];
}

void
intmatrixn::setColumn( int x, const intvectorn& vec )
{
	for (int i=0; i<n; i++)
		v[i][x] = vec[i];
}

void
intmatrixn::setSize( int nRow, int nColumn )
{
	if(n==nRow && m==nColumn) return;

	if(on<nRow*nColumn)	// buffer가 모자랄때만 다시 생성
	{
		if(on)
			delete[] buffer;
		// 한번에 크게 할당한다. 기존의 모든 벡터마다 loop를 돌면서 할당하는 방법은 대략 좋지않다.
		// 이유는 new operator는 상당히 시간이 오래걸리는 연산이기 때문이다. (delete도 마찬가지)
		buffer=new int[nRow*nColumn];	
		on=nRow*nColumn;
	}

	v.resize(nRow);

	for(int i=0; i<nRow; i++)
	{
		ASSERT(v[i].on==0);
		v[i].n=nColumn;
		if(nColumn!=0)
			v[i].v=&buffer[nColumn*i];
		v[i].m_pParent=this;
	}
	n = nRow;
	m = nColumn;
}

void
intmatrixn::resize( int nRow, int nColumn )
{
	if(nColumn==column() && nRow*nColumn<on)
	{
		int prev_row=row();
		setSize(nRow, nColumn);
		setValue(prev_row, nRow, 0, column(), 0);		
		return;
	}
	intmatrixn backup;
	backup.assign(*this);

	// buffer를 넉넉하게 잡는다. (잦은 재할당을 막는다.)
	int capacity=MAX(on,50);
	// capacity가 nsize를 포함할때까지 doubling
	while(capacity<nRow*nColumn)	capacity*=2;
	if(on<capacity)	// buffer가 모자랄때만 다시 생성
	{
		if(on)
			delete[] buffer;
		// 한번에 크게 할당한다. 기존의 모든 벡터마다 loop를 돌면서 할당하는 방법은 대략 좋지않다.
		// 이유는 new operator는 상당히 시간이 오래걸리는 연산이기 때문이다. (delete도 마찬가지)
		buffer=new int[capacity];	
		on=capacity;
	}

	setSize(nRow, nColumn);
	int backupRow=MIN(backup.row(), nRow);
	int backupColumn=MIN(backup.column(), nColumn);

	this->setAllValue(0);
	for(int i=0; i<backupRow; i++)
		for(int j=0; j<backupColumn; j++)
			(*this)[i][j]=backup[i][j];
}

intmatrixn&
intmatrixn::assign( intmatrixn const& a )
{
    intmatrixn &c = (*this);
    c.setSize( a.row(), a.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[i][j] = a[i][j];

    return c;
}

intmatrixn&
intmatrixn::operator+=( intmatrixn const& a )
{
    intmatrixn &c = (*this);
    c.setSize( a.row(), a.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[i][j] += a[i][j];

    return c;
}

intmatrixn&
intmatrixn::operator-=( intmatrixn const& a )
{
    intmatrixn &c = (*this);
    c.setSize( a.row(), a.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[i][j] -= a[i][j];

    return c;
}

intmatrixn&
intmatrixn::operator*=( int a )
{
    intmatrixn &c = (*this);

    for( int i=0; i<c.row(); i++ )
    for( int j=0; j<c.column(); j++ )
        c[i][j] *= a;

    return c;
}

intmatrixn&
intmatrixn::operator/=( int a )
{
    intmatrixn &c = (*this);

    for( int i=0; i<c.row(); i++ )
    for( int j=0; j<c.column(); j++ )
        c[i][j] /= a;

    return c;
}

intmatrixn&
intmatrixn::mult( intmatrixn const& a, intmatrixn const& b )
{
    intmatrixn &c = (*this);
    assert( a.column()==b.row() );
    c.setSize( a.row(), b.column() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<b.column(); j++ )
    {
        c[i][j] = 0;
        for( int k=0; k<a.column(); k++ )
            c[i][j] += a[i][k] * b[k][j];
    }

    return c;
}

intmatrixn&
intmatrixn::transpose( intmatrixn const& a )
{
    intmatrixn &c = (*this);
    c.setSize( a.column(), a.row() );

    for( int i=0; i<a.row(); i++ )
    for( int j=0; j<a.column(); j++ )
        c[j][i] = a[i][j];

    return c;
}
/*
ostream& operator<<( ostream& os, intmatrixn const& a )
{
    for( int i=0; i< a.row(); i++ ) os << a.v[i] << endl;
    return os;
}

istream& operator>>( istream& is, intmatrixn& a )
{
    for( int i=0; i< a.row(); i++ ) is >> a.v[i];
    return is;
}
*/
void intmatrixn::toVector(intvectorn& vec)
{
	// concat all column vector of this matrix into one large vector.
	vec.setSize(row()*column());

	for(int i=0; i<row(); i++)
	{
		memcpy(&(vec[i*column()]),&(operator[](i).operator[](0)), sizeof(int)*column() );
	}
}

void intmatrixn::makeVectorIndex(int row, int col)
{
	setSize(row, col);

	int count=0;
	for(int i=0; i<row; i++)
		for(int j=0; j<col; j++)
		{
			(*this)[i][j]=count;
			count++;
		}
}

void intmatrixn::makeMatrixIndex(int row, int col)
{
	// (*this)[j*col+i][0]=i, (*this)[j*col+i][1]=j

	setSize(row*col,2);

	int count=0;
	for(int i=0; i<row; i++)
		for(int j=0; j<col; j++)
		{
			(*this)[count][0]=i;
			(*this)[count][1]=j;
			count++;
		}
}

int intmatrixn::findFirstRow(const intvectorn& rowVec) const
{
	for(int i=0; i<row(); i++)
		if((*this)[i]==rowVec)
			return i;
	return -1;
}

void intmatrixn::push_back(const intvectorn& rowVec)
{
	ASSERT(row()==0 || rowVec.size()==column());
	resize(row()+1, rowVec.size());
	setRow(row()-1, rowVec);
}

void intmatrixn::deleteRows(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numRows=end-start;

	for(int i=end; i<row(); i++)
		(*this)[i-numRows].assign((*this)[i]);

	resize(row()-numRows, column());
}

void intmatrixn::deleteCols(int start, int end)
{
	// end이하는 end-start만큼 위로 올라간다. 즉 matrix크기가 end-start만큼 세로로 작아진다.
	int numCols=end-start;

	for(int i=end; i<column(); i++)
	{
		for(int j=0; j<row(); j++)
            (*this)[j][i-numCols]=(*this)[j][i];
	}

	resize(row(), column()-numCols);
}

void intmatrixn::bubbles(int nrow, int nbubbles)
{
	// nrow이하는 nbubble만큼 아래로 내린다. 즉 matrix크기가 nbubble만큼 세로로 커지고, 빈칸이 생긴다.
	int prev_row=row();
	resize(row()+nbubbles, column());

	for(int i=prev_row-1; i>=nrow; i--)
		(*this)[i+nbubbles].assign((*this)[i]);

	for(i=nrow; i<nrow+nbubbles; i++)
		(*this)[i].setAllValue(0);
}

void intmatrixn::bubbleColumns(int ncolumn, int nbubbles)
{
	// ncolumn우측은 nbubble만큼 우측으로 민다. 즉 matrix크기가 nbubble만큼 가로로 커지고, 빈칸이 생긴다.
	int prev_col=column();
	resize(row(), column()+nbubbles);

	for(int i=0; i<row(); i++)
	{
		for(int j=prev_col-1; j>=ncolumn; j--)
			(*this)[i][j+nbubbles]=(*this)[i][j];

		for(int j=ncolumn; j<ncolumn+nbubbles; j++)
			(*this)[i][j]=0;
	}
}
