#include "stdafx.h"
#include "../math/mathclass.h"
#include "MotionRetarget.h"
#include "../math/nr/nr.h"
#include "MotionUtil.h"
#include "MotionUtilTwo.h"
#include "motion.h"
#include "MotionLoader.h"
#include "../baselib/math/Operator.h"
#include "../baselib/utility/OperatorString.h"
#include "../baselib/math/gnuplot.h"
#include "../baselib/math/operatorstitch.h"
#include "Path2D.h"
using namespace MotionUtil;

namespace v1
{
	struct inverseFunction : public _op
	{
		inverseFunction(){}
		virtual void calc(vectorn& c, const vectorn& a) const;
	};

	void inverseFunction ::calc(vectorn& c, const vectorn& a) const
	{
		m_real minTime=a[0];
		m_real maxTime=a[a.size()-1];

		ASSERT(isSimilar(minTime,0.0));

		// c 길이가 정수가 되도록 rounding한다. 1을 더하는 이유는, 정수로 바꾸면 연속된 동작세그먼트간 1프레임 오버랩이 생기기 때문.
		int nDesiredLen=ROUND(maxTime)+1;

		vectorn aa(a);
		aa*=1.0/maxTime*((m_real)nDesiredLen-1.0);
		
		ASSERT(nDesiredLen>=1);

		int curInterval=0;

		c.setSize(nDesiredLen);
		for(int i=0; i<nDesiredLen; i++)
		{
			m_real t=m_real (i);
			int j=curInterval+1;
			for(; j<aa.size(); j++)
			{
				if(aa[j]>=t-FERR)
					break;
			}

			curInterval=j-1;

			m_real frac=((m_real)i-aa[curInterval])/(aa[j]-aa[curInterval]);

			c[i]=(m_real(curInterval)+frac);
		}
	}

}


namespace m0
{
	// curve의 가속도를 최대한 유지하면서, a[time]--> mCon이 되도록 고친다.
	// curve의 시작 위치와, 시작, 끝 속도는 변하지 않는다.
	struct adjustOnline: public _op
	{
		adjustOnline(int time, vectorn const & con):mTime(time), mCon(con){}

		int mTime;
		vectorn mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 가속도를 최대한 유지하면서, 모든 a[time]--> mCon이 되도록 고친다. 즉 constraint개수제한 없음.
	// curve의 시작 위치와, 시작, 끝 속도는 변하지 않는다.
	struct adjustOnlineMultiCon: public _op
	{
		adjustOnlineMultiCon(intvectorn const& time, matrixn const & con):mTime(time), mCon(con){}

		intvectorn const& mTime;
		matrixn const& mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 가속도를 최대한 유지하면서, a[time]--> mCon이 되도록 고친다.
	// curve의 시작 위치와, 시작 속도는 변하지 않는다.
	struct adjustOnline2: public _op
	{
		adjustOnline2(int time, vectorn const & con):mTime(time), mCon(con){}

		int mTime;
		vectorn mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 가속도를 최대한 유지하면서, a[time]-->mCon이 되도록 고친다.
	// curve의 시작위치와 끝 위치는 변하지 않는다.
	struct adjust: public _op
	{
		adjust(int time, vectorn const & con):mTime(time), mCon(con){}

		int mTime;
		vectorn mCon;
		virtual void calc(matrixn& c) const;
	};

	// curve의 속도를 최대한 유지하면서, 그 합이 특정 값이 되도록 한다.
	struct adjustSumOnline: public _op
	{
		adjustSumOnline(int time, vectorn const & con):mTime(time), mDelta(con){}

		int mTime;
		vectorn mDelta;
		virtual void calc(matrixn& c) const;
	};

	void adjustOnline::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'.. -r..)^2

		// constraint :
		// r.(start) == r'.(start)
		// r.(end)==r'.(end)
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=4;
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);
		// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
		
		vectorn weight;
		weight.setSize(nsample-2);
		HessianQuadratic h(nsample);
		intvectorn index;
		vectorn coef;
		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(3, 1.0, -2.0, 1.0);

			weight[i]=SQR(sop::clampMap(i, 0, mTime, 1.0, 2.2));

			coef*=weight[i];
			h.addSquaredH(index, coef);
		}

		H=h.H;

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][row-2]=1.0;
		A[2][row-1]=-1.0;
		A[3][mTime]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
#define USE_LUDCMP
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		invAugmented.inverse(Augmented);
#endif
		vectorn c(row-2);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-2; i++)
				c[i]=f(i)-2.0*f(i+1)+f(i+2);

			// set hessian residuals.
			h.R.setAllValue(0);
			for(int i=0; i<nsample-2; i++)
			{
				index.setValues(3, i, i+1,i+2);
				coef.setValues(4, 1.0, -2.0, 1.0, -1.0*c[i]);
				coef*=weight[i];
				h.addSquaredR(index, coef);
			}
			d.range(0, h.R.size())=h.R;

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=f(row-2)-f(row-1);
			d[row+3]=mCon[dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}

	void adjustOnlineMultiCon::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'.. -r..)^2

		// constraint :
		// r.(start) == r'.(start)
		// r.(end)==r'.(end)
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=3+mCon.rows();
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);
		// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
		H.setAllValue(0.0);
		for(int i=0; i<nsample; i++)
		{
			if(0<=i && i<nsample-2)
			{
				H[i][i]+=2.0;
				H[i][i+1]-=4.0;
				H[i][i+2]+=2.0;
			}
			if(1<=i && i<nsample-1)
			{
				H[i][i-1]-=4.0;
				H[i][i]+=8.0;
				H[i][i+1]-=4.0;
			}
			if(2<=i && i<nsample)
			{
				H[i][i-2]+=2.0;
				H[i][i-1]-=4.0;
				H[i][i]+=2.0;
			}
		}

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][row-2]=1.0;
		A[2][row-1]=-1.0;

		for(int i=0; i<mTime.size(); i++)
			A[3+i][mTime[i]]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
#define USE_LUDCMP
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		invAugmented.inverse(Augmented);
#endif
		vectorn c(row-2);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-2; i++)
				c[i]=f(i)-2.0*f(i+1)+f(i+2);

			// set hessian residuals.
			for(int i=0; i<nsample; i++)
			{
				d[i]=0;
				if(0<=i && i<nsample-2)
					d[i]+=2.0*c[i];
				if(1<=i && i<nsample-1)
					d[i]-=4.0*c[i-1];
				if(2<=i && i<nsample)
					d[i]+=2.0*c[i-2];
			}

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=f(row-2)-f(row-1);

			for(int i=0; i<mTime.size(); i++)
				d[row+3+i]=mCon[i][dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}

	void adjust::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'.. -r..)^2

		// constraint :
		// r.(start) == r'.(start)
		// r.(end)==r'.(end)
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=5;
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);
		// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2])^2} 의 hessian(=gradient의 계수).
		H.setAllValue(0.0);
		for(int i=0; i<nsample; i++)
		{
			if(0<=i && i<nsample-2)
			{
				H[i][i]+=2.0;
				H[i][i+1]-=4.0;
				H[i][i+2]+=2.0;
			}
			if(1<=i && i<nsample-1)
			{
				H[i][i-1]-=4.0;
				H[i][i]+=8.0;
				H[i][i+1]-=4.0;
			}
			if(2<=i && i<nsample)
			{
				H[i][i-2]+=2.0;
				H[i][i-1]-=4.0;
				H[i][i]+=2.0;
			}
		}

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][row-2]=1.0;
		A[3][row-1]=1.0;
		A[4][mTime]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
#define USE_LUDCMP
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		invAugmented.inverse(Augmented);
#endif
		vectorn c(row-2);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-2; i++)
				c[i]=f(i)-2.0*f(i+1)+f(i+2);

			// set hessian residuals.
			for(int i=0; i<nsample; i++)
			{
				d[i]=0;
				if(0<=i && i<nsample-2)
					d[i]+=2.0*c[i];
				if(1<=i && i<nsample-1)
					d[i]-=4.0*c[i-1];
				if(2<=i && i<nsample)
					d[i]+=2.0*c[i-2];
			}

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=f(row-2);
			d[row+3]=f(row-1);
			d[row+4]=mCon[dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}

	void adjustOnline2::calc(matrixn& curve) const
	{
		// minimize velocity difference
		// objective function to be minimized :
		// f=sum(r'. -r.)^2

		// constraint :
		// r.(start) == r'.(start)		
		// r(time)==pos2d

		/* LAGRANGIAN MULTIPLIER version*/
		//    Augmented* x= d , w=(X, lamda)
		//   (H A^T) (X    )= (d)
		//   (A 0  ) (lamda)
		int row=curve.rows();
		int nsample=row;
		int numCon=3;
		matrixn Augmented(row+numCon, row+numCon);
		matrixnView H=Augmented.range(0,row, 0, row);
		matrixnView A=Augmented.range(row, row+numCon, 0, row);
		matrixnView At=Augmented.range(0, row, row, row+numCon);
		vectorn x, d(row+numCon);

		HessianQuadratic h(row);
		intvectorn index;
		vectorn coef;

		// set hessian matrix. ( sum_0^{n-2} {(x[i]-x[i+1]-c[i])^2} 의 hessian(=gradient의 계수).
		for(int i=0; i<nsample-1; i++)
		{
			index.setValues(2, i, i+1);
			coef.setValues(2, -1.0, 1.0);
			h.addSquaredH(index, coef);
		}

		for(int i=0; i<nsample-2; i++)
		{
			index.setValues(3, i, i+1, i+2);
			coef.setValues(3, 1.0, -2.0, 1.0);
			h.addSquaredH(index, coef);
		}

		H=h.H;

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;
		A[1][1]=1.0;
		A[2][mTime]=1.0;

		At.transpose(A);
		Augmented.range(row, row+numCon, row, row+numCon).setAllValue(0.0);
//#define USE_LUDCMP
#ifdef USE_LUDCMP
		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);
#else
		matrixn invAugmented;
		invAugmented.inverse(Augmented);
#endif

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]

			for(int i=0; i<row-1; i++)
			{
				index.setValues(2, i, i+1);
				coef.setValues(3, -1.0, 1.0, -1.0*(f(i+1)-f(i)));
				h.addSquaredR(index, coef);
			}

			for(int i=0; i<row-2; i++)
			{
				index.setValues(3, i, i+1, i+2);
				coef.setValues(4, 1.0, -2.0, 1.0, -1.0*(f(i)-2.0*f(i+1)+f(i+2)));
				h.addSquaredR(index, coef);
			}

			d.range(0, nsample)=h.R;

			// set constraint
			d[row]=f(0);
			d[row+1]=f(1);
			d[row+2]=mCon[dim];
			//NR::frprmn(p, 1.0e-6, iter, fret, linstitch::f, linstitch::df);

#ifdef USE_LUDCMP
			x=d;
			NR::lubksb(LU,indx,x);
#else
			x.multmat(invAugmented, d);
#endif
			// save results.
			for(int i=0; i<row; i++)
				f(i)=x[i];
		}
	}
	void adjustSumOnline::calc(matrixn& curve) const
	{
		// minimize acceleration difference
		// objective function to be minimized :
		// f=sum(r'. -r.)^2
		//  f=sum ( (x[i]-x[i-1])-(f[i]-f[i-1]))^2 

		// constraint :
		// r(start) == r'(start)
		// sum_start^time (r(i))== mDelta;

		// LAGRANGIAN MULTIPLIER version
		//    Augmented * w      =    d ,               w=(X, lamda)
		//   (H A^T)     (X)     =   (d)
		//   (A 0  )     (lamda)

		const int numCon=3;
		int xsize=curve.rows();
		matrixn Augmented(xsize+numCon, xsize+numCon);
		matrixnView H=Augmented.range(0,xsize, 0, xsize);
		matrixnView A=Augmented.range(xsize, xsize+numCon, 0, xsize);
		matrixnView At=Augmented.range(0, xsize, xsize, xsize+numCon);

		vectorn x, d(xsize+numCon);
		// set hessian matrix. ( f의 hessian(=gradient의 계수).
		H.setAllValue(0.0);
		for(int i=0; i<xsize; i++)
		{
			if(1<=i && i<xsize)
			{
				H[i][i]+=2.0;
				H[i][i-1]-=2.0;
			}
			if(0<=i && i<xsize-1)
			{
				H[i][i+1]-=2.0;
				H[i][i]+=2.0;
			}
		}

		// set constraint matrix (constraint Ax=b)
		A.setAllValue(0.0);
		A[0][0]=1.0;	// con1
		A.row(1).range(0, mTime+1).setAllValue(1.0);	// con2
		A[2][xsize-1]=1.0;	// con3
		At.transpose(A);

		Augmented.range(xsize, xsize+numCon, xsize, xsize+numCon).setAllValue(0.0);

		DP p;
		matrixn LU(Augmented);
		Vec_INT indx(Augmented.rows());
		NR::ludcmp(LU,indx,p);

		vectorn c(xsize-1);

		for(int dim=0; dim<curve.cols(); dim++)
		{
#define f(xxx)	curve[xxx][dim]


			for(int i=0; i<xsize-1; i++)
				c[i]=f(i+1)-f(i);

			// set constraint
			d[xsize]=f(0);
			d[xsize+1]=0;
			for(int i=0; i<mTime+1; i++)
				d[xsize+1]+=f(i);
			d[xsize+1]+=mDelta[dim];
			d[xsize+2]=f(xsize-1);
			// set hessian residuals.
			for(int i=0; i<xsize; i++)
			{
				d[i]=0;
				if(1<=i && i<xsize)
					d[i]=2.0*c[i-1];
				if(0<=i && i<xsize-1)
					d[i]-=2.0*c[i];
			}

			x=d;
			NR::lubksb(LU,indx,x);

			// save results.
			for(int i=0; i<xsize; i++)
				f(i)=x[i];

		}
	}
}

RetargetOnline2D::RetargetOnline2D(Motion& target, int start, int eRetargetQMethod)
:mTarget(target)
{
	getCurve(start);
}

void RetargetOnline2D::getCurve(int start)
{	
	mStart=start-2;	// 첫 두프레임은 변하지 않는다.
	if(mStart<1) mStart=1;
}

void RetargetOnline2D::adjust(int time, quater const& oriY, vector3 const& pos2D)
{
	// constrained optimization for root orientation.
	// acceleration difference should be minimized.

	// constriant: positional constraint, orientational constraint.
	

	adjust(time, oriY);
	adjust(time, pos2D);
}

void RetargetOnline2D::adjust(int time, vector3 const& pos2D)
{
	vectorn con(2);
	con[0]=pos2D.x;
	con[1]=pos2D.z;

	matrixn curve;

	curve.setSize(mTarget.NumFrames()-mStart, 2);

	for(int i=mStart; i<mTarget.NumFrames(); i++)
	{
		curve[i-mStart][0]=mTarget.Pose(i).m_aTranslations[0].x;
		curve[i-mStart][1]=mTarget.Pose(i).m_aTranslations[0].z;
	}
	
	curve.op0(m0::adjustOnline(time-mStart,  con));

	for(int i=mStart; i<mTarget.NumFrames(); i++)
	{
		mTarget.Pose(i).m_aTranslations[0].x=curve[i-mStart][0];
		mTarget.Pose(i).m_aTranslations[0].z=curve[i-mStart][1];
	}
}


void RetargetOnline2D::adjust(int time, quater const& oriY)
{
	quater qdiff;
	qdiff.difference(mTarget.Pose(time).m_rotAxis_y, oriY);

	m_real deltarot=qdiff.rotationAngleAboutAxis(vector3(0,1,0));
	adjust(time, deltarot);
}

void RetargetOnline2D::adjustSafe(int time, m_real deltarot)
{
	mCurve.setSize(mTarget.NumFrames()-mStart,1);

	for(int i=mStart; i<mTarget.NumFrames(); i++)
	{
		mTarget.Pose(i).decomposeRot();
	}
	mCurve[0][0]=mTarget.Pose(mStart).m_rotAxis_y.rotationAngleAboutAxis(vector3(0,1,0));

	quater q;
	for(int i=mStart+1; i<mTarget.NumFrames(); i++)
	{
		q.difference(mTarget.Pose(i-1).m_rotAxis_y, mTarget.Pose(i).m_rotAxis_y);
		mCurve[i-mStart][0]=mCurve[i-mStart-1][0]+q.rotationAngleAboutAxis(vector3(0,1,0));
	}

	vectorn con(1);
	con[0]=deltarot+mCurve[time-mStart][0];
	mCurve.range(0, mCurve.rows(), 0, 1).op0(m0::adjust(time-mStart, con));


	for(int i=mStart; i<mTarget.NumFrames(); i++)
	{
		mTarget.Pose(i).m_rotAxis_y.setRotation(vector3(0,1,0), mCurve[i-mStart][0]);		
		mTarget.Pose(i).m_aRotations[0].mult(mTarget.Pose(i).m_rotAxis_y, mTarget.Pose(i).m_offset_q);
	}

}
void RetargetOnline2D::adjust(int time, m_real deltarot)
{
	if(mRetargetQMethod==RETARGET_ROTY)
	{
		mCurve.setSize(mTarget.NumFrames()-mStart,1);

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mTarget.Pose(i).decomposeRot();
		}
		mCurve[0][0]=mTarget.Pose(mStart).m_rotAxis_y.rotationAngleAboutAxis(vector3(0,1,0));

		quater q;
		for(int i=mStart+1; i<mTarget.NumFrames(); i++)
		{
			q.difference(mTarget.Pose(i-1).m_rotAxis_y, mTarget.Pose(i).m_rotAxis_y);
			mCurve[i-mStart][0]=mCurve[i-mStart-1][0]+q.rotationAngleAboutAxis(vector3(0,1,0));
		}

		vectorn con(1);
		con[0]=deltarot+mCurve[time-mStart][0];
		mCurve.range(0, mCurve.rows(), 0, 1).op0(m0::adjustOnline(time-mStart, con));


		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mTarget.Pose(i).m_rotAxis_y.setRotation(vector3(0,1,0), mCurve[i-mStart][0]);		
			mTarget.Pose(i).m_aRotations[0].mult(mTarget.Pose(i).m_rotAxis_y, mTarget.Pose(i).m_offset_q);
		}

		mTarget._reconstructPosByDifference(mStart-1);
	}
	else
	{
		mTarget.CalcInterFrameDifference(mStart-1);

		// dv, drot
		mCurve.setSize(mTarget.NumFrames()-mStart,1 );

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mCurve[i-mStart][0]=mTarget.Pose(i).m_dq.rotationAngleAboutAxis(vector3(0,1,0));
		}

		vectorn con(1);
		con[0]=deltarot;
		mCurve.range(0, mCurve.rows(), 0, 1).op0(m0::adjustSumOnline(time-mStart, con));

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mTarget.Pose(i).m_dq.setRotation(vector3(0, mCurve[i-mStart][0],0));
		}

		mTarget.ReconstructDataByDifference(mStart-1);
	}
}

void RetargetOnline2D::adjust(int criticalTimeBefore, int criticalTimeAfter, intvectorn& times)
{
	Motion& inout=mTarget;
	int playEnd=mStart+2;

	//    playEnd         criticalTimeBefore
	// ---|---------------|-------|
	//    | adjustMot     |
	//                    |leftMot|


	// case 1:
	// ---|-----------|-------|
	//                criticalTimeAfter


	// case 2:
	// ---|-------------------|-------|
	//                criticalTimeAfter


	int leftMotionSize=inout.NumFrames()-criticalTimeBefore;

	if(leftMotionSize<0) 
	{
		printf("strange error\n");
		return;
	}
//#define USE_ONLINE_ADJUST
#ifdef USE_ONLINE_ADJUST
	// online adjust 사용.

	int start=playEnd-1;	// 첫 한프레임은 변하지 않는다.

	// timewarping function
	//                             +   (y=x)                                      + y=P(x)
	//                           .                                              .
	//                         .                                              .
	//                       .                                             .
	//                     *                                            * 
	//                   .                                          .
	//                 .                                         .
	//			     .                                        .
	//             .                                        .
	//           .                                        .
	//           0 1 2 3 4 5 6 7 8 9            ->        0 1 2 3 4 5 6 7 8 9 0 1 2
	//			 PE        C         N                    PE            C'          N'

	// timewarping function을 P라하면, 
	// P(C')=C,
	// P(0)=0
	// P(N'-1)=N 이 나오는 P(x), x=0,1,...,N' 을 구해야한다.

	// 하지만 부드러운 P를 구하기가 쉽지 않다. (N'이 미정이기 때문)

	// 반면, P의 역함수 Q는 구하기 쉽다.
	// -->  Q(0)=0, Q(C)=C' 인 임의의 부드러운 함수를 구하면 됨.
	// 이로부터 N'을 구하고,

	vectorn Q;
	Q.linspace(0, inout.NumFrames()-start-1, inout.NumFrames()-start);
	
	m0::adjustOnline a(criticalTimeBefore-start, vectorn(1, criticalTimeAfter-start)); 
	a.calc(Q.cols());

	int N_=ROUND(Q[Q.size()-1])+start+1;

	// 다시 P를 만든다.
	vectorn P;
	P.linspace(0, N_-start-1, N_-start);
	
	m0::adjustOnline b(criticalTimeAfter-start, vectorn(1, criticalTimeBefore-start));
	b.calc(P.cols());

	// P(N'-1)=N-1이라는 보장이 없다. 하지만 비슷할것이다. 노멀라이즈 한다.
	P*=(inout.NumFrames()-start-1)/P[P.size()-1];

	P+=start;

	for(int i=times.size()-1; i>=0; i--)
	{
		if(times[i]>=start)
			times[i]=P.argNearest(times[i])+start;
		else break;
	}

	static Motion temp2;
	MotionUtil::timewarpingLinear(temp2, inout, P);

	inout.Resize(inout.NumFrames()+P.size()-Q.size());

	for(int i=start; i<start+P.size(); i++)
		inout.Pose(i)=temp2.Pose(i-start);

#else
	// USE_PIECEWISE_LINEAR_TIMEWARPING
	static Motion temp2;
	vectorn timewarpFunction(criticalTimeAfter-playEnd+1);
	timewarpFunction.linspace(playEnd, criticalTimeBefore);	

	for(int i=times.size()-1; i>=0; i--)
	{
		if(times[i]>=criticalTimeBefore)
			times[i]+=criticalTimeAfter-criticalTimeBefore;
		else if(times[i]>=playEnd)
			times[i]=timewarpFunction.argNearest(times[i])+playEnd;
		else break;
	}

	MotionUtil::timewarpingLinear(temp2, inout, timewarpFunction);

	if(criticalTimeAfter>criticalTimeBefore)
	{
		// increasing size
		int numFrameOld=inout.NumFrames();
		inout.Resize(inout.NumFrames()+criticalTimeAfter-criticalTimeBefore);
		for(int i=numFrameOld-1; i>=criticalTimeBefore; i--)
			inout.Pose(i-numFrameOld+inout.NumFrames())=inout.Pose(i);

		for(int i=playEnd; i<criticalTimeAfter; i++)
			inout.Pose(i)=temp2.Pose(i-playEnd);
	}
	else
	{
		// decreasing size
		for(int i=0; i<leftMotionSize; i++)
			inout.Pose(criticalTimeAfter+i)=inout.Pose(criticalTimeBefore+i);
		inout.Resize(inout.NumFrames()+criticalTimeAfter-criticalTimeBefore);


		for(int i=playEnd; i<criticalTimeAfter; i++)
			inout.Pose(i)=temp2.Pose(i-playEnd);
	}
#endif
}

void decompose(quater const& q, vector3& qq)
{
	quater qy, qoffset;
	q.decompose(qy, qoffset);
	qq[0]=qy.rotationAngle(vector3(0,1,0));
	qoffset.align(quater(1,0,0,0));
	vector3 rot=qoffset.rotationVector();
	qq[1]=rot.x;
	qq[2]=rot.z;
}

void inverseDecompose(quater& q, vector3 const& qq)
{
	quater qy, qoffset;
	qy.setRotation(vector3(0,1,0), qq[0]);
	qoffset.setRotation(vector3(qq[1],0,qq[2]));
	q.mult(qy, qoffset);
}

void alignAngle(m_real prev, m_real& next)
{
	m_real nprev=prev;
	// prev는 [-PI,PI]에 있다고 보장을 못한다. 따라서 이범위에 들어오는 nprev를 구한다.
	while(nprev>M_PI+FERR)
		nprev-=2.0*M_PI;
	while(nprev<-1.0*M_PI-FERR)
		nprev+=2.0*M_PI;

	if(ABS(nprev-next)>M_PI)
	{
		if(next>nprev)
			next-=2.0*M_PI;
		else
			next+=2.0*M_PI;
	}

	// 다시 원래의 prev범위로 되돌린다.
	next+=prev-nprev;
}

void quaterlinstitch_decompose(quaterN const& a, quaterN const& b, quaterN & c)
{
	vector3N ma, mb, mc;

	ma.setSize(a.rows());
	mb.setSize(b.rows());

	for(int i=0; i<ma.rows(); i++)
		decompose(a.row(i), ma.row(i));

	for(int i=0; i<mb.rows(); i++)
		decompose(b.row(i), mb.row(i));

	// -pi < ma[0], mb[0]< pi
	// we need to align them so that inter frame difference do not exceeds pi

	for(int i=0; i<ma.rows()-1; i++)
		alignAngle(ma[i][0], ma[i+1][0]);
	alignAngle(ma[ma.rows()-1][0], mb[0][0]);
	for(int i=0; i<mb.rows()-1; i++)
		alignAngle(mb[i][0], mb[i+1][0]);

	mc.linstitch(ma, mb);

	c.setSize(mc.rows());
	for(int i=0; i<c.rows(); i++)
		inverseDecompose(c.row(i), mc.row(i));
}

vector3 quaterLog(quater const& a, quater const& ref)
{
	quater b;
	b.difference(ref, a);
	return b.log();
}

void quaterlinstitch_log(quaterN const& a, quaterN const& b, quaterN & c)
{
	vector3N ma, mb, mc;

	ma.setSize(a.rows());
	mb.setSize(b.rows());

	quater refCoord;
	refCoord.safeSlerp(a.row(a.rows()-1), b.row(0), 0.5);

	for(int i=0; i<ma.rows(); i++)
		ma.row(i)=quaterLog(a.row(i), refCoord);

	for(int i=0; i<mb.rows(); i++)
		mb.row(i)=quaterLog(b.row(i), refCoord);

	mc.linstitch(ma, mb);

	c.setSize(mc.rows());
	for(int i=0; i<c.rows(); i++)
		c.row(i).mult(mc.row(i).exp(), refCoord);
}

void concatUsingOpponent(Motion& front, Motion & add)
{
	int numFrameOld = front.NumFrames();
	int addMount= add.NumFrames();

	// first concat
	// 가정: 한프레임이 겹친다. 즉 front.Pose(NumFrame()-1)은, add.Pose(0)과 같은 순간, 즉
	// contact force minimum에 해당한다.
	front.pose2(numFrameOld-1).decomposeRot();
#define USE_OPPONENT_POS
#ifdef USE_OPPONENT_POS
	vector3 globalOpponent;
	globalOpponent.rotate(front.pose2(numFrameOld-1).m_rotAxis_y, front.pose2(numFrameOld-1).m_oppenentPos);

	quater qq, q, qoff;
	qq.axisToAxis(add.pose2(0).m_oppenentPos, globalOpponent);
	qq.decompose(q, qoff);
#else
	quater q=front.pose2(numFrameOld-1).m_rotAxis_y;
#endif
	add.pose2(0).m_rotAxis_y=q;
	add.pose2(0).m_aTranslations[0].x=front.pose2(numFrameOld-1).m_aTranslations[0].x;
	add.pose2(0).m_aTranslations[0].z=front.pose2(numFrameOld-1).m_aTranslations[0].z;
	add.ReconstructDataByDifference(0);

	front.Resize(front.NumFrames()+addMount-1);	
	for(int i=numFrameOld; i<front.NumFrames();i++)
		front.pose2(i).Clone(&add.pose2(i-numFrameOld+1));
}



void MotionUtil::stitchUsingOpponent(int nPrevSafe, int afterSafe, Motion& front, Motion const& addd)
{
	static Motion add;
	add.Init(addd);

	int numFrameOld = front.NumFrames();
	int addMount= add.NumFrames();
	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	int startSafe=MAX(0, numFrameOld-nPrevSafe-2);

	concatUsingOpponent(front, add);

	// second stitch:
	int nForwardSafe=MIN(addMount, afterSafe+2);

	front.discontinuity().clearAt(numFrameOld);

	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(add);

	vector3N trajA, trajB, traj;
	quaterN otrajA, otrajB, otraj;

	frontGetSignal.root(trajA, startSafe, numFrameOld);
	addGetSignal.root(trajB, 0, nForwardSafe);
	traj.linstitch(trajA, trajB);
#ifdef TRACE_SIGNAL
	matrixn t;
	t.assign(traj);
	t.op0(m0::drawSignals("root.bmp", 0,0,true));
#endif
	frontSetSignal.root(traj, startSafe);

	// stitch opponent pos
	MotionUtilTwo::getOpponentPos(front, trajA, startSafe, numFrameOld);
	MotionUtilTwo::getOpponentPos(add, trajB, 0, nForwardSafe);
	//traj.linstitch(trajA, trajB);
	traj.c0stitch(trajA, trajB);
	MotionUtilTwo::setOpponentPos(front, traj, startSafe);

	for(int i=0; i<add.NumJoints(); i++)
	{
		frontGetSignal.joint(i,otrajA, startSafe, numFrameOld);
		addGetSignal.joint(i, otrajB, 0, nForwardSafe);


		//otraj.linstitch(otrajA, otrajB);

		if(i==0)
			quaterlinstitch_decompose(otrajA, otrajB, otraj);
		else
			quaterlinstitch_log(otrajA, otrajB, otraj);

		//otraj.c0stitch(otrajA, otrajB);
		frontSetSignal.joint(i,otraj, startSafe);		

#ifdef TRACE_SIGNAL
		t.assign(otraj);
		t.op0(m0::drawSignals(sz1::format("joint%d.bmp", i), 0,0,true));
		matrixn t2(otrajA.rows()+otrajB.rows(),4);
		t2.range(0,otrajA.rows()).assign(otrajA);
		t2.range(otrajA.rows(),t2.rows()).assign(otrajB);
		t2.op0(m0::drawSignals(sz1::format("joint%do.bmp", i), 0,0,true));
#endif
	}
}


void MotionUtil::stitchUsingRoot(int prevSafe, int afterSafe, Motion& front, Motion const& add)
{
	int numFrameOld = front.NumFrames();
	int addMount= add.NumFrames();
	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	int startSafe=MAX(0, numFrameOld-prevSafe-2);

	front.Resize(front.NumFrames()+addMount-1);	
	
	int numJoint=front.NumJoints();
	for(int i=numFrameOld; i<front.NumFrames();i++)		
		front.Pose(i).Clone(&add.Pose(i-numFrameOld+1));

	// second stitch:
	int nForwardSafe=MIN(addMount, afterSafe+2);

	front.discontinuity().clearAt(numFrameOld);

	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(add);

	vector3N trajA, trajB, traj;
	quaterN otrajA, otrajB, otraj;

	frontGetSignal.joint(0,otrajA, startSafe, numFrameOld);
	addGetSignal.joint(0, otrajB, 0, add.NumFrames());

	// 같은 orientation으로 돌린다.
	quater dq, dqY, dqOffset;
	dq.difference(otrajB.row(0), otrajA.row(otrajA.rows()-1));
	dq.decompose(dqY, dqOffset);
	for(int i=0; i<otrajB.rows(); i++) otrajB.row(i).leftMult(dqY);


	quaterlinstitch_decompose(otrajA, otrajB.range(0, nForwardSafe), otraj);
	//otraj.linstitch(otrajA, otrajB.range(0, nForwardSafe));
	frontSetSignal.joint(0,otraj, startSafe);
	frontSetSignal.joint(0, otrajB.range(nForwardSafe, otrajB.size()), numFrameOld+nForwardSafe-1);

	frontGetSignal.root(trajA, startSafe, numFrameOld);
	addGetSignal.root(trajB, 0, add.NumFrames());

	// 같은 위치로 이동시킨다.
	vector3 trans;
	trans=-1*trajB.row(0);	trans.y=0;
	trajB.translate(trans);	
	trajB.rotate(dqY);

	trans=trajA.row(trajA.rows()-1);	trans.y=0;
	trajB.translate(trans);	
		
	traj.linstitch(trajA, trajB.range(0, nForwardSafe));
	frontSetSignal.root(traj, startSafe);
	frontSetSignal.root(trajB.range(nForwardSafe, trajB.size()), numFrameOld+nForwardSafe-1);

	if(dynamic_cast<TwoPosture*>(&front.Pose(0)))
	{
		// stitch opponent pos
		MotionUtilTwo::getOpponentPos(front, trajA, startSafe, numFrameOld);
		MotionUtilTwo::getOpponentPos(add, trajB, 0, add.NumFrames());
		//traj.linstitch(trajA, trajB.range(0, nForwardSafe));
		traj.c0stitch(trajA, trajB.range(0, nForwardSafe));
		MotionUtilTwo::setOpponentPos(front, traj, startSafe);
		MotionUtilTwo::setOpponentPos(front, trajB.range(nForwardSafe, trajB.size()), numFrameOld+nForwardSafe-1);
	}

	for(int i=1; i<add.NumJoints(); i++)
	{
		frontGetSignal.joint(i,otrajA, startSafe, numFrameOld);
		addGetSignal.joint(i, otrajB, 0, add.NumFrames());

		otraj.linstitch(otrajA, otrajB.range(0, nForwardSafe));
		//quaterlinstitch_log(otrajA, otrajB.range(0, nForwardSafe), otraj);
		frontSetSignal.joint(i,otraj, startSafe);
		frontSetSignal.joint(i, otrajB.range(nForwardSafe, otrajB.size()), numFrameOld+nForwardSafe-1);
	}
}

void stitchTest2(const char* filename)
{
	vector3N test;
	test.setSize(40);

	for(int i=0; i<20; i++)
	{
		test[i].x=20-i;
		test[i].y=i;
		test[i].z=20-i;
	}

	for(int i=20; i<40; i++)
	{
		test[i].x=20-i+10;
		test[i].y=i+10;
		test[i].z=i;
	}

	matrixn before(40,6);
	matrixn after(40,6);
	
	before.range(0,40, 0,3).assign(test);

	vector3N testA(20), testB(20);
	vector3N test2;
	for(int i=0; i<20; i++)
	{
		testA[i]=test[i];
		testB[i]=test[20+i];
	}
	test2.linstitch(testA, testB);
	after.range(0,39, 0,3).assign(test2);
	after.row(39).range(0,3).assign(test2.row(test2.rows()-1));

	for(int i=0; i<20; i++)
	{
		test[i].x=20-cos(i/10.0);
		test[i].y=cos(i/15.0);
		test[i].z=20-i;
	}

	for(int i=20; i<40; i++)
	{
		test[i].x=20-cos(i/10.0)+10;
		test[i].y=i+10;
		test[i].z=sin(i/20.0);
	}

	before.range(0,40, 3,6).assign(test);
	
	for(int i=0; i<20; i++)
	{
		testA[i]=test[i];
		testB[i]=test[20+i];
	}
	test2.linstitch(testA, testB);
	after.range(0,39, 3,6).assign(test2);
	after.row(39).range(3,6).assign(test2.row(test2.rows()-1));

	before.op0(m0::drawSignals(sz1::format("%s_before.bmp", filename),0,0,true));
	after.op0(m0::drawSignals(sz1::format("%s_after.bmp", filename),0,0,true));
}
void stitchTest3(const m2::_op& op, const char* filename, int normalize)
{
	matrixn before;
	before.setSize(40,6);

	for(int i=0; i<20; i++)
	{
		before[i][0]=20-i;
		before[i][1]=i;
		before[i][2]=20-i;
		before[i][3]=20-cos(i/10.0);
		before[i][4]=cos(i/15.0);
		before[i][5]=20-i;
	}

	for(int i=20; i<40; i++)
	{
		before[i][0]=20-i+10;
		before[i][1]=i+10;
		before[i][2]=i;
		before[i][3]=20-cos(i/10.0)+10;
		before[i][4]=i+10;
		before[i][5]=sin(i/20.0);
	}

	if(normalize==3)
	{
		for(int i=0; i<40; i++)
		{
			before.row(i).range(0,3).normalize();
			before.row(i).range(3,6).normalize();
		}
	}

	matrixn after(40,3);
	matrixn test2;


	for(int iter=0; iter<2; iter++)
	{
		gnuPlotQueue q(sz1::format("%s_%d", filename,iter), 3);

		test2.op2(op, before.range(0,20, 0+iter*3, 3+iter*3), before.range(20,40, 0+iter*3, 3+iter*3));
		after.range(0,39).assign(test2);
		after.row(39).assign(test2.row(test2.rows()-1));

		if(normalize==3)
		{
//			q.plotScattered(after.range(0,39, 0,3), "after1");
			for(int i=0; i<40; i++)
			{
				after.row(i).normalize();
			}
		}
		q.plotScattered(before.range(0,20, 0+iter*3,3+iter*3), "before1");
		q.plotScattered(before.range(20,40, 0+iter*3,3+iter*3), "before2");
		q.plotScattered(after.range(0,39, 0,3), "after");
	}

	/*
	before.op0(m0::drawSignals(sz1::format("%s_before.bmp", filename),0,0,true));
	test2.op2(op, before.range(0,20), before.range(20,40));
	test2.op0(m0::drawSignals(sz1::format("%s_after.bmp", filename),0,0,true));

	printf("results are exported to workingDirectory/%s_before.bmp\n", filename);*/
	printf("results are exported to gnuplot/%s.dem\n", filename);

}

void stitchTestQ(const m2::_op& op, const char* filename)
{
	matrixn before;
	before.setSize(40,6);

	for(int i=0; i<20; i++)
	{
		before[i][0]=20-i;
		before[i][1]=i;
		before[i][2]=20-i;
		before[i][3]=20-cos(i/10.0);
		before[i][4]=cos(i/15.0);
		before[i][5]=20-i;
		before.row(i).range(0,3).normalize();
		before.row(i).range(3,6).normalize();
	}

	for(int i=20; i<40; i++)
	{
		before[i][0]=20-i+10;
		before[i][1]=i+10;
		before[i][2]=i;
		before[i][3]=20-cos(i/10.0)+10;
		before[i][4]=i+10;
		before[i][5]=sin(i/20.0);
		before.row(i).range(0,3).normalize();
		before.row(i).range(3,6).normalize();
	}

	matrixn after(40,3);
	matrixn test2;

	for(int iter=0; iter<2; iter++)
	{
		test2.op2(op, before.range(0,20, 0+iter*3, 3+iter*3), before.range(20,40, 0+iter*3, 3+iter*3));
		after.range(0,39).assign(test2);
		after.row(39).assign(test2.row(test2.rows()-1));

		TString fn;
		fn.format("%s_%d", filename, iter);
		TStrings fns;
		fns.setStrings(3, (fn+"_1").ptr(), (fn+"_2").ptr(), (fn+"_s").ptr());
		gnuPlot::plotScattered(fns[0], before.range(0,20, 0+iter*3,3+iter*3), fns[0]);
		gnuPlot::plotScattered(fns[1], before.range(20,40, 0+iter*3,3+iter*3), fns[1]);
		gnuPlot::plotScattered(fns[2], after.range(0,39, 0,3), fns[2]);
		gnuPlot::mergedPlotScattered(fns, 3, fn);
	}

	before.op0(m0::drawSignals(sz1::format("%s_before.bmp", filename),0,0,true));
	after.op0(m0::drawSignals(sz1::format("%s_after.bmp", filename),0,0,true));
}

///////////////////
//testing code backup

void qstitchTest2(const char* filename)
{
	quaterN test;
	quaterN testA;
	quaterN testB;
	test.setSize(40);
	testA.setSize(20);
	testB.setSize(20);

	for(int i=0; i<20; i++)
	{
		testA[i].setRotation(vector3(0,1,0), (20-i)/20.f);
		testB[i].setRotation(vector3(0,1,0), (i+10)/20.f);
	}

	matrixn temp(40,4);
	temp.range(0,20).assign(testA);
	temp.range(20,40).assign(testB);
	temp.op0(m0::drawSignals("qtemp1.bmp",-1.f,1.f,true));

	//test.linstitch(testA, testB);
	//test.c0stitch(testA, testB);
	test.c1stitch(testA, testB);
	
	temp.assign(test);
	temp.op0(m0::drawSignals("qtemp2.bmp",-1.f,1.f,true));
}


void MotionUtil::exportBVH(Motion const& mot, const char* filename, int start, int end)
{

	const bool bFullDOF=true;	// packBVH에서도 똑같이 세팅할 것.

	if(start<0) start=0;
	if(end>mot.NumFrames()) end=mot.NumFrames();


	FILE* file;
	file=fopen(filename, "w");
	mot.skeleton().getBoneByRotJointIndex(0).packBVH(file, 0, &mot.skeleton());

	fprintf(file, "\nMOTION\nFrames: %d\n", end-start);
	fprintf(file, "Frame Time: %f\n", mot.FrameTime());

	Bone* pBone;
	
	m_real eulerAngles[3];

	TString tc, channels;
	for(int iframe=start; iframe<end; iframe++)
	{
		for(int ijoint=0; ijoint<mot.NumJoints(); ijoint++)
		{
			pBone=&mot.skeleton().getBoneByRotJointIndex(ijoint);


			tc=pBone->getTranslationalChannels();
			if(tc.length())
			{
				ASSERT(ijoint==0);
				fprintf(file, "%f %f %f ", mot.Pose(iframe).m_aTranslations[0].x, mot.Pose(iframe).m_aTranslations[0].y, mot.Pose(iframe).m_aTranslations[0].z);
			}

			channels=pBone->getRotationalChannels();
			mot.Pose(iframe).m_aRotations[ijoint].getRotation(channels, eulerAngles);
			
			if(bFullDOF)
			{
				for(int c=0; c<3; c++)
					fprintf(file, "%f ", TO_DEGREE(eulerAngles[c]));
			}
			else
			{
				for(int c=0; c<channels.length(); c++)
					fprintf(file, "%f ", TO_DEGREE(eulerAngles[c]));
			}

			
		}

		fprintf(file, "\n");
	}

	fclose(file);
}

void MotionUtil::RetargetOnline2D::adjustToPath(Path2D & path, int thr, m_real angleThr, m_real distThr)
{
	mCurve.setSize(mTarget.NumFrames()-mStart, 2);// first allocate memory
	//first retarget orientations.	
	{
		mCurve.setSize(mTarget.NumFrames()-mStart,1);

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mTarget.Pose(i).decomposeRot();
		}
		mCurve[0][0]=mTarget.Pose(mStart).m_rotAxis_y.rotationAngleAboutAxis(vector3(0,1,0));

		quater q;
		for(int i=mStart+1; i<mTarget.NumFrames(); i++)
		{
			q.difference(mTarget.Pose(i-1).m_rotAxis_y, mTarget.Pose(i).m_rotAxis_y);
			mCurve[i-mStart][0]=mCurve[i-mStart-1][0]+q.rotationAngleAboutAxis(vector3(0,1,0));
		}

		matrixn mCon;
		mCon.setSize(mTarget.NumFrames()-mStart, 1);
		mCon.setAllValue(FLT_MAX);

		int pathEnd=path.start()+path.size();
		mCon[path.start()-1-mStart][0]=path.ori(path.start()-1).rotationAngleAboutAxis(vector3(0,1,0));
		for(int i=path.start(); i<path.start()+path.size(); i++)
			mCon[i-mStart][0]=
			mCon[i-1-mStart][0]+path.dq(i).rotationAngleAboutAxis(vector3(0,1,0));
		
		//printf("mStart %d pathstart %d pathend %d \n curveend %d", mStart, pathStart, pathEnd, mTarget.NumFrames());
		
		int numCon=(pathEnd-path.start())/(int)thr+1;  // eg if thr==10 and 10<=(pathEnd-start)<20 , then numCon=2
		if(ABS(mCurve[pathEnd-1-mStart]-mCon[pathEnd-1-mStart])<angleThr)
			numCon=1;
		intvectorn time(numCon);
		matrixn con(numCon, 1);

		
		for(int i=0; i<numCon; i++)
		{
			// i=-1일때 pathStart, i=numCon-1일때 pathEnd-1가 나오면 됨.
			int t= ROUND((m_real(i+1)/m_real(numCon))*m_real(pathEnd-1-path.start()))+path.start();
			time[i]=t-mStart;
			con[i][0]=mCon[t-mStart][0];
		}
		
		mCurve.op0(m0::adjustOnlineMultiCon(time, con));

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mTarget.Pose(i).m_rotAxis_y.setRotation(vector3(0,1,0), mCurve[i-mStart][0]);		
			mTarget.Pose(i).m_aRotations[0].mult(mTarget.Pose(i).m_rotAxis_y, mTarget.Pose(i).m_offset_q);
		}

		mTarget._reconstructPosByDifference(mStart-1);
	}

	//then retarget positions
	{
		mCurve.setSize(mTarget.NumFrames()-mStart, 2);

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mCurve[i-mStart][0]=mTarget.Pose(i).m_aTranslations[0].x;
			mCurve[i-mStart][1]=mTarget.Pose(i).m_aTranslations[0].z;
		}

		matrixn mCon;
		mCon.setSize(mTarget.NumFrames()-mStart, 2);
		mCon.setAllValue(FLT_MAX);
		int pathStart=path.start()-1;	// -1부터 얻어올수 있다.
		int pathEnd=pathStart+path.size()+1;

		for(int i=pathStart; i<pathEnd; i++)
		{
			mCon[i-mStart][0]=path.pos(i).x;
			mCon[i-mStart][1]=path.pos(i).z;
		}

		//printf("mStart %d pathstart %d pathend %d \n curveend %d", mStart, pathStart, pathEnd, mTarget.NumFrames());

		int numCon=(pathEnd-pathStart)/(int)thr+1;  // eg if thr==10 and 10<=(pathEnd-start)<20 , then numCon=2
		if(mCurve.row(pathEnd-1-mStart).distance(mCon.row(pathEnd-1-mStart))<distThr)
			numCon=1;
		intvectorn time(numCon);
		matrixn con(numCon, 2);

		for(int i=0; i<numCon; i++)
		{
			// i=-1일때 pathStart, i=numCon-1일때 pathEnd-1가 나오면 됨.
			int t= ROUND((m_real(i+1)/m_real(numCon))*m_real(pathEnd-1-pathStart))+pathStart;
			time[i]=t-mStart;
			con[i][0]=mCon[t-mStart][0];
			con[i][1]=mCon[t-mStart][1];
		}

		mCurve.op0(m0::adjustOnlineMultiCon(time, con));

		for(int i=mStart; i<mTarget.NumFrames(); i++)
		{
			mTarget.Pose(i).m_aTranslations[0].x=mCurve[i-mStart][0];
			mTarget.Pose(i).m_aTranslations[0].z=mCurve[i-mStart][1];
		}
	}
}
