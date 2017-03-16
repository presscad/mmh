#include "stdafx.h"
#include "motion.h"
#include "motionloader.h"
#include "motionutil.h"
#include "motionutilskeleton.h"

#include "motionAnalysis.h"
#include "../math/nr/nr.h"
#include "../math/intervals.h"
#include "../math/BSpline.h"
#include "../math/GnuPlot.h"
#include "../math/conversion.h"
#include "../math/operatorsignal.h"
#include "../math/operator.h"
#include "../math/numericalspline.h"
#define INVALID_PATH FLT_MAX
#define CHECK_INVALID FLT_MAX/2.0

namespace MotionUtil
{
	void insertBalanceJoint(MotionLoader& skel)
	{
		Motion& mot=skel.m_cPostureIP;
		matrixn aRoot;
		matrixn aRootOri;
		//MotionAnalysis::LocoAnalysis la(mot, MotionAnalysis::LocoAnalysis::ROOT_PATH);
		MotionAnalysis::COMAnalysis la(mot, 1.0);

		aRoot.setSize(mot.NumFrames(), 3);
		aRootOri.setSize(mot.NumFrames(), 4);

		for(int iframe=0; iframe<mot.NumFrames(); iframe++)
		{
			if(la.isValid(iframe))
			{
				vector3 acc;
				acc.x=la.mCAcc[iframe][0];
				acc.z=la.mCAcc[iframe][1];
				acc.y=0;

				acc/=SQR(mot.FrameTime());

				m_real tanTheta=acc.length()/980;	// amount of leaning
				m_real theta=atan(tanTheta);

				m_real r=175/2.0;
				m_real h=r*cos(theta);

				vector3 dir;
				dir.normalize(acc);

				vector3 start=la.mPath.pos(iframe)-dir*r*sin(theta);
				vector3 end=la.mPath.pos(iframe)+dir*r*sin(theta)+2*h*vector3(0,1,0);

				vector3 axis;
				axis.cross(vector3(0,1,0),dir);
				axis.normalize();
				quater q;
				q.setRotation(axis, theta);
				aRoot.row(iframe).assign((start+end)/2.0);
				aRootOri.row(iframe).assign(q*la.mPath.ori(iframe));
			}			
			else
			{
				mot.setConstraint(iframe, POSE_IS_INVALID, true);
				aRoot.row(iframe).assign(vector3(0,0,0));
				aRootOri.row(iframe).assign(quater(1,0,0,0));
			}
		}

		insertRootJoint(skel, aRoot, aRootOri, "COM");
	}
}

namespace MotionAnalysis
{
void pushBackNoDup(vectorn& a, m_real b)
{
	if(b>=0.0 && (a.size()==0 || a[a.size()-1]!=b))
		a.pushBack(b);
}

////////////////////////////////////
COMAnalysis::COMAnalysis(Motion const& src, m_real kernelSize)
:mPath(src.NumFrames(), src.NumFrames())
{
	for(int i=0; i<src.NumFrames(); i++)
		src.Pose(i).decomposeRot();

	MotionUtil::PhysicalHuman ph(src);
	ph.COM(mCOM);

	intIntervals paths;

	MotionUtil::SegmentFinder seg(src);

	paths.setSize(seg.numSegment());
	for(int i=0; i<seg.numSegment(); i++)
	{
		paths.start(i)=seg.startFrame(i);
		paths.end(i)=seg.endFrame(i);

		mCOM.range(seg.startFrame(i), seg.endFrame(i)).op0(m0::useUnary(m1::filter(src.KernelSize(kernelSize))));
	}

	mPath1.setSize(src.NumFrames(), 2);
	mPath1.setAllValue(INVALID_PATH);
	mPath2.setSize(src.NumFrames(), 2);
	mPath2.setAllValue(INVALID_PATH);
	mCPath.setSize(src.NumFrames(), 2);
	mCPath.setAllValue(INVALID_PATH);	
	
	mVel1=mCPath;	// analytic velocity 
	mAcc1=mCPath;	// analytic acceleration
	mVel2=mCPath;	// analytic velocity 
	mAcc2=mCPath;	// analytic acceleration
	mCVel=mCPath;	// analytic velocity 
	mCAcc=mCPath;	// analytic acceleration

	mOri1.setSize(src.NumFrames());
	mOri1.setAllValue(INVALID_PATH);
	mCAngAcc=mAngAcc2=mAngAcc1=mCAngVel=mAngVel2=mAngVel1=mCOri=mOri2=mOri1;

	for(int path=0; path<paths.size(); path++)
	{
		vectorn keytime1;
		vectorn keytime2;

		keytime1.op0(v0::colon(paths.start(path), paths.end(path), 2));
		keytime2.op0(v0::colon(paths.start(path)+1, paths.end(path), 2));

		_calcPath(src, keytime1, mPath1, "N", "N");
		_calcPath(src, keytime2, mPath2, "N", "N");

		_calcOri(src, keytime1, mOri1, "N", "N");
		_calcOri(src, keytime2, mOri2, "N", "N");
	}

	for(int i=0; i<mCPath.rows(); i++)
	{
		if(mPath1[i][0]!=INVALID_PATH && mPath2[i][0]!=INVALID_PATH)
		{
			mCPath.row(i).each2(s2::AVG, mPath1.row(i), mPath2.row(i));
			mCVel.row(i).each2(s2::AVG, mVel1.row(i), mVel2.row(i));
			mCAcc.row(i).each2(s2::AVG, mAcc1.row(i), mAcc2.row(i));
		}

		if(mOri1[i]!=INVALID_PATH && mOri2[i]!=INVALID_PATH )
		{
			mCOri[i]=(mOri1[i]+mOri2[i])/2.0;
			mCAngVel[i]=(mAngVel1[i]+mAngVel2[i])/2.0;
			mCAngAcc[i]=(mAngAcc1[i]+mAngAcc2[i])/2.0;
		}
	}
	// calc onlinePath2D
	
	for(int path=0; path<paths.size(); path++)
	{
		int start=paths.start(path);
		int end=paths.end(path)-1;

#ifdef ORIENTATION_FROM_VELOCITY
		OnlinePath2D opath(mCPath.range(start, end+1), mCVel.range(start, end+1));
#else
		OnlinePath2D opath(mCPath.range(start, end+1), mCOri.range(start, end+1));
#endif
		for(int i=start; i<end+1; i++)
		{
			mPath.pos(i)=opath.pos(i-start);
			mPath.ori(i)=opath.ori(i-start);
			mPath.dv(i)=opath.dv(i-start);
			mPath.dq(i)=opath.dq(i-start);
		}
	}	
}

void COMAnalysis::_calcOri(Motion const& src, vectorn const& lkeytime, vectorn& ori, TString const& startType, TString const& endType)
{
	vectorn controlPoints;
	controlPoints.setSize(lkeytime.size());

	int startTime=ROUND(lkeytime[0]);
	int endTime=ROUND(lkeytime[lkeytime.size()-1]);

	vectorn _angles(endTime-startTime+1);
	vectornView angles=vecViewOffset(_angles, startTime);

	for(int i=startTime; i<=endTime; i++)
	{
		quater rotY, offset;
		src.Pose(i).m_aRotations[0].decompose(rotY, offset);
		angles[i]=rotY.rotationAngleAboutAxis(vector3(0,1,0));
	}
	
	_angles.op0(v0::alignAngles(_angles[0]));
		
	for(int i=0; i<lkeytime.size(); i++)
	{
		int t=ROUND(lkeytime[i]);

		controlPoints[i]=angles[t];
	}

	// zero initial velocity
	NonuniformSpline::boundaryCondition bc0=NonuniformSpline::zeroVel();
	NonuniformSpline::boundaryCondition bcn=NonuniformSpline::zeroVel();

	if(startType[0]!='S')
		bc0=NonuniformSpline::zeroAcc();
	if(endType[endType.length()-1]!='S')
		bcn=NonuniformSpline::zeroAcc();

	NonuniformSpline spline(lkeytime, controlPoints.column(), bc0, bcn);

	int minTime=ROUND(lkeytime[0]);
	int maxTime=ROUND(lkeytime[lkeytime.size()-1]);

	vectorn time;
	time.colon(minTime, 1.0, maxTime-minTime+1);
	spline.getCurve(time, ori.range(minTime, maxTime+1).column());

	// mark as invalid the start and end segment if they are from ZERO_ACC condition.
	if(bc0.mType==NonuniformSpline::boundaryCondition::ZERO_ACC)
	{
		for(int i=lkeytime[0]; i<lkeytime[1]-1; i++)
			ori[i]=INVALID_PATH;
	}

	if(bcn.mType==NonuniformSpline::boundaryCondition::ZERO_ACC)
	{
		for(int i=lkeytime[lkeytime.size()-2]+1; i<=maxTime; i++)
			ori[i]=INVALID_PATH;
	}

	if(&ori==&mOri1)
	{
		spline.getFirstDeriv(time, mAngVel1.range(minTime, maxTime+1).column());
		spline.getSecondDeriv(time, mAngAcc1.range(minTime, maxTime+1).column());
	}
	else
	{
		spline.getFirstDeriv(time, mAngVel2.range(minTime, maxTime+1).column());
		spline.getSecondDeriv(time, mAngAcc2.range(minTime, maxTime+1).column());
	}
}
void COMAnalysis::_calcPath(Motion const& src, vectorn const& lkeytime, matrixn& path, TString const& startType, TString const& endType)
{
	matrixn controlPoints;
	controlPoints.setSize(lkeytime.size(), 2);
	for(int i=0; i<lkeytime.size(); i++)
	{
		int t=ROUND(lkeytime[i]);

		controlPoints[i][0]=mCOM.row3(t).x;
		controlPoints[i][1]=mCOM.row3(t).z;
	}
	
	/*
	// give very slow initial velocity
	vector3 front0=src.Pose(ROUND(lkeytime[0])).front()*0.1;
	vector3 frontn=src.Pose(ROUND(lkeytime[lkeytime.size()-1])).front()*0.1;

	NonuniformSpline::boundaryCondition bc0=NonuniformSpline::velocity(vectorn(2, front0.x, front0.z));
	NonuniformSpline::boundaryCondition bcn=NonuniformSpline::velocity(vectorn(2, frontn.x, frontn.z));*/

	// zero initial velocity
	NonuniformSpline::boundaryCondition bc0=NonuniformSpline::zeroVel();
	NonuniformSpline::boundaryCondition bcn=NonuniformSpline::zeroVel();

	if(startType[0]!='S')
		bc0=NonuniformSpline::zeroAcc();
	if(endType[endType.length()-1]!='S')
		bcn=NonuniformSpline::zeroAcc();

	NonuniformSpline spline(lkeytime, controlPoints, bc0, bcn);

	int minTime=ROUND(lkeytime[0]);
	int maxTime=ROUND(lkeytime[lkeytime.size()-1]);

	vectorn time;
	time.colon(minTime, 1.0, maxTime-minTime+1);
	spline.getCurve(time, path.range(minTime, maxTime+1));

	// mark as invalid the start and end segment if they are from ZERO_ACC condition.
	if(bc0.mType==NonuniformSpline::boundaryCondition::ZERO_ACC)
	{
		for(int i=lkeytime[0]; i<lkeytime[1]-1; i++)
			path[i][0]=INVALID_PATH;
	}

	if(bcn.mType==NonuniformSpline::boundaryCondition::ZERO_ACC)
	{
		for(int i=lkeytime[lkeytime.size()-2]+1; i<=maxTime; i++)
			path[i][0]=INVALID_PATH;
	}

	if(&path==&mPath1)
	{
		spline.getFirstDeriv(time, mVel1.range(minTime, maxTime+1));
		spline.getSecondDeriv(time, mAcc1.range(minTime, maxTime+1));
	}
	else
	{
		spline.getFirstDeriv(time, mVel2.range(minTime, maxTime+1));
		spline.getSecondDeriv(time, mAcc2.range(minTime, maxTime+1));
	}
}



bool COMAnalysis::isValid(int iframe)
{
	if(mCPath[iframe][0]> CHECK_INVALID )
		return false;
	return true;
}

bool COMAnalysis::isValid(int start, int end)
{
	if(end>=mCPath.rows()) return false;

	for(int i=start; i<=end; i++)
		if(!isValid(i)) return false;

	return true;
}

void COMAnalysis::_calcValidInterval(matrixn const& path, int iframe, int length, int& start, int& end)
{
	start=MAX(0, iframe-length);
	end=MIN(iframe+length, path.rows());

	if(path[iframe][0]>CHECK_INVALID)
	{
		start=end=iframe;		
		return;
	}

	// shrink path range
	for(int i=iframe; i<end; i++)
	{
		if(path[i][0]>CHECK_INVALID)
		{
			end=i;
			break;
		}
	}

	for(int i=iframe; i>=start; i--)
	{
		if(path[i][0]>CHECK_INVALID)
		{
			start=i+1;
			break;
		}
	}
}
}