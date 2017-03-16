#include "stdafx.h"
#include "motion.h"
#include "motionloader.h"
#include "Path2D.h"
#include "../baselib/math/operator.h"

namespace sop
{
	// x가 0에서는 기울기가 0이고, x>start일때는 기울기가 1이되는 부드러운 함수. x>=0에서 정의됨.
	m_real responseFunction(m_real x, m_real start)
	{
		// 앞부분에 y=ax^2를 잘라서 사용함.
		// 2a*start=1.0
		m_real a=0.5/start;

		if(x<start)
		{
			return a*SQR(x);
		}		
		return x-start+a*SQR(start);
	}

	// x가 b와 c사이에서는 값이 0가나오고, a보다 작을때나, d보다 클때는 기울기가 1이된다.
	m_real offsetFunction(m_real x, m_real a, m_real b, m_real c, m_real d)
	{
		if(x<b)
			//b-x>start 즉,  b-start>x
			return responseFunction(b-x, b-a)*-1;
		else if(x>c)
			return responseFunction(x-c, d-c);
		else
			return 0;
	}
}

// update dq(i) based on ori(i-1) and ori(i)
void Path2D::updateDQ(int i)
{
	quater &dq_i=dq(i);
	quater inv_q;
	inv_q.inverse(ori(i-1));
	dq_i.mult(ori(i), inv_q);
	dq_i.align(quater(1,0,0,0));	// m_dq should be small.
}

Path2D::Path2D(Motion const& mot, int start, int end)
{
	mStart=start;
	mPos.setSize(end-start+1);
	mOri.setSize(end-start+1);	// y component
	mDV.setSize(end-start);
	mDQ.setSize(end-start);

	mot.Pose(start-1).decomposeRot();

	mPos[0]=mot.Pose(start-1).m_aTranslations[0];
	mPos[0].y=0;
	mOri[0]=mot.Pose(start-1).m_rotAxis_y;

	for(int i=start; i<end; i++)
	{
		vector3& dv_i=mDV[i-start];
		quater& dq_i=mDQ[i-start];
		pos(i)=mot.Pose(i).m_aTranslations[0];
		pos(i).y=0;

		///////////////////////////////////////////////////////////////////////////////
		//  calculation m_dv and offset_y
		///////////////////////////////////////////////////////////////////////////////
		dv_i = (mot.Pose(i).m_aTranslations[0] - mot.Pose(i-1).m_aTranslations[0]);
		dv_i.y = 0;
	
		quater inv_q;
		inv_q.inverse(mot.Pose(i-1).m_rotAxis_y);
		dv_i.rotate(inv_q,dv_i);

		///////////////////////////////////////////////////////////////////////////////
		//  calculation rotAxis_y	Pose(i).m_aRotations[0] = m_rotAxis_y*m_offset_q 
		//							Pose(i).m_dq * Pose(i-1).m_rotAxis_y = Pose(i).m_rotAxis_y
		//			  		  thus, Pose(i).m_dq = Pose(i).m_rotAxis_y * (Pose(i-1).m_rotAxis_y)^(-1)
		///////////////////////////////////////////////////////////////////////////////
		
		mot.Pose(i).decomposeRot();
		ori(i)=mot.Pose(i).m_rotAxis_y;

		dq_i.mult(mot.Pose(i).m_rotAxis_y, inv_q);
		dq_i.align(quater(1,0,0,0));	// m_dq should be small.
	}
}

void Path2D::updateDV(int i)
{
	vector3& dv_i=dv(i);
	///////////////////////////////////////////////////////////////////////////////
	//  calculation dv
	///////////////////////////////////////////////////////////////////////////////
	dv_i.sub(pos(i), pos(i-1));
	dv_i.y = 0;

	quater inv_q;
	inv_q.inverse(ori(i-1));
	dv_i.rotate(inv_q,dv_i);
}

Path2D::~Path2D(void)
{
}

void Path2D::updateOri(int i)
{
	ori(i).mult(dq(i), ori(i-1));
}

void Path2D::updatePos(int i)
{
	vector3 dvg;
	dvg.rotate(ori(i-1), dv(i));
	pos(i).add(pos(i-1), dvg);
}

void Path2D::updatePath(int i)
{
	updateOri(i);
	updatePos(i);
}

void Path2D::setMotion(Motion & mot, int start)
{
	for(int i=0; i<size(); i++)
	{
		mot.Pose(start+i).m_aTranslations[0].x=pos(start+i).x;
		mot.Pose(start+i).m_aTranslations[0].z=pos(start+i).z;
		mot.Pose(start+i).m_aRotations[0].mult(ori(start+i), mot.Pose(start+i).m_offset_q);
	}
}

void Path2D::setMotionDelta(Motion & mot, int start)
{
	for(int i=0; i<size(); i++)
	{
		mot.Pose(start+i).m_dv=dv(start+i);
		mot.Pose(start+i).m_dq=dq(start+i);
	}
}

TwoPath2D::TwoPath2D(Motion const& mot1, Motion const& mot2, int start, int end)
:mPath1(mot1, start,end),
mPath2(mot2, start, end)
{
	mStart=start;
	mTarget[0].setSize(end-start);
	mTarget[1].setSize(end-start);

	for(int i=start; i<end; i++)
	{
		mTarget[0][i-start]=mot1.pose2(i).m_oppenentPos;
		mTarget[0][i-start].y=0;
		mTarget[1][i-start]=mot2.pose2(i).m_oppenentPos;
		mTarget[1][i-start].y=0;
	}
}

vector3 TwoPath2D::calcTargetDir(int ePath, int i, bool bGlobal) const
{
	int ePairPath=(ePath+1)%2;
	vector3 targetDirection;
	targetDirection.difference(path(ePath).pos(i), path(ePairPath).pos(i));

	if(!bGlobal)
		targetDirection.rotate(path(ePath).ori(i).inverse());

	return targetDirection;
}

m_real TwoPath2D::calcRetargetAngle(int ePath, int correctTime) const
{
	int ePairPath=(ePath+1)%2;

	quater rotThis, rotOther;
	vector3 actualTargetDir;

	actualTargetDir=calcTargetDir(ePath, correctTime);
	rotThis.axisToAxis(mTarget[ePath][correctTime-mStart], actualTargetDir);
	m_real retargetAngle=rotThis.rotationAngleAboutAxis(vector3(0,1,0));

	//m_real importance=sop::clampMap(ABS(retargetAngle), TO_RADIAN(5), TO_RADIAN(15), 0.0, 1.0);
	// smooth
	//retargetAngle*=0.5;
	//retargetAngle*=0.25*importance;
	//return retargetAngle;
	return sop::offsetFunction(retargetAngle, TO_RADIAN(25)*-1, 0, 0, TO_RADIAN(25));	
}

m_real TwoPath2D::calcRetargetAngle2(int ePath, int i) const
{
	m_real smoothedAngle1;
	m_real angle1i=calcRetargetAngle(ePath, i);
	m_real angle1=calcRetargetAngle(ePath, i+1);
	//smoothedAngle1=smoothedAngle1*0.5+angle1*0.5;

	if(angle1i*angle1<0)
	{
		// do nothing.
		smoothedAngle1=0.0;
	}
	else if(ABS(angle1)<ABS(angle1i))
	{
		// 각도가 줄어드는 중이면.
		// do nothing.
		smoothedAngle1=0.0;			
	}
	else
	{
		smoothedAngle1=angle1i;
	}

	return smoothedAngle1;
}

// weight가 0 일때 target 0을 사용.
m_real TwoPath2D::calcRetargetDistance(int correctTime, m_real weight) const
{
	m_real actualDist=calcTargetDir(0, correctTime, true).length();
	m_real dist1=mTarget[0][correctTime-mStart].length();
	m_real dist2=mTarget[1][correctTime-mStart].length();

	return (dist1*(1.0-weight)+dist2*weight)-actualDist;
}



m_real responseFunction(m_real actualDist, m_real dist1, m_real dist2)
{
	m_real retargetDist;

	m_real minDist=MIN(dist1, dist2);
	m_real maxDist=MAX(dist1, dist2);

	return -1*sop::offsetFunction(actualDist, 
		minDist*0.5,
		sop::interpolate(0.3, minDist, maxDist),
		sop::interpolate(0.7, minDist, maxDist),
		maxDist*1.5);
}

#include "../math/gnuplot.h"
void testResponseFunction()
{
	matrixn test(100, 2);
	test.column(0).linspace(0, 5, 100);

	for(int i=0; i<100; i++)
		test[i][1]=responseFunction(test[i][0], 2, 3);

	gnuPlot::plotScattered("responseFunction", test);
}

m_real TwoPath2D::calcRetargetDistance(int correctTime) const
{
	m_real actualDist=calcTargetDir(0, correctTime, true).length();
	m_real dist1=mTarget[0][correctTime-mStart].length();
	m_real dist2=mTarget[1][correctTime-mStart].length();

	return responseFunction(actualDist, dist1, dist2);
}

m_real TwoPath2D::calcRetargetDistance2(int i) const
{
	m_real dist;
	m_real disti=calcRetargetDistance(i);
	m_real distii=calcRetargetDistance(i+1);

	if(disti*distii<0)
	{
		// do nothing.
		dist=0.0;
	}
	else if(ABS(distii)<ABS(disti))
	{
		// 거리차가 줄어드는 중이면.
		// do nothing.
		dist=0.0;			
	}
	else
	{
		dist=disti;
	}
	return dist;
}
void TwoPath2D::retarget(m_real weight, int end )
{
	if(end>mStart+path(0).size()) end=mStart+path(0).size();

	int start=mStart;
	m_real smoothedAngle1=0.0;
	m_real smoothedAngle2=0.0;
	m_real smoothedDeltaDist=0.0;
	for(int i=start; i<end; i++)
	{
		path(0).updatePath(i);
		path(1).updatePath(i);									
		//path(0).updatePath(i+1);
		//path(1).updatePath(i+1);									


		m_real angle1=calcRetargetAngle(0, i);
		m_real angle2=calcRetargetAngle(1, i);

		//smoothedAngle1=smoothedAngle1*0.5+angle1*0.5;
		smoothedAngle1=angle1;
		path(0).dq(i).leftMult(quater(smoothedAngle1, vector3(0,1,0)));
		
		//smoothedAngle2=smoothedAngle2*0.5+angle2*0.5;
		smoothedAngle2=angle2;
		path(1).dq(i).leftMult(quater(smoothedAngle2, vector3(0,1,0)));

		path(0).updatePath(i);
		path(1).updatePath(i);
		//path(0).updatePath(i+1);
		//path(1).updatePath(i+1);

		m_real deltaDist=calcRetargetDistance(i);
		//smoothedDeltaDist=smoothedDeltaDist*0.5+deltaDist*0.5;
		smoothedDeltaDist=deltaDist;
		vector3 delta;
		delta.difference(path(0).pos(i), path(1).pos(i));
		delta.normalize();
		delta*=smoothedDeltaDist;
		
		path(0).pos(i)-=delta*(1.0-weight);
		path(1).pos(i)+=delta*weight;
		path(0).updateDV(i);
		path(1).updateDV(i);
	}
}

