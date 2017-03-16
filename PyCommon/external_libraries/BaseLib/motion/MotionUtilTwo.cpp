#include "stdafx.h"
#include "motion.h"
#include "motionloader.h"
#include "../image/imageprocessor.h"
#include ".\motionutiltwo.h"
#include "../baselib/math/operator.h"
#include "../math/OperatorSignal.h"
#include "../math/conversion.h"
void MotionUtilTwo::getOpponentPos(Motion const& m_Motion, vector3N& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start);
	for(int i=start; i<end; i++)
		out[i-start]=m_Motion.pose2(i).m_oppenentPos;
}

void MotionUtilTwo::setOpponentPos(Motion& m_Motion, const vector3N& in, int start)
{
	for(int i=0; i<in.rows(); i++)
		m_Motion.pose2(i+start).m_oppenentPos=in[i];
}


void MotionUtilTwo::getOpponentPos(Motion const& m_Motion, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start, 2);

	quater q;
	vector3 v;
	vector3 front(0,0,1);
	for(int i=start; i<end; i++)
	{
		out[i-start][0]=m_Motion.pose2(i).m_oppenentPos.length();
		v.divide(m_Motion.pose2(i).m_oppenentPos, out[i-start][0]);
		q.axisToAxis(front, v);
		out[i-start][1]=q.rotationAngleAboutAxis(vector3(0,1,0));
	}

	out.column(1).op0(v0::alignAngles(out[0][1]));
}

void MotionUtilTwo::setOpponentPos(Motion& m_Motion, const matrixn& in, int start)
{
	m_real len;
	quater q;
	vector3 v;
	vector3 front(0,0,1);
	for(int i=0; i<in.rows(); i++)
	{
		v.rotate(quater(in[i][1], vector3(0,1,0)), front);
		m_Motion.pose2(i+start).m_oppenentPos.mult(v, in[i][0]);
	}
}
