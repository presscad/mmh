#include "stdafx.h"
#include "motion/motion.h"
#include ".\concat.h"
#include ".\footprint.h"
#include "../baselib/motion/motionutil.h"

using namespace MotionUtil;

void ReconstructConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.NumFrames();
	front.Concat(&add);

	if(numFrameOld!=0)
		front.ReconstructDataByDifference(numFrameOld-1);
}

void ReconstructAdjustConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.NumFrames();
	front.Concat(&add);

	quater q_y, q_z;
	
	q_y.setRotation(vector3(0,1,0), adjustAngleYFirst);
	front.Pose(numFrameOld).m_dq.leftMult(q_y);

	q_y.setRotation(vector3(0,1,0), adjustAngleY/(float)add.NumFrames());
	q_z.setRotation(vector3(0,0,1), adjustAngleZ);

	for(int i=numFrameOld; i<front.NumFrames(); i++)
	{
		front.Pose(i).m_dq.leftMult(q_y);
		front.Pose(i).m_offset_q.leftMult(q_z);
	}

	if(numFrameOld!=0)
		front.ReconstructDataByDifference(numFrameOld-1);
}

void ReconstructAdjustConcatTwoWay::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.NumFrames();
	front.Concat(&add);
	quater q_y;
	if(safe==0)
	{
		q_y.setRotation(vector3(0,1,0), adjustAngleY);
		front.Pose(numFrameOld).m_dq.leftMult(q_y);
		if(numFrameOld!=0)
			front.ReconstructDataByDifference(numFrameOld-1);
	}
	else
	{
		int kernelsize=safe*2+1;
		vectorn kernel;
		Filter::GetGaussFilter(kernelsize, kernel);
		
		for(int i=-1*safe; i<=safe; i++)
		{
			q_y.setRotation(vector3(0,1,0), adjustAngleY*kernel[i+safe]);
			front.Pose(i+numFrameOld).m_dq.leftMult(q_y);
		}
		front.ReconstructDataByDifference(numFrameOld-safe-1);
	}

}

void RootTrajectoryConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.NumFrames();

	front.Concat(&add);

	if(numFrameOld!=0)
	{
		vector3 dvFront, dvAdd;
		dvFront.difference(front.Pose(numFrameOld-2).m_aTranslations[0], front.Pose(numFrameOld-1).m_aTranslations[0]);
		dvAdd.difference(front.Pose(numFrameOld).m_aTranslations[0], front.Pose(numFrameOld+1).m_aTranslations[0]);

		// 너무 느리면 root orientation을 사용하도록 해야할 듯.

        dvFront.y=0;
		dvAdd.y=0;

		quater q;
		//q.setAxisRotation(vector3(0,1,0), dvAdd, dvFront);		
		q.axisToAxis(dvAdd, dvFront);

		MotionUtil::rotate(front, q, numFrameOld, front.NumFrames());

		// translate
		vector3 translate;
		translate.difference(front.Pose(numFrameOld).m_aTranslations[0], front.Pose(numFrameOld-1).m_aTranslations[0]+dvFront);
		translate.y=0;

		MotionUtil::translate(front, translate, numFrameOld, front.NumFrames());
	}	
}

void RootOrientationConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.NumFrames();

	front.Concat(&add);

	if(numFrameOld!=0)
	{/*
		quater dqFront, dqAdd;
		dqFront.difference(front.Pose(numFrameOld-2).m_aRotations[0], front.Pose(numFrameOld-1).m_aRotations[0]);
		dqAdd.difference(front.Pose(numFrameOld).m_aRotations[0], front.Pose(numFrameOld+1).m_aRotations[0]);

		quater dq_mid;
		dq_mid.safeSlerp(dqFront, dqAdd, 0.5);

		quater target;
		target.mult(dq_mid, front.Pose(numFrameOld-1).m_aRotations[0]);

		quater dif, difY, offset;
		dif.difference(front.Pose(numFrameOld).m_aRotations[0], target);
		dif.decompose(difY, offset);

		front.rotate(numFrameOld, front.NumFrames(), difY);*/

		quater roty_2, roty_1, roty, roty1, offset;
		front.Pose(numFrameOld-2).m_aRotations[0].decompose(roty_2, offset);
		front.Pose(numFrameOld-1).m_aRotations[0].decompose(roty_1, offset);
		front.Pose(numFrameOld).m_aRotations[0].decompose(roty, offset);
		front.Pose(numFrameOld+1).m_aRotations[0].decompose(roty1, offset);

		quater dqFront, dqAdd;
		dqFront.difference(roty_2, roty_1);
		dqAdd.difference(roty, roty1);

		quater dq_mid;
		dq_mid.safeSlerp(dqFront, dqAdd, 0.5);

		quater target;
		target.mult(dq_mid, roty_1);

		quater dif;
		dif.difference(roty, target);
		MotionUtil::rotate(front, dif, numFrameOld, front.NumFrames());

		vector3 dvFront, dvAdd, dv;
		dvFront.difference(front.Pose(numFrameOld-2).m_aTranslations[0], front.Pose(numFrameOld-1).m_aTranslations[0]);
		dvAdd.difference(front.Pose(numFrameOld).m_aTranslations[0], front.Pose(numFrameOld+1).m_aTranslations[0]);
		dv.lerp(dvFront, dvAdd, 0.5);

		// translate
		vector3 translate;
		translate.difference(front.Pose(numFrameOld).m_aTranslations[0], front.Pose(numFrameOld-1).m_aTranslations[0]+dv);		
		translate.y=0;

		MotionUtil::translate(front, translate, numFrameOld, front.NumFrames());
	}	
}

void FootAdjustConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.NumFrames();
	mFirstConcat.concat(front, add);
	
	enum {LEFT, RIGHT, NUM_CON};

	int cnstr[NUM_CON];
	cnstr[0]=CONSTRAINT_LEFT_TOE;
	cnstr[1]=CONSTRAINT_RIGHT_TOE;
	
	int constraint1;
	int constraint2;
	matrixn footPos;
	vector3 footPos1[NUM_CON];
	vector3 footPos2[NUM_CON];
	bool needStitch[NUM_CON];
	
	int footStart, footEnd;
	intvectorn interval;

	for(int con=0; con<NUM_CON; con++)
	{
		constraint1=front.isConstraint(numFrameOld-front.NumFrames(0.12), numFrameOld, cnstr[con]);
		constraint2=front.isConstraint(numFrameOld,numFrameOld+front.NumFrames(0.12), cnstr[con]);

		if(constraint1!=-1 && constraint2!=-1)
		{
			needStitch[con]=true;
			mFootPrint.getFootInterval(front, constraint1, cnstr[con], footStart, footEnd);
			mFootPrint.getFootPrints(front, footStart, numFrameOld, cnstr[con], interval, footPos);

			if(footPos.rows()==0)
			{
				needStitch[con]=false;
			}
			else
			{
				footPos1[con]=footPos.row(footPos.rows()-1).toVector3();

				mFootPrint.getFootInterval(front, constraint2, cnstr[con], footStart, footEnd);
				mFootPrint.getFootPrints(front, numFrameOld, footEnd, cnstr[con], interval, footPos);
				
				if(footPos.rows()==0)
					needStitch[con]=false;
				else
					footPos2[con]=footPos.row(0).toVector3();
			}
		}
		else
			needStitch[con]=false;
	}

	
	if(needStitch[0] && needStitch[1])
	{
		// 두발의 방향과 센터를 align한다.
        vector3 delta1, delta2;		
		vector3 center1, center2;
		delta1.difference(footPos1[LEFT], footPos1[RIGHT]);
		delta2.difference(footPos2[LEFT], footPos2[RIGHT]);
		delta1.y=0;
		delta2.y=0;
		delta1.normalize();
		delta2.normalize();

		quater q;
		q.axisToAxis(delta2, delta1);

		center1.add(footPos1[LEFT], footPos1[RIGHT]);
		center2.add(footPos2[LEFT], footPos2[RIGHT]);
		center1/=2.0f;
		center2/=2.0f;

		/*
		front.translate(numFrameOld, front.NumFrames(), center2*-1);
		front.rotate(numFrameOld, front.NumFrames(), q);
		front.translate(numFrameOld, front.NumFrames(), center1);
		*/
		
		MotionUtil::translate(front, center1-center2,numFrameOld, front.NumFrames());
		
    }
	else if(needStitch[0] && !needStitch[1])
	{
		vector3 delta;
		delta.difference(footPos2[0], footPos1[0]);
		delta.y=0;
		MotionUtil::translate(front, delta,numFrameOld, front.NumFrames());		
	}
	else if(!needStitch[0] && needStitch[1])
	{
		vector3 delta;
		delta.difference(footPos2[1], footPos1[1]);
		delta.y=0;
		MotionUtil::translate(front, delta, numFrameOld, front.NumFrames());
	}
}
