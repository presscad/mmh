#include "stdafx.h"
#include "../math/mathclass.h"
#include ".\retarget.h"
#include "motion.h"
#include "motionutil.h"
#include "iksolver.h"
#include "../baselib/motion/motionloader.h"
#include "FootPrint.h"
#include "../math/intervals.h"
using namespace MotionUtil;

void Retarget::retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{
	intvectorn left, right;
	left.setSize(interval.size()/2);
	right.setSize(interval.size()/2+1);

	for(int i=0; i<left.size(); i++)
	{
		left[i]=interval[i*2];
		right[i]=interval[i*2+1];
	}

	right[right.size()-1]=-1;
	int currFootPrint=0;
	
	intvectorn iFootPrints;
	
	iFootPrints.setSize(endSafe-startSafe);
	iFootPrints.setAllValue(-1);	// means no constraint
	
#define setFootPrint(currFrame, currCon) iFootPrints[(currFrame)-startSafe]=(currCon)
#define getFootPrint(currFrame)	iFootPrints[(currFrame)-startSafe]

	for(int i=0; i<footPrints.rows(); i++)
	{
		for(int j=left[i]; j<right[i]; j++)
			setFootPrint(j, i);
	}

	intvectorn index;
	quaterN displacement[3];
	quaterN delta_rot;
	intvectorn displacementIndex(endSafe-startSafe);
	displacementIndex.setAllValue(-1);	// means no IK.
	
#define setDisplacement(currFrame, displacement) displacementIndex[(currFrame)-startSafe]=(displacement)
#define getDisplacement(currFrame) displacementIndex[(currFrame)-startSafe]

//#define SAVE_SIGNAL
#ifdef SAVE_SIGNAL
	matrixn saveSignal, signal2;
	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1);
#endif
	int conCount=iFootPrints.count(s2::INT_NOT_EQUAL, -1);
	displacement[0].setSize(conCount+1);
	displacement[1].setSize(conCount+1);
	displacement[2].setSize(conCount+1);
	
	int count=0;
	IKSolver ik;

	// 일단 ik를 수행후 결과를 저장한다.
	for(int i=startFrame; i<endFrame; i++)
	{
		if(getFootPrint(i)!=-1)
		{
			m_mot.skeleton().setPose(m_mot.Pose(i));
						
			ik.limbIK(m_mot.skeleton(), con, footPrints.row(getFootPrint(i)).toVector3(), index, delta_rot);
						
			float angle1=(m_mot.Pose(i).m_aRotations[index[2]]*delta_rot[2]).rotationAngle();
			float angle2=delta_rot[2].rotationAngle();
			
			//if(SQR(angle1)<0.1 && SQR(angle2) >0.1 )
			//if(SQR(angle1)<0.1 )|| ( SQR(angle1)<0.2 && SQR(angle2) >0.2))
			if(0)
			{
				//SetConstraint(i, con, false);

				// if leg is too straight, ignore IK
				int cur_footstep=getFootPrint(i);
				if(i==left[cur_footstep])
				{
					if(left[cur_footstep]+1<right[cur_footstep])
					{
						left[cur_footstep]++;
						setFootPrint(i,-1);
						continue;
					}						
				}
				else
				{					
					for(int k=i; k<right[cur_footstep]; k++)
						setFootPrint(k,-1);					
					right[cur_footstep]=i;
					continue;
				}
			}
			setDisplacement(i, count);
			displacement[0][count]=delta_rot[1];
			displacement[1][count]=delta_rot[2];
			displacement[2][count]=delta_rot[3];
			count++;

			m_mot.Pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			m_mot.Pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			m_mot.Pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
#ifdef SAVE_SIGNAL
			saveSignal[i-startFrame].assign(delta_rot[3]);
			signal2[i-startFrame][0]=1.f;
#endif
		}
	}
		
	if(count==0) return ;

	return;
	displacement[0][count]=quater(1,0,0,0);
	displacement[1][count]=quater(1,0,0,0);
	displacement[2][count]=quater(1,0,0,0);

	for(int i=startSafe; i<endSafe; i++)
	{
		if(getDisplacement(i)==-1)
			setDisplacement(i, count);
	}

	int numFootPrint= left.size();

	quaterN displacementMap;


	for(int i=0; i<=numFootPrint; i++)
	{
		int start=(i==0)?startSafe:right[i-1]-1;
		int end=(i==numFootPrint)?endSafe-1:left[i];

		if(end<startFrame) continue;
		if(start>=endFrame) continue;		

		//printf("[%d,%d)>[%d,%d)", start, end, startFrame, endFrame);
#ifdef SAVE_SIGNAL
		signal2[start-startFrame][2]=0;
		signal2[end-startFrame][2]=1;
#endif
		for(int j=0; j<3; j++)
		{
			if(end-start-1>0)
			{
				// transition version (c0 continuous)
				if(getDisplacement(start)==count||
					getDisplacement(end)==count)
					displacementMap.transition(displacement[j][getDisplacement(start)], 
												displacement[j][getDisplacement(end)], end-start-1);
				else
                    displacementMap.transition0(displacement[j][getDisplacement(start)], 
												displacement[j][getDisplacement(end)], end-start-1);

				for(int iframe=start+1; iframe<end; iframe++)
				{
					if(iframe<m_mot.NumFrames()) 
						m_mot.Pose(iframe).m_aRotations[index[j+1]].leftMult(displacementMap[iframe-(start+1)]);
#ifdef SAVE_SIGNAL
					saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start+1)]);
					signal2[iframe-startFrame][1]=1.f;
#endif
				}
				
				
				// c1 continuous version
				/*quater a, b;
				if(start-startFrame-1>0 && displacementIndex[start-startFrame-1]!=-1)
					a=displacement[j][displacementIndex[start-startFrame-1]];
				else 
					a=displacement[j][displacementIndex[start-startFrame]];

				if(end-startFrame+1<count && displacementIndex[end-startFrame+1]!=-1)
					b=displacement[j][displacementIndex[end-startFrame+1]];
				else
					b=displacement[j][displacementIndex[end-startFrame]];

				displacementMap.hermite0(a, displacement[j][displacementIndex[start-startFrame]], end-start+1,
										displacement[j][displacementIndex[end-startFrame]], b, 2.f);

				for(int iframe=start+1; iframe<end; iframe++)
				{
					Pose(iframe).m_aRotations[index[j+1]].leftMult(displacementMap[iframe-(start)]);
#ifdef SAVE_SIGNAL
					saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start)]);
					signal2[iframe-startFrame][1]=1.f;
#endif
				}
				*/

			}
		}
	}
#ifdef SAVE_SIGNAL
	saveSignal.op1(m1::each(v1::concat()), signal2); 
	CString filename;
	filename.Format("saveSignal%d.bmp", con);
	saveSignal.op0(m0::drawSignals(filename,-1, 1, true));

	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1.f);
#endif
}

void RetargetTest::retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{
	intvectorn index;
	quaterN delta_rot;

	IKSolver ik;

	for(int footStep=0; footStep<interval.size()/2; footStep++)
	{
		int left=interval[footStep*2];
		int right=interval[footStep*2+1];

		vector3 conPos=footPrints.row(footStep).toVector3();
		for(int i=left; i<right; i++)
		{
			m_mot.skeleton().setPose(m_mot.Pose(i));
						
			ik.limbIK(m_mot.skeleton(), con, conPos, index, delta_rot);
						
			m_mot.Pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			m_mot.Pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			m_mot.Pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
		}
	}
}

void Retarget2::retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{
	intvectorn index;
	quaterN prevDelta_rot;
	quaterN delta_rot;
	quaterN displacementMap;

	IKSolver ik;

	prevDelta_rot.setSize(4);
	prevDelta_rot[0].identity();
	prevDelta_rot[1].identity();
	prevDelta_rot[2].identity();
	prevDelta_rot[3].identity();

	intIntervals intervals;
	intervals.decodeFromVector(interval);

	
	int numFootPrint=intervals.size();

	for(int footStep=0; footStep<intervals.size(); footStep++)
	{
        if(intervals.start(footStep)>endFrame)
		{
			numFootPrint=footStep;
			break;
		}
		else if(intervals.end(footStep)>endFrame)
		{
			intervals.end(footStep)=endFrame;
		}
	}
	
	for(int footStep=0; footStep<numFootPrint; footStep++)
	{
		int left=intervals.start(footStep);
		int right=intervals.end(footStep);

		m_mot.skeleton().setPose(m_mot.Pose(left));
		ik.limbIK(m_mot.skeleton(), con, footPrints.row(footStep).toVector3(), index, delta_rot);
		m_mot.Pose(left).m_aRotations[index[1]].leftMult(delta_rot[1]);
		m_mot.Pose(left).m_aRotations[index[2]].leftMult(delta_rot[2]);
		m_mot.Pose(left).m_aRotations[index[3]].leftMult(delta_rot[3]);

		// fill gap "before" or "inbetween" footsteps

		int start=(footStep==0)?startSafe:intervals.end(footStep-1);
		int end=left;

		for(int j=1; j<4; j++)
		{
			if(end-start-1>0)
			{
				if(end-start>6)
					displacementMap.transition0(prevDelta_rot[j], delta_rot[j], end-start);
				else
					displacementMap.transition(prevDelta_rot[j], delta_rot[j], end-start);

				for(int iframe=start; iframe<end; iframe++)
				{
					if(iframe<m_mot.NumFrames()) 
						m_mot.Pose(iframe).m_aRotations[index[j]].leftMult(displacementMap[iframe-start]);
				}
			}
		}
		// IK
		for(int i=left+1; i<right; i++)
		{
			m_mot.skeleton().setPose(m_mot.Pose(i));
						
			ik.limbIK(m_mot.skeleton(), con, footPrints.row(footStep).toVector3(), index, delta_rot);
						
			m_mot.Pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			m_mot.Pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			m_mot.Pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
		}

		prevDelta_rot=delta_rot;
	}

	if(numFootPrint)
	{
		// fill gap after.
		int start=intervals.end(numFootPrint-1);
		int end=endSafe;

		for(int j=1; j<4; j++)
		{
			if(end-start-1>0)
			{
				delta_rot[j].identity();
				if(end-start>6)
					displacementMap.transition0(prevDelta_rot[j], delta_rot[j], end-start);
				else
					displacementMap.transition(prevDelta_rot[j], delta_rot[j], end-start);

				for(int iframe=start; iframe<end; iframe++)
				{
					if(iframe<m_mot.NumFrames()) 
						m_mot.Pose(iframe).m_aRotations[index[j]].leftMult(displacementMap[iframe-start]);
				}
			}
		}
	}
	
}

/*
void RetargetOnline::retarget(int startFrame, int endFrame, const intvectorn& aConstraint, const CTArray<matrixn>& conPos, CTArray<intvectorn>&  left, CTArray<intvectorn>& right)

{
	int numCon=aConstraint.size();

	vector3 temp;

	static matrixn importance;
	static hypermatrixn footPosition;
	static bitvectorn undefined;
	static intvectorn undefinedGrp;
	static vectorn time;
	static vectorn signal;
	static quaterN delta_rot;
	static intvectorn index;
	
	importance.setSize(numCon, endFrame-startFrame);
	
	footPosition.setSize(numCon, endFrame-startFrame, 3);

	// calc importances and goal foot positions
	for(int con=0; con<numCon; con++)
	{
		undefined.setSize(endFrame-startFrame);
		undefined.setAll();

		// set first frame
		undefined.clearAt(0);
		importance[con][0]=0.f;
		
		skeleton().setPose(Pose(startFrame));		
		skeleton().GetSiteBone(skeleton().GetJointIndexFromConstraint(aConstraint[con])).getTranslation(temp);
		footPosition[con][0].assign(temp);

		// set keyframes (importance==1.f)
		for(int cur_footstep=0; cur_footstep<left[con].size(); cur_footstep++)
		{
			int start=left[con][cur_footstep];
			int end=right[con][cur_footstep];

			for(int i=start; i<end; i++)
			{
				undefined.clearAt(i-startFrame);
				importance[con][i-startFrame]=1.f;
				footPosition[con][i-startFrame]=conPos[con][cur_footstep];
			}
		}

		// set last frame
		if(undefined[endFrame-startFrame-1])
		{
			undefined.clearAt(endFrame-startFrame-1);
			importance[con][endFrame-startFrame-1]=0.f;

			skeleton().setPose(Pose(endFrame-1));
			skeleton().GetSiteBone(skeleton().GetJointIndexFromConstraint(aConstraint[con])).getTranslation(temp);
			footPosition[con][endFrame-startFrame-1].assign(temp);
		}

		undefinedGrp.runLengthEncode(undefined);
        
		// fill undefined importance and foot positions
		for(int i=0; i<undefinedGrp.size()/2; i++)
		{
			int start=undefinedGrp[i*2];
			int end=undefinedGrp[i*2+1];

			vectorn& footStart=footPosition[con][start-1];
			vectorn& footEnd=footPosition[con][end];

			time.uniform(0, 1, end-start);
			signal.op1(v1::each(s1::SMOOTH_TRANSITION), time);

			for(int iframe=start; iframe<end; iframe++)
			{
				footPosition[con][iframe].interpolate(footStart, footEnd, signal[iframe-start]);
			}

			if(importance[con][start-1]==0 && importance[con][end]==0)
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=0;
			}
			else if(importance[con][start-1]==0)
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=ABS(time[iframe-start]);
			}
			else if(importance[con][end]==0)
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=ABS(1-time[iframe-start]);
			}
			else
			{
				for(int iframe=start; iframe<end; iframe++)
					importance[con][iframe]=ABS((time[iframe-start]-0.5)*2.f);
			}
		}
	}

	vector3 rootAdjust;
	for(int iframe=startFrame; iframe<endFrame; iframe++)
	{
		m_pSkeleton->setPose(Pose(iframe));

		ImIKSolver::ApplyIK(*m_pSkeleton, footPosition[0][iframe-startFrame].toVector3(), footPosition[1][iframe-startFrame].toVector3(), 
			importance[0][iframe-startFrame], importance[1][iframe-startFrame], rootAdjust, index, delta_rot);

		Pose(iframe).m_aTranslations[0]+=rootAdjust;

		for(int i=0; i<index.size(); i++)
			Pose(iframe).m_aRotations[index[i]].leftMult(delta_rot[i]);
	}
}
*/

/*
void Motion::adjustRootToConstraints(int startFrame, int endFrame, const intvectorn& aConstraint, const CTArray<matrixn>& aConPositions, CTArray<intvectorn>&  left, CTArray<intvectorn>& right)
{
	int numCon=aConstraint.size();
	for(int i=0; i<numCon; i++)
		right[i].push_back(-1);

	intvectorn curInterval(numCon);
	curInterval.setAllValue(0);
	
	CTArray<intvectorn> abConstraint;
	
	abConstraint.Init(numCon);
	for(int con=0; con<numCon; con++)
	{
		abConstraint[con].setSize(endFrame-startFrame);
		abConstraint[con].setAllValue(-1);	// means no constraint
	}

	// 매프레임마다 몇번째 foot print에 해당하는지를 저장한다.
	for(int i=startFrame; i<endFrame; i++)
	{
		for(int con=0; con<numCon; con++)
		{
			if(curInterval[con]<left[con].size())
			{
				if( i >= right[con][curInterval[con]]) curInterval[con]++;
				if(i<right[con][curInterval[con]] && left[con][curInterval[con]]<=i)
					abConstraint[con][i-startFrame]=curInterval[con];
			}
		}
	}

	for(int i=0; i<numCon; i++)
		VERIFY(right[i].popBack()==-1);

	matrixn rootDisplacement;
	rootDisplacement.setSize(0, 3);
	intvectorn displacementIndex(endFrame-startFrame);
	displacementIndex.setAllValue(-1);
	matrixn deltaConstraint;

//#define SAVE_SIGNAL
#ifdef SAVE_SIGNAL
	matrixn saveSignal, signal2;
	saveSignal.setSize(endFrame-startFrame, 3);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1);
#endif
	// for each constraint, retarget.
	int count=0;
	for(int i=startFrame; i<endFrame; i++)
	{
		m_pSkeleton->setPose(Pose(i));
		deltaConstraint.setSize(0,3);
		
		for(int con=0; con<numCon; con++)
		{
			int cur_footstep;
			if((cur_footstep=abConstraint[con][i-startFrame])!=-1)
			{
				vectorn deltaPos;
				vector3 curPos;
				m_pSkeleton->GetSiteBone(m_pSkeleton->GetJointIndexFromConstraint(aConstraint[con])).getTranslation(curPos);
				deltaPos.sub(aConPositions[con][cur_footstep], curPos);
				deltaConstraint.pushBack(deltaPos);
			}
		}

		if(deltaConstraint.rows()==1)
		{
			rootDisplacement.resize(count+1,3);
			rootDisplacement.row(count).avg(deltaConstraint);
			Pose(i).m_aTranslations[0]+=rootDisplacement.row(count).toVector3();

#ifdef SAVE_SIGNAL
			saveSignal[i-startFrame].assign(rootDisplacement.row(count));
			signal2[i-startFrame][0]=1;
#endif

			displacementIndex[i-startFrame]=count;
			count++;
		}
	}
	if(count==0) return;

	rootDisplacement.pushBack(vectorn(0,0,0));
	if(displacementIndex[0]==-1) displacementIndex[0]=count;
	if(displacementIndex[endFrame-startFrame-1]==-1) displacementIndex[endFrame-startFrame-1]=count;

	// root adjustment 구간을 만든다. left, right, numAdjust
	bitvectorn group;
	group.op(s2::EQUAL, displacementIndex, -1);
	intvectorn rootAdjust;
	rootAdjust.runLengthEncode(group);
	int numAdjust=rootAdjust.size()/2;
	vector3N displacementMap;

	for(int i=0; i<numAdjust; i++)
	{
		int start=rootAdjust[i*2]-1+startFrame;
		int end=rootAdjust[i*2+1]+startFrame;
#ifdef SAVE_SIGNAL
		signal2[start-startFrame][2]=0;
		signal2[end-startFrame][2]=1;
#endif
		if(end-start-1>0)
		{
			// transition
			displacementMap.transition(rootDisplacement[displacementIndex[start-startFrame]].toVector3(), 
									rootDisplacement[displacementIndex[end-startFrame]].toVector3(), end-start-1);

			for(int iframe=start+1; iframe<end; iframe++)
			{
				Pose(iframe).m_aTranslations[0]+=displacementMap[iframe-(start+1)];
#ifdef SAVE_SIGNAL
				saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start+1)]);
				signal2[iframe-startFrame][1]=1.f;
#endif
			}

		}
	}
#ifdef SAVE_SIGNAL
	saveSignal.op1(m1::each(v1::concat()), signal2); 
	saveSignal.op0(m0::drawSignals("saveRootSignal.bmp",-1, 1, true));
#endif
}
*/
/*
void Motion::retargetingConstraints(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, intvectorn& left, intvectorn& right)
{	

	right.push_back(-1);
	int currFootPrint=0;
	
	intvectorn iFootPrints;
	
	iFootPrints.setSize(endSafe-startSafe);
	iFootPrints.setAllValue(-1);	// means no constraint
	
#define setFootPrint(currFrame, currCon) iFootPrints[(currFrame)-startSafe]=(currCon)
#define getFootPrint(currFrame)	iFootPrints[(currFrame)-startSafe]

	for(int i=0; i<footPrints.rows(); i++)
	{
		for(int j=left[i]; j<right[i]; j++)
			setFootPrint(j, i);
	}

	intvectorn index;
	quaterN displacement[3];
	quaterN delta_rot;
	intvectorn displacementIndex(endSafe-startSafe);
	displacementIndex.setAllValue(-1);	// means no IK.
	
#define setDisplacement(currFrame, displacement) displacementIndex[(currFrame)-startSafe]=(displacement)
#define getDisplacement(currFrame) displacementIndex[(currFrame)-startSafe]

//#define SAVE_SIGNAL
#ifdef SAVE_SIGNAL
	matrixn saveSignal, signal2;
	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1);
#endif
	int conCount=iFootPrints.count(s2::NOTEQUAL, -1);
	displacement[0].setSize(conCount+1);
	displacement[1].setSize(conCount+1);
	displacement[2].setSize(conCount+1);
	

	int count=0;

	// 일단 ik를 수행후 결과를 저장한다.
	for(int i=startFrame; i<endFrame; i++)
	{
		if(getFootPrint(i)!=-1)
		{
			m_pSkeleton->setPose(Pose(i));
						
			ImIKSolver::ApplyIK(*m_pSkeleton, con, true, footPrints[getFootPrint(i)].toVector3(), index, delta_rot);
						
			float angle1=(Pose(i).m_aRotations[index[2]]*delta_rot[2]).rotationAngle();
			float angle2=delta_rot[2].rotationAngle();
			
			//if(SQR(angle1)<0.1 && SQR(angle2) >0.1 )
			//if(SQR(angle1)<0.1 )|| ( SQR(angle1)<0.2 && SQR(angle2) >0.2))
			if(0)
			{
				//SetConstraint(i, con, false);

				// if leg is too straight, ignore IK
				int cur_footstep=getFootPrint(i);
				if(i==left[cur_footstep])
				{
					if(left[cur_footstep]+1<right[cur_footstep])
					{
						left[cur_footstep]++;
						setFootPrint(i,-1);
						continue;
					}						
				}
				else
				{					
					for(int k=i; k<right[cur_footstep]; k++)
						setFootPrint(k,-1);					
					right[cur_footstep]=i;
					continue;
				}
			}
			setDisplacement(i, count);
			displacement[0][count]=delta_rot[1];
			displacement[1][count]=delta_rot[2];
			displacement[2][count]=delta_rot[3];
			count++;

			Pose(i).m_aRotations[index[1]].leftMult(delta_rot[1]);
			Pose(i).m_aRotations[index[2]].leftMult(delta_rot[2]);
			Pose(i).m_aRotations[index[3]].leftMult(delta_rot[3]);
#ifdef SAVE_SIGNAL
			saveSignal[i-startFrame].assign(delta_rot[3]);
			signal2[i-startFrame][0]=1.f;
#endif
		}
	}
		
	if(count==0) return ;

	displacement[0][count]=quater(1,0,0,0);
	displacement[1][count]=quater(1,0,0,0);
	displacement[2][count]=quater(1,0,0,0);

	for(int i=startSafe; i<endSafe; i++)
	{
		if(getDisplacement(i)==-1)
			setDisplacement(i, count);
	}

	int numFootPrint= left.size();

	quaterN displacementMap;


	for(int i=0; i<=numFootPrint; i++)
	{
		int start=(i==0)?startSafe:right[i-1]-1;
		int end=(i==numFootPrint)?endSafe-1:left[i];

		if(end<startFrame) continue;
		if(start>=endFrame) continue;

		//printf("[%d,%d)>[%d,%d)", start, end, startFrame, endFrame);
#ifdef SAVE_SIGNAL
		signal2[start-startFrame][2]=0;
		signal2[end-startFrame][2]=1;
#endif
		for(int j=0; j<3; j++)
		{
			if(end-start-1>0)
			{
				// transition version (c0 continuous)
				if(getDisplacement(start)==count||
					getDisplacement(end)==count)
					displacementMap.transition(displacement[j][getDisplacement(start)], 
												displacement[j][getDisplacement(end)], end-start-1);
				else
                    displacementMap.transition0(displacement[j][getDisplacement(start)], 
												displacement[j][getDisplacement(end)], end-start-1);

				for(int iframe=start+1; iframe<end; iframe++)
				{
					if(iframe<NumFrames()) 
						Pose(iframe).m_aRotations[index[j+1]].leftMult(displacementMap[iframe-(start+1)]);
#ifdef SAVE_SIGNAL
					saveSignal[iframe-startFrame].assign(displacementMap[iframe-(start+1)]);
					signal2[iframe-startFrame][1]=1.f;
#endif
				}
				
		

			}
		}
	}
#ifdef SAVE_SIGNAL
	saveSignal.op1(m1::each(v1::concat()), signal2); 
	CString filename;
	filename.Format("saveSignal%d.bmp", con);
	saveSignal.op0(m0::drawSignals(filename,-1, 1, true));

	saveSignal.setSize(endFrame-startFrame, 4);
	saveSignal.setAllValue(0);
	signal2.setSize(endFrame-startFrame, 3);
	signal2.setAllValue(-1.f);
#endif
}
*/

/*
// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
void Motion::retargetingConstraints(int startSafe, int startFrame, int endFrame, int endSafe)
{
	matrixn conPos;
	intvectorn left;
	intvectorn right;

	GetFootPrints(startSafe, endSafe, conPos, CONSTRAINT_LEFT_FOOT, left, right);
	retargetingConstraints(startSafe, startFrame, endFrame, endSafe, CONSTRAINT_LEFT_FOOT, conPos, left, right);
	
	GetFootPrints(startSafe, endSafe, conPos, CONSTRAINT_RIGHT_FOOT, left, right);
	retargetingConstraints(startSafe, startFrame, endFrame, endSafe, CONSTRAINT_RIGHT_FOOT, conPos, left, right);
}
*/

/// MOTION RETARGETTING


