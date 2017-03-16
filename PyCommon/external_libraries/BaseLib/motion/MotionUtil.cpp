#include "stdafx.h"
#include "motion.h"
#include "motionloader.h"
#include "../image/imageprocessor.h"
#include ".\motionutil.h"
#include "../baselib/math/operator.h"

using namespace MotionUtil;

void Coordinate::setCoordinate(vector3 const& origin, vector3 const& front) 
{
	mOrigin=origin;
	mOrientation.setAxisRotation(vector3(0,1,0) , vector3(0,0,1), front);
	mOrientation.align(quater(1,0,0,0));
}

quater Coordinate::toLocalRot(quater const& ori) const
{
	quater local;
	local.difference(mOrientation, ori);
	return local;
}

quater Coordinate::toGlobalRot(quater const& ori) const
{
	return ori*mOrientation;
}

quater Coordinate::toLocalDRot(quater const& drot) const
{
	// drot=ori2*ori1.inv;
	// localDrot= ori2 * mOri.inv * (ori1* mOri.inv ).inv
	//			= ori2 * mOri.inv * mOri *ori1.inv 
	//			= ori2* ori1.inv =drot

	return drot;
}

quater Coordinate::toGlobalDRot(quater const& ori) const
{
	return ori;
}

vector3 Coordinate::toLocalPos(vector3 const& pos) const
{
	vector3 lpos;
	lpos.rotate(mOrientation.inverse(), pos-mOrigin);
	return lpos;
}

vector3 Coordinate::toGlobalPos(vector3 const& lpos) const
{
	vector3 gpos;
	gpos.rotate(mOrientation, lpos);
	return gpos+mOrigin;
}

vector3 Coordinate::toLocalDir(vector3 const& dir) const
{
	vector3 gdir;
	gdir.rotate( mOrientation.inverse(), dir);
	return gdir;
}

vector3 Coordinate::toGlobalDir(vector3 const& dir) const
{
	vector3 ldir;
	ldir.rotate( mOrientation, dir);
	return ldir;
}

// retrieve joint translations	-> Only for translational joints!!!
// out: (end-start) by (ajoint.size()*3) matrix
// to retrieve the signal of the joint ajoint[i].
//  -> vec3ViewCol(out, i*3)
//  or out.range(startF, endF, i*3, (i+1)*3).toVector3N()
void GetSignal::transJointAll(const intvectorn& ajoint, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames())
		end=m_Motion.NumFrames();

	out.setSize(end-start, 3*ajoint.size());

	// get joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		vector3NView outJ(out.range(0, out.rows(), k*3, (k+1)*3).toVector3N());
		for(int i=start; i<end; i++)
			outJ.row(i-start)=m_Motion.Pose(i).m_aTranslations[j];
	}
}

void GetSignal::jointAll(const intvectorn& ajoint, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames())
		end=m_Motion.NumFrames();

	out.setSize(end-start, 4*ajoint.size());

	// get joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		quaterNView outJ(out.range(0, out.rows(), k*4, (k+1)*4).toQuaterN());
		for(int i=start; i<end; i++)
			outJ.row(i-start)=m_Motion.Pose(i).m_aRotations[j];
	}
}

void SetSignal::jointAll(const intvectorn& ajoint, matrixn const& in, int start)
{
	// set joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		quaterNView inJ(in.range(0, in.rows(), k*4, (k+1)*4).toQuaterN());
		for(int i=0; i<in.rows(); i++)
		{
			m_Motion.Pose(i+start).m_aRotations[j]=inJ.row(i);
		}
	}
}

		// set joint translations (for translational joints only)
		// out: (end-start) by (ajoint.size()*3) matrix
		// to retrieve the signal of the joint ajoint[i].
		//   -> vec3ViewCol(out, i*3)
		//   or out.range(startF, endF, i*3, (i+1)*3).toVector3N() 
void SetSignal::transJointAll(const intvectorn& ajoint, matrixn const& in, int start)
{
	// set joints
	for(int k=0; k<ajoint.size();k++)
	{
		int j=ajoint[k];
		vector3NView inJ(in.range(0, in.rows(), k*3, (k+1)*3).toVector3N());
		for(int i=0; i<in.rows(); i++)
		{
			m_Motion.Pose(i+start).m_aTranslations[j]=inJ.row(i);
		}
	}
}

void GetSignal::jointGlobalAll(int eRootCoord, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames())
		end=m_Motion.NumFrames();

	
	out.setSize(end-start, 4*m_Motion.NumJoints());

	// get root
	quaterNView root(out.range(0, out.rows(), 0, 4).toQuaterN());

	if(eRootCoord==GLOBAL_COORD)
	{
		for(int i=start; i<end; i++)
			root.row(i-start)=m_Motion.Pose(i).m_aRotations[0];
	}
	else
	{
		for(int i=start; i<end; i++)
			root.row(i-start).identity();
	}

	// get joints
	for(int j=1; j<m_Motion.NumJoints(); j++)
	{
		int parentJoint=m_Motion.Parent(j);
		quaterNView outJ(out.range(0, out.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView parentJ(out.range(0, out.rows(), parentJoint*4, (parentJoint+1)*4).toQuaterN());
		for(int i=start; i<end; i++)
			outJ.row(i-start).mult(parentJ.row(i-start), m_Motion.Pose(i).m_aRotations[j]);
	}
}
void SetSignal::jointGlobalAll(int eRootCoord, matrixn const& in, int start)
{
	// set root
	quaterNView root(in.range(0, in.rows(), 0, 4).toQuaterN());

	if(eRootCoord==GLOBAL_COORD)
	{
		for(int i=0; i<in.rows(); i++)
			m_Motion.Pose(i+start).m_aRotations[0]=root.row(i);
	}
	
	// set joints
	for(int j=1; j<m_Motion.NumJoints(); j++)
	{
		int parentJoint=m_Motion.Parent(j);
		quaterNView inJ(in.range(0, in.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView parentJ(in.range(0, in.rows(), parentJoint*4, (parentJoint+1)*4).toQuaterN());
		for(int i=0; i<in.rows(); i++)
		{
			m_Motion.Pose(i+start).m_aRotations[j].mult(parentJ.row(i).inverse(), inJ.row(i));
		}
	}
}

void GetSignal::root(matrixn& out, int start, int end)
{
if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

out.setSize(end-start,3);
for(int i=start; i<end; i++)
	out.row(i-start).assign(m_Motion.Pose(i).m_aTranslations[0]);
}

		
void GetSignal::additionalLinear(matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,m_Motion.Pose(start).m_additionalLinear.size());
	for(int i=start; i<end; i++)
		out.row(i-start).assign(m_Motion.Pose(i).m_additionalLinear);
}

void GetSignal::additionalQuater(matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

	out.setSize(end-start,m_Motion.Pose(start).m_additionalQuater.size());
	for(int i=start; i<end; i++)
		out.row(i-start).assign(m_Motion.Pose(i).m_additionalQuater);

}

void SetSignal::additionalLinear(matrixn const& in, int start)
{
	int end=start+in.rows();
	for(int i=start; i<end; i++)
		m_Motion.Pose(i).m_additionalLinear=in.row(i-start);
}

void SetSignal::additionalQuater(matrixn const& in, int start)
{
	int end=start+in.rows();
	for(int i=start; i<end; i++)
		m_Motion.Pose(i).m_additionalQuater=in.row(i-start);
}
void GetSignal::jointGlobal(int ijoint, matrixn& out, int start, int end)	
{
	jointGlobal(m_Motion.skeleton().getBoneByRotJointIndex(ijoint), out, start, end);
}

void GetSignal::jointPos(int ijoint, matrixn& out, int start, int end)			
{
	jointPos(m_Motion.skeleton().getBoneByRotJointIndex(ijoint), out, start, end);
}

void GetSignal::root(vector3N& out, int start, int end)
{
if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

out.setSize(end-start);
for(int i=start; i<end; i++)
	out[i-start]=m_Motion.Pose(i).m_aTranslations[0];
}

void GetSignal::joint(int ijoint, matrixn& out, int start, int end)
{
if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

out.setSize(end-start,4);
for(int i=start; i<end; i++)
	out.row(i-start).assign(m_Motion.Pose(i).m_aRotations[ijoint]);
}

void GetSignal::transJoint(int ijoint, matrixn& out, int start, int end)
{
if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

out.setSize(end-start,3);
for(int i=start; i<end; i++)
	out.row(i-start).assign(m_Motion.Pose(i).m_aTranslations[ijoint]);
}

void GetSignal::transJoint(int ijoint, vector3N& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

out.setSize(end-start);
for(int i=start; i<end; i++)
	out.row(i-start)=m_Motion.Pose(i).m_aTranslations[ijoint];
}


void GetSignal::joint(int ijoint, quaterN& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start);
	for(int i=start; i<end; i++)
		out.row(i-start)=m_Motion.Pose(i).m_aRotations[ijoint];
}

void GetSignal::jointGlobal(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,4);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.Pose(i));
		quater rot;
		bone.getRotation(rot);
		out.row(i-start).assign(rot);
	}
}

void MotionUtil::GetSignal::constraintPositions(matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

	out.setSize(end-start,6);

	for(int i=start; i<end; i++)
	{
		out.row(i-start).setVec3(0, m_Motion.Pose(i).m_conToeL);
		out.row(i-start).setVec3(3, m_Motion.Pose(i).m_conToeR);
	}
}


void GetSignal::jointFixed(int ijoint, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,4);
	Posture pose;
	for(int i=start; i<end; i++)
	{
		pose=m_Motion.Pose(i);
		pose.decomposeRot();
		pose.m_aTranslations[0]=vector3(0,pose.m_aTranslations[0].y, 0);
		pose.m_aRotations[0]=pose.m_offset_q;

		m_Motion.skeleton().setPose(pose);

		quater rot;

		m_Motion.skeleton().getBoneByRotJointIndex(ijoint).getRotation(rot);
		out.row(i-start).assign(rot);
	}
}

void GetSignal::jointPos(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,3);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.Pose(i));
		vector3 pos;
		bone.getTranslation(pos);
		out.row(i-start).assign(pos);
	}
}

void GetSignal::jointOri(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	aaPos.setSize(aJointIndex.size(), end-start, 4);

	for(int i=0; i<aJointIndex.size(); i++)
	{
		switch(eCoord)
		{
		case GLOBAL_COORD:
			jointGlobal(aJointIndex[i], aaPos[i], start, end);
			break;
		case LOCAL_COORD:
			joint(aJointIndex[i], aaPos[i], start, end);
			break;
		case FIXED_COORD:
			jointFixed(aJointIndex[i], aaPos[i], start, end);
			break;
		default:
			ASSERT(0);
		}
	}
}

void GetSignal::jointPos(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	aaPos.setSize(aJointIndex.size(), end-start, 3);

	for(int i=0; i<aJointIndex.size(); i++)
	{
		switch(eCoord)
		{
		case GLOBAL_COORD:
			jointPos(aJointIndex[i], aaPos[i], start, end);
			break;
		case LOCAL_COORD:
			jointPosLocal(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i], start, end);
			break;
		case FIXED_COORD:
			jointPosFixed(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i], start, end);
			break;
		case FIRST_FRAME_CENTERED_COORD:
			jointPosFirstCentered(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i], start, end);
			break;
		}
	}
}

void GetSignal::jointPosLocal(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,3);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.Pose(i));
		vector3 lpos, pos;
		bone.getTranslation(pos);
		pos-=m_Motion.Pose(i).m_aTranslations[0];
		lpos.rotate(m_Motion.Pose(i).m_aRotations[0].inverse(), pos);
		out.row(i-start).assign(lpos);
	}
}

void GetSignal::jointPosFirstCentered(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,3);
	Posture pose;
	vector3 pos;

	quater q;
	vector3 trans;
	m_Motion.Pose(start).decomposeRot();
	q.difference(m_Motion.Pose(start).m_rotAxis_y , quater(1,0,0,0));
	trans.difference(m_Motion.Pose(start).m_aTranslations[0], vector3(0, m_Motion.Pose(start).m_aTranslations[0].y,0));

	for(int i=start; i<end; i++)
	{
		pose=m_Motion.Pose(i);
		pose.m_aTranslations[0]+=trans;
		pose.m_aTranslations[0].rotate(q);
		pose.m_aRotations[0].leftMult(q);
		
		m_Motion.skeleton().setPose(pose);
		bone.getTranslation(pos);
		out.row(i-start).assign(pos);
	}
}


void GetSignal::jointPosFixed(const Bone& bone, matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start,3);
	for(int i=start; i<end; i++)
	{
		m_Motion.skeleton().setPose(m_Motion.Pose(i));
		vector3 lpos, pos;
		bone.getTranslation(pos);
		pos-=m_Motion.Pose(i).m_aTranslations[0];

		quater q, rotY, rotXZ;
		m_Motion.Pose(i).m_aRotations[0].decomposeTwistTimesNoTwist(vector3(0,1,0), rotY, rotXZ);

		lpos.rotate(rotY.inverse(), pos);

		lpos.y+=m_Motion.Pose(i).m_aTranslations[0].y;
		out.row(i-start).assign(lpos);
	}
}

void GetSignal::constraint(int iconstraint, bitvectorn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start);

	for(int i=start; i<end; i++)
		out.setValue(i-start, m_Motion.isConstraint(i, iconstraint));
}


void GetSignals::jointVel(const Bone& bone, matrixn& aVel, int eCoord, float fSmoothKernel)
{
	MotionUtil::SegmentFinder seg(m_Motion, 0, m_Motion.NumFrames());

	GetSignal sig(m_Motion);
	aVel.setSize(m_Motion.NumFrames(), 3);

	matrixn aSegPos;
	matrixn aSegVel;
    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		switch(eCoord)
		{
		case GLOBAL_COORD:
			sig.jointPos(bone, aSegPos, stt, end);
			break;
		case LOCAL_COORD:
			sig.jointPosLocal(bone, aSegPos, stt, end);
			break;
		case FIXED_COORD:
			sig.jointPosFixed(bone, aSegPos, stt, end);
			break;
		}
		
		aSegVel.derivative(aSegPos);

		if(fSmoothKernel!=0.f)
			aSegVel.op0(m1::filter(m_Motion.KernelSize(fSmoothKernel)));
		
		aVel.setValue(stt, 0, aSegVel);
	}
}

void GetSignals::jointPosVel(const Bone& bone, matrixn& aPos, matrixn& aVel, int eCoord, float fSmoothKernel)
{
	MotionUtil::SegmentFinder seg(m_Motion, 0, m_Motion.NumFrames());

	GetSignal sig(m_Motion);
	aPos.setSize(m_Motion.NumFrames(), 3);
	aVel.setSize(m_Motion.NumFrames(), 3);

	matrixn aSegPos;
	matrixn aSegVel;
    for(int iseg=0; iseg< seg.numSegment(); iseg++)
	{
		int stt=seg.startFrame(iseg);
		int end=seg.endFrame(iseg);

		switch(eCoord)
		{
		case GLOBAL_COORD:
			sig.jointPos(bone, aSegPos, stt, end);
			break;
		case LOCAL_COORD:
			sig.jointPosLocal(bone, aSegPos, stt, end);
			break;
		case FIXED_COORD:
			sig.jointPosFixed(bone, aSegPos, stt, end);
			break;
		}
		
		aSegVel.derivative(aSegPos);

		if(fSmoothKernel!=0.f)
			aSegVel.op0(m1::filter(m_Motion.KernelSize(fSmoothKernel)));
		
		aPos.setValue(stt, 0, aSegPos);
		aVel.setValue(stt, 0, aSegVel);
	}
}

void GetSignals::jointPos(const intvectorn& aJointIndex, hypermatrixn& aaPos, int eCoord)
{
	aaPos.setSize(aJointIndex.size(), m_Motion.NumFrames(), 3);

	GetSignal sig(m_Motion);
	for(int i=0; i<aJointIndex.size(); i++)
	{
		switch(eCoord)
		{
		case GLOBAL_COORD:
			sig.jointPos(aJointIndex[i], aaPos[i]);
			break;
		case LOCAL_COORD:
			sig.jointPosLocal(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i]);
			break;
		case FIXED_COORD:
			sig.jointPosFixed(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), aaPos[i]);
			break;
		}
	}	
}

void GetSignals::jointVel(const intvectorn& aJointIndex, hypermatrixn& aaVel, int eCoord, float fSmooth)
{
	aaVel.setSize(aJointIndex.size(), m_Motion.NumFrames(), 3);
	
	matrixn temp;
	for(int i=0; i<aJointIndex.size(); i++)
	{
		jointVel(m_Motion.skeleton().getBoneByRotJointIndex(aJointIndex[i]), temp, eCoord, fSmooth);
		aaVel[i].assign(temp);
	}
}

void SetSignal::root(const matrixn& in)
{
for(int i=0; i<m_Motion.NumFrames(); i++)
	m_Motion.Pose(i).m_aTranslations[0]=in.row(i).toVector3();
}

void SetSignal::constraint(int iconstraint, bitvectorn const& con, int start)
{
	for(int i=0; i<con.size(); i++)
		m_Motion.setConstraint(i+start, iconstraint, con[i]);
}

void SetSignal::root(const vector3N& in, int start)
{
for(int i=0; i<in.rows(); i++)
	m_Motion.Pose(i+start).m_aTranslations[0]=in[i];
}

void SetSignal::joint(int ijoint, const matrixn& in)
{
for(int i=0; i<m_Motion.NumFrames(); i++)
	m_Motion.Pose(i).m_aRotations[ijoint]=in.row(i).toQuater();
}

void SetSignal::joint(int ijoint, const quaterN& in, int start)
{
for(int i=0; i<in.rows(); i++)
	m_Motion.Pose(i+start).m_aRotations[ijoint]=in[i];
}

void SetSignal::transJoint(int ijoint, const matrixn& in)
{
	for(int i=0; i<in.rows(); i++)
		m_Motion.Pose(i).m_aTranslations[ijoint]=in.row(i).toVector3();
}

void SetSignal::transJoint(int ijoint, const vector3N& in, int start)
{
	for(int i=0; i<in.rows(); i++)
		m_Motion.Pose(i+start).m_aTranslations[ijoint]=in[i];
}


m_real MotionUtil::transitionCost(const Motion& mMotion, int from, int to)
{
	//!<  from까지 play하고 to+1부터 play하는 경우 transition cost
	const int interval=3;
	
	// find valid left range 
	int i, left_i, right_i;
	for(i=0; i>=-1*interval ;i--)
	{
		if(!(i+from-1>=0 && i+from-1<mMotion.NumFrames() &&	i+to-1>=0 && i+to-1<mMotion.NumFrames())
		||	(mMotion.IsDiscontinuous(i+from) || mMotion.IsDiscontinuous(i+to)) )
			break;
	}
	left_i=i;

	// find valid right range
	for(i=-1; i<interval ;i++)
	{
		if(!(i+from+1>=0 && i+from+1<mMotion.NumFrames() &&	i+to+1>=0 && i+to+1<mMotion.NumFrames())
		||	(mMotion.IsDiscontinuous(i+from+1) || mMotion.IsDiscontinuous(i+to+1)) )
			break;
	}
	right_i=i;

	int size;
	size=right_i-left_i+1;

	if(size<2)	return FLT_MAX;

	vectorn pointsA, pointsB;
	
	intvectorn EEIndex;
	
	EEIndex.setSize(13);
	int ind=0;
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::HIPS);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTANKLE);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTANKLE);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTKNEE);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTKNEE);	
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTWRIST);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTWRIST);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTELBOW);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTELBOW);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTSHOULDER);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTSHOULDER);	
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::CHEST2);
	EEIndex[ind++]=mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::HEAD);

	intvectorn& jointIndex=EEIndex;

	static hypermatrixn aaPosA, aaPosB;

	MotionUtil::GetSignal sig(mMotion);
	
	sig.jointPos(jointIndex, aaPosA, true, left_i+from, right_i+from+1);
	sig.jointPos(jointIndex, aaPosB, true, left_i+to, right_i+to+1);
	
	static matrixn temp;
	pointsA.fromMatrix(temp.fromHyperMat(aaPosA));
	pointsB.fromMatrix(temp.fromHyperMat(aaPosB));

	static KovarMetric metric;
	m_real distance=metric.CalcDistance(pointsA, pointsB);
	//distance : 최소 L2Distance
	//normalize by size-> sqrt(distance*distance/size);
	return distance*sqrt(1/(m_real)size);
}

void AngleAdjust::normalize(int startFrame, int endFrame, const intervalN& angleBound)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>mMotion.NumFrames()) endFrame=mMotion.NumFrames();
	ASSERT(angleBound.size()==mMotion.NumJoints());

	intervalN currAngleBound;
	calcAngleBound(startFrame, endFrame, currAngleBound);

/*	// scale each joint angle by (anglkeBound[j]/maxAngle[j]).
	for(int j=0; j<NumJoints(); j++)
	{
		m_real scaleFactor=angleBound[j]/maxAngle[j];
		for(int i=startFrame; i<endFrame; i++)
		{
			Pose(i).m_aRotations[j].scale(scaleFactor);			
		}
	}*/
}

void AngleAdjust::scaleJointAngles(int startFrame, int endFrame, const vectorn & scaleFactor)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>mMotion.NumFrames()) endFrame=mMotion.NumFrames();
	ASSERT(scaleFactor.size()==mMotion.NumJoints());

	// scale each joint angle by (anglkeBound[j]/maxAngle[j]).
	for(int j=0; j<mMotion.NumJoints(); j++)
	{
		for(int i=startFrame; i<endFrame; i++)
		{
			mMotion.Pose(i).m_aRotations[j].scale(scaleFactor[j]);
		}
	}
}

void AngleAdjust::calcAngleBound(int startFrame, int endFrame, intervalN& angleBound)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>mMotion.NumFrames()) endFrame=mMotion.NumFrames();
	angleBound.setSize(mMotion.NumJoints());
	angleBound.initEmpty();

	vectorn angle(mMotion.NumJoints());
	for(int i=startFrame; i<endFrame; i++)
	{
		for(int j=0; j<mMotion.NumJoints(); j++)
			angle[j]=mMotion.Pose(i).m_aRotations[j].rotationAngle();

		angleBound.enlarge(angle);
	}
}

void MotionUtil::upsample(Motion& out, const Motion& in, int startFrame, int endFrame, int nSuperSample)
{
	out.SetIdentifier(in.GetIdentifier());
	// in has no discontinuity
	ASSERT(in.IsValid(startFrame, endFrame));

	GetSignal signal(in);
	SetSignal signalOut(out);

	int inNumFrame=endFrame-startFrame;
	out.InitEmpty(in, inNumFrame*nSuperSample);

	matrixn root(inNumFrame,3);
	hypermatrixn joint(in.NumJoints(), inNumFrame, 4);

	matrixn rootOut(out.NumFrames(),3);
	hypermatrixn jointOut(in.NumJoints(), out.NumFrames(), 4);

	signal.root(root, startFrame, endFrame);
	rootOut.op1(m1::superSampling(nSuperSample),root);
	signalOut.root(rootOut);

	for(int ijoint=0; ijoint<in.NumJoints(); ijoint++)
	{
		signal.joint(ijoint, joint[ijoint], startFrame, endFrame);
		joint[ijoint].align();
		jointOut[ijoint].op1(m1::superSampling(nSuperSample), joint[ijoint]);
		jointOut[ijoint].each0(&vectorn::normalize);
		signalOut.joint(ijoint, jointOut[ijoint]);
	}
	
}

void MotionUtil::upsample(Motion& out, const Motion& in, int nSuperSample)
{
	out.SetIdentifier(in.GetIdentifier());

	MotionUtil::SegmentFinder sf(in,0, in.NumFrames());
	out.InitEmpty(in, 0);		
	out.FrameTime(in.FrameTime()/(float)nSuperSample);
	Motion temp;
	for(int iseg=0; iseg<sf.numSegment(); iseg++)
	{
		MotionUtil::upsample(temp, in, sf.startFrame(iseg), sf.endFrame(iseg), nSuperSample);
		out.Concat(&temp);	
	}

	ASSERT(out.NumFrames()==in.NumFrames()*nSuperSample);
}

void swap(quater& a, quater& b)
{
	quater c;
	c=a;
	a=b;
	b=c;
}

void MotionUtil::transpose(Motion& out, const Motion& in)
{
	out.InitEmpty(in, in.NumFrames());

	Posture globalpose;
	globalpose.Init(in.NumJoints(), in.NumTransJoints());

	quater rotAxisY;
	quater rotAxisX;
	quater offset_q;

	for(int i=0; i<in.NumFrames(); i++)
	{
		globalpose=in.Pose(i);
		in.skeleton().setPose(globalpose);
		
		for(int j=0; j<in.NumJoints(); j++)
			in.skeleton().getBoneByRotJointIndex(j).getRotation(globalpose.m_aRotations[j]);

		// transpose global pose.
		//  calculation rotAxis_y	global_rot = rotAxis_y*offset_q 
		// first transpose not-root joints
		for(int j=0; j<in.NumJoints(); j++)
		{
			quater& global_rot=globalpose.m_aRotations[j];

			global_rot.decomposeNoTwistTimesTwist(vector3(1,0,0), offset_q, rotAxisX);
			//y축으로 거꾸로 돌려 대칭 시킨다. 
			global_rot=offset_q.inverse()*rotAxisX;
		}

		// 왼팔다리, 오른팔다리 바꿔끼우기.
		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexByName("LEFTCOLLAR")],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexByName("RIGHTCOLLAR")]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTSHOULDER)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTSHOULDER)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTELBOW)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTELBOW)]);
		
		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTWRIST)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTWRIST)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTKNEE)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTKNEE)]);

		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTHIP)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTHIP)]);
		
		swap(globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTANKLE)],
			globalpose.m_aRotations[in.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTANKLE)]);

		out.Pose(i).m_aRotations[0]=globalpose.m_aRotations[0];
		// convert to local pose.
		for(int j=1; j<in.NumJoints(); j++)
			out.Pose(i).m_aRotations[j]=globalpose.m_aRotations[dep_GetParentJoint(in.skeleton(), j)].inverse()*globalpose.m_aRotations[j];

		out.Pose(i).m_aTranslations[0]=in.Pose(i).m_aTranslations[0];
		out.Pose(i).m_aTranslations[0].x*=-1;

		if(in.IsDiscontinuous(i))
			out.setDiscontinuity(i, true);
    }
}


void MotionUtil::downsample(Motion& out, const Motion& in, int nDownSample)
{
	int len=0;
	for(int i=0; i<in.NumFrames(); i+=nDownSample)
		len++;
    
	out.InitEmpty(in, len);
	out.FrameTime(in.FrameTime()*nDownSample);

	for(int i=0; i<in.NumFrames(); i+=nDownSample)
	{
		out.Pose(i/nDownSample)=in.Pose(i);
		for(int j=0; j<nDownSample; j++)
		{
			if(i+j<in.NumFrames() && in.IsDiscontinuous(i+j))
			{
				out.setDiscontinuity(i/nDownSample, true);
				break;
			}
		}
	}
} 

MotionUtil::PhysicalHuman::PhysicalHuman(const Motion& srcMotion)
:mMotion(srcMotion)
{
	//HEAD, LUPPERARM, RUPPERARM, LLOWERARM, RLOWERARM, LHAND, RHAND, PELVIS, TORSO, LTHIGH, RTHIGH, LSHIN ,RSHIN, LFOOT, RFOOT 
	// 7.1   3.3                     1.9                 0.6           15.3   22+4  10.5              6           1.5
	mSegMass.setValues(NUM_SEGMENT, 7.1, 3.3, 3.3, 1.9, 1.9, 0.6, 0.6, 15.3, 26.0, 10.5, 10.5, 6.0, 6.0, 1.5, 1.5);
	_init();
}

MotionUtil::PhysicalHuman::PhysicalHuman(const Motion& srcMotion, const vectorn& massDistribution)
:mMotion(srcMotion)
{
	mSegMass=massDistribution;
	_init();
}

vector3 posVoca(MotionLoader const& s, int jointVoca) 
{ vector3 trans; s.getBoneByVoca(jointVoca).getTranslation(trans); return trans;}

void MotionUtil::PhysicalHuman::_init()
{
	mSegLength.setSize(NUM_SEGMENT);

	MotionLoader& s=mMotion.skeleton();
	
	mSegLength[HEAD]=posVoca(s,MotionLoader::HEAD).distance(posVoca(s,MotionLoader::NECK))*2;
	mSegLength[LUPPERARM]=posVoca(s,MotionLoader::LEFTSHOULDER).distance(posVoca(s,MotionLoader::LEFTELBOW));
	mSegLength[RUPPERARM]=posVoca(s,MotionLoader::RIGHTSHOULDER).distance(posVoca(s,MotionLoader::RIGHTELBOW));
	mSegLength[LLOWERARM]=posVoca(s,MotionLoader::LEFTWRIST).distance(posVoca(s,MotionLoader::LEFTELBOW));
	mSegLength[RLOWERARM]=posVoca(s,MotionLoader::RIGHTWRIST).distance(posVoca(s,MotionLoader::RIGHTELBOW));
	mSegLength[LHAND]= 0;
	mSegLength[RHAND]= 0;
	mSegLength[PELVIS]=mSegLength[HEAD];
	mSegLength[TORSO]= posVoca(s,MotionLoader::HIPS).distance(posVoca(s,MotionLoader::NECK));
	mSegLength[LTHIGH]= posVoca(s,MotionLoader::LEFTHIP).distance(posVoca(s,MotionLoader::LEFTKNEE));
	mSegLength[RTHIGH]= posVoca(s,MotionLoader::RIGHTHIP).distance(posVoca(s,MotionLoader::RIGHTKNEE));
	mSegLength[LSHIN]= posVoca(s,MotionLoader::LEFTANKLE).distance(posVoca(s,MotionLoader::LEFTKNEE));
	mSegLength[RSHIN]= posVoca(s,MotionLoader::RIGHTANKLE).distance(posVoca(s,MotionLoader::RIGHTKNEE));
	mSegLength[LFOOT]= 0;
	mSegLength[RFOOT]= 0;
}

void MotionUtil::PhysicalHuman::segPositions(hypermatrixn& aaSegCOG, int start, int end)
{
	if(start<0) start=0;
	if(end>mMotion.NumFrames()) end=mMotion.NumFrames();

	aaSegCOG.setSize(NUM_SEGMENT, end-start, 3);

	

	for(int i=start; i<end; i++)
	{

		mMotion.setSkeleton(i);
		MotionLoader& s=mMotion.skeleton();

		aaSegCOG[HEAD].row(i-start).assign(posVoca(s,MotionLoader::HEAD));
		aaSegCOG[LUPPERARM].row(i-start).assign((posVoca(s,MotionLoader::LEFTSHOULDER)+posVoca(s,MotionLoader::LEFTELBOW))/2.f);
		aaSegCOG[RUPPERARM].row(i-start).assign((posVoca(s,MotionLoader::RIGHTSHOULDER)+posVoca(s,MotionLoader::RIGHTELBOW))/2.f);
		aaSegCOG[LLOWERARM].row(i-start).assign((posVoca(s,MotionLoader::LEFTWRIST)+posVoca(s,MotionLoader::LEFTELBOW))/2.f);
		aaSegCOG[RLOWERARM].row(i-start).assign((posVoca(s,MotionLoader::RIGHTWRIST)+posVoca(s,MotionLoader::RIGHTELBOW))/2.f);
		aaSegCOG[LHAND].row(i-start).assign(posVoca(s,MotionLoader::LEFTWRIST));
		aaSegCOG[RHAND].row(i-start).assign(posVoca(s,MotionLoader::RIGHTWRIST));
		aaSegCOG[PELVIS].row(i-start).assign(posVoca(s,MotionLoader::HIPS));
		aaSegCOG[TORSO].row(i-start).assign(posVoca(s,MotionLoader::HIPS)*0.4+posVoca(s,MotionLoader::NECK)*0.6);
		aaSegCOG[LTHIGH].row(i-start).assign(( posVoca(s,MotionLoader::LEFTHIP)+posVoca(s,MotionLoader::LEFTKNEE))/2.f);
		aaSegCOG[RTHIGH].row(i-start).assign(( posVoca(s,MotionLoader::RIGHTHIP)+posVoca(s,MotionLoader::RIGHTKNEE))/2.f);
		aaSegCOG[LSHIN].row(i-start).assign(( posVoca(s,MotionLoader::LEFTANKLE)+posVoca(s,MotionLoader::LEFTKNEE))/2.f);
		aaSegCOG[RSHIN].row(i-start).assign(( posVoca(s,MotionLoader::RIGHTANKLE)+posVoca(s,MotionLoader::RIGHTKNEE))/2.f);
		vector3 toe;
		dep_GetSiteBoneVoca(s, MotionLoader::LEFTANKLE).getTranslation(toe);
		aaSegCOG[LFOOT].row(i-start).assign((posVoca(s,MotionLoader::LEFTANKLE)+toe)/2.f);
		dep_GetSiteBoneVoca(s, MotionLoader::RIGHTANKLE).getTranslation(toe);
		aaSegCOG[RFOOT].row(i-start).assign((posVoca(s,MotionLoader::RIGHTANKLE)+toe)/2.f);
	}
}

void MotionUtil::PhysicalHuman::segVelocity(hypermatrixn& aaSegVel, const hypermatrixn& aaSegPos, float kernelSize)
{
	aaSegVel.setSameSize(aaSegPos);
	aaSegVel.each(m1::derivative(), aaSegPos);
	for(int page=0; page<aaSegVel.page(); page++)
	{
		aaSegVel[page]/=mMotion.FrameTime();

		aaSegVel[page].op0(m1::filter(mMotion.KernelSize(kernelSize)));		
	}
}

void MotionUtil::PhysicalHuman::segAcceleration(hypermatrixn& aaSegAcc, const hypermatrixn& aaSegVel, float kernelSize)
{
	// calc derivative
	segVelocity(aaSegAcc, aaSegVel, kernelSize);   
}


void MotionUtil::PhysicalHuman::COM(matrixn& aCOM, const hypermatrixn& aaSegPos)
{
	int numFrame=aaSegPos.rows();
	aCOM.setSize(numFrame, 3);

	m_real totalMass=mSegMass.sum();
	for(int i=0; i<numFrame; i++)
	{
		aCOM.setRow(i, (m_real)0.0);
		for(int seg=0; seg<NUM_SEGMENT; seg++)
			aCOM.row(i).multAdd(aaSegPos[seg].row(i), mSegMass[seg]);
		aCOM.row(i)/=totalMass;
	}
}

void MotionUtil::PhysicalHuman::linearMomentum(matrixn& aLmomentum, const matrixn& aCOM)
{
	aLmomentum.derivative(aCOM);
	aLmomentum/=mMotion.FrameTime();
}
void MotionUtil::PhysicalHuman::angularMomentum(std::set<int> except,matrixn& aAmomentum, const matrixn& aCOM, const hypermatrixn& aaSegPos, const hypermatrixn& aaSegVel)
{
	int numFrame=aaSegPos.rows();
	aAmomentum.setSize(numFrame, 3);

	vector3 ri, h, hi;
	for(int i=0; i<numFrame; i++)
	{
		h.x=h.y=h.z=0;
		for(int seg=0; seg<NUM_SEGMENT; seg++)
		{
			if(except.find(seg)!=except.end())continue;
			ri=(aaSegPos[seg].row(i)-aCOM.row(i)).toVector3();
			hi.cross(ri, aaSegVel[seg].row(i).toVector3());
			hi*=mSegMass[seg];
			h+=hi;
		}
		aAmomentum.row(i).assign(h);
	}
}

void MotionUtil::PhysicalHuman::angularMomentum(matrixn& aAmomentum, const matrixn& aCOM, const hypermatrixn& aaSegPos, const hypermatrixn& aaSegVel)
{
	int numFrame=aaSegPos.rows();
	aAmomentum.setSize(numFrame, 3);

	vector3 ri, h, hi;
	for(int i=0; i<numFrame; i++)
	{
		h.x=h.y=h.z=0;
		for(int seg=0; seg<NUM_SEGMENT; seg++)
		{
			ri=(aaSegPos[seg].row(i)-aCOM.row(i)).toVector3();
			hi.cross(ri, aaSegVel[seg].row(i).toVector3());
			hi*=mSegMass[seg];
			h+=hi;
		}
		aAmomentum.row(i).assign(h);
	}
}

// MeterPerUnit is required for calculating gravity G in the virtual unit system.
void MotionUtil::PhysicalHuman::ZMP(matrixn& aZMP, float MeterPerUnit, const hypermatrixn& aaSegPos, const hypermatrixn& aaSegAcc)
{
	float g=9.8;		//meter/second^2
	g/=MeterPerUnit;	// unit/second^2
	g*=-1;
	
	int numFrame=aaSegPos.rows();
	aZMP.setSize(numFrame, 3);

	vector3 ri, ri__;
	m_real mi, A, B, C, D, E;
	for(int f=0; f<numFrame; f++)
	{
		A= B= C= D= E= 0.f;
		for(int i=0; i<NUM_SEGMENT; i++)
		{
			ri=aaSegPos[i].row(f).toVector3();
			ri__=aaSegAcc[i].row(f).toVector3();
			mi=mSegMass[i];

			A+=mi*(ri__.y-g  )*ri.x;
			B+=mi*(ri__.x-0.f)*ri.y;
			C+=mi*(ri__.y-g  );
			D+=mi*(ri__.y-g  )*ri.z;
			E+=mi*(ri__.z-0.f)*ri.y;
		}

		aZMP[f][0]=(A-B)/C;
		aZMP[f][1]=0.f;
		aZMP[f][2]=(D-E)/C;		
	}
}

void MotionUtil::PhysicalHuman::energy(vectorn& aEnergy, bool bSmooth)
{
	int start=0;
	int end=mMotion.NumFrames();

	hypermatrixn aaJointPositions;
	hypermatrixn aaJointRotations;
	
	intvectorn aJointIndex(MotionLoader::NUM_JOINT_VOCA);
	
	aaJointPositions.setSize(MotionLoader::NUM_JOINT_VOCA, end-start, 3);
	aaJointRotations.setSize(MotionLoader::NUM_JOINT_VOCA, end-start, 4);

	GetSignal gs(mMotion);

	for(int i=0; i<MotionLoader::NUM_JOINT_VOCA; i++)
	{
		aJointIndex[i]=mMotion.skeleton().getRotJointIndexFromVoca(i);

		gs.jointPos(aJointIndex[i], aaJointPositions[i], start, end);
		gs.jointGlobal(aJointIndex[i], aaJointRotations[i], start, end);
	}

	vectorn aSegMass;
	segMass(aSegMass);
	vectorn aLength;
	segLength(aLength, aaJointPositions);
	
	hypermatrixn aaSegCOG;
	segPositions(aaSegCOG, 0, mMotion.NumFrames());

	hypermatrixn aaSegCOGvel;
	aaSegCOGvel.setSameSize(aaSegCOG);
	for(int i=0; i<aaSegCOG.page(); i++)	aaSegCOGvel[i].derivative(aaSegCOG[i]);
	matrixn aaSegCOGspeed(aaSegCOGvel.page(), aaSegCOGvel.rows());
	for(int i=0; i<aaSegCOG.page(); i++)
	{
		aaSegCOGspeed.row(i).aggregate(CAggregate::LENGTH, aaSegCOGvel[i]);
		aaSegCOGspeed.row(i)*=0.5/mMotion.FrameTime();
	}

	hypermatrixn aaSegCOGangVel;
	aaSegCOGangVel.setSize(NUM_SEGMENT, aaJointPositions.rows(), aaJointPositions.cols());

	// quaternion smoothing
	if(bSmooth)
	{
		vectorn kernel;
		float fFrameTime=mMotion.FrameTime();
		int kernel_size=Filter::CalcKernelSize(0.6, fFrameTime);
		Filter::GetGaussFilter(kernel_size, kernel);

		for(int i=0; i<aaJointRotations.page(); i++)
			Filter::LTIFilterQuat(10, kernel, aaJointRotations.page(i));
	}

	aaSegCOGangVel[HEAD].derivativeQuater(aaJointRotations[MotionLoader::HEAD]);
	aaSegCOGangVel[LUPPERARM].derivativeQuater(aaJointRotations[MotionLoader::LEFTSHOULDER]);
	aaSegCOGangVel[RUPPERARM].derivativeQuater(aaJointRotations[MotionLoader::RIGHTSHOULDER]);
	aaSegCOGangVel[LLOWERARM].derivativeQuater(aaJointRotations[MotionLoader::LEFTELBOW]);
	aaSegCOGangVel[RLOWERARM].derivativeQuater(aaJointRotations[MotionLoader::RIGHTELBOW]);
	aaSegCOGangVel[LHAND].derivativeQuater(aaJointRotations[MotionLoader::LEFTWRIST]);
	aaSegCOGangVel[RHAND].derivativeQuater(aaJointRotations[MotionLoader::RIGHTWRIST]);
	aaSegCOGangVel[TORSO].derivativeQuater(aaJointRotations[MotionLoader::HIPS]);
	aaSegCOGangVel[LTHIGH].derivativeQuater(aaJointRotations[MotionLoader::LEFTHIP]);
	aaSegCOGangVel[RTHIGH].derivativeQuater(aaJointRotations[MotionLoader::RIGHTHIP]);
	aaSegCOGangVel[LSHIN].derivativeQuater(aaJointRotations[MotionLoader::LEFTKNEE]);
	aaSegCOGangVel[RSHIN].derivativeQuater(aaJointRotations[MotionLoader::RIGHTKNEE]);
	aaSegCOGangVel[LFOOT].derivativeQuater(aaJointRotations[MotionLoader::LEFTANKLE]);
	aaSegCOGangVel[RFOOT].derivativeQuater(aaJointRotations[MotionLoader::RIGHTANKLE]);

	for(int i=0; i<aaSegCOGangVel.page(); i++)
		aaSegCOGangVel.page(i)*=0.5/mMotion.FrameTime();

	// 0.6초 정도의 kernel size
    
	vectorn blurFilter;
	Filter::GetGaussFilter(Filter::CalcKernelSize(0.4, mMotion.FrameTime()), blurFilter);

	aEnergy.setSize(aaSegCOGspeed.cols());
	aEnergy.setAllValue(0);
	//aaSegCOGspeed.save("segCOGspeed.txt",false);
	//aaSegCOGangSpeed.save("segCOGangSpeed.txt",false);

	for(int i=0; i<NUM_SEGMENT; i++)
	{
		TString msg=aaSegCOGspeed.row(i).output("%f", 0, 20);
		TRACE(msg+"\n");
		if(bSmooth)
		{
			Filter::LTIFilter(1, blurFilter, aaSegCOGspeed.row(i));
			Filter::LTIFilter(1, blurFilter, aaSegCOGangVel[i]);
		}

		// 1/2 mv^2
		aaSegCOGspeed.row(i).each1(s1::SQUARE, aaSegCOGspeed.row(i));
		aaSegCOGspeed.row(i)*=aSegMass[i]/2.f;

		aEnergy+=aaSegCOGspeed.row(i);

		// 1/2 Iw^2 

		// I(t)=R(t)*I_body*R(t)^T
		// T= w*(I(t)w)/2

		for(int iframe=0; iframe<aaSegCOGangVel.rows(); iframe++)
		{
			quater qR;
			matrix4 I_body;

			// slender rod
			I_body.setIdentityRot();
			// Ixx
			I_body._11=1.f/12.f*aSegMass[i]*aLength[i]*aLength[i];
			// Iyy
			I_body._22=0;
			// Izz
			I_body._33=1.f/12.f*aSegMass[i]*aLength[i]*aLength[i];

			//aSegMass[i]
			vector3 w=aaSegCOGangVel.page(i).row(iframe).toVector3();
			switch(i)
			{
			case HEAD:qR=aaJointRotations.page(MotionLoader::HEAD).row(iframe).toQuater();break;
			case LUPPERARM:qR=aaJointRotations.page(MotionLoader::LEFTSHOULDER).row(iframe).toQuater();break;
			case RUPPERARM:qR=aaJointRotations.page(MotionLoader::RIGHTSHOULDER).row(iframe).toQuater();break;
			case LLOWERARM:qR=aaJointRotations.page(MotionLoader::LEFTELBOW).row(iframe).toQuater();break;
			case RLOWERARM:qR=aaJointRotations.page(MotionLoader::RIGHTELBOW).row(iframe).toQuater();break;
			case LHAND:qR=aaJointRotations.page(MotionLoader::LEFTWRIST).row(iframe).toQuater();break;
			case RHAND:qR=aaJointRotations.page(MotionLoader::RIGHTWRIST).row(iframe).toQuater();break;
			case TORSO:qR=aaJointRotations.page(MotionLoader::HIPS).row(iframe).toQuater();break;
			case LTHIGH:qR=aaJointRotations.page(MotionLoader::LEFTHIP).row(iframe).toQuater();break;
			case RTHIGH:qR=aaJointRotations.page(MotionLoader::RIGHTHIP).row(iframe).toQuater();break;
			case LSHIN:qR=aaJointRotations.page(MotionLoader::LEFTKNEE).row(iframe).toQuater();break;
			case RSHIN:qR=aaJointRotations.page(MotionLoader::RIGHTKNEE).row(iframe).toQuater();break;
			case LFOOT:qR=aaJointRotations.page(MotionLoader::LEFTANKLE).row(iframe).toQuater();break;
			case RFOOT:qR=aaJointRotations.page(MotionLoader::RIGHTANKLE).row(iframe).toQuater();break;
			}

			matrix4 matI;
			quater qRT;
			qRT.inverse(qR);
			matI.mult(I_body, qRT);
			matI.leftMultRotation(qR);
            vector3 Iw;
			Iw.mult(matI, w);
			aEnergy[iframe]+=(w%Iw)/2;
		}
	}
}

void MotionUtil::PhysicalHuman::segLength(vectorn& aLength, hypermatrixn& aaJointPositions)
{
	aLength.setSize(NUM_SEGMENT);
	aLength[HEAD]=aaJointPositions[MotionLoader::HEAD].row(0).distance(aaJointPositions[MotionLoader::NECK].row(0))*2;
	aLength[LUPPERARM]=aaJointPositions[MotionLoader::LEFTSHOULDER].row(0).distance(aaJointPositions[MotionLoader::LEFTELBOW].row(0));
	aLength[RUPPERARM]=aaJointPositions[MotionLoader::RIGHTSHOULDER].row(0).distance(aaJointPositions[MotionLoader::RIGHTELBOW].row(0));
	aLength[LLOWERARM]=aaJointPositions[MotionLoader::LEFTWRIST].row(0).distance(aaJointPositions[MotionLoader::LEFTELBOW].row(0));
	aLength[RLOWERARM]=aaJointPositions[MotionLoader::RIGHTWRIST].row(0).distance(aaJointPositions[MotionLoader::RIGHTELBOW].row(0));
	aLength[LHAND]= 0;
	aLength[RHAND]= 0;
	aLength[PELVIS]=aLength[HEAD];
	aLength[TORSO]= aaJointPositions[MotionLoader::HIPS].row(0).distance(aaJointPositions[MotionLoader::NECK].row(0));
	aLength[LTHIGH]= aaJointPositions[MotionLoader::LEFTHIP].row(0).distance(aaJointPositions[MotionLoader::LEFTKNEE].row(0));
	aLength[RTHIGH]= aaJointPositions[MotionLoader::RIGHTHIP].row(0).distance(aaJointPositions[MotionLoader::RIGHTKNEE].row(0));
	aLength[LSHIN]= aaJointPositions[MotionLoader::LEFTANKLE].row(0).distance(aaJointPositions[MotionLoader::LEFTKNEE].row(0));
	aLength[RSHIN]= aaJointPositions[MotionLoader::RIGHTANKLE].row(0).distance(aaJointPositions[MotionLoader::RIGHTKNEE].row(0));
	aLength[LFOOT]= 0;
	aLength[RFOOT]= 0;
}

void MotionUtil::PhysicalHuman::calcAllAggregate(int start, int end, matrixn& aCOM, matrixn& aLmomentum, matrixn& aAmomentum, matrixn& aZMP, float meterPerUnit)
{
	MotionUtil::SegmentFinder seg(mMotion, start, end);

	hypermatrixn aaSegPos;
	hypermatrixn aaSegVel;
	hypermatrixn aaSegAcc;

	aCOM.setSize(end-start, 3);
	aLmomentum.setSize(end-start, 3);
	aAmomentum.setSize(end-start, 3);
	aZMP.setSize(end-start,3);

	matrixn partialCOM;
	matrixn partialLmomentum;
	matrixn partialAmomentum;
	matrixn partialZMP;
	for(int iseg=0; iseg<seg.numSegment(); iseg++)
	{
		int segStart=seg.startFrame(iseg);
		int segEnd=seg.endFrame(iseg);

		segPositions(aaSegPos, segStart, segEnd);
		segVelocity(aaSegVel, aaSegPos, 1.f);
		segAcceleration(aaSegAcc, aaSegVel, 1.f);

		COM(partialCOM, aaSegPos);
		linearMomentum(partialLmomentum, partialCOM);
		angularMomentum(partialAmomentum, partialCOM, aaSegPos, aaSegVel);
		ZMP(partialZMP, meterPerUnit,  aaSegPos, aaSegAcc);

		aCOM.range(segStart-start, segEnd-start).assign(partialCOM);		
		aLmomentum.range(segStart-start, segEnd-start).assign(partialLmomentum);
		aAmomentum.range(segStart-start, segEnd-start).assign(partialAmomentum);
		aZMP.range(segStart-start, segEnd-start).assign(partialZMP);
	}	
}

void MotionUtil::PhysicalHuman::COM(matrixn& aCOM, int start, int end)
{
	if(start<0) start=0;
	if(end>mMotion.NumFrames()) end=mMotion.NumFrames();

	MotionUtil::SegmentFinder seg(mMotion, start, end);

	hypermatrixn aaSegPos;
	aCOM.setSize(end-start, 3);
	matrixn partialCOM;

	for(int iseg=0; iseg<seg.numSegment(); iseg++)
	{
		int segStart=seg.startFrame(iseg);
		int segEnd=seg.endFrame(iseg);

		segPositions(aaSegPos, segStart, segEnd);
		COM(partialCOM, aaSegPos);
		aCOM.range(segStart-start, segEnd-start).assign(partialCOM);
	}
}

void MotionUtil::smooth(Motion& out, const Motion& in, float kernelRoot, float kernelJoint)
{
	Msg::error("Not implemented yet");
/*
	Motion* pSmoothMotion=Clone();
	// 0.23초 vs 0.1초 


	/////////////////////////////////////////////////////////////////////////	
	// Smoothing for root position values
	int KERNEL_SIZE= Filter::CalcKernelSize(0.23f, GetFrameTime());
	int NUM_ITER= 1;

	//#joint orientation filter
	int NUM_ITER_B= Filter::CalcKernelSize(0.1f, GetFrameTime());
	int KERNEL_SIZE_B= 3;

	// bluring filter of kernel_size
	vectorn ai_array, bi_array;

	Filter::GetBoxFilter(KERNEL_SIZE, ai_array);
	Filter::GetBlurFilter(KERNEL_SIZE_B, bi_array);
	pSmoothMotion->LTIFilter(NUM_ITER, ai_array, NUM_ITER_B, bi_array);

	void PostureIP::LTIFilter(int numIterA, const vectorn& kernelA, int numIterB, const vectorn& kernelB)
{
	D3DXQUATERNION* aQuat = new D3DXQUATERNION[GetNumPosture()];
	D3DXVECTOR3* aVec3 = new D3DXVECTOR3[GetNumPosture()];

	for(int i=0; i<GetNumPosture(); i++)
		aVec3[i]=GetPosture(i)->m_aTranslations[0];

	Filter::LTIFilter(numIterA, kernelA, GetNumPosture(), aVec3);

	for(i=0; i<GetNumPosture(); i++)
		GetPosture(i)->m_aTranslations[0]=aVec3[i];

	
	for(int j=0; j<m_numJoint; j++)
	{
		for(int i=0; i<GetNumPosture(); i++)
			aQuat[i]=GetPosture(i)->m_aRotations[j];			

		Filter::AlignUnitQuaternions(GetNumPosture(), aQuat);
		Filter::LTIFilter(numIterB, kernelB, GetNumPosture(), aQuat);

		for(i=0; i<GetNumPosture(); i++)
			GetPosture(i)->m_aRotations[j]=aQuat[i];
	}
	
	delete[] aQuat;
	delete[] aVec3;
}*/

}

void MotionUtil::scale(Motion& inout, float ratio)
{
	//!< root translation을 scale한다.
	for(int i=0; i<inout.NumFrames(); i++)
		inout.Pose(i).m_aTranslations[0]*=ratio;
}

void MotionUtil::translate(Motion& inout, const vector3& trans, int start, int end)
{
	//!< Motion 전체를 Translate한다. 즉 root position의 translation
	if(start<0) start=0;
	if(end>inout.NumFrames()) end=inout.NumFrames();

	for(int i=start; i<end; i++)
	{
		inout.Pose(i).m_aTranslations[0]+=trans;	
	}
}

void MotionUtil::rotate(Motion& inout, const quater& q, int start, int end )
{
	//!< MotionData 전체를 Rotate한다. root orientation quat앞에 곱해져서 제일 나중에 적용된다.
	if(start<0) start=0;
	if(end>inout.NumFrames()) end=inout.NumFrames();

	for(int i=start; i<end; i++)
	{
		inout.Pose(i).m_aRotations[0].leftMult(q);
		inout.Pose(i).m_aTranslations[0].rotate(q, inout.Pose(i).m_aTranslations[0]);
	}
}


void MotionUtil::timewarping(Motion& out, const Motion& srcMotion, const vectorn& timewarpFunction)
{
	// timewarp
	out.InitEmpty(srcMotion, timewarpFunction.size());

	for(int i=0; i<timewarpFunction.size(); i++)
	{
		// float 0.5 가 정확하게 integer 0에 mapping된다.
		// 즉 0.6등은 0과 1을 0에 가중치를 크게 둬서 섞은게 된다.
		// world좌표와 pixel좌표에 쓰는 float, integer converting scheme을 따랐다.

		srcMotion.SamplePose(out.Pose(i), timewarpFunction[i]);
	}
}

void MotionUtil::timewarpingLinear(Motion& out, const Motion& srcMotion, const vectorn& timewarpFunction)
{
	// timewarp
	out.InitEmpty(srcMotion, timewarpFunction.size());

	for(int i=0; i<timewarpFunction.size(); i++)
	{
		srcMotion.samplePose(out.Pose(i), timewarpFunction[i]);
	}
}

Motion* MotionUtil::untimewarpingLinear(const Motion& srcMotion, const vectorn& invtmwpFunction)
{
	Motion* out =new Motion();
	m_real minTime=invtmwpFunction[0];
	m_real maxTime=invtmwpFunction[invtmwpFunction.size()-1];

	ASSERT(srcMotion.NumFrames()==invtmwpFunction.size());
	ASSERT(isSimilar(minTime,0.0));

	// 동작 길이가 정수가 되도록 rounding한다. 1을 더하는 이유는, 정수로 바꾸면 연속된 동작세그먼트간 1프레임 오버랩이 생기기 때문.
	int nDesiredLen=ROUND(maxTime)+1;

	vectorn invTmwpFunction(invtmwpFunction);
	invTmwpFunction*=1.0/maxTime*((m_real)nDesiredLen-1.0);
	
	ASSERT(nDesiredLen>=1);
	// timewarp
	out->InitEmpty(srcMotion, nDesiredLen);

#ifdef BUGGY
	int start, end;
	m_real criticalTime1;
	m_real criticalTime2;

	for(int i=0; i<invTmwpFunction.size()-1; i++)
	{
		criticalTime1=invTmwpFunction[i];
		criticalTime2=invTmwpFunction[i+1];
		
		start=ROUND(criticalTime1);
		end=ROUND(criticalTime2);

		int End;
		if(i==invTmwpFunction.size()-2)
			End=end+1;
		else 
			End=end;
        for(int j=start; j<End; j++)
		{
			m_real frac=((m_real)j-criticalTime1)/(criticalTime2-criticalTime1);
			//   0< frac+i <srcMotion.size()-1
			srcMotion.samplePose(out->Pose(j), frac+i);
		}


	}

#ifdef _DEBUG

	for(int i=0; i<out->NumFrames(); i++)
		ASSERT(out->Pose(i).m_numJoint);
#endif
#else
#endif
	int curInterval=0;
	for(int i=0; i<nDesiredLen; i++)
	{
		m_real t=m_real (i);
		int j=curInterval+1;
		for(; j<invTmwpFunction.size(); j++)
		{
			if(invTmwpFunction[j]>=t-FERR)
				break;
		}

		curInterval=j-1;

		m_real frac=((m_real)i-invTmwpFunction[curInterval])/(invTmwpFunction[j]-invTmwpFunction[curInterval]);

		srcMotion.samplePose(out->Pose(i), m_real(curInterval)+frac);
		

	}

	return out;
}

void MotionUtil::timewarping(Motion& inout, int playEnd, int criticalTimeBefore, int criticalTimeAfter)
{	
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
	Motion temp2;
	vectorn timewarpFunction(criticalTimeAfter-playEnd);
	timewarpFunction.op0(v0::uniformSampling((float)playEnd,(float)(criticalTimeBefore)));
	MotionUtil::timewarping(temp2, inout, timewarpFunction);

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
}

MotionUtil::SegmentFinder::SegmentFinder(const Motion& motion, int startFrame, int endFrame)
{
	if(startFrame<0) startFrame=0;
	if(endFrame>motion.NumFrames()) endFrame=motion.NumFrames();

	m_vStart.pushBack(startFrame);
	for(int i=startFrame+1; i<endFrame; i++)
	{
		if(motion.IsDiscontinuous(i))
		{
			m_vEnd.pushBack(i);
			m_vStart.pushBack(i);
		}
	}
	m_vEnd.pushBack(endFrame);
	ASSERT(m_vStart.size()==m_vEnd.size());
}

void GetSignal::offsetQ(matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	out.setSize(end-start, 4);

	// dv, drot
	for(int i=start; i<end; i++)
	{
		out.row(i-start)=m_Motion.Pose(i).m_offset_q;
	}
}

void MotionUtil::SetSignal::offsetQ(matrixn const& in, int start)
{
	int end=m_Motion.NumFrames();

	if(end>in.rows()+start)
		end=in.rows()+start;

	// dv, drot
	for(int i=start; i<end; i++)
	{
		m_Motion.Pose(i).m_offset_q=in.row(i-start).toQuater(0);
	}
}

void MotionUtil::GetSignal::interframeDifference(matrixn& out, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	// dv(2), dq(1), height (1), offset(4) total 8 dimensions

	out.setSize(end-start, 8);

	// dv, drot
	for(int i=start; i<end; i++)
	{
		out[i-start][0]=m_Motion.Pose(i).m_dv.x;
		out[i-start][1]=m_Motion.Pose(i).m_dv.z;
		out[i-start][2]=m_Motion.Pose(i).m_dq.rotationAngle(vector3(0,1,0));
		out[i-start][3]=m_Motion.Pose(i).m_offset_y;
		out.row(i-start).setQuater(4, m_Motion.Pose(i).m_offset_q);
	}
}

void MotionUtil::SetSignal::interframeDifference(matrixn const& in, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
		end=m_Motion.NumFrames();

	if(end>in.rows()+start)
		end=in.rows()+start;
	// dv, drot
	for(int i=start; i<end; i++)
	{
		m_Motion.Pose(i).m_dv.x=in[i-start][0];
		m_Motion.Pose(i).m_dv.z=in[i-start][1];
		m_Motion.Pose(i).m_dq.setRotation(vector3(0,1,0), in[i-start][2]);
		m_Motion.Pose(i).m_offset_y=in[i-start][3];		
		m_Motion.Pose(i).m_offset_q=in.row(i-start).toQuater(4);
	}
}

void MotionUtil::SetSignal::constraintPositions(matrixn const& in, int start, int end)
{
	if(end>m_Motion.NumFrames()) 
	end=m_Motion.NumFrames();

	if(end>in.rows()+start)
		end=in.rows()+start;

	for(int i=start; i<end; i++)
	{
		m_Motion.Pose(i).m_conToeL=in.row(i-start).toVector3(0);
		m_Motion.Pose(i).m_conToeR=in.row(i-start).toVector3(3);
	}
}


