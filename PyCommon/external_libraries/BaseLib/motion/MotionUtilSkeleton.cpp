#include "stdafx.h"
#include "motion.h"
#include "motionloader.h"
#include "motionutil.h"
#include "motionutilskeleton.h"
#include "../baselib/math/operator.h"
#include "../math/OperatorSignal.h"


using namespace MotionUtil;

void MotionUtil::insertCOMjoint(MotionLoader& skel, m_real kernelSize, m_real kernelQuat)
{
	Motion& srcMot=skel.m_cPostureIP;
	PhysicalHuman ph(srcMot);
	matrixn aCOM;
	matrixn aRoot;
	ph.COM(aCOM);

	MotionUtil::GetSignal gs(srcMot);
	MotionUtil::GetSignal ss(srcMot);

	gs.joint(0, aRoot);

	vectorn kernel;
	Filter::GetGaussFilter(srcMot.KernelSize(kernelSize), kernel);
	vectorn kernelQ;
	Filter::GetGaussFilter(srcMot.KernelSize(kernelQuat), kernelQ);

	MotionUtil::SegmentFinder seg(srcMot);
	for(int iseg=0; iseg<seg.numSegment(); iseg++)
	{
		int segStart=seg.startFrame(iseg);
		int segEnd=seg.endFrame(iseg);
		
		// smooth COM and ROOT orientations.
		aCOM.range(segStart, segEnd).op0(m0::useUnary(m1::filter(kernel)));
		Filter::LTIFilterQuat(1, kernelQ, aRoot.range(segStart, segEnd));
	}

	insertRootJoint(skel, aCOM, aRoot, "COM");

	
}

void MotionUtil::insertRootJoint(MotionLoader& skel, matrixn const& aRootPos, matrixn const& aRoot, const char* rootNameID)
{
	Motion& srcMot=skel.m_cPostureIP;

	Bone& bone=skel.getBoneByVoca(MotionLoader::HIPS);
	skel.insertChildBone(bone, "hips2", true);
	skel.insertJoint(skel.getBoneByName("hips2"), "RT");

	quater q0, q1;

	for(int i=0; i<srcMot.NumFrames(); i++)
	{
		q0=srcMot.Pose(i).m_aRotations[0];
		q1=aRoot.row(i).toQuater();		
		srcMot.Pose(i).m_aTranslations[1].difference(aRootPos.row3(i), srcMot.Pose(i).m_aTranslations[0]);
		srcMot.Pose(i).m_aTranslations[0]=aRootPos.row3(i);
		srcMot.Pose(i).m_aTranslations[1].rotate(q1.inverse());

		srcMot.Pose(i).m_aRotations[1].mult(q1.inverse(), q0);		
		srcMot.Pose(i).m_aRotations[0]=q1;
	}

	skel._changeJointIndex(MotionLoader::HIPS, 1);
	skel.getBoneByRotJointIndex(0).SetNameId(rootNameID);
	skel.getBoneByRotJointIndex(1).SetNameId("hips");

}