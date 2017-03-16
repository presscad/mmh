#include "stdafx.h"
#include "../baselib/math/mathclass.h"
#include "FullbodyIK.h"
#include "motion/motion.h"
#include "motion/motionutil.h"
#include "motion/motionloader.h"
#include "motion/iksolver.h"

using namespace MotionUtil;

Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint) ;

void FullbodyIK::IKsolve(Posture& pose, vector3N const& con)
{
	static intvectorn joint_index;
	static quaterN delta_rot;
	IKsolve(pose, con, joint_index, delta_rot);
	for(int j=0; j<joint_index.size();j++)
		pose.m_aRotations[joint_index[j]].leftMult(delta_rot[j]);
}
int _GetConFromBone(MotionLoader const& ml, Bone* bone) 
{
	int aCon[]={CONSTRAINT_LEFT_HEEL, CONSTRAINT_RIGHT_HEEL, CONSTRAINT_LEFT_TOE, CONSTRAINT_RIGHT_TOE,
		CONSTRAINT_LEFT_HAND,CONSTRAINT_RIGHT_HAND,CONSTRAINT_LEFT_FINGERTIP,CONSTRAINT_RIGHT_FINGERTIP};
	
	for(int i=0; i<8; i++)
	{
		if(&dep_GetBoneFromCon(ml, aCon[i])== bone)
			return aCon[i];
	}

	return NUM_CONSTRAINT;
}


class FullbodyIK_limbik: public FullbodyIK
{
	std::vector<Effector> mEffectors;
	MotionLoader& mSkeleton;
public:
	IKSolver ik;
	FullbodyIK_limbik(MotionLoader& skeleton, std::vector<Effector>& effectors):mSkeleton(skeleton),mEffectors(effectors){}
	virtual~FullbodyIK_limbik(){}

	virtual void IKsolve(Posture const& pose, vector3N const& con, intvectorn & index, quaterN& delta_rot)
	{
		intvectorn index2;
		quaterN delta_rot2;
		index.reserve(mEffectors.size()*3);
		delta_rot.reserve(mEffectors.size()*3);
		index.setSize(0);
		delta_rot.setSize(0);
		
		for(int i=0; i<mEffectors.size(); i++)
		{
			int eCon=_GetConFromBone(mSkeleton, mEffectors[i].bone);

			if(eCon==NUM_CONSTRAINT) continue;

			mSkeleton.setPose(pose);
			
			ik.limbIK(mSkeleton, eCon, con[i], index2, delta_rot2);
			
			for(int id=1; id<4; id++)
			{
				index.push_back(index2[id]);
				delta_rot.pushBack(delta_rot2[id]);
			}
		}
	}
};

#include "IK_sdls/Node.h"
#include "IK_sdls/Tree.h"
#include "IK_sdls/Jacobian.h"

VectorR3 ToIKSLDS(vector3 const& v)
{
	return VectorR3(v.x, v.y, v.z);
}
vector3 ToBase(VectorR3 const& v)
{
	return vector3(v.x, v.y, v.z);
}

extern VectorR3 target[];

class FullbodyIK_MultiTarget: public FullbodyIK
{
	std::vector<Effector> mEffectors;
	MotionLoader& mSkeleton;

	
	IK_sdls::Tree mTree;
	IK_sdls::Jacobian *mJacob;

	struct NodeWrap
	{
		NodeWrap() {for(int i=0; i<3; i++) node[i]=NULL; }
		IK_sdls::Node* node[3];
		TString axes;
		IK_sdls::Node* back()
		{
			return axes.length()?node[axes.length()-1]:NULL;
		}
		void createEffector(Bone* _bone, vector3 localPos)
		{
			bone=_bone;
			axes="EFFECTOR";
			VectorR3 trans=ToIKSLDS(bone->getTranslation()+localPos);
			node[0]=new IK_sdls::Node(trans, VectorR3(0,0,0), 0.08, EFFECTOR);
		}
		void createNodes(Bone* _bone, const char* _axes)
		{
			bone=_bone;
			axes=_axes;
			VectorR3 trans=ToIKSLDS(bone->getTranslation());

			ASSERT(axes.length()<=3);
			VectorR3 unitx (1,0,0);
			VectorR3 unity (0,1,0);
			VectorR3 unitz (0,0,1);
			VectorR3 ax;
			for(int i=0, len=axes.length(); i<len; i++)
			{

				if(axes[i]=='Z')
					ax=unitz;
				else if(axes[i]=='Y')
					ax=unity;
				else
					ax=unitx;

				node[i]=new IK_sdls::Node(trans, ax, 0.08, JOINT);
			}
		}
		Bone* bone;
	};

	std::vector<NodeWrap> mNode;
	vector3 mRootPos;
	bitvectorn mEffectorAttached;
	intvectorn mBoneToNode;
public:
	void copyTree(Bone* bone, IK_sdls::Node* parent)
	{
		if(bone->m_eType==Bone::END)
		{
			return;
		}
		else if(mEffectorAttached[bone->GetIndex()])
		{
			if(parent==NULL)
			{
				mRootPos=bone->getTranslation();
				mNode.push_back(NodeWrap());

				TString channels=bone->getRotationalChannels();
				mNode.back().createNodes(bone, channels);
				mTree.InsertRoot(mNode.back().node[0]);
				for(int c=1; c<channels.length(); c++)
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
				mBoneToNode[bone->GetIndex()]=mNode.size()-1;
			}
			else
			{
				mNode.push_back(NodeWrap());
				TString channels=bone->getRotationalChannels();
				mNode.back().createNodes(bone, channels);

				mTree.InsertChild_automatic(parent, mNode.back().node[0]);
				for(int c=1; c<channels.length(); c++)
					mTree.InsertLeftChild(mNode.back().node[c-1], mNode.back().node[c]);
				mBoneToNode[bone->GetIndex()]=mNode.size()-1;
			}


			if(bone->m_pChildHead)
			{
				copyTree((Bone*)bone->m_pChildHead, mNode[mBoneToNode[bone->GetIndex()]].back());
			}
		}

		if(bone->m_pSibling)
		{
			copyTree((Bone*)bone->m_pSibling, mNode[mBoneToNode[bone->m_pParent->GetIndex()]].back());
		}
	}

	FullbodyIK_MultiTarget(MotionLoader& skeleton, std::vector<Effector>& effectors):mSkeleton(skeleton),mEffectors(effectors)
	{
		skeleton.UpdateInitialBone();
		
		/*
		아래 주석 처리된 코드와 유사한 내용이 리커시브하게 수행된다.
		// pelvis.
		bone=&skeleton.getBoneByVoca(MotionLoader::HIPS);

		mRootPos=bone->getTranslation();
		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "ZXY");
		mTree.InsertRoot(mNode.back().node[0]);
		mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
		mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
		parent=mNode.back().node[2];

		// righthip
		bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTHIP);
		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "ZXY");
		mTree.InsertLeftChild(parent, mNode.back().node[0]);
		mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
		mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
		parent=mNode.back().node[2];

		// knee
		bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTKNEE);
		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "ZXY");
		mTree.InsertLeftChild(parent, mNode.back().node[0]);
		mTree.InsertLeftChild(mNode.back().node[0], mNode.back().node[1]);
		mTree.InsertLeftChild(mNode.back().node[1], mNode.back().node[2]);
		parent=mNode.back().node[2];

		// ankle
		bone=&skeleton.getBoneByVoca(MotionLoader::RIGHTANKLE);

		mNode.push_back(NodeWrap());
		mNode.back().createNodes(bone, "EFFECTOR");
		mTree.InsertLeftChild(parent, mNode.back().node[0]);
	
		*/

		// Effector랑 관련이 없는 bone은 IK 트리만들때 제외하기 위해, 먼저 각 본이 effector와 관련이 되있는지 아닌지를 mEffectorAttached에 저장한다.
		mEffectorAttached.setSize(skeleton.GetNumTreeNode());
		mBoneToNode.setSize(skeleton.GetNumTreeNode());
		mBoneToNode.setAllValue(-1);
		mEffectorAttached.clearAll();

		for(int i=0; i<effectors.size(); i++)
		{
			for(Node* bone=effectors[i].bone; bone!=NULL; bone=bone->m_pParent)
				mEffectorAttached.setAt(bone->GetIndex());
		}

		Bone* bone;
		VectorR3 zero (0,0,0);

		copyTree(&skeleton.getBoneByRotJointIndex(0), NULL);

		for(int i=0; i<mEffectors.size(); i++)
		{
			bone=mEffectors[i].bone;

			mNode.push_back(NodeWrap());
			mNode.back().createEffector(bone, mEffectors[i].localpos);

			mTree.InsertChild_automatic(mNode[mBoneToNode[bone->GetIndex()]].back(), mNode.back().node[0]);
		}

		mTree.Init();
		mTree.Compute();
	
#ifdef _DEBUG
		mTree.Print();
		compareTrees(vector3(0,0,0));
#endif

		mJacob = new IK_sdls::Jacobian(&mTree);
		mJacob->Reset();


	}


	virtual~FullbodyIK_MultiTarget(){ delete mJacob;}

	void compareTrees(vector3 trans)
	{
		for(int i=0; i<mNode.size(); i++)
		{
			printf("bone %s (%d)\n", mNode[i].bone->NameId, i);
			
			if(mNode[i].node[0]->IsJoint())
			{
				printf("node %d:%d:%s\n", mNode[i].node[0]->GetJointNum(), mNode[i].node[0]->GetParentJointNum(), (ToBase(mNode[i].node[0]->GetS())+trans).output().ptr());
				printf("NODE %d:%d:%s\n", mNode[i].back()->GetJointNum(), mNode[i].back()->GetParentJointNum(), (ToBase(mNode[i].back()->GetS())+trans).output().ptr());
			}
			else
			{
				printf("efct %d:%d:%s\n", mNode[i].node[0]->GetEffectorNum(),mNode[i].node[0]->GetParentJointNum(), (ToBase(mNode[i].node[0]->GetS())+trans).output().ptr());
			}
			printf("bone %d:%s\n", mNode[i].bone->GetIndex(), mNode[i].bone->getTranslation().output().ptr());
		}
	}

	virtual void IKsolve(Posture const& pose, vector3N const& con, intvectorn & index, quaterN& delta_rot)
	{
		ASSERT(pose.numTransJoint()==1);
		// set tree. (position, orientation) : quaternion->euler

		index.reserve(mNode.size()-con.size());
		index.setSize(0);// excluding EFFECTOR
		delta_rot.reserve(mNode.size()-con.size());
		delta_rot.setSize(0);

		for(int i=0; i<mNode.size(); i++)
		{
			Bone* bone=mNode[i].bone;
			int treeIndex=bone->GetIndex();

			if(mNode[i].node[0]->IsJoint())
			{				
				index.push_back(mSkeleton.getRotJointIndexFromTreeIndex(treeIndex));
				m_real euler[3];
				TString channels=bone->getRotationalChannels();
				pose.m_aRotations[index.back()].getRotation(channels, euler);

				for(int c=0; c<channels.length(); c++)
					mNode[i].node[c]->SetTheta(euler[c]);
			}
		}

		ASSERT(index.size()==mNode.size()-con.size());
		
		mTree.Compute();
		
		

#ifdef _DEBUG
		//mTree.Print();
		mSkeleton.setPose(pose);

		compareTrees(pose.m_aTranslations[0]);
#endif
		// set target.
		for(int i=0; i<con.size(); i++)
		{
			target[i]=ToIKSLDS(con[i]-pose.m_aTranslations[0]+mRootPos);
		}

		mJacob->SetJendActive();

		double prevCost=DBL_MAX;
		for(int iter=0; iter<1000; iter++)
		{			
			mJacob->ComputeJacobian();

			// over-determined case에 대해서는 SDLS가 동작하지 않음.
			if(mJacob->ActiveJacobian().GetNumRows() > mJacob->ActiveJacobian().GetNumColumns() )
			//	mJacob->CalcDeltaThetasTranspose();		// Jacobian transpose method
				mJacob->CalcDeltaThetasDLS();			// Damped least squares method
			//	mJacob->CalcDeltaThetasPseudoinverse();	// Pure pseudoinverse method
			else
				mJacob->CalcDeltaThetasSDLS();			// Selectively damped least squares method

			//mJacob->CalcDeltaThetasTranspose();		// Jacobian transpose method
			//mJacob->CalcDeltaThetasDLS();			// Damped least squares method
						
			mJacob->UpdateThetas();						// Apply the change in the theta values
			mJacob->UpdatedSClampValue();

			if(iter%10==0)
			{
				// evaluate IK results.
				m_real cost=0;

		
				ASSERT(mEffectors.size()==con.size());
				for(int i=0; i<mEffectors.size(); i++)
				{
					IK_sdls::Node* node=mNode[mBoneToNode[mEffectors[i].bone->GetIndex()]].back();
					double dist=(node->GetS()-target[i]).Norm();
					cost+=dist;		
				}
#ifdef _DEBUG
				Msg::print2("%d:%f", iter, cost);
#endif

				if(ABS(prevCost-cost)<0.000001) break;	// 수렴하면 멈춘다.
				prevCost=cost;
			}
		}

		// set delta rot : euler->quaternion
		for(int i=0; i<mNode.size(); i++)
		{
			m_real euler[3];
			if(mNode[i].node[0]->IsJoint())
			{
				quater q;
				

				TString channels=mNode[i].bone->getRotationalChannels();

				for(int c=0; c<channels.length(); c++)
				{
					euler[c]=mNode[i].node[c]->GetTheta();
				}

				q.setRotation(channels, euler);

				q.difference(pose.m_aRotations[index[delta_rot.size()]], q);				

				delta_rot.pushBack(q);
			}
		}
	}
};













FullbodyIK* MotionUtil::createFullbodyIk_LimbIK(MotionLoader& skeleton, std::vector<Effector>& effectors)
{
	return new FullbodyIK_limbik(skeleton, effectors);
}


FullbodyIK* MotionUtil::createFullbodyIk_MultiTarget(MotionLoader& skeleton, std::vector<Effector>& effectors)
{
	return new FullbodyIK_MultiTarget(skeleton, effectors);
}
