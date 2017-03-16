// stdafx.h : 잘 변경되지 않고 자주 사용하는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이 
// 들어 있는 포함 파일입니다.

#include "stdafx.h"


#include "utility/stdtemplate.h"
#include "utility/configtable.h"
#include "image/imageclass.h"
#include "utility/typestring.h"
#include "utility/garray.h"
#include "motion/motion.h"
#include "motion/motionloader.h"
#include "utility/tword.h"

#include "math/optimize.h"

#include "MATHCLASS/mathclass.h"
#include "PmQm/pm.h"

#include "ConvertMotionToPmQm.h"




PmMaskType ConvertMotionToPmQm::enumToMask(int _enum)
{
	return MaskBit(_enum) ;
}
/*
		switch(_enum)
		{
			case 0: return  PM_MASK_PELVIS;
			case 1: return  PM_MASK_SPINE_1;
			case 2: return  PM_MASK_SPINE_2;
			case 3: return  PM_MASK_SPINE_3;
			case 4: return  PM_MASK_SPINE_4;
			case 5: return  PM_MASK_CHEST;
			case 6: return  PM_MASK_NECK;
			case 7: return  PM_MASK_HEAD;
			case 8: return  PM_MASK_RIGHT_SHOULDER;
			case 9: return  PM_MASK_LEFT_SHOULDER;
			case 10: return PM_MASK_RIGHT_COLLAR;
			case 11: return PM_MASK_LEFT_COLLAR;
			case 12: return PM_MASK_UPPER_RIGHT_ARM;
			case 13: return PM_MASK_UPPER_LEFT_ARM;
			case 14: return PM_MASK_LOWER_RIGHT_ARM;
			case 15: return PM_MASK_LOWER_LEFT_ARM;
			case 16: return PM_MASK_UPPER_RIGHT_LEG;
			case 17: return PM_MASK_UPPER_LEFT_LEG;
			case 18: return PM_MASK_LOWER_RIGHT_LEG;
			case 19: return PM_MASK_LOWER_LEFT_LEG;
			case 20: return PM_MASK_RIGHT_FOOT;
			case 21: return PM_MASK_LEFT_FOOT;
			case 22: return PM_MASK_RIGHT_TOE;
			case 23: return PM_MASK_LEFT_TOE;
			case 24: return PM_MASK_RIGHT_PALM;
			case 25: return PM_MASK_LEFT_PALM;
			case 26: return PM_MASK_RIGHT_HEEL;
			case 27: return PM_MASK_LEFT_HEEL;
			case 28: return PM_MASK_RIGHT_FINGER_11;
			case 29: return PM_MASK_RIGHT_FINGER_12;
			case 30: return PM_MASK_RIGHT_FINGER_13;
			case 31: return PM_MASK_RIGHT_FINGER_21;
			case 32: return PM_MASK_RIGHT_FINGER_22;
			case 33: return PM_MASK_RIGHT_FINGER_23;
			case 34: return PM_MASK_RIGHT_FINGER_31;
			case 35: return PM_MASK_RIGHT_FINGER_32;
			case 36: return PM_MASK_RIGHT_FINGER_33;
			case 37: return PM_MASK_RIGHT_FINGER_41;
			case 38: return PM_MASK_RIGHT_FINGER_42;
			case 39: return PM_MASK_RIGHT_FINGER_43;
			case 40: return PM_MASK_RIGHT_FINGER_51;
			case 41: return PM_MASK_RIGHT_FINGER_52;
			case 42: return PM_MASK_RIGHT_FINGER_53;
			case 43: return PM_MASK_LEFT_FINGER_11;
			case 44: return PM_MASK_LEFT_FINGER_12;
			case 45: return PM_MASK_LEFT_FINGER_13;
			case 46: return PM_MASK_LEFT_FINGER_21;
			case 47: return PM_MASK_LEFT_FINGER_22;
			case 48: return PM_MASK_LEFT_FINGER_23;
			case 49: return PM_MASK_LEFT_FINGER_31;
			case 50: return PM_MASK_LEFT_FINGER_32;
			case 51: return PM_MASK_LEFT_FINGER_33;
			case 52: return PM_MASK_LEFT_FINGER_41;
			case 53: return PM_MASK_LEFT_FINGER_42;
			case 54: return PM_MASK_LEFT_FINGER_43;
			case 55: return PM_MASK_LEFT_FINGER_51;
			case 56: return PM_MASK_LEFT_FINGER_52;
			case 57: return PM_MASK_LEFT_FINGER_53;
		}
		return PM_MASK_SCALE;
	}
*/

	ConvertMotionToPmQm::ConvertMotionToPmQm(MotionLoader const& skel):mSkeleton(skel)
	{
		NUM_ENUM=PM_HUMAN_NUM_LINKS ;
		mHuman=new PmHuman();
		createPmHuman(mHuman);
	}

	int ConvertMotionToPmQm::_searchUnusedEnum()
	{
		for(int i=0; i<NUM_ENUM; i++)
		{
			bool bUsed=false;
			for(int j=0; j<treeIndex2enum.size(); j++)
			{
				if(i==treeIndex2enum[j])
					bUsed=true;
			}

			if(!bUsed)
				return i;
		}

		ASSERT(0);
		return PmHuman::UNDEFINED;
	}

	void ConvertMotionToPmQm_setMap(ConvertMotionToPmQm* a, int treeIndex, int _enum)
	{
#ifdef _DEBUG
		ASSERT(a->treeIndex2enum[treeIndex]==PmHuman::UNDEFINED);

		for(int i=0; i<a->treeIndex2enum.size(); i++)
			ASSERT(a->treeIndex2enum[i]!=_enum);
#endif
		a->treeIndex2enum[treeIndex]=_enum;
	}
	void ConvertMotionToPmQm::createPmHuman(PmHuman* newHuman)
	{
 		const MotionLoader& skeleton=mSkeleton;

		int numTreeNode=skeleton.GetNumTreeNode();
		treeIndex2enum.resize(numTreeNode);

		for(int i=0; i<numTreeNode; i++)
		{
			treeIndex2enum[i]=PmHuman::UNDEFINED;
		}

		// LEFTHIP , LEFTKNEE, LEFTANKLE, 
		// RIGHTHIP, RIGHTKNEE, RIGHTANKLE, 
		// LEFTSHOULDER, LEFTELBOW, LEFTWRIST, 
		// RIGHTSHOULDER, RIGHTELBOW, RIGHTWRIST

		ASSERT(skeleton.getTreeIndexFromVoca(MotionLoader::HIPS)==1);

		ConvertMotionToPmQm_setMap(this, 1,PmHuman::PELVIS);

		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::LEFTHIP),PmHuman::UPPER_LEFT_LEG);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::LEFTKNEE),PmHuman::LOWER_LEFT_LEG);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::LEFTANKLE),PmHuman::LEFT_FOOT);

		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::RIGHTHIP),PmHuman::UPPER_RIGHT_LEG);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::RIGHTKNEE),PmHuman::LOWER_RIGHT_LEG);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::RIGHTANKLE),PmHuman::RIGHT_FOOT);
		
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::LEFTSHOULDER),PmHuman::UPPER_LEFT_ARM);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::LEFTELBOW),PmHuman::LOWER_LEFT_ARM);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::LEFTWRIST),PmHuman::LEFT_PALM);

		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::RIGHTSHOULDER),PmHuman::UPPER_RIGHT_ARM);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::RIGHTELBOW),PmHuman::LOWER_RIGHT_ARM);
		ConvertMotionToPmQm_setMap(this, skeleton.getTreeIndexFromVoca(MotionLoader::RIGHTWRIST),PmHuman::RIGHT_PALM);

		// set spine
		Bone& neck=skeleton.getBoneByVoca(MotionLoader::NECK);
		
		ConvertMotionToPmQm_setMap(this, neck.treeIndex(),PmHuman::NECK);

		int count=0;
		
		Bone* bone=&neck;
		Bone* pelvis=&skeleton.getBoneByVoca(MotionLoader::HIPS);

		while(bone!=pelvis)
		{
			bone=bone->parent();
			count++;
		}

		if(count==4)
		{
			bone=neck.parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_3);
			bone=bone->parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_2);
			bone=bone->parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_1);
			ASSERT(bone->parent()==pelvis);
		}
		else if(count==5)
		{
			bone=neck.parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::CHEST);
			bone=bone->parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_3);
			bone=bone->parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_2);
			bone=bone->parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_1);
			ASSERT(bone->parent()==pelvis);
		}
		else if(count==3)
		{
			bone=neck.parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_2);
			bone=bone->parent();
			ConvertMotionToPmQm_setMap(this, bone->treeIndex(),PmHuman::SPINE_1);
			ASSERT(bone->parent()==pelvis);
		}
		else
		{
			ASSERT(0);
			// 아직 구현안됨.			
		}

		// arms
		for(int i=0; i<2; i++)
		{
			Bone* shoulder;
			if(i==1)
				shoulder=&skeleton.getBoneByVoca(MotionLoader::LEFTSHOULDER);
			else
				shoulder=&skeleton.getBoneByVoca(MotionLoader::RIGHTSHOULDER);
			
			int icollar=shoulder->parent()->treeIndex();

			if(treeIndex2enum[icollar]==PmHuman::UNDEFINED)
			{
				if(i==1)
					ConvertMotionToPmQm_setMap(this, icollar,PmHuman::LEFT_COLLAR);
				else
					ConvertMotionToPmQm_setMap(this, icollar,PmHuman::RIGHT_COLLAR);
			}
		}

		for(int i=1; i<numTreeNode; i++)
		{
			if(treeIndex2enum[i]==PmHuman::UNDEFINED && skeleton.getBoneByTreeIndex(i).child())
			{
				ConvertMotionToPmQm_setMap(this, i, _searchUnusedEnum());
			}
		}

		newHuman->num_links=0;
		for(int i=1; i<numTreeNode; i++)
		{
			if(treeIndex2enum[i]!=PmHuman::UNDEFINED )
				newHuman->num_links++;
		}

		newHuman->mask=PM_MASK_NULL;

		for(int i=1; i<numTreeNode; i++)
		{
			Bone& bone=skeleton.getBoneByTreeIndex(i);

			if(bone.child()==NULL) continue;	// site본은 추가하지 않는다.

			ASSERT(treeIndex2enum[i]!=PmHuman::UNDEFINED);

			// set mask
			int e=treeIndex2enum[i];
			newHuman->mask |= enumToMask(e);

			// set parent_list
			if(i==1)
			{
				newHuman->parent_list[treeIndex2enum[i]]=-1;
			}
			else
			{
				int parentTreeIndex=bone.parent() ->treeIndex();
				newHuman->parent_list[treeIndex2enum[i]]=treeIndex2enum[parentTreeIndex];
			}

			// set base_transf
			vector3 offset;
			bone.getOffset(offset);
			newHuman->base_transf[treeIndex2enum[i]]=jhm::transf(ToJHM(quater(1,0,0,0)), ToJHM(offset));
		}

#ifdef _DEBUG
		for ( int i = 1; i<PM_HUMAN_NUM_LINKS; i++ )
		{
			//std::cout << "sd" << ' ' << newMotion->getMask() << ' ' <<MaskBit(i) << std::endl;
			if ( newHuman->getMask() & MaskBit(i) )
			{
				ASSERT(newHuman->getParent(i)>=0);
			}
		}
#endif
	}	

	void ConvertMotionToPmQm::setPmPosture(Posture const& pose, PmPosture& posture)
	{
		PmHuman* body=getBody();
		posture.setBody(body);
		posture.setMask(body->mask);
		posture.setTranslation(ToJHM(pose.m_aTranslations[0]));

		for(int i=1; i<getSkeleton().GetNumTreeNode(); i++)
		{
			if(treeIndex2enum[i]==PmHuman::UNDEFINED) continue;

			int rotJoint=getSkeleton().getRotJointIndexFromTreeIndex(i);
			if(rotJoint==-1)
				posture.setRotation(treeIndex2enum[i], jhm::quater(1,0,0,0));
			else
				posture.setRotation(treeIndex2enum[i], ToJHM(pose.m_aRotations[rotJoint]));
		}
	}

	void ConvertMotionToPmQm::getPmPosture(PmPosture const& posture, Posture & pose)
	{		
		pose.Init(getSkeleton().GetNumRotJoint(), getSkeleton().GetNumTransJoint());

		pose.m_aTranslations[0]=ToBase(posture.getTranslation());

		for(int i=0; i<pose.numRotJoint(); i++)
			pose.m_aRotations[i]=ToBase(posture.getRotation(treeIndex2enum[getSkeleton().getTreeIndexFromRotJointIndex(i)]));
	}

	// motion already has a body (PmHuman)
	void ConvertMotionToPmQm::setPmLinearMotion(Motion const& mot, PmLinearMotion& motion)
	{
		ASSERT(&((const MotionLoader&)mot.skeleton())==&mSkeleton);

		motion.setBody(getBody());
		motion.setSize(mot.NumFrames());

		for(int i=0; i<mot.NumFrames(); i++)
		{
			setPmPosture(mot.Pose(i), motion.getPosture(i));
		}
	}
	void ConvertMotionToPmQm::getPmLinearMotion(PmLinearMotion const& motion, Motion & mot)
	{
		ASSERT(&((const MotionLoader&)mot.skeleton())==&mSkeleton);

		mot.InitEmpty((MotionLoader*)&getSkeleton(), motion.getSize());
		
		for(int i=0; i<mot.NumFrames(); i++)
		{
			getPmPosture(motion.getPosture(i), mot.Pose(i));
		}
	}
