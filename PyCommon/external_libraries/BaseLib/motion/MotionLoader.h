#pragma once

#include "node.h"
#include "ModelLoader.h"
#include "modelLoader.h"
#include "motion.h"
#include "../utility/nametable.h"

class Bone : public Node
{
public:
	enum BoneType { DUMMY, ROOT,JOINT,END } m_eType;
	Bone ();
	virtual ~Bone();

	inline Bone* child() const		{ return (Bone*)m_pChildHead;}
	inline Bone* sibling() const	{ return (Bone*)m_pSibling;}
	inline Bone* parent() const		{ return (Bone*)m_pParent;}
	inline int treeIndex() const	{ return GetIndex();}

	//! CHANNEL (loading 과 관련)
	enum { XPosition, YPosition, ZPosition, ZRotation, XRotation, YRotation};
	int m_numChannel;
	int *m_aeChannels;

	/// e.g. setChannels("XYZ", "ZXY")
	void setChannels(const char* translation_axis, const char* rotation_axis);
	/// e.g. "ZXY"=getRotationalChannels()
	TString getRotationalChannels() const;
	TString getTranslationalChannels() const;

	m_real length() const;
	void getOffset(vector3& offset) const;
	void getTranslation(vector3& trans) const;
	vector3 getTranslation() const;
	void getRotation(quater& rot) const;

	matrix4 m_matRotOrig;	//!< translation후 rotation하는 matrix, initial posture를 저장한다. (BVH 파일에서는 translation만 갖고 있다.)
	matrix4 m_matRot;		//!< translation후 rotation하는 matrix, MotionLoader::SetPose에서 사용된다.
	matrix4 m_matCombined;	//!< MotionLoader::SetPose에서 사용된다. global rotation하고 translation이 저장되어 있다.

	virtual void pack(BinaryFile& bf, int version) const;
	virtual void unpack(BinaryFile& bf, int version);

	void packBVH(FILE* file, int level, MotionLoader* pLoader);



};

/**
 * \ingroup group_motion
 *
 * 동작의 skeleton을 로딩하고 tree 자료구조를 갖는다. 로딩한 동작 데이타는 m_cPostureIP에 저장한다.
 *
 * Bone은 하나의 transformation매트릭스에 해당하고, 하나의 본은 rotationalJoint와 translationJoint에 동시에 해당할수도 있고 둘중 하나에만 해당할 수 있다. 물론 joint가 전혀 연결되지 않을수도 있다. 이경우 이 트랜스폼 매트릭스는 constant가 된다.
 */

class MotionLoader : public ModelLoader 
{
	TFactory<Posture>* m_pFactory;
public:
	MotionLoader();

	// filename: "~.skl" or "~.mot"  (.mot인경우 m_cPostureIP도 만들어진다.)
	// option=="loadSkeletonOnly" or NULL
	MotionLoader(const char* filename, const char* option=NULL);
	virtual ~MotionLoader();

	// exportSkeleton("~.skl");
	void exportSkeleton(const char* filename) const;

	// loadAnimation("~.anim" or "~.mot"), 상속한 클래스에서는 "~.bvh" 나 "~.amc"를 읽을 때도 사용된다.
	virtual void loadAnimation(Motion& mot, const char* filename) const;

	void ReadJointIndex(const char* filename);

	int GetNumRotJoint() const;	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
	int GetNumTransJoint() const;
	int GetNumBone() const; //!< 

	void setPose(const Posture& pose);
	// 모든 트리를 update하지 않고, 한개의 chain만 update한다.
	void setChain(const Posture& pose, int ijoint);

	Motion m_cPostureIP;	//!< file에서 읽어진 일반적인 모션 

	// Posture대신 상속한 클래스를 사용하고 싶은경우 호출.
	void changeFactory(TFactory<Posture>* pFactory)	;
	const TFactory<Posture>* factory() const	{ return m_pFactory;}
	///////////////////////////////////////////////////////
	// 뼈대 편집
	///////////////////////////////////////////////////////
	/// child의 transform은 identity가 되고, bMoveChildren==true일경우 parent의 children은 모두 새로운 차일드에 붙는다.
	void insertChildBone(Bone& parent, const char* nameId, bool bMoveChildren=true);	
	/// type="T" (translational-아직 구현안됨) or "R" (rotational) or "RT" (both)
	void insertJoint(Bone& target, const char* type);
	///////////////////////////////////////////////////////
	// 내용 받아오기.
	///////////////////////////////////////////////////////


	Bone& getBoneByTreeIndex(int index)	const;
	Bone& getBoneByRotJointIndex(int iRotJoint)	const;
	Bone& getBoneByTransJointIndex(int iTransJoint)	const;
	Bone& getBoneByVoca(int jointVoca)	const;
	Bone& getBoneByName(const char*) const;

	int getTreeIndexByName(const char* name) const					{ return GetIndex(name);}
	int getTreeIndexFromRotJointIndex(int rotjointIndex) const		{ return m_aRotJoint2TreeIndex[rotjointIndex];};
	int getTreeIndexFromTransJointIndex(int transjointIndex) const  { return m_aTransJoint2TreeIndex[transjointIndex];}
	int getTreeIndexFromVoca(int jointVoca) const					{ return m_aRotJoint2TreeIndex[m_aVoca2RotJointIndex[jointVoca]];}

	// slow because of full search.
	int getRotJointIndexByName(const char* nameID) const;
	int getRotJointIndexFromTreeIndex(int treeIndex) const;
	int getRotJointIndexFromVoca(int jointVoca) const;
	int getJointVocaFromRotJointIndex(int rotjointIndex) const;

	int getTransJointIndexByName(const char* nameID);
	int getTransJointIndexFromTreeIndex(int treeIndex) const;

	void UpdateInitialBone();	//!<참조 동작의 global joint포지션 구하기 위해서 사용된다.


	void Scale(float fScale);
	void scale(float fScale, Motion& mot);


	// almost private
	void _changeJointIndex(int jointVoca, int newJointIndex)	{ m_aVoca2RotJointIndex[jointVoca]=newJointIndex;}
	
	// 아래는 deprecated..
	// 주의 : NodeIndex와 JointIndex는 다르다. GetTreeIndex를 하면 tree node의 index가 나오고, getRotJointIndexFromTreeIndex를 하면 joint node의 index가 나온다.
	// joint node는 tree node중 모션 캡쳐 데이타와 연결된 노드로, 순서를 따로 매긴것이다. 즉 PostureIP의 m_aRotations의 인덱스이다..
	
	

	/*
	

	vector3 pos(int rotjoint) const					{ vector3 trans; getBoneByRotJointIndex(rotjoint).getTranslation(trans); return trans;}
	vector3 pos(Bone& bone) const					{ vector3 trans; bone.getTranslation(trans); return trans;}
*/

	// translation table (joint vocaburary)
	enum { HIPS, LEFTHIP , LEFTKNEE, LEFTANKLE, RIGHTHIP, RIGHTKNEE, RIGHTANKLE, CHEST, CHEST2, LEFTCOLLAR, LEFTSHOULDER, LEFTELBOW, LEFTWRIST, RIGHTCOLLAR, RIGHTSHOULDER, RIGHTELBOW, RIGHTWRIST, NECK, HEAD, NUM_JOINT_VOCA};
	
	

	void pack(BinaryFile & bf, int nVersion) const;	

	int getHandle() const							{ return m_handle;}


	// translation table (joint vocaburary)
	NameTable m_translationTable;

protected:
	
	int unpack(BinaryFile & bf) ;

	// Motion Manager specific
	friend class MotionManager;
	int m_handle;

	void UpdateBone();	//!< global joint포지션 구하기 위해서 사용된다.

	// tree는 있지만, node array가 없는 경우 또는 node array도 있기는 하지만 순서가 DFS순임이 보장되지 않는 경우 call해준다.
	void MakeBoneArrayFromTree(int& numChannel);


	//////////////////////////////////////////////////////////////////
	// 아래 내용은 모두 MakeBoneArrayFromTree수행시 계산됨
	//////////////////////////////////////////////////////////////////

	// Posture의 Rotational joint관련 
	intvectorn m_aRotJoint2TreeIndex;	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index
	intvectorn m_aTree2RotJointIndex;	//!< m_aTree2RotJointIndex[tree node index]=joint index
	intvectorn m_aVoca2RotJointIndex;		//!< m_aVoca2RotJointIndex[voca]=joint index

	// Posture의 Translational joint관련 
	intvectorn m_aTransJoint2TreeIndex;
	intvectorn m_aTree2TransJointIndex;

	// 종류별 노드 개수. 
	int m_nNumRotNode;
	int m_nNumTransNode;
	int m_nNumEndNode;	//!< EndNode(site)도 모션과 연결되어있지 않음.
	
};


// deprecated.
int dep_GetParentJoint(MotionLoader const&, int rotjointIndex) ;
Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint) ;	// ToeBone
Bone& dep_GetSiteBone(MotionLoader const& ml, int ijoint)	;
Bone& dep_GetSiteBoneVoca(MotionLoader const& ml, int jointVoca) ;
void dep_MakeQuaternionFromChannel(Bone& bone, int channelStartIndex, m_real *aValue, quater& q, bool bRightToLeft=false);
void dep_makeEulerAngleFromChannel(Bone& bone, int channelStartIndex, quater const& q, m_real *aValue, bool bRightToLeft=false);
