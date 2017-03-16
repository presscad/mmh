#include "stdafx.h"
#include "TRCLoader.h"
#include "motionutil.h"
#include "../utility/util.h"
#include "../baselib/utility/Parser.h"
#include "../baselib/utility/operatorString.h"
#include "fullbodyik.h"

#define	SCALE	3.0
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TRCTransform::TRCTransform(BoneType eType)
:Bone()
{
	m_eType=eType;
	parentMarkerIndex=-1;
	markerIndex=-1;
	bTranslational=true;
};

TRCTransform::~TRCTransform()
{
};

TRCIP::TRCIP()
{
}

TRCIP::~TRCIP()
{
}

void TRCIP::Unpack(Parser* file, bool isTranspose)
{
	// Frame rate (sec/frame)
	m_fFrameTime=1.0f/((float)atof(file->getToken()));
	file->getToken();	// skip camerarate

	// Number of Frame
	m_numFrames=(float)atoi(file->getToken());
	cout << m_numFrames << endl;

	// Number of Markers
	m_numMarkers=(float)atoi(file->getToken());
	file->getToken();
	file->getToken();
	file->getToken();
	file->getToken();

	VERIFY(file->getToken()=="Frame#");
	VERIFY(file->getToken()=="Time");

	// skip marker names
	for(int i=0; i < m_numMarkers; i++)
		file->getToken();
	// skip marker coordinate
	for(int i=0; i < m_numMarkers*3; i++){
		file->getToken();
	}

	m_aaKeyvalue.resize(m_numMarkers);
	
	for(int i=0; i<m_numMarkers; i++)
	{
		m_aaKeyvalue[i].setSize(m_numFrames,3);
	}
	
	for(int i=0; i<m_numFrames; i++)
	{
		file->getToken(); // skip Frame#
		file->getToken(); // skip Time

		if(isTranspose == false){
			// x,y,z를 z,x,y로 축을 바꿔 파싱 TRC는 z축이 up방향이라 가정
			for( int j=0; j<m_numMarkers; j++)
			{	
				m_aaKeyvalue[j][i][2]=atof(file->getToken())/SCALE;	// scale factor 1/3을 곱해준다.
				m_aaKeyvalue[j][i][0]=atof(file->getToken())/SCALE;	// scale factor 1/3을 곱해준다.
				m_aaKeyvalue[j][i][1]=atof(file->getToken())/SCALE;	// scale factor 1/3을 곱해준다.
			}
		}
		else{
			// x,y,z를 x,z,y로 축을 바꿔 파싱 TRC는 z축이 up방향이라 가정
			for( int j=0; j<m_numMarkers; j++)
			{	
				m_aaKeyvalue[j][i][0]=atof(file->getToken())/SCALE;	// scale factor 1/3을 곱해준다.
				m_aaKeyvalue[j][i][2]=atof(file->getToken())/SCALE;	// scale factor 1/3을 곱해준다.
				m_aaKeyvalue[j][i][1]=atof(file->getToken())/SCALE;	// scale factor 1/3을 곱해준다.
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

void TRCLoader::setValue(TRCTransform *ptr, char *name)
{
	ptr->m_numChannel=3;
	ptr->m_aeChannels=new int[3];
	ptr->m_aeChannels[0]=Bone::XPosition;
	ptr->m_aeChannels[1]=Bone::YPosition;
	ptr->m_aeChannels[2]=Bone::ZPosition;
	ptr->SetNameId(name);
}

void TRCLoader::createTranslationalJoint(int parentMarkerIndex, int markerIndex, const char* tx, const char* rx)
{
	TRCTransform* node;
	if(parentMarkerIndex==-1)
		node=new TRCTransform(Bone::ROOT);		
	else
		node=new TRCTransform(Bone::JOINT);
	if(parentMarkerIndex==-1)
		node->SetNameId(sz1::format("marker_%d", markerIndex));
	else
		node->SetNameId(sz1::format("marker_%d_%d", parentMarkerIndex, markerIndex));
	node->bTranslational=true;
	node->parentMarkerIndex=parentMarkerIndex;
	node->markerIndex=markerIndex;
	node->setChannels(tx,rx);
	_temporaryNodes.push_back(node);
}

void TRCLoader::createRotationalJoint2(int parentMarkerIndex, int markerIndex, vector3 offsetDir, const char* axis, const char* axis2)
{
	TRCTransform* node=new TRCTransform(Bone::JOINT);
	node->SetNameId(sz1::format("connector_%d_%d", parentMarkerIndex, markerIndex));
	node->bTranslational=false;
	node->vOffsetDir=vector3(0,0,0);
	// initialize matrix
	node->m_matRot.setIdentityRot();
	node->m_matRot.setTranslation(node->vOffsetDir);
	node->m_matRotOrig=node->m_matRot;

	node->parentMarkerIndex=parentMarkerIndex;
	node->markerIndex=parentMarkerIndex;
	node->setChannels("",axis);
	_temporaryNodes.push_back(node);

	createRotationalJoint(parentMarkerIndex, markerIndex, offsetDir, axis2);
}

void TRCLoader::createRotationalJoint(int parentMarkerIndex, int markerIndex, vector3 offsetDir, const char* axis)
{
	TRCTransform* node=new TRCTransform(Bone::JOINT);
	node->SetNameId(sz1::format("marker_%d_%d", parentMarkerIndex, markerIndex));
	node->bTranslational=false;
	node->vOffsetDir=offsetDir;

	m_real avgLen=0;
	for(int i=0; i<cTRCIP.m_numFrames; i++)
	{
		vector3 diff;
		diff.difference(cTRCIP.m_aaKeyvalue[parentMarkerIndex].row3(0), 
			cTRCIP.m_aaKeyvalue[markerIndex].row3(0));

		avgLen+=diff.length();
	}
	avgLen/=cTRCIP.m_numFrames;

	node->vOffsetDir.normalize();
	node->vOffsetDir*=avgLen;
	// initialize matrix
	node->m_matRot.setIdentityRot();
	node->m_matRot.setTranslation(node->vOffsetDir);
	node->m_matRotOrig=node->m_matRot;

	node->parentMarkerIndex=parentMarkerIndex;
	node->markerIndex=markerIndex;
	node->setChannels("",axis);
	_temporaryNodes.push_back(node);
}

void TRCLoader::createEffector(int parentMarkerIndex, int markerIndex, vector3 offsetDir)
{
	createRotationalJoint(parentMarkerIndex, markerIndex, offsetDir, "");
	
	_temporaryNodes.back()->m_eType=Bone::END;	
}

void TRCLoader::buildHierarchy(TRCIP& cTRCIP)
{
	std::vector<TRCTransform*> markerToBone;
	int numMarkers=cTRCIP.m_aaKeyvalue.size();
	markerToBone.resize(numMarkers);

	for(int i=0; i<numMarkers; i++)		markerToBone[i]=NULL;

	ASSERT(_temporaryNodes[0]->parentMarkerIndex==-1);
	m_pTreeRoot->AddChild(_temporaryNodes[0]);
	markerToBone[_temporaryNodes[0]->markerIndex]=_temporaryNodes[0];

	for(int i=1; i<_temporaryNodes.size(); i++)
	{
		TRCTransform* child=_temporaryNodes[i];
		TRCTransform* parent=markerToBone[child->parentMarkerIndex];

		ASSERT(parent);
		parent->AddChild(child);

		// connector인경우 자식도 달아준다.
		if(child->parentMarkerIndex==child->markerIndex)
		{
			parent=child;
			i++;
			child=_temporaryNodes[i];
			parent->AddChild(child);
		}

		// 덮어 쓰지 않는다. 따라서 marker에 해당하는 본이 여러개인경우 가장 먼저 나온놈이 parent 가 된다.
		if(markerToBone[_temporaryNodes[i]->markerIndex]==NULL)
			markerToBone[_temporaryNodes[i]->markerIndex]=_temporaryNodes[i];
	}


	for(int i=1; i<_temporaryNodes.size(); i++)
	{
		TRCTransform* parent=_temporaryNodes[i];

		// End로 끝나지 않는 경우 END를 추가해준다.
		if(!parent->m_pChildHead && parent->m_eType!=Bone::END)
		{
			// SITE
			parent->AddChild(new TRCTransform(Bone::END));
		}
	}
}

TRCLoader::TRCLoader(const char* filename, const char* _option)
:MotionLoader()
{
	Parser file(filename);

	while(file.getToken() != "DataRate"){}
	VERIFY(file.getToken()=="CameraRate");
	VERIFY(file.getToken()=="NumFrames");
	VERIFY(file.getToken()=="NumMarkers");
	VERIFY(file.getToken()=="Units");
	VERIFY(file.getToken()=="OrigDataRate");
	VERIFY(file.getToken()=="OrigDataStartFrame");
	VERIFY(file.getToken()=="OrigNumFrames");

	m_pTreeRoot=new TRCTransform(Bone::DUMMY);	// dummy root를 달아 놓는다. 이는 나중에 모델을 자유롭게 translate또는 rotate하는데 사용될수 있다.
	m_pTreeRoot->NameId=new char[6];
	strcpy(m_pTreeRoot->NameId,"DUMMY");

	bool isTranspose=false;
	cTRCIP.Unpack(&file, isTranspose);

	
	
	TString option=_option;

	if(option=="USE_TRANSLATION_JOINT")
	{
	
		createTranslationalJoint(-1, 3, "XYZ", "ZXY");
		// head
		createTranslationalJoint(3, 2, "XYZ", "");
		createTranslationalJoint(2, 1, "XYZ", "");
		createTranslationalJoint(1, 0, "XYZ", "");

		// left arm
		createTranslationalJoint(2, 9, "XYZ", "");
		createTranslationalJoint(9, 10, "XYZ", "");
		createTranslationalJoint(10, 11, "XYZ", "");

		// right arm
		createTranslationalJoint(2, 12, "XYZ", "");
		createTranslationalJoint(12, 13, "XYZ", "");
		createTranslationalJoint(13, 14, "XYZ", "");

		// tail
		createTranslationalJoint(3, 4, "XYZ", "");
		createTranslationalJoint(4, 15, "XYZ", "");
		createTranslationalJoint(15, 16, "XYZ", "");
		createTranslationalJoint(16, 17, "XYZ", "");
		createTranslationalJoint(17, 18, "XYZ", "");

		// left leg
		createTranslationalJoint(4, 5, "XYZ", "");
		createTranslationalJoint(5, 6, "XYZ", "");
		createTranslationalJoint(6, 7, "XYZ", "");
		createTranslationalJoint(7, 8, "XYZ", "");
		
		// right leg
		createTranslationalJoint(4, 19, "XYZ", "");
		createTranslationalJoint(19, 20, "XYZ", "");
		createTranslationalJoint(20, 21, "XYZ", "");
		createTranslationalJoint(21, 22, "XYZ", "");
	}
	else
	{

		//       z-axis
		// x-axis  .0 (head)
		//         .1 
		// .  .  . .2 .  .  . 
		// 14 13 12.3 9 10 11 
		// . . . . .4  .  .  .  .
		//19202122 .5 15 16 17 18
		//         .6 
		//         .7
		//         .8 (tail)
		//
		// head
		vector3 unitz(0,0,1);
		vector3 unitx(1,0,0);

		createTranslationalJoint(-1, 3, "XYZ", "XY");

		createRotationalJoint(3, 2, unitz, "XY");
		createRotationalJoint(2, 1, unitz, "XY");
		createEffector(1, 0, unitz);

		// left arm
		createRotationalJoint2(2, 12, unitx, "YZ", "YZX");
		createRotationalJoint(12, 13, unitx, "Z");
		createEffector(13, 14, unitx);
		
		// right arm
		createRotationalJoint2(2, 9, unitx*-1, "YZ", "YZX");
		createRotationalJoint(9, 10, unitx*-1, "Z");
		createEffector(10, 11, unitx*-1);

		// body
		createRotationalJoint2(3, 4, unitz*-1, "XY", "XY");
		// tail		
		createRotationalJoint(4, 5, unitz*-1, "XY");
		createRotationalJoint(5, 6, unitz*-1, "XY");
		createRotationalJoint(6, 7, unitz*-1, "XY");
		createEffector(7, 8, unitz*-1);
		
		// left leg
		createRotationalJoint2(4, 19, unitx, "YZ", "YZX");
		//createRotationalJoint(4, 19, unitx, "YZX");
		createRotationalJoint(19, 20, unitx, "Z");
		createRotationalJoint(20, 21, unitx, "YZX");
		createEffector(21, 22, unitx);

		// right leg
		createRotationalJoint2(4, 15, unitx*-1,"YZ", "YZX");
		//createRotationalJoint(4, 15, unitx*-1, "YZX");
		createRotationalJoint(15, 16, unitx*-1, "Z");
		createRotationalJoint(16, 17, unitx*-1, "YZX");
		createEffector(17, 18, unitx*-1);		
	}
	buildHierarchy(cTRCIP);

	// Count Total Number of channels, total Number of END node
	MakeBoneArrayFromTree(cTRCIP.m_numChannel);	

	getBoneByRotJointIndex(0).printHierarchy();

	_treeIndexToMarkerIndex.init(m_apNode.size());
	for(int i=1; i<m_apNode.size(); i++)
	{
		TRCTransform* bone=(TRCTransform*)m_apNode[i];
		if(bone->markerIndex!=-1 && !bone->isConnector())
			_treeIndexToMarkerIndex.map(i, bone->markerIndex);		
	}

	//_treeIndexToMarkerIndex
	// Tree 생성 - 실행 순서 바꿈 by cluster
	//((TRCTransform*)(m_pTreeRoot->m_pChildHead))->Unpack(&file);

	MakePositionIPfromTRCIP(cTRCIP, isTranspose);
	
	VERIFY(file.getToken()=="");		// NULL까지 읽어야 file이 닫힌다.
}

void TRCLoader::MakePositionIPfromTRCIP(const TRCIP& cTRCIP, bool isTranspose)
{
	Bone* pBone;
	
	m_cPostureIP.InitSkeleton(this);
	m_cPostureIP._Init(cTRCIP.m_numFrames, GetNumRotJoint(), GetNumTransJoint(), cTRCIP.m_fFrameTime);
	m_real aValue[3];

	if(m_nNumRotNode>1)
	{
		// initialize
		for(int iframe=0; iframe<cTRCIP.m_numFrames; iframe++)
		{
			int currChannel=0;
			int ijoint=0;
			pBone=&getBoneByRotJointIndex(ijoint);

			vector3 root=cTRCIP.m_aaKeyvalue[_treeIndexToMarkerIndex(1)].row3(iframe);
			vector3 childHead=cTRCIP.m_aaKeyvalue[_treeIndexToMarkerIndex(2)].row3(iframe);
			m_cPostureIP.Pose(iframe).m_aTranslations[0]=root;

			// 최소한 root orientation의 초기값은 주는것이 IK를 잘 풀리게 한다.
			vector3 vecTarget;
			vecTarget.difference(root, childHead);
			m_cPostureIP.Pose(iframe).m_aRotations[0].setAxisRotation(vector3(0,1,0), vector3(0,0,1), vecTarget);
			for(int j=1; j<m_cPostureIP.NumJoints(); j++)
				m_cPostureIP.Pose(iframe).m_aRotations[j].identity();
		}


		// define effectors(all markers except the root marker)
		std::vector<MotionUtil::Effector> effector;
		std::vector<TRCTransform*> effector_target;
		effector.reserve(cTRCIP.m_numMarkers);
		
		
		for(int i=0; i<cTRCIP.m_numMarkers; i++)
		{
			int treeIndex=_treeIndexToMarkerIndex.inverse(i);
			if(treeIndex!=-1)
			{
				TRCTransform* bone=(TRCTransform*)&getBoneByTreeIndex(treeIndex);
				if(bone->m_eType==Bone::JOINT || bone->m_eType==Bone::END)
				//if(bone->m_eType==Bone::JOINT && bone->m_pChildHead && bone->child()->m_eType==Bone::END)
				{
					effector.resize(effector.size()+1);
					effector.back().bone=(Bone*)(bone->m_pParent);
					effector.back().localpos=bone->vOffsetDir;
					effector_target.push_back(bone);
				}
			}
		}

		// perform IK

		MotionUtil::FullbodyIK* ik=MotionUtil::createFullbodyIk_MultiTarget(*this, effector);

		intvectorn joint_index;
		quaterN delta_rot;
		vector3N conPos;
		conPos.setSize(effector.size());

#ifdef _DEBUG
		for(int i=0; i<20; i++)
#else
		for(int i=0; i<cTRCIP.m_numFrames; i++)
		//for(int i=0; i<20; i++)
#endif
		{
			Msg::print2("Performing IK (frame %d)", i);
			for(int e=0; e<effector.size(); e++)
			{
				conPos[e]=cTRCIP.m_aaKeyvalue[effector_target[e]->markerIndex].row3(i);
			}
			
			ik->IKsolve(m_cPostureIP.Pose(i), conPos, joint_index, delta_rot);

			for(int j=0; j<joint_index.size();j++)
			{
				m_cPostureIP.Pose(i).m_aRotations[joint_index[j]].leftMult(delta_rot[j]);
			}
		}

		delete ik;
	}
	else if(m_nNumRotNode==1)
	{
		for(int iframe=0; iframe<cTRCIP.m_numFrames; iframe++)
		{
			int currChannel=0;
			int ijoint=0;
			pBone=&getBoneByRotJointIndex(ijoint);

			// root orientation만 있고 그외는 모두 translation.
			// ( channel startIndex, channel value, quaternion)
			m_real fx,fy,fz; // root front orientation

			// iguana의 marker3이 루트, marker2에서 marker3를 가리키는 벡터가 루트의 orientation
			vector3 rootPosition(cTRCIP.m_aaKeyvalue[3].row3(iframe));
			vector3 frontVector;
			frontVector.difference(rootPosition, cTRCIP.m_aaKeyvalue[2].row3(iframe));

			MotionUtil::Coordinate coor;
			coor.setCoordinate(rootPosition,frontVector); 

			// root translation
			m_cPostureIP.Pose(iframe).m_aTranslations[0]=coor.mOrigin;
			m_cPostureIP.Pose(iframe).m_aRotations[0]=coor.mOrientation;

			// marker translation
			for(int j=0; j<cTRCIP.m_numMarkers; j++)
			{
				int treeIndex=_treeIndexToMarkerIndex.inverse(j);

				int k=m_cPostureIP.skeleton().getTransJointIndexFromTreeIndex(treeIndex);
				// l은 부모 조인트의 인덱스.

				if(k==0) continue;

				int parentIndex=m_cPostureIP.skeleton().getBoneByTreeIndex(treeIndex).m_pParent->GetIndex();
				int l=m_cPostureIP.skeleton().getTransJointIndexFromTreeIndex(parentIndex);

				vector3 parentPos(cTRCIP.m_aaKeyvalue[_treeIndexToMarkerIndex(parentIndex)].row3(iframe));
				coor.setCoordinate(parentPos,frontVector);

				vector3 curPos(cTRCIP.m_aaKeyvalue[_treeIndexToMarkerIndex(treeIndex)].row3(iframe));
				m_cPostureIP.Pose(iframe).m_aTranslations[k]=coor.toLocalPos(curPos);
			}
		}
	}
	else
		ASSERT(0);
}

TRCLoader::~TRCLoader()
{
}
