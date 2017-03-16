// BVHLoader.cpp: implementation of the BVHLoader class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "BVHLoader.h"
#include "../utility/util.h"
#include "../baselib/utility/Parser.h"
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

BVHTransform::BVHTransform(BVHType eType)
:Bone()
{
	m_eType=ToBoneType(eType);
	m_offset.setValue(0,0,0);
};

BVHTransform::~BVHTransform()
{
};


Bone::BoneType	BVHTransform::ToBoneType(BVHType type)
{
	switch(type)
	{
	case BVH_DUMMY:
		return Bone::DUMMY;
	case BVH_ROOT:
		return Bone::ROOT;
	case BVH_JOINT:
		return Bone::JOINT;
	case BVH_END:
		return Bone::END;
	}
	ASSERT(0);
	return Bone::DUMMY;
}

BVHTransform::BVHType BVHTransform::CheckNodeType(Parser* file)
{
	TString token;
	BVHType eType;
	token=file->getToken();
	token.makeUpper();

	if(token=="ROOT")
		eType=BVH_ROOT;
	else if(token=="JOINT")
		eType=BVH_JOINT;
	else if(token=="END")
		eType=BVH_END;
	else if(token=="}")
		eType=BVH_NO_MORE_NODE;
	else ASSERT(0);
	return eType;
}

void BVHTransform::Unpack(Parser* file)
{
	TString token;
	token=file->getToken();
	NameId=new char[token.length()+1];
	strcpy(NameId,token.ptr());
	token=file->getToken();
	ASSERT(token=="{");
	
	token=file->getToken();	
	ASSERT(token.toUpper()=="OFFSET");

	m_offset.x=(float)atof(file->getToken());
	m_offset.y=(float)atof(file->getToken());
	m_offset.z=(float)atof(file->getToken());

	// initialize matrix
	m_matRot.setIdentityRot();
	m_matRot.setTranslation(m_offset);
	m_matRotOrig=m_matRot;

	token=file->getToken();
	token.makeUpper();
	if(token=="}") return;
	if(token=="CHANNELS")
	{
		m_numChannel=atoi(file->getToken());
		m_aeChannels=new int[m_numChannel];
		for(int i=0; i<m_numChannel; i++)
		{
			token=file->getToken().toUpper();

			if(token=="XPOSITION")
				m_aeChannels[i]=XPosition;
			else if(token=="YPOSITION")
				m_aeChannels[i]=YPosition;
			else if(token=="ZPOSITION")
				m_aeChannels[i]=ZPosition;
			else if(token=="ZROTATION")
				m_aeChannels[i]=ZRotation;
			else if(token=="XROTATION")
				m_aeChannels[i]=XRotation;
			else if(token=="YROTATION")
				m_aeChannels[i]=YRotation;
		}
	}

	BVHType eType;
	// Recursively load every child nodes
	while((eType=CheckNodeType(file))!=BVH_NO_MORE_NODE)
	{
		BVHTransform* pchild=new BVHTransform(eType);
		AddChild(pchild);
		pchild->Unpack(file);
	}
}

BVHIP::BVHIP()
{
	m_aaKeyvalue=NULL;
}

BVHIP::~BVHIP()
{
	if(m_aaKeyvalue)
	{
		for(int i=0; i<m_numFrames; i++)
			delete[] m_aaKeyvalue[i];
		delete[] m_aaKeyvalue;
	}
}

void BVHIP::Unpack(Parser* file)
{
	// Load Motion
	TString token;
	token = file->getToken();
	if(token.toUpper()=="MOTION")
		token=file->getToken();
	ASSERT(token.toUpper()=="FRAMES:");
	m_numFrames=atoi(file->getToken());
	token=file->getToken();
	token=file->getToken();
	ASSERT(token.toUpper()=="TIME:");
	m_fFrameTime=(float)atof(file->getToken());

	m_aaKeyvalue= new m_real*[m_numFrames];
	for(int i=0; i<m_numFrames; i++)
	{
		m_aaKeyvalue[i]=new m_real[m_numChannel];
		for( int j=0; j<m_numChannel; j++)
		{
			m_aaKeyvalue[i][j]=atof(file->getToken());
		}
	}
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

BVHLoader::BVHLoader(const char* filename, const char* option)
:MotionLoader()
{
	std::cout << "start of bvh" << std::endl;
	Parser file(filename);
	
	
	VERIFY(file.getToken().toUpper()=="HIERARCHY");

	m_pTreeRoot=new BVHTransform(BVHTransform::BVH_DUMMY);	// dummy root를 달아 놓는다. 이는 나중에 모델을 자유롭게 translate또는 rotate하는데 사용될수 있다.
	m_pTreeRoot->NameId=new char[6];
	strcpy(m_pTreeRoot->NameId,"DUMMY");
	
	VERIFY(BVHTransform::CheckNodeType(&file)==BVHTransform::BVH_ROOT);
	m_pTreeRoot->AddChild(new BVHTransform(BVHTransform::BVH_ROOT));

	// Tree 생성 
	((BVHTransform*)(m_pTreeRoot->m_pChildHead))->Unpack(&file);

	int numChannel;
	// Count Total Number of channels, total Number of END node
	MakeBoneArrayFromTree(numChannel);	

	if(!option || TString("loadSkeletonOnly")!=option)
	{
		BVHIP cBVHIP;
		cBVHIP.m_numChannel=numChannel;
		cBVHIP.Unpack(&file);
		MakePositionIPfromBVHIP(m_cPostureIP, cBVHIP);
		
		VERIFY(file.getToken()=="");		// NULL까지 읽어야 file이 닫힌다.
	}
}

int BVHLoader::countTotalChannels() const
{
	int numChannel=0;
	for(int i=0; i<GetNumTreeNode(); i++)
	{
		Bone& bone=getBoneByTreeIndex(i);
		if(bone.m_numChannel)
			numChannel+=bone.m_numChannel;
	}
	return numChannel;
}

void BVHLoader::loadAnimation(Motion& mot, const char* filename) const
{
	TRACE("Loading animation %s\n", filename);

	TString fn=filename;
	if(fn.right(4).toUpper()==".BVH") 
	{
		Parser file(filename);
		TString token;
		while(1)
		{
			token = file.getToken();
			VERIFY(token.length());
			if(token.toUpper()=="MOTION")
				break;
		}
		

		BVHIP cBVHIP;
		cBVHIP.m_numChannel=countTotalChannels() ;
		cBVHIP.Unpack(&file);
		((BVHLoader*)this)->MakePositionIPfromBVHIP(mot, cBVHIP);
		
		VERIFY(file.getToken()=="");		// NULL까지 읽어야 file이 닫힌다.
	}
	else
		MotionLoader::loadAnimation(mot, filename);
}

void BVHLoader::MakePositionIPfromBVHIP(Motion& mot, const BVHIP& cBVHIP)
{
	Bone* pBone;
	
	mot.InitSkeleton(this);
	mot._Init(cBVHIP.m_numFrames, GetNumRotJoint(), 1, cBVHIP.m_fFrameTime);
	m_real aValue[3];

	TString channels;
	for(int iframe=0; iframe<cBVHIP.m_numFrames; iframe++)
	{
		int currChannel=0;
		int ijoint=0;
		pBone=&getBoneByRotJointIndex(ijoint);
		mot.Pose(iframe).m_aTranslations[0].x=cBVHIP.m_aaKeyvalue[iframe][0];
		mot.Pose(iframe).m_aTranslations[0].y=cBVHIP.m_aaKeyvalue[iframe][1];
		mot.Pose(iframe).m_aTranslations[0].z=cBVHIP.m_aaKeyvalue[iframe][2];
		// ( channel startIndex, channel value, quaternion)
		
		
		channels=pBone->getRotationalChannels();
		for(int c=0; c<channels.length(); c++)
			aValue[c]=TO_RADIAN(cBVHIP.m_aaKeyvalue[iframe][3+c]);

		mot.Pose(iframe).m_aRotations[ijoint].setRotation(channels, aValue);

		currChannel+=pBone->m_numChannel;

		for(ijoint=1; ijoint<GetNumRotJoint(); ijoint++)
		{
			pBone=&getBoneByRotJointIndex(ijoint);


			channels=pBone->getRotationalChannels();
			for(int c=0; c<channels.length(); c++)
				aValue[c]=TO_RADIAN(cBVHIP.m_aaKeyvalue[iframe][currChannel+c]);

			mot.Pose(iframe).m_aRotations[ijoint].setRotation(channels, aValue);
			currChannel+=pBone->m_numChannel;
		}
	}
//	SetTargetIndex(&mot);
}

BVHLoader::~BVHLoader()
{
}

