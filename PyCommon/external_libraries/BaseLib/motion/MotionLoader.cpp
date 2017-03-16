// MotionLoader.cpp: implementation of the MotionLoader class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MotionLoader.h"
#include "../baselib/utility/ConfigTable.h"
#include "version.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
Bone::Bone()
: Node()
{
	NodeType=BONE;
	m_numChannel=0;
	m_aeChannels=NULL;
	m_matRotOrig.setIdentityRot();
}

Bone::~Bone()
{
	if(m_aeChannels) delete[] m_aeChannels;
	m_aeChannels=NULL;
}

void Bone::pack(BinaryFile& bf, int version) const
{
	Node::pack(bf, version);

	bf.packInt((int)m_eType);
	bf.packInt(m_numChannel);
	for(int i=0; i<m_numChannel; i++)
		bf.packInt(m_aeChannels[i]);

	bf.pack(m_matRotOrig);
	bf.pack(m_matRot);		//!< translation후 rotation하는 matrix, MotionLoader::SetPose에서 사용된다.
	bf.pack(m_matCombined);	//!< MotionLoader::SetPose에서 사용된다. global rotation하고 translation이 저장되어 있다.
}

void Bone::setChannels(const char* tx, const char* rx)
{
	TString taxis=tx;
	TString raxis=rx;

	m_numChannel=taxis.length()+raxis.length();
	m_aeChannels=new int[m_numChannel];

	for(int i=0; i<taxis.length(); i++)
	{
		switch(taxis[i])
		{
		case 'X':
			m_aeChannels[i]=Bone::XPosition;
			break;
		case 'Y':
			m_aeChannels[i]=Bone::YPosition;
			break;
		case 'Z':
			m_aeChannels[i]=Bone::ZPosition;
			break;
		default:
			ASSERT(0);
		}
	}

	for(int i=0; i<raxis.length(); i++)
	{
		switch(raxis[i])
		{
		case 'X':
			m_aeChannels[taxis.length()+i]=Bone::XRotation;
			break;
		case 'Y':
			m_aeChannels[taxis.length()+i]=Bone::YRotation;
			break;
		case 'Z':
			m_aeChannels[taxis.length()+i]=Bone::ZRotation;
			break;
		default:
			ASSERT(0);
		}
	}
}

TString Bone::getTranslationalChannels() const
{
	int startIndex=0;
	
	TString channels;
	channels.reserve(4);

	for(int i=startIndex; i<m_numChannel; i++)
	{
		switch(m_aeChannels[i])
		{
		case XPosition:
			channels.add("%c", 'X');
			break;
		case YPosition:
			channels.add("%c", 'Y');
			break;
		case ZPosition:
			channels.add("%c", 'Z');
			break;
		default:
			break;
		}
	}
	return channels;
}


TString Bone::getRotationalChannels() const
{
	if(m_numChannel==0) return TString("");

	int startIndex=0;
	while(m_aeChannels[startIndex]<=ZPosition) startIndex++;

	TString channels;
	channels.reserve(4);

	for(int i=startIndex; i<m_numChannel; i++)
	{
		switch(m_aeChannels[i])
		{
		case ZRotation:
			channels.add("%c", 'Z');
			break;
		case XRotation:
			channels.add("%c", 'X');
			break;
		case YRotation:
			channels.add("%c", 'Y');
			break;
		}
	}
	return channels;
}

void Bone::unpack(BinaryFile& bf, int version)
{
	Node::unpack(bf, version);

	m_eType=(BoneType)bf.unpackInt();
	m_numChannel=bf.unpackInt();
	m_aeChannels=new int[m_numChannel];
	for(int i=0; i<m_numChannel; i++)
		m_aeChannels[i]=bf.unpackInt();

	bf.unpack(m_matRotOrig);
	bf.unpack(m_matRot);		//!< translation후 rotation하는 matrix, MotionLoader::SetPose에서 사용된다.
	bf.unpack(m_matCombined);	//!< MotionLoader::SetPose에서 사용된다. global rotation하고 translation이 저장
}

void printNewline(FILE* file, int level)
{
	fprintf(file, "\n");
	for(int i=0; i<level; i++)
		fprintf(file, "  ");
}

void Bone::packBVH(FILE* file, int level, MotionLoader* pLoader)
{
	if(level==0)
		fprintf(file, "HIERARCHY\nROOT ");
	else 
	{
		printNewline(file, level);
		if(!m_pChildHead)
			fprintf(file, "End ");
		else
			fprintf(file, "JOINT ");
	}

/*
	TString NameId;
	if(!m_pChildHead)
		NameId="Site";
	else
	{
		int jointIndex=pLoader->getJointVocaFromRotJointIndex(pLoader->getRotJointIndexFromTreeIndex(pLoader->GetIndex(this)));
		if(jointIndex==-1)
			NameId="?";
		else
			NameId=pLoader->m_translationTable[jointIndex];
	}
*/	
	if(!m_pChildHead)
		fprintf(file, "Site");
	else
		fprintf(file, "%s", NameId);

	printNewline(file, level);

	vector3 offset;
	getOffset(offset);
	fprintf(file, "{");
	printNewline(file, level+1);
	fprintf(file, "OFFSET %f %f %f", offset.x, offset.y, offset.z);
	
	if(!m_pChildHead)
	{
		printNewline(file, level);
		fprintf(file, "}");		
	}
	else
	{
		printNewline(file, level+1);
		

		const bool bFullDOF=true;

		

		if(bFullDOF)
		{
			TString rc, tc;
			tc=getTranslationalChannels();
			rc=getRotationalChannels();

			int numChannels=tc.length()?3:0;
			numChannels+=rc.length()?3:0;
			fprintf(file, "CHANNELS %d ", numChannels);

			if(tc.length())
			{
				for(int i=0; i<tc.length(); i++)
				{
					switch(tc[i])
					{
					case 'X':
						fprintf(file, "Xposition ");
						break;
					case 'Y':
						fprintf(file, "Yposition ");
						break;
					case 'Z':
						fprintf(file, "Zposition ");
						break;
					}
				}
			}

			if(rc.length())
			{
				char aChannel[3];
				for(int i=0; i<rc.length(); i++)
					aChannel[i]=rc[i];

				//길이가 3보다 작으면 나머지 채널을 채워준다.
				for(int i=rc.length(); i<3; i++)
				{
					aChannel[i]='X';

					for(int k=0; k<2; k++)
					{
						for(int j=0; j<i; j++)
							if(aChannel[i]==aChannel[j]) aChannel[i]++;
					}
				}

				for(int i=0; i<3; i++)
				{
					switch(aChannel[i])
					{
						case 'Z':
						fprintf(file, "Zrotation ");
						break;
						case 'X':
						fprintf(file, "Xrotation ");
						break;
						case 'Y':
						fprintf(file, "Yrotation ");
						break;
					}
				}
			}
		}
		else
		{
			fprintf(file, "CHANNELS %d ", m_numChannel);
			for(int i=0; i<m_numChannel; i++)
			{
				switch(m_aeChannels[i])
				{
				case XPosition:
					fprintf(file, "Xposition ");
					break;
				case YPosition:
					fprintf(file, "Yposition ");
					break;
				case ZPosition:
					fprintf(file, "Zposition ");
					break;
				case ZRotation:
					fprintf(file, "Zrotation ");
					break;
				case XRotation:
					fprintf(file, "Xrotation ");
					break;
				case YRotation:
					fprintf(file, "Yrotation ");
					break;
				}
			}
		}

		for(Bone* node=(Bone*)m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
		{
			node->packBVH(file, level+1, pLoader);
		}

		printNewline(file, level);
		fprintf(file, "}");
	}
}

void dep_MakeQuaternionFromChannel(Bone& bone, int channelStartIndex, m_real *aValue, quater& q, bool bRightToLeft)
{
	char aChannel[4];
	
	for(int i=0; i<3; i++)
	{
		switch(bone.m_aeChannels[channelStartIndex+i])
		{
		case Bone::XRotation: aChannel[i]='X'; break;
		case Bone::YRotation: aChannel[i]='Y'; break;
		case Bone::ZRotation: aChannel[i]='Z'; break;
		}
	}

	aChannel[3]=0;
	q.setRotation(aChannel, aValue, bRightToLeft);	
}

void dep_makeEulerAngleFromChannel(Bone& bone, int channelStartIndex, quater const& q, m_real *aValue, bool bRightToLeft)
{

	char aChannel[4];

	for(int i=0; i<3; i++)
	{
		switch(bone.m_aeChannels[channelStartIndex+i])
		{
		case Bone::XRotation: aChannel[i]='X'; break;
		case Bone::YRotation: aChannel[i]='Y'; break;
		case Bone::ZRotation: aChannel[i]='Z'; break;
		}
	}
	aChannel[3]=0;

	q.getRotation(aChannel, aValue, bRightToLeft);

//	quater qtest;
//	MakeQuaternionFromChannel(channelStartIndex, aValue, qtest);
//	printf("error %f %s\n", TO_DEGREE(qtest.distance(q)), ((quater&)q).output().ptr());
}

m_real Bone::length() const
{
	vector3 trans;
	trans.translation(m_matRotOrig);
	return trans.length();
}

void Bone::getOffset(vector3& trans) const
{
	// mat rot original의 translation term만 뽑는다.
	trans.translation(m_matRotOrig);
}

void Bone::getTranslation(vector3& trans) const
{
	// mat combined의 translation term만 뽑는다.
	trans.translation(m_matCombined);
}

vector3 Bone::getTranslation() const
{
	vector3 v;
	getTranslation(v);
	return v;
}

void Bone::getRotation(quater& rot) const
{
	rot.setRotation(m_matCombined);
}

void _initTranslationTable(NameTable &m_translationTable)
{
	// 순서 중요. header와 같은 순으로 할 것.
	m_translationTable.Insert("HIPS");
	m_translationTable.Insert("LEFTHIP");
	m_translationTable.Insert("LEFTKNEE");
	m_translationTable.Insert("LEFTANKLE");
	m_translationTable.Insert("RIGHTHIP");
	m_translationTable.Insert("RIGHTKNEE");
	m_translationTable.Insert("RIGHTANKLE");
	m_translationTable.Insert("CHEST");
	m_translationTable.Insert("CHEST2");
	m_translationTable.Insert("LEFTCOLLAR");
	m_translationTable.Insert("LEFTSHOULDER");
	m_translationTable.Insert("LEFTELBOW");
	m_translationTable.Insert("LEFTWRIST");
	m_translationTable.Insert("RIGHTCOLLAR");
	m_translationTable.Insert("RIGHTSHOULDER");
	m_translationTable.Insert("RIGHTELBOW");
	m_translationTable.Insert("RIGHTWRIST");
	m_translationTable.Insert("NECK");
	m_translationTable.Insert("HEAD");
	ASSERT(m_translationTable.Size()==MotionLoader::NUM_JOINT_VOCA);
}
MotionLoader::MotionLoader()
:ModelLoader()
{
	_initTranslationTable(m_translationTable);
	m_pFactory=new TDefaultFactory<Posture>();

}

MotionLoader::MotionLoader(const char* filename, const char* option)
:ModelLoader()
{
	_initTranslationTable(m_translationTable);
	m_pFactory=new TDefaultFactory<Posture>();

	BinaryFile bf(false, filename);
	
	int version=unpack(bf);

	int type=bf.unpackInt();
	
	if(type==POSTUREIP)
	{
		if(!option || TString("loadSkeletonOnly")!=option)
		{
			m_cPostureIP.InitSkeleton(this);
			m_cPostureIP._unpack(bf, version);
		}
	}

	bf.close();
}

void MotionLoader::changeFactory(TFactory<Posture>* pF)		
{
	delete m_pFactory; 
	m_pFactory=pF; 
}

void MotionLoader::exportSkeleton(const char* filename) const
{
	BinaryFile bf(true, filename);

	pack(bf, MOT_RECENT_VERSION);
	bf.packInt(-1);
	bf.close();
}

void MotionLoader::loadAnimation(Motion& mot, const char* filename) const
{
	TString fn=filename;
	if(fn.right(4).toUpper()==".MOT")
	{
		BinaryFile bf(false, filename);
		
		MotionLoader temp;
		int version=temp.unpack(bf);

		int type=bf.unpackInt();

		ASSERT(type==POSTUREIP);
		ASSERT(temp.GetNumBone()==GetNumBone());
		ASSERT(temp.GetNumRotJoint()==GetNumRotJoint());
		ASSERT(temp.GetNumTransJoint()==GetNumTransJoint());

		mot.InitSkeleton((MotionLoader*)this);
		mot._unpack(bf, version);
		
		bf.close();
	}
	else	// ANIM
	{
		BinaryFile bf(false, filename);
		int version=bf.unpackInt();
		int type=bf.unpackInt();
		Msg::verify(type==POSTUREIP, "loadAnimationError");

		mot.InitSkeleton((MotionLoader*)this);
		mot._unpack(bf, version);
		bf.close();
	}
}


MotionLoader::~MotionLoader()
{
}

int MotionLoader::getJointVocaFromRotJointIndex(int jointIndex) const
{
	for(int i=0; i<NUM_JOINT_VOCA; i++)
	{ 
		if(jointIndex==m_aVoca2RotJointIndex[i])
		return i;
	}
	return -1;
}

int MotionLoader::getRotJointIndexFromTreeIndex(int treeIndex) const			{ return m_aTree2RotJointIndex[treeIndex];};
int MotionLoader::getTransJointIndexFromTreeIndex(int treeIndex) const			{return m_aTree2TransJointIndex[treeIndex];};
int MotionLoader::getRotJointIndexFromVoca(int jointVoca) const	{ return m_aVoca2RotJointIndex[jointVoca];};

void MotionLoader::UpdateBone()
{
	// 모든 matCombined가 완성되어 있어야 한다.

	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot->m_pChildHead;	// dummy노드는 사용안함.
	int index=-1;
		
	while(TRUE)
	{
		while(src)
		{
			index++;
			switch(src->NodeType)
			{
			case BONE:
				{
					Bone* pBone=(Bone*)src;
					if(m_TreeStack.GetTop())
						pBone->m_matCombined.mult(((Bone*)m_TreeStack.GetTop())->m_matCombined, pBone->m_matRot);
					else
						pBone->m_matCombined=pBone->m_matRot;
				}
				break;
			default:
				ASSERT(0);
			}
			m_TreeStack.Push(src);

			src=src->m_pChildHead;
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}

}


void MotionLoader::UpdateInitialBone()
{
	// 모든 matCombined가 완성되어 있어야 한다.

	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot->m_pChildHead;	// dummy노드는 사용안함.
	int index=-1;

	while(TRUE)
	{
		while(src)
		{
			index++;
			switch(src->NodeType)
			{
			case BONE:
				{
					Bone* pBone=(Bone*)src;
					if(m_TreeStack.GetTop())
						pBone->m_matCombined.mult(((Bone*)m_TreeStack.GetTop())->m_matCombined, pBone->m_matRotOrig);
					else
						pBone->m_matCombined=pBone->m_matRotOrig;
				}
				break;
			default:
				ASSERT(0);
			}
			m_TreeStack.Push(src);

			src=src->m_pChildHead;
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}

}

void MotionLoader::MakeBoneArrayFromTree(int& numChannel)
{
	m_nNumTreeNode=CountTreeNode();

	m_TreeStack.Initiate();
	m_apNode.resize(m_nNumTreeNode);
	m_aRotJoint2TreeIndex.setSize(m_nNumTreeNode);	// NumRotJoint()만큼의 array면 되지만, 아직 개수를 모르는 만큼, upper bound로 넉넉하게 준다.
	m_aTransJoint2TreeIndex.setSize(m_nNumTreeNode);  // NumTransJoint()만큼의 array면 되지만, 아직 개수를 모르는 만큼, upper bound로 넉넉하게 준다.
	m_aTree2RotJointIndex.setSize(m_nNumTreeNode);
	m_aTree2RotJointIndex.setAllValue(-1);
	m_aTree2TransJointIndex.setSize(m_nNumTreeNode);
	m_aTree2TransJointIndex.setAllValue(-1);
	Node *src=m_pTreeRoot;
	int inode=-1;
	numChannel=0;
	m_nNumEndNode=0;
	m_nNumRotNode=0;
	m_nNumTransNode=0;
	
	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			// Do somthing for node
			inode++;
			Bone* pBone;
			m_apNode[inode]=src;
			src->m_nIndex=inode;

			pBone=(Bone*)src;
			
			switch(pBone->m_eType)
			{
			case Bone::DUMMY:
				break;
			case Bone::ROOT:
			case Bone::JOINT:
				{
					if(pBone->m_numChannel)
					{
						TString trans=pBone->getTranslationalChannels();
						TString rot=pBone->getRotationalChannels();
						
						if(trans.length())
						{
							m_aTransJoint2TreeIndex[m_nNumTransNode]=inode;
							m_aTree2TransJointIndex[inode]=m_nNumTransNode;
							m_nNumTransNode++;
						}

						if(rot.length())
						{
							m_aRotJoint2TreeIndex[m_nNumRotNode]=inode;
							m_aTree2RotJointIndex[inode]=m_nNumRotNode;
							m_nNumRotNode++;
						}

						numChannel+=pBone->m_numChannel;
					}
				}
				break;
			case Bone::END:
				m_nNumEndNode++;
				break;
			default:
				ASSERT(0);
			}
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}
	//ASSERT(irotjoint==GetNumRotJoint());
	m_aRotJoint2TreeIndex.resize(GetNumRotJoint());

	// make parent index array
#ifdef _DEBUG
	Node* parent;
	for(int i=0; i<GetNumTreeNode(); i++)
	{
		parent=GetNode(i);
		for(Node* child=GetFirstChild(parent); child!=NULL; child=GetNextSibling(child))
			ASSERT(child->m_pParent==parent);
	}
#endif
}

int dep_GetParentJoint(MotionLoader const& ml, int rotjointIndex) 
{
	int parentIndex=ml.getBoneByRotJointIndex(rotjointIndex).m_pParent->GetIndex();
	while(parentIndex>=0) 
	{
		int pj=ml.getRotJointIndexFromTreeIndex(parentIndex);
		if(pj!=-1) return pj;
		Node* parent=ml.getBoneByTreeIndex(parentIndex).m_pParent;
		if(!parent) return -1;
		parentIndex=parent->GetIndex();
	}

	return -1;
}

int MotionLoader::getTransJointIndexByName(const char* nameID)
{
	int index=GetIndex(nameID);

	if(index==-1) return -1;
	return m_aTree2TransJointIndex[index];
}

int MotionLoader::getRotJointIndexByName(const char* nameid) const
{
	int numJoint=GetNumRotJoint();

	TString name;
	TString nameID=nameid;
	// case insensitive
	nameID.makeUpper();

	for(int j=0;j<numJoint; j++)
	{
		name=getBoneByRotJointIndex(j).NameId;
		//cout << name << endl;
		name.makeUpper();
		if(name==nameID) return j;
	}
	ASSERT(0);
	return -1;
}

int MotionLoader::GetNumRotJoint() const	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
{	
	return m_nNumRotNode;
}

int MotionLoader::GetNumTransJoint() const	//!< KeyFrame되어 있는 조인트의 개수를 구한다.
{	
	return m_nNumTransNode;
}

//joint 개수 + end노드 포함
int MotionLoader::GetNumBone() const
{
	//joint 개수 end노드 포함, Dummy 제외.
	return m_nNumTreeNode-1;
}

/// child의 transform은 identity가 되고, parent로 옮겨진다.
void MotionLoader::insertChildBone(Bone& parent, const char* nameId, bool bMoveChildren)
{
	Bone* newChild=new Bone();

	newChild->m_eType=Bone::JOINT;

	if(parent.m_eType==Bone::END)
	{
		parent.m_eType=Bone::JOINT;
		newChild->m_eType=Bone::END;
	}
	newChild->SetNameId(nameId);
	
	// initialize matrix
	newChild->m_matRotOrig.identity();
	newChild->m_matRot.identity();

	newChild->m_numChannel=0;

	std::list<Node*> children;
	if(bMoveChildren)
		parent.detachAllChildren(children);

	parent.AddChild(newChild);

	if(bMoveChildren)
		newChild->addChildren(children);
	
	int numChannel;
	MakeBoneArrayFromTree(numChannel);
}

/// type="T" (translational) or "R" (rotational) or "RT" (both)
void MotionLoader::insertJoint(Bone& target, const char* type)
{
	if(type[0]=='R')
	{
		int nchannel=3;
		if(type[1]=='T')
		{
			nchannel+=3;
			if(target.m_numChannel!=0)
			{
				Msg::print("Bone %s already has keys\n", target.NameId);
				return;
			}
		}
		if(target.m_numChannel>0 && 
			(target.m_aeChannels[0]==Bone::XRotation||
			target.m_aeChannels[0]==Bone::YRotation||
			target.m_aeChannels[0]==Bone::ZRotation))
		{
			Msg::print("Bone %s already has rotational keys\n", target.NameId);
			return;
		}

		///////////////////////
		// make channels
		target.m_numChannel=nchannel;
		target.m_aeChannels=new int[nchannel];

		int curc=0;
		if(nchannel==6)
		{
			target.m_aeChannels[curc++]=Bone::XPosition;
			target.m_aeChannels[curc++]=Bone::YPosition;
			target.m_aeChannels[curc++]=Bone::ZPosition;
		}
		target.m_aeChannels[curc++]=Bone::ZRotation;
		target.m_aeChannels[curc++]=Bone::XRotation;
		target.m_aeChannels[curc++]=Bone::YRotation;

		// Backup indexes
		intvectorn prevTree2RotJointIndex=m_aTree2RotJointIndex;
		intvectorn prevTree2TransJointIndex=m_aTree2TransJointIndex;

		// Update indexes
		int numChannel;
		MakeBoneArrayFromTree(numChannel);

		// index의 변화를 보고, m_cPostureIP를 고쳐준다.
		// rotational joint
		if(!(prevTree2RotJointIndex==m_aTree2RotJointIndex))
		{
			for(int i=0; i<prevTree2RotJointIndex.size(); i++)
			{
				if(prevTree2RotJointIndex[i]!=-1)
				{
					int prevJoint=prevTree2RotJointIndex[i];
					int newjoint=m_aTree2RotJointIndex[i];

					if(prevJoint==newjoint) continue;
					assert(newjoint==prevJoint+1);
					
					// prevJoint이후가 prevJoint+1로 옮겨지면 된다.

					if(m_aVoca2RotJointIndex.size())
					{
						for(int voca=0; voca<NUM_JOINT_VOCA; voca++)
						{
							if(m_aVoca2RotJointIndex[voca]>=prevJoint)
								m_aVoca2RotJointIndex[voca]++;
						}
					}

					quater key;
					key.setRotation(target.m_matRotOrig);

					int prevNumRotJoint=m_cPostureIP.Pose(0).numRotJoint();
					for(int i=0; i<m_cPostureIP.NumFrames(); i++)
					{
						m_cPostureIP.Pose(i).m_aRotations.resize(prevNumRotJoint+1);
						for(int j=prevNumRotJoint-1; j>=prevJoint; j--)
						{
							m_cPostureIP.Pose(i).m_aRotations[j+1]=
								m_cPostureIP.Pose(i).m_aRotations[j];
						}
						m_cPostureIP.Pose(i).m_aRotations[prevJoint]=key;
					}					
					break;
				}
			}
		}

		// translational joint
		if(!(prevTree2TransJointIndex==m_aTree2TransJointIndex))
		{
			for(int i=0; i<m_aTree2TransJointIndex.size(); i++)
			{
				if(m_aTree2TransJointIndex[i]!=-1)
				{
					int prevJoint=prevTree2TransJointIndex[i];
					int newjoint=m_aTree2TransJointIndex[i];

					if(prevJoint==newjoint) continue;
					assert(newjoint==prevJoint+1 || prevJoint==-1);

					int prevNumTransJoint=m_cPostureIP.Pose(0).numTransJoint();
					vector3 key;
					key.translation(target.m_matRotOrig);

					if(newjoint==prevJoint+1 )
					{
						// prevJoint이후가 prevJoint+1로 옮겨지면 된다.
						for(int i=0; i<m_cPostureIP.NumFrames(); i++)
						{
							m_cPostureIP.Pose(i).m_aTranslations.resize(prevNumTransJoint+1);
							for(int j=prevNumTransJoint-1; j>=prevJoint; j--)
							{
								m_cPostureIP.Pose(i).m_aTranslations[j+1]=
									m_cPostureIP.Pose(i).m_aTranslations[j];
							}
							m_cPostureIP.Pose(i).m_aTranslations[prevJoint]=key;
						}					
					}
					else
					{
						// 추가.
						assert(newjoint==prevNumTransJoint);

						for(int i=0; i<m_cPostureIP.NumFrames(); i++)
						{
							m_cPostureIP.Pose(i).m_aTranslations.resize(prevNumTransJoint+1);
							m_cPostureIP.Pose(i).m_aTranslations[newjoint]=key;
						}					
					}
					break;
				}
			}
		}

	}
}

void MotionLoader::ReadJointIndex(const char* filename)
{
	ConfigTable jointVoca(filename);
	
	m_aVoca2RotJointIndex.setSize(NUM_JOINT_VOCA);
	TString name;
	for(int i=0; i<NUM_JOINT_VOCA; i++)
	{
		name=jointVoca.Find(m_translationTable[i]);
		if(name.length()==0) 
		{
			Msg::print("warning! %s is not exist in %s", m_translationTable[i], filename);
			m_aVoca2RotJointIndex[i]=-1;
			continue;
		}
		name.trimLeft(" ");
		name.trimRight(" ");
		m_aVoca2RotJointIndex[i]=getRotJointIndexByName(name);
	}
}

void MotionLoader::Scale(float fScale)
{
	m_TreeStack.Initiate();

	Node *src=m_pTreeRoot;
	int index=-1;

	while(TRUE)
	{
		for(;src;src=src->m_pChildHead)
		{
			m_TreeStack.Push(src);
			index++;
			// do something for src
			// src의 index는 현재 index이다.

			Bone *pBone=(Bone*)src;
			vector3 offset;
			pBone->getOffset(offset);
			pBone->m_matRotOrig.setTranslation(offset*fScale);
			pBone->m_matRot=pBone->m_matRotOrig;
		}
		m_TreeStack.Pop(&src);
		if(!src) break;
		src=src->m_pSibling;
	}

	for(int i=0; i<m_cPostureIP.NumFrames(); i++)
	{
		for(int j=0; j<m_cPostureIP.NumTransJoints(); j++)
		{
			m_cPostureIP.Pose(i).m_aTranslations[j]*=fScale;
		}
	}

	m_cPostureIP.CalcInterFrameDifference();
}

void MotionLoader::scale(float fScale, Motion& mot)
{
	Scale(fScale);

	for(int i=0; i<mot.NumFrames(); i++)
	{
		for(int j=0; j<mot.NumTransJoints(); j++)
		{
			mot.Pose(i).m_aTranslations[j]*=fScale;
		}
	}
	mot.CalcInterFrameDifference();
}

/*
void MotionLoader::setPose(const Posture& pose)
{
	Bone& target=getBoneByRotJointIndex(0);
	
	// update root position and rotations	

	vector3 offset;
	target.getOffset(offset);
	target.m_matRot.setTransform(pose.m_aRotations[0], pose.m_aTranslations[0]+offset);	

	for(int ijoint=1; ijoint<GetNumRotJoint(); ijoint++)
	{
		// update rotations
        Bone& target=getBoneByRotJointIndex(ijoint);
		target.getOffset(offset);
		target.m_matRot.setTransform(pose.m_aRotations[ijoint], offset);
	}

	UpdateBone();
}*/

void MotionLoader::setPose(const Posture& pose)
{
	vector3 offset;	
	// update root position and rotations	
	for(int ijoint=0; ijoint<GetNumRotJoint(); ijoint++)
	{
		// update rotations
        Bone& target=getBoneByRotJointIndex(ijoint);
		target.getOffset(offset);
		target.m_matRot.setRotation(pose.m_aRotations[ijoint]);
		target.m_matRot.setTranslation(offset);
	}

	for(int ijoint=0; ijoint<GetNumTransJoint(); ijoint++)
	{
		// update translations
        Bone& target=getBoneByTransJointIndex(ijoint);
		target.m_matRot.setTranslation(pose.m_aTranslations[ijoint]);
	}

	UpdateBone();
}

Bone& MotionLoader::getBoneByTreeIndex(int index)	const {return *((Bone*)GetNode(index));}
	Bone& MotionLoader::getBoneByRotJointIndex(int iRotJoint)	const			{ return *((Bone*)GetNode(getTreeIndexFromRotJointIndex(iRotJoint)));}
	Bone& MotionLoader::getBoneByTransJointIndex(int iTransJoint)	const		{ return *((Bone*)GetNode(getTreeIndexFromTransJointIndex(iTransJoint)));}
	Bone& MotionLoader::getBoneByVoca(int jointVoca)	const	{ return *((Bone*)GetNode(getTreeIndexFromRotJointIndex(getRotJointIndexFromVoca(jointVoca))));}

void MotionLoader::setChain(const Posture& pose, int iTargetJoint)
{
	
	int ijoint=iTargetJoint;
	vector3 offset;

	// update rotation chains
	do
	{
		Bone& target=getBoneByRotJointIndex(ijoint);
		target.getOffset(offset);
		target.m_matRot.setRotation(pose.m_aRotations[ijoint]);
		target.m_matRot.setTranslation(offset);
	}
	while((ijoint=dep_GetParentJoint(*this, ijoint))!=-1);

	for(int ijoint=0; ijoint<GetNumTransJoint(); ijoint++)
	{
		// update translations
        Bone& target=getBoneByTransJointIndex(ijoint);
		target.m_matRot.setTranslation(pose.m_aTranslations[ijoint]);
	}

	int chain[10];

	int ibone=getTreeIndexFromRotJointIndex(iTargetJoint)+1;	// 하나 아래노드까지 update한다.(site가 달린경우때문)
		
	int chain_size=0;

	while(1)
	{
		chain[chain_size++]=ibone;

		Node* parent=getBoneByTreeIndex(ibone).m_pParent;
		if(!parent) break;
		ibone=parent->GetIndex();
	}
	
	ASSERT(GetNode(chain[chain_size-1])==m_pTreeRoot);	// dummy node
	ASSERT(GetNode(chain[chain_size-2])==m_pTreeRoot->m_pChildHead);

	Bone* pParentBone=(Bone*)GetNode(chain[chain_size-2]);
	pParentBone->m_matCombined=pParentBone->m_matRot;
	for(int i=chain_size-3; i>=0; i--)
	{
		Bone* pBone=(Bone*)GetNode(chain[i]);
		pBone->m_matCombined.mult(pParentBone->m_matCombined, pBone->m_matRot);
		pParentBone=pBone;
	}

}

Bone& MotionLoader::getBoneByName(const char* name) const
{
	int index=GetIndex(name);
	if(index==-1) Msg::error("getBoneByName %s", name);
	return *((Bone*)GetNode(index));
}

Bone& dep_GetBoneFromCon(MotionLoader const& ml, int constraint) 
{
	Bone* bone;
	bool bSite=true;
	switch(constraint)
	{
	case CONSTRAINT_LEFT_HEEL:
		bSite=false;
	case CONSTRAINT_LEFT_TOE:
	case CONSTRAINT_LEFT_FOOT:	
		bone=&ml.getBoneByVoca(MotionLoader::LEFTANKLE);
		break;

	case CONSTRAINT_RIGHT_HEEL:
		bSite=false;
	case CONSTRAINT_RIGHT_TOE:	
	case CONSTRAINT_RIGHT_FOOT:
		bone=&ml.getBoneByVoca(MotionLoader::RIGHTANKLE);
		break;

	case CONSTRAINT_LEFT_HAND:
		bSite=false;
	case CONSTRAINT_LEFT_FINGERTIP:
		bone=&ml.getBoneByVoca(MotionLoader::LEFTWRIST);
		break;

	case CONSTRAINT_RIGHT_HAND:
		bSite=false;
	case CONSTRAINT_RIGHT_FINGERTIP:
		bone=&ml.getBoneByVoca(MotionLoader::RIGHTWRIST);
		break;
	}

	if(bSite)
		return *bone->child();
	else
		return *bone;	
}

Bone& dep_GetSiteBone(MotionLoader const& ml, int ijoint) 
{ 
	ASSERT(ml.getBoneByRotJointIndex(ijoint).m_pChildHead);
	Node* curr;	
	// go down to leaf
	for(curr=&ml.getBoneByRotJointIndex(ijoint); curr->m_pChildHead!=NULL; curr=curr->m_pChildHead);
	return *((Bone*)curr);
}
#include "constraintmarking.h"

	
void MotionLoader::pack(BinaryFile & bf, int version) const
{
	
	bf.packInt(version);// version 1.
	bf.packInt(m_nNumTreeNode);

	if(version>=3)
	{
		bf.pack(MOT_VERSION_STRING[version]);
		bf.packInt(m_nNumRotNode);
		bf.packInt(m_nNumTransNode);
	}
	
	for(int i=0; i<GetNumTreeNode(); i++)
		GetNode(i)->pack(bf, version);

	bf.pack(m_aRotJoint2TreeIndex);	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index
	bf.pack(m_aTree2RotJointIndex);	//!< m_aTree2RotJointIndex[tree node index]=joint index

	if(version>=3)
	{
		bf.pack(m_aTransJoint2TreeIndex);
		bf.pack(m_aTree2TransJointIndex);
	}

	intvectorn aParentIndex;
	aParentIndex.setSize(GetNumTreeNode());
	aParentIndex[0]=-1;
	for(int i=1; i<GetNumTreeNode(); i++)
		aParentIndex[i]=getBoneByTreeIndex(i).m_pParent->GetIndex();

	bf.pack(aParentIndex);
	bf.pack(m_aVoca2RotJointIndex);
	bf.packInt(m_nNumEndNode);	//!< skeleton의 EndNode에는 KeyFrame이 들어가지 않으므로 중요.

}

int MotionLoader::unpack(BinaryFile & bf) 
{
	int version=bf.unpackInt();
	Msg::verify(version>=1, "version is too old");
	bf.unpackInt(m_nNumTreeNode);

	if(version>=3)
	{
		TString str;
		bf.unpack(str);
		bf.unpackInt(m_nNumRotNode);
		bf.unpackInt(m_nNumTransNode);
	}
	m_apNode.resize(m_nNumTreeNode);

	for(int i=0; i<GetNumTreeNode(); i++)
	{
		Msg::verify(bf.unpackInt()==BONE, "nodetype!=BONE");

		m_apNode[i]=new Bone();
		m_apNode[i]->m_nIndex=i;
		m_apNode[i]->unpack(bf, version);
	}

	bf.unpack(m_aRotJoint2TreeIndex);	//!< m_aRotJoint2TreeIndex[joint index]=TreeNode index
	bf.unpack(m_aTree2RotJointIndex);	//!< m_aTree2RotJointIndex[tree node index]=joint index

	if(version>=3)
	{
		bf.unpack(m_aTransJoint2TreeIndex);
		bf.unpack(m_aTree2TransJointIndex);
	}
	else
	{
		m_aTransJoint2TreeIndex.setSize(1);
		m_aTransJoint2TreeIndex[0]=1;
		m_aTree2TransJointIndex.setSize(2);
		m_aTree2TransJointIndex[0]=-1;
		m_aTree2TransJointIndex[1]=0;

	}

	intvectorn aParentIndex;
	bf.unpack(aParentIndex);
	bf.unpack(m_aVoca2RotJointIndex);
	bf.unpackInt(m_nNumEndNode);	//!< skeleton의 EndNode에는 KeyFrame이 들어가지 않으므로 중요.

	if(version<3)
	{
		m_nNumRotNode=m_nNumTreeNode-m_nNumEndNode-1;
		m_nNumTransNode=1;
	}

	for(int i=0; i<GetNumTreeNode(); i++)
	{
		if(aParentIndex[i]!=-1)
			m_apNode[aParentIndex[i]]->AddChild(m_apNode[i]);
	}

	m_pTreeRoot=m_apNode[0];
	return version;
}


Bone& dep_GetSiteBoneVoca(MotionLoader const& ml, int jointVoca) { return dep_GetSiteBone(ml, ml.getRotJointIndexFromVoca(jointVoca));}
