#include "stdafx.h"
#include ".\twoposture.h"

TwoPosture::TwoPosture(void)
:Posture()
{
}

TwoPosture::TwoPosture(const Posture& other)
:Posture()
{
	Clone(&other);
}

TwoPosture::~TwoPosture(void)
{
}


Posture* TwoPosture::clone() const
{
	TwoPosture *rtn=new TwoPosture();
	rtn->Clone(this);

	return rtn;
}

void TwoPosture::Clone(const Posture* pPosture)
{
	__super::Clone(pPosture);
	const TwoPosture* pTwoPosture;
	Msg::verify(pTwoPosture=dynamic_cast<const TwoPosture*>(pPosture), "no 2 posture");
	m_oppenentPos= pTwoPosture->m_oppenentPos;
	m_targetDir=pTwoPosture->m_targetDir;
	m_realRotAxis_y=pTwoPosture->m_realRotAxis_y;
	m_realOffset_q=pTwoPosture->m_realOffset_q;
}


void TwoPosture::Blend(const Posture& a, const Posture& b, m_real weight)
{

	__super::Blend(a,b,weight);
	m_oppenentPos.interpolate(weight,dynamic_cast<const TwoPosture&>(a).m_oppenentPos,dynamic_cast<const TwoPosture&>(b).m_oppenentPos);
}

//just joint angle blending
void TwoPosture::BlendJustForPhysics(const Posture& b, m_real weight)
{
	Posture &a=*this;
	ASSERT(a.numRotJoint() == b.numRotJoint());
	if(numRotJoint()!=a.numRotJoint() ||
		numTransJoint()!=a.numTransJoint())
	{
		Init(a.numRotJoint(), a.numTransJoint());
	}

//	m_dv.interpolate(weight, a.m_dv, b.m_dv);
//	m_aTranslations[0].interpolate(weight, a.m_aTranslations[0] , b.m_aTranslations[0]);


	for(int i=3; i<numRotJoint(); i++)
		m_aRotations[i].safeSlerp(a.m_aRotations[i], b.m_aRotations[i], weight);

//	__super::Blend(*this,b,weight);
}

void TwoPosture::Blend(const Posture** apPostures, const vectorn& weight)
{
	__super::Blend(apPostures, weight);
	m_oppenentPos.setValue(0,0,0);
/*
	// linear terms
	int n=weight.size();
	for(int i=0; i<n; i++)
	{
		m_oppenentPos.multadd(dynamic_cast<const TwoPosture*>(apPostures[i])->m_oppenentPos, weight[i]);
	}
*/

	//상대방 위치 블랜딩하기....
	//d_디콤포지션이 필요한지 안한지 몰겠음....

	matrixn targetRot;
	targetRot.setSize(weight.size(), 4);
	m_real scaling=0;
	vector3 targetDir;
	quater targetRot1;
	vector3 front(0,0,1);
	for(int i=0; i<weight.size(); i++)
	{
		targetDir=dynamic_cast<const TwoPosture*>(apPostures[i])->m_oppenentPos;
		targetRot1.axisToAxis(front, targetDir);
		targetRot.row(i).assign(targetRot1);
		scaling+=weight[i]*targetDir.length();
	}
	//타겟 디렉션 블렌딩
	targetRot1.blend(weight, targetRot);
	// calc blended targetdir
	targetDir.rotate(targetRot1, front);
	targetDir*=scaling;
	m_oppenentPos=targetDir;

}

void TwoPosture::decomposeRot() const
{
#ifdef TARGET_IS_FRONT
	// posture클래스와 달리, 이 클래스에서는 m_rotAxis_y는 targetDirection을 앞으로 가정한다.

	m_aRotations[0].decompose(m_realRotAxis_y, m_realOffset_q);

	// 이중에 targetdirection과 관련한걸 다시 계산해야함.
	m_rotAxis_y.axisToAxis(vector3(0,0,1), m_targetDir);
	m_rotAxis_y.align(m_realRotAxis_y);

	// m_aRotation[0]=m_rotAxis_y*m_offset_q; 따라서
	m_offset_q.mult(m_rotAxis_y.inverse(), m_aRotations[0]);
#else
	m_aRotations[0].decompose(m_rotAxis_y, m_offset_q);
	m_realRotAxis_y=m_rotAxis_y;
	m_realOffset_q=m_offset_q;
#endif
}
