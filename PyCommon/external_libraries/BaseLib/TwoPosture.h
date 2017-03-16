#pragma once

#include "motion/postureip.h"
class TwoPosture : public Posture
{
public:
	TwoPosture(void);
	TwoPosture(const Posture& other);
	~TwoPosture(void);

	// in local coordinate (이때 m_realRotAxis_y를 사용해야 한다. m_rotAxis_y를 사용하면, (0,0,1)이 되기 때문.
	// 나중에 한번에 이름을 m_localTargetDir로 바꿀것임.
	vector3 m_oppenentPos;	
	vector3 m_targetDir;	// in global coordinate

	virtual Posture* clone() const;

	virtual void Blend(const Posture& a, const Posture& b, m_real weight);
	virtual void BlendJustForPhysics( const Posture& b, m_real weight);
	//m_oppenetPos에 대한 블렌딩은 아래 함수에 대해서만 작성되 있다.
	virtual void Blend(const Posture** apPostures, const vectorn& aWeights);
	// m_aRotations[0]->m_rotAxis_y*m_offset_q로 decompose
	virtual void decomposeRot() const;

	mutable quater m_realRotAxis_y; //!< y 방향 회전만을 포함하는 rotation
	mutable quater m_realOffset_q;  //!< m_dq가 포함하지 않는 x 및 z 방향 오리엔테이션 정보, 즉 기울기로 local 좌표계에서 정의된다.
	
	virtual void Clone(const Posture* pPosture);
private:
	TwoPosture(const TwoPosture& other) {assert(0);}
	TwoPosture& operator=(const TwoPosture&) {assert(0);}	
	TwoPosture& operator=(const Posture&) {assert(0);}
};
