//  Motion.h: interface for the Motion class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOTION_H__617260DC_091F_4D68_AB38_B9A459A43E2D__INCLUDED_)
#define AFX_MOTION_H__617260DC_091F_4D68_AB38_B9A459A43E2D__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

//#include "MotionLoader.h"

#include "PostureIP.h"
#include "../TwoPosture.h"
#include "../math/PCA.h"
#include "../math/filter.h"
#include "stitch.h"


class PLDPrimSkin;
class ModelLoader;
class MotionLoader;


//! 뼈대 정보와 해당 모션 정보를 갖는 클래스.
/*! 뼈대 정보는 m_pSkeleton에 저장되있다. 고, 모션 data는 m_pMotionData에 저장된다. 모션에 대한 High level manipulation함수들은 이 클래스에 있고, 
low level manipulation함수들은 멤버인 m_pMotionData즉 PostureIP클래스가 갖고 있다.
--> taesoo가 refectoring중이다. 모션 data를 MotinoClass가 갖도록 하고, PostureIP는 없앨 예정. 모션에 대한 HighLevel manipulation함수들은 MotionUtil namespace로 이동.
	\ingroup group_motion
*/
 
class Motion  : public Node
{
private:
	float m_fFrameTime;
	TString m_strIdentifier;	//!< default로는 NULL string, 필요하면 SetIdentifier해서 사용하시오.
	bitvectorn m_aDiscontinuity;	//!< 연결 되지 않는 동작 클립을 concat한경우 첫프레임마다 discontinuity가 true로 세팅된다.

	int m_numFrame;
	int m_maxCapacity;
	TArray<Posture> keyvalue;//!< PLD에 저장되어 있는 시간에 따른 translation값 
	MotionLoader* m_pSkeleton;	//!< 항상 reference이다. 즉 각 bvh별로 MotionLoader를 미리 생성해 놓고 포인터만 넘겨서 사용한다.

public:
	
	inline MotionLoader& skeleton() const	{ return *m_pSkeleton;};

	// make skeleton() be Pose(iframe)
	void setSkeleton(int iframe) const;
	void setPose(int iframe, const Posture& pose);
	


	Motion();
	/**
	 * source로부터 motion을 생성한다. 
	 * \param pSource Original motion을 갖고 있다.
	 * \param pNew NULL인 경우 m_pMotionData는 원본의 reference가 된다. new 만한 임의의 PostureIP를 넘기면, 내용을 original에서 복사한다.
	 * \return 
	 */
	Motion(MotionLoader* pSource);
	Motion(const Motion& srcMotion, int startFrame, int endFrame=INT_MAX);
	Motion(Motion const& other);	// copy constructor;
	Motion& operator=(const Motion& other);
	virtual ~Motion();

	
	

	// 동작의 길이는 "키프레임 개수-1" 로 정의한다. 즉 인터프레임간 길이를 1로 계산한것.
	int length() const				{return NumFrames()-1;}
	void changeLength(int length)	{ Resize(length+1); }

	// Init 
	void empty();

	void setMaxCapacity(int maxCapacity)	{ Msg::verify(m_numFrame==0, "SetMaxCapacityError"); m_maxCapacity=maxCapacity;}

	/// frame time과 skeleton을 source와 동일하게 한다.
	void InitEmpty(const Motion& source, int numFrames);

	/// frame time이 MotionLoader가 갖고 있는 동작 데이타와 같게 세팅한다. 만약 동작 데이타가 없는경우 30FPS로 세팅된다. 필요하면 frameTime()을 직접 세팅해줄것.
	void InitEmpty(MotionLoader* pSource, int numFrames);
	
	// skeleton만 초기화
	void InitSkeleton(MotionLoader* pSource);

	// skeleton을 초기화한 후, skeleton이 동작데이타(m_cPostureIP)를 가지고 있는경우 동작도 초기화.
	void Init(MotionLoader* pSource);
	
	//! 똑같은 모션을 만든다.(일부분을 따와서 만들 수도 있다.)
	void Init(const Motion& srcMotion, int startFrame=0, int endFrame=INT_MAX); 
	//! 똑같은 모션을 만든다.(일부분을 따와서 만들 수도 있다.)
	Motion* Clone(int startFrame=0, int endFrame=INT_MAX) const { return new Motion(*this, startFrame, endFrame);}
	
	void SetIdentifier(const char* id)			{ m_strIdentifier=id;};			//!< use as you want.
	const char* GetIdentifier() const			{ return m_strIdentifier;};		//!< use as you want.

	// skeleton과 모션을 모두 저장. ".mot"로 export. loading은 MotionLoader::loadAnimaion에서 가능하다. 
	void exportMOT(const char* filename) const;	

	// 0 <= criticalTime <= numFrames()-1
	// 즉, 0일때 Pose(0)이, numFrames()-1일때 Pose(numFrames()-1)이 return된다.
	// 기존에는 동작 데이타를 discrete한 개념으로 보았기 때문에, 위 SamplePose함수가 적합하지만
	// 2006년 6월 이후 부터는 동작데이터를 연속된 커브의 샘플링 값으로 취급하고 있다. 따라서 이 함수가 적합하다.
	void samplePose(Posture& pose, m_real criticalTime) const;

	//! pose어레이의 크기를 바꾼다. 길이가 늘어나는 경우 빈 pose들이 뒤쪽에 생긴다.
	void Resize(int frame);
	// endFrame포함 안함. 다른곳과 맞추기 위해서 고쳤습니다.
	void Concat(const Motion* pAdd, int startFrame=0, int endFrame=INT_MAX, bool bTypeCheck=true);
		
	int NumJoints()	const				{ return ((m_numFrame>0)?Pose(0).numRotJoint():0);}
	int NumTransJoints()	const		{ return ((m_numFrame>0)?Pose(0).numTransJoint():0);}
	int NumFrames() const				{ return m_numFrame;}

	float totalTime() const				{ return m_fFrameTime*length();}
	// length+1 을 리턴한다.
	int numFrames(float second) const	{ return ROUND(second/FrameTime())+1;}
	
	float FrameTime() const				{ return m_fFrameTime;}
	void FrameTime(float ftime)			{ m_fFrameTime=ftime;}
	int FrameRate() const				{ return int(1.f/FrameTime()+0.5f);}
	int KernelSize(float fTime) const	{ return Filter::CalcKernelSize(fTime, FrameTime());};
	
	bool isConstraint(int fr, int eConstraint) const;
	void setConstraint(int fr, int con, bool bSet=true);

	bool IsDiscontinuous(int fr) const			{ return m_aDiscontinuity[fr%m_maxCapacity];}	
	void setDiscontinuity(int fr, bool value)	{ m_aDiscontinuity.setValue(fr%m_maxCapacity, value);}


	int Parent(int jointIndex) const;//			{ return m_pSkeleton->GetParentJoint(jointIndex);};
	inline Posture& Pose(int iframe) const;
	TwoPosture& pose2(int iframe) const;
										
	enum { LOCAL_COORD, GLOBAL_COORD, FIXED_COORD, FIRST_ARRANGED_COORD, PELVIS_LOCAL_COORD, PELVIS_FIXED_COORD, NUM_COORD };

	void ChangeCoord(int eCoord);
	void CalcInterFrameDifference(int startFrame = 0);
	void ReconstructDataByDifference(int startFrame = 0, bool bForward=true)  
			{ 	_reconstructRotByDifference(startFrame,bForward);	_reconstructPosByDifference(startFrame, bForward);}
	void _reconstructPosByDifference(int startFrame = 0, bool bForward=true);
	void _reconstructRotByDifference(int startFrame = 0, bool bForward=true);


	//void changeFactory(TFactory<Posture>* pFactory)	;

	// do not use the followin functions unless you exactly know what you are doing.
	virtual void _Init(int nkey, int numRotJoint, int numTransJoint, float fFrameTime);
	virtual void _unpack(BinaryFile& File, int nVersion);
	virtual void _pack(BinaryFile& File, int nVersion) const;

	/////////////////////////////////////////////////////////////////////
	// 아래는 모두 deprecated functions: 사용하지 말것.
	/////////////////////////////////////////////////////////////////////

	// 0.5 <=criticalTime< numFrames()-0.5
	// 즉, 0.5일때 Pose(0)이, numFrames()-0.5일때 Pose(numFrames()-1)이 return 된다.
	void SamplePose(Posture& pose, m_real criticalTime) const;
	// totalTime과 TotalTime은 다른결과를 낸다는데 주의할것. -> TotalTime은 deprecated function.
	float TotalTime() const				{ return m_fFrameTime*NumFrames();}

	// If no frame in the interval is constrained, return -1. otherwise, return the indexed of first constrained frame;
	int isConstraint(int start, int end, int eConstraint) const;

	bool IsValid(int startFrame, int endFrame) const;
	bool IsValid(int iframe) const		{ return iframe>=0 && iframe<NumFrames();}
	bitvectorn& discontinuity()			{ Msg::verify(m_numFrame<m_maxCapacity, "discontinuity vector is invalid"); return m_aDiscontinuity;}
	bool IsContinuous(int iframe) const { return (iframe<NumFrames() && !IsDiscontinuous(iframe)); }

	// numFrames와 다른 결과를 낸다.
	int NumFrames(float fTime) const	{ return int(fTime/FrameTime()+0.5);}

private:
	void Release();
		
	// called only in constructor.	
	void _firstinit();

	void cloneFrom(const Motion& other, int otherStartFrame, int otherEndFrame, int thisStart, bool bTypeCheck=true);
};

#endif // !defined(AFX_MOTION_H__617260DC_091F_4D68_AB38_B9A459A43E2D__INCLUDED_)
