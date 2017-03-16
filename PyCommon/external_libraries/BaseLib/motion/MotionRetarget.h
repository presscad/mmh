#pragma once

class Motion;
class matrixn;
class vector3N;
class quaterN;
class bitvectorn;
class Bone;
class Path2D;

namespace MotionUtil
{
	class RetargetOnline2D
	{
		int mStart;
		int mRetargetQMethod;
		matrixn mCurve;
		Motion& mTarget;

		void getCurve(int start);
	public:
		enum {RETARGET_ROTY, RETARGET_DQ};

		// source는 smooth한 동작으로 가정한다. start를 포함하여 그 이후의 동작만 조정한다.
		RetargetOnline2D(Motion& target, int start, int eRetargetQMethod=RETARGET_ROTY);
						
		// 가정: time>start
		void adjust(int time, quater const& oriY, vector3 const& pos2D);
		void adjust(int time, quater const& oriY);
		void adjust(int time, vector3 const& pos2D);
		void adjust(int time, m_real deltarot);
		// 마지막 프레임의 orientation을 변화시키지 않는다. 그 대신, time에 틀어진만큼 이후에 돌아오는 회전이 추가된다.
		void adjustSafe(int time, m_real deltarot);
		// time을 time2가 되도록 timewarping한다. times는 이전에 frame이 retarget후 어느 프레임에 해당하는지를 계산한다.
		// times은 증가순으로 소팅되어있다고 가정한다.
		void adjust(int time, int time2, intvectorn& times);	

		// path2D의 마지막 프레임에서 정확히 같도록 조정하되, path2D랑 오차가 너무 크지 않도록 중간중간 컨스트레인이 추가됨.
		void adjustToPath(Path2D & path, int frameThr=INT_MAX, m_real angleThr=FLT_MAX, m_real distThr=FLT_MAX);
	};
	

	//여기 있는 stitch함수들은 stitch나 concat클래스들과 달리, 동작 세그먼트가 한프레임씩 겹치는 가정으로 구현되었음.
	// 이 방식이 더 최신 방식임. 

	// 앞쪽으로 prevSafe만큼은 바뀌어도 된다. 뒤쪽으로 afterSafe만큼은 바뀌어도 된다.
	// front 에 add를 부드럽게 연결한다.
	void stitchUsingOpponent(int prevSafe, int afterSafe, Motion& front, Motion const& add);


	void stitchUsingRoot(int prevSafe, int afterSafe, Motion& front, Motion const& add);

	void exportBVH(Motion const& mot, const char* filename, int start=0, int end=INT_MAX);
}