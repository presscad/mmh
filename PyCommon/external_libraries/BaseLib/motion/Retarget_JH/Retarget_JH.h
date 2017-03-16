#pragma once

namespace MotionUtil
{
	struct Constraint
	{
		enum { C_POSITION, C_ORIENTATION, C_TRANFORM} ;
		Constraint() { type=C_POSITION; bone=NULL;}
		int type;
		Bone* bone;
		quater rotation;
		vector3 translation;
	};

	class FullbodyIK_JH
	{
	public:
		void* m_private_data;	
		FullbodyIK_JH(MotionLoader const& skeleton);
		~FullbodyIK_JH();

		void IKsolve(Posture& poseInOut, std::vector<Constraint>& constraints);
	};
}