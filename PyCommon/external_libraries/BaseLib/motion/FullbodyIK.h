#pragma once

class Bone;
class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
class Posture;

namespace MotionUtil
{
	struct Effector
	{
		Effector()	{localpos=vector3(0,0,0);}
		Bone* bone;
		vector3 localpos;
	};

	// Abstract class. All IK solvers should reimplement IKsolve(...)
	class FullbodyIK
	{
	public:
		FullbodyIK(){}
		FullbodyIK(MotionLoader& skeleton, std::vector<Effector>& effectors){}
		virtual~ FullbodyIK(){}

		// interface type 1. All derived classes should reimplement this interface.
		virtual void IKsolve(Posture const& input_pose, vector3N const& constraintPositions, intvectorn & joint_index, quaterN& delta_rot){}

		// interface type 2. Utility function.
		void IKsolve(Posture& poseInOut, vector3N const& constraintPositions);
	};

	// effector가 발일때만 동작.
	FullbodyIK* createFullbodyIk_LimbIK(MotionLoader& skeleton, std::vector<Effector>& effectors);

	// 모든 경우에 동작하지만 느림.
	FullbodyIK* createFullbodyIk_MultiTarget(MotionLoader& skeleton, std::vector<Effector>& effectors);

}