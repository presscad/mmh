#pragma once

class Motion;
class matrixn;
class vector3N;
class quaterN;
class bitvectorn;
class Bone;

namespace MotionUtil
{
	void insertCOMjoint(MotionLoader& skel, m_real kernelSize=1.0,m_real kernelQuat=1.0);
	void insertRootJoint(MotionLoader& skel, matrixn const& aRootPos, matrixn const& aRootOri, const char* rootNameID);
}
