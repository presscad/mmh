#pragma once

class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
namespace MotionUtil
{
	class Retarget
	{
	public:
		Retarget(Motion& mot):m_mot(mot){}
		virtual ~Retarget(void){}
		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);
		Motion& m_mot;
	};


	// IK가 잘 동작하는지 테스트 한다. (perframe IK only)
	class RetargetTest : public Retarget
	{
	public:
		RetargetTest(Motion& mot):Retarget(mot){}
		virtual ~RetargetTest(){}

		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);

	};


	class Retarget2: public Retarget
	{
	public:
		Retarget2(Motion& mot):Retarget(mot){}
		virtual ~Retarget2(){}

		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);

	};

}