#pragma once

namespace MotionUtil
{
	class KovarMetricY : public Metric
{
public:
	KovarMetricY(){}
	virtual ~KovarMetricY(){}

	// outputs
	matrix4 m_transfB;
	matrixn matA, matB, m_transformedB;	//!< Three n*3 matrixes.
	
	/**
	 * point cloud B를 y축으로 m_transfB로 변환하면 weighted 최소 거리 제곱합이 되도록 matching된다. 그때의 L2 거리를 return
	 * 
	 * \param a a는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \param b b는 n*3 matrix로부터 변환된 것으로 가정한다. 즉 (x1,y1,z1,x2,y2,z2,...)
	 * \return a와 b의 KovarDistance
	 */
	virtual m_real CalcDistance(const vectorn& a, const vectorn& b);
	virtual Metric* Clone() const;
	
	static void extractFeature(const Motion& mot, vectorn& feature, intvectorn const& frames);
};

class TransitionCost
{
	matrixn mFeatureVectors;
	bitvectorn mValid;
	int mMetric;
public:
	enum {GLOBAL_POS, FIXED_POS, ANGLE};	// GLOBAL_POS의 경우 kovar metric사용. 
	TransitionCost(const Motion& mot, int metric=ANGLE);
	virtual ~TransitionCost(void);

	m_real transitionCost(int from, int to) const;
};

}