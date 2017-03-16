#include "stdafx.h"
#include "Motion.h"
#include "BVHLoader.h"

#include "TransitionCost.h"
#include "../BASELIB/MOTION/MOTIONUTIL.H"
using namespace MotionUtil;


Metric* KovarMetricY::Clone() const {return new KovarMetricY();}

m_real KovarMetricY::CalcDistance(const vectorn& a, const vectorn& b)
{
	matA.fromVector(a,3);
	matB.fromVector(b,3);

	int n=matA.rows();
	ASSERT(matB.rows()==n);
	
	// Refer SnapTogetherMotion.pdf by H.J.Shin
	// Basically, I will find best matching y_axis rotation
	m_real theta, x0, z0;
	
	// Calculate theta
	m_real xbar=0, x2bar=0, zbar=0, z2bar=0;
	enum {X=0, Y=1, Z=2};
	m_real wi=1.0/((m_real)n);

	for(int i=0; i<n; i++)
	{
		xbar+=wi*matA[i][X];
		x2bar+=wi*matB[i][X];
		zbar+=wi*matA[i][Z];
		z2bar+=wi*matB[i][Z];
	}

	m_real sumA=0, sumB=0;

	for(int i=0; i<n; i++)
	{
		m_real xi=matA[i][X];
		m_real zi=matA[i][Z];
		m_real xi2=matB[i][X];
		m_real zi2=matB[i][Z];
        		
		sumA+=wi*(xi*zi2-xi2*zi);
		sumB+=wi*(xi*xi2+zi*zi2);
	}

	sumA-=(xbar*z2bar-x2bar*zbar);
	sumB-=(xbar*x2bar+zbar*z2bar);
	theta=atan2(sumA,sumB);

	// calculate x0 and z0
	x0=xbar-x2bar*cos(theta)-z2bar*sin(theta);
	z0=zbar+x2bar*sin(theta)-z2bar*cos(theta);

	//printf("theta %f , x0 %f z0 %f\n",theta, x0, z0 );

	// calculate matching distance
	m_transfB.setIdentityRot();
	m_transfB.leftMultRotation(vector3(0,1,0), theta);
	m_transfB.leftMultTranslation(vector3(x0, 0, z0));
	
	// transform and calc distance
	m_transformedB.setSameSize(matB);
	
	for(int i=0; i<n; i++)
	{
		vector3 transformed;
		transformed.mult(m_transfB, matB.row(i).toVector3());
		m_transformedB.row(i).assign(transformed);
	}
	
	m_real distance=0;
	for(int i=0; i<n; i++)
	{
		m_real temp=matA.row(i).distance(m_transformedB.row(i));
		distance+=SQR(temp);
	}

	return sqrt(distance);
}

void MotionUtil::KovarMetricY::extractFeature(const Motion& mot, vectorn& feature, intvectorn const& frames)
{
	intvectorn jointIndex;
	jointIndex.colon(0, mot.NumJoints());

	hypermatrixn aaPos;
	matrixn temp;
	matrixn temp2;

	int start=frames[0];
	int end=frames[frames.size()-1]+1;
	{
		MotionUtil::GetSignal sig(mot);
		sig.jointPos(jointIndex, aaPos, MotionUtil::GLOBAL_COORD, frames[0], frames[frames.size()-1]+1);
		temp.fromHyperMat(aaPos);

		temp2.setSize(frames.size(), temp.cols());
		for(int i=0; i<frames.size(); i++)
			temp2.row(i)=temp.row(frames[i]-frames[0]);			

		feature.fromMatrix(temp2);
	}
}

TransitionCost::TransitionCost(const Motion& mMotion,int metric)
{
	mMetric=metric;

	const int interval=2;	// interval has to be divided by step
	const int step=2;	// encode joint positions for every 2 frames.

	intvectorn EEIndex;
	
	int ind = 0;
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::HIPS));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::CHEST));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::HEAD));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTSHOULDER));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTSHOULDER));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTHIP));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTHIP));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTELBOW));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTELBOW));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTKNEE));
	EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTKNEE));

	if(metric!=ANGLE)
	{
		EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTANKLE));
		EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTANKLE));
		EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::LEFTWRIST));		
		EEIndex.pushBack( mMotion.skeleton().getRotJointIndexFromVoca(MotionLoader::RIGHTWRIST));
	}

	intvectorn& jointIndex=EEIndex;

	hypermatrixn aaPosA;

	MotionUtil::GetSignal sig(mMotion);
	
	switch(mMetric)
	{
	case GLOBAL_POS:
		sig.jointPos(jointIndex, aaPosA, MotionUtil::GLOBAL_COORD);
		break;
	case FIXED_POS:
		sig.jointPos(jointIndex, aaPosA, MotionUtil::FIXED_COORD);
		break;
	case ANGLE:
		sig.jointOri(jointIndex, aaPosA, MotionUtil::FIXED_COORD);
		break;
	}

	static matrixn temp;
	mValid.setSize(mMotion.NumFrames());
	mValid.setAll();

	int eachDim=3;
	if(mMetric==ANGLE) eachDim=4;
	mFeatureVectors.setSize(mMotion.NumFrames(), aaPosA.page()*eachDim*((interval/step)*2+1));
	for(int i=0; i<mMotion.NumFrames(); i++)
	{
		for(int j=i-interval; j<=i+interval; j++)
		{
			if(j<1 || j>=mMotion.NumFrames() || mMotion.IsDiscontinuous(j-1))
			{
				mValid.clearAt(i);
				break;
			}
		}
		
		if(mValid[i])
		{
			temp.resize(0,eachDim);

			for(int j=i-interval; j<=i+interval; j+=step)
			{
				for(int k=0; k<aaPosA.page(); k++)
					temp.pushBack(aaPosA.page(k).row(j));
			}

			ASSERT(temp.cols()==eachDim);
			mFeatureVectors.row(i).fromMatrix(temp);
		}
	}
}

TransitionCost::~TransitionCost(void)
{
}

m_real TransitionCost::transitionCost(int from, int to) const
{
	static KovarMetricY metric;
	static QuaterMetric metricQ;


	m_real distance;
	if(mValid[from] && mValid[to])
	{
		if(mMetric==ANGLE)
			distance=metricQ.CalcDistance(mFeatureVectors.row(from), mFeatureVectors.row(to));
		else
			distance=metric.CalcDistance(mFeatureVectors.row(from), mFeatureVectors.row(to));
	}
	else
		distance=DBL_MAX;
	//distance : ÃÖ¼Ò L2Distance
	//normalize by size-> sqrt(distance*distance/size);
	return distance;
}

