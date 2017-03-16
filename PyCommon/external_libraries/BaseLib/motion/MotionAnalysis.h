#pragma once

#include "../math/intervals.h"
#include "onlinepath2d.h"
	
class ParseGrcFile;
class MovingCone;

namespace MotionAnalysis
{
	class COMAnalysis
	{
		void _calcPath(Motion const& src, vectorn const& keytime, matrixn& path, TString const& startType, TString const& endType);
		void _calcOri(Motion const& src, vectorn const& keytime, vectorn& ori, TString const& startType, TString const& endType);
		
	public:
		// results
		matrixn mCOM;
		matrixn mPath1;
		matrixn mPath2;
		matrixn mCPath;	
		// analytic velocity 
		matrixn mVel1;
		matrixn mVel2;
		matrixn mCVel;	
		// analytic acceleration
		matrixn mAcc1;
		matrixn mAcc2;
		matrixn mCAcc;	

		vectorn mOri1;
		vectorn mOri2;
		vectorn mCOri;	
		// analytic velocity 
		vectorn mAngVel1;
		vectorn mAngVel2;
		vectorn mCAngVel;	
		// analytic acceleration
		vectorn mAngAcc1;
		vectorn mAngAcc2;
		vectorn mCAngAcc;	

		// Estimate path2D from CPath and CVel.
		OnlinePath2D mPath;
		
		COMAnalysis(Motion const& src, m_real kernelSize);
		virtual ~COMAnalysis(){ }

		bool isValid(int iframe);
		bool isValid(int start, int end);
		void _calcValidInterval(matrixn const& path, int iframe, int length, int& start, int& end);
	 };

	void pushBackNoDup(vectorn& a, m_real b);

}