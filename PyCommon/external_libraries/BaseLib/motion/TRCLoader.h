#pragma once

// TRCLoader.h: interface for the TRCLoader class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "motionloader.h"
class Parser;
//! TRC파일의 tree구조의 한 노드에 해당한다. 
class TRCTransform : public Bone
{
public:
	int parentMarkerIndex;
	int markerIndex;

	bool bTranslational;
	vector3 vOffsetDir;

	bool isConnector() const	{return markerIndex==parentMarkerIndex;}
	TRCTransform(BoneType NodeType);
	virtual ~TRCTransform();
};

//! TRC 파일 내용의 Motion부분을 그대로 저장한다. 나중에 Motion으로 적당히 변형된다.
class TRCIP
{
public:
	int m_numChannel;
	int m_numFrames;
	int m_numMarkers;
	m_real m_fFrameTime;
	
	std::vector<matrixn> m_aaKeyvalue;

	void Unpack(Parser* file, bool isTranspose);

	TRCIP();
	~TRCIP();
};
#include "../baselib/math/optimize.h"

//! TRC를 읽고 tree hierarchy와 PostureIP를 만든다.
/*! \ingroup group_motion */
class TRCLoader : public MotionLoader
{
	TRCIP cTRCIP;

public:
	TRCLoader(const char *filename, const char* option);
	void setValue(TRCTransform *ptr, char *name);
	virtual ~TRCLoader();

	/// parentMarkerIndex=-1 if there is no parent marker.
	void createTranslationalJoint(int parentMarkerIndex, int markerIndex, const char* tx, const char* rx);
	void createRotationalJoint(int parentMarkerIndex, int markerIndex, vector3 offsetDir, 
const char* axis);
	void createRotationalJoint2(int parentMarkerIndex, int markerIndex, vector3 offsetDir, 	const char* axis1, const char* axis2);
void createEffector(int parentMarkerIndex, int markerIndex, vector3 offsetDir);
private:
	std::vector<TRCTransform*> _temporaryNodes;
	IndexMapping _treeIndexToMarkerIndex;
	void buildHierarchy(TRCIP& cTRCIP);
	void MakePositionIPfromTRCIP(const TRCIP& cTRCIP, bool isTranspose);
	
};

