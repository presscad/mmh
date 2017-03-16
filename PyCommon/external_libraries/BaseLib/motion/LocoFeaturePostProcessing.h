#pragma once
#include "../baselib/utility/tlist.h"
#include "leda/graph.h"
#include "featureExtractor.h"
namespace MotionSegmentation
{

class LocoFeaturePostProcessing
{
public:
	class FeatureString
	{
	public:
		FeatureString()	{}
		FeatureString(const TString& feature, int start, int end):m_szFeature(feature), m_nStart(start), m_nEnd(end){}
		TString m_szFeature;
		int length()	{ return m_nEnd-m_nStart;}
		int m_nStart;
		int m_nEnd;
	};

	LocoFeaturePostProcessing(bitVectorN& abCutState, Motion* pMotion);
	~LocoFeaturePostProcessing(void);

private:
	
	NameTable m_aNodeIndex;
	TArray<TString> m_aNodeName;
	std::vector<LEDA::node> m_aNode;
	std::vector<int> m_aPriority;
	LEDA::graph m_graph;	

	void newNode(const char*, int priority);
	int numNode()						{ return m_aNode.size();}
	LEDA::node node(int i)				{ return m_aNode[i];}
	LEDA::node node(const char* sz)		{ if(m_aNodeIndex.find(sz)) return m_aNode[m_aNodeIndex[sz]]; return NULL; }
	TString& name(LEDA::node v)			{ return m_aNodeName[v->index()];}
	int priority(LEDA::node v)			{ return m_aPriority[v->index()];}
	TString& name(int i)				{ return m_aNodeName[i];}


	void mergeBefore(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i);
	void mergeAfter(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i);
	bool mergeable(const TString& feature1, const TString& feature2);
	TString merge(const TString& feature1, const TString& feature2);
};

class LocoFeaturePostProcessingShin
{
public:
	class FeatureString
	{
	public:
		FeatureString()	{m_bMerged=false;}
		FeatureString(const TString& feature, int start, int end):m_szFeature(feature), m_nStart(start), m_nEnd(end), m_bMerged(false){}
		TString m_szFeature;
		int length()	{ return m_nEnd-m_nStart;}
		int m_nStart;
		int m_nEnd;
		bool m_bMerged;
	};

	LocoFeaturePostProcessingShin(bitVectorN& abCutState, Motion* pMotion, MotionClustering::FeatureExtractor& featureExt);
	~LocoFeaturePostProcessingShin(void);

private:
	
	NameTable m_aNodeIndex;
	TArray<TString> m_aNodeName;
	std::vector<LEDA::node> m_aNode;
	std::vector<int> m_aPriority;
	LEDA::graph m_graph;	

	void newNode(const char*, int priority);
	int numNode()						{ return m_aNode.size();}
	LEDA::node node(int i)				{ return m_aNode[i];}
	LEDA::node node(const char* sz)		{ if(m_aNodeIndex.find(sz)) return m_aNode[m_aNodeIndex[sz]]; return NULL; }
	TString& name(LEDA::node v)			{ return m_aNodeName[v->index()];}
	int priority(LEDA::node v)			{ return m_aPriority[v->index()];}
	TString& name(int i)				{ return m_aNodeName[i];}


	void mergeBefore(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i);
	void mergeAfter(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i);
	bool mergeable(const TString& feature1, const TString& feature2);
	TString merge(const TString& feature1, const TString& feature2);
	TString _merge(const TString& feature1, const TString& feature2);
};

}