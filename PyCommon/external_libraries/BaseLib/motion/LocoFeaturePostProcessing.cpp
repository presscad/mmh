#include "StdAfx.h"
#include <list>
#include ".\locofeaturepostprocessing.h"
#include "featureextractor.h"

using namespace MotionSegmentation;

void LocoFeaturePostProcessing::newNode(const char* sz, int priority)
{
	m_aNodeIndex.Insert(sz);
	m_aNode.push_back(m_graph.newNode());
	m_aPriority.push_back(priority);
	m_aNodeName.pushBack(new TString(sz));
	
}

LocoFeaturePostProcessing::LocoFeaturePostProcessing(bitVectorN& abCutState, Motion* pMotion)
{
	MotionClustering::LocoFeatureExtractor featureExt(pMotion, abCutState);

	TArray<vectorN> aFeature;
	featureExt.calcFeatureVector(aFeature, abCutState);

	intVectorN encoding;
	encoding.runLengthEncodeCut(abCutState);

	TList<FeatureString> listFeatureString;
	for(int i=0; i<aFeature.size(); i++)
	{
		int start=encoding[i*2];
		int end=encoding[i*2+1];
		listFeatureString.pushBack(new FeatureString(featureExt.featureString(aFeature[i]), start, end));
	}

	// generic graph
	newNode("LR", 1);
	newNode("RL", 1);
	newNode("TLT", 1);
	newNode("TRT", 1);
	newNode("S", 1);
	newNode("TLS", 2);
	newNode("TRS",2);
	newNode("LS", 2);
	newNode("RS", 2);
	newNode("SLT",2);
	newNode("SRT",2);	
	newNode("SL",2);
	newNode("SR",2);
	newNode("LRT",3);
	newNode("RLT",3);
	newNode("TLR",3);
	newNode("TRL",3);
	
	// edge도 만들어 놓는다. (사용안함)
	m_graph.newEdge(node("LR"), node("RL"));	
	m_graph.newEdge(node("LR"), node("RLT"));
	m_graph.newEdge(node("LR"), node("RS"));

	m_graph.newEdge(node("RL"), node("LR"));
	m_graph.newEdge(node("RL"), node("LRT"));
	m_graph.newEdge(node("RL"), node("LS"));

    m_graph.newEdge(node("TLT"), node("TRT"));
	m_graph.newEdge(node("TLT"), node("TRL"));
	m_graph.newEdge(node("TLT"), node("TRS"));

	m_graph.newEdge(node("TRT"), node("TLT"));
	m_graph.newEdge(node("TRT"), node("TLR"));
	m_graph.newEdge(node("TRT"), node("TLS"));

	m_graph.newEdge(node("S"), node("S"));
	m_graph.newEdge(node("S"), node("SL"));
	m_graph.newEdge(node("S"), node("SR"));
	m_graph.newEdge(node("S"), node("SLT"));
	m_graph.newEdge(node("S"), node("SRT"));

	// transition
    m_graph.newEdge(node("LRT"), node("TLT"));
	m_graph.newEdge(node("RLT"), node("TRT"));

	m_graph.newEdge(node("TLR"), node("RL"));
	m_graph.newEdge(node("TRL"), node("LR"));

	m_graph.newEdge(node("LS"), node("S"));
	m_graph.newEdge(node("RS"), node("S"));

	m_graph.newEdge(node("SL"), node("LR"));
	m_graph.newEdge(node("SR"), node("RL"));

	m_graph.newEdge(node("TLS"), node("S"));
	m_graph.newEdge(node("TRS"), node("S"));

	m_graph.newEdge(node("SLT"), node("TRT"));
	m_graph.newEdge(node("SRT"), node("TLT"));

	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		FeatureString& fs=*i;
		if(!node(fs.m_szFeature))
		{
			// merge with next
			// 앞에꺼랑 붙여보기
			TString szMergeBefore;
			if(i!=listFeatureString.begin())
			{
				i--;
				szMergeBefore=merge((*i).m_szFeature, fs.m_szFeature);				
				i++;
			}

			// 뒤에꺼랑 붙여보기
			TString szMergeAfter;
			if(i.next()!=listFeatureString.end())
			{
				i++;
				szMergeAfter=merge(fs.m_szFeature, (*i).m_szFeature);
				i--;
			}
            
			if(node(szMergeBefore))
			{
				if(node(szMergeAfter))
				{
					// i를 i-1이랑 붙일수도 i+1이랑 붙일수도 있다.
					// 1. compare priority
					// 2. if priority is same, compare Len(i-1,i)-Len(i+1) vs Len(i-1)-Len(i,i+1)

					int prio1=priority(node(szMergeBefore));
					int prio2=priority(node(szMergeAfter));
					if(prio1<prio2)
						mergeBefore(listFeatureString, i);						
					else if(prio2<prio1)
						mergeAfter(listFeatureString,i);
					else
					{
						int len1=i.prev().data().length();
						int len2=i.data().length();
						int len3=i.next().data().length();				
						
						if(ABS(len1+len2-len3)<ABS(len1-(len2+len3)))
							mergeBefore(listFeatureString, i);						
						else
							mergeAfter(listFeatureString,i);
					}
				}
				else
					mergeBefore(listFeatureString, i);
			}
			else if(node(szMergeAfter))
			{
				mergeAfter(listFeatureString, i);
			}
			//else do nothing
		}
	}

	abCutState.clearAll();
	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		abCutState.setAt((*i).m_nStart);
	}
	abCutState|=pMotion->data().discontinuity();
}


LocoFeaturePostProcessing::~LocoFeaturePostProcessing(void)
{
}


bool LocoFeaturePostProcessing::mergeable(const TString& feature1, const TString& feature2)
{
	if( (feature1.right(1)=="L" && feature2.left(1)=="L") ||
		(feature1.right(1)=="R" && feature2.left(1)=="R") ||
		(feature1.right(2)=="RT" && feature2.left(2)=="TL") ||
		(feature1.right(2)=="LT" && feature2.left(2)=="TR"))
		return true;

	return false;
}

TString LocoFeaturePostProcessing::merge(const TString& feature1, const TString& feature2)
{
	if((feature1.right(1)=="L" && feature2.left(1)=="L") ||
		(feature1.right(1)=="R" && feature2.left(1)=="R"))
	{
		return feature1+feature2.right(feature2.length()-1);        
	}
	else if((feature1.right(2)=="RT" && feature2.left(2)=="TL") ||
			(feature1.right(2)=="LT" && feature2.left(2)=="TR"))
	{
		return feature1.left(feature1.length()-1)+feature2.right(feature2.length()-1);
	}
	else if(feature1=="T")
		return feature2;
	else if(feature2=="T")
		return feature1;
	return TString();
}

void LocoFeaturePostProcessing::mergeBefore(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i)
{
	i--;
	mergeAfter(listFeatureString, i);
}

void LocoFeaturePostProcessing::mergeAfter(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i)
{
	FeatureString& fs=*i;
	i++;
	fs.m_szFeature=merge(fs.m_szFeature, (*i).m_szFeature);
	fs.m_nEnd=(*i).m_nEnd;
	TList<FeatureString>::iterator backI=i;
	i--;
	listFeatureString.erase(backI);
}


void LocoFeaturePostProcessingShin::newNode(const char* sz, int priority)
{
	m_aNodeIndex.Insert(sz);
	m_aNode.push_back(m_graph.newNode());
	m_aPriority.push_back(priority);
	m_aNodeName.pushBack(new TString(sz));	
}

LocoFeaturePostProcessingShin::LocoFeaturePostProcessingShin(bitVectorN& abCutState, Motion* pMotion, MotionClustering::FeatureExtractor& featureExt)
{
	TArray<vectorN> aFeature;
	featureExt.calcFeatureVector(aFeature, abCutState);

	intVectorN encoding;
	encoding.runLengthEncodeCut(abCutState);

	TList<FeatureString> listFeatureString;
	for(int i=0; i<aFeature.size(); i++)
	{
		int start=encoding[i*2];
		int end=encoding[i*2+1];
		listFeatureString.pushBack(new FeatureString(featureExt.featureString(aFeature[i]), start, end));
	}

	// generic graph
	// cyclic motion 에 가장 우선순위를 두고 그 외에는 string이 긴것에 우선순위를 둔다.
	// LDR+RF+FL+LDR->LDR+RDL+LDR

	// flight가 들어간 transition은 가운데가 D인것과 F인것이 모호하므로 둘다 추가해놓고 나중에 한노드로 합친다.
	newNode("LDR", 0);
	newNode("RDL", 0);
	newNode("FLF", 0);
	newNode("FRF", 0);
	newNode("S", 0);

	newNode("FLDRS", 1);
	newNode("FRDLS",1);
	newNode("SLDRF",1);
	newNode("SRDLF",1);	
	
	// problematic peaks
	newNode("FLFRS", 1);
	newNode("FRFLS",1);
	newNode("SLFRF",1);	
	newNode("SRFLF",1);	
	
	newNode("LDRS", 2);
	newNode("RDLS", 2);
	newNode("SLDR",2);
	newNode("SRDL",2);
	
	newNode("LDRF",3);
	newNode("RDLF",3);
	newNode("FLDR",3);
	newNode("FRDL",3);

	// problematic peaks
	newNode("LFRF",4);
	newNode("RFLF",4);
	newNode("FLFR",4);
	newNode("FRFL",4);
	
	// edge도 만들어 놓는다. (사용안함)
	m_graph.newEdge(node("LDR"), node("RDL"));	
	m_graph.newEdge(node("LDR"), node("RDLF"));
	m_graph.newEdge(node("LDR"), node("RDLS"));

	m_graph.newEdge(node("RDL"), node("LDR"));
	m_graph.newEdge(node("RDL"), node("LDRF"));
	m_graph.newEdge(node("RDL"), node("LDRS"));

    m_graph.newEdge(node("FLF"), node("FRF"));
	m_graph.newEdge(node("FLF"), node("FRDL"));
	m_graph.newEdge(node("FLF"), node("FRDLS"));

	m_graph.newEdge(node("FRF"), node("FLF"));
	m_graph.newEdge(node("FRF"), node("FLDR"));
	m_graph.newEdge(node("FRF"), node("FLDRS"));

	m_graph.newEdge(node("S"), node("S"));
	m_graph.newEdge(node("S"), node("SLDR"));
	m_graph.newEdge(node("S"), node("SRDL"));
	m_graph.newEdge(node("S"), node("SLDRF"));
	m_graph.newEdge(node("S"), node("SRDLF"));

	// transition
    m_graph.newEdge(node("LDRF"), node("FLF"));
	m_graph.newEdge(node("RDLF"), node("FRF"));

	m_graph.newEdge(node("FLDR"), node("RDL"));
	m_graph.newEdge(node("FRDL"), node("LDR"));

	m_graph.newEdge(node("LDRS"), node("S"));
	m_graph.newEdge(node("RDLS"), node("S"));

	m_graph.newEdge(node("SLDR"), node("RDL"));
	m_graph.newEdge(node("SRDL"), node("LDR"));

	m_graph.newEdge(node("FLDRS"), node("S"));
	m_graph.newEdge(node("FRDLS"), node("S"));

	m_graph.newEdge(node("SLDRF"), node("FLF"));
	m_graph.newEdge(node("SRDLF"), node("FRF"));
	
	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		FeatureString& fs=*i;
		if(!node(fs.m_szFeature))
		{
			TString prev, next;
			if(i!=listFeatureString.begin())
			{
				i--;
				prev=(*i).m_szFeature;
				i++;
			}

			if(i.next()!=listFeatureString.end())
			{
				i++;
				next=(*i).m_szFeature;
				i--;
			}

			// merge three
			if(node(prev+fs.m_szFeature+next))
			{
				i--;
				mergeAfter(listFeatureString, i);
				mergeAfter(listFeatureString, i);
			}
			else
			{
				// merge with next
				// 앞에꺼랑 붙여보기
				TString szMergeBefore;
				if(prev.length())
					szMergeBefore=merge(prev, fs.m_szFeature);				

				// 뒤에꺼랑 붙여보기
				TString szMergeAfter;
				if(next.length())
					szMergeAfter=merge(fs.m_szFeature, next);
		        
				if(node(szMergeBefore))
				{
					if(node(szMergeAfter))
					{
						printf("ambiguity at %d\n", (*i).m_nStart);
						// i를 i-1이랑 붙일수도 i+1이랑 붙일수도 있다.
						// 1. compare priority
						// 2. if priority is same, compare Len(i-1,i)-Len(i+1) vs Len(i-1)-Len(i,i+1)

						int prio1=priority(node(szMergeBefore));
						int prio2=priority(node(szMergeAfter));
						if(prio1<prio2)
							mergeBefore(listFeatureString, i);						
						else if(prio2<prio1)
							mergeAfter(listFeatureString,i);
						else
						{
							int len1=i.prev().data().length();
							int len2=i.data().length();
							int len3=i.next().data().length();				
							
							if(ABS(len1+len2-len3)<ABS(len1-(len2+len3)))
								mergeBefore(listFeatureString, i);						
							else
								mergeAfter(listFeatureString,i);
						}
					}
					else
						mergeBefore(listFeatureString, i);
				}
				else if(node(szMergeAfter))
				{
					mergeAfter(listFeatureString, i);
				}
			}
		}
	}

// 한번 더하기-.- 이번에는 3줄 아래의 if문을 없애고 한다. FLDR + S의 케이스가 있어서.-.-
	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		FeatureString& fs=*i;
		//if(!node(fs.m_szFeature))
		{
			TString prev, next;
			if(i!=listFeatureString.begin())
			{
				i--;
				prev=(*i).m_szFeature;
				i++;
			}

			if(i.next()!=listFeatureString.end())
			{
				i++;
				next=(*i).m_szFeature;
				i--;
			}

			// merge three
			if(node(prev+fs.m_szFeature+next))
			{
				i--;
				mergeAfter(listFeatureString, i);
				mergeAfter(listFeatureString, i);
			}
			else
			{
				// merge with next
				// 앞에꺼랑 붙여보기
				TString szMergeBefore;
				if(prev.length())
					szMergeBefore=merge(prev, fs.m_szFeature);				

				// 뒤에꺼랑 붙여보기
				TString szMergeAfter;
				if(next.length())
					szMergeAfter=merge(fs.m_szFeature, next);
		        
				if(node(szMergeBefore))
				{
					if(node(szMergeAfter))
					{
						printf("ambiguity at %d\n", (*i).m_nStart);
						// i를 i-1이랑 붙일수도 i+1이랑 붙일수도 있다.
						// 1. compare priority
						// 2. if priority is same, compare Len(i-1,i)-Len(i+1) vs Len(i-1)-Len(i,i+1)

						int prio1=priority(node(szMergeBefore));
						int prio2=priority(node(szMergeAfter));
						if(prio1<prio2)
							mergeBefore(listFeatureString, i);						
						else if(prio2<prio1)
							mergeAfter(listFeatureString,i);
						else
						{
							int len1=i.prev().data().length();
							int len2=i.data().length();
							int len3=i.next().data().length();				
							
							if(ABS(len1+len2-len3)<ABS(len1-(len2+len3)))
								mergeBefore(listFeatureString, i);						
							else
								mergeAfter(listFeatureString,i);
						}
					}
					else
						mergeBefore(listFeatureString, i);
				}
				else if(node(szMergeAfter))
				{
					mergeAfter(listFeatureString, i);
				}
			}
		}
	}
	abCutState.clearAll();
	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		abCutState.setAt((*i).m_nStart);
	}
	abCutState|=pMotion->data().discontinuity();

	// print statistics
	LEDA::node_int_array count(m_graph);
	LEDA::node_int_array countMerged(m_graph);
	count.setAllValue(0);
	countMerged.setAllValue(0);
	LEDA::node v;

	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		if(pMotion->IsValid((*i).m_nStart, (*i).m_nEnd) && 
			pMotion->IsValid((*i).m_nEnd) &&
			!pMotion->data().IsDiscontinuous((*i).m_nEnd))
		{
			if(v=node((*i).m_szFeature))
			{
				count[v]++;
				if((*i).m_bMerged)
					countMerged[v]++;
			}
			else
				printf("Discard %s : [%d,%d) \n", (*i).m_szFeature.ptr(), (*i).m_nStart, (*i).m_nEnd);
		}
	}

	for_all_node(v, m_graph)
		printf("%s: %d (%d %.1f%%)\n", name(v).ptr(), count[v], countMerged[v], (float)countMerged[v]/(float)count[v]*100 );
	end_for;


	// Cut stop motions. (Align start timing: S아닌부분의 1/5이하로 줄여준다.)
	for(TList<FeatureString>::iterator i=listFeatureString.begin(); i!=listFeatureString.end(); i++)
	{
		int start=(*i).m_nStart;
		int end=(*i).m_nEnd;
		
        TString& feature=(*i).m_szFeature;

		if(feature[0]=='S')
		{
			int stopEnd;
			for(int i=start; i<end; i++)
			{
				if(!pMotion->isConstraint(i, CONSTRAINT_LEFT_FOOT) || !pMotion->isConstraint(i, CONSTRAINT_RIGHT_FOOT))
				{
					stopEnd=i;
					break;
				}
			}
			stopEnd=i;

			if(stopEnd-start>(end-stopEnd)/5)
			{
				//stopEnd-(end-stopEnd)/5> start
				abCutState.setAt(stopEnd-(end-stopEnd)/5);
			}
		}
		else if(feature[feature.length()-1]=='S')
		{
			int stopStart;
			for(int i=end-1; i>=start; i--)
			{
				if(!pMotion->isConstraint(i, CONSTRAINT_LEFT_FOOT) || !pMotion->isConstraint(i, CONSTRAINT_RIGHT_FOOT))
				{
					stopStart=i+1;
					break;
				}
			}

			if(end-stopStart>(stopStart-start)/5)
			{
				//stopStart+(stopStart-start)/5<end
				abCutState.setAt(stopStart+(stopStart-start)/5);
			}
		}
	}
}

LocoFeaturePostProcessingShin::~LocoFeaturePostProcessingShin(void)
{
}


bool LocoFeaturePostProcessingShin::mergeable(const TString& feature1, const TString& feature2)
{
	if( (feature1.right(1)=="L" && feature2.left(1)=="L") ||
		(feature1.right(1)=="R" && feature2.left(1)=="R") ||
		(feature1.right(2)=="RF" && feature2.left(3)=="FLS") ||
		(feature1.right(2)=="LF" && feature2.left(3)=="FRS") ||
		(feature1.right(3)=="SRF" && feature2.left(3)=="FLF") ||
		(feature1.right(3)=="SLF" && feature2.left(3)=="FRF") )
		return true;

	return false;
}

TString LocoFeaturePostProcessingShin::merge(const TString& feature1, const TString& feature2)
{
	if(!mergeable(feature1, feature2)) return TString();
	return _merge(feature1, feature2);
}

TString LocoFeaturePostProcessingShin::_merge(const TString& feature1, const TString& feature2)
{
	if((feature1.right(1)=="L" && feature2.left(1)=="L") ||
		(feature1.right(1)=="R" && feature2.left(1)=="R"))
	{
		return feature1+feature2.right(feature2.length()-1);        
	}
	else if((feature1.right(2)=="RF" && feature2.left(2)=="FL") ||
			(feature1.right(2)=="LF" && feature2.left(2)=="FR"))
	{
		// peak 무시
		return feature1.left(feature1.length()-1)+"D"+feature2.right(feature2.length()-1);
	}	
	return TString();
}

void LocoFeaturePostProcessingShin::mergeBefore(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i)
{
	i--;
	mergeAfter(listFeatureString, i);
}

void LocoFeaturePostProcessingShin::mergeAfter(TList<FeatureString>& listFeatureString, TList<FeatureString>::iterator& i)
{
	FeatureString& fs=*i;
	i++;
	
	printf("merging (%s, %s)", fs.m_szFeature.ptr(), (*i).m_szFeature.ptr());
	fs.m_szFeature=_merge(fs.m_szFeature, (*i).m_szFeature);
	fs.m_nEnd=(*i).m_nEnd;	
	fs.m_bMerged=true;
	printf("->%s %d\n", fs.m_szFeature.ptr(),(int)(fs.m_bMerged));
	
	TList<FeatureString>::iterator backI=i;
	i--;
	listFeatureString.erase(backI);
}
