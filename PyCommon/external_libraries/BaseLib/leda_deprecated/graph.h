#pragma once
#include "../utility/tarray.h"
#include "../utility/tlist.h"
#include "../math/mathclass.h"
namespace LEDA_deprecated
{
	// taesoo's implementation of LEDA_deprecated interface.

class node_struct;
class edge_struct;
typedef node_struct* node;
typedef edge_struct* edge;

class node_struct
{
public:
	node_struct():_index(-1), _aIncomingE(true), _aOutgoingE(true)	{}
	virtual ~node_struct()		{}

	int index()	const		{ return _index;}

	int  outdeg()    const	{ return outEdge().size(); }
	int  indeg()     const	{ return inEdge().size(); }
	int  degree()    const	{ return outdeg()+indeg();}
	const TList<edge_struct>& outEdge() const	{ return _aOutgoingE;}
	const TList<edge_struct>& inEdge() const		{ return _aIncomingE;}	

private:
	void _init(int index) { _index=index;}
	int _index;			// internal name (index)  
	
	//original LEDA_deprecated의 경우 OUTGOING=0, INCOMING=1로 표현한다.
	TList<edge_struct> _aIncomingE;	//!< reference array로 초기화 함에 주의
	TList<edge_struct> _aOutgoingE;//!< reference array로 초기화 함에 주의
	friend class graph;
};

class edge_struct
{
public:
	edge_struct():_index(-1),_s(NULL),_t(NULL)	{}
	virtual ~edge_struct()						{}

	node source()    const { return _s;}
	node target()    const { return _t;}
	int index() const		{ return _index;}
private:
	void _init(node v, node w, int index) {_index=index;_s=v;_t=w;}
	int  _index;          // internal name (index)  
	node _s;             // source node 
	node _t;             // target node
	friend class graph;
};
class base_node_array;
class base_edge_array;

/** 가정: 에지나 node가 추가로 생길수는 있지만 중간에 지워지지는 않는다. 그래프 자료구조에서는 상관없지만, node_array나 edge_array의 구현에서 빠른 indexing 속도와 쉬운 구현을 위해 array를 사용했기 때문. 나중에 이 가정을 없앨 필요가 있으면 아마도 interface는 유지한채로 재구현이 가능할 것이다.
*/
class graph {
public:
	graph();
	// derived class에서 노드와 edge도 derive된 경우, 이거를 call하든지, 각각 list의 changeFactory를 call할것.
    graph(TFactory<node_struct>* pVFactory, TFactory<edge_struct>* pEFactory);
	virtual ~graph()		{ clear(); }
	virtual void clear();

	int  numNodes() const   { return m_aV.size(); }
	int  numEdges() const   { return m_aE.size(); }

	/// 새 노드생성과 edge생성은 반드시 newNode, newEdge를 사용할것.
	node newNode();
	edge newEdge(node v, node w);
    edge findEdge(node v, node w);
	node findNode(int index) const;

	/// derived class에서 node_struct, edge_struct factory를 알아서 바꿔주면 된다.
	TList<node_struct> m_aV;              //!< list of all nodes 
	TList<edge_struct> m_aE;              //!< list of all edges

	
protected:
	int m_nEdge;
	int m_nNode;	

	mutable std::list<base_node_array*> m_listNodeArray;
	mutable std::list<base_edge_array*> m_listEdgeArray;

	friend class base_node_array;
	friend class base_edge_array;
};


class base_node_array
{
protected:
	base_node_array(){mGraph=NULL;}
	virtual ~base_node_array(){}
	virtual int size() const			{ return 0;}
	virtual void init(const graph& G)	
	{
		if(mGraph && mGraph!=&G)
			mGraph->m_listNodeArray.remove(this);
		G.m_listNodeArray.push_back(this);
		mGraph=&G;
	}
	virtual void update(const graph& G) {}
	friend class graph;
	const graph* mGraph;
};

class base_edge_array
{
protected:
	base_edge_array(){mGraph=NULL;}
	virtual ~base_edge_array(){}
	virtual int size() const			{ return 0;}
	virtual void init(const graph& G)	
	{
		if(mGraph && mGraph!=&G)
			mGraph->m_listEdgeArray.remove(this);
		G.m_listEdgeArray.push_back(this);
		mGraph=&G;
	}
	virtual void update(const graph& G) {}
	
	friend class graph;
	const graph* mGraph;
};

template <class T>
class edge_array : protected TArray<T>, protected base_edge_array
{
public:
	edge_array():TArray<T>(),base_edge_array(){}
	edge_array(const graph& G):TArray<T>() { init(G);}
	~edge_array(){}

	virtual int size() const				{ return TArray<T>::size();}
	virtual void init(const graph& G)		{ __super::init(G); TArray<T>::init(G.numEdges());}
	virtual void update(const graph& G)		{ TArray<T>::resize(G.numEdges()); }
	T& operator[](edge E) const		{ return TArray<T>::data(E->index());}
};

class edge_int_array : public intvectorn, protected base_edge_array
{
	public:
	edge_int_array ():intvectorn(),base_edge_array(){}
	edge_int_array (const graph& G):intvectorn(){ init(G);}
	~edge_int_array (){}

	virtual void init(const graph& G)			{ __super::init(G); setSize(G.numEdges());}
	virtual void update(const graph& G) { resize(G.numEdges()); }
	int& operator[](edge E) const		{ return value(E->index());}
};

template <class T>
class node_array : protected TArray<T>, protected base_node_array
{
public:
	node_array():TArray<T>(),base_node_array(){}
	node_array(const graph& G):TArray<T>(){ init(G);}
	~node_array(){}

	virtual void init(const graph& G)		{ __super::init(G); TArray<T>::init(G.numNodes());}
	virtual void update(const graph& G) { TArray<T>::resize(G.numNodes()); }
	T& operator[](node V) const		{ return TArray<T>::data(V->index());}
};

class node_int_array : public intvectorn, protected base_node_array
{
	public:
	node_int_array ():intvectorn(),base_node_array(){}
	node_int_array (const graph& G):intvectorn(){ init(G);}
	~node_int_array (){}

	virtual void init(const graph& G)			{ __super::init(G); setSize(G.numEdges());}
	virtual void update(const graph& G)				{ resize(G.numNodes()); }
	int& operator[](node V) const		{ return value(V->index());}
};

bool BELLMAN_FORD(const graph& G, node s, const edge_array<float>& cost, 
                                                node_array<float>& dist, 
                                                node_array<edge>& pred ) ;

void PATH(node s, node t, const node_array<edge>& pred, TList<node>& path);
void PATH_EDGE(node s, node t, const node_array<edge>& pred, TList<edge>& path);
void BFS(const graph& G, node v, node_array<int>& dist);
void DFS(const graph& G, node v, node_array<bool>& reached, TList<node>& L);
void DRAW(const graph& G, const node_array<TString>& name, const char* filename);
void DRAW(const graph& G, const node_array<TString>& name, const edge_array<TString>& nameE, const char* filename);
}// end namespace

// 반드시 for_... 과 end_for 쌍을 사용할 것.
#define end_for }}
#define for_all_node for_all_nodes
#define forall_nodes for_all_nodes
#define for_all_edge for_all_edges
#define for_all_adj_edges for_outgoing_edges
#define for_all_nodes(v,g)\
{	TList<LEDA_deprecated::node_struct>::iterator _i_node;\
	for (_i_node=(g).m_aV.begin(), (v)=&(*_i_node); _i_node!=(g).m_aV.end(); ++_i_node,(v)=&(*_i_node)) {
#define for_all_edges(e,g)\
{	TList<LEDA_deprecated::edge_struct>::iterator _i_edge;\
	for (_i_edge=(g).m_aE.begin(), (e)=&(*_i_edge); _i_edge!=(g).m_aE.end(); ++_i_edge,(e)=&(*_i_edge)) {
#define for_outgoing_edges(e,v)\
{	TList<LEDA_deprecated::edge_struct>::iterator _i_edge;\
	for(_i_edge=(v)->outEdge().begin(), (e)=&(*_i_edge); _i_edge!=(v)->outEdge().end(); ++_i_edge, (e)=&(*_i_edge)){

/*
forall_adj_nodes(v, w) { ... }
iterates over all nodes v that are adjacent to the node w. In the case of a directed graph, these are all v for which there is an edge (w,v) in the graph. In the undirected case, these are all v for which there is an edge (w,v) or an edge (v,w). */

#define forall_adj_nodes(v,w)\
{	TList<LEDA_deprecated::edge_struct>::iterator _i_edge;\
	for(_i_edge=(w)->outEdge().begin(); _i_edge!=(w)->outEdge().end(); ++_i_edge)\
	{\
		(v)=(*_i_edge).target();