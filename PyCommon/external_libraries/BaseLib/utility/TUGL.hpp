#pragma once
#include <vector>
#include <list>
#include "../baselib/utility/tlist.h" // -tlist dependency will be removed.
#include "../baselib/math/mathclass.h"
namespace TUGL
{
	// undirected graph
struct EmptyEdge
{
};

struct edge_struct;
struct node_struct
{
	node_struct():_index(-1), _data(NULL){}
	node_struct(int index):_data(NULL) { _index=index;}
	~node_struct()		{}
	
	int _index;			// internal name (index)  
	
	std::vector<edge_struct*> _aE;	//!< reference array로 초기화 함에 주의
	void* _data;
};

template <class NodeType, class EdgeType>
class edge;

template <class NodeType, class EdgeType=EmptyEdge>
class node
{
public:
	// always reference
	node_struct* _ptr;

	node(): _ptr(NULL){}
	node(node_struct* p):_ptr(p){}

	int index()	const		{ return _ptr->_index;}
	NodeType& data() const			{ return *((NodeType*)_ptr->_data);}
	NodeType* operator->() const	{ return &data();}

	edge<NodeType, EdgeType>& edge(int index) const;

	edge_struct* getEdgePtr(int index) const		{ return _edge()[index];}

	int  degree()    const	{ return _edge().size(); }

	inline operator node_struct*()					{ return _ptr;}
	friend bool operator==(node const& a, node const& b) 
	{
		return a._ptr==b._ptr;
	}

	// almost private: 
	const std::vector<edge_struct*>& _edge() const	{ return _ptr->_aE;}
};

struct edge_struct
{
	edge_struct():_index(-1),_s(NULL),_t(NULL),_data(NULL)	{}
	~edge_struct()						{}
	edge_struct(node_struct* v, node_struct* w, int index) {_index=index;_s=v;_t=w;}
	
	int  _index;          // internal name (index)  
	node_struct* _s;             // source node (a node with smaller index)
	node_struct* _t;             // target node
	void* _data;
};

template <class NodeType, class EdgeType=EmptyEdge>
class edge
{
public:
	edge():_ptr(NULL){}
	edge_struct* _ptr;

	node<NodeType, EdgeType> v1()    const { return _ptr->_s;}
	node<NodeType, EdgeType> v2()    const { return _ptr->_t;}
	node<NodeType, EdgeType> target(node<NodeType, EdgeType> v)
	{
		if(v==v1()) return v2();
		if(v==v2()) return v1();
		return node<NodeType,EdgeType>();
	}
	int index() const		{ return _ptr->_index;}
	
	EdgeType& data() const			{ return *((EdgeType*)_ptr->_data);}
	EdgeType* operator->() const	{ return &data();}

	inline operator edge_struct*()					{ return _ptr;}
	friend bool operator==(edge const& a, edge const& b) 
	{
		return a._ptr==b._ptr;
	}
};

template <class NodeType, class EdgeType>
edge<NodeType, EdgeType>& node<NodeType, EdgeType>::edge(int index) const
{
	edge<NodeType, EdgeType> e;
	e._ptr=getEdgePtr(index);
	return e;
}

/** 가정: 에지나 node가 추가로 생길수는 있지만 중간에 지워지지는 않는다. 그래프 자료구조에서는 상관없지만, node_array나 edge_array의 구현에서 빠른 indexing 속도와 쉬운 구현을 위해 array를 사용했기 때문. 나중에 이 가정을 없앨 필요가 있으면 아마도 interface는 유지한채로 재구현이 가능할 것이다.
*/
template <class NodeType, class EdgeType=EmptyEdge>
class graph {

	std::vector<node_struct*> m_aV;              //!< list of all nodes 
	std::vector<edge_struct*> m_aE;              //!< list of all edges

public:
	graph();
	virtual ~graph()		{ clear(); }
	virtual void clear();

	int  numNodes() const   { return m_aV.size(); }
	int  numEdges() const   { return m_aE.size(); }

	// if you know the number of nodes to be created, please reserve NodeArray for efficiency
	void reserveNodeArray(int numNodes)		{ m_aV.reserve(numNodes);}
	void reserveEdgeArray(int numEdges)		{ m_aE.reserve(numEdges);}

	/// 새 노드생성과 edge생성은 반드시 newNode, newEdge를 사용할것.
	node<NodeType, EdgeType> newNode();
	edge<NodeType, EdgeType> newEdge(node<NodeType, EdgeType> v, node<NodeType, EdgeType> w);
    edge<NodeType, EdgeType> findEdge(node<NodeType, EdgeType> v, node<NodeType, EdgeType> w);
	edge<NodeType, EdgeType> findEdge(int index) const { edge<NodeType, EdgeType> e; e._ptr=(edge_struct*)(m_aE[index]); return e;}
	node<NodeType, EdgeType> findNode(int index) const { node<NodeType, EdgeType> v; v._ptr=(node_struct*)(m_aV[index]); return v;}

	inline node_struct* getNodePtr(int index) const	{ return (node_struct*)m_aV[index];}
	inline edge_struct* getEdgePtr(int index) const	{ return (edge_struct*)m_aE[index];}


};

}// end namespace

#define TUGL_for_all_node TUGL_for_all_nodes
#define TUGL_forall_nodes TUGL_for_all_nodes
#define TUGL_for_all_edge TUGL_for_all_edges

#define TUGL_for_all_nodes(v,g)\
	for(int _tugl_i=0; _tugl_i<(g).numNodes() &&((v)._ptr=(g).getNodePtr(_tugl_i))!=NULL; _tugl_i++)

#define TUGL_for_all_edges(e,g)\
	for(int _tugl_i=0; _tugl_i<(g).numEdges() && ((e)._ptr=(g).getEdgePtr(_tugl_i))!=NULL; _tugl_i++)

#define TUGL_for_adj_edges(e,v)\
	for(int _tugl_i=0; _tugl_i<(v).degree() && ((e)._ptr=(v).getEdgePtr(_tugl_i))!=NULL; _tugl_i++)

/*
forall_adj_nodes(v, w) { ... }
iterates over all nodes v that are adjacent to the node w. In the case of a directed graph, these are all v for which there is an edge (w,v) in the graph. In the undirected case, these are all v for which there is an edge (w,v) or an edge (v,w). */

#define TUGL_forall_adj_nodes(v,w)\
	for(int _tugl_i=0; _tugl_i<(w).outdeg() && ((v)._ptr=(w).getOutEdgePtr(0)->_t)!=NULL; _tugl_i++)


#include <queue>
#include <stack>

namespace TUGL
{

template <class T, class TE>
graph<T, TE>::graph()
{
}

template <class T, class TE>
node<T,TE> graph<T, TE>::newNode()
{
	m_aV.resize(m_aV.size()+1);
	m_aV.back()=new node_struct(m_aV.size()-1);
	m_aV.back()->_data=new T();
	
	return m_aV.back();
}


template <class T, class TE>
void graph<T, TE>::clear()
{
	for(int i=0; i<m_aV.size(); i++)
		delete m_aV[i];
	m_aV.resize(0);

	for(int i=0; i<m_aE.size();i++)
		delete m_aE[i];

	m_aE.resize(0);

}


template <class T, class TE>
edge<T,TE> graph<T, TE>::newEdge(node<T,TE> v, node<T,TE> w)
{
	if(v.index()>w.index())
		std::swap(v,w);

#ifdef _DEBUG
	ASSERT(findNode(v.index())==v);
	ASSERT(findNode(w.index())==w);
	ASSERT(!findEdge(v,w));
#endif
	m_aE.resize(m_aE.size()+1);
	
	edge_struct* e=new edge_struct(v, w, m_aE.size()-1);
	e->_data=new TE();
	m_aE.back()=e;
	
	
	v._ptr->_aE.push_back(e);
	w._ptr->_aE.push_back(e);
	
	edge<T,TE> ee;
	ee._ptr=e;
	return ee; 
}


template <class T, class TE>
edge<T,TE> graph<T, TE>::findEdge(node<T,TE> v, node<T,TE> w)
{
	if(v.index()>w.index())
		std::swap(v,w);

	edge<T,TE> e;
	TUGL_for_adj_edges(e,v)
	{
		if(e.target(v)==w)
			return e;
	}
	
	e._ptr=NULL;
	return e;
}



}// end of namespace TUGL
