#include "stdafx.h"
#include "graph.h"
#include <queue>
#include <stack>
namespace LEDA_deprecated
{

graph::graph()
{
	m_nNode=0;
	m_nEdge=0;
}

graph::graph(TFactory<node_struct>* pVFactory, TFactory<edge_struct>* pEFactory)
:m_aV(pVFactory),
m_aE(pEFactory)
{
}

node graph::newNode()
{
	m_aV.pushBack();
	m_aV.back()._init(m_nNode);
	m_nNode++;

	
	for(std::list<base_node_array*>::iterator i=m_listNodeArray.begin();
		i!=m_listNodeArray.end();
		++i)
	{
		(*i)->update(*this);
	}
	return &m_aV.back();
}

void graph::clear()
{
	m_aV.release();
	m_aE.release();

	m_nNode=0;
	m_nEdge=0;

	for(std::list<base_node_array*>::iterator i=m_listNodeArray.begin();
	i!=m_listNodeArray.end();
	++i)
	{
		if((*i)->mGraph==this) (*i)->mGraph=NULL;
	}

	for(std::list<base_edge_array*>::iterator i=m_listEdgeArray.begin();
		i!=m_listEdgeArray.end();
		++i)
	{
		if((*i)->mGraph==this) (*i)->mGraph=NULL;
	}

	m_listNodeArray.clear();
	m_listEdgeArray.clear();
}

edge graph::newEdge(node v, node w)
{
#ifdef _DEBUG
	ASSERT(findNode(v->index())==v);
	ASSERT(findNode(w->index())==w);
#endif
	m_aE.pushBack();
	edge e=&m_aE.back();
	e->_init(v, w, m_nEdge);
	m_nEdge++;
	v->_aOutgoingE.pushBack(e);
	w->_aIncomingE.pushBack(e);
	

	for(std::list<base_edge_array*>::iterator i=m_listEdgeArray.begin();
		i!=m_listEdgeArray.end();
		++i)
	{
		(*i)->update(*this);
	}
	
	return e ; 
}


edge graph::findEdge(node v, node w)
{
	edge e;
	for_outgoing_edges(e,v)
		if(e->target()==w)
			return e;
	end_for;
	return NULL;
}

node graph::findNode(int index) const
{
	LEDA_deprecated::node v;
	for_all_nodes(v, *this)
		if(v->index()==index)
			return v;
	end_for;
	return NULL;
}

bool BELLMAN_FORD(const graph& G, node s, const edge_array<float>& cost, 
                                                node_array<float>& dist, 
                                                node_array<edge>& pred ) 

/* single source shortest paths from s using a queue (breadth first search)
   computes for all nodes v:
   a) dist[v] = cost of shortest path from s to v
   b) pred[v] = predecessor edge of v in shortest paths tree
*/
{ 
	node_array<int> count;
	count.init(G);
	dist.init(G);
	pred.init(G);

	int n = G.numNodes();

	TList<node> Q;

	node u,v;
	edge e;

	for_all_nodes(v,G) 
		pred[v] = 0;
		dist[v] = FLT_MAX; 
	end_for;

	dist[s] = 0;
	Q.pushBack(s);

	while(! Q.empty() )
	{
		u = Q.popBack();

		if (++count[u] > n) return false;   // negative cycle

		float du = dist[u];

		for_all_adj_edges(e,u) 
			v = e->target();
			float c = du + cost[e];
			if (c < dist[v]) 
			{ 
				dist[v] = c; 
				pred[v] = e;
				if (!Q.member(v)) Q.pushBack(v);
			}
		end_for;
	}
  return true;
}

void PATH(node s, node t, const node_array<edge>& pred, TList<node>& path)
{
	path.init();
	
	for(node n=t; n!=s; n=pred[n]->source())
		path.pushFront(n);
	path.pushFront(s);
}

void PATH_EDGE(node s, node t, const node_array<edge>& pred, TList<edge>& path)
{
	path.init();
	edge e;
	for(e=pred[t]; e->source()!=s; e=pred[e->source()])
		path.pushFront(e);
	path.pushFront(e);
}

void DRAW(const graph& G, const node_array<TString>& name, const char* filename)
{
	
	TString tfilename;
	tfilename.format("../graph/%s", filename);
	FILE* dotfile;
	dotfile=fopen(tfilename+".dot", "wt");
	Msg::verify(dotfile, "%s.dot open failed", tfilename.ptr());

	fprintf(dotfile," digraph G {\n");
	node v;
	edge e;
	for_all_node(v, G)
	{
		fprintf(dotfile,"%d [label=\"%s\"];\n", v->index(), name[v].ptr());
	}
	end_for;

	for_all_edge (e, G)
	{
		fprintf(dotfile,"%d -> %d;\n", e->source()->index(), e->target()->index());
	}
	end_for;

	fprintf(dotfile,"}\n");
	fclose(dotfile);

	dotfile=fopen(tfilename+".bat", "wt");
	Msg::verify(dotfile, "%s.bat open failed", tfilename.ptr());

	fprintf(dotfile,"dot -Tjpg %s.dot -o %s.jpg\n", filename, filename);
	fprintf(dotfile,"dot -Tps %s.dot -o %s.ps\n", filename, filename);
	fclose(dotfile);
}

void DRAW(const graph& G, const node_array<TString>& name, const edge_array<TString>& nameE, const char* filename)
{
	TString tfilename;
	tfilename.format("../graph/%s", filename);
	FILE* dotfile;
	dotfile=fopen(tfilename+".dot", "wt");
	Msg::verify(dotfile, "%s.dot open failed", tfilename.ptr());

	fprintf(dotfile," digraph G {\n");
	node v;
	edge e;

	for_all_node(v, G)
	{
		fprintf(dotfile,"%d [label=\"%s\"];\n", v->index(), name[v].ptr());
	}
	end_for;

	for_all_edge (e, G)
	{
		fprintf(dotfile,"%d -> %d [label=\"%s\"];\n", e->source()->index(), e->target()->index(), nameE[e].ptr());
	}
	end_for;

	fprintf(dotfile,"}\n");
	fclose(dotfile);

	dotfile=fopen(tfilename+".bat", "wt");
	Msg::verify(dotfile, "%s.bat open failed", tfilename.ptr());

	fprintf(dotfile,"dot -Tjpg %s.dot -o %s.jpg\n", filename, filename);
	fprintf(dotfile,"dot -Tps %s.dot -o %s.ps\n", filename, filename);
	fclose(dotfile);
}

void DFS(const graph& G, node v, node_array<bool>& reached, TList<node>& L)
{ 
	std::stack<node> S;
	node w;

	if ( !reached[v] ) {
	reached[v] = true;
	S.push(v);
	}

	while ( !S.empty() ) {

		v = S.top();
		S.pop(); 
		L.pushBack(v);
		forall_adj_nodes(w,v) 
			if ( !reached[w] ) {
				reached[w] = true;
				S.push(w);
			}
		end_for;
	}
}

void BFS(const graph& G, node v, node_array<int>& dist)
{
	std::queue<node> Q;
	dist.init(G);
	node w;

	forall_nodes(w,G)
		dist[w] = -1;
	end_for;

	dist[v] = 0;
	Q.push(v);

	while ( !Q.empty() ) 
	{
		v = Q.front();
		Q.pop();
		forall_adj_nodes(w,v)
			if (dist[w] < 0) {
				Q.push(w); 
				dist[w] = dist[v]+1;
			}
		end_for;
	}
}

}
