// Node.h: interface for the Node class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
#define AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "my.h"
#include "../utility/TFile.h"
#include "../utility/TextFile.h"
#include <list>
/* TreeNode
*/
class Node  
{
public:
	Node();
	virtual ~Node();
	
	virtual void pack(BinaryFile& file, int nVersion) const;
	virtual void unpack(BinaryFile& file, int nVersion) ;
	void AddChild(Node* pChild);	

	void detachAllChildren(std::list<Node*>& children);
	void addChildren(std::list<Node*>& children);

	void SetNameId(const char* name);
	const char* GetNameId() const	{return NameId;}
	int CountChildren();

	void printHierarchy(int depth=0);

	char* NameId;	//!< Node의 이름을 갖는 경우 사용됨 
	NodeType_T NodeType;	//!< TransformNode인지 IndexedFaceSetNode인지 등을 구분한다.
	Node *m_pChildHead;	//!< 자식노드 list의 head, singly linked list이다. 
	Node *m_pChildTail; //!< 자식노드 list의 tail
	Node *m_pSibling;	//!< 자식노드들끼리 연결하는 pointer, 오른쪽 sibling을 뜻한다. 
	Node *m_pParent;
	int m_nIndex;
	int GetIndex() const	{return m_nIndex;}
};

#endif // !defined(AFX_NODE_H__85014E04_F8E9_11D3_B82A_00A024452D72__INCLUDED_)
