
#include "LinearR3.h"
#include "Node.h"

#ifndef _CLASS_TREE
#define _CLASS_TREE

namespace IK_sdls
{

class Tree {

public:
	Tree();

	int GetNumNode() const { return nNode; }
	int GetNumEffector() const { return nEffector; }
	int GetNumJoint() const { return nJoint; }
	void InsertRoot(IK_sdls::Node*);
	void InsertLeftChild(IK_sdls::Node* parent, IK_sdls::Node* child);
	void InsertRightSibling(IK_sdls::Node* parent, IK_sdls::Node* child);
	// by taesoo
	void InsertChild_automatic(IK_sdls::Node* parent, IK_sdls::Node* child);

	// Accessors based on node numbers
	IK_sdls::Node* GetJoint(int);
	IK_sdls::Node* GetEffector(int);
	const VectorR3& GetEffectorPosition(int);

	// Accessors for tree traversal
	IK_sdls::Node* GetRoot() const { return root; }
	IK_sdls::Node* GetSuccessor ( const IK_sdls::Node* ) const;
	IK_sdls::Node* GetParent( const IK_sdls::Node* node ) const { return node->realparent; }

	void Compute();
	void Print();
	void Init();
	void UnFreeze();

private:

	IK_sdls::Node* root;
	int nNode;			// nNode = nEffector + nJoint
	int nEffector;
	int nJoint;
	void SetSeqNum(IK_sdls::Node*);
	IK_sdls::Node* SearchJoint(IK_sdls::Node*, int);
	IK_sdls::Node* SearchEffector(IK_sdls::Node*, int);
	void ComputeTree(IK_sdls::Node*);
	void PrintTree(int depth, IK_sdls::Node*);
	void InitTree(IK_sdls::Node*);
	void UnFreezeTree(IK_sdls::Node*);
};

inline IK_sdls::Node* Tree::GetSuccessor ( const IK_sdls::Node* node ) const
{
	if ( node->left ) {
		return node->left;
	}
	while ( true ) {
		if ( node->right ) {
			return ( node->right );
		}
		node = node->realparent;
		if ( !node ) {
			return 0;		// Back to root, finished traversal
		} 
	}
}
}
#endif