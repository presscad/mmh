#include "stdafx.h"
#include <math.h>

#ifdef WIN32
#include <windows.h>
#endif


#include "LinearR3.h"
#include "MathMisc.h"
#include "Node.h"

using namespace IK_sdls;
extern int RotAxesOn;

Node::Node(const VectorR3& attach, const VectorR3& v, double size, Purpose purpose, double minTheta, double maxTheta, double restAngle)
{
	Node::freezed = false;
	Node::size = size;
	Node::purpose = purpose;
	seqNumJoint = -1;
	seqNumEffector = -1;
	Node::attach = attach;		// Global attachment point when joints are at zero angle
	r.Set(0.0, 0.0, 0.0);		// r will be updated when this node is inserted into tree
	Node::v = v;				// Rotation axis when joints at zero angles
	theta = 0.0;
	Node::minTheta = minTheta;
	Node::maxTheta = maxTheta;
	Node::restAngle = restAngle;
	left = right = realparent = 0;
}

// Compute the global position of a single node
void Node::ComputeS(void)
{
	Node* y = this->realparent;
	Node* w = this;
	s = r;							// Initialize to local (relative) position
	while ( y ) {
		s.Rotate( y->theta, y->v );
		y = y->realparent;
		w = w->realparent;
		s += w->r;
	}
}

// Compute the global rotation axis of a single node
void Node::ComputeW(void)
{
	Node* y = this->realparent;
	w = v;							// Initialize to local rotation axis
	while (y) {
		w.Rotate(y->theta, y->v);
		y = y->realparent;
	}
}


void Node::PrintNode(int depth)
{
	for(int i=0; i<depth; i++) cerr << " ";
	cerr << seqNumJoint<< ": Attach : (" << attach << ")\n";
	for(int i=0; i<depth; i++) cerr << " ";
	cerr << "r : (" << r << ")\n";
	for(int i=0; i<depth; i++) cerr << " ";
	cerr << "s : (" << s << ")\n";
	for(int i=0; i<depth; i++) cerr << " ";
	cerr << "w : (" << w << ")\n";
	if(realparent)
	{
		for(int i=0; i<depth; i++) cerr << " ";
		cerr << "realparent : " << realparent->seqNumJoint << "\n";
	}
}

void Node::InitNode()
{
	theta = 0.0;
}
