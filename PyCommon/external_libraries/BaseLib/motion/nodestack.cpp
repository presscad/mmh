// NodeStack.cpp: implementation of the NodeStack class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "NodeStack.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NodeStack::NodeStack()
{
	top = -1;
}

void NodeStack::Initiate()
{
	top = -1;
}

NodeStack::~NodeStack()
{
}

void NodeStack::Push(Node* src, AlzzaCTransform *dst)
{
	top++;
	assert(top<NODE_STACK_SIZE);
	ASSERT(top<NODE_STACK_SIZE);
	stack[top].src= src;
	stack[top].dst= (AlzzaTransform*)dst;
}

void NodeStack::Pop(Node** psrc, AlzzaCTransform** pdst)
{
	if(top==-1) *psrc=NULL;
	else
	{
		*psrc=stack[top].src;
		*pdst=(AlzzaCTransform*)stack[top].dst;
		top--;
	}
}


void NodeStack::Push(Node* src, AlzzaTransform *dst)
{
	top++;
	ASSERT(top<NODE_STACK_SIZE);
	stack[top].src= src;
	stack[top].dst= dst;
}

void NodeStack::Pop(Node** psrc, AlzzaTransform** pdst)
{
	if(top==-1) *psrc=NULL;
	else
	{
		*psrc=stack[top].src;
		*pdst=stack[top].dst;
		top--;
	}
}

void NodeStack::GetTopNth(Node** psrc, AlzzaTransform **pdst, int n)
{
	ASSERT(n>=0 && top-n>=0);
	*psrc=stack[top-n].src;
	*pdst=stack[top-n].dst;
}