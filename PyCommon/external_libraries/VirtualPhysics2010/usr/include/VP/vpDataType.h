/*
VirtualPhysics v0.9

VirtualPhysics was developed as part of the project entitled "Development of 
Real-time Physics Simulation Engine for e-Entertainments" which was financially
supported by the grant from the strategic technology development program
(Project No. 2008-F-033-02) of both the MKE(Ministry of Knowledge Economy) and
MCST(Ministry of Culture, Sports and Tourism) of Korea.

Copyright (c) 2008-2010, Jinwook Kim, Korea Institute of Science and Technology
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, 
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
	  and/or other materials provided with the distribution.

   3. Only research partners of the project "Development of Real-time Physics 
      Simulation Engine for e-Entertainments" can modify this list of 
	  conditions and the following disclaimer with the consent of the author, 
	  where the research partners refer to all principal investigators 
	  involved in the project. 

THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#ifdef WIN32
#pragma warning(disable : 4996)
#endif

#ifndef VP_DATA_TYPE
#define VP_DATA_TYPE

#include <VP/rmatrix3.h>
#include <VP/smatrix.h>
#include <VP/LieGroup.h>
#include <VP/_array.h>
#include <set>
#include <vector>
#include <list>
#if !defined(__APPLE__) || defined(__APPLE_OMP__)
#include <omp.h>
#endif

// Defining VP_PROTECT_SRC macro will hide all inline functions to protect source codes.
//#define VP_PROTECT_SRC

// Defining VP_PROFILING_STATISTICS macro will generate statistical information for a profiling purpose at an overhead.
// To log or reset profiling statistical information, call vpWorld::ReportStatistics() or vpWorld::ResetStatistics() 
//#define VP_PROFILING_STATISTICS

class vpWorld;
class vpSystem;
class vpJoint;
class vpBody;
class vpSpring;
class vpState;
class vpGeom;
class vpMaterial;
class vpCollisionDetector;

//!  Predefined Enumeration in VirtualPhysics
namespace VP
{
	enum SIGN							{	PLUS = +1,	MINUS = -1	};

	enum SIDE							{	LEFT,		RIGHT		};

	/*!
		Types of integrator used in interating governing dynamics equations
	*/
	enum INTEGRATOR_TYPE
	{
		EULER,						/*!< explicit Euler method */
		RK4,						/*!< explicit Runge-Kutta 4th method */
		IMPLICIT_EULER,				/*!< implicit Euler method */
		IMPLICIT_EULER_FAST			/*!< approximated implicit Euler method */
	};

	/*!
		Status of a joint/body when solving hybrid dynamics
	*/
	enum HD_TYPE
	{
		KINEMATIC,			/*!< find torque for a given acceleration */
		DYNAMIC				/*!< find acceleration for a given torque */
	};

	void SetLogFileName(const char *);
	void LogInfo(const char *, ...);
}

struct vpBodyPair
{
	vpBodyPair()
	{
		pLeftBody = pRightBody = NULL;
	}
	
	vpBodyPair(vpBody *left, vpBody *right)
	{
		assert(left != right && "vpBodyPair -> identical body");
		pLeftBody = left > right ? left : right;
		pRightBody = left > right ? right : left;
	}

	bool operator < (const vpBodyPair &pair) const
	{
		assert(pLeftBody != NULL && pRightBody != NULL && pair.pLeftBody != NULL && pair.pRightBody != NULL && "vpBodyPair::operator = -> incomplete vpBodyPair");
		return pLeftBody == pair.pLeftBody ? pRightBody < pair.pRightBody : pLeftBody < pair.pLeftBody;
	}
	
	vpBody								*pLeftBody;
	vpBody								*pRightBody;
};

struct vpCollisionInfo
{
	Vec3								 point;			/*!< point of collision in a global frame */
	Vec3								 normal;		/*!< surface normal of the right body at the collision point in a global frame */
	scalar								 penetration;	/*!< penetration depth */

	Vec3								 leftPosition;
	Vec3								 rightPosition;
	Vec3								 tangentialVelocity;
	scalar								 collidingVelocity;
	scalar								 spinningVelocity;
	scalar								 contactForce;
};

/*!
	\struct	vpCollisionDSC
	\brief Collision Descriptor	
*/
struct vpCollisionDSC
{
	vpBody								*pLeftBody;		/*!< pointer to a body of collision */
	vpBody								*pRightBody;	/*!< pointer to the other body of collision */
	vpState								*pState;

	_array<vpCollisionInfo>				 colInfo;
};

struct LUTIndex
{
	int				first;
	int				second;
	set<int>		impactSet;
};

typedef _rmatrix<scalar>				 RMatrix;
typedef Inertia							 SpatialSpring;
typedef Inertia							 SpatialDamper;
typedef _array<bool>					 boolArray;
typedef _array<scalar>					 scalarArray;
typedef _array<se3>						 se3Array;
typedef _array<SE3>						 SE3Array;
typedef _array<dse3>					 dse3Array;
typedef _array<vpJoint *>				 vpJointPtrArray;
typedef _array<vpBody *>				 vpBodyPtrArray;
typedef _array<vpSpring *>				 vpSpringPtrArray;
typedef _array<vpState>					 vpStateArray;
typedef _array<vpJointPtrArray>			 vpJointPtrDbAry;
typedef _array<vpSystem *>				 vpSystemPtrArray;
typedef _array<const vpMaterial *>		 vpMaterialPtrArray;
typedef _array<RMatrix>					 RMatrixArray;
typedef _array<RMatrixArray>			 RMatrixDbAry;
typedef _array<vpCollisionDSC>			 vpColPairArray;
typedef _array<vpGeom *>				 vpGeomPtrArray;
typedef	_array<vpSystemPtrArray>		 vpSystemPtrDbAry;
typedef _array<se3Array>				 se3DbAry;
typedef _smatrix<scalar>				 SMatrix;
typedef _array<SMatrix>					 SMatrixArray;
typedef _array<vpCollisionInfo>			 vpCollisionInfoArray;

typedef set<vpBodyPair>					 vpBodyPairSet;
typedef std::list<int>						 intList;
typedef std::list<LUTIndex>					 LUTIndexList;
typedef _array<LUTIndexList>			 LUTIndexListAry;
typedef _array<_array<LUTIndex> >		 LUTIndexDbAry;
typedef set<vpBody *>					 vpBodyPtrSet;


#ifdef VP_PROTECT_SRC
	#define VP_INLINE
#else
	#define VP_INLINE inline
#endif

#ifdef VP_PROFILING_STATISTICS
	#define VP_TIMER_ACTION(timer, action) (timer.action());
#else
	#define VP_TIMER_ACTION(timer, action) ;
#endif

#endif
