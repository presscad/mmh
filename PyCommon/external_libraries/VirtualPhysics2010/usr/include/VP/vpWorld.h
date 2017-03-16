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

#ifndef VP_WORLD
#define VP_WORLD

#include <VP/vpDataType.h>
#include <VP/vpTimer.h>
#include <string>
#include <map>

/*!
	\class vpWorld
	\brief world
	
	vpWorld is a class to model your virtual world goverened by physics law.
*/
class vpWorld
{
	friend class						 vpSystem;
	friend class						 vpSingleSystem;
	friend class						 vpSingleGroundSystem;
	friend class						 vpBody;
	friend class						 vpJoint;

public :
										 vpWorld();

	/*!
		add a body to the world
	*/
	void								 AddBody(vpBody *);

	/*!
		add a world

		World's parameters such as a time step and the gravity will not be updated with newly added world'parameters.
		Only the body and joints will be added.
	*/
	void								 AddWorld(vpWorld *);

	/*!
		set a global frame

		All the coordinate frame is represented by this global frame.
		Should be followed by vpWorld::Initialize.
	*/
	void								 SetGlobalFrame(const SE3 &);
	
	/*!
		set a global frame
	*/
	const SE3							&GetGlobalFrame(void) const;

	/*!
		initialize the world.
		Before simulation, the world should be initialized.
	*/
	void								 Initialize(void);

	/*!
		simulate the world during a specified time step.
		\sa SetTimeStep()
	*/
	virtual void						 StepAhead(void);

	/*!
		set simulation time step used in integrating dynamics equations.
		Default time step is 0.001. For stable simulation, smaller time step is preferred.
		However how small the time step can depend on your model. 
	*/
	void								 SetTimeStep(scalar);

	/*!
		get a current time step.
	*/
	scalar								 GetTimeStep(void) const;

	/*!
		choose an integration algorithm used in simulation.
		\param type VP::EULER is faster but less accurate than VP::RK4.
	*/
	void								 SetIntegrator(VP::INTEGRATOR_TYPE type);

	/*!
		get a bounding sphere including whole world.
	*/
	scalar								 GetBoundingSphere(Vec3 &center) const;

	/*!
		set a gravity. Default is zero gravity.
	*/
	void								 SetGravity(const Vec3 &);

	/*!
		get a gravity.
	*/
	const Vec3							&GetGravity(void) const;

	/*!
		enable or disable collision between bodies.
		\sa vpWorld::IgnoreCollision()
	*/
	void								 EnableCollision(bool = true);

	/*!
		declare that collision of B0 and B1 will be ignored.
	*/
	void								 IgnoreCollision(vpBody *B0, vpBody *B1);

	/*!
		get a simulation time.
	*/
	scalar								 GetSimulationTime(void) const;

	/*!
		get a kinetic energy.
	*/
	scalar								 GetKineticEnergy(void) const;

	/*!
		get a potential energy.
	*/
	scalar								 GetPotentialEnergy(void) const;

	/*!
		get a total energy(= kinetic energy + potential energy).
	*/
	scalar								 GetTotalEnergy(void) const;

	/*!
		get a number of bodies in the world.
	*/
	int									 GetNumBody(void) const;

	/*!
		get a number of geometries in the world.
	*/
	int									 GetNumGeometry(void) const;

	/*!
		get a pointer to the ith body.

		\sa vpBody::GetID
	*/
	const vpBody						*GetBody(int) const;
	vpBody								*GetBody(int);

	/*!
		get a pointer to the body with the name
	*/
	const vpBody						*GetBodyByName(const string &name) const;
	
	/*!
		keep current states which are transformation matrices, velocities, joint angles, joint velocities.
		\sa vpWorld::RestoreState
	*/
	void								 BackupState(void);

	/*!
		restore states to the values kept by BackupState()
		\sa vpWorld::KeepCurrentState
	*/
	void								 RollbackState(void);

	/*!
		update transformation matrices from joint angles.
		
		It is useful when you change joint angles and want to compute corresponding transformation matrices of articulated bodies.
		Basically, VP does not compute transformation matrices of bodies even if you change the joint angles.
		The transformation matrices or relevant values will be updated after the simulation which is typically done by calling vpWorld::StepAhead().		
	*/
	void								 UpdateFrame(void);

	/*!
		get a number of materials defined in the world
	*/
	int									 GetNumMaterial(void) const;

	/*!
		get a pointer to the ith material
	*/
	const vpMaterial					*GetMaterial(int) const;

	/*!
		get a pointer to the material with the name
	*/
	const vpMaterial					*GetMaterialByName(const string &name) const;

	/*!
		get a number of joints in the world
	*/
	int									 GetNumJoint(void) const;

	/*!
		get a pointer to the ith joint
	*/
	const vpJoint						*GetJoint(int) const;

	/*!
		get a pointer to the joint with the name
	*/
	const vpJoint						*GetJointByName(const string &name) const;

	virtual								~vpWorld();
	
	/*!
		clear all the instances managed by the world
	*/
	void								 Clear(void);

	/*!
		print out current configuration of the wolrd in XML format.
	*/
	friend ostream						&operator << (ostream &, const vpWorld &);
	
	/*!
		read the configuration generated from output stream
	*/
	friend istream						&operator >> (istream &, vpWorld &);

	void								 report(ostream &);

	int									 GetNumCollision(void) const;
	int									 GetNumContact(void) const;

	void								 SetNumThreads(int);
	int									 GetNumThreads(void);

	void								 SetGlobalDamping(scalar);
	scalar								 GetGlobalDampling(void);

	int									 GetFrameCount(void) const;

	void								 ReportStatistics(void);
	void								 ResetStatistics(void);

protected :

	void								 FindAdjacentBodies(vpJoint *, vpBody *, vpBodyPtrArray &, vpJointPtrArray &, vpBodyPtrSet &);
	void								 BreakJoints(void);
	void								 SetContactEPS(void);
	void								 BuildCollisionGraph(void);
	void								 ResolveCollision(void);
	void								 ResolveContact(void);
	void								 RelaxPenetration(void);

	vpCollisionDetector					*m_pCollisionDetector;
	bool						 		 m_bDetectCollision;
	bool						 		 m_bIsInitialized;
	bool								 m_bFirstReportStatistics;
	unsigned int						 m_iFrameCount;
	unsigned int						 m_iResetFrameCount;
	int									 m_iNumThreads;
	int									 m_iMaxIterSolver;
	scalar						 		 m_rTime;
	scalar						 		 m_rContactEPS;
	scalar						 		 m_rTimeStep;
	scalar								 m_rGlobalDamping;
	Vec3						 		 m_sGravity;
	SE3									 m_sGlobalFrame;
	RMatrixArray				 		 m_sDelV;
	RMatrixArray				 		 m_sP;
	RMatrixArray				 		 m_sB;
	se3DbAry					 		 m_sUnitDelV;
	LUTIndexListAry						 m_sCollisionPairL, m_sContactPairL;
	LUTIndexDbAry						 m_sCollisionPair, m_sContactPair;
	vpSystemPtrDbAry			 		 m_pCollisionSystem, m_pContactSystem;
	vpSystemPtrArray			 		 m_pSystem;
	vpBodyPtrSet				 		 m_pRegisteredBody;
	vpBodyPtrArray				 		 m_pBody;
	vpJointPtrArray				 		 m_pJoint;
	vpSpringPtrArray			 		 m_pSpring;
	vpMaterialPtrArray			 		 m_pMaterial;
	map<string, const vpBody *>			 m_sBodyNameTable;
	map<string, const vpMaterial *>		 m_sMaterialNameTable;
	map<string, const vpJoint *>		 m_sJointNameTable;
	SMatrixArray						 m_sContactK;
	SMatrixArray						 m_sCollisionK;

#ifdef VP_PROFILING_STATISTICS
	vpTimer								 m_sStepAheadTimer;
	vpTimer								 m_sDynamicsTimer;
	vpTimer								 m_sColDetTimer;
	vpTimer								 m_sColGraphTimer;
	vpTimer								 m_sColResTimer;
	vpTimer								 m_sConResTimer;
	vpTimer								 m_sRelaxationTimer;
	vpTimer								 m_sProfilingTimer[10];

	int									 m_iMaxCollisionSize;
	vector<int>							 m_iCollisionSizeHistogram;
	int									 m_iNumCallSolveAxEqualB;
	int									 m_iSuccessfulSolveAxEqualB;
	int									 m_iNumCallSecondSolveAxEqualB;
	int									 m_iNumCollisionGroup;

	int									 m_iMaxContactSize;
	vector<int>							 m_iContactSizeHistogram;
	int									 m_iNumCallSORSolveLCP;
	int									 m_iSuccessfulSORSolveLCP;
	int									 m_iNumContactGroup;

#endif

	// Note that this function declaration MUST be placed at the end of this class declaration.
	// So Do NOT add memeber variables below here.
	void								(vpSystem::*IntegrateDynamics)(scalar);
	char								 memBoundary[1024];
};

#ifndef VP_PROTECT_SRC
	#include "vpWorld.inl"
#endif

#endif
