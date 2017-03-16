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

#include <VP/vphysics.h>
#include "../../vpRenderer/vpBasicRenderer.h"

#define NUM_BALL	30
#define NUM_RAGDOLL	3
#define NUM_ROPE	5

struct vpRagdoll
{
	vpBody			B_head, B_trunk, B_pelvis, B_upper_arm[2], B_lower_arm[2], B_thigh[2], B_calf[2];
	vpUJoint		J_neck, J_hip[2];
	vpBJoint		J_waist, J_shoulder[2];
	vpRJoint		J_elbow[2], J_knee[2];

	vpBody			G, B_rope[NUM_ROPE+1];
	vpUJoint		J_rope[NUM_ROPE+1];

	// when hanging, 19 DOF character + 12 DOF rope = 31 DOF
	// when falling, 25 DOF character + 10 DOF rope = 35 DOF
	void Create(vpWorld *pWorld)
	{
		scalar neck_elasticity = 50;
		scalar waist_elasticity = 50;
		scalar shoulder_elasticity = 50;
		scalar elbow_elasticity = 50;
		scalar hip_elasticity = 50;
		scalar knee_elasticity = 50;

		scalar rope_damping = 10.0;
		scalar neck_damping = 10.0;
		scalar waist_damping = 10.0;
		scalar shoulder_damping = 10.0;
		scalar elbow_damping = 10.0;
		scalar hip_damping = 10.0;
		scalar knee_damping = 10.0;

		vpMaterial::GetDefaultMaterial()->SetRestitution(0.3);
		vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.3);

		// rope
		pWorld->AddBody(&G);
		G.SetGround();
		G.SetJoint(&J_rope[0], Vec3(0, 0, 2 * NUM_ROPE + 15));
		for ( int i = 0; i < NUM_ROPE; i++ )
		{
			B_rope[i].SetJoint(&J_rope[i], Vec3(0, 0, 1));
			B_rope[i].SetJoint(&J_rope[i+1], Vec3(0, 0, -1));
			J_rope[i].SetDamping(0, rope_damping);
			J_rope[i].SetDamping(1, rope_damping);
			B_rope[i].AddGeometry(new vpCapsule(0.2, 2.4));
			pWorld->IgnoreCollision(&B_rope[i], &B_rope[i+1]);
		}

		B_trunk.SetFrame(Vec3(0, 0, 13));
		B_trunk.SetJoint(&J_rope[NUM_ROPE], Vec3(0, -1.5, -1.5));
		J_rope[NUM_ROPE].SetDamping(0, rope_damping);
		J_rope[NUM_ROPE].SetDamping(1, rope_damping);
		
		// ragdoll
		B_trunk.SetFrame(Vec3(0, 0, 13));
		B_trunk.AddGeometry(new vpBox(Vec3(2.4, 1.8, 3)));
		B_trunk.SetJoint(&J_neck, Vec3(0, 0, 2));
		J_neck.SetElasticity(0, neck_elasticity);
		J_neck.SetDamping(0, neck_damping);
		J_neck.SetElasticity(1, neck_elasticity);
		J_neck.SetDamping(1, neck_damping);
	
		B_head.SetJoint(&J_neck, Vec3(0, 0, -1));
		B_head.AddGeometry(new vpSphere(1.0));
	
		B_trunk.SetJoint(&J_waist, Vec3(0, 0, -2));
		B_pelvis.SetJoint(&J_waist, Vec3(0, 0, 0.8));
		J_waist.SetElasticity(SpatialSpring(waist_elasticity));
		J_waist.SetDamping(SpatialDamper(waist_damping));
		B_pelvis.AddGeometry(new vpBox(Vec3(2.4, 1.8, 1.8)));
		
		// left side
		B_trunk.SetJoint(&J_shoulder[0], Vec3(1.8, 0, 1.5));
		B_upper_arm[0].SetJoint(&J_shoulder[0], Vec3(0, 0, 1.2));
		J_shoulder[0].SetElasticity(SpatialSpring(shoulder_elasticity));
		J_shoulder[0].SetDamping(SpatialDamper(shoulder_damping));
		J_shoulder[0].SetOrientation(EulerZYX(Vec3(0, -0.5, 0)));
		B_upper_arm[0].AddGeometry(new vpCapsule(0.6, 2.5));
	
		B_upper_arm[0].SetJoint(&J_elbow[0], Vec3(0, 0, -1.5));
		B_lower_arm[0].SetJoint(&J_elbow[0], Vec3(0, 0, 1.2));
		J_elbow[0].SetAxis(Vec3(1, 0, 0));
		J_elbow[0].SetElasticity(elbow_elasticity);
		J_elbow[0].SetDamping(elbow_damping);
		B_lower_arm[0].AddGeometry(new vpCapsule(0.6, 2.5));
		B_lower_arm[0].AddGeometry(new vpSphere(0.7), Vec3(0, 0, -1.5));

		pWorld->IgnoreCollision(&B_pelvis, &B_thigh[0]);
		B_pelvis.SetJoint(&J_hip[0], Vec3(1.5, 0, -1));
		B_thigh[0].SetJoint(&J_hip[0], Vec3(0, 0, 1.7));
		J_hip[0].SetAngle(1, -0.2);
		J_hip[0].SetElasticity(0, hip_elasticity);
		J_hip[0].SetDamping(0, hip_damping);
		J_hip[0].SetElasticity(1, hip_elasticity);
		J_hip[0].SetDamping(1, hip_damping);
		J_hip[0].SetUpperLimit(0, 1.5);
		J_hip[0].SetLowerLimit(0, -1.5);
		J_hip[0].SetUpperLimit(1, 1.0);
		J_hip[0].SetLowerLimit(1, -1.0);
		J_hip[0].SetRestitution(0, 0.5);
		J_hip[0].SetRestitution(1, 0.5);
		B_thigh[0].AddGeometry(new vpCapsule(0.7, 3.0));

		B_thigh[0].SetJoint(&J_knee[0], Vec3(0, 0, -1.7));
		B_calf[0].SetJoint(&J_knee[0], Vec3(0, 0, 2));
		J_knee[0].SetAxis(Vec3(1, 0, 0));
		J_knee[0].SetElasticity(knee_elasticity);
		J_knee[0].SetDamping(knee_damping);
		J_knee[0].SetUpperLimit(0.1);
		J_knee[0].SetLowerLimit(-1.0);
		B_calf[0].AddGeometry(new vpCapsule(0.7, 3.3));
		B_calf[0].AddGeometry(new vpBox(Vec3(1.5, 2.4, 0.5)), Vec3(0, 0.6, -2));

		// right side
		B_trunk.SetJoint(&J_shoulder[1], Vec3(-1.8, 0, 1.5));
		B_upper_arm[1].SetJoint(&J_shoulder[1], Vec3(0, 0, 1.2));
		J_shoulder[1].SetElasticity(SpatialSpring(shoulder_elasticity));
		J_shoulder[1].SetDamping(SpatialDamper(shoulder_damping));
		J_shoulder[1].SetOrientation(EulerZYX(Vec3(0, 0.5, 0)));
		B_upper_arm[1].AddGeometry(new vpCapsule(0.6, 2.5));
	
		B_upper_arm[1].SetJoint(&J_elbow[1], Vec3(0, 0, -1.5));
		B_lower_arm[1].SetJoint(&J_elbow[1], Vec3(0, 0, 1.2));
		J_elbow[1].SetAxis(Vec3(1, 0, 0));
		J_elbow[1].SetElasticity(elbow_elasticity);
		J_elbow[1].SetDamping(elbow_damping);
		B_lower_arm[1].AddGeometry(new vpCapsule(0.6, 2.5));
		B_lower_arm[1].AddGeometry(new vpSphere(0.7), Vec3(0, 0, -1.5));

		pWorld->IgnoreCollision(&B_pelvis, &B_thigh[1]);
		B_pelvis.SetJoint(&J_hip[1], Vec3(-1.5, 0, -1));
		B_thigh[1].SetJoint(&J_hip[1], Vec3(0, 0, 1.7));
		J_hip[1].SetAngle(1, 0.2);
		J_hip[1].SetElasticity(0, hip_elasticity);
		J_hip[1].SetDamping(0, hip_damping);
		J_hip[1].SetElasticity(1, hip_elasticity);
		J_hip[1].SetDamping(1, hip_damping);
		J_hip[1].SetUpperLimit(0, 1.5);
		J_hip[1].SetLowerLimit(0, -1.5);
		J_hip[1].SetUpperLimit(1, 1.0);
		J_hip[1].SetLowerLimit(1, -1.0);
		J_hip[1].SetRestitution(0, 0.5);
		J_hip[1].SetRestitution(1, 0.5);
		B_thigh[1].AddGeometry(new vpCapsule(0.7, 3.0));

		B_thigh[1].SetJoint(&J_knee[1], Vec3(0, 0, -1.7));
		B_calf[1].SetJoint(&J_knee[1], Vec3(0, 0, 2));
		J_knee[1].SetAxis(Vec3(1, 0, 0));
		J_knee[1].SetElasticity(knee_elasticity);
		J_knee[1].SetDamping(knee_damping);
		J_knee[1].SetUpperLimit(0.1);
		J_knee[1].SetLowerLimit(-1.0);
		B_calf[1].AddGeometry(new vpCapsule(0.7, 3.3));
		B_calf[1].AddGeometry(new vpBox(Vec3(1.5, 2.4, 0.5)), Vec3(0.0, 0.6, -2.0));
	}
};

vpWorld			 world;
vpBody			 ground;
vpRagdoll		 ragdoll[NUM_RAGDOLL][NUM_RAGDOLL];
vpBody			 ball[NUM_BALL];

void initialize(void)
{
	VP_BASIC_RENDERER_WORLD(world);
	set_view(-0.73, 0.33, -0.59, 0.00, -0.68, -0.41, 0.61, 0.00, -0.04, 0.85, 0.52, 0.00, -1.19, -6.59, -87.06, 1.00);

	world.SetGravity(Vec3(0.0, 0.0, -10.0));

	ground.SetGround();
	ground.AddGeometry(new vpBox(Vec3(100.0, 100.0, 5.0)), Vec3(0.0, 0.0, -5.0));
	world.AddBody(&ground);
	
	for ( int i = 0; i < NUM_RAGDOLL; i++ )
	for ( int j = 0; j < NUM_RAGDOLL; j++ )
	{
		ragdoll[i][j].Create(&world);
		ragdoll[i][j].G.SetFrame(Vec3(12 * (j - NUM_RAGDOLL / 2), 8 * (i - NUM_RAGDOLL / 2), 0));
	}

	for ( int i = 0; i < NUM_BALL; i++ )
	{
		ball[i].AddGeometry(new vpSphere(1));
		world.AddBody(&ball[i]);
		ball[i].SetFrame(Vec3(0, 3*i, -100));
		ball[i].SetInertia(Inertia(5.0));
	}

	world.SetNumThreads(0);
	world.SetIntegrator(VP::IMPLICIT_EULER_FAST);
	world.SetTimeStep(0.005);

	world.Initialize();
	world.BackupState();
}

void frame(void)
{
	for ( int i = 0; i < 5; i++ )
		world.StepAhead();

	printf(10, 10, "press SPACEBAR to shoot a ball");
	printf(10, 30, "press 'r' to reset");
	printf(10, 50, "press 'b' to break joints");
}

void keyboard(unsigned char key, int x, int y)
{
	if ( key == 'r' ) world.RollbackState();
	if ( key == ' ' )
	{
		static int ball_id = 0;
		ball[ball_id].SetFrame(Vec3(drand(-6.0 * NUM_RAGDOLL, 6.0 * NUM_RAGDOLL), 30.0, drand(10.0, 20.0)));
		ball[ball_id].SetVelocity(Vec3(0, -30, 5));
		(++ball_id) %= NUM_BALL;
	}
	if ( key == 'b' )
	{
		static int jidx = 0;
		static int kidx = 0;
		ragdoll[jidx][kidx].J_rope[NUM_ROPE].Break();
		kidx++;
		if ( kidx == NUM_RAGDOLL )
		{
			jidx++;
			if ( jidx == NUM_RAGDOLL ) jidx = 0;
			kidx = 0;
		}
	}
}
