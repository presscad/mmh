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

vpWorld		world;
vpBody		ground, pendulum_left, pendulum_right;
vpRJoint	joint;
vpRJoint	joint_right;
vpSpring	spring;

double spring_dist=2;

void initialize(void)
{
	VP_BASIC_RENDERER_WORLD(world);
	set_view(1.00, -0.00, 0.01, 0.00, 0.01, 0.09, -1.00, 0.00, 0.00, 1.00, 0.09, 0.00, 0.00, 0.00, -14.51, 1.00);

	
	ground.SetGround();
	ground.AddGeometry(new vpBox(Vec3(3, 3, 0.5)));

	pendulum_left.AddGeometry(new vpSphere(0.5));
	pendulum_left.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));

	pendulum_right.AddGeometry(new vpSphere(0.5));
	pendulum_right.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	
	spring.SetInitialDistance(2.0);
	spring.SetDamping(1);
	spring.SetElasticity(10.0);
	spring.Connect(&pendulum_right, &pendulum_left, Vec3(0,0,0), Vec3(0,0,0));

	
	pendulum_left.SetJoint(&joint, Vec3(0, 0, 5));
	ground.SetJoint(&joint, Vec3(1, 0, 0));
	joint.SetAxis(Vec3(0, 1, 0));
	joint.SetAngle(0.5);

	pendulum_right.SetJoint(&joint_right, Vec3(0, 0, 5));
	ground.SetJoint(&joint_right, Vec3(-1, 0, 0));
	joint_right.SetAxis(Vec3(0, 1, 0));
	joint_right.SetAngle(-0.5);

	world.AddBody(&ground);
	world.AddBody(&pendulum_left);
	world.AddBody(&pendulum_right);

	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.Initialize();
}

void frame(void)
{
	world.StepAhead();
	printf(10, 10, "t = %3.2f   E = %3.2f(Ek = %3.2f   Ev = %3.2f)", world.GetSimulationTime(), world.GetKineticEnergy() + world.GetPotentialEnergy(), world.GetKineticEnergy(), world.GetPotentialEnergy());
	printf(10, 30, "angle = %3.2f   g = (%3.2f, %3.2f, %3.2f)", joint.GetAngle(), world.GetGravity()[0], world.GetGravity()[1], world.GetGravity()[2]);
	printf(10, 480, "press SPACEBAR to change gravity direction");
}

void keyboard(unsigned char key, int x, int y)
{
	if ( key == ' ' ) world.SetGravity(-1.0 * world.GetGravity());
	if ( key == 's' ) spring.SetInitialDistance((spring_dist+=0.5));
}
