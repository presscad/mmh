#include <VP/vphysics.h>
#include "../../vpRenderer/vpBasicRenderer.h"

vpWorld		world;
vpBody		ground, pendulum_left, pendulum_right;
vpBody box[20];
vpRJoint	joint;
vpRJoint	joint_right;


void initialize(void)
{
	VP_BASIC_RENDERER_WORLD(world);
	set_view(1.00, -0.00, 0.01, 0.00, 0.01, 0.09, -1.00, 0.00, 0.00, 1.00, 0.09, 0.00, 0.00, 0.00, -14.51, 1.00);

	

	for(int i=0; i<5; i++)
	{
		box[i].AddGeometry(new vpBox(Vec3(0.5)));
		box[i].SetFrame(Vec3(0,0,i+0.6));
	}
	
	ground.SetGround();
	ground.AddGeometry(new vpBox(Vec3(30, 30, 0.5)));

	pendulum_left.AddGeometry(new vpSphere(0.5));
	pendulum_left.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));

	pendulum_right.AddGeometry(new vpSphere(0.5));
	pendulum_right.AddGeometry(new vpCapsule(0.2, 4), Vec3(0, 0, 2));
	
	pendulum_left.SetJoint(&joint, Vec3(0, 0, 5));
	ground.SetJoint(&joint, Vec3(1, 0, 0));
	joint.SetAxis(Vec3(0, 1, 0));
	joint.SetAngle(0.5);

	pendulum_right.SetJoint(&joint_right, Vec3(0, 0, 5));
	ground.SetJoint(&joint_right, Vec3(-1, 0, 0));
	joint_right.SetAxis(Vec3(0, 1, 0));
	joint_right.SetAngle(-0.5);

	for(int i=0; i<5; i++)
		world.AddBody(&box[i]);

	world.AddBody(&ground);
	

	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.Initialize();
}

void frame(void)
{
	//for(int i=0; i<5; i++)
	//	world.StepAhead();
	printf(10, 10, "t = %3.2f   E = %3.2f(Ek = %3.2f   Ev = %3.2f)", world.GetSimulationTime(), world.GetKineticEnergy() + world.GetPotentialEnergy(), world.GetKineticEnergy(), world.GetPotentialEnergy());
	printf(10, 30, "angle = %3.2f   g = (%3.2f, %3.2f, %3.2f)", joint.GetAngle(), world.GetGravity()[0], world.GetGravity()[1], world.GetGravity()[2]);
	printf(10, 480, "press SPACEBAR to change gravity direction");
}

void keyboard(unsigned char key, int x, int y)
{
	if ( key == ' ' ) world.SetGravity(-1.0 * world.GetGravity());
	if ( key == 'k' ) world.StepAhead();
}
