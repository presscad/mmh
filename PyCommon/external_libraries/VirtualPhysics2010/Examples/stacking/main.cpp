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

#define NUM_BLOCK_WIDTH		3
#define NUM_BLOCK_HEIGHT	7
#define NUM_BALL			10

vpWorld		world;
vpBody		ground;
vpBody		ball[NUM_BALL];
vpBody		block[NUM_BLOCK_WIDTH][NUM_BLOCK_HEIGHT * (NUM_BLOCK_HEIGHT + 1) / 2];
	
void initialize(void)
{
	VP_BASIC_RENDERER_WORLD(world);
	set_view(0.59, -0.34, 0.73, 0.00, 0.80, 0.21, -0.56, 0.00, 0.04, 0.91, 0.41, 0.00, 0.68, -2.10, -21.68, 1.00);

	// increase restitution for more bouncing
	vpMaterial::GetDefaultMaterial()->SetRestitution(0.1);
	
	// decrease dynamic friction for more slippery motion
	vpMaterial::GetDefaultMaterial()->SetDynamicFriction(0.3);
		
	for ( int i = 0; i < NUM_BLOCK_WIDTH; i++ )
	{
		int n = 0;
		for ( int j = 0; j < NUM_BLOCK_HEIGHT; j++ )
		{
			for ( int k = 0; k <= j; k++, n++ )
			{
				block[i][n].AddGeometry(new vpBox(Vec3(1.0)));
				block[i][n].SetFrame(Vec3(1.0 * (k - j * 0.5), 3.0 * (i - NUM_BLOCK_WIDTH / 2), 1.0 * (NUM_BLOCK_HEIGHT - j) - 0.5));
				world.AddBody(&block[i][n]);
			}
		}
	}

	ground.AddGeometry(new vpBox(Vec3(5.0 * max(NUM_BLOCK_WIDTH, NUM_BLOCK_HEIGHT), 5.0 * max(NUM_BLOCK_WIDTH, NUM_BLOCK_HEIGHT), 1.0)), Vec3(0.0, 0.0, -0.5));
	ground.SetGround();
	world.AddBody(&ground);

	for ( int i = 0; i < NUM_BALL; i++ )
	{
		ball[i].AddGeometry(new vpSphere(drand(0.5, 1.0)));
		ball[i].SetFrame(Vec3(0.0, 3 * i, -20.0));
		world.AddBody(&ball[i]);
	}

	world.SetTimeStep(0.005);
	world.SetGravity(Vec3(0.0, 0.0, -10.0));
	world.Initialize();
	world.BackupState();
}

void frame(void)
{
	for ( int i = 0; i < 5; i++ )
		world.StepAhead();

	printf(10, 10, "press 'r' to reset");
	printf(10, 30, "press SPACEBAR to shoot a ball");
}

void keyboard(unsigned char key, int x, int y)
{
	if ( key == 'r' ) world.RollbackState();
	if ( key == ' ' )
	{
		static int ball_id = 0;
		ball[ball_id].SetFrame(Vec3(drand(2.0), -10.0, drand(2.0, 5.0)));
		ball[ball_id].SetVelocity(Vec3(0, 50, 5));
		(++ball_id) %= NUM_BALL;
	}
}
