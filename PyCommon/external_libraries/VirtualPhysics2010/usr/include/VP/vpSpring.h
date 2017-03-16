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

#ifndef VP_SPRING
#define VP_SPRING

#include <VP/vpDataType.h>
/*!
	\class vpSpring
	\brief Elastic spring
	
	vpSpring is a class to model a elastic spring.
	It is a massless component to push or pull connecting bodies.
*/
class vpSpring
{
	friend class		 vpWorld;
	friend class		 vpSystem;
	friend class		 vpSingleSystem;

public:
						 vpSpring();

	/*!
		connect bodies B0 and B1 with the spring.
		\param p0 a position of the spring attached to B0. It is represented in a body fixed frame of B0.
		\param p1 a position of the spring attached to B1. It is represented in a body fixed frame of B1.
	*/
	void				 Connect(vpBody *B0, vpBody *B1, const Vec3 &p0, const Vec3 &p1);

	/*!
		set an elasticity of the spring.
	*/
	void				 SetElasticity(scalar);

	/*!
		set a damping parameter of the spring.
	*/
	void				 SetDamping(scalar);

	/*!
		set an initial distance of the spring.
	*/
	void				 SetInitialDistance(scalar);

	/*!
		get an elasticity of the spring.
	*/
	scalar				 GetElasticity(void) const;

	/*!
		get a damping parameter of the spring.
	*/
	scalar				 GetDamping(void) const;

	/*!
		get an initial distance of the spring.
	*/
	scalar				 GetInitialDistance(void) const;

	/*!
		remove the joint
	*/
	void				 Remove(void);

protected:

	void				 UpdateForce(void);
	scalar				 GetPotentialEnergy(void) const;

	vpBody				*m_pLeftBody;
	vpBody				*m_pRightBody;
	Vec3				 m_sLeftBodyPosition;
	Vec3				 m_sRightBodyPosition;
	scalar				 m_rPotentialEnergy;
	scalar				 m_rSpringCoef;
	scalar				 m_rDampingCoef;
	scalar				 m_rInitialDistance;
};

#ifndef VP_PROTECT_SRC
	#include "vpSpring.inl"
#endif

#endif
