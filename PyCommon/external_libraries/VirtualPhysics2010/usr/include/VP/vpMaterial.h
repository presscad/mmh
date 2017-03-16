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

#ifndef VP_MATERIAL
#define VP_MATERIAL

#include <VP/vpDataType.h>

/*!
	\class vpMaterial
	\brief Material properties of rigid bodies
	
	vpMaterial is a class to define material properties of rigid bodies.
*/
class vpMaterial
{
	friend class			 vpBody;

public:

							 vpMaterial();

	/*!
		get a density of the material.
	*/
	scalar					 GetDensity(void) const;

	/*!
		set a density of the material.
	*/
	void					 SetDensity(scalar);

	/*!
		get a restitution parameter of the material.
	*/
	scalar					 GetRestitution(void) const;

	/*!
		set a restitution parameter of the material.
		\param e If 1, perfectly elastic. If 0, perfectly plastic.
	*/
	void					 SetRestitution(scalar e);

	/*!
		get a static friction parameter of the material.
	*/
	scalar					 GetStaticFriction(void) const;

	/*!
		set a static friction parameter of the material.
	*/
	void					 SetStaticFriction(scalar);

	/*!
		get a dynamic friction parameter of the material.
	*/
	scalar					 GetDynamicFriction(void) const;

	/*!
		set a dynamic friction parameter of the material.
	*/
	void					 SetDynamicFriction(scalar);

	/*!
		get a spinning friction parameter of the material.
	*/
	scalar					 GetSpinningFriction(void) const;

	/*!
		set a spinning friction parameter of the material.
	*/
	void					 SetSpinningFriction(scalar);

	/*!
		get a default material used for bodies which do not have their own materials.
	*/
	static vpMaterial		*GetDefaultMaterial(void);

	string					 m_szName;

protected:

	scalar					 m_rDensity;
	scalar					 m_rRestitution;
	scalar					 m_rStaticFriction;
	scalar					 m_rDynamicFriction;
	scalar					 m_rSpinningFriction;
	static vpMaterial		*m_pDefaultMaterial;
};

#ifndef VP_PROTECT_SRC
	#include "vpMaterial.inl"
#endif

#endif
