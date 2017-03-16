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

#ifndef VP_GEOM
#define VP_GEOM

#include <VP/vpDataType.h>

class			 vpSphere;		// S
class			 vpBox;			// B
class			 vpCapsule;		// C
class			 vpPlane;		// P
class			 vpCylinder;	// L
class			 vpTorus;		// T

/*!
	\class vpGeom
	\brief Abstract class for primitive geometries
	
	vpGeom is an abstract class to represent geometric primitives.
	A vpBody consists of several vpGeom instances.
*/
class vpGeom
{
	friend class			 vpBody;

public:

							 vpGeom();

	/*!
		get a coordinate frame of the geometry w.r.t a body frame.
	*/
	const SE3				&GetLocalFrame(void) const;

	/*!
		get a coordinate frame of the geometry w.r.t a global frame.
	*/
	const SE3				&GetGlobalFrame(void) const;

	/*!
		get an inertia of the geometry.
	*/
	virtual Inertia			 GetInertia(scalar) const = 0;

	/*!
		get a radius of a bounding sphere.
	*/
	virtual	scalar			 GetBoundingSphereRadius(void) const = 0;
	
	/*!
		get a shape information.
	*/
	virtual void			 GetShape(char *, scalar *) const = 0;

	void					 UpdateGlobalFrame(void);
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const = 0;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const = 0;

	//ys
	virtual const vector<Vec3>& getVerticesLocal() { vector<Vec3> v; return v;}
	virtual const vector<Vec3>& getVerticesGlobal(){ vector<Vec3> v; return v;}

	virtual void draw(void) const {}

protected:

	SE3						 m_sLocalFrame;
	SE3						 m_sGlobalFrame;
	vpBody					*m_pBody;
};

/*!
	\class vpSphere
	\brief Sphere primitive geometry
	
	vpSphere is a class to model a sphere.
	<img src="\vp_site/sphere.gif">
*/
class vpSphere : public vpGeom
{
public:

	/*!
		default radius of the sphere is SCALAR_1_2.
	*/
							 vpSphere();
	/*!
		construct a sphere of the given radius.
	*/
							 vpSphere(scalar radius);
	/*!
		set a radius of the sphere.
	*/
	void					 SetRadius(scalar radius);

	/*!
		get a radius of the sphere.
	*/
	scalar					 GetRadius(void) const;

	/*!
		get a shape information.
		return type = 'S', data[0] = radius
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar density) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const;

protected:

	scalar					 m_rRadius;
};

/*!
	\class vpBox
	\brief Box primitive geometry
	
	vpBox is a class to model a rectangular box.
	<img src="\vp_site/box.gif">
*/
class vpBox : public vpGeom
{
public:

	/*!
		default size of the box is 1 X 1 X 1.
	*/
							 vpBox();

	/*!
		construct a box of the given size.
	*/
							 vpBox(const Vec3 &);

	/*!
		set a size of the box.
	*/
	void					 SetSize(const Vec3 &);

	/*!
		get a size of the box.
	*/
	Vec3					 GetSize(void) const;

	/*!
		get a shape information.
		return type = 'B', data[0] = size_x, data[1] = size_y, data[2] = size_z
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;

	const Vec3				&GetHalfSize(void) const;
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const;

protected:

	Vec3					 m_sHalfSize;
};

/*!
	\class vpCapsule
	\brief Capsule primitive geometry
	
	vpCapsule is a class to model a cyliner capped with hemispheres.
	<img src="\vp_site/capsule.gif">
*/
class vpCapsule : public vpGeom
{
public:

	/*!
		default radius and height of the capsule is SCALAR_1_2 and 1.5, respectively.
	*/
							 vpCapsule();
	/*!
		construct a capsule of the given size.
	*/
							 vpCapsule(scalar radius, scalar height);

	/*!
		set a size of the capsule.
		\param r radius of the capsule
		\param h height of the capsule
	*/
	void					 SetSize(scalar r, scalar h);

	/*!
		get a radius of the capsule.
	*/
	scalar					 GetRadius(void) const;

	/*!
		get a height of the capsule.
	*/
	scalar					 GetHeight(void) const;

	/*!
		get a shape information.
		return type = 'C', data[0] = radius, data[1] = height
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const;

protected:

	scalar					 m_rRadius;
	scalar					 m_rHalfHeight;
};

/*!
	\class vpPlane
	\brief infinite plane primitive geometry
	
	vpPlane is a class to model an infinite plane.
	Plane primitive is allowed only for a ground body.
	<img src="\vp_site/plane.gif">
*/
class vpPlane : public vpGeom
{
public:

	/*!
		default plane toward (0, 0, 1)
	*/
							 vpPlane();
	/*!
		construct a plane with a plane normal
	*/
							 vpPlane(const Vec3 &normal);

	/*!
		set plane normal
	*/
	void					 SetNormal(const Vec3 &normal);

	/*!
		get plane normal
	*/
	const Vec3				&GetNormal(void) const;

	/*!
		get a shape information.
		return type = 'P', data[0] = normal[0], data[1] = normal[1], data[2] = normal[2]
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const;
	
protected:

	Vec3					 m_sNormal;
};

/*!
	\class vpCylinder
	\brief Cylinder primitive geometry
	
	vpCylinder is a class to model a cyliner.
	<img src="\vp_site/cylinder.gif">
*/
class vpCylinder : public vpGeom
{
public:

	/*!
		default radius and height of the cylinder is SCALAR_1_2 and 1, respectively.
	*/
							 vpCylinder();
	/*!
		construct a cylinder of the given size.
	*/
							 vpCylinder(scalar radius, scalar height);

	/*!
		set a size of the capsule.
		\param r radius of the cylinder
		\param h height of the cylinder
	*/
	void					 SetSize(scalar r, scalar h);

	/*!
		get a radius of the cylinder.
	*/
	scalar					 GetRadius(void) const;

	/*!
		get a height of the cylinder.
	*/
	scalar					 GetHeight(void) const;

	/*!
		get a shape information.
		return type = 'L', data[0] = radius, data[1] = height
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const;
	
protected:

	scalar					 m_rRadius;
	scalar					 m_rHalfHeight;
};

/*!
	\class vpTorus
	\brief torus primitive geometry
	
	vpTorus is a class to model a ring torus.
	<img src="\vp_site/torus.gif">
*/
class vpTorus : public vpGeom
{
public:

	/*!
		default ring radius and tube radius the torus is SCALAR_1 and SCALAR_1_2, respectively.
	*/
							 vpTorus();
	/*!
		construct a torus of the given size.
	*/
							 vpTorus(scalar ring_rad, scalar tube_rad);

	/*!
		set a size of the capsule.
		\param r1 ring radius of the torus
		\param r2 tube height of the torus
	*/
	void					 SetSize(scalar r, scalar h);

	/*!
		get a ring radius of the torus.
	*/
	scalar					 GetRingRadius(void) const;

	/*!
		get a tube radius of the torus.
	*/
	scalar					 GetTubeRadius(void) const;

	/*!
		get a shape information.
		return type = 'T', data[0] = ring radius, data[1] = tube radius
	*/
	virtual void			 GetShape(char *type, scalar *data) const;
	virtual Inertia			 GetInertia(scalar) const;
	virtual scalar			 GetBoundingSphereRadius(void) const;
	
	virtual bool			 DetectCollision(const vpGeom *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpBox *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpSphere *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCapsule *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpPlane *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpCylinder *, vpCollisionInfoArray &) const;
	virtual bool			 DetectCollision(const vpTorus *, vpCollisionInfoArray &) const;
	
protected:

	scalar					 m_rRingRadius;
	scalar					 m_rTubeRadius;
};

#ifndef VP_PROTECT_SRC
	#include "vpGeom.inl"
#endif

#endif
