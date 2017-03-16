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

//////////////////////////////////////////////////////////////////////////////////
//
//		title		:	LieGroup.cpp
//						
//		version		:	v0.991
//		author		:	Jinwook Kim (jwkim@imrc.kist.re.kr)
//		last update	:	2010.2.10
//
//////////////////////////////////////////////////////////////////////////////////

#include <VP/LieGroup.h>
#include <iomanip>

ostream &operator << (ostream &os, const Vec3 &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= SCALAR_0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const Axis &v)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 3; i++ )
	{ 
		if ( v[i] >= SCALAR_0 ) os << " " << setw(6) << v[i] << " ";
		else os << setw(7) << v[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const se3 &s)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( s[i] >= SCALAR_0 ) os << " " << setw(6) << s[i] << " ";
		else os << setw(7) << s[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const dse3 &t)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[ ";
	for ( int i = 0; i < 6; i++ )
	{ 
		if ( t[i] >= SCALAR_0 ) os << " " << setw(6) << t[i] << " ";
		else os << setw(7) << t[i] << " ";
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

ostream &operator << (ostream &os, const SE3 &T)
{
	ios_base::fmtflags flags = os.setf(ios::left | ios::fixed);
	streamsize sz = os.precision(3);
	os << "[" << endl;
	for ( int i = 0; i < 4; i++ )
	{
		for ( int j = 0; j < 4; j++ )
		{
			if ( T(i,j) >= SCALAR_0 ) os << " " << setw(6) << T(i,j) << " ";
			else os << setw(7) << T(i,j) << " ";
		}
		os << ";" << endl;
	}
	os << "];" << endl;
	os.setf(flags);
	os.precision(sz);
	return os;
}

Inertia BoxInertia(scalar density, const Vec3 &size)
{
	scalar mass = (scalar)8.0 * density * size[0] * size[1] * size[2];
	scalar ix = mass * (size[1] * size[1] + size[2] * size[2]) / SCALAR_3;
	scalar iy = mass * (size[0] * size[0] + size[2] * size[2]) / SCALAR_3;
	scalar iz = mass * (size[0] * size[0] + size[1] * size[1]) / SCALAR_3;
	return Inertia(mass, ix, iy, iz);
}

Inertia SphereInertia(scalar density, scalar rad)  
{
	rad *= rad;
	scalar mass = density * M_PI * rad;
	scalar i = (scalar)0.4 * mass * rad;
	return Inertia(mass, i, i, i);
}

Inertia CylinderInertia(scalar density, scalar rad, scalar height)
{
	rad *= rad;
	scalar mass = density * M_PI * rad * height;
	scalar ix = mass * height * height  / (scalar)12.0 + SCALAR_1_4 * mass * rad;
	scalar iy = ix;
	scalar iz = SCALAR_1_2 * mass * rad;
	return Inertia(mass, ix, iy, iz);
}

Inertia TorusInertia(scalar density, scalar ring_rad, scalar tube_rad)
{
	scalar mass = density * SCALAR_2 * M_PI_SQR * ring_rad * tube_rad * tube_rad;
	scalar ix = mass * ((scalar)0.625 * tube_rad * tube_rad + SCALAR_1_2 * ring_rad + ring_rad);
	scalar iy = ix;
	scalar iz = mass * ((scalar)0.75 * tube_rad * tube_rad + ring_rad + ring_rad);
	return Inertia(mass, ix, iy, iz);
}
