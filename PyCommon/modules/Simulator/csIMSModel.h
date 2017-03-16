#pragma once

#include "../../common_sources/bputil.h"

class Physics_ParticleSystem;
class Physics_SpringForce;

class ParticleConfig
{
public:
	ParticleConfig()
	{
		position = make_tuple(0.,0.,0.);
		mass = 1.;
		initialVelocity = make_tuple(0.,0.,0.);
		dynamicMu = 1.;
		staticMu = 1.;
	}
	ParticleConfig(const object& position_, const double mass_, const object& initialVelocity_, const double dynamicMu_, const double staticMu_)
		:position(position_), mass(mass_), initialVelocity(initialVelocity_), dynamicMu(dynamicMu_), staticMu(staticMu_)
	{}
	object position;
	double mass;
	object initialVelocity;
	double dynamicMu;
	double staticMu;
	string __str__();
};
struct ParticleConfig_pickle_suite : pickle_suite
{
	static tuple getstate(const ParticleConfig& o)
	{
		return boost::python::make_tuple(o.position, o.mass, o.initialVelocity, o.dynamicMu, o.staticMu);
	}
	static void setstate(ParticleConfig& o, tuple state)
	{
		o.position = state[0];
		o.mass = XD(state[1]);
		o.initialVelocity = state[2];
		o.dynamicMu = XD(state[3]);
		o.staticMu = XD(state[4]);
	}
};

class SpringConfig
{
public:
	SpringConfig(int particleIndex0_, int particleIndex1_, double Ks_, double Kd_)
		:particleIndex0(particleIndex0_), particleIndex1(particleIndex1_), Ks(Ks_), Kd(Kd_)
	{}
	int particleIndex0;
	int particleIndex1;
	double Ks;
	double Kd;
	string subspringsName;
	string __str__();
};
struct SpringConfig_pickle_suite : pickle_suite
{
	static tuple getinitargs(SpringConfig& o)
	{
		return boost::python::make_tuple(o.particleIndex0, o.particleIndex1, o.Ks, o.Kd, o.subspringsName);
	}
	static tuple getstate(const SpringConfig& o)
	{
		return boost::python::make_tuple(o.particleIndex0, o.particleIndex1, o.Ks, o.Kd, o.subspringsName);
	}
	static void setstate(SpringConfig& o, tuple state)
	{
		o.particleIndex0 = XI(state[0]);
		o.particleIndex1 = XI(state[1]);
		o.Ks = XD(state[2]);
		o.Kd = XD(state[3]);
		o.subspringsName = XS(state [4]);
	}
};

class SystemConfig
{
public:
	SystemConfig()
	{
		g = make_tuple(0.,-9.8,0.);
		tangentLockingVel = .01;
	}
	object g;
	double tangentLockingVel;
	string __str__();
};
struct SystemConfig_pickle_suite : pickle_suite
{
	static tuple getstate(const SystemConfig& o)
	{
		return boost::python::make_tuple(o.g, o.tangentLockingVel);
	}
	static void setstate(SystemConfig& o, tuple state)
	{
		o.g = state[0] ;
		o.tangentLockingVel = XD(state[1]);
	}
};

// Implicit Mass Spring Model
class IMSModel
{
public:
	Physics_ParticleSystem* _pSystem;
	vector<Physics_SpringForce*> _springs;
	dict _subspringsMap;
public:
	void buildModel(const list& particleConfigs, const list& springConfigs);
public:	// expose to python
	IMSModel(const list& particleConfigs,  const list& springConfigs, const SystemConfig& systemConfig);
	void updateSprings(const list& springLengths);
	void step(double timeStep);
	tuple getPosition(int index);
	tuple getVelocity(int index);
//	void setMu(const list& dynamicMuList, const list& staticMuList);
	void setMu(double dynamicMu, double staticMu, const list& vertexIndices);
	list getPositions();
	list getVelocities();
	list getContactParticleIndices();
// 	void setVelocity(int index, const object& velocity); 
	int getParticleNum();
	list getState();
	void setState(const object& state);
};
