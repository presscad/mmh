#pragma once

#include "stdafx_implicit.h"
#include "../../common_sources/bputil.h"

#include "cImplicitUtil.h"
#include "../../external_libraries/implicitMassSpringSolver2/Physics.h"

#include "csIMSModel.h"

#include "cImplicit.h"


string ParticleConfig::__str__()
{
	stringstream ss;
	ss << "position: " << XC(position.attr("__str__")());
	ss << ", mass: " << mass;
	ss << ", initialVelocity: " << XC(initialVelocity.attr("__str__")());
	ss << ", dynamicMu: " << dynamicMu;
	ss << ", staticMu: " << staticMu;
	return ss.str();
}
std::string SpringConfig::__str__()
{
	stringstream ss;
	ss << "particleIndex0: " << particleIndex0;
	ss << ", particleIndex1: " << particleIndex1;
	ss << ", Ks: " << Ks;
	ss << ", Kd: " << Kd;
	return ss.str();
}
std::string SystemConfig::__str__()
{
	stringstream ss;
	ss << "g: " << XC(g.attr("__str__")());
	return ss.str();
}

IMSModel::IMSModel( const list& particleConfigs,  const list& springConfigs, const SystemConfig& systemConfig )
{
	_pSystem = new Physics_ParticleSystem(len(particleConfigs));
	_pSystem->m_iIntegrationMethod = INTEGRATE_IMPLICIT;
	_pSystem->Reset();

	buildModel(particleConfigs, springConfigs);

	_pSystem->SetupMatrices();

	_pSystem->m_cfg.tangentialLockingThresholdVel = systemConfig.tangentLockingVel;

	Physics_Vector3 _tmp_gravity = PYSEQ_2_VECTOR3(systemConfig.g);
	Physics_GravityForce *pGravity = new Physics_GravityForce(_tmp_gravity);
	_pSystem->AddForce( *pGravity );
}
void IMSModel::buildModel(const list& particleConfigs, const list& springConfigs)
{
	Physics_Vector3 v;
	for(int i=0; i<_pSystem->m_iParticles; ++i)
	{
		ParticleConfig& pc = extract<ParticleConfig&>(particleConfigs[i]);

		_pSystem->Position(i) = PYSEQ_2_VECTOR3(pc.position);
		_pSystem->Velocity(i) = PYSEQ_2_VECTOR3(pc.initialVelocity);
		_pSystem->SetMass(i, pc.mass);
		_pSystem->m_cfg.m_dynamicFrictionCoef[i].x = pc.dynamicMu;
		_pSystem->m_cfg.m_staticFrictionCoef[i].x = pc.staticMu;
	}

	Physics_SpringForce* pSpring;
	for(int i=0; i<len(springConfigs); ++i)
	{
		SpringConfig& sc = extract<SpringConfig&>(springConfigs[i]);

		pSpring = new Physics_SpringForce();
		pSpring->m_iParticle[0] = sc.particleIndex0;
		pSpring->m_iParticle[1] = sc.particleIndex1;

		VECTOR3_SUB(_pSystem->Position(pSpring->m_iParticle[0]), _pSystem->Position(pSpring->m_iParticle[1]), v);
		pSpring->m_RestDistance = v.Length();

		pSpring->m_kSpring = sc.Ks;
		pSpring->m_kSpringDamp = sc.Kd;

		_pSystem->AddForce(*pSpring);
		_springs.push_back(pSpring);

		if(!_subspringsMap.has_key(sc.subspringsName))
			_subspringsMap[sc.subspringsName] = list();
		list subsprings = extract<list>(_subspringsMap[sc.subspringsName]);
		subsprings.append(i);
	}
}
//void IMSModel::setMu(const list& dynamicMuList, const list& staticMuList)
//{
//	for(int i=0; i<len(dynamicMuList); ++i)
//	{
//		_pSystem->m_cfg.m_dynamicFrictionCoef[i].x = XD(dynamicMuList[i]);
//		_pSystem->m_cfg.m_staticFrictionCoef[i].x = XD(staticMuList[i]);
//	}
//}
void IMSModel::setMu(double dynamicMu, double staticMu, const list& vertexIndices)
{
	for(int i=0; i<len(vertexIndices); ++i)
	{
		int index = XI(vertexIndices[i]);
		_pSystem->m_cfg.m_dynamicFrictionCoef[index].x = dynamicMu;
		_pSystem->m_cfg.m_staticFrictionCoef[index].x = staticMu;
		if(dynamicMu==0.0 || staticMu==0.0)
		{
			_pSystem->mContacts.contactStatus[index] = Physics_Contacts::no_contact;
			_pSystem->ReleaseConstraint(index);
		}
	}
}
void IMSModel::step(double timeStep)
{
	for(int i=0, ni=_pSystem->numParticle(); i<ni; i++)
	{
		if(_pSystem->Position(i).y < 0.0+DBL_EPSILON)
		{
			Physics_t dp = _pSystem->Position(i).y;

			_pSystem->mContacts.setContacts(i,
				_pSystem->Velocity(i).x, _pSystem->Velocity(i).y, _pSystem->Velocity(i).z,
				0, -1.0*dp, 0, 
				0, 1, 0, NULL);
		}
		else
			_pSystem->mContacts.setNoContact(i);
	}

	_pSystem->Update(timeStep);
}
void IMSModel::updateSprings(const list& springLengths)
{
	for(int i=0; i<_springs.size(); ++i)
		_springs[i]->m_RestDistance = XD(springLengths[i]);
}
boost::python::tuple IMSModel::getPosition( int index )
{
	Physics_Vector3 pos = _pSystem->Position(index);
	return make_tuple(pos[0], pos[1], pos[2]);
}
tuple IMSModel::getVelocity(int index)
{
	Physics_Vector3 vel = _pSystem->Velocity(index);
	return make_tuple(vel[0], vel[1], vel[2]);
}

list IMSModel::getPositions()
{
	list ls = list();
	for(int i=0; i<_pSystem->m_iParticles; ++i)
		ls.append(VECTOR3_2_PYTUPLE(_pSystem->Position(i)));
	return ls;
}

list IMSModel::getVelocities()
{
	list ls = list();
	for(int i=0; i<_pSystem->m_iParticles; ++i)
		ls.append(VECTOR3_2_PYTUPLE(_pSystem->Velocity(i)));
	return ls;
}

list IMSModel::getContactParticleIndices()
{
	list ls;
	for(int i=0; i<_pSystem->m_iParticles; ++i)
		//if(_pSystem->Position(i)[1] < 0.0 && _pSystem->m_cfg.m_dynamicFrictionCoef[i].x > 0.0)
		if(_pSystem->mContacts.contactStatus[i]!=Physics_Contacts::no_contact && _pSystem->m_cfg.m_dynamicFrictionCoef[i].x > 0.0)
			ls.append(i);
	return ls;
}

// void IMSModel::setVelocity( int index, const object& velocity)
// {
// 	_pSystem->Velocity(index) = PYSEQ_2_VECTOR3(velocity);
// }

int IMSModel::getParticleNum()
{
	return _pSystem->m_iParticles;
}

list IMSModel::getState()
{
	list ls = list();
	for(int i=0; i<_pSystem->m_iParticles; ++i)
	{
		ls.append(make_tuple(_pSystem->Position(i)[0], _pSystem->Position(i)[1], _pSystem->Position(i)[2],
							_pSystem->Velocity(i)[0], _pSystem->Velocity(i)[1], _pSystem->Velocity(i)[2]));
	}
	return ls;
}
void IMSModel::setState(const object& state)
{
	for(int i=0; i<_pSystem->m_iParticles; ++i)
	{
		_pSystem->Position(i) = Physics_Vector3(XD(state[i][0]), XD(state[i][1]), XD(state[i][2]));
		_pSystem->Velocity(i) = Physics_Vector3(XD(state[i][3]), XD(state[i][4]), XD(state[i][5]));
	}
}
