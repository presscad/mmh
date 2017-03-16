/*
	VirtualPhysics v0.9

	2008.Feb.21
	Imaging Media Research Center, KIST
	jwkim@imrc.kist.re.kr
*/

#ifndef VP_INTEGRATOR
#define VP_INTEGRATOR

#include <VP/vpDataType.h>

class vpAbstractIntegrator
{
public:
	vpAbstractIntegrator() { }
	virtual void IntegrateDynamics(void) = 0;

	vpWorld *m_pWorld;
};

class vpEulerIntegrator : public vpAbstractIntegrator
{
public:
				vpEulerIntegrator(vpWorld *);
	virtual void IntegrateDynamics(void);
};

class vpImplicitEulerIntegrator : public vpAbstractIntegrator
{
public:
	virtual void IntegrateDynamics(void);
};

class vpRK4Integrator : public vpAbstractIntegrator
{
public:
	virtual void IntegrateDynamics(void);
};

#endif
