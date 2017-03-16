#include "stdafx.h"

#include "utility/stdtemplate.h"
#include "utility/configtable.h"
#include "image/imageclass.h"
#include "utility/typestring.h"
#include "utility/garray.h"
#include "motion/motion.h"
#include "motion/motionloader.h"
#include "utility/tword.h"

#include "math/optimize.h"

#include "MATHCLASS/mathclass.h"
#include "PmQm/pm.h"

#include "ConvertMotionToPmQm.h"

#include "Retarget_JH.h"



ConvertMotionToPmQm& get_converter(MotionUtil::FullbodyIK_JH* ik)
{
	return *((ConvertMotionToPmQm*)(ik->m_private_data));
}

MotionUtil::FullbodyIK_JH::FullbodyIK_JH(MotionLoader const& skeleton)
{
	m_private_data=(void*)new ConvertMotionToPmQm(skeleton);
}

MotionUtil::FullbodyIK_JH::~FullbodyIK_JH()
{
	delete &get_converter(this);
}

void MotionUtil::FullbodyIK_JH::IKsolve(Posture& poseInOut, std::vector<Constraint>& constraints)
{
	PmPosture p;

	ConvertMotionToPmQm& conv=get_converter(this);
	p.setBody(conv.getBody());
	conv.setPmPosture(poseInOut, p);

	//PenvIKsolverType = PENV_IK_HYBRID;
	//PenvIKsolverType = PENV_IK_ANALYTICAL;				
	PenvIKsolverType = PENV_IK_NUMERICAL;				

	PenvIKsolverMethod = PENV_IK_CG_METHOD;

	PmConstraint c;

	for(int i=0; i<constraints.size(); i++)
	{
		int pmEnum=conv.treeIndex2enum(constraints[i].bone->treeIndex());

		switch(constraints[i].type)
		{
		case Constraint::C_POSITION:
			c.push(pmEnum, ToJHM(constraints[i].translation));
			break;

		case Constraint::C_ORIENTATION:
			c.push(pmEnum, ToJHM(constraints[i].rotation));
			break;

		case Constraint::C_TRANFORM:
			c.push(pmEnum, jhm::transf(ToJHM(constraints[i].rotation), ToJHM(constraints[i].translation)));
			break;
		}
	}

	if(PenvIKsolverType==PENV_IK_NUMERICAL)
		p.ik_body(c, PM_MASK_ALL_BODY);
	else
		p.ik_torso(c, 0);

	conv.getPmPosture(p, poseInOut);
}
