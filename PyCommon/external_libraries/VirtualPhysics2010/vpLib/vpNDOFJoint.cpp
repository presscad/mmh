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

#include <VP/vpNDOFJoint.h>
#include <VP/vpSystem.h>

vpNDOFJoint::vpNDOFJoint(int dof)
{
	m_iDOF = dof;

	m_rQ.resize(m_iDOF);
	m_rDq.resize(m_iDOF);
	m_rDdq.resize(m_iDOF);
	m_rActuationTau.resize(m_iDOF);
	m_rSpringDamperTau.resize(m_iDOF);
	m_rImpulsiveTau.resize(m_iDOF);
	m_rQi.resize(m_iDOF);
	m_rK.resize(m_iDOF);
	m_rC.resize(m_iDOF);
	m_rRestitution.resize(m_iDOF);
	m_sS.resize(m_iDOF);
	m_bHasUpperLimit.resize(m_iDOF);
	m_bHasLowerLimit.resize(m_iDOF);
	m_sH.resize(m_iDOF);

	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_rQ[i] = m_rDq[i] = m_rDdq[i] = m_rActuationTau[i] = m_rSpringDamperTau[i] = m_rSpringDamperTau[i] = m_rImpulsiveTau[i] = m_rQi[i] = m_rK[i] = m_rC[i] = SCALAR_0;
		m_rRestitution[i] = SCALAR_1;
		m_sS[i] = SCALAR_0;
		m_bHasUpperLimit[i] = m_bHasLowerLimit[i] = false;
		m_sH[i].resize(m_iDOF);
	}
	
	m_sO = Zeros<scalar>(m_iDOF, m_iDOF);
	m_sT = Zeros<scalar>(m_iDOF, 1);
	m_sVl = SCALAR_0;

	m_pTransform = NULL;
}

void vpNDOFJoint::UpdateTorqueID(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_rActuationTau[i] = m_sF * m_sS[i];
}

void vpNDOFJoint::UpdateTorqueHD(void)
{
	for ( int i = 0; i < m_iDOF; i++ )	m_rActuationTau[i] = m_sDV * m_sL[i] + m_sB * m_sS[i];
}

void vpNDOFJoint::UpdateVelocity(const se3 &V_parent)
{
	m_pTransform->GetJacobian(m_rQ, m_sS);

	m_pTransform->GetHessian(m_rQ, m_sH);

	m_sDSdq = SCALAR_0;
	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_sDSdq += (m_rDq[i] * m_rDq[i]) * m_sH[i][i];
		for ( int j = i + 1; j < m_iDOF; j++ ) m_sDSdq += (SCALAR_2 * m_rDq[i] * m_rDq[j]) * m_sH[i][j];
	}

	m_sVl = SCALAR_0;
	for ( int i = 0; i < m_iDOF; i++ ) m_sVl += m_sS[i] * m_rDq[i];

	m_sV  = V_parent;
	m_sV += m_sVl;

	m_sW.ad(m_sV, m_sVl);
	m_sW += m_sDSdq;
}

void vpNDOFJoint::UpdateAccelerationID(const se3 &DV)
{
	m_sDV  = DV;
	m_sDV += m_sW;
	for ( int i = 0; i < m_iDOF; i++ ) m_sDV += m_sS[i] * m_rDdq[i];
}

void vpNDOFJoint::UpdateAccelerationFD(const se3 &DV)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] -= DV * m_sL[i];
	m_sP = m_sO * m_sT;
	for ( int i = 0; i < m_iDOF; i++ ) m_rDdq[i] = m_sP[i];
	m_sDV = DV;
	for ( int i = 0; i < m_iDOF; i++ ) m_sDV += m_sS[i] * m_rDdq[i];
}

void vpNDOFJoint::UpdateAInertia(AInertia &tmpI)
{
	tmpI = m_sJ;

	for ( int i = 0; i < m_iDOF; i++ )
	{
		tmpI.SubtractKroneckerProduct(m_sO(i,i) * m_sL[i], m_sL[i]);
		for ( int j = i + 1; j < m_iDOF; j++ ) tmpI.SubtractKroneckerProduct(m_sO(i,j) * m_sL[i], m_sL[j]);
	}
}

void vpNDOFJoint::UpdateLOTP(void)
{	
	for ( int i = 0; i < m_iDOF; i++ ) m_sL[i] = m_sJ * m_sS[i];

	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_sO(i,i) = m_sL[i] * m_sS[i];
		for ( int j = i + 1; j < m_iDOF; j++ ) m_sO(i,j) = m_sO(j,i) = m_sL[i] * m_sS[j];
	}
	
	m_sO = Inv(m_sO);

	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] = GetTorque_(i) - m_sC * m_sS[i];
	
	m_sP = m_sO * m_sT;
}

void vpNDOFJoint::UpdateTP(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] = m_rImpulsiveTau[i] - m_sB * m_sS[i];
	m_sP = m_sO * m_sT;
}

void vpNDOFJoint::UpdateLP(void)
{
	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_sL[i] = m_sJ * m_sS[i];
		m_sP[i] = m_rDdq[i];
	}
}

dse3 vpNDOFJoint::GetLP(void)
{
	dse3 re = m_sL[0] * m_sP[0];
	for ( int i = 1; i < m_iDOF; i++ ) re += m_sL[i] * m_sP[i];
	return re;
}

void vpNDOFJoint::ClearTP(void)
{
	for ( int i = 0; i < m_iDOF; i++ ) m_sT[i] = m_sP[i] = SCALAR_0;
}

void vpNDOFJoint::SwapBody(void)
{
	vpJoint::SwapBody();
	
	for ( int i = 0; i < m_iDOF; i++ )
	{
		m_rQ[i] = -m_rQ[i];
		m_rDq[i] = -m_rDq[i];
		m_rActuationTau[i] = -m_rActuationTau[i];
		m_rQi[i] = -m_rQi[i];
		
		scalar tmp = m_rQul[i];
		m_rQul[i] = -m_rQll[i];
		m_rQll[i] = -tmp;

		bool tmp2 = m_bHasUpperLimit[i];
		m_bHasUpperLimit[i] = m_bHasLowerLimit[i];
		m_bHasLowerLimit[i] = tmp2;
	}
}

void vpNDOFJoint::BuildKinematics(void)
{
	for ( int i = 0; i < m_iDOF; i++ )
		GetState().push_back(vpState(this, i));
}

void TransformNDOF::GetJacobian(const scalarArray &q, se3Array &J)
{
	SE3 T = GetTransform(q);
	scalarArray dq(m_iDOF);
	for ( int i = 0; i < m_iDOF; i++ ) dq[i] = q[i];

	for ( int i = 0; i < m_iDOF; i++ )
	{
		dq[i] += m_rEPS;
		J[i] = (SCALAR_1 / m_rEPS) * Linearize(T % GetTransform(dq));
		dq[i] -= m_rEPS;
	}
}

void TransformNDOF::GetHessian(const scalarArray &q, se3DbAry &H)
{
	se3Array J(m_iDOF), dJ(m_iDOF);
	
	GetJacobian(q, J);

	scalarArray dq(m_iDOF);
	for ( int i = 0; i < m_iDOF; i++ ) dq[i] = q[i];

	for ( int i = 0; i < m_iDOF; i++ )
	{
		dq[i] += m_rEPS;
		GetJacobian(dq, dJ);
		for ( int j = i + 1; j < m_iDOF; j++ ) H[i][j] = (SCALAR_1 / m_rEPS) * (dJ[j] - J[j]);
		dq[i] -= m_rEPS;
	}
}

#ifdef VP_PROTECT_SRC
	#include <VP/vpNDOFJoint.inl>
#endif
