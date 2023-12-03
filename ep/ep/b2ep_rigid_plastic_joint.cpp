/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
* 
* Simo Nikula started 2018
* This constraint is designed to simulate ductile materials like
* steel when elastic part cannot be taken into account due to high natural frequencies
* Main focus is on bending.
*/

#include "b2ep_rigid_plastic_joint.h"
#include "dynamics/b2_body.h"
#include "dynamics/b2_world.h"
#include "common/b2_block_allocator.h"
#include "dynamics/b2_fixture.h"

/**
bA is currently assumed to be master body which is possibly connected to multiple
bodies
*/
void b2RigidPlasticJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

b2RigidPlasticJoint::b2RigidPlasticJoint(const b2RigidPlasticJointDef* def)
: b2ElasticPlasticJoint(def)
{
}

void b2RigidPlasticJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Log("  b2RigidPlasticJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.localAnchorA.Set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB.Set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
	b2Log("  jd.maxForce = (%.15lef, %.15lef);\n", m_maxForce.x, m_maxForce.y);
	b2Log("  jd.maxTorque = %.15lef;\n", m_maxTorque);
	b2Log("  jd.maxStrain = %.15lef;\n", m_maxStrain);
	b2Log("  jd.maxRotation = %.15lef;\n", m_maxRotation);
	b2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

void b2RigidPlasticJoint::InitVelocityConstraints(const b2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_mbi = m_indexA;
	savedOverLoads=overLoads;
	overLoads.reset();
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;
	float aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	float aB = data.positions[m_indexB].a;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;
	b2Rot qA(aA), qB(aB);
	m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;
	float iM = mA + mB;
	m_linearMass.SetZero();
	m_linearMass.ex.x = iM != 0.f ? 1.f / iM : 0.f;
	m_linearMass.ey.y = m_linearMass.ex.x;
	m_angularMass = iA + iB;
	if (m_angularMass > 0.0f)
	{
		m_angularMass = 1.0f / m_angularMass;
	}
	if (wasOverLoaded(RZ)) {
		m_dw0 = wB - wA;
		//K.GetInverse22(&m_mass);
		//float invM = iA + iB;
		//m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
	}else{
		m_dw0 = 0;
	}
	if (!initImpulseDone) {
		m_linearImpulse.SetZero();
		m_angularImpulse = 0.0f;
		m_impulse.SetZero();
	}
	// ep
	m_maxImpulse = GetMaxImpulse(data.step.dt);
	velocityIteration = 0;
	positionIteration = 0;
	if (nullptr != debugListener) {
		debugListener->EndInitVelocityConstraints(this, data);
	}
}

bool b2RigidPlasticJoint::SolvePositionConstraints(const b2SolverData& data)
{
	if (isOverLoaded()) {
		return true;
	}
	b2Vec2 cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	b2Rot qA(aA);
	b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localAnchorB);
	data.positions[m_indexB].c = cA+rA;
	data.positions[m_indexB].a = aA;
	return true;
}

void b2RigidPlasticJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	float mA = m_invMassA, mB = m_invMassB;
//	float iA = m_invIA, iB = m_invIB;
#ifdef EP_LOG
	if (epLogActive && epLogEnabled) {
		if (mA != 0) {
			epLog("RPJ:VC:%d vA1=%g %g %g\n", id,
				vA.x, vA.y, wA);
		}
		if (mB != 0) {
			epLog("RPJ:VC:%d vB1=%g %g %g\n", id,
				vB.x, vB.y, wB);
		}
		epLog("RPJ:VC:%d m_impulse=%g %g %g\n", id,
			m_impulse.x, m_impulse.y, m_impulse.z);
	}
#endif
	if (nullptr != debugListener) {
		debugListener->BeginVelocityIteration(this, data);
	}
	b2Vec3 impulse;
	Cdot.SetZero();
	impulse.SetZero();
	float h = data.step.dt;
	/** Adapted from b2Friction.cpp
	*
	*/
	// Solve linear part
	b2Vec2 lCdot = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
	if (lCdot.LengthSquared() > 0.f) {
		b2Vec2 lImpulse = -b2Mul(m_linearMass, lCdot);
		b2Vec2 oldLImpulse = m_linearImpulse;
		m_linearImpulse += lImpulse;
		if (m_linearImpulse.LengthSquared() > m_maxForce.LengthSquared())
		{
			m_linearImpulse.Normalize();
			m_linearImpulse *= m_maxForce.Length();
		}
		impulse.x = m_linearImpulse.x - oldLImpulse.x;
		impulse.y = m_linearImpulse.y - oldLImpulse.y;
		Cdot.x = lCdot.x;
		Cdot.y = lCdot.y;
		wA -= m_invIA * b2Cross(m_rA, lImpulse);
		wB += m_invIB * b2Cross(m_rB, lImpulse);
	}
	// Solve angular part
	float aCdot = wB - wA - m_dw0;
	if (b2Abs(aCdot) > 0.f) {
		float aImpulse = -m_angularMass * aCdot;
		float oldAImpulse = m_angularImpulse;
		float maxImpulse = h * m_maxTorque;
		m_angularImpulse = b2Clamp
		(m_angularImpulse + aImpulse, -maxImpulse, maxImpulse);
		aImpulse = m_angularImpulse - oldAImpulse;
		impulse.z = aImpulse;
	}
	Cdot.z = aCdot;
	m_impulse += impulse;
#ifdef EP_LOG
	if (epLogActive && epLogEnabled) {
		if (b2Dot(Cdot, Cdot) > 0.f) {
			epLog("RPJ:VC:%d Cdot=%g %g %g\n", id,
				Cdot.x, Cdot.y, Cdot.z);
			epLog("RPJ:VC:%d impulse=%g %g %g\n", id,
				impulse.x, impulse.y, impulse.z);
		}
	}
#endif
	if (nullptr != debugListener) {
		debugListener->EndVelocityIteration(this, data);
	}
	velocityIteration++;
}

b2Vec2 b2RigidPlasticJoint::GetReactionForce(float inv_dt) const
{
	b2Vec2 P(m_jim.x, m_jim.y);
	return inv_dt * P;
}

float b2RigidPlasticJoint::GetReactionTorque(float inv_dt) const
{
	return inv_dt * m_jim.z;
}

/**
* After velocity iterations
*/
void b2RigidPlasticJoint::UpdatePlasticity(const b2SolverData & data)
{
	if (!isOverLoaded()) {
		return;
	}
	b2Vec2 cA = data.positions[m_indexA].c;
	b2Vec2 cB = data.positions[m_indexB].c;
	float newDistance = (cA - cB).Length();
	float origDistance = m_localAnchorA.Length() + m_localAnchorB.Length();
	// this needs more analysis
	// current implementation is based on idea
	// that tearing joint up joint should be more meaningful
	// than pushing
	if ((isOverLoaded(X) || isOverLoaded(Y)) && origDistance < newDistance) {
		float sf = newDistance / origDistance;
		m_localAnchorA *= sf;
		m_localAnchorB *= sf;
		m_currentStrain += b2Abs(newDistance - origDistance);
	}
	if (!isOverLoaded(RZ)) {
		return;
	}
	updateRotationalPlasticity(data, 0.f);
}