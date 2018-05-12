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

#include "Box2D/Dynamics/Joints/b2RigidPlasticJoint.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2World.h"
#include "Box2D/Common/b2BlockAllocator.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/b2TimeStep.h"

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
	overLoads.reset();
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;
	float32 aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	float32 wA = data.velocities[m_indexA].w;
	float32 aB = data.positions[m_indexB].a;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float32 wB = data.velocities[m_indexB].w;
	b2Rot qA(aA), qB(aB);
	m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	float32 mA = m_invMassA, mB = m_invMassB;
	float32 iA = m_invIA, iB = m_invIB;
	b2Mat33 K;
	K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	K.ez.x = -m_rA.y * iA - m_rB.y * iB;
	K.ex.y = K.ey.x;
	K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	K.ez.y = m_rA.x * iA + m_rB.x * iB;
	K.ex.z = K.ez.x;
	K.ey.z = K.ez.y;
	K.ez.z = iA + iB;
	if (K.ez.z == 0.0f)
	{
		K.GetInverse22(&m_mass);
		m_gamma = 0.0f;
		m_bias = 0.0f;
	}
	else
	{
		K.GetSymInverse33(&m_mass);
		m_gamma = 0.0f;
		m_bias = 0.0f;
	}
	if (!initImpulseDone) {
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
	return true;
}

void b2RigidPlasticJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[m_indexA].v;
	float32 wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float32 wB = data.velocities[m_indexB].w;

	float32 mA = m_invMassA, mB = m_invMassB;
	float32 iA = m_invIA, iB = m_invIB;
#ifdef EP_LOG
	if (epLogActive && epLogEnabled) {
		if (mA != 0) {
			epLog("J:VC:%d vA1=%g %g %g\n", id,
				vA.x, vA.y, wA);
		}
		if (mB != 0) {
			epLog("J:VC:%d vB1=%g %g %g\n", id,
				vB.x, vB.y, wB);
		}
		epLog("J:VC:%d m_impulse=%g %g %g\n", id,
			m_impulse.x, m_impulse.y, m_impulse.z);
	}
#endif
	if (nullptr != debugListener) {
		debugListener->BeginVelocityIteration(this, data);
	}
	b2Vec2 Cdot1;
	float32 Cdot2;
	if (isOverLoaded()) {
		Cdot1.x = 0;
		Cdot1.y = 0;
		Cdot2 = 0;
	}
	else {
		Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
		Cdot2 = wB - wA;
	}
	b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

	b2Vec3 impulse = -b2Mul(m_mass, Cdot);
	m_impulse += impulse;
#ifdef EP_LOG
			if (epLogActive && epLogEnabled) {
				epLog("J:VC:%d Cdot=%g %g %g\n", id,
					Cdot.x, Cdot.y, Cdot.z);
				epLog("J:VC:%d impulse=%g %g %g\n", id,
					impulse.x, impulse.y, impulse.z);
			}
#endif
	if (nullptr != debugListener) {
		Cdot = b2Vec3(Cdot1.x, Cdot1.y, Cdot2);
		debugListener->EndVelocityIteration(this, data);
	}
	velocityIteration++;
}

b2Vec2 b2RigidPlasticJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 P(m_jim.x, m_jim.y);
	return inv_dt * P;
}

float32 b2RigidPlasticJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * m_jim.z;
}
