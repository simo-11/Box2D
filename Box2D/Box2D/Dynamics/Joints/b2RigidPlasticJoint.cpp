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
* steel when elastic part cannot be taken into account.
* Main focus is on bending.
*/

#include "Box2D/Dynamics/Joints/b2RigidPlasticJoint.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2World.h"
#include "Box2D/Common/b2BlockAllocator.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/b2TimeStep.h"


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
	float32 invM = iA + iB;
	b2Mat33 K;
	K.ez.z = iA + iB;
	m_k = 0.f;
	m_gamma = 0.f;
	m_bias = 0.f;
	float32 iM = mA + mB;
	m_mass.ex.SetZero();
	m_mass.ey.SetZero();
	m_mass.ez.SetZero();
	m_mass.ex.x = iM != 0.f ? 1.f / iM : 0.f;
	m_mass.ey.y = m_mass.ex.x;
	m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
	if (data.step.warmStarting)
	{
		if (!initImpulseDone) {
			// Scale impulses to support a variable time step.
			m_impulse *= data.step.dtRatio;
		}
		b2Vec2 P(m_impulse.x, m_impulse.y);
		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + m_impulse.z);
		vB += mB * P;
		wB += iB * (b2Cross(m_rB, P) + m_impulse.z);
	}
	else
	{
		m_impulse.SetZero();
	}
	solveOrder = b2ElasticPlasticJoint::getForceFirst();
	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
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
	b2Vec2 cA = data.positions[m_indexA].c;
	float32 aA = data.positions[m_indexA].a;
	b2Vec2 cB = data.positions[m_indexB].c;
	float32 aB = data.positions[m_indexB].a;
	b2Rot qA(aA), qB(aB);
	float32 mA = m_invMassA, mB = m_invMassB;
	float32 iA = m_invIA, iB = m_invIB;
	b2Vec2 rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	b2Vec2 rB = b2Mul(qB, m_localAnchorB - m_localCenterB);
	b2Mat33 K;
	K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.ez.x = -rA.y * iA - rB.y * iB;
	K.ex.y = K.ey.x;
	K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.ez.y = rA.x * iA + rB.x * iB;
	K.ex.z = K.ez.x;
	K.ey.z = K.ez.y;
	K.ez.z = iA + iB;
	if (nullptr != debugListener) {
		debugListener->BeginPositionIteration(this, data);
	}
	b2Vec2 C1 = cB + rB - cA - rA;
	float32 C2 = aB - aA - m_referenceAngle;
	positionError = C1.Length();
	angularError = b2Abs(C2);
#ifdef EP_LOG
	epLog("J:PC:%d positionError=%g, angularError=%g\n", id,
		positionError, angularError);
#endif
	b2Vec3 C(C1.x, C1.y, C2);
	b2Vec3 impulse;
	if (K.ez.z > 0.0f)
	{
		impulse = -K.Solve33(C);
	}
	else
	{
		b2Vec2 impulse2 = -K.Solve22(C1);
		impulse.Set(impulse2.x, impulse2.y, 0.0f);
	}
	b2Vec2 P = Clamp(b2Vec2(impulse.x, impulse.y), data);
	cA -= mA * P;
	float32 M = Clamp((b2Cross(rA, P) + impulse.z), data);
#ifdef EP_LOG
	epLog("J:PC:%d P=%g %g, M1=%g", id,
			P.x, P.y, M);
#endif
	aA -= iA * M;
	cB += mB * P;
	M = Clamp((b2Cross(rB, P) + impulse.z), data);
#ifdef EP_LOG
	epLog(", M2=%g\n", M);
#endif
	aB += iB * M;
	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;
	jointOk = positionError <= b2_linearSlop && angularError <= b2_angularSlop;
	if (nullptr != debugListener) {
		debugListener->EndPositionIteration(this, data);
	}
	positionIteration++;
	return jointOk;
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
		epLog("J:VC:%d m_impulse1=%g %g %g\n", id,
			m_impulse.x, m_impulse.y, m_impulse.z);
	}
#endif
	if (nullptr != debugListener) {
		debugListener->BeginVelocityIteration(this, data);
	}
	b2Vec2 Cdot1;
	float32 Cdot2, impulse2;
	for (int i = 0; i < 2; i++) {
		switch (solveOrder[i]) {
		case MOMENT:
			Cdot2 = wB - wA;
			impulse2 = GetClampedDeltaImpulse(Cdot2, data);
			m_impulse.z += impulse2;

			wA -= iA * impulse2;
			wB += iB * impulse2;
#ifdef EP_LOG
			if (epLogActive && epLogEnabled) {
				epLog("J:VC:%d Cdot2=%g, impulse2=%g, wA=%g, wB=%g\n", id,
					Cdot2, impulse2, wA, wB);
			}
#endif
			break;
		case FORCE:
			Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);
			b2Vec2 impulse1 = GetClampedDeltaImpulse(Cdot1, data);
			m_impulse.x += impulse1.x;
			m_impulse.y += impulse1.y;
			b2Vec2 P = impulse1;
			vA -= mA * P;
			wA -= iA * b2Cross(m_rA, P);
			vB += mB * P;
			wB += iB * b2Cross(m_rB, P);
#ifdef EP_LOG
			if (epLogActive && epLogEnabled) {
				epLog("J:VC:%d Cdot1=%g %g\n", id,
					Cdot1.x, Cdot1.y);
				epLog("J:VC:%d impulse=%g %g\n", id,
					impulse1.x, impulse1.y);
				if (mA != 0) {
					epLog("J:VC:%d vA2=%g %g %g\n", id,
						vA.x, vA.y, wA);
				}
				if (mB != 0) {
					epLog("J:VC:%d vB2=%g %g %g\n", id,
						vB.x, vB.y, wB);
				}
			}
#endif
			break;
		}
	}
	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	if (nullptr != debugListener) {
		Cdot = b2Vec3(Cdot1.x, Cdot1.y, Cdot2);
		debugListener->EndVelocityIteration(this, data);
	}
	velocityIteration++;
}

