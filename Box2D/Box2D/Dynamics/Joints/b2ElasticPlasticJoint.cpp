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
* Simo Nikula modifications started 2017
* This constraint is designed to simulate ductile materials like
* steel.
* Main focus is on bending.
* Actual code is based on WeldJoint.
*/

#include "Box2D/Dynamics/Joints/b2ElasticPlasticJoint.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/b2TimeStep.h"

/*
These are smaller than normal b2 limits as
there are often many connected joints and
error accumulates.
These can be tuned.
*/
namespace
{
	float32 b2ep_linearSlop=0.0005f;
	float32 b2ep_angularSlop=0.0005f;
	int32 epId = 0;
}

void b2ElasticPlasticJointDef::Initialize(b2Body* bA, b2Body* bB, const b2Vec2& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

void b2ElasticPlasticJoint::resetEpId(){
	epId = 0;
}
b2ElasticPlasticJoint::b2ElasticPlasticJoint(const b2ElasticPlasticJointDef* def)
: b2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_referenceAngle = def->referenceAngle;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_impulse.SetZero();

	m_maxForce = def->maxForce;
	m_maxTorque = def->maxTorque;
	m_maxElasticRotation = def->maxElasticRotation;
	m_maxStrain = def->maxStrain;
	m_maxRotation = def->maxRotation;
	m_currentStrain = 0.f;
	m_currentRotation = 0.f;
	angularError = 0.f;
	positionError = 0.f;
	aInitialized = false;
	bInitialized = false;
	m_forceExceeded = false;
	m_torqueExceeded = false;
	id = epId++;
}

void b2ElasticPlasticJoint::InitVelocityConstraints(const b2SolverData& data)
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

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

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

	if (m_frequencyHz > 0.0f)
	{
		K.GetInverse22(&m_mass);

		float32 invM = iA + iB;
		float32 m = invM > 0.0f ? 1.0f / invM : 0.0f;

		// Frequency
		float32 omega = 2.0f * b2_pi * m_frequencyHz;

		// Damping coefficient
		float32 d = 2.0f * m * m_dampingRatio * omega;

		// Spring stiffness
		float32 k = m * omega * omega;
		m_k=k; // save for possible reuse during UpdateAnchors
		// magic formulas
		float32 h = data.step.dt;
		m_gamma = h * (d + h * k);
		m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
		invM += m_gamma;
		m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
		if(m_maxElasticRotation!=0.f){
			m_maxTorque = k*m_maxElasticRotation;
		}
		float32 C = aB - aA - m_referenceAngle;
		m_bias = C * h * k * m_gamma;
	}
	else if (K.ez.z == 0.0f)
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

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= data.step.dtRatio;

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

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	// ep
	m_maxImpulse.z = m_maxTorque*data.step.dt;
	// rotate original maxForce to match current average rotation
	// this could probably made more precise but this scenario is seldom
	// significant
	b2Vec2 rotatedForce = GetRotatedMaxForce();
	m_maxImpulse.x = (rotatedForce.x)*data.step.dt;
	m_maxImpulse.y = (rotatedForce.y)*data.step.dt;
}

/**
during velocity iterations
*/
void b2ElasticPlasticJoint::UpdateAnchors(const b2SolverData & data)
{
	if (!m_forceExceeded && !m_torqueExceeded) {
		return;
	}
	float32 elasticPart = 0.f;
	if (m_frequencyHz > 0.f) {
		elasticPart = 0.f; // TODO, define better
	}
	b2Vec2 cA = data.positions[m_indexA].c;
	b2Vec2 cB = data.positions[m_indexB].c;

	float32 newDistance = (1.f - elasticPart)*(cA - cB).Length();
	float32 origDistance = m_localAnchorA.Length() + m_localAnchorB.Length();
	// this needs more analysis
	// current implementation is based on idea
	// that tearing joint up joint should be more meaningful
	// than pushing
	if (m_forceExceeded || origDistance < newDistance) {
		float32 sf = newDistance / origDistance;
		m_localAnchorA *= sf;
		m_localAnchorB *= sf;
		m_currentStrain += b2Abs(newDistance - origDistance);
	}
	m_forceExceeded = false;
	if (!m_torqueExceeded) {
		return;
	}
	if (m_frequencyHz == 0.f) {
		updateRotationalPlasticity(data, 0.f);
	}
	else {
		float32 h = data.step.dt;
		float32 elasticRotation = m_maxImpulse.z / h / m_k;
		updateRotationalPlasticity(data, elasticRotation);
	}
	m_torqueExceeded = false;
}

/**
* After velocity iterations
*/
void b2ElasticPlasticJoint::UpdatePlasticity(const b2SolverData & data)
{
	if (!m_forceExceeded && !m_torqueExceeded) {
		return;
	}
	float32 elasticPart = 0.f;
	if (m_frequencyHz > 0.f) {
		elasticPart = 0.f; // TODO, define better
	}
	b2Vec2 cA = data.positions[m_indexA].c;
	b2Vec2 cB = data.positions[m_indexB].c;

	float32 newDistance = (1.f - elasticPart)*(cA - cB).Length();
	float32 origDistance = m_localAnchorA.Length() + m_localAnchorB.Length();
	// this needs more analysis
	// current implementation is based on idea
	// that tearing joint up joint should be more meaningful
	// than pushing
	if (m_forceExceeded || origDistance < newDistance) {
		float32 sf = newDistance / origDistance;
		m_localAnchorA *= sf;
		m_localAnchorB *= sf;
		m_currentStrain += b2Abs(newDistance - origDistance);
	}
	m_forceExceeded = false;
	if (!m_torqueExceeded) {
		return;
	}
	if (m_frequencyHz == 0.f) {
		updateRotationalPlasticity(data, 0.f);
	}
	else {
		float32 h = data.step.dt;
		float32 elasticRotation = m_maxImpulse.z / h / m_k;
		updateRotationalPlasticity(data, elasticRotation);
	}
	m_torqueExceeded = false;
}


void b2ElasticPlasticJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[m_indexA].v;
	float32 wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float32 wB = data.velocities[m_indexB].w;

	float32 mA = m_invMassA, mB = m_invMassB;
	float32 iA = m_invIA, iB = m_invIB;

	if (m_frequencyHz > 0.0f)
	{
		float32 Cdot2 = wB - wA;

		float32 impulse2 = GetClampedDeltaImpulse(Cdot2, data);
		m_impulse.z += impulse2;

		wA -= iA * impulse2;
		wB += iB * impulse2;

		b2Vec2 Cdot1 = vB + b2Cross(wB, m_rB) - vA - b2Cross(wA, m_rA);

		b2Vec2 impulse1 = GetClampedDeltaImpulse(Cdot1, data);
		m_impulse.x += impulse1.x;
		m_impulse.y += impulse1.y;

		b2Vec2 P = impulse1;

		vA -= mA * P;
		wA -= iA * b2Cross(m_rA, P);

		vB += mB * P;
		wB += iB * b2Cross(m_rB, P);
	}
	else
	{
		b2Vec2 vAa = vA + b2Cross(wA, m_rA);
		b2Vec2 vBa = vB + b2Cross(wB, m_rB);
		b2Vec2 Cdot1 =  vBa- vAa;
		float32 Cdot2 = wB - wA;
		b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

		// ep, limit impulse so that m_impulse does not exceed
		// m_maxImpulse
		b2Vec3 impulse = GetClampedDeltaImpulse(Cdot, data);
		m_impulse += impulse;

		b2Vec2 P(impulse.x, impulse.y);

		vA -= mA * P;
		wA -= iA * (b2Cross(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (b2Cross(m_rB, P) + impulse.z);
#ifdef SN_LOG
		b2Log("J:VC m_impulse=%e %e %e\n",
			m_impulse.x,m_impulse.y,m_impulse.z);
		b2Log("J:VC impulse=%e %e %e\n", 
			impulse.x, impulse.y, impulse.z);
		if (mA != 0) {
			b2Log("J:VC vA=%e %e %e\n",
				vA.x, vA.y, wA);
		}
		if (mB != 0) {
			b2Log("J:VC vB=%e %e %e\n",
				vB.x, vB.y, wB);
		}
#endif
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	UpdateAnchors(data);
}

bool b2ElasticPlasticJoint::WantsToBreak(){
	if (m_currentRotation > m_maxRotation){
		return true;
	}
	if (m_currentStrain > m_maxStrain){
		return true;
		// TODO,
		// avoid breaking due to pure compression
		//
		b2Vec2 iv;
		iv.x = m_impulse.x;
		iv.y = m_impulse.y;
		float32 dotv=b2Dot(m_localAnchorA,iv);
		if (dotv<0){
			return true;
		}
	}
	return false;
}
/**
*/
b2Vec3 b2ElasticPlasticJoint::GetClampedDeltaImpulse(b2Vec3 Cdot, 
	const b2SolverData& data){
	b2Vec3 impulse = -b2Mul(m_mass, Cdot);
	b2Vec3 clamped;
	b2Vec3 maxImpulse = GetClampedMaxImpulse(Cdot, data);
	b2Vec3 high = maxImpulse - m_impulse;
	b2Vec3 low = -maxImpulse - m_impulse;
	clamped.x = b2Clamp(impulse.x, low.x, high.x);
	clamped.y = b2Clamp(impulse.y, low.y, high.y);
	if (impulse.x != clamped.x || impulse.y != clamped.y){
		m_forceExceeded = true;
	}
	else {
		m_forceExceeded = false;
	}
	clamped.z = b2Clamp(impulse.z, low.z, high.z);
	if (impulse.z != clamped.z){
		m_torqueExceeded = true;
	}
	else {
		m_torqueExceeded = false;
	}
	return clamped;
}
/**
* scale maxImpulse if joint is about to break
*/
b2Vec3 b2ElasticPlasticJoint::GetClampedMaxImpulse(b2Vec3 Cdot, 
	const b2SolverData& data) {
	float32 dt = data.step.dt;
	b2Vec3 d = dt*Cdot;
	float32 maxStrain = m_maxStrain - m_currentStrain;
	float32 maxRotation = m_maxRotation - m_currentRotation;
	float32 ssx = (d.x !=0 ? maxStrain / b2Abs(d.x) :b2_maxFloat);
	float32 ssy = (d.y != 0 ? maxStrain / b2Abs(d.y) :b2_maxFloat);
	float32 sr = (d.z != 0 ? maxRotation / b2Abs(d.z) :b2_maxFloat);
	float32 s = b2Min(b2Min(ssx, ssy),sr);
	if (s > 1) {
		return m_maxImpulse;
	}
	else {
		return s*m_maxImpulse;
	}
}

b2Vec2 b2ElasticPlasticJoint::GetClampedDeltaImpulse(b2Vec2 Cdot, 
	const b2SolverData& data){
	b2Vec2 impulse = -b2Mul22(m_mass, Cdot);
	b2Vec2 clamped;
	b2Vec3 maxImpulse = GetClampedMaxImpulse(Cdot, data);
	b2Vec3 high = maxImpulse - m_impulse;
	b2Vec3 low = -maxImpulse - m_impulse;
	clamped.x = b2Clamp(impulse.x, low.x, high.x);
	clamped.y = b2Clamp(impulse.y, low.y, high.y);
	if (impulse.x != clamped.x || impulse.y != clamped.y){
		m_forceExceeded = true;
	}
	else {
		m_forceExceeded = false;
	}
	return clamped;
}

/**
* scale maxImpulse if joint is about to break
*/
b2Vec3 b2ElasticPlasticJoint::GetClampedMaxImpulse(b2Vec2 Cdot, 
	const b2SolverData& data) {
	float32 dt = data.step.dt;
	b2Vec2 d = dt*Cdot;
	float32 maxStrain = m_maxStrain - m_currentStrain;
	float32 ssx = (d.x!=0?maxStrain / b2Abs(d.x):b2_maxFloat);
	float32 ssy = (d.y!=0?maxStrain / b2Abs(d.y):b2_maxFloat);
	float32 s =b2Min(ssx, ssy);
	if (s > 1) {
		return m_maxImpulse;
	}
	else {
		return s*m_maxImpulse;
	}
}


float32 b2ElasticPlasticJoint::GetClampedDeltaImpulse(float32 Cdot, 
	const b2SolverData& data){
	float32 impulse = -m_mass.ez.z * (Cdot + m_bias + m_gamma * m_impulse.z);
	float32 clamped;
	float32 maxImpulse = GetClampedMaxImpulse(Cdot, data);
	float32 high = maxImpulse - m_impulse.z;
	float32 low = -maxImpulse - m_impulse.z;
	clamped = b2Clamp(impulse, low, high);
	if (impulse != clamped){
		m_torqueExceeded = true;
	}
	else {
		m_torqueExceeded = false;
	}
	return clamped;
}

#define CLAMP_POS_CORRECTION_IMPULSE
/**
* Clamp impulse in position correction
*/
float32 b2ElasticPlasticJoint::Clamp(float32 M,
	const b2SolverData& data /* unused */) {
#ifdef CLAMP_POS_CORRECTION_IMPULSE
	float32 low = -m_maxImpulse.z - m_impulse.z;
	float32 high = m_maxImpulse.z - m_impulse.z;
	float32 clamped = b2Clamp(M,low, high);
	if (clamped != M) {
		bool placeHolder = true;
	}
	return clamped;
#else
	return M;
#endif
}

/**
* Clamp impulse in position correction
*/
b2Vec2 b2ElasticPlasticJoint::Clamp(b2Vec2 P,
	const b2SolverData& data /* unused */) {
#ifdef CLAMP_POS_CORRECTION_IMPULSE
	b2Vec2 low = b2Vec2(-m_maxImpulse.x - m_impulse.x ,-m_maxImpulse.y - m_impulse.y);
	b2Vec2 high = b2Vec2(m_maxImpulse.x - m_impulse.x, m_maxImpulse.y - m_impulse.y);
	b2Vec2 clamped = b2Clamp(P, low, high);
	if (clamped.x != P.x || clamped.y!=P.y) {
		bool placeHolder = true;
	}
	return clamped;
#else
	return P;
#endif
}

/**
* scale maxImpulse if joint is about to break
*/
float32 b2ElasticPlasticJoint::GetClampedMaxImpulse(float32 Cdot, 
	const b2SolverData& data) {
	if (Cdot != 0) {
		float32 dt = data.step.dt;
		float32 d = dt*Cdot;
		float32 maxRotation = m_maxRotation - m_currentRotation;
		float32 s = maxRotation / b2Abs(d);
		if (s < 1) {
			return s*m_maxImpulse.z;
		}
	}
	return m_maxImpulse.z;
	
}

void b2ElasticPlasticJoint::updateRotationalPlasticity
	(const b2SolverData& data, float32 elasticRotation)
{
	float32 aA = data.positions[m_indexA].a;
	float32 aB = data.positions[m_indexB].a;

	float32 currentAngleDiff = aB - aA;
	if (m_impulse.z < 0) {
		elasticRotation =-elasticRotation;
	}
	float32 newReferenceAngle = currentAngleDiff + elasticRotation;
	m_currentRotation += b2Abs(newReferenceAngle - m_referenceAngle);
	m_referenceAngle = newReferenceAngle;
	m_torqueExceeded = false;
}

b2Vec2 b2ElasticPlasticJoint::GetRotatedMaxForce()
{
	b2Vec2 rf;
	b2Rot q((m_bodyB->GetAngle() - m_bodyA->GetAngle()) / 2);
	rf.x = (b2Abs(q.c)*m_maxForce.x + b2Abs(q.s)*m_maxForce.y);
	rf.y = (b2Abs(q.s)*m_maxForce.x + b2Abs(q.c)*m_maxForce.y);
	return rf;
}


bool b2ElasticPlasticJoint::SolvePositionConstraints(const b2SolverData& data)
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

	if (m_frequencyHz > 0.0f)
	{
		b2Vec2 C1 = cB + rB - cA - rA;

		positionError = C1.Length();
		angularError = 0.0f;

		b2Vec2 P = Clamp(-K.Solve22(C1),data);
		float32 M = Clamp(b2Cross(rA, P),data);
		cA -= mA * P;
		aA -= iA * M;

		M = Clamp(b2Cross(rB, P),data);
		cB += mB * P;
		aB += iB * M;
	}
	else
	{
		b2Vec2 C1= cB + rB - cA - rA;
		float32 C2=aB - aA - m_referenceAngle;

		positionError = C1.Length();
		angularError = b2Abs(C2);

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

		b2Vec2 P=Clamp(b2Vec2(impulse.x, impulse.y),data);

		cA -= mA * P;
		float32 M = Clamp((b2Cross(rA, P) + impulse.z), data);
#ifdef SN_LOG
		b2Log("J:PC P=%e %e, M1=%e",
			P.x, P.y, M);
#endif
		aA -= iA * M;

		cB += mB * P;
		M = Clamp((b2Cross(rB, P) + impulse.z), data);
#ifdef SN_LOG
		b2Log(", M2=%e\n", M);
#endif
		aB += iB * M;
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	jointOk=positionError <= b2ep_linearSlop && angularError <= b2ep_angularSlop;
	return jointOk;
}

void b2ElasticPlasticJoint::SetLinearSlop(float32 value){
	b2ep_linearSlop = value;
}

float32 b2ElasticPlasticJoint::GetLinearSlop(){
	return b2ep_linearSlop;
}

void b2ElasticPlasticJoint::SetAngularSlop(float32 value){
	b2ep_angularSlop = value;
}

float32 b2ElasticPlasticJoint::GetAngularSlop(){
	return b2ep_angularSlop;
}

b2Vec2 b2ElasticPlasticJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b2Vec2 b2ElasticPlasticJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

b2Vec2 b2ElasticPlasticJoint::GetReactionForce(float32 inv_dt) const
{
	b2Vec2 P(m_impulse.x, m_impulse.y);
	return inv_dt * P;
}

float32 b2ElasticPlasticJoint::GetReactionTorque(float32 inv_dt) const
{
	return inv_dt * m_impulse.z;
}

void b2ElasticPlasticJoint::SetMaxForce(b2Vec2 force)
{
	b2Assert(b2IsValid(force.x) && force.x >= 0.0f && 
		b2IsValid(force.y) && force.y >= 0.0f);
	m_maxForce = force;
}

b2Vec2 b2ElasticPlasticJoint::GetMaxForce() const
{
	return m_maxForce;
}

void b2ElasticPlasticJoint::SetMaxTorque(float32 torque)
{
	b2Assert(b2IsValid(torque) && torque >= 0.0f);
	m_maxTorque = torque;
}

void b2ElasticPlasticJoint::SetMaxElasticRotation(float32 val)
{
	m_maxElasticRotation = val;
}

float32 b2ElasticPlasticJoint::GetMaxTorque() const
{
	return m_maxTorque;
}


void b2ElasticPlasticJoint::Dump()
{
	int32 indexA = m_bodyA->m_islandIndex;
	int32 indexB = m_bodyB->m_islandIndex;

	b2Log("  b2ElasticPlasticJointDef jd;\n");
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
