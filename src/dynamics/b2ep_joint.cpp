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

#include "box2d/b2ep_joint.h"
#include "box2d/b2_body.h"
#include "box2d/b2_world.h"
#include "box2d/b2_block_allocator.h"
#include "box2d/b2_fixture.h"

/*
These are smaller than normal b2 limits as
there are often many connected joints and
error accumulates.
These can be tuned.
*/
namespace
{
	float b2ep_linearSlop=0.0005f;
	float b2ep_angularSlop=0.0005f;
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

static SOLVE_ORDER momentFirst[] = {MOMENT,FORCE};
static SOLVE_ORDER forceFirst[] = {FORCE,MOMENT};

SOLVE_ORDER* b2ElasticPlasticJoint::getMomentFirst() {
	return momentFirst;
}
SOLVE_ORDER* b2ElasticPlasticJoint::getForceFirst() {
	return forceFirst;
}

bool b2ElasticPlasticJoint::isOverLoaded(OVERLOAD_DIRECTION d)
{
	switch (d) {
	case ANY:
		return overLoads.any();
	}
	return overLoads.test(d);
}

void b2ElasticPlasticJoint::setOverLoaded(OVERLOAD_DIRECTION d, bool v)
{
	overLoads.set(d,v);
}

bool b2ElasticPlasticJoint::wasOverLoaded(OVERLOAD_DIRECTION d)
{
	switch (d) {
	case ANY:
		return savedOverLoads.any();
	}
	return savedOverLoads.test(d);
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
	switch (def->type) {
	case e_elasticPlasticJoint:
		m_frequencyHz = def->frequencyHz;
		m_dampingRatio = def->dampingRatio;
		m_maxElasticRotation = def->maxElasticRotation;
		break;
	case e_rigidPlasticJoint:
		m_frequencyHz = 0.f;
		m_dampingRatio = 0.f;
		m_maxElasticRotation = 0.f;
		break;
	}
	m_impulse.SetZero();
	m_jim.SetZero();
	m_maxForce = def->maxForce;
	m_maxTorque = def->maxTorque;
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
	debugListener = nullptr;
	impulseInitializer = nullptr;
	positionIteration = 0;
	velocityIteration = 0;
	m_bias = 0.f;
	m_gamma = 0.f;
	initImpulseDone = false;
}

b2ElasticPlasticJoint::~b2ElasticPlasticJoint()
{
	if (nullptr != impulseInitializer) {
		m_bodyA->m_world->m_blockAllocator.Free
			(impulseInitializer, sizeof(b2ImpulseInitializer));
		impulseInitializer = nullptr;
	}
}

b2ImpulseInitializer * b2ElasticPlasticJoint::GetImpulseInitializer()
{
	if (nullptr == impulseInitializer) {
		void* mem = m_bodyA->m_world->m_blockAllocator.Allocate
			(sizeof(b2ImpulseInitializer));
		impulseInitializer = new (mem)b2ImpulseInitializer();
	}
	return impulseInitializer;
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

	float aA = data.positions[m_indexA].a;
	b2Vec2 vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;

	float aB = data.positions[m_indexB].a;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

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

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	b2Mat33 K;
	K.ez.z = iA + iB;
	if (m_frequencyHz > 0.0f || K.ez.z == 0.f) {
		K.ex.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
		K.ey.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
		K.ez.x = -m_rA.y * iA - m_rB.y * iB;
		K.ex.y = K.ey.x;
		K.ey.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
		K.ez.y = m_rA.x * iA + m_rB.x * iB;
		K.ex.z = K.ez.x;
		K.ey.z = K.ez.y;
	}
	solveOrder = momentFirst;
	if (m_frequencyHz > 0.0f || K.ez.z != 0.0f)
	{

		float invM = iA + iB;
		float m = invM > 0.0f ? 1.0f / invM : 0.0f;

		if (m_frequencyHz > 0.0f) {
			K.GetInverse22(&m_mass);
			float h = data.step.dt;
			// Frequency
			float omega = 2.0f * b2_pi * m_frequencyHz;
			// Damping coefficient
			float d = 2.0f * m * m_dampingRatio * omega;
			// Spring stiffness
			float k = m * omega * omega;
			m_k = k; // save for possible reuse during UpdateAnchors
	  		// magic formulas
			m_gamma = h * (d + h * k);
			m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
			invM += m_gamma;
			if (m_maxElasticRotation != 0.f) {
				m_maxTorque = m_k*m_maxElasticRotation;
			}
			float C = aB - aA - m_referenceAngle;
			m_bias = C * h * m_k * m_gamma;
		}
		else {
			m_k = 0.f;
			m_gamma = 0.f;
			m_bias = 0.f;
			float iM = mA + mB;
			m_mass.ex.SetZero();
			m_mass.ey.SetZero();
			m_mass.ez.SetZero();
			m_mass.ex.x = iM != 0.f ? 1.f / iM : 0.f;
			m_mass.ey.y = m_mass.ex.x;
			solveOrder = forceFirst;
		}
		m_mass.ez.z = invM != 0.0f ? 1.0f / invM : 0.0f;
	}
	else if (K.ez.z == 0.0f)
	{
		K.GetInverse22(&m_mass);
		m_gamma = 0.0f;
		m_bias = 0.0f;
	}

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

/**
during velocity iterations
*/
void b2ElasticPlasticJoint::UpdateAnchors(const b2SolverData & data)
{
	if (!m_forceExceeded && !m_torqueExceeded) {
		return;
	}
	float elasticPart = 0.f;
	if (m_frequencyHz > 0.f) {
		elasticPart = 0.f; // TODO, define better
	}
	b2Vec2 cA = data.positions[m_indexA].c;
	b2Vec2 cB = data.positions[m_indexB].c;

	float newDistance = (1.f - elasticPart)*(cA - cB).Length();
	float origDistance = m_localAnchorA.Length() + m_localAnchorB.Length();
	// this needs more analysis
	// current implementation is based on idea
	// that tearing joint up joint should be more meaningful
	// than pushing
	if (m_forceExceeded && origDistance < newDistance) {
		float sf = newDistance / origDistance;
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
		float h = data.step.dt;
		float elasticRotation = m_maxImpulse.z / h / m_k;
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
	float elasticPart = 0.f;
	if (m_frequencyHz > 0.f) {
		elasticPart = 0.f; // TODO, define better
	}
	b2Vec2 cA = data.positions[m_indexA].c;
	b2Vec2 cB = data.positions[m_indexB].c;

	float newDistance = (1.f - elasticPart)*(cA - cB).Length();
	float origDistance = m_localAnchorA.Length() + m_localAnchorB.Length();
	// this needs more analysis
	// current implementation is based on idea
	// that tearing joint up joint should be more meaningful
	// than pushing
	if (m_forceExceeded && origDistance < newDistance) {
		float sf = newDistance / origDistance;
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
		float h = data.step.dt;
		float elasticRotation = m_maxImpulse.z / h / m_k;
		updateRotationalPlasticity(data, elasticRotation);
	}
	m_torqueExceeded = false;
}


void b2ElasticPlasticJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	b2Vec2 vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	b2Vec2 vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;
#ifdef EP_LOG
	if (epLogActive && epLogEnabled) {
		if (mA != 0) {
			epLog("J:VC:%d vA1=%g %g %g\n",id,
				vA.x, vA.y, wA);
		}
		if (mB != 0) {
			epLog("J:VC:%d vB1=%g %g %g\n",id,
				vB.x, vB.y, wB);
		}
		epLog("J:VC:%d m_impulse1=%g %g %g\n",id,
			m_impulse.x, m_impulse.y, m_impulse.z);
	}
#endif
	if (nullptr != debugListener) {
		debugListener->BeginVelocityIteration(this, data);
	}
	b2Vec2 Cdot1;
	float Cdot2=0,impulse2;
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

bool b2ElasticPlasticJoint::WantsToBreak(){
	if (m_currentRotation > m_maxRotation){
		return true;
	}
	if (m_currentStrain > m_maxStrain){
		if(true) return true;
		// TODO,
		// avoid breaking due to pure compression
		//
		b2Vec2 iv;
		iv.x = m_impulse.x;
		iv.y = m_impulse.y;
		float dotv=b2Dot(m_localAnchorA,iv);
		if (dotv<0){
			return true;
		}
	}
	return false;
}

b2Vec2 b2ElasticPlasticJoint::GetClampedDeltaImpulse(b2Vec2 p_Cdot, 
	const b2SolverData& data){
	b2Vec2 impulse = -b2Mul22(m_mass, p_Cdot);
	b2Vec2 clamped;
	b2Vec3 maxImpulse;
	if (m_forceExceeded) {
		maxImpulse = GetClampedMaxImpulse(p_Cdot, data);
	}
	else {
		maxImpulse = m_maxImpulse;
	}
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
b2Vec3 b2ElasticPlasticJoint::GetClampedMaxImpulse(b2Vec2 p_Cdot, 
	const b2SolverData& data) {
	if (p_Cdot.LengthSquared() < b2ep_linearSlop) {
		return m_maxImpulse;
	}
	float dt = data.step.dt;
	b2Vec2 d = dt*p_Cdot;
	float maxStrain = m_maxStrain - m_currentStrain;
	float ssx = (d.x!=0?maxStrain / b2Abs(d.x):b2_maxFloat);
	float ssy = (d.y!=0?maxStrain / b2Abs(d.y):b2_maxFloat);
	float s =b2Min(ssx, ssy);
	if (s > 1) {
		return m_maxImpulse;
	}
	else {
		return s*m_maxImpulse;
	}
}


float b2ElasticPlasticJoint::GetClampedDeltaImpulse(float p_Cdot, 
	const b2SolverData& data){
	float impulse = -m_mass.ez.z * (p_Cdot + m_bias + m_gamma * m_impulse.z);
	float clamped;
	float maxImpulse;
	if (m_torqueExceeded) {
		maxImpulse = GetClampedMaxImpulse(p_Cdot, data);
	}
	else {
		maxImpulse = m_maxImpulse.z;
	}
	float high = maxImpulse - m_impulse.z;
	float low = -maxImpulse - m_impulse.z;
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
float b2ElasticPlasticJoint::Clamp(float M,
	const b2SolverData&  /* unused */) {
#ifdef CLAMP_POS_CORRECTION_IMPULSE
	float low = -m_maxImpulse.z - m_impulse.z;
	float high = m_maxImpulse.z - m_impulse.z;
	float clamped = b2Clamp(M,low, high);
	if (clamped != M) {
		{}
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
	const b2SolverData&  /* unused */) {
#ifdef CLAMP_POS_CORRECTION_IMPULSE
	b2Vec2 low = b2Vec2(-m_maxImpulse.x - m_impulse.x ,-m_maxImpulse.y - m_impulse.y);
	b2Vec2 high = b2Vec2(m_maxImpulse.x - m_impulse.x, m_maxImpulse.y - m_impulse.y);
	b2Vec2 clamped = b2Clamp(P, low, high);
	if (clamped.x != P.x || clamped.y!=P.y) {
		{}
	}
	return clamped;
#else
	return P;
#endif
}

/**
* scale maxImpulse if joint is about to break
* avoid processing of small velocities
* as dividing by denormal numbers may trigger numerical issues
*/
float b2ElasticPlasticJoint::GetClampedMaxImpulse(float p_Cdot, 
	const b2SolverData& data) {
	float aCdot = b2Abs(p_Cdot);
	if (aCdot>b2ep_angularSlop) {
		float dt = data.step.dt;
		float d = dt*aCdot;
		float maxRotation = m_maxRotation - m_currentRotation;
		float s = maxRotation/d;
		if (s < 1) {
			return s*m_maxImpulse.z;
		}
	}
	return m_maxImpulse.z;
	
}

void b2ElasticPlasticJoint::updateRotationalPlasticity
	(const b2SolverData& data, float elasticRotation)
{
	float aA = data.positions[m_indexA].a;
	float aB = data.positions[m_indexB].a;

	float currentAngleDiff = aB - aA;
	if (m_impulse.z < 0) {
		elasticRotation =-elasticRotation;
	}
	float newReferenceAngle = currentAngleDiff + elasticRotation;
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
	float aA = data.positions[m_indexA].a;
	b2Vec2 cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	b2Rot qA(aA), qB(aB);

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

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

	if (m_frequencyHz > 0.0f)
	{
		b2Vec2 C1 = cB + rB - cA - rA;

		positionError = C1.Length();
		angularError = 0.0f;

		b2Vec2 P = Clamp(-K.Solve22(C1),data);
		float M = Clamp(b2Cross(rA, P),data);
		cA -= mA * P;
		aA -= iA * M;

		M = Clamp(b2Cross(rB, P),data);
		cB += mB * P;
		aB += iB * M;
	}
	else
	{
		b2Vec2 C1= cB + rB - cA - rA;
		float C2=aB - aA - m_referenceAngle;

		positionError = C1.Length();
		angularError = b2Abs(C2);
#ifdef EP_LOG
		epLog("J:PC:%d positionError=%g, angularError=%g\n",id,
			positionError,angularError);
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

		b2Vec2 P=Clamp(b2Vec2(impulse.x, impulse.y),data);

		cA -= mA * P;
		float M = Clamp((b2Cross(rA, P) + impulse.z), data);
#ifdef EP_LOG
		epLog("J:PC:%d P=%g %g, M1=%g",id,
			P.x, P.y, M);
#endif
		aA -= iA * M;

		cB += mB * P;
		M = Clamp((b2Cross(rB, P) + impulse.z), data);
#ifdef EP_LOG
		epLog(", M2=%g\n", M);
#endif
		aB += iB * M;
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	jointOk=positionError <= b2ep_linearSlop && angularError <= b2ep_angularSlop;
	if (nullptr != debugListener) {
		debugListener->EndPositionIteration(this, data);
	}
	positionIteration++;
	return jointOk;
}

b2Vec3 b2ElasticPlasticJoint::GetMaxImpulse(float dt)
{
	b2Vec3 mi = b2Vec3();
	mi.z = m_maxTorque*dt;
	// rotate original maxForce to match current average rotation
	// this could probably made more precise but this scenario is seldom
	// significant
	b2Vec2 rotatedForce = GetRotatedMaxForce();
	mi.x = (rotatedForce.x)*dt;
	mi.y = (rotatedForce.y)*dt;
	return mi;
}

bool b2ElasticPlasticJoint::CanHandleInitialImpulse(const b2SolverData * data)
{
	b2Vec3 mi = GetMaxImpulse(data->step.dt);
	if (b2Abs(m_impulse.x) > mi.x) {
		return false;
	}
	if (b2Abs(m_impulse.y) > mi.y) {
		return false;
	}
	if (b2Abs(m_impulse.z) > mi.z) {
		return false;
	}
	return true;
}

void b2ElasticPlasticJoint::SetLinearSlop(float value){
	b2ep_linearSlop = value;
}

float b2ElasticPlasticJoint::GetLinearSlop(){
	return b2ep_linearSlop;
}

void b2ElasticPlasticJoint::SetAngularSlop(float value){
	b2ep_angularSlop = value;
}

float b2ElasticPlasticJoint::GetAngularSlop(){
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

b2Vec2 b2ElasticPlasticJoint::GetReactionForce(float inv_dt) const
{
	b2Vec2 P(m_impulse.x, m_impulse.y);
	return inv_dt * P;
}

float b2ElasticPlasticJoint::GetReactionTorque(float inv_dt) const
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

void b2ElasticPlasticJoint::SetMaxTorque(float torque)
{
	b2Assert(b2IsValid(torque) && torque >= 0.0f);
	m_maxTorque = torque;
}

void b2ElasticPlasticJoint::SetMaxElasticRotation(float val)
{
	m_maxElasticRotation = val;
}


float b2ElasticPlasticJoint::GetMaxTorque() const
{
	return m_maxTorque;
}

bool hasPositionIterations() {
	return true;
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
#include "box2d/b2ep_joint.h"

b2RigidJointHandler::b2RigidJointHandler()
{
}

void b2RigidJointHandler::handle()
{
	reset();
	checkLimits();
	handleLoads();
	updateBodies();
}

void b2RigidJointHandler::reset()
{
	mbi = masterJoint->m_mbi;
}

void b2RigidJointHandler::handleLoads()
{
	handleMoment();
	handleForce();
}

void b2RigidJointHandler::updateBodies()
{
	b2Vec2 cA= data->positions[mbi].c;
	// float aA = data->positions[mbi].a;
	b2Vec2 vA = data->velocities[mbi].v;
	float wA = data->velocities[mbi].w;
	b2Rot r;
	r.SetIdentity();
	for (int32 i = 0; i < ejCount; i++) {
		b2ElasticPlasticJoint* joint = ejStack[i];
		int ib = joint->m_indexB;
		if (ib == mbi) {
			continue;
		}
		b2Vec2 cB = data->positions[ib].c;
		b2Vec2 vB= vA + b2Cross(wA, cB - cA);
		float wB=wA;
		data->velocities[ib].v = vB;
		data->velocities[ib].w = wB;
	}
}

void b2RigidJointHandler::handleForce()
{
}

/**
* for non overload
* reset to state at beginning of step
* for overload
* get joint inertia of b-bodies
* and update velocities 
*/
void b2RigidJointHandler::handleMoment()
{
	/* velocities from beginning of step are used */
	b2Body* b = masterJoint->m_bodyB;
	b2Vec2 v = b->GetLinearVelocity();
	float w = b->GetAngularVelocity();
	if (!masterJoint->isOverLoaded(RZ)) {
		int32 mba = masterJoint->m_indexA;
		mbi = mba;
		int32 mbb = masterJoint->m_indexB;
		data->velocities[mbb].v = v;
		data->velocities[mbb].w = w ;
		return;
	}
	float jm = 0.f;
	float ji = 0.f;
	for (int32 i = 0; i < ejCount; i++) {
		b2ElasticPlasticJoint* joint = ejStack[i];
		b2Body* bb = joint->m_bodyB;
		float m = bb->GetMass();
		jm += m;
		float mr2 = m * joint->m_rB.LengthSquared();
		ji += mr2;
		joint->setOverLoaded(RZ);
	}
	mbi = masterJoint->m_indexB;
	if (masterJoint->m_mbi != mbi) {
		masterJoint->m_mbi = mbi;
		data->velocities[mbi].w = w;
		data->velocities[mbi].v = v + b2Cross(w, masterJoint->m_rB);
	}
	float m = masterJoint->m_jim.z;
	float mm = masterJoint->m_maxImpulse.z;
	float em;
	if (m > mm) {
		em = m - mm;
	}
	else if(m<-mm) {
		em = m + mm;
	}
	else {
		em = m;
	}
	float dw = -em / ji;
	b2Vec2 dv = -b2Cross(dw, masterJoint->m_rB);
	data->velocities[mbi].v = v+dv;
	data->velocities[mbi].w = w+dw;
}

void b2RigidJointHandler::checkLimits()
{
	bool v = b2Abs(masterJoint->m_jim.z) >= masterJoint->m_maxImpulse.z;
	masterJoint->setOverLoaded(RZ, v);
	v = b2Abs(masterJoint->m_jim.x) >= masterJoint->m_maxImpulse.x;
	masterJoint->setOverLoaded(X, v);
	v = b2Abs(masterJoint->m_jim.y) >= masterJoint->m_maxImpulse.y;
	masterJoint->setOverLoaded(Y, v);
}
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

#include "box2d/b2ep_joint.h"
#include "box2d/b2_body.h"
#include "box2d/b2_world.h"
#include "box2d/b2_block_allocator.h"
#include "box2d/b2_fixture.h"

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
}/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
* Simo Nikula 4/2017 
*/

#include "box2d/b2_Distance.h"
#include "dynamics/b2_Island.h"
#include "box2d/b2ep_joint.h"
#include "box2d/b2_Body.h"
#include "box2d/b2_Fixture.h"
#include "box2d/b2_World.h"
#include "box2d/b2_Contact.h"
#include "dynamics/b2_Contact_Solver.h"
#include "box2d/b2_Joint.h"
#include "Box2d/b2_Stack_Allocator.h"
#include "Box2d/b2_Timer.h"

bool b2ImpulseInitializer::IsInitImpulsesNeeded(b2Island *ic)
{
	return bodyCount!=ic->m_bodyCount 
		|| jointCount!=ic->m_jointCount
		|| contactCount!=ic->m_contactCount;
}

void b2ImpulseInitializer::InitImpulses(){
	for (int32 i = 0; i < epCount; i++){
		b2ElasticPlasticJoint* joint = epStack[i];
		joint->m_impulse.SetZero();
		joint->aInitialized = false;
		joint->bInitialized = false;
	}
	for (int32 i = 0; i < startJointCount; i++){
		currentStartJoint = sjStack[i];
		addImpulses(currentStartJoint);
		//checkImpulses(currentStartJoint);
	}
}

/**
Loop over all bodies that are connected to startJoint with 
b2ElasticPlasticJoints until some other startJoint is nearer 
than current one.
Currently start point is assumed to be connected to rigid body.
*/
b2Vec3 b2ImpulseInitializer::addImpulses(b2ElasticPlasticJoint* startJoint){
	float h = solverData->step.dt;
	b2Body* b = startJoint->GetBodyA();
	b2Vec2 jointPoint = startJoint->GetAnchorA();
	b2Vec2 d,sp;
	if (b->GetType() == b2_dynamicBody && !startJoint->aInitialized){
		d = startJoint->GetLocalAnchorA();
		sp = startJoint->GetAnchorA();
		startJoint->aInitialized = true;
	}
	else{
		b = startJoint->GetBodyB();
		d = startJoint->GetLocalAnchorB();
		sp = startJoint->GetAnchorB();
		startJoint->bInitialized = true;
	}
	b2Vec3 p;
	float sm = b->GetMass()*b->m_gravityScale;
	b2Vec2 f = sm* (*gravity) + b->m_force;
	float m = b->m_torque - b2Cross(d, f);
	p.Set(f.x, f.y, m);
	p = h*p;
	// add impulses from contacts
	b2Vec3 ci=getContactImpulses(b);
	p.x += ci.x;
	p.y += ci.y;
	b2Vec2 cf;
	cf.x = ci.x;
	cf.y = ci.y;
	p.z += ci.z - b2Cross(d, cf);
	startJoint->m_impulse -= p;
	b2ElasticPlasticJoint* nextJoint;
	while ((nextJoint = getNextJoint(startJoint)) != NULL){
		b2Vec3 np = addImpulses(nextJoint);
		b2Vec2 njf(np.x,np.y);
		// for rigidPlastic both anchors are at same point
		b2Vec2 jd = nextJoint->GetAnchorA()-sp; 
		np.z += b2Cross(jd, njf);
		startJoint->m_impulse += np;
	}
	startJoint->initImpulseDone = true;
	return startJoint->m_impulse;
}
/** check if joint can handle impulse and stop bodies
 if impulse can be handled
 TODO partial
*/
void b2ImpulseInitializer::checkImpulses
	(b2ElasticPlasticJoint * startJoint)
{
	for (b2ElasticPlasticJoint* j = startJoint; 
		j != NULL; 
		j = getNextJoint(j)) {
		if (!j->CanHandleInitialImpulse(solverData)) {
			return;
		}
	}

	for (b2ElasticPlasticJoint* j = startJoint;
		j != NULL;
		j = getNextJoint(j)) {
		b2Body* ba = j->m_bodyA;
		b2Body* bb = j->m_bodyB;
		solverData->velocities[ba->m_islandIndex].v=b2Vec2(0.f,0.f);
		solverData->velocities[ba->m_islandIndex].w = 0.f;
		solverData->velocities[bb->m_islandIndex].v = b2Vec2(0.f, 0.f);
		solverData->velocities[bb->m_islandIndex].w = 0.f;
		for (int32 i = 0; i < island->m_contactCount; ++i)
		{
			b2Contact* c = island->m_contacts[i];
			b2Fixture* fA = c->GetFixtureA();
			b2Fixture* fB = c->GetFixtureB();
			// Skip sensors
			if (fA->IsSensor() || fB->IsSensor())
			{
				continue;
			}
			b2Body* bA = fA->GetBody();
			b2Body* bB = fB->GetBody();
			if (bA == ba || bA==bb) {
				solverData->velocities[bB->m_islandIndex].v= b2Vec2(0.f, 0.f);
				solverData->velocities[bB->m_islandIndex].w = 0.f;
			}
			if(bB == ba || bB ==bb) {
				solverData->velocities[bA->m_islandIndex].v = b2Vec2(0.f, 0.f);
				solverData->velocities[bA->m_islandIndex].w = 0.f;
			}
		}
	}
}
/**
* These impulses will stop other body
*/
b2Vec3 b2ImpulseInitializer::getContactImpulses(b2Body * b)
{
	b2Vec3 cv=b2Vec3();
	cv.SetZero();
	for (int32 i = 0; i < island->m_contactCount; ++i)
	{
		b2Contact* c = island->m_contacts[i];
		b2Fixture* fA = c->GetFixtureA();
		b2Fixture* fB = c->GetFixtureB();
		// Skip sensors
		if (fA->IsSensor() || fB->IsSensor())
		{
			continue;
		}
		b2Body* bA = fA->GetBody();
		b2Body* bB = fB->GetBody();
		b2Body *bO;
		if (bA == b) {
			bO = bB;
		}
		else if (bB == b) {
			bO = bA;
		}
		else {
			continue;
		}
		b2MassData massData=bO->GetMassData();
		float mO = bO->m_mass;
		if (mO == 0.f) {
			continue;
		}
		float iO = bO->m_I;
		int32  bi = bO->m_islandIndex;
		// float aO = solverData->positions[bi].a;
		b2Vec2 pO = solverData->positions[bi].c;
		b2Vec2 vO = solverData->velocities[bi].v;
		float wO = solverData->velocities[bi].w;
		cv.x += mO*vO.x;
		cv.y += mO*vO.y;
		b2Vec2 f(cv.x,cv.y);
		b2Vec2 d;
		d = b->GetWorldCenter() - bO->GetWorldCenter();
		cv.z += iO*wO- b2Cross(d, f);
	}
	return cv;
}
b2ElasticPlasticJoint* b2ImpulseInitializer::getNextJoint
	(b2ElasticPlasticJoint* startJoint){
	b2Body* bA = startJoint->GetBodyA();
	b2Body* bB = startJoint->GetBodyB();
	for (int32 i = 0; i < epCount; i++){
		b2ElasticPlasticJoint* joint = epStack[i];
		if (joint == startJoint){
			continue;
		}
		bool foundNext = false;
		b2Body* nA = joint->GetBodyA();
		b2Body* nB = joint->GetBodyB();
		if ((nA == bA || nB==bA) && bA->GetType() == b2_dynamicBody){
			foundNext = true;
			if (startJoint->aInitialized){
				if (nA == bA){
					joint->aInitialized = true;
				}
				else{
					joint->bInitialized = true;
				}
			}
		}
		if ((nA == bB || nB == bB) && bB->GetType() == b2_dynamicBody){
			foundNext = true;
			if (startJoint->bInitialized){
				if (nA == bB){
					joint->aInitialized = true;
				}
				else{
					joint->bInitialized = true;
				}
			}
		}
		if (!foundNext){
			continue;
		}
		if ((joint->aInitialized || nA->GetType() != b2_dynamicBody) &&
			(joint->bInitialized || nB->GetType() != b2_dynamicBody)){
			continue;
		}
		if (isNearEnough(joint)){
			return joint;
		}
	}
	return NULL;
}

/**
@return true if there no other starting point nearer
*/
bool b2ImpulseInitializer::isNearEnough
(b2ElasticPlasticJoint* joint){
	b2Vec2 jp = joint->GetAnchorA();
	float d = (currentStartJoint->GetAnchorA()-jp).LengthSquared();
	for (int32 i = 0; i < startJointCount; i++){
		b2ElasticPlasticJoint* startJoint = sjStack[i];
		if (startJoint == joint){
			continue;
		}
		float dc = (startJoint->GetAnchorA() - jp).LengthSquared();
		if (dc < d){
			return false;
		}
	}
	return true;
}