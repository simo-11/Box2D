#include "b2RigidJointHandler.h"

b2RigidJointHandler::b2RigidJointHandler()
{
}

void b2RigidJointHandler::handle()
{
	reset();
	checkLimits();
	handleOverLoads();
	updateBodies();
}

void b2RigidJointHandler::reset()
{
	xfol = yfol = zmol = false;
	mbi = masterJoint->m_indexA;
}

void b2RigidJointHandler::handleOverLoads()
{
	if (zmol) {
		handleMomentOverLoad();
	}
	if (xfol || yfol) {
		handleForceOverLoad();
	}
}

void b2RigidJointHandler::updateBodies()
{
	b2Vec2 cA= data->positions[mbi].c;
	float32 aA = data->positions[mbi].a;
	b2Vec2 vA = data->velocities[mbi].v;
	float32 wA = data->velocities[mbi].w;
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
		float32 wB=wA;
		data->velocities[ib].v = vB;
		data->velocities[ib].w = wB;
	}
}

void b2RigidJointHandler::handleForceOverLoad()
{
}

/**
* get joint inertia of b-bodies
* and add 
*/
void b2RigidJointHandler::handleMomentOverLoad()
{
	float32 jm = 0.f;
	float32 ji = 0.f;
	for (int32 i = 0; i < ejCount; i++) {
		b2ElasticPlasticJoint* joint = ejStack[i];
		b2Body* bb = joint->m_bodyB;
		float32 m = bb->GetMass();
		jm += m;
		float32 mr2 = m * joint->m_rB.LengthSquared();
		ji += mr2;
	}
	mbi = masterJoint->m_indexB;
	b2Vec2 c = data->positions[mbi].c;
	float32 a = data->positions[mbi].a;
	b2Vec2 v = data->velocities[mbi].v;
	float32 w = data->velocities[mbi].w;
	float32 m = masterJoint->m_jim.z;
	float32 mm = masterJoint->m_maxImpulse.z;
	float32 em;
	if (m > 0) {
		em = m - mm;
	}
	else {
		em = m + mm;
	}
	float32 dw = em / ji;
	b2Vec2 dv = dw * masterJoint->m_rB;
	data->velocities[mbi].v += dv;
	data->velocities[mbi].w += dw;
}

void b2RigidJointHandler::checkLimits()
{
	if (b2Abs(masterJoint->m_jim.z) >= masterJoint->m_maxImpulse.z) {
		zmol = true;
	}
	if (b2Abs(masterJoint->m_jim.x) >= masterJoint->m_maxImpulse.x) {
		xfol = true;
	}
	if (b2Abs(masterJoint->m_jim.y) >= masterJoint->m_maxImpulse.y) {
		yfol = true;
	}
}
