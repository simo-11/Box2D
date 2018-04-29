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
		float32 aB=aA;
		data->velocities[ib].v = vB;
		data->velocities[ib].w = wB;
		data->positions[ib].c = cB;
		data->positions[ib].a = aB;
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
		ji += bb->GetInertia() + m * joint->m_rB.LengthSquared();
	}
	mbi = masterJoint->m_indexB;
	b2Vec2 c = data->positions[mbi].c;
	float32 a = data->positions[mbi].a;
	b2Vec2 v = data->velocities[mbi].v;
	float32 w = data->velocities[mbi].w;
	float dw = masterJoint->m_jim.z / ji;
	b2Vec2 P(masterJoint->m_jim.x, masterJoint->m_jim.y);
	b2Vec2 dv = 1.f / jm * P + w * masterJoint->m_rB;
	float da = data->step.dt*w;
	b2Vec2 dc= data->step.dt*v;
	data->positions[mbi].c += dc;
	data->positions[mbi].a += da;
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
