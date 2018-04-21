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
	masterBody = masterJoint->m_bodyA;
}

void b2RigidJointHandler::handleOverLoads()
{
}

void b2RigidJointHandler::updateBodies()
{
	int32 ia = masterJoint->m_indexA;
	b2Vec2 cA= data->positions[ia].c;
	float32 aA = data->positions[ia].a;
	b2Vec2 vA = data->velocities[ia].v;
	float32 wA = data->velocities[ia].w;
	b2Rot r;
	r.SetIdentity();
	for (int32 i = 0; i < ejCount; i++) {
		b2ElasticPlasticJoint* joint = ejStack[i];
		int ib = joint->m_indexB;
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

void b2RigidJointHandler::checkLimits()
{
	if (b2Abs(masterJoint->m_jim.z) > masterJoint->m_maxImpulse.z) {
		zmol = true;
	}
	if (b2Abs(masterJoint->m_jim.x) > masterJoint->m_maxImpulse.x) {
		xfol = true;
	}
	if (b2Abs(masterJoint->m_jim.y) > masterJoint->m_maxImpulse.y) {
		yfol = true;
	}
}
