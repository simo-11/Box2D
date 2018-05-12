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
	mbi = masterJoint->m_mbi;
}

void b2RigidJointHandler::handleOverLoads()
{
	if (masterJoint->isOverLoaded(RZ)) {
		handleMomentOverLoad();
	}
	if (masterJoint->isOverLoaded(X)  || 
		masterJoint->isOverLoaded(Y)) {
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
		joint->m_impulse.z = 0.f;
		joint->setOverLoaded(RZ);
	}
	b2Vec2 v = data->velocities[mbi].v;
	float32 w = data->velocities[mbi].w;
	mbi = masterJoint->m_indexB;
	if (masterJoint->m_mbi != mbi) {
		masterJoint->m_mbi = mbi;
		data->velocities[mbi].w = w;
		data->velocities[mbi].v = v + b2Cross(w, masterJoint->m_rB);
	}
	float32 m = masterJoint->m_jim.z;
	float32 mm = masterJoint->m_maxImpulse.z;
	float32 em;
	if (m > mm) {
		em = m - mm;
	}
	else if(m<-mm) {
		em = m + mm;
	}
	else {
		em = m;
	}
	float32 dw = -em / ji;
	b2Vec2 dv = -b2Cross(dw, masterJoint->m_rB);
	data->velocities[mbi].v += dv;
	data->velocities[mbi].w += dw;
	masterJoint->m_impulse.z=0.f;
}

void b2RigidJointHandler::checkLimits()
{
	if (!masterJoint->isOverLoaded(RZ)) {
		if (b2Abs(masterJoint->m_jim.z) >= masterJoint->m_maxImpulse.z) {
			masterJoint->setOverLoaded(RZ, true);
		}
	}
	if (!masterJoint->isOverLoaded(X)) {
		if (b2Abs(masterJoint->m_jim.x) >= masterJoint->m_maxImpulse.x) {
			masterJoint->setOverLoaded(X, true);
		}
	}
	if (!masterJoint->isOverLoaded(Y)) {
		if (b2Abs(masterJoint->m_jim.y) >= masterJoint->m_maxImpulse.y) {
			masterJoint->setOverLoaded(Y, true);
		}
	}
}
