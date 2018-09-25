#include "b2RigidJointHandler.h"

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
	// float32 aA = data->positions[mbi].a;
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
	/* velocities from beginng of step are used */
	b2Body* b = masterJoint->m_bodyB;
	b2Vec2 v = b->GetLinearVelocity();
	float32 w = b->GetAngularVelocity();
	if (!masterJoint->isOverLoaded(RZ)) {
		int32 mba = masterJoint->m_indexA;
		mbi = mba;
		int32 mbb = masterJoint->m_indexB;
		data->velocities[mbb].v = v;
		data->velocities[mbb].w = w ;
		return;
	}
	float32 jm = 0.f;
	float32 ji = 0.f;
	for (int32 i = 0; i < ejCount; i++) {
		b2ElasticPlasticJoint* joint = ejStack[i];
		b2Body* bb = joint->m_bodyB;
		float32 m = bb->GetMass();
		jm += m;
		float32 mr2 = m * joint->m_rB.LengthSquared();
		ji += mr2;
		joint->setOverLoaded(RZ);
	}
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
