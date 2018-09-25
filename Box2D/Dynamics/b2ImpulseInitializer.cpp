/*
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

#include "Box2D/Collision/b2Distance.h"
#include "Box2D/Dynamics/b2Island.h"
#include "Box2D/Dynamics/b2ImpulseInitializer.h"
#include "Box2D/Dynamics/Joints/b2ElasticPlasticJoint.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/b2World.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"
#include "Box2D/Dynamics/Contacts/b2ContactSolver.h"
#include "Box2D/Dynamics/Joints/b2Joint.h"
#include "Box2D/Common/b2StackAllocator.h"
#include "Box2D/Common/b2Timer.h"

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
	float32 h = solverData->step.dt;
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
	float32 sm = b->GetMass()*b->m_gravityScale;
	b2Vec2 f = sm* (*gravity) + b->m_force;
	float32 m = b->m_torque - b2Cross(d, f);
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
		b2MassData massData;
		bO->GetMassData(&massData);
		float32 mO = bO->m_mass;
		if (mO == 0.f) {
			continue;
		}
		float32 iO = bO->m_I;
		int32  bi = bO->m_islandIndex;
		// float32 aO = solverData->positions[bi].a;
		b2Vec2 pO = solverData->positions[bi].c;
		b2Vec2 vO = solverData->velocities[bi].v;
		float32 wO = solverData->velocities[bi].w;
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
	float32 d = (currentStartJoint->GetAnchorA()-jp).LengthSquared();
	for (int32 i = 0; i < startJointCount; i++){
		b2ElasticPlasticJoint* startJoint = sjStack[i];
		if (startJoint == joint){
			continue;
		}
		float32 dc = (startJoint->GetAnchorA() - jp).LengthSquared();
		if (dc < d){
			return false;
		}
	}
	return true;
}