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

void b2ImpulseInitializer::InitImpulses(){
	for (int32 i = 0; i < startJointCount; i++){
		currentStartJoint = sjStack[i];
		addImpulses(currentStartJoint);
	}
}

/**
Loop over all bodies that are connected to startJoint with 
b2ElasticPlasticJoints until some other startJoint is nearer 
than current one.
Currently start point is assumed to be connected to rigid body.
*/
b2Vec3 b2ImpulseInitializer::addImpulses(b2ElasticPlasticJoint* startJoint){
	float32 h = step->dt;
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
	startJoint->m_impulse -= p;
	b2ElasticPlasticJoint* nextJoint;
	while ((nextJoint = getNextJoint(startJoint)) != NULL){
		b2Vec3 np = addImpulses(nextJoint);
		b2Vec2 njf(np.x,np.y);
		b2Vec2 jd = nextJoint->GetAnchorA()-sp;
		np.z += b2Cross(jd, njf);
		startJoint->m_impulse += np;
	}
	return startJoint->m_impulse;
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