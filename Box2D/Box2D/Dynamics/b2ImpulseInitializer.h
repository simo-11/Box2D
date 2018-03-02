/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
* Simo Nikula, 4/2017 for elastic plastic
*/

#ifndef B2_IMPULSE_INITIALIZER_H
#define B2_IMPULSE_INITIALIZER_H

#include "Box2D/Common/b2Math.h"
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2TimeStep.h"

class b2Contact;
class b2Joint;
class b2StackAllocator;
class b2ContactListener;
struct b2ContactVelocityConstraint;
struct b2Profile;

/// This is an internal class.
class b2ImpulseInitializer
{
public:
	b2ImpulseInitializer(){};
	~b2ImpulseInitializer(){};

	void InitImpulses();
	b2Vec3 addImpulses(b2ElasticPlasticJoint*);
	b2ElasticPlasticJoint* getNextJoint(b2ElasticPlasticJoint*);
	bool isNearEnough(b2ElasticPlasticJoint*);
	const b2Vec2* gravity;
	b2Island* island;
	b2SolverData* solverData;
	int32 nonDynamicBodyCount = 0, startJointCount = 0, epCount = 0;
	b2Body** ndbStack = NULL; // non dynamic bodies
	b2ElasticPlasticJoint** sjStack = NULL; // corresponding starting joints
	b2ElasticPlasticJoint* currentStartJoint;
	b2ElasticPlasticJoint** epStack = NULL; // all ep joints
};
#endif
