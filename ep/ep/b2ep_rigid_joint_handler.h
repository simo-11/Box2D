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
*/

/**
If joined impulse components in m_jim are not too high
sync other bodies (mB:s in ejStack) to match mA using rigid body statistics
and add required impulses.
If limits are exceeded during process restart is needed.
To avoid restarts mA should be picked well.
Currently it is done manually.

For simple moment limited cases (cantilever beam) nearest mB can be used as master body
if moment is exceeded.

For other scenarios work is needed.
*/

#ifndef B2_RIGID_JOINT_HANDLER_H
#define B2_RIGID_JOINT_HANDLER_H

#include "Box2D/Dynamics/Joints/b2Joint.h"
#include "Box2D/Dynamics/Joints/b2ElasticPlasticJoint.h"
#include "Box2D/Dynamics/b2ImpulseInitializer.h"

class b2RigidJointHandler
{
public:
	b2RigidJointHandler();
	b2ElasticPlasticJoint* masterJoint;
	int32 mbi; // master body index
	int32 ejCount;
	b2ElasticPlasticJoint** ejStack;
	b2SolverData* data;
	void handle(),reset(),handleLoads(),updateBodies();
	void handleMoment(), handleForce();
	void checkLimits();
};

#endif
