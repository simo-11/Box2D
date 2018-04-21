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
* Simo Nikula modifications started 2017 for elastic plastic
* RigidPlastic 2018
*/

#ifndef B2_RIGID_PLASTIC_JOINT_H
#define B2_RIGID_PLASTIC_JOINT_H

#include "Box2D/Dynamics/Joints/b2Joint.h"
#include "Box2D/Dynamics/Joints/b2ElasticPlasticJoint.h"
#include "Box2D/Dynamics/b2ImpulseInitializer.h"

struct b2RigidPlasticJointDef : public b2ElasticPlasticJointDef
{
	b2RigidPlasticJointDef()
	{
		type = e_rigidPlasticJoint;
		localAnchorA.Set(0.0f, 0.0f);
		localAnchorB.Set(0.0f, 0.0f);
		referenceAngle = 0.0f;
		dampingRatio = 0.0f;
		maxElasticRotation = 0.f;
		maxTorque = 0.f;
	}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);
};

class b2RigidPlasticJoint  : public b2ElasticPlasticJoint
{
public:
	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;
	void Dump();
protected:
	friend class b2Joint;
	friend class b2ImpulseInitializer;
	friend class b2Island;
	b2RigidPlasticJoint(const b2RigidPlasticJointDef* def);
	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);
};


#endif
