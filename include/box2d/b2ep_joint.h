// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
/*
* Simo Nikula modifications started 2017
*/
#pragma once
#include "joint.h"
#include <bitset>

enum SOLVE_ORDER {
	FORCE,
	MOMENT,
};

enum OVERLOAD_DIRECTION {
	X,
	Y,
	RZ,
	ANY,
};

/// Base on Weld joint definition. 
typedef struct b2ElasticPlasticJointDef 
{
	b2BodyId bodyIdA;
	b2BodyId bodyIdB;
	b2Vec2 localAnchorA;
	b2Vec2 localAnchorB;
	float referenceAngle;
	float linearHertz;
	/// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
	float angularHertz;
	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float linearDampingRatio;
	/// Linear damping ratio, non-dimensional. Use 1 for critical damping.
	float angularDampingRatio;
	/// Set this flag to true if the attached bodies should collide
	bool collideConnected;
	/// User data pointer
	void* userData;
	/// Used internally to detect a valid definition. DO NOT SET.
	int32_t internalValue;
	/// maximum joint forced in N.
	b2Vec2 maxForce;
	/// The maximum joint torque in N-m.
	float maxTorque;
	// or max elastic rotation
	float maxElasticRotation;
	// meters, typically less than about 10 % of joint distance
	float maxStrain;
	// radians, typically 1 - 6 
	float maxRotation;
} b2ElasticPlasticJointDef;
B2_API b2ElasticPlasticJointDef b2DefaultElasticPlasticJointDef( void );