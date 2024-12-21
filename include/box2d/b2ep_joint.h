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

typedef struct b2ElasticPlasticJoint
{
	float referenceAngle;
	float linearHertz;
	float linearDampingRatio;
	float angularHertz;
	float angularDampingRatio;

	b2Softness linearSoftness;
	b2Softness angularSoftness;
	b2Vec2 linearImpulse;
	float angularImpulse;

	int indexA;
	int indexB;
	b2Vec2 anchorA;
	b2Vec2 anchorB;
	b2Vec2 deltaCenter;
	float deltaAngle;
	float axialMass;
	b2Vec2 maxForce;
	float maxTorque;
	float maxElasticRotation;
	float currentStrain;
	float currentRotation;
	float angularError;
	float positionError;
	float maxStrain;
	float maxRotation;
	b2Vec2 rotatedMaxforce;
	int id;
} b2ElasticPlasticJoint;
