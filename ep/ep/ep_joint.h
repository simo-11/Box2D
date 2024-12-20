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

* Simo Nikula 2018/02
* Collection of various helpers for 
* demos and debugging of elastic plastic joints
*/
#pragma once

#include <box2d/box2d.h>
#include "draw.h"
#include "box2d/b2ep_joint.h"
#include "test.h"
#include <stdlib.h>
struct Settings;
struct SelectedEPJoint;
class Test;
/**
Collects data from iterations
*/
class EpDebug{
public:
	float *xyData;
	float *vxA, *vxB, *vyA, *vyB, *vaA, *vaB;
	float *pxA, *pxB, *pyA, *pyB, *paA, *paB;
	float *cdotx, *cdoty, *cdotz;
	float *ix, *iy,*ia;
	bool showA, showB;
	int velocityIterations, positionIterations, vo, po, pi;
	int epDebugSteps,stepsStored;
	EpDebug();
	virtual ~EpDebug();
	/**
	virtual void EndInitVelocityConstraints
	(b2ElasticPlasticJoint* joint, const b2SolverData& data);
	virtual void BeginVelocityIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data);
	virtual void EndVelocityIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data);
	virtual void BeginPositionIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data);
	virtual void EndPositionIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data);
	*/
	static Settings* settings;
	static void Ui(Test *t, SelectedEPJoint* j);
	static void xyPlot(const char *label, float*v, int count);
	static bool IsWanted(b2Body *b);
};

struct EPBeam
{
	unsigned char label;
	bool deleteSbody = false;
	float position[2]; // for static body
	b2BodySim * body, *sBody;
	b2ElasticPlasticJoint* joint;
	EPBeam* next;
	EPBeam() {
		label = 1;
		next = nullptr;
		body = nullptr;
		joint = nullptr;
		sBody = nullptr;
		position[0] = 0;
		position[1] = 0;
	}
	void translate(b2Vec2 t);
};

#define EP_MAX_VALUES 10
struct SelectedEPJoint
{
	int id; // allows survival during restarts
	bool highlight;
	float *forces = NULL; // store at most 3*EP_MAX_VALUES values
	float *capacities = NULL; // store at most 5*EP_MAX_VALUES values
	b2Vec2 maxForce; // max (rotated) force
	float maxTorque;
	b2ElasticPlasticJoint * joint;
	bool dep; // show EpDebug window
	EpDebug* epd;
	SelectedEPJoint* next;
	SelectedEPJoint();
	~SelectedEPJoint();
	EpDebug* getEpDebug();
	void StartDebug(),StopDebug(),AttachEpDebug(),DetachEpDebug();
};

struct RigidTriangle
{
	unsigned char label;
	float position[2] = {};
	b2BodySim * body;
	RigidTriangle* next;
	RigidTriangle() {
		label = 1;
		next = nullptr;
		body = nullptr;
	}
};
