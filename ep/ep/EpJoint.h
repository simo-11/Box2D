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

#ifndef EP_JOINT_H
#define EP_JOINT_H

#include <Box2D/Box2D.h>
#include "DebugDraw.h"
#include "Test.h"
#include <stdlib.h>
struct Settings;
struct SelectedEPJoint;
class Test;
/**
Collects data from iterations
*/
class EpDebug : public epDebugListener {
public:
	float *vxA, *vxB, *vyA, *vyB, *vaA, *vaB;
	float *pxA, *pxB, *pyA, *pyB, *paA, *paB;
	float *ix, *iy,*ia;
	bool showA, showB;
	unsigned char velocityIterations, positionIterations,vo,po;
	EpDebug();
	virtual ~EpDebug();
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
	b2Body * body, *sBody;
	b2ElasticPlasticJoint* joint;
	EPBeam* next;
	EPBeam() {
		label = 1;
		next = nullptr;
		body = nullptr;
	}
};

#define EP_MAX_VALUES 10
struct SelectedEPJoint
{
	int32 id; // allows survival during restarts
	float *forces = NULL; // store at most 3*EP_MAX_VALUES values
	float *capacities = NULL; // store at most 5*EP_MAX_VALUES values
	b2Vec2 maxForce; // max (rotated) force
	float32 maxTorque;
	b2ElasticPlasticJoint * joint;
	bool dep; // show EpDebug window
	EpDebug* epd;
	SelectedEPJoint* next;
	SelectedEPJoint();
	~SelectedEPJoint();
	EpDebug* getEpDebug();
	void StartDebug();
	void StopDebug();
};

struct RigidTriangle
{
	unsigned char label;
	float position[2];
	b2Body * body;
	RigidTriangle* next;
	RigidTriangle() {
		label = 1;
		next = nullptr;
		body = nullptr;
	}
};
#endif
