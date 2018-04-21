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
*/

#include "Box2D/Collision/b2Distance.h"
#include "Box2D/Dynamics/b2Island.h"
#include "Box2D/Dynamics/b2ImpulseInitializer.h" // ep
#include "Box2D/Dynamics/Joints/b2ElasticPlasticJoint.h" // ep
#include "Box2D/Dynamics/Joints/b2RigidJointHandler.h" // ep
#include "Box2D/Dynamics/b2Body.h"
#include "Box2D/Dynamics/b2Fixture.h"
#include "Box2D/Dynamics/b2World.h"
#include "Box2D/Dynamics/Contacts/b2Contact.h"
#include "Box2D/Dynamics/Contacts/b2ContactSolver.h"
#include "Box2D/Dynamics/Joints/b2Joint.h"
#include "Box2D/Common/b2StackAllocator.h"
#include "Box2D/Common/b2Timer.h"

/*
Position Correction Notes
=========================
I tried the several algorithms for position correction of the 2D revolute joint.
I looked at these systems:
- simple pendulum (1m diameter sphere on massless 5m stick) with initial angular velocity of 100 rad/s.
- suspension bridge with 30 1m long planks of length 1m.
- multi-link chain with 30 1m long links.

Here are the algorithms:

Baumgarte - A fraction of the position error is added to the velocity error. There is no
separate position solver.

Pseudo Velocities - After the velocity solver and position integration,
the position error, Jacobian, and effective mass are recomputed. Then
the velocity constraints are solved with pseudo velocities and a fraction
of the position error is added to the pseudo velocity error. The pseudo
velocities are initialized to zero and there is no warm-starting. After
the position solver, the pseudo velocities are added to the positions.
This is also called the First Order World method or the Position LCP method.

Modified Nonlinear Gauss-Seidel (NGS) - Like Pseudo Velocities except the
position error is re-computed for each constraint and the positions are updated
after the constraint is solved. The radius vectors (aka Jacobians) are
re-computed too (otherwise the algorithm has horrible instability). The pseudo
velocity states are not needed because they are effectively zero at the beginning
of each iteration. Since we have the current position error, we allow the
iterations to terminate early if the error becomes smaller than b2_linearSlop.

Full NGS or just NGS - Like Modified NGS except the effective mass are re-computed
each time a constraint is solved.

Here are the results:
Baumgarte - this is the cheapest algorithm but it has some stability problems,
especially with the bridge. The chain links separate easily close to the root
and they jitter as they struggle to pull together. This is one of the most common
methods in the field. The big drawback is that the position correction artificially
affects the momentum, thus leading to instabilities and false bounce. I used a
bias factor of 0.2. A larger bias factor makes the bridge less stable, a smaller
factor makes joints and contacts more spongy.

Pseudo Velocities - the is more stable than the Baumgarte method. The bridge is
stable. However, joints still separate with large angular velocities. Drag the
simple pendulum in a circle quickly and the joint will separate. The chain separates
easily and does not recover. I used a bias factor of 0.2. A larger value lead to
the bridge collapsing when a heavy cube drops on it.

Modified NGS - this algorithm is better in some ways than Baumgarte and Pseudo
Velocities, but in other ways it is worse. The bridge and chain are much more
stable, but the simple pendulum goes unstable at high angular velocities.

Full NGS - stable in all tests. The joints display good stiffness. The bridge
still sags, but this is better than infinite forces.

Recommendations
Pseudo Velocities are not really worthwhile because the bridge and chain cannot
recover from joint separation. In other cases the benefit over Baumgarte is small.

Modified NGS is not a robust method for the revolute joint due to the violent
instability seen in the simple pendulum. Perhaps it is viable with other constraint
types, especially scalar constraints where the effective mass is a scalar.

This leaves Baumgarte and Full NGS. Baumgarte has small, but manageable instabilities
and is very fast. I don't think we can escape Baumgarte, especially in highly
demanding cases where high constraint fidelity is not needed.

Full NGS is robust and easy on the eyes. I recommend this as an option for
higher fidelity simulation and certainly for suspension bridges and long chains.
Full NGS might be a good choice for ragdolls, especially motorized ragdolls where
joint separation can be problematic. The number of NGS iterations can be reduced
for better performance without harming robustness much.

Each joint in a can be handled differently in the position solver. So I recommend
a system where the user can select the algorithm on a per joint basis. I would
probably default to the slower Full NGS and let the user select the faster
Baumgarte method in performance critical scenarios.
*/

/*
Cache Performance

The Box2D solvers are dominated by cache misses. Data structures are designed
to increase the number of cache hits. Much of misses are due to random access
to body data. The constraint structures are iterated over linearly, which leads
to few cache misses.

The bodies are not accessed during iteration. Instead read only data, such as
the mass values are stored with the constraints. The mutable data are the constraint
impulses and the bodies velocities/positions. The impulses are held inside the
constraint structures. The body velocities/positions are held in compact, temporary
arrays to increase the number of cache hits. Linear and angular velocity are
stored in a single array since multiple arrays lead to multiple misses.
*/

/*
2D Rotation

R = [cos(theta) -sin(theta)]
    [sin(theta) cos(theta) ]

thetaDot = omega

Let q1 = cos(theta), q2 = sin(theta).
R = [q1 -q2]
    [q2  q1]

q1Dot = -thetaDot * q2
q2Dot = thetaDot * q1

q1_new = q1_old - dt * w * q2
q2_new = q2_old + dt * w * q1
then normalize.

This might be faster than computing sin+cos.
However, we can compute sin+cos of the same angle fast.
*/

b2Island::b2Island(
	int32 bodyCapacity,
	int32 contactCapacity,
	int32 jointCapacity,
	b2StackAllocator* allocator,
	b2ContactListener* listener)
{
	m_bodyCapacity = bodyCapacity;
	m_contactCapacity = contactCapacity;
	m_jointCapacity	 = jointCapacity;
	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;

	m_allocator = allocator;
	m_listener = listener;

	m_bodies = (b2Body**)m_allocator->Allocate(bodyCapacity * sizeof(b2Body*));
	m_contacts = (b2Contact**)m_allocator->Allocate(contactCapacity	 * sizeof(b2Contact*));
	m_joints = (b2Joint**)m_allocator->Allocate(jointCapacity * sizeof(b2Joint*));

	m_velocities = (b2Velocity*)m_allocator->Allocate(m_bodyCapacity * sizeof(b2Velocity));
	m_positions = (b2Position*)m_allocator->Allocate(m_bodyCapacity * sizeof(b2Position));
	ndbStack=NULL;
	sjStack=NULL;
	epStack=NULL;
}

b2Island::~b2Island()
{
	// Warning: the order should reverse the constructor order.
	m_allocator->Free(m_positions);
	m_allocator->Free(m_velocities);
	m_allocator->Free(m_joints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_bodies);
}

// ep
namespace {
	bool doInitImpulses=false;
}
bool b2Island::IsInitImpulses(){
	return doInitImpulses;
}
void b2Island::SetInitImpulses(bool value){
	doInitImpulses=value;
}
// end ep

void b2Island::Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, bool allowSleep)
{
	b2Timer timer;

	float32 h = step.dt;

	// Integrate velocities and apply damping. Initialize the body state.
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* b = m_bodies[i];

		b2Vec2 c = b->m_sweep.c;
		float32 a = b->m_sweep.a;
		b2Vec2 v = b->m_linearVelocity;
		float32 w = b->m_angularVelocity;

		// Store positions for continuous collision.
		b->m_sweep.c0 = b->m_sweep.c;
		b->m_sweep.a0 = b->m_sweep.a;

		if (b->m_type == b2_dynamicBody)
		{
			// Integrate velocities.
			v += h * (b->m_gravityScale * gravity + b->m_invMass * b->m_force);
			w += h * b->m_invI * b->m_torque;

			// Apply damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Pade approximation:
			// v2 = v1 * 1 / (1 + c * dt)
			v *= 1.0f / (1.0f + h * b->m_linearDamping);
			w *= 1.0f / (1.0f + h * b->m_angularDamping);
		}

		m_positions[i].c = c;
		m_positions[i].a = a;
		m_velocities[i].v = v;
		m_velocities[i].w = w;
	}

	timer.Reset();

	// Solver data
	b2SolverData solverData;
	solverData.step = step;
	solverData.positions = m_positions;
	solverData.velocities = m_velocities;

	// Initialize velocity constraints.
	b2ContactSolverDef contactSolverDef;
	contactSolverDef.step = step;
	contactSolverDef.contacts = m_contacts;
	contactSolverDef.count = m_contactCount;
	contactSolverDef.positions = m_positions;
	contactSolverDef.velocities = m_velocities;
	contactSolverDef.allocator = m_allocator;

	b2ContactSolver contactSolver(&contactSolverDef);
	contactSolver.InitializeVelocityConstraints();

	if (step.warmStarting)
	{
		contactSolver.WarmStart();
	}
	// ep start
	nonDynamicBodyCount=startJointCount=epCount=0;
	InitEpStacks();

	if (doInitImpulses) {
		timer.Reset();
		InitRigidPlasticJoints(solverData, gravity);
		profile->initImpulse = timer.GetMilliseconds();
	}
	else {
		profile->initImpulse = 0;
	}
	if (m_jointCount > 0) {
		epLogActive = true;
	}
	else {
		epLogActive = false;
	}
	// ep end

	for (int32 i = 0; i < m_jointCount; ++i)
	{
		m_joints[i]->InitVelocityConstraints(solverData);
	}

	profile->solveInit = timer.GetMilliseconds();
	// Solve velocity constraints
	timer.Reset();
	for (int32 i = 0; i < step.velocityIterations; ++i)
	{
#ifdef EP_LOG
			epLog("velocityIteration: %d\n", i);
#endif
		for (int32 j = 0; j < m_jointCount; ++j)
		{
			m_joints[j]->SolveVelocityConstraints(solverData);
		}
		contactSolver.SolveVelocityConstraints();
		UpdateRigidPlasticJoints(solverData);
	}

	// Store impulses for warm starting
	contactSolver.StoreImpulses();
	profile->solveVelocity = timer.GetMilliseconds();

	// Integrate positions
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Vec2 c = m_positions[i].c;
		float32 a = m_positions[i].a;
		b2Vec2 v = m_velocities[i].v;
		float32 w = m_velocities[i].w;

		// Check for large velocities
		b2Vec2 translation = h * v;
		if (b2Dot(translation, translation) > b2_maxTranslationSquared)
		{
			float32 ratio = b2_maxTranslation / translation.Length();
			v *= ratio;
		}

		float32 rotation = h * w;
		if (rotation * rotation > b2_maxRotationSquared)
		{
			float32 ratio = b2_maxRotation / b2Abs(rotation);
			w *= ratio;
		}

		// Integrate
		c += h * v;
		a += h * w;

		m_positions[i].c = c;
		m_positions[i].a = a;
		m_velocities[i].v = v;
		m_velocities[i].w = w;
	}
	// ep start
	for (int32 i = 0; i < m_jointCount; ++i)
	{
		switch (m_joints[i]->GetType()) {
		case e_elasticPlasticJoint:
		case e_rigidPlasticJoint:
			((b2ElasticPlasticJoint*)(m_joints[i]))->UpdatePlasticity(solverData);
			break;
		}
	}
	// ep end

	// Solve position constraints
	timer.Reset();
	bool positionSolved = false;
	for (int32 i = 0; i < step.positionIterations; ++i)
	{
#ifdef EP_LOG
		if (m_jointCount > 0) {
			epLog("positionIteration: %d\n", i);
		}
#endif
		bool contactsOkay = contactSolver.SolvePositionConstraints();

		bool jointsOkay = true;
		for (int32 j = 0; j < m_jointCount; ++j)
		{
			bool jointOkay = m_joints[j]->SolvePositionConstraints(solverData);
			jointsOkay = jointsOkay && jointOkay;
		}

		if (contactsOkay && jointsOkay)
		{
			// Exit early if the position errors are small.
			positionSolved = true;
			break;
		}
	}

	// Copy state buffers back to the bodies
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* body = m_bodies[i];
		body->m_sweep.c = m_positions[i].c;
		body->m_sweep.a = m_positions[i].a;
		body->m_linearVelocity = m_velocities[i].v;
		body->m_angularVelocity = m_velocities[i].w;
		body->SynchronizeTransform();
	}

	profile->solvePosition = timer.GetMilliseconds();

	Report(contactSolver.m_velocityConstraints);

	if (allowSleep)
	{
		float32 minSleepTime = b2_maxFloat;

		const float32 linTolSqr = b2_linearSleepTolerance * b2_linearSleepTolerance;
		const float32 angTolSqr = b2_angularSleepTolerance * b2_angularSleepTolerance;

		for (int32 i = 0; i < m_bodyCount; ++i)
		{
			b2Body* b = m_bodies[i];
			if (b->GetType() == b2_staticBody)
			{
				continue;
			}

			if ((b->m_flags & b2Body::e_autoSleepFlag) == 0 ||
				b->m_angularVelocity * b->m_angularVelocity > angTolSqr ||
				b2Dot(b->m_linearVelocity, b->m_linearVelocity) > linTolSqr)
			{
				b->m_sleepTime = 0.0f;
				minSleepTime = 0.0f;
			}
			else
			{
				b->m_sleepTime += h;
				minSleepTime = b2Min(minSleepTime, b->m_sleepTime);
			}
		}

		if (minSleepTime >= b2_timeToSleep && positionSolved)
		{
			for (int32 i = 0; i < m_bodyCount; ++i)
			{
				b2Body* b = m_bodies[i];
				b->SetAwake(false);
			}
		}
	}
	if (sjStack != NULL) {
		m_allocator->Free(sjStack);
		sjStack = NULL;
	}
	if (ndbStack != NULL) {
		m_allocator->Free(ndbStack);
		ndbStack = NULL;
	}
	if (epStack != NULL) {
		m_allocator->Free(epStack);
		epStack = NULL;
	}

	epLogActive = true;
}

void b2Island::SolveTOI(const b2TimeStep& subStep, int32 toiIndexA, int32 toiIndexB)
{
	b2Assert(toiIndexA < m_bodyCount);
	b2Assert(toiIndexB < m_bodyCount);

	// Initialize the body state.
	for (int32 i = 0; i < m_bodyCount; ++i)
	{
		b2Body* b = m_bodies[i];
		m_positions[i].c = b->m_sweep.c;
		m_positions[i].a = b->m_sweep.a;
		m_velocities[i].v = b->m_linearVelocity;
		m_velocities[i].w = b->m_angularVelocity;
	}

	b2ContactSolverDef contactSolverDef;
	contactSolverDef.contacts = m_contacts;
	contactSolverDef.count = m_contactCount;
	contactSolverDef.allocator = m_allocator;
	contactSolverDef.step = subStep;
	contactSolverDef.positions = m_positions;
	contactSolverDef.velocities = m_velocities;
	b2ContactSolver contactSolver(&contactSolverDef);

	// Solve position constraints.
	for (int32 i = 0; i < subStep.positionIterations; ++i)
	{
		bool contactsOkay = contactSolver.SolveTOIPositionConstraints(toiIndexA, toiIndexB);
		if (contactsOkay)
		{
			break;
		}
	}

#if 0
	// Is the new position really safe?
	for (int32 i = 0; i < m_contactCount; ++i)
	{
		b2Contact* c = m_contacts[i];
		b2Fixture* fA = c->GetFixtureA();
		b2Fixture* fB = c->GetFixtureB();

		b2Body* bA = fA->GetBody();
		b2Body* bB = fB->GetBody();

		int32 indexA = c->GetChildIndexA();
		int32 indexB = c->GetChildIndexB();

		b2DistanceInput input;
		input.proxyA.Set(fA->GetShape(), indexA);
		input.proxyB.Set(fB->GetShape(), indexB);
		input.transformA = bA->GetTransform();
		input.transformB = bB->GetTransform();
		input.useRadii = false;

		b2DistanceOutput output;
		b2SimplexCache cache;
		cache.count = 0;
		b2Distance(&output, &cache, &input);

		if (output.distance == 0 || cache.count == 3)
		{
			cache.count += 0;
		}
	}
#endif

	// Leap of faith to new safe state.
	m_bodies[toiIndexA]->m_sweep.c0 = m_positions[toiIndexA].c;
	m_bodies[toiIndexA]->m_sweep.a0 = m_positions[toiIndexA].a;
	m_bodies[toiIndexB]->m_sweep.c0 = m_positions[toiIndexB].c;
	m_bodies[toiIndexB]->m_sweep.a0 = m_positions[toiIndexB].a;

	// No warm starting is needed for TOI events because warm
	// starting impulses were applied in the discrete solver.
	contactSolver.InitializeVelocityConstraints();

// Solve velocity constraints.
for (int32 i = 0; i < subStep.velocityIterations; ++i)
{
	contactSolver.SolveVelocityConstraints();
}

// Don't store the TOI contact forces for warm starting
// because they can be quite large.

float32 h = subStep.dt;

// Integrate positions
for (int32 i = 0; i < m_bodyCount; ++i)
{
	b2Vec2 c = m_positions[i].c;
	float32 a = m_positions[i].a;
	b2Vec2 v = m_velocities[i].v;
	float32 w = m_velocities[i].w;

	// Check for large velocities
	b2Vec2 translation = h * v;
	if (b2Dot(translation, translation) > b2_maxTranslationSquared)
	{
		float32 ratio = b2_maxTranslation / translation.Length();
		v *= ratio;
	}

	float32 rotation = h * w;
	if (rotation * rotation > b2_maxRotationSquared)
	{
		float32 ratio = b2_maxRotation / b2Abs(rotation);
		w *= ratio;
	}

	// Integrate
	c += h * v;
	a += h * w;

	m_positions[i].c = c;
	m_positions[i].a = a;
	m_velocities[i].v = v;
	m_velocities[i].w = w;

	// Sync bodies
	b2Body* body = m_bodies[i];
	body->m_sweep.c = c;
	body->m_sweep.a = a;
	body->m_linearVelocity = v;
	body->m_angularVelocity = w;
	body->SynchronizeTransform();
}

Report(contactSolver.m_velocityConstraints);
}

void b2Island::Report(const b2ContactVelocityConstraint* constraints)
{
	if (m_listener == nullptr)
	{
		return;
	}

	for (int32 i = 0; i < m_contactCount; ++i)
	{
		b2Contact* c = m_contacts[i];

		const b2ContactVelocityConstraint* vc = constraints + i;

		b2ContactImpulse impulse;
		impulse.count = vc->pointCount;
		for (int32 j = 0; j < vc->pointCount; ++j)
		{
			impulse.normalImpulses[j] = vc->points[j].normalImpulse;
			impulse.tangentImpulses[j] = vc->points[j].tangentImpulse;
		}

		m_listener->PostSolve(c, &impulse);
	}
}

void b2Island::InitEpStacks() {
	for (int32 i = 0; i < m_jointCount; i++) {
		b2Joint  *joint = m_joints[i];
		switch (joint->GetType()) {
		case e_elasticPlasticJoint:
		case e_rigidPlasticJoint:
			break;
		default:
			continue;
		}
		b2ElasticPlasticJoint * epJoint = (b2ElasticPlasticJoint*)joint;
		epJoint->initImpulseDone = false;
		if (epJoint->m_frequencyHz>0.f) { // skip elastic ones
			continue;
		}
		if (epStack == NULL) {
			epStack = (b2ElasticPlasticJoint**)m_allocator->Allocate
			(m_jointCount * sizeof(b2ElasticPlasticJoint*));
		}
		epStack[epCount++] = epJoint;
		bool ndb = false;
		for (int bi = 0; bi<2; bi++) {
			b2Body *body = (bi == 0) ? joint->GetBodyA() : joint->GetBodyB();
			if (body->GetType() != b2_dynamicBody) {
				if (ndbStack == NULL) {
					ndbStack = (b2Body**)m_allocator->Allocate
					(m_bodyCount * sizeof(b2Body*));
				}
				ndbStack[nonDynamicBodyCount++] = body;
				ndb = true;
			}
		}
		if (ndb) {
			if (sjStack == NULL) {
				sjStack = (b2ElasticPlasticJoint**)m_allocator->Allocate
				(m_jointCount * sizeof(b2ElasticPlasticJoint*));
			}
			sjStack[startJointCount++] = epJoint;
		}
	}
}
/**
Scan through joints and init impulses 
and select master bodies
for rigidPlastic joints

* if they are connected to non kinematic body (first phase)
* restricted by contacts or other joints (future work)
* matrix force method could also be applied (future work)
*/
void b2Island::InitRigidPlasticJoints(b2SolverData& solverData, const b2Vec2& gravity)
{
	if (epCount == 0) {
		return;
	}
	b2ImpulseInitializer *ii=(epStack[0])->GetImpulseInitializer();
	// if there is no change in contacts or bodies rely on warmStarting
	if (ii->IsInitImpulsesNeeded(this)) {
		if (startJointCount > 0) {
			ii->bodyCount = m_bodyCount;
			ii->contactCount = m_contactCount;
			ii->jointCount = m_jointCount;
			ii->epCount = epCount;
			ii->epStack = epStack;
			ii->startJointCount = startJointCount;
			ii->sjStack = sjStack;
			ii->nonDynamicBodyCount = nonDynamicBodyCount;
			ii->ndbStack = ndbStack;
			ii->solverData = &solverData;
			ii->gravity = &gravity;
			ii->island = this;
			ii->InitImpulses();
		}
	}
}

/**
*/
void b2Island::UpdateRigidPlasticJoints(b2SolverData& solverData)
{
	if (epCount == 0) {
		return;
	}
	b2RigidJointHandler rjh;
	for (int32 i = 0; i < epCount; i++) {
		b2ElasticPlasticJoint  *joint = epStack[i];
		switch (joint->GetType()) {
		case e_elasticPlasticJoint:
			continue;
		case e_rigidPlasticJoint:
			break;
		default:
			continue;
		}
		b2ElasticPlasticJoint** ejStack = (b2ElasticPlasticJoint**)m_allocator->Allocate
		(epCount* sizeof(b2ElasticPlasticJoint*));
		int32 ejCount = 0;
		b2Vec3 jim;
		jim.SetZero();
		for (int32 j = i; j < epCount; j++) {
			b2ElasticPlasticJoint  *jj = epStack[j];
			if (joint->m_bodyA != jj->m_bodyA) {
				continue;
			}
			switch (joint->GetType()) {
			case e_elasticPlasticJoint:
				continue;
			case e_rigidPlasticJoint:
				break;
			default:
				continue;
			}
			ejStack[ejCount++]=jj;
			jim += jj->m_impulse;
		}
		joint->m_jim = jim;
		rjh.ejCount = ejCount;
		rjh.ejStack = ejStack;
		rjh.masterJoint = joint;
		rjh.data = &solverData;
		rjh.handle();
		m_allocator->Free(ejStack);
	}
}