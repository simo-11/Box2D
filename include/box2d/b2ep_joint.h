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
* Simo Nikula modifications started 2017
*/

#ifndef B2_ELASTIC_PLASTIC_JOINT_H
#define B2_ELASTIC_PLASTIC_JOINT_H

#include "box2d/b2_joint.h"
#include "box2d/b2_math.h"
#include "box2d/b2_body.h"
#include "box2d/b2_time_step.h"
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

class EpDebugListener;
/// Base on Weld joint definition. 
struct b2ElasticPlasticJointDef : public b2JointDef
{
	b2ElasticPlasticJointDef()
	{
		type = e_elasticPlasticJoint;
		localAnchorA.Set(0.0f, 0.0f);
		localAnchorB.Set(0.0f, 0.0f);
		referenceAngle = 0.0f;
		dampingRatio = 0.0f;
		maxElasticRotation = 0.f;
		maxTorque = 0.f;
		frequencyHz = 0;
		maxForce = b2Vec2(0,0);
		maxRotation = 0;
		maxStrain = 0;
	}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	float referenceAngle;

	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float dampingRatio;

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

};

class b2ElasticPlasticJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const;
	b2Vec2 GetAnchorB() const;

	b2Vec2 GetReactionForce(float inv_dt) const;
	float GetReactionTorque(float inv_dt) const;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Get the reference angle.
	float GetReferenceAngle() const { return m_referenceAngle; }

	/// Set the maximum plastic force in N.
	void SetMaxForce(b2Vec2 force);

	/// Get the maximum plastic force in N.
	b2Vec2 GetMaxForce() const;

	/// Set the maximum plastic torque in N*m.
	void SetMaxTorque(float torque);
	// or set maximum elastic rotation
	void SetMaxElasticRotation(float val);
	float getCurrentStrain(){ return m_currentStrain; }
	float getCurrentRotation(){ return m_currentRotation; }
	float getAngularError() { return angularError; }
	float getPositionError() { return positionError; }
	float getMaxStrain(){ return m_maxStrain; }
	float getMaxRotation(){ return m_maxRotation; }
	b2Vec2 GetRotatedMaxForce();
	int32 GetIslandIndexForA() { return m_indexA; }
	int32 GetIslandIndexForB() { return m_indexB; }
	/// Get the maximum friction torque in N*m.
	float GetMaxTorque() const;
	bool WantsToBreak();
	/// Dump to b2Log
	void Dump();
	static void SetLinearSlop(float value);
	static float GetLinearSlop();
	static void SetAngularSlop(float value);
	static float GetAngularSlop();
	static void resetEpId();
	int32 GetId() { return id; }
	EpDebugListener* GetDebugListener() { return debugListener; }
	b2ImpulseInitializer* GetImpulseInitializer();
	void SetDebugListener(EpDebugListener* listener) { debugListener = listener; }
	virtual bool hasPositionIterations() { return true; }
	int velocityIteration,positionIteration;
	b2Vec3 Cdot;
	float m_bias;
	float m_gamma,m_dw0;
	bool initImpulseDone;
	SOLVE_ORDER *solveOrder;
	static SOLVE_ORDER* getMomentFirst();
	static SOLVE_ORDER* getForceFirst();
	b2Vec3 m_jim; // joined impulse
	std::bitset<3> overLoads,savedOverLoads;
	bool isOverLoaded(OVERLOAD_DIRECTION d=ANY);
	bool wasOverLoaded(OVERLOAD_DIRECTION d = ANY);
	void setOverLoaded(OVERLOAD_DIRECTION, bool value=true);
	int32 id;
protected:
	EpDebugListener* debugListener;
	b2ImpulseInitializer *impulseInitializer;
	friend class b2Joint;
	friend class b2ImpulseInitializer;
	friend class b2Island;
	friend class b2RigidJointHandler;

	b2ElasticPlasticJoint(const b2ElasticPlasticJointDef* def);
	virtual ~b2ElasticPlasticJoint();
	void InitVelocityConstraints(const b2SolverData& data);
	void UpdateAnchors(const b2SolverData& data);
	void UpdatePlasticity(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);
	b2Vec3 GetMaxImpulse(float dt);
	bool CanHandleInitialImpulse(const b2SolverData* data);

	float m_frequencyHz; 
	float m_dampingRatio;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	float m_referenceAngle;
	b2Vec3 m_impulse;
	// ep
	float m_k;
	b2Vec3 m_maxImpulse;
	b2Vec2 GetClampedDeltaImpulse(b2Vec2 Cdot, const b2SolverData& data);
	float GetClampedDeltaImpulse(float Cdot, const b2SolverData& data);
	b2Vec3 GetClampedMaxImpulse(b2Vec2 Cdot, const b2SolverData& data);
	float GetClampedMaxImpulse(float Cdot, const b2SolverData& data);
	b2Vec2 Clamp(b2Vec2 P, const b2SolverData& data);
	float Clamp(float M, const b2SolverData& data);
	void updateRotationalPlasticity(const b2SolverData& data, float elasticRotation);
	b2Vec2 m_maxForce;
	float m_maxTorque;
	float m_maxElasticRotation;
	float m_maxStrain, m_maxRotation, m_currentStrain, m_currentRotation;
	float positionError, angularError;
	bool m_forceExceeded, m_torqueExceeded;
	// Impulse initialization
	bool aInitialized, bInitialized;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	int32 m_mbi;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	b2Mat33 m_mass;
	bool jointOk;
};

class EpDebugListener
{
public:
	virtual ~EpDebugListener() {}

	virtual void EndInitVelocityConstraints
	(b2ElasticPlasticJoint* joint, const b2SolverData& data)
	{
		B2_NOT_USED(joint);
		B2_NOT_USED(data);
	}
	virtual void BeginVelocityIteration
		(b2ElasticPlasticJoint* joint, const b2SolverData& data) 
		{ 
			B2_NOT_USED(joint); 
			B2_NOT_USED(data);
		}
	virtual void EndVelocityIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data)
	{
		B2_NOT_USED(joint);
		B2_NOT_USED(data);
	}
	virtual void BeginPositionIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data)
	{
		B2_NOT_USED(joint);
		B2_NOT_USED(data);
	}
	virtual void EndPositionIteration
	(b2ElasticPlasticJoint* joint, const b2SolverData& data)
	{
		B2_NOT_USED(joint);
		B2_NOT_USED(data);
	}
};

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
	b2Vec2 GetReactionForce(float inv_dt) const;
	float GetReactionTorque(float inv_dt) const;
	void UpdatePlasticity(const b2SolverData & data);
	virtual bool hasPositionIterations() {
		return false;
	}
	void Dump();
protected:
	friend class b2Joint;
	friend class b2ImpulseInitializer;
	friend class b2Island;
	b2RigidPlasticJoint(const b2RigidPlasticJointDef* def);
	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);
	b2Mat22 m_linearMass;
	float m_angularMass;
	b2Vec2 m_linearImpulse;
	float m_angularImpulse;
};

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
	void checkImpulses(b2ElasticPlasticJoint*);
	b2Vec3 getContactImpulses(b2Body *b);
	b2ElasticPlasticJoint* getNextJoint(b2ElasticPlasticJoint*);
	bool isNearEnough(b2ElasticPlasticJoint*);
	const b2Vec2* gravity=NULL;
	b2Island* island = NULL;
	b2SolverData* solverData = NULL;
	int32 nonDynamicBodyCount = 0, startJointCount = 0, epCount = 0;
	b2Body** ndbStack = NULL; // non dynamic bodies
	b2ElasticPlasticJoint** sjStack = NULL; // corresponding starting joints
	b2ElasticPlasticJoint* currentStartJoint=NULL;
	b2ElasticPlasticJoint** epStack = NULL; // all ep joints
	int32 bodyCount=0, jointCount=0, contactCount=0;
	bool IsInitImpulsesNeeded(b2Island*);
};

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
