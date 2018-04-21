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

#include "Box2D/Dynamics/Joints/b2Joint.h"
#include "Box2D/Dynamics/b2ImpulseInitializer.h"

enum SOLVE_ORDER {
	FORCE,
	MOMENT,
};

class epDebugListener;
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
	}

	/// Initialize the bodies and offsets using the current transforms.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const b2Vec2& anchor);

	/// The local anchor point relative to bodyA's origin.
	b2Vec2 localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	b2Vec2 localAnchorB;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	float32 referenceAngle;

	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	float32 frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float32 dampingRatio;

	/// maximum joint forced in N.
	b2Vec2 maxForce;

	/// The maximum joint torque in N-m.
	float32 maxTorque;
	// or max elastic rotation
	float32 maxElasticRotation;
	// meters, typically less than about 10 % of joint distance
	float32 maxStrain;
	// radians, typically 1 - 6 
	float32 maxRotation;

};

class b2ElasticPlasticJoint : public b2Joint
{
public:
	b2Vec2 GetAnchorA() const;
	b2Vec2 GetAnchorB() const;

	b2Vec2 GetReactionForce(float32 inv_dt) const;
	float32 GetReactionTorque(float32 inv_dt) const;

	/// The local anchor point relative to bodyA's origin.
	const b2Vec2& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const b2Vec2& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Get the reference angle.
	float32 GetReferenceAngle() const { return m_referenceAngle; }

	/// Set the maximum plastic force in N.
	void SetMaxForce(b2Vec2 force);

	/// Get the maximum plastic force in N.
	b2Vec2 GetMaxForce() const;

	/// Set the maximum plastic torque in N*m.
	void SetMaxTorque(float32 torque);
	// or set maximum elastic rotation
	void SetMaxElasticRotation(float32 val);
	float32 getCurrentStrain(){ return m_currentStrain; }
	float32 getCurrentRotation(){ return m_currentRotation; }
	float32 getAngularError() { return angularError; }
	float32 getPositionError() { return positionError; }
	float32 getMaxStrain(){ return m_maxStrain; }
	float32 getMaxRotation(){ return m_maxRotation; }
	b2Vec2 GetRotatedMaxForce();
	int32 GetIslandIndexForA() { return m_indexA; }
	int32 GetIslandIndexForB() { return m_indexB; }
	/// Get the maximum friction torque in N*m.
	float32 GetMaxTorque() const;
	bool WantsToBreak();
	/// Dump to b2Log
	void Dump();
	static void SetLinearSlop(float32 value);
	static float32 GetLinearSlop();
	static void SetAngularSlop(float32 value);
	static float32 GetAngularSlop();
	static void resetEpId();
	int32 GetId() { return id; }
	epDebugListener* GetDebugListener() { return debugListener; }
	b2ImpulseInitializer* GetImpulseInitializer();
	void SetDebugListener(epDebugListener* listener) { debugListener = listener; }
	int velocityIteration,positionIteration;
	b2Vec3 Cdot;
	float32 m_bias;
	float32 m_gamma;
	bool initImpulseDone;
	SOLVE_ORDER *solveOrder;
	static SOLVE_ORDER* getMomentFirst();
	static SOLVE_ORDER* getForceFirst();
	b2Vec3 m_jim; // joined impulse
protected:
	epDebugListener* debugListener;
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
	b2Vec3 GetMaxImpulse(float32 dt);
	bool CanHandleInitialImpulse(const b2SolverData* data);

	float32 m_frequencyHz; 
	float32 m_dampingRatio;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	float32 m_referenceAngle;
	b2Vec3 m_impulse;
	// ep
	float32 m_k;
	b2Vec3 m_maxImpulse;
	b2Vec2 GetClampedDeltaImpulse(b2Vec2 Cdot, const b2SolverData& data);
	float32 GetClampedDeltaImpulse(float32 Cdot, const b2SolverData& data);
	b2Vec3 GetClampedMaxImpulse(b2Vec2 Cdot, const b2SolverData& data);
	float32 GetClampedMaxImpulse(float32 Cdot, const b2SolverData& data);
	b2Vec2 Clamp(b2Vec2 P, const b2SolverData& data);
	float32 Clamp(float32 M, const b2SolverData& data);
	void updateRotationalPlasticity(const b2SolverData& data, float32 elasticRotation);
	b2Vec2 m_maxForce;
	float32 m_maxTorque;
	float32 m_maxElasticRotation;
	float32 m_maxStrain, m_maxRotation, m_currentStrain, m_currentRotation;
	float32 positionError, angularError;
	bool m_forceExceeded, m_torqueExceeded;
	// Impulse initialization
	bool aInitialized, bInitialized;
	int32 id;

	// Solver temp
	int32 m_indexA;
	int32 m_indexB;
	b2Vec2 m_rA;
	b2Vec2 m_rB;
	b2Vec2 m_localCenterA;
	b2Vec2 m_localCenterB;
	float32 m_invMassA;
	float32 m_invMassB;
	float32 m_invIA;
	float32 m_invIB;
	b2Mat33 m_mass;
	bool jointOk;
};

class epDebugListener
{
public:
	virtual ~epDebugListener() {}

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

#endif
