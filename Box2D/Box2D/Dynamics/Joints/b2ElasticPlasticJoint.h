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

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct b2ElasticPlasticJointDef : public b2JointDef
{
	b2ElasticPlasticJointDef()
	{
		type = e_elasticPlasticJoint;
		localAnchorA.Set(0.0f, 0.0f);
		localAnchorB.Set(0.0f, 0.0f);
		referenceAngle = 0.0f;
		dampingRatio = 0.0f;
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

};

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
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

	/// Set the maximum friction force in N.
	void SetMaxForce(b2Vec2 force);

	/// Get the maximum friction force in N.
	b2Vec2 GetMaxForce() const;

	/// Set the maximum friction torque in N*m.
	void SetMaxTorque(float32 torque);

	/// Get the maximum friction torque in N*m.
	float32 GetMaxTorque() const;

	/// Dump to b2Log
	void Dump();
	static void SetLinearSlop(float32 value);
	static float32 GetLinearSlop();
	static void SetAngularSlop(float32 value);
	static float32 GetAngularSlop();
	static void resetEpId();
protected:

	friend class b2Joint;
	friend class b2ImpulseInitializer;
	friend class b2Island;

	b2ElasticPlasticJoint(const b2ElasticPlasticJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);

	float32 m_frequencyHz; 
	float32 m_dampingRatio;
	float32 m_bias;

	// Solver shared
	b2Vec2 m_localAnchorA;
	b2Vec2 m_localAnchorB;
	float32 m_referenceAngle;
	float32 m_gamma;
	b2Vec3 m_impulse;
	b2Vec2 m_maxForce;
	float32 m_maxTorque;
	// ep
	float32 positionError, angularError;
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

#endif
