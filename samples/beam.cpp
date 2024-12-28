// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Beam modifications by Simo Nikula

#include "beam.h"

#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

Beam::Beam( b2WorldId worldId, b2Vec2 position)
{
	m_L=Beam::L;
	m_w=Beam::w;
	m_h = Beam::h;
	m_density = Beam::density;
	m_E = Beam::E;
	m_fy = Beam::fy;
	m_contacts = nullptr;
	m_joints = nullptr;
	m_contactCount = 0;
	m_jointCount = 0;
	b2BodyDef bodyDef = b2DefaultBodyDef();
	m_groundId = b2CreateBody( worldId, &bodyDef );
	float hx = 0.5f * m_L;
	float hh = 0.5f * m_h;
	b2Polygon box = b2MakeBox( hx, hh );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = m_density;
	b2WeldJointDef jointDef = b2DefaultWeldJointDef();
	bodyDef.type = b2_dynamicBody;
	bodyDef.isAwake = false;
	bodyDef.position = {hx, 0.0f };
	m_bodyId = b2CreateBody( worldId, &bodyDef );
	shapes.push_back(b2CreatePolygonShape( m_bodyId, &shapeDef, &box ));
	b2Vec2 pivot = { 0.f, 0.0f };
	jointDef.bodyIdA = m_groundId;
	jointDef.bodyIdB = m_bodyId;
	jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
	jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
	m_jointId = b2CreateWeldJoint( worldId, &jointDef );
}
float Beam::L, Beam::w, Beam::h, Beam::density, Beam::E, Beam::fy;
void Beam::reset()
{
Beam::L = 10.0f;
Beam::w = 1.0f;
Beam::h = 0.5f;
Beam::density = 7800.f;
Beam::E = 210E9;
Beam::fy = 350E6;
}

void Beam::update(float timeStep)
{
	if (!b2Body_IsAwake(m_bodyId)) {
		return;
	}
	b2Vec2 force = { 0.f, 0.f };
	float moment = 0.f;
	int jointCount = b2Body_GetJointCount( m_bodyId );
	if ( jointCount != m_jointCount )
	{
		if ( m_joints != nullptr )
		{
			free( m_joints );
		}
		m_jointCount = jointCount;
		m_joints = (b2JointId*)malloc( m_jointCount * sizeof( b2JointId ) );
	}
	if ( m_joints != nullptr )
	{
		jointCount = b2Body_GetJoints( m_bodyId, m_joints, m_jointCount );
		for ( int i = 0; i < jointCount; i++ )
		{
			b2Vec2 f=b2Joint_GetConstraintForce( m_joints[i] );
			force.x += b2AbsFloat( f.x );
			force.y += b2AbsFloat( f.y );
			float m = b2Joint_GetConstraintTorque( m_joints[i] );
			moment += fabs( m );
		}
	}
	int contactCount = b2Body_GetContactCapacity( m_bodyId );
	if ( contactCount != m_contactCount )
	{
		if ( m_contacts != nullptr )
		{
			free( m_contacts );
		}
		m_contactCount = contactCount;
		m_contacts = (b2ContactData*)malloc( m_contactCount * sizeof( b2ContactData ) );
	}
	if ( m_contacts != nullptr )
	{
		float inv_h = 1.f/timeStep;
		contactCount = b2Body_GetContactData( m_bodyId, m_contacts, m_contactCount );
		for ( int i = 0; i < contactCount; i++ )
		{
			b2ContactData cd = m_contacts[i];
			b2Manifold manifold = cd.manifold;
			b2Vec2 normal = manifold.normal;
			b2Vec2 tangent = b2RightPerp( normal );
			for ( int pi = 0; pi < manifold.pointCount; pi++ )
			{
				b2ManifoldPoint manifoldPoint = manifold.points[pi];
				float nf=inv_h*manifoldPoint.maxNormalImpulse;
				float tf = inv_h * manifoldPoint.tangentImpulse;
				b2Vec2 f = b2MulSV( nf, normal );
				force.x += b2AbsFloat( f.x);
				force.y += b2AbsFloat( f.y );
				// moment += fabs( m );
				f = b2MulSV( tf, tangent );
				force.x += b2AbsFloat( f.x );
				force.y += b2AbsFloat( f.y );
				// moment += fabs( m );
			}
		}
	}

	
	// is worth update or return
	// update shape or recreate it
}

Beam::~Beam()
{
	b2DestroyJoint( m_jointId );
	b2DestroyBody( m_bodyId );
	if ( m_contacts != nullptr )
	{
		free( m_contacts );
	}
	if ( m_joints != nullptr )
	{
		free( m_joints );
	}
}
