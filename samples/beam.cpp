// SPDX-FileCopyrightText: 2022 Erin Catto
// SPDX-License-Identifier: MIT
// Beam modifications by Simo Nikula

#include "beam.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"

#include <assert.h>

Beam::Beam( b2WorldId worldId, b2Vec2 position, float rotation, int beamFlags)
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
	bodyDef.position = position;
	b2Rot rot = b2MakeRot( rotation );
	if ( beamFlags & BeamFlags_ClampedAtStart || BeamFlags_HingeAtStart )
	{
		m_groundIdStart = b2CreateBody( worldId, &bodyDef );
	}
	if ( beamFlags & BeamFlags_ClampedAtStart || BeamFlags_HingeAtStart )
	{
		bodyDef.position.x = position.x+rot.c*m_L;
		bodyDef.position.y = position.y + rot.s * m_L;
		m_groundIdEnd = b2CreateBody( worldId, &bodyDef );
	}
	float hx = 0.5f * m_L;
	float hh = 0.5f * m_h;
	b2Polygon box = b2MakeBox( hx, hh );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = m_density;
	bodyDef.type = b2_dynamicBody;
	bodyDef.isAwake = false;
	bodyDef.position = {position.x+hx, position.y };
	bodyDef.rotation = rot;
	m_bodyId = b2CreateBody( worldId, &bodyDef );
	m_shapes.push_back(b2CreatePolygonShape( m_bodyId, &shapeDef, &box ));
	if ( beamFlags & BeamFlags_ClampedAtStart )
	{
		b2Vec2 pivot = { 0.f, 0.0f };
		b2WeldJointDef jointDef = b2DefaultWeldJointDef();
		jointDef.bodyIdA = m_groundId;
		jointDef.bodyIdB = m_bodyId;
		jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
		jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
		m_jointId = b2CreateWeldJoint( worldId, &jointDef );
	}
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

void Beam::DoBeamAnalysis( b2UpdateData updateData )
{
	if (!b2Body_IsAwake(m_bodyId)) {
		return;
	}
	CleanLoads();
	CollectLoads( updateData );
	if ( IsModelUpdateNeeded() )
	{
		UpdateModel();
	}
}

bool Beam::IsModelUpdateNeeded()
{
	return 2 > 1;
}

void Beam::UpdateModel()
{
}

void Beam::CollectLoads( b2UpdateData& updateData )
{
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
			b2JointId jointId = m_joints[i];
			b2BodyId ba = b2Joint_GetBodyA( jointId );
			b2Vec2 anchor;
			if ( ba.index1 == m_bodyId.index1 )
			{
				anchor = b2Joint_GetLocalAnchorA( jointId );
			}
			else
			{
				anchor = b2Joint_GetLocalAnchorB( jointId );
			}
			Load* load = new Load();
			load->p = anchor;
			load->f = b2Joint_GetConstraintForce( jointId );
			load->m = b2Joint_GetConstraintTorque( jointId );
			m_loads.push_back( load );
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
		float inv_h = 1.f / updateData.timeStep;
		contactCount = b2Body_GetContactData( m_bodyId, m_contacts, m_contactCount );
		for ( int i = 0; i < contactCount; i++ )
		{
			b2ContactData cd = m_contacts[i];
			bool useA = false;
			b2BodyId ba = b2Shape_GetBody( cd.shapeIdA );
			if ( ba.index1 == m_bodyId.index1 )
			{
				useA = true;
			}
			b2Manifold manifold = cd.manifold;
			b2Vec2 normal = manifold.normal;
			b2Vec2 tangent = b2RightPerp( normal );
			for ( int pi = 0; pi < manifold.pointCount; pi++ )
			{
				b2ManifoldPoint manifoldPoint = manifold.points[pi];
				b2Vec2 anchor;
				if ( useA )
				{
					anchor = manifoldPoint.anchorA;
				}
				else
				{
					anchor = manifoldPoint.anchorB;
				}
				float nf = inv_h * manifoldPoint.normalImpulse;
				float tf = inv_h * manifoldPoint.tangentImpulse;
				Load* load = new Load();
				load->p = anchor;
				load->f = b2MulSV( nf, normal );
				m_loads.push_back( load );
				load = new Load();
				load->p = anchor;
				load->f = b2MulSV( tf, tangent );
			}
		}
	}
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
	CleanLoads();
}

void Beam::CleanLoads()
{
	while ( !m_loads.empty() )
	{
		delete m_loads.back();
		m_loads.pop_back();
	}
}
