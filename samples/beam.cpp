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
	b2CreatePolygonShape( m_bodyId, &shapeDef, &box );
	b2Vec2 pivot = { 0.f, 0.0f };
	jointDef.bodyIdA = m_groundId;
	jointDef.bodyIdB = m_bodyId;
	jointDef.localAnchorA = b2Body_GetLocalPoint( jointDef.bodyIdA, pivot );
	jointDef.localAnchorB = b2Body_GetLocalPoint( jointDef.bodyIdB, pivot );
	m_jointId = b2CreateWeldJoint( worldId, &jointDef );
}
void Beam::reset()
{
Beam::L = 10.0f;
Beam::w = 1.0f;
Beam::h = 0.5f;
Beam::density = 7800.f;
Beam::E = 210E9;
Beam::fy = 350E6;
}

Beam::~Beam()
{
	b2DestroyJoint( m_jointId );
	b2DestroyBody( m_bodyId );
}