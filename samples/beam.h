// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Beam modifications by Simo Nikula

#pragma once

#include "box2d/types.h"
#include <vector>

class Beam
{
public:
	/** Creates cantilever beam */
	Beam(b2WorldId worldId, b2Vec2 position);
	~Beam();
	b2BodyId m_groundId,m_bodyId;
	b2JointId m_jointId;
	/** update shapes based on current forces */
	void update(float timeStep);
	/** reset statics that are used for creation */
	static void reset();
	static float L, w, h, E, fy,density;
	float m_L, m_w, m_h, m_E, m_fy, m_density;

protected:
	std::vector<b2ShapeId> shapes;
	int m_jointCount, m_contactCount;
	b2JointId* m_joints;
	b2ContactData* m_contacts;
};
