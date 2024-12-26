// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Beam modifications by Simo Nikula

#pragma once

#include "box2d/types.h"

class Beam
{
public:
	Beam(b2WorldId worldId, b2Vec2 position);
	~Beam();
	b2BodyId m_groundId,m_bodyId;
	b2JointId m_jointId;
	static void reset();
	static float L, w, h, E, fy,density;
	float m_L, m_w, m_h, m_E, m_fy, m_density;
};
