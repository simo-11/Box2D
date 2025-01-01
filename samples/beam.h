// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
// Beam modifications by Simo Nikula

#pragma once

#include "box2d/types.h"
#include <vector>
#include "draw.h"

typedef struct b2UpdateData
{
	float timeStep;
	Draw* g_draw;
	Camera* g_camera;
} b2UpdateData;

class Load
{
public:
	b2Vec2 p; // in local frame of body/beam
	b2Vec2 f; // force
	float m;  // moment
};

enum BeamFlags_
{
	BeamFlags_None = 0,
	BeamFlags_ClampedAtStart = 1,
	BeamFlags_ClampedAtEnd = 1 << 1,
	BeamFlags_HingeAtStart = 1<<2,
	BeamFlags_HingeAtEnd = 1 << 3,
};


class Beam
{
public:
	/** Creates Beam 
	* beamFlags can be used to create few selected joints
	*/
	Beam( b2WorldId worldId, 
		b2Vec2 position = {0.,0.}, 
		float rotationInRadians=0, 
		int beamFlags=0);
	~Beam();
	void CleanLoads();
	b2BodyId m_groundIdStart,m_groundIdEnd,m_bodyId;
	virtual void DoBeamAnalysis( b2UpdateData updateData );
	virtual void CollectLoads( b2UpdateData& updateData );
	void CollectJoints();
	virtual bool IsModelUpdateNeeded();
	virtual void UpdateModel();
	/** reset statics that are used for creation */
	static void reset();
	static float L, w, h, E, fy,density, rotation;
	static int flags;
	static char* flag_labels[4];
	float m_L, m_w, m_h, m_E, m_fy, m_density;

protected:
	std::vector<b2ShapeId> m_shapes;
	std::vector<Load*> m_loads;
	int m_jointCount, m_contactCount;
	b2JointId* m_joints;
	b2ContactData* m_contacts;
};
