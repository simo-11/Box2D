/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef TEST_H
#define TEST_H

#include <Box2D/Box2D.h>
#include "DebugDraw.h"
#include "RigidTriangle.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#endif
#include <glfw/glfw3.h>

#include <stdlib.h>
#include "SelectedEPJoint.h"

class Test;
struct Settings;

typedef Test* TestCreateFcn(Settings*);

#define	RAND_LIMIT	32767
#define DRAW_STRING_NEW_LINE 16

/// Random number in range [-1,1]
inline float32 RandomFloat()
{
	float32 r = (float32)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = 2.0f * r - 1.0f;
	return r;
}

/// Random floating point number in range [lo, hi]
inline float32 RandomFloat(float32 lo, float32 hi)
{
	float32 r = (float32)(rand() & (RAND_LIMIT));
	r /= RAND_LIMIT;
	r = (hi - lo) * r + lo;
	return r;
}

/// Test settings. Some can be controlled in the GUI.
struct Settings
{
	Settings()
	{
		reset();
	}
	void reset(){
		hz = 60.0f;
		mouseJointForceScale = 30.f;
		velocityIterations = 8;
		positionIterations = 3;
		drawShapes = true;
		drawJoints = true;
		drawJointReactions = true;
		drawAABBs = false;
		drawContactPoints = false;
		drawContactNormals = false;
		drawContactImpulse = false;
		drawFrictionImpulse = false;
		drawCOMs = false;
		drawStats = false;
		drawProfile = false;
		drawNotes = true;
		enableWarmStarting = true;
		enableContinuous = true;
		enableSubStepping = false;
		enableSleep = true;
		pause = false;
		singleStep = false;
		forceScale = 0.1f;
		momentScale = 0.3f;
		epbScale = 100.f;
		epbHz = 0.f;
		epbMassScale = 300.f;
		addRigidTriangles = false;
		addEPBeams = false;
		initImpulses = false;
	}
	float32 hz;
	float32 mouseJointForceScale; 
	int32 velocityIterations;
	int32 positionIterations;
	bool drawShapes;
	bool drawJoints;
	bool drawJointReactions;
	bool drawAABBs;
	bool drawContactPoints;
	bool drawContactNormals;
	bool drawContactImpulse;
	bool drawFrictionImpulse;
	bool drawCOMs;
	bool drawStats;
	bool drawProfile;
	bool drawNotes;
	bool enableWarmStarting;
	bool enableContinuous;
	bool enableSubStepping;
	bool enableSleep;
	bool pause;
	bool singleStep;
	bool addRigidTriangles, addMasses, addEPBeams,selectEPJoint; // using ALT-MB1 if active
	float32 addMass = 1000000;
	bool initImpulses;
	float32 forceScale, momentScale, epbScale, epbMassScale, epbHz;
};

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern TestEntry g_testEntries[];
// This is called when a joint in the world is implicitly destroyed
// because an attached body is destroyed. This gives us a chance to
// nullify the mouse joint.
class DestructionListener : public b2DestructionListener
{
public:
	void SayGoodbye(b2Fixture* fixture) { B2_NOT_USED(fixture); }
	void SayGoodbye(b2Joint* joint);

	Test* test;
};

const int32 k_maxContactPoints = 2048;

struct ContactPoint
{
	b2Fixture* fixtureA;
	b2Fixture* fixtureB;
	b2Vec2 normal;
	b2Vec2 position;
	b2PointState state;
	float32 normalImpulse;
	float32 tangentImpulse;
	float32 separation;
};

class Test : public b2ContactListener
{
public:

	Test(Settings* settings);
	virtual ~Test();

	void DrawTitle(const char *string);
	virtual void Step();
	virtual void Keyboard(int key) { B2_NOT_USED(key); }
	virtual void KeyboardUp(int key) { B2_NOT_USED(key); }
	void ShiftMouseDown(const b2Vec2& p);
	void ControlMouseDown(const b2Vec2& p);
	void AltMouseDown(const b2Vec2& p);
	virtual void MouseDown(const b2Vec2& p, int32 mods);
	virtual void MouseUp(const b2Vec2& p);
	void MouseMove(const b2Vec2& p);
	void LaunchBomb();
	void LaunchBomb(const b2Vec2& position, const b2Vec2& velocity);
	
	void SpawnBomb(const b2Vec2& worldPt);
	void CompleteBombSpawn(const b2Vec2& p);

	// Let derived tests know that a joint was destroyed.
	virtual void JointDestroyed(b2Joint* joint) { B2_NOT_USED(joint); }

	// Callbacks for derived classes.
	virtual void BeginContact(b2Contact* contact) { B2_NOT_USED(contact); }
	virtual void EndContact(b2Contact* contact) { B2_NOT_USED(contact); }
	virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);
	virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
	{
		B2_NOT_USED(contact);
		B2_NOT_USED(impulse);
	};
	void ShiftOrigin(const b2Vec2& newOrigin);
	// ep-start
	virtual void Ui(){};
	virtual void UpdateCamera() {};
	virtual void drawNotes(){};
	void LogJoint(b2Joint* j, float32 fScale, float32 mScale, float[4],
		const char * fmt = "%5.2f", float32 maxValue = (float32)FLT_MAX, float32 minMaxValue = 0.f);
	void LogSelectedJoints(float32 fScale, float32 mScale, float[4],
		const char * fmt = "%5.2f", float32 maxValue = (float32)FLT_MAX, float32 minMaxValue = 0.f);
	virtual void LogContact(ContactPoint* cp, float32 scale, float[3],
		const char * fmt = "%5.2f");
	virtual void LogEpCapasity(b2ElasticPlasticJoint* j,float[4]);
	virtual void LogEpCapasityForSelectedJoints(float[4]);
	virtual void LogEpJointErrors(b2ElasticPlasticJoint* j, float[2]);
	virtual void LogEpJointErrorsForSelectedJoints(float[2]);
	virtual void HighLightJoint(b2Joint* j);
	virtual bool IsSelectedJoint(b2Joint* j);
	virtual void SelectJoint(const b2Vec2& p);
	virtual void AddSelectedJoint(b2ElasticPlasticJoint* j);
	void DeleteSelectedJoint(b2Joint* j);
	void SelectedJointDeleted(b2Joint* j);
	void DeleteSelectedJoints();
	virtual SelectedEPJoint* GetSelectedJointList();
	virtual SelectedEPJoint* GetLastSelectedJoint();
	virtual void UpdatePlotValues(SelectedEPJoint*);
	virtual void SyncSelectedJoints();
	void StartTextHighLight();
	void EndTextHighLight();
	/* Add masses */
	virtual bool WantMasses();
	virtual void AddMass(const b2Vec2& p);
	/* Rigid triangles */
	virtual bool WantRigidTriangles();
	virtual void AddRigidTriangle(const b2Vec2& p);
	// static Allows RigidTriangles to survive during restarts
	static void DeleteRigidTriangle(unsigned char label);
	static void DeleteRigidTriangles();
	virtual void CreateRigidTriangles();
	virtual void AddRigidTriangleBody(RigidTriangle*);
	virtual RigidTriangle* GetRigidTriangleList();
	virtual RigidTriangle* GetLastRigidTriangle();
	/* Elastic plastic beams */
	virtual bool WantEPBeams();
	virtual void AddEPBeam(const b2Vec2& p);
	// static Allows EPBeam to survive during restarts
	static void DeleteEPBeam(unsigned char label);
	static void DeleteEPBeams();
	static void DeleteSBodyforEPBeams(b2Body * b);
	virtual void CreateEPBeams();
	virtual void AddEPBeamBody(EPBeam*);
	virtual EPBeam* GetEPBeamList();
	virtual EPBeam* GetLastEPBeam();
	virtual float getEpBeamDensity() { return 100; } /* rectangular, hollow 4% wall 10% in depth*/
	virtual float getEpBeamXSizeFactor() { return 0.05f; }
	virtual float  getEpBeamMaxForce() { return 1; }
	/** reset configurable settings */
	virtual void reset(){};
	virtual float getBombDensity(){ return 1000; }
	virtual float getBombRadius(){ return 0.3f; }
	virtual void wakeConnectedBodies(b2Body*);
	float32 steppedTime;
	b2Body* m_movingBody;
	// ep-end
protected:
	friend class DestructionListener;
	friend class BoundaryListener;
	friend class ContactListener;

	b2Body* m_groundBody;
	b2AABB m_worldAABB;
	ContactPoint m_points[k_maxContactPoints];
	int32 m_pointCount;
	DestructionListener m_destructionListener;
	int32 m_textLine;
	b2World* m_world;
	b2Body* m_bomb;
	b2MouseJoint* m_mouseJoint;
	// ep
	b2Vec2 m_localPointForMovingBody;
	b2Body* loggedBody;
	b2Vec3 max, min;
	// ep
	b2Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	b2Vec2 m_mouseWorld;
	int32 m_stepCount;

	b2Profile m_maxProfile;
	b2Profile m_totalProfile;
	Settings *settings;
};

#endif
