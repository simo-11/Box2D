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

#if defined(__APPLE_CC__)
#define GLFW_INCLUDE_GLCOREARB
#include <OpenGL/gl3.h>
#else
#include "Testbed/glad/glad.h"
#endif

#include <glfw/glfw3.h>

#include <stdlib.h>
#include "EpJoint.h"

class Test;
struct Settings;
struct SelectedEPJoint;
struct RigidTriangle;
struct EPBeam;

typedef Test* TestCreateFcn(Settings*);

enum BombShape {
	CIRCLE=0,
	RECTANGLE=1,
};

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
		epDebugSteps = 10;
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
		// 100*100*4 mm RHS@350MPa
		epbMaxForce = 15.3f*100*275/2; // 15 cm^2 275 MPa
		epbMaxMoment = 54.9*275; // 54.9cm^3  275 MPa [Nm]
		epbMaxStrain = 0.1f;
		epbMaxRotation = 3;
		epbX = 0.1f;
		epbY = 3;
		epbMaxElasticRotation = 0;
		epbDebugListener = true;
		epbHz = 0.f;
		epbMass = 36.f;
		addRigidTriangles = false;
		addEPBeams = false;
		initImpulses = false;
		gravityRampUpTime = 0.0f;
		bombMass = 1000;
		bombShape = CIRCLE;
		bombWidth = 2;
		bombMultiplier = 10;
		bombVelocity = b2Vec2(5, 0);
		bombSpawnPoint = b2Vec2(-0.5f*bombWidth, 0.5f*bombWidth);
	}
	float32 hz;
	float32 mouseJointForceScale; 
	int32 velocityIterations;
	int32 epDebugSteps;
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
	float32 targetTime = 60;
	bool singleStep;
	bool addRigidTriangles, addMasses, addEPBeams,selectEPJoint; // using ALT-MB1 if active
	float32 addMass = 1000000, addMassSize=2;
	bool initImpulses;
	float32 forceScale, momentScale;
	float32 epbMaxForce, epbMaxMoment, epbX, epbY,
		epbMass, epbHz, epbMaxRotation, epbMaxStrain, epbMaxElasticRotation;
	bool epbDebugListener;
	float32 gravityRampUpTime;
	float32 bombMass, bombWidth, bombMultiplier;
	BombShape bombShape;
	b2Vec2 bombVelocity,bombSpawnPoint;
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
	void UpdateBombVelocity(const b2Vec2& p);

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
	virtual SelectedEPJoint* AddSelectedJoint(b2ElasticPlasticJoint* j);
	void DeleteSelectedJoint(b2Joint* j);
	virtual bool OpenEPJoints() { return false; }
	void SelectedJointDeleted(b2Joint* j);
	void DeleteSelectedJoints();
	virtual SelectedEPJoint* GetSelectedJointList();
	virtual SelectedEPJoint* GetLastSelectedJoint();
	virtual SelectedEPJoint* GetSelectedEPJoint(b2Joint *j);
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
	/** handles recreation of RigidTriangle:s, EPBeam:s and SelectedEPJoint:s */
	virtual void CommonEpInit();
	virtual void AddRigidTriangleBody(RigidTriangle*);
	virtual RigidTriangle* GetRigidTriangleList();
	virtual RigidTriangle* GetLastRigidTriangle();
	/* Elastic plastic beams */
	virtual bool WantEPBeams();
	virtual EPBeam* AddEPBeam(const b2Vec2& p);
	// static Allows EPBeam to survive during restarts
	static void DeleteEPBeam(unsigned char label);
	static void DeleteEPBeams();
	static void DeleteSBodyforEPBeams(b2Body * b);
	virtual void CreateEPBeams();
	virtual void AddEPBeamBody(EPBeam*);
	virtual EPBeam* GetEPBeamList();
	virtual EPBeam* GetLastEPBeam();
	/** reset configurable settings */
	virtual void reset(){};
	virtual float getBombMass();
	virtual float getBombWidth();
	virtual float getBombVelocity();
	b2Vec2 getBombSpawnPoint();
	void setBombSpawnPoint(b2Vec2);
	virtual void wakeConnectedBodies(b2Body*);
	float32 steppedTime;
	b2Body* m_movingBody;
	static bool restartPending,isRestart;
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
	b2Vec2 pv,av;
	float32 pva, aa;
	bool validAcc=false;
	b2Body* pb;
	b2Vec3 max, min;
	void ResetMinAndMax();
	// ep
	b2Vec2 m_bombSpawnPoint;
	bool m_bombSpawning;
	b2Vec2 m_bombVelocity;
	b2Vec2 m_mouseWorld;
	int32 m_stepCount;

	b2Profile m_maxProfile;
	b2Profile m_totalProfile;
	Settings *settings;
};

#endif
