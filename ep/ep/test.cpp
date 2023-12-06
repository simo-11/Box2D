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
#define DEFINE_EPTEST_NS
#define _CRT_SECURE_NO_WARNINGS
#include "test.h"
#include "ep_joint.h"
#include <stdio.h>
#include <imgui/imgui.h>
#include "dynamics/b2_island.h"

namespace {
	b2World* rtWorld=NULL;
	bool allowAddRigidTriangle = false;
	bool rigidTriangleInitialized = false;
	b2PolygonShape rigidTriangle;
	RigidTriangle* rigidTriangleList = nullptr;
	bool allowEPBeam = false;
	EPBeam* epBeamList = nullptr;
	SelectedEPJoint* currentJointList=nullptr;
}

void DestructionListener::SayGoodbye(b2Joint* joint)
{
	if (test->m_mouseJoint == joint)
	{
		test->m_mouseJoint = NULL;
	}
	else
	{
		test->JointDestroyed(joint);
	}
	test->SelectedJointDeleted(joint);
}

void Test::ResetMinAndMax() {
	max.Set(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
}

bool Test::isRestart = false;
bool Test::restartPending = false;

Test::Test(Settings *sp)
{
	if (restartPending) {
		isRestart = true;
	}
	restartPending = false;
	b2ElasticPlasticJoint::resetEpId();
	settings = sp;
	EpDebug::settings = sp;
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	m_world = new b2World(gravity);
	m_bomb = NULL;
	m_textLine = 30;
	m_mouseJoint = NULL;
	m_movingBody = NULL;
	m_bombSpawnPoint = sp->bombSpawnPoint;
	m_bombVelocity = b2Vec2(10, 0);
	loggedBody = NULL;
	steppedTime = 0;
	m_pointCount = 0;
	ResetMinAndMax();
	m_destructionListener.test = this;
	m_world->SetDestructionListener(&m_destructionListener);
	m_world->SetContactListener(this);
	m_world->SetDebugDraw(&g_debugDraw);
	
	m_bombSpawning = false;

	m_stepCount = 0;

	b2BodyDef bodyDef;
	m_groundBody = m_world->CreateBody(&bodyDef);

	memset(&m_maxProfile, 0, sizeof(b2Profile));
	memset(&m_totalProfile, 0, sizeof(b2Profile));
	rtWorld = m_world;
}

Test::~Test()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete m_world;
	m_world = NULL;
	rtWorld = NULL;
	// joints in list are no longer valid
	for (SelectedEPJoint* j = GetSelectedJointList();
		j != nullptr; j = j->next) {
		j->joint = NULL;
		j->DetachEpDebug();
	}
	epLogClose();
}

SelectedEPJoint* Test::AddSelectedJoint(b2ElasticPlasticJoint * j)
{
	SelectedEPJoint* rt = GetSelectedEPJoint(j);
	if (rt!=nullptr) {
		return rt; // already selected
	}
	rt = GetLastSelectedJoint();
	if (rt->joint != nullptr) {
		rt->next = new SelectedEPJoint();
		rt = rt->next;
	}
	rt->id = j->GetId();
	rt->joint = j;
	rt->highlight = true;
	return rt;
}

SelectedEPJoint * Test::GetSelectedEPJoint(b2Joint * jc)
{
	for (SelectedEPJoint* j = GetSelectedJointList();
		j != nullptr; j = j->next) {
		if (j->joint == jc) {
			return j;
		}
	}
	return nullptr;
}


bool Test::IsSelectedJoint(b2Joint * jc)
{
	return (GetSelectedEPJoint(jc) != nullptr);
}

void Test::SelectedJointDeleted(b2Joint * j)
{
	SelectedEPJoint* rt = currentJointList;
	while (rt != nullptr) {
		if (rt->joint == j) {
			rt->joint = nullptr;
			return;
		}
		rt = rt->next;
	}
}

void Test::DeleteSelectedJoint(b2Joint * j)
{
	SelectedEPJoint* rt = currentJointList;
	SelectedEPJoint* rtn = nullptr, *rtp = nullptr;
	while (rt != nullptr) {
		rtn = rt->next;
		if (rt->joint == j) {
			if (rt->forces != NULL) {
				delete rtn->forces;
			}
			if (rt->capacities != NULL) {
				delete rtn->capacities;
			}
			delete rt;
			if (rtp != nullptr) { // second or later
				rtp->next = rtn;
			}
			goto deleteDone;
		}
		rtp = rt;
		rt = rtn;
	}
deleteDone:
	if (rt == currentJointList) { // if first was deleted
		currentJointList = rtn;
	}
	DeleteSBodyforEPBeams(j->GetBodyA());
}

void Test::SyncSelectedJoints()
{
	SelectedEPJoint* rt = GetSelectedJointList();
	SelectedEPJoint* rtn = nullptr, *rtp = rt;
	while (rt != nullptr) {
		rtn = rt->next;
		for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
		{
			switch (j->GetType()) {
			case e_rigidPlasticJoint:
			case e_elasticPlasticJoint:
				b2Vec2 jp = 0.5f*(j->GetAnchorA() + j->GetAnchorB());
				b2ElasticPlasticJoint* epj = (b2ElasticPlasticJoint*)j;
				if (rt->id == epj->GetId()) {
					rt->joint = epj;
					if (rt->dep) {
						rt->AttachEpDebug();
					}
					goto found;
				}
				break;
			}
		}
		// not found, remove from list
		if (rt == currentJointList) { // if first was deleted
			currentJointList = rtn;
		}
		else {
			rtp->next = rtn;
		}
		delete rt;
		rt = rtp;
	found:
		rtp = rt;
		rt = rtn;
	}
}


void Test::DeleteSelectedJoints()
{
	SelectedEPJoint* rt = currentJointList;
	while (rt != nullptr) {
		SelectedEPJoint* rtn = rt->next;
		if (rt->forces != NULL) {
			delete rt->forces;
		}
		if (rt->capacities != NULL) {
			delete rt->capacities;
		}
		delete rt;
		rt = rtn;
	}
	currentJointList = nullptr;
}

SelectedEPJoint * Test::GetSelectedJointList()
{
	return currentJointList;
}

SelectedEPJoint * Test::GetLastSelectedJoint()
{
	SelectedEPJoint* rt = currentJointList;
	if (rt == nullptr) {
		currentJointList = new SelectedEPJoint();
		return currentJointList;
	}
	while (rt->next != nullptr) {
		rt = rt->next;
	}
	return rt;
}


bool Test::WantRigidTriangles(){
	return true;
}
bool Test::WantMasses() {
	return true;
}
void Test::CommonEpInit(){
	RigidTriangle* rt = rigidTriangleList;
	while (rt != nullptr){
		AddRigidTriangleBody(rt);
		rt = rt->next;
	}
	if (WantEPBeams()) {
		CreateEPBeams();
	}
	SyncSelectedJoints();
}


void Test::DeleteRigidTriangles(){
	RigidTriangle* rt = rigidTriangleList;
	while (rt != nullptr){
		RigidTriangle* rtn = rt->next;
		if (rtWorld){
			rtWorld->DestroyBody(rt->body);
		}
		delete rt;
		rt = rtn;
	}
	rigidTriangleList = nullptr;
	rigidTriangleInitialized = false;
}

void Test::DeleteRigidTriangle(unsigned char label){
	RigidTriangle* rt = rigidTriangleList;
	RigidTriangle* rtn = nullptr, *rtp=nullptr;
	while (rt != nullptr){
		rtn = rt->next;
		if (rt->label == label){
			if (rtWorld){
				rtWorld->DestroyBody(rt->body);
			}
			delete rt;
			if (rtp != nullptr){ // second or later
				rtp->next = rtn;
			}
			goto deleteDone;
		}
		rtp = rt;
		rt = rtn;
	}
	deleteDone:
	if (rt == rigidTriangleList){ // if first was deleted
		rigidTriangleList = rtn;
	}
}

RigidTriangle* Test::GetRigidTriangleList(){
	return rigidTriangleList;
}
RigidTriangle* Test::GetLastRigidTriangle(){
	RigidTriangle* rt = rigidTriangleList;
	if (rt == nullptr){
		rigidTriangleList = new RigidTriangle();
		return rigidTriangleList;
	}
	while (rt->next!=nullptr){
		rt = rt->next;
	}
	return rt;
}
void Test::AddRigidTriangle(const b2Vec2& p){
	if (!rigidTriangleInitialized){
		b2Vec2 vertices[3];
		float zoom = g_camera.m_zoom;
		vertices[0].Set(-0.5f*zoom, 0.0f);
		vertices[1].Set(0.5f*zoom, 0.0f);
		vertices[2].Set(0.0f, 1.5f*zoom);
		rigidTriangle.Set(vertices, 3);
		rigidTriangleInitialized = true;
	}
	RigidTriangle* rt = GetLastRigidTriangle();
	if (rt->body != nullptr){
		rt->next = new RigidTriangle();
		rt->next->label = (rt->label + 1);
		rt = rt->next;
	}
	rt->position[0] = p.x;
	rt->position[1] = p.y;
	AddRigidTriangleBody(rt);
}

void Test::AddMass(const b2Vec2& p) {
	b2FixtureDef fd;
	b2PolygonShape shape;
	float halfSize = settings->addMassSize / 2;
	shape.SetAsBox(halfSize, halfSize);
	fd.shape = &shape;
	fd.density = settings->addMass/4.f/ halfSize / halfSize;
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(p.x,p.y);
	b2Body* body = m_world->CreateBody(&bd);
	body->SetBullet(true);
	body->CreateFixture(&fd);
}

void Test::AddRigidTriangleBody(RigidTriangle* rt){
	b2FixtureDef fd;
	fd.shape = &rigidTriangle;
	b2BodyDef bd;
	bd.type = b2_staticBody;
	bd.position.Set(rt->position[0], rt->position[1]);
	b2Body* body = m_world->CreateBody(&bd);
	body->CreateFixture(&fd);
	rt->body = body;
}

bool Test::WantEPBeams() {
	return true;
}
void Test::CreateEPBeams() {
	EPBeam* rt = epBeamList;
	while (rt != nullptr) {
		AddEPBeamBody(rt);
		rt = rt->next;
	}
}

void Test::DeleteSBodyforEPBeams(b2Body *b) {
	EPBeam* rt = epBeamList;
	while (rt != nullptr) {
		EPBeam* rtn = rt->next;
		if (rt->sBody == b) {
			rt->deleteSbody = true;
		}
		rt = rtn;
	}
}

void Test::DeleteEPBeams() {
	EPBeam* rt = epBeamList;
	while (rt != nullptr) {
		EPBeam* rtn = rt->next;
		if (rtWorld) {
			rtWorld->DestroyBody(rt->body);
			rtWorld->DestroyBody(rt->sBody);
		}
		delete rt;
		rt = rtn;
	}
	epBeamList = nullptr;
}

void Test::DeleteEPBeam(unsigned char label) {
	EPBeam* rt = epBeamList;
	EPBeam* rtn = nullptr, *rtp = nullptr;
	while (rt != nullptr) {
		rtn = rt->next;
		if (rt->label == label) {
			if (rtWorld) {
				rtWorld->DestroyBody(rt->body);
				rtWorld->DestroyBody(rt->sBody);
			}
			delete rt;
			if (rtp != nullptr) { // second or later
				rtp->next = rtn;
			}
			goto deleteDone;
		}
		rtp = rt;
		rt = rtn;
	}
deleteDone:
	if (rt == epBeamList) { // if first was deleted
		epBeamList = rtn;
	}
}

EPBeam* Test::GetEPBeamList() {
	return epBeamList;
}
EPBeam* Test::GetLastEPBeam() {
	EPBeam* rt = epBeamList;
	if (rt == nullptr) {
		epBeamList = new EPBeam();
		return epBeamList;
	}
	while (rt->next != nullptr) {
		rt = rt->next;
	}
	return rt;
}
EPBeam* Test::AddEPBeam(const b2Vec2& p) {
	EPBeam* rt = GetLastEPBeam();
	if (rt->body != nullptr) {
		rt->next = new EPBeam();
		rt->next->label = (rt->label + 1);
		rt = rt->next;
	}
	rt->position[0] = p.x;
	rt->position[1] = p.y;
	AddEPBeamBody(rt);
	return rt;
}

void Test::AddEPBeamBody(EPBeam* rt) {
	b2FixtureDef fd;
	b2Vec2 vertices[3];
	b2PolygonShape epBeam, epBeamHolder;
	float hx = settings->epbX / 2;
	float hy = settings->epbY/2;
	epBeam.SetAsBox(hx, hy);
	float bhs = b2Min(hx, hy);
	epBeamHolder.SetAsBox(bhs, bhs);
	fd.density = settings->epbMass/4/hx/hy;
	fd.shape = &epBeam;
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(rt->position[0], rt->position[1]+hy+2*hx);
	bd.angle = 0.0f;
	b2Body* body = m_world->CreateBody(&bd);
	body->CreateFixture(&fd);
	b2ElasticPlasticJointDef jd;
	jd.maxForce.x = settings->epbMaxForce;
	jd.maxForce.y = settings->epbMaxForce;
	/**
	* maxTorque should be maxForce* (h/4 ... h)
	* h/4 for solid rectangle
	* h is theoretical maximum for infinite thin web(s)
	* 
	*/
	jd.maxTorque = settings->epbMaxMoment;
	jd.maxRotation = settings->epbMaxRotation;
	jd.maxStrain = settings->epbMaxStrain;
	jd.frequencyHz = settings->epbHz;
	if (jd.frequencyHz > 0.f) {
		jd.maxElasticRotation = settings->epbMaxElasticRotation;
	}
	jd.dampingRatio = 0.1f;
	rt->body = body;
	const b2Vec2 anchor(rt->position[0],rt->position[1]+2*hx);
	bd.type = b2_staticBody;
	bd.position.Set(rt->position[0], rt->position[1]);
	b2Body* sBody = m_world->CreateBody(&bd);
	fd.density = 0.f;
	fd.shape = &epBeamHolder;
	sBody->CreateFixture(&fd);
	rt->sBody = sBody;
	jd.Initialize(sBody, body, anchor);
	b2ElasticPlasticJoint* joint=
		(b2ElasticPlasticJoint*)m_world->CreateJoint(&jd);
	rt->joint = joint;
}

void Test::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
	const b2Manifold* manifold = contact->GetManifold();

	if (manifold->pointCount == 0)
	{
		return;
	}

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	b2PointState state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
	b2GetPointStates(state1, state2, oldManifold, manifold);

	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);

	for (int32 i = 0; i < manifold->pointCount && m_pointCount < k_maxContactPoints; ++i)
	{
		ContactPoint* cp = m_points + m_pointCount;
		cp->fixtureA = fixtureA;
		cp->fixtureB = fixtureB;
		cp->position = worldManifold.points[i];
		cp->normal = worldManifold.normal;
		cp->state = state2[i];
		cp->normalImpulse = manifold->points[i].normalImpulse;
		cp->tangentImpulse = manifold->points[i].tangentImpulse;
		cp->separation = worldManifold.separations[i];
		++m_pointCount;
	}
}

void Test::DrawTitle(const char *string)
{
    g_debugDraw.DrawString(5, DRAW_STRING_NEW_LINE, string);
    m_textLine = 3 * DRAW_STRING_NEW_LINE;
}

class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = NULL;
	}

	bool ReportFixture(b2Fixture* fixture)
	{
		bool inside = fixture->TestPoint(m_point);
		if (inside)
		{
			m_fixture = fixture;
			// We are done, terminate the query.
			return false;
		}
		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

void Test::MouseDown(const b2Vec2& p, int32 mods)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint != NULL)
	{
		return;
	}

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	m_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture)
	{
		b2Body* body = callback.m_fixture->GetBody();
		loggedBody = body;
		ResetMinAndMax();
		if (!(mods&GLFW_MOD_CONTROL)){
			if (body->GetType() == b2_dynamicBody)
			{
				b2MouseJointDef md;
				md.bodyA = m_groundBody;
				md.bodyB = body;
				md.target = p;
				md.maxForce = settings->mouseJointForceScale* body->GetMass();
				m_mouseJoint = (b2MouseJoint*)m_world->CreateJoint(&md);
				body->SetAwake(true);
			}
			else{
				m_movingBody = body;
				b2Vec2 np;
				m_localPointForMovingBody=body->GetLocalPoint(p);
			}
		}
	}
	else{
		loggedBody = NULL;
		ResetMinAndMax();
	}
}

void Test::SpawnBomb(const b2Vec2& worldPt)
{
	m_bombSpawnPoint = worldPt;
	m_bombSpawning = true;
}
  
b2Vec2 Test::getBombSpawnPoint()
{
	return m_bombSpawnPoint;
}
void Test::setBombSpawnPoint(const b2Vec2 p)
{
	m_bombSpawnPoint=p;
}
void Test::CompleteBombSpawn(const b2Vec2& p)
{
	if (m_bombSpawning == false)
	{
		return;
	}
	UpdateBombVelocity(p);
	LaunchBomb(m_bombSpawnPoint, m_bombVelocity);
	m_bombSpawning = false;
}

void Test::UpdateBombVelocity(const b2Vec2& p)
{
	if (m_bombSpawning == false)
	{
		return;
	}
	b2Vec2 vel = m_bombSpawnPoint - p;
	m_bombVelocity = settings->bombMultiplier*vel;
}

float Test::getBombVelocity() {
	if (m_bombSpawning == false)
	{
		return 0.f;
	}
	return m_bombVelocity.Length();
}

void Test::ShiftMouseDown(const b2Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint != NULL)
	{
		return;
	}

	SpawnBomb(p);
}

void Test::ControlMouseDown(const b2Vec2& /* p */)
{
}
void Test::AltMouseDown(const b2Vec2& p)
{
	if (settings->addRigidTriangles){
		AddRigidTriangle(p);
	}else if (settings->addMasses) {
		AddMass(p);
	}else if(settings->addEPBeams) {
		AddEPBeam(p);
	}else if (settings->selectEPJoint) {
		SelectJoint(p);
	}
}

void Test::SelectJoint(const b2Vec2 & p)
{
	for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
	{
		switch (j->GetType()) {
		case e_rigidPlasticJoint:
		case e_elasticPlasticJoint:
			b2ElasticPlasticJoint* epj=(b2ElasticPlasticJoint*)j;
			b2Vec2 jp = 0.5f*(j->GetAnchorA() + j->GetAnchorB());
			float r2 = (j->GetAnchorA() - j->GetBodyA()->GetWorldCenter()).LengthSquared();
			float d2 = (p - jp).LengthSquared();
			if (d2 < r2) {
				AddSelectedJoint(epj);
			}
			break;
		}
	}
}

void Test::MouseUp(const b2Vec2& p)
{
	if (m_mouseJoint)
	{
		m_world->DestroyJoint(m_mouseJoint);
		m_mouseJoint = NULL;
	}
	if (m_movingBody != NULL){
		m_movingBody = NULL;
	}
	
	if (m_bombSpawning)
	{
		CompleteBombSpawn(p);
	}
}

void Test::MouseMove(const b2Vec2& p)
{
	m_mouseWorld = p;
	
	if (m_mouseJoint)
	{
		// Scale force so that 10 % of windows extents gives normalized force
		// and longer distance make force larger
		float d = (m_mouseJoint->GetAnchorB() - m_mouseJoint->GetAnchorA()).Length();
		float scale = b2Max(1.f,10*d / g_camera.m_width*g_camera.m_zoom);
		float force = scale*settings->mouseJointForceScale* m_mouseJoint->GetBodyB()->GetMass();
		m_mouseJoint->SetMaxForce(force);
		m_mouseJoint->SetTarget(p);
	}
	if (m_movingBody!=NULL){
		b2Vec2 np = p - m_localPointForMovingBody;
		m_movingBody->SetTransform(np,0);
		wakeConnectedBodies(m_movingBody);
	}
	if (m_bombSpawning)
	{
		UpdateBombVelocity(p);
	}

}

void Test::wakeConnectedBodies(b2Body* body){
	b2JointEdge* jointEdge=body->GetJointList();
	while (jointEdge != NULL){
		jointEdge->other->SetAwake(true);
		jointEdge = jointEdge->next;
	}
	b2ContactEdge* contactEdge=body->GetContactList();
	while (contactEdge != NULL){
		contactEdge->other->SetAwake(true);
		contactEdge = contactEdge->next;
	}
}

void Test::LaunchBomb()
{
	LaunchBomb(m_bombSpawnPoint, settings->bombVelocity);
}

void Test::LaunchBomb(const b2Vec2& position, const b2Vec2& velocity)
{
	if (m_bomb)
	{
		m_world->DestroyBody(m_bomb);
		m_bomb = NULL;
	}

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position = position;
	bd.bullet = true;
	m_bomb = m_world->CreateBody(&bd);
	m_bomb->SetLinearVelocity(velocity);
	
	b2FixtureDef fd;
	float area=0;
	float r = 0.5f*getBombWidth();
	b2CircleShape circle;
	b2PolygonShape rectangle;
	switch (settings->bombShape) {
	case CIRCLE:
		circle.m_radius = r;
		area= b2_pi*r*r;
		fd.shape = &circle;
		break;
	case RECTANGLE:
		rectangle.SetAsBox(r, r);
		area = 4.f*r*r;
		fd.shape = &rectangle;
		break;
	}
	fd.density = getBombMass()/area;
	fd.restitution = 0.0f;
	fd.friction = 0.05f; // ice or oil
	m_bomb->CreateFixture(&fd);
}

float Test::getBombMass()
{
	return settings->bombMass;
}
float Test::getBombWidth()
{
	return settings->bombWidth;
}

void Test::Step()
{
	float timeStep = settings->hz > 0.0f ? 1.0f / settings->hz : float(0.0f);
	if (!settings->pause &&
		settings->targetTime <= steppedTime + timeStep) {
		settings->pause = true;
	}
	if (settings->pause)
	{
		if (settings->singleStep)
		{
			settings->singleStep = 0;
		}
		else
		{
			timeStep = 0.0f;
		}

		g_debugDraw.DrawString(5, m_textLine, "****PAUSED****");
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	uint32 flags = 0;
	flags += settings->drawShapes			* b2Draw::e_shapeBit;
	flags += settings->drawJoints			* b2Draw::e_jointBit;
	flags += settings->drawAABBs			* b2Draw::e_aabbBit;
	flags += settings->drawCOMs				* b2Draw::e_centerOfMassBit;
	g_debugDraw.SetFlags(flags);
	b2Island::SetInitImpulses(settings->initImpulses);

	m_world->SetAllowSleeping(settings->enableSleep);
	m_world->SetWarmStarting(settings->enableWarmStarting);
	m_world->SetContinuousPhysics(settings->enableContinuous);
	m_world->SetSubStepping(settings->enableSubStepping);

	m_pointCount = 0;
	b2Vec2 gravity;
	gravity.Set(0.0f, -10.0f);
	if (steppedTime < settings->gravityRampUpTime) {
		float scale = steppedTime / settings->gravityRampUpTime;
		gravity = scale*gravity;
	}
	m_world->SetGravity(gravity);
#ifdef EP_LOG
	if (!settings->pause) {
		epLogActive = false;
	}
	else {
		epLogActive = true;
	}
	if (epLogActive && timeStep > 0.f) {
		epLog("Step from %g, timeStep=%g, gravity=%g\n",
			steppedTime, timeStep, gravity.y);
	}
#endif
	m_world->Step(timeStep, settings->velocityIterations, settings->positionIterations);
	for (b2Joint* j = m_world->GetJointList(); j;)
	{
		b2Joint* nj = j->GetNext();
		switch (j->GetType())
		{
		case e_rigidPlasticJoint:
		case e_elasticPlasticJoint:
		{
			b2ElasticPlasticJoint* ej = (b2ElasticPlasticJoint*)j;
			if (ej->WantsToBreak()){
				m_destructionListener.SayGoodbye(ej);
				m_world->DestroyJoint(ej);
			}
			break;
		}
		default:
			break;
		}
		j = nj;
	}

	if (timeStep > 0.f){
		for (SelectedEPJoint* j = GetSelectedJointList();
			j != nullptr; j = j->next) {
			UpdatePlotValues(j);
		}
	}
	UpdateCamera();
	m_world->DebugDraw();
    g_debugDraw.Flush();

	if (timeStep > 0.0f)
	{
		++m_stepCount;
		steppedTime += timeStep;
	}

	if (settings->drawStats)
	{
		int32 bodyCount = m_world->GetBodyCount();
		int32 contactCount = m_world->GetContactCount();
		int32 jointCount = m_world->GetJointCount();
		g_debugDraw.DrawString(5, m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
		m_textLine += DRAW_STRING_NEW_LINE;

		int32 proxyCount = m_world->GetProxyCount();
		int32 height = m_world->GetTreeHeight();
		int32 balance = m_world->GetTreeBalance();
		float quality = m_world->GetTreeQuality();
		g_debugDraw.DrawString(5, m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	// Track maximum profile times
	if (timeStep>0.0f)
	{
		const b2Profile& p = m_world->GetProfile();
		m_maxProfile.step = b2Max(m_maxProfile.step, p.step);
		m_maxProfile.collide = b2Max(m_maxProfile.collide, p.collide);
		m_maxProfile.solve = b2Max(m_maxProfile.solve, p.solve);
		m_maxProfile.solveInit = b2Max(m_maxProfile.solveInit, p.solveInit);
		m_maxProfile.initImpulse = b2Max(m_maxProfile.initImpulse, p.initImpulse);
		m_maxProfile.solveVelocity = b2Max(m_maxProfile.solveVelocity, p.solveVelocity);
		m_maxProfile.solvePosition = b2Max(m_maxProfile.solvePosition, p.solvePosition);
		m_maxProfile.solveTOI = b2Max(m_maxProfile.solveTOI, p.solveTOI);
		m_maxProfile.broadphase = b2Max(m_maxProfile.broadphase, p.broadphase);

		m_totalProfile.step += p.step;
		m_totalProfile.collide += p.collide;
		m_totalProfile.solve += p.solve;
		m_totalProfile.solveInit += p.solveInit;
		m_totalProfile.initImpulse += p.initImpulse;
		m_totalProfile.solveVelocity += p.solveVelocity;
		m_totalProfile.solvePosition += p.solvePosition;
		m_totalProfile.solveTOI += p.solveTOI;
		m_totalProfile.broadphase += p.broadphase;
	}

	if (settings->drawProfile)
	{
		const b2Profile& p = m_world->GetProfile();

		b2Profile aveProfile;
		memset(&aveProfile, 0, sizeof(b2Profile));
		if (m_stepCount > 0)
		{
			float scale = 1.0f / m_stepCount;
			aveProfile.step = scale * m_totalProfile.step;
			aveProfile.collide = scale * m_totalProfile.collide;
			aveProfile.solve = scale * m_totalProfile.solve;
			aveProfile.solveInit = scale * m_totalProfile.solveInit;
			aveProfile.initImpulse = scale * m_totalProfile.initImpulse;
			aveProfile.solveVelocity = scale * m_totalProfile.solveVelocity;
			aveProfile.solvePosition = scale * m_totalProfile.solvePosition;
			aveProfile.solveTOI = scale * m_totalProfile.solveTOI;
			aveProfile.broadphase = scale * m_totalProfile.broadphase;
		}

		g_debugDraw.DrawString(5, m_textLine, "step [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.step, aveProfile.step, m_maxProfile.step);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "collide [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.collide, aveProfile.collide, m_maxProfile.collide);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "solve [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solve, aveProfile.solve, m_maxProfile.solve);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "solve init [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveInit, aveProfile.solveInit, m_maxProfile.solveInit);
		m_textLine += DRAW_STRING_NEW_LINE;
		if (settings->initImpulses){
			g_debugDraw.DrawString(5, m_textLine, "init impulse [ave] (max) = %5.2f [%6.2f] (%6.2f)",
				p.initImpulse, aveProfile.initImpulse, m_maxProfile.initImpulse);
			m_textLine += DRAW_STRING_NEW_LINE;
		}
		g_debugDraw.DrawString(5, m_textLine, "solve velocity [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveVelocity, aveProfile.solveVelocity, m_maxProfile.solveVelocity);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "solve position [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solvePosition, aveProfile.solvePosition, m_maxProfile.solvePosition);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "solveTOI [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.solveTOI, aveProfile.solveTOI, m_maxProfile.solveTOI);
		m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, m_textLine, "broad-phase [ave] (max) = %5.2f [%6.2f] (%6.2f)", p.broadphase, aveProfile.broadphase, m_maxProfile.broadphase);
		m_textLine += DRAW_STRING_NEW_LINE;
	}

	if (m_mouseJoint)
	{
		b2Vec2 p1 = m_mouseJoint->GetAnchorB();
		b2Vec2 p2 = m_mouseJoint->GetTarget();

		b2Color c;
		c.Set(0.0f, 1.0f, 0.0f);
		g_debugDraw.DrawPoint(p1, 4.0f, c);
		g_debugDraw.DrawPoint(p2, 4.0f, c);

		c.Set(0.8f, 0.8f, 0.8f);
		g_debugDraw.DrawSegment(p1, p2, c);
	}
	b2Body* lb = loggedBody;
	if (lb == NULL && m_bomb != NULL) {
		lb = m_bomb;
	}
	// ep
	if (lb){
		const b2Vec2 p = lb->GetWorldPoint(b2Vec2(0, 0));
		b2Color c;
		c.Set(0.0f, 0.0f, 1.0f);
		g_debugDraw.DrawPoint(p, 4.0f, c);
		g_debugDraw.DrawString(5, m_textLine, "(x,y,a) = %6.2f %6.2f %6.2f",
			p.x, p.y, lb->GetAngle());
		m_textLine += DRAW_STRING_NEW_LINE;
		b2Vec2 v = lb->GetLinearVelocity();
		float m = lb->GetMass();
		float i = lb->GetInertia();
		float va = lb->GetAngularVelocity();
		float ke = 0.5f*(m*v.LengthSquared() + i*va*va);
		g_debugDraw.DrawString(5, m_textLine,
			"(vx,vy,va,m,i,ke) = %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
			v.x,v.y,va,m,i,ke);
		m_textLine += DRAW_STRING_NEW_LINE;
		if (pb == lb) {
			if (timeStep > 0.f) {
				float f = 1.f / timeStep;
				av = f*(v - pv);
				aa = f*(va - pva);
				validAcc = true;
			}
			if (validAcc) {
				g_debugDraw.DrawString(5, m_textLine,
					"(ax,ay,aa) = %6.2f %6.2f %6.2f",
					av.x, av.y, aa);
			}
			else {
				g_debugDraw.DrawString(5, m_textLine,
					"no validAcc");
			}
			m_textLine += DRAW_STRING_NEW_LINE;
		}
		if (timeStep > 0.f) {
			pv = v;
			pva = va;
			pb = lb;
		}
		min.x = b2Min(min.x, p.x);
		min.y = b2Min(min.y, p.y);
		min.z = b2Min(min.z, lb->GetAngle());
		g_debugDraw.DrawString(5, m_textLine, "min (x,y,a) = %6.2f %6.2f %6.2f",
			min.x, min.y, min.z);
		m_textLine += DRAW_STRING_NEW_LINE;
		max.x = b2Max(max.x, p.x);
		max.y = b2Max(max.y, p.y);
		max.z = b2Max(max.z, lb->GetAngle());
		g_debugDraw.DrawString(5, m_textLine, "max (x,y,a) = %6.2f %6.2f %6.2f",
			max.x, max.y, max.z);
		m_textLine += DRAW_STRING_NEW_LINE;
	}
	
	if (m_bombSpawning)
	{
		b2Color c;
		c.Set(0.0f, 0.0f, 1.0f);
		g_debugDraw.DrawPoint(m_bombSpawnPoint, 4.0f, c);

		c.Set(0.8f, 0.8f, 0.8f);
		g_debugDraw.DrawSegment(m_mouseWorld, m_bombSpawnPoint, c);
		b2Vec2 p = 0.5*(m_mouseWorld + m_bombSpawnPoint) + b2Vec2(0, g_camera.m_zoom);
		char buff[20];
		sprintf(buff, "(%6.2f,%6.2f)",m_bombVelocity.x,m_bombVelocity.y);
		g_debugDraw.DrawString(p,buff);
	}

	if (settings->drawContactPoints)
	{
		const float k_impulseScale = 0.1f;
		const float k_axisScale = 0.3f;

		for (int32 i = 0; i < m_pointCount; ++i)
		{
			ContactPoint* point = m_points + i;

			if (point->state == b2_addState)
			{
				// Add
				g_debugDraw.DrawPoint(point->position, 10.0f, b2Color(0.3f, 0.95f, 0.3f));
			}
			else if (point->state == b2_persistState)
			{
				// Persist
				g_debugDraw.DrawPoint(point->position, 5.0f, b2Color(0.3f, 0.3f, 0.95f));
			}

			if (settings->drawContactNormals == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_axisScale * point->normal;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.9f));
			}
			else if (settings->drawContactImpulse == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_impulseScale * point->normalImpulse * point->normal;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}

			if (settings->drawFrictionImpulse == 1)
			{
				b2Vec2 tangent = b2Cross(point->normal, 1.0f);
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = p1 + k_impulseScale * point->tangentImpulse * tangent;
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9f, 0.9f, 0.3f));
			}
		}
	}
}

void Test::LogSelectedJoints(float fScale, float mScale, 
	float* locs, const char * fmt, float maxValue, float minMaxValue)
{
	for (SelectedEPJoint* sj=GetSelectedJointList(); sj!=nullptr; sj=sj->next)
	{
		b2Joint* j = sj->joint;
		if (j != nullptr) {
			LogJoint(j, fScale, mScale, locs, fmt, maxValue, minMaxValue);
		}
	}

}

void Test::LogEpCapasityForSelectedJoints(float* locs)
{
	for (SelectedEPJoint* sj = GetSelectedJointList(); sj != nullptr; sj = sj->next)
	{
		b2Joint* j = sj->joint;
		if (j != nullptr) {
			LogEpCapasity((b2ElasticPlasticJoint*)j, locs);
		}
	}
}

void Test::LogEpJointErrorsForSelectedJoints(float* locs)
{
	for (SelectedEPJoint* sj = GetSelectedJointList(); sj != nullptr; sj = sj->next)
	{
		b2Joint* j = sj->joint;
		if (j != nullptr) {
			LogEpJointErrors((b2ElasticPlasticJoint*)j, locs);
		}
	}
}

void Test::UpdatePlotValues(SelectedEPJoint * epj)
{
	b2ElasticPlasticJoint* j = epj->joint;
	if (NULL == j || (epj->forces==NULL && epj->capacities==NULL)) {
		return;
	}
	float idt = g_debugDraw.GetInvDt();
	b2Vec2 f = j->GetReactionForce(idt);
	float m = j->GetReactionTorque(idt);
	b2Vec2 mf = j->GetRotatedMaxForce();
	epj->maxForce.x = b2Max(mf.x,epj->maxForce.x);
	epj->maxForce.y = b2Max(mf.y, epj->maxForce.y);
	float* values = epj->forces;
	if (NULL != values) {
		// move old values [1-9] to [0-8]
		for (int i = 0; i < 3; i++) {
			int index = i*EP_MAX_VALUES + 1;
			memmove(&values[index - 1], &values[index], (EP_MAX_VALUES - 1) * sizeof(float));
		}
		// store latest as last value
		values[EP_MAX_VALUES - 1] = f.x;
		values[2 * EP_MAX_VALUES - 1] = f.y;
		values[3 * EP_MAX_VALUES - 1] = m;
	}
	values = epj->capacities;
	if (NULL != values) {
		// store latest as last value
		for (int i = 0; i < 5; i++) {
			int index = i*EP_MAX_VALUES + 1;
			memmove(&values[index - 1], &values[index], (EP_MAX_VALUES - 1) * sizeof(float));
		}
		float mm = j->GetMaxTorque();
		if (mm == 0.f) {
			mm = 1.f;
		}
		values[EP_MAX_VALUES - 1] = b2Abs(100.f*f.x / mf.x);
		values[2 * EP_MAX_VALUES - 1] = b2Abs(100.f*f.y / mf.y);
		values[3 * EP_MAX_VALUES - 1] = b2Abs(100.f*m / mm);
		values[4 * EP_MAX_VALUES - 1] = 100.f*j->getCurrentStrain() / j->getMaxStrain();
		values[5 * EP_MAX_VALUES - 1] = 100.f*j->getCurrentRotation() / j->getMaxRotation();
	}
}

void Test::LogJoint(b2Joint* j, float fScale, float mScale, float* locs,
	const char * fmt, float maxValue, float minMaxValue) {
	b2Vec2 p = 0.5f*(j->GetAnchorA() + j->GetAnchorB());
	float idt = g_debugDraw.GetInvDt();
	b2Vec2 f = j->GetReactionForce(idt);
	float m = j->GetReactionTorque(idt);
	float va[3];
	va[0] = fScale*f.x;
	va[1] = fScale*f.y;
	va[2] = mScale*m;
	float largest = 0.0f;
	for (int i = 0; i < 3; i++) {
		float absValue = b2Abs(va[i]);
		if ( absValue> maxValue) {
			return;
		}
		largest = b2Max(largest, absValue);
	}
	if (largest < minMaxValue) {
		return;
	}
	if (IsSelectedJoint(j)) {
		StartTextHighLight();
	}
	ImGui::BeginGroup();
	ImGui::Text(fmt, va[0]); ImGui::SameLine(locs[0]);
	ImGui::Text(fmt, va[1]); ImGui::SameLine(locs[1]);
	ImGui::Text(fmt, va[2]); ImGui::SameLine(locs[2]);
	ImGui::Text("%4.1f",p.x); ImGui::SameLine(locs[3]);
	ImGui::Text("%4.1f",p.y);
	ImGui::EndGroup();
	if (IsSelectedJoint(j)) {
		EndTextHighLight();
	}
	if (ImGui::IsItemHovered()) {
		HighLightJoint(j);
	}
}

void Test::StartTextHighLight(){
	ImGui::PushStyleColor(ImGuiCol_Text, ImColor(1.f, 0.6f, 0.6f).Value);
}

void Test::EndTextHighLight() {
	ImGui::PopStyleColor();
}

void Test::HighLightJoint(b2Joint* j) {
	if (j == nullptr) {
		return;
	}
	b2Vec2 p = 0.5f*(j->GetAnchorA() + j->GetAnchorB());
	b2Vec2 bac = j->GetBodyA()->GetWorldCenter();
	b2Vec2 bbc = j->GetBodyB()->GetWorldCenter();
	float radius = (bbc - bac).Length();
	b2Color color(1.0f, 1.0f, 1.0f);
	g_debugDraw.DrawCircle(p, radius, color);
	radius *= 0.3f;
	g_debugDraw.DrawCircle(bac, radius, color);
	g_debugDraw.DrawString(bac, "A");
	g_debugDraw.DrawCircle(bbc, radius, color);
	g_debugDraw.DrawString(bbc, "B");
}

void Test::LogContact(ContactPoint * cp, float scale, float* locs,
	const char * fmt)
{
	b2Vec2 f,p=cp->position;
	float idt = g_debugDraw.GetInvDt();
	f = idt*(cp->normalImpulse*cp->normal + cp->tangentImpulse*cp->normal.Skew());
	ImGui::Text(fmt, scale*f.x); ImGui::SameLine(locs[0]);
	ImGui::Text(fmt, scale*f.y); ImGui::SameLine(locs[1]);
	ImGui::Text("%4.1f", p.x); ImGui::SameLine(locs[2]);
	ImGui::Text("%4.1f", p.y);
}


void Test::LogEpCapasity(b2ElasticPlasticJoint* j, float* locs){
	float idt = g_debugDraw.GetInvDt();
	b2Vec2 f = j->GetReactionForce(idt);
	b2Vec2 mf = j->GetMaxForce();
	float m = j->GetReactionTorque(idt);
	float mm = j->GetMaxTorque();
	if (mm == 0.f) {
		mm = 1.f;
	}
	bool isSelected = IsSelectedJoint(j);
	if (isSelected) {
		StartTextHighLight();
	}
	ImGui::BeginGroup();
	ImGui::Text("%3.0f", 100.f*f.x / mf.x); ImGui::SameLine(locs[0]);
	ImGui::Text("%3.0f", 100.f*f.y/mf.y); ImGui::SameLine(locs[1]);
	ImGui::Text("%3.0f", 100.f*m/mm); ImGui::SameLine(locs[2]);
	ImGui::Text("%3.0f", 100.f*j->getCurrentStrain()/j->getMaxStrain());
	ImGui::SameLine(locs[3]);
	ImGui::Text("%3.0f", 100.f*j->getCurrentRotation() / j->getMaxRotation());
	ImGui::EndGroup();
	if (isSelected) {
		EndTextHighLight();
	}
	if (ImGui::IsItemHovered()) {
		HighLightJoint(j);
	}
}

void Test::LogEpJointErrors(b2ElasticPlasticJoint * j, float* locs)
{
	bool isSelected = IsSelectedJoint(j);
	if (isSelected) {
		StartTextHighLight();
	}
	ImGui::BeginGroup();
	ImGui::Text("%.4f", j->getAngularError()); ImGui::SameLine(locs[0]);
	ImGui::Text("%.4f", j->getPositionError()); 
	ImGui::EndGroup();
	if (isSelected) {
		EndTextHighLight();
	}
	if (ImGui::IsItemHovered()) {
		HighLightJoint(j);
	}
}

void Test::ShiftOrigin(const b2Vec2& newOrigin)
{
	m_world->ShiftOrigin(newOrigin);
}
