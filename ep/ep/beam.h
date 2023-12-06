/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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
* Simo Nikula based on Cantilever.h
*/

#ifndef BEAM_H
#define BEAM_H
#include <imgui/imgui.h>
#include "Test.h"
#include <box2d/b2ep_joint.h>
// It is difficult to make a cantilever made of links completely rigid with weld joints.
// You will have to use a high number of iterations to make them stiff.
// So why not go ahead and use soft weld joints? They behave like a revolute
// joint with a rotational spring.
//
// Simo searches way to 
// * stable processing
// * make iterations more efficient
// * how to automatically decompose object into appropriate pieces
//

enum BeamType {
	None=0,
	Cantilever,
	Axial,
	Elastic,
	Empty,
};
namespace {
	BeamType beamType = None;
	int so_count;
	float baseHz;
	float baseDampingRatio;
	float density;
	float hx;
	float hy;
	float fy; //MPa
	float maxElasticRotation, maxRotation, maxStrain;
	bool addSoft, addHard, addFriction,
		addElasticPlastic, addRigidPlastic,
		showElasticPlastic, firstIsHinge, horizontal;
	float lastCx,lastCy;
	bool openLists;
}

namespace bo {
	int b1,b2,b3,b4;
	void reset() {
		b1 =b2=b3=b4= 0;
	}
}


class Beam : public Test
{
public:
	b2Vec2 noteWeld1,
		noteFriction1,
		noteSoftWeld1, 
		noteElasticPlastic1, 
		noteRigidPlastic1;
	virtual b2Body* getStaticBody(b2PolygonShape staticShape, float sy){
		b2FixtureDef sfd;
		sfd.shape = &staticShape;
		b2BodyDef sbd;
		sbd.type = b2_staticBody;
		sbd.position.Set(-hy, sy);
		b2Body* sbody = m_world->CreateBody(&sbd);
		sbody->CreateFixture(&sfd);
		return sbody;
	}
	bool showControls() {
		switch (beamType) {
		case Empty:
			return false;
		}
		return true;
	}
	virtual bool isMyType();
	virtual void reset();
	virtual void build();
	virtual void BeamExtraUi();
	virtual bool OpenEPJoints() { return (bo::b1||bo::b2||bo::b3); }
	virtual float getFloorMinX(), getFloorMaxX();
	Beam(Settings* sp) :Test(sp)
	{
		createDynamicItems();
	}
	void createDynamicItems() {
		if (!isMyType()) {
			return;
		}
		if (bo::b1) {
			m_bombSpawnPoint = settings->bombSpawnPoint;
			LaunchBomb();
		}
		if (bo::b4) {
			settings->addMassPoint = b2Vec2(so_count*hx, so_count*hx*2+4*hy+5);
			AddMass(settings->addMassPoint);
		}
	}
	bool showMenu = true;
	virtual void Ui(){
		int menuWidth = 220;
		if (showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2((float)g_camera.m_width - menuWidth - 200 - 10, 10));
			ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)g_camera.m_height - 20));
			if (ImGui::Begin("Beam Controls##Bean", &showMenu,
				ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)){
				if (showControls() && ImGui::CollapsingHeader("Settings", 0))
				{
					ImGui::Text("sub body count");
					ImGui::SliderInt("##sub body count", &so_count, 1, 50);
					ImGui::Text("sub body half length");
					ImGui::SliderFloat("##sub body half length", &hx, 0.1f, 30, "%.2f", 2.f);
					ImGui::Text("sub body half height");
					ImGui::SliderFloat("##sub body half height", &hy, 0.05f, 3, "%.2f");
					if (addSoft || addElasticPlastic){
						ImGui::Text("Frequency for soft joints");
						ImGui::SliderFloat("Hz##Hertz", &baseHz, 0.f, 10.f, "%.2f");
						if (baseHz > 0.f) {
							ImGui::Text("DampingRatio for soft joints");
							ImGui::SliderFloat("##dratio", &baseDampingRatio, 0.f, 1.0f, "%.3f");
						}
					}
					ImGui::Text("density");
					ImGui::SliderFloat("kg/m^3##density", &density, 1000.f, 20000.f, "%.0f");
					if (addElasticPlastic && baseHz>0) {
						ImGui::Text("max elastic rotation");
						ImGui::SliderFloat("radians##maxElasticRotation", 
							&maxElasticRotation, 0.f, 1.f, "%.2f");
					}
					ImGui::Text("yield stress");
					ImGui::SliderFloat("MPa##fy", &fy, 10.f, 1000.f, "%.0f");
					ImGui::Text("maxRotation");
					ImGui::SliderFloat("radians##maxRotation",
						&maxRotation, 0.01f, 10.f, "%.2f");
					ImGui::Text("maxStrain");
					ImGui::SliderFloat("##maxStrain",
						&maxStrain, 0.01f, 10.f, "%.2f");
					ImGui::Separator();
					ImGui::Checkbox("Soft", &addSoft);
					ImGui::SameLine();
					ImGui::Checkbox("Hard", &addHard);
					ImGui::SameLine();
					ImGui::Checkbox("Friction", &addFriction);
					ImGui::Checkbox("ElasticPlastic", &addElasticPlastic);
					ImGui::SameLine();
					ImGui::Checkbox("RigidPlastic", &addRigidPlastic);
					ImGui::Checkbox("FirstIsHinge", &firstIsHinge);
					ImGui::SameLine();
					ImGui::Checkbox("Horizontal", &horizontal);
					if (ImGui::IsItemHovered() && !horizontal) {
						ImGui::SetTooltip("Vertical");
					}
				}
				float locs[4] = { 40, 80, 120, 160 };
				if (ImGui::CollapsingHeader("Joint forces MN/MNm", true))
				{
					ImGui::Text(" x-f"); ImGui::SameLine(locs[0]);
					ImGui::Text(" y-f"); ImGui::SameLine(locs[1]);
					ImGui::Text(" z-m"); ImGui::SameLine(locs[2]);
					ImGui::Text(" j-x"); ImGui::SameLine(locs[3]);
					ImGui::Text(" j-y");
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						LogJoint(j, 1e-6f, 1e-6f, locs,"%.0f");
					}
				}
				else {
					LogSelectedJoints(1e-6f, 1e-6f, locs);
				}
				if (ImGui::CollapsingHeader("Contact forces MN"))
				{
					float slocs[] = { 60, 120, 160 };
					ImGui::Text(" x-f"); ImGui::SameLine(slocs[0]);
					ImGui::Text(" y-f"); ImGui::SameLine(slocs[1]);
					ImGui::Text(" c-x"); ImGui::SameLine(slocs[2]);
					ImGui::Text(" c-y");
					for (int i = 0; i < m_pointCount; i++) {
						LogContact(m_points + i, 1e-6f, slocs, "%.2f");
					}
				}
				if (showElasticPlastic && ImGui::CollapsingHeader
					("Capacity usage [%]", 0, true))
				{
					ImGui::Text(" x"); ImGui::SameLine(locs[0]);
					ImGui::Text(" y"); ImGui::SameLine(locs[1]);
					ImGui::Text(" z"); ImGui::SameLine(locs[2]);
					ImGui::Text(" s"); ImGui::SameLine(locs[3]);
					ImGui::Text(" r");
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						switch (j->GetType()){
						case e_elasticPlasticJoint:
						case e_rigidPlasticJoint:
							LogEpCapasity((b2ElasticPlasticJoint*)j, locs);
							break;
						}
					}
				}
				else{
					LogEpCapasityForSelectedJoints(locs);
				}
				float jelocs[] = { 100 };
				if (ImGui::CollapsingHeader("Joint errors", 0, true))
				{
					ImGui::Text(" p"); ImGui::SameLine(jelocs[0]);
					ImGui::Text(" a");
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						switch (j->GetType()) {
						case e_elasticPlasticJoint:
						case e_rigidPlasticJoint:
							LogEpJointErrors((b2ElasticPlasticJoint*)j, jelocs);
							break;
						}
					}
				}
				else {
					LogEpJointErrorsForSelectedJoints(jelocs);
				}
			}
			BeamExtraUi();
			ImGui::End();
		}
		if (settings->drawNotes){
			drawNotes();
		}
	}
	virtual void drawNotes(){
		if (addHard){
			g_debugDraw.DrawString(noteWeld1, "WeldJoint");
		}
		if (addFriction) {
			g_debugDraw.DrawString(noteFriction1, "FrictionJoint");
		}
		if (addSoft){
			g_debugDraw.DrawString(noteSoftWeld1, "Soft WeldJoint");
		}
		if (addElasticPlastic){
			g_debugDraw.DrawString(noteElasticPlastic1, "ElasticPlasticJoint");
		}
		if (addRigidPlastic) {
			g_debugDraw.DrawString(noteRigidPlastic1, "RigidPlasticJoint");
		}
	}
	static Test* Create(Settings *settings)
	{
		Beam* t = new Beam(settings);
		if (t->OpenEPJoints()) {
			t->DeleteSelectedJoints();;
		}
		t->build();
		t->CommonEpInit();
		return t;
	}

	b2Body* m_middle;
};

bool Beam::isMyType(){
	return beamType == Cantilever;
}

void Beam::BeamExtraUi()
{
	if (ImGui::CollapsingHeader("BeamOptions", 0, true)) {
		float uihx=0.f,by=0.f;
		if (ImGui::Button("None")) {
			bo::reset();
		}
		if (ImGui::RadioButton("B1-1", &bo::b1,1)) {
			bo::reset();
			bo::b1 = 1;
			uihx = 10;
			by = 27.01f;
			settings->epDebugSteps = 10;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
density=7800\n\
hz=0\n\
10000000 kg mass\n\
hx=10\n\
");
		}
		ImGui::SameLine();
		if (ImGui::RadioButton("B1-2", &bo::b1,2)) {
			bo::reset();
			bo::b1 = 2;
			uihx = 3;
			by = 13.01f;
			settings->epDebugSteps = 10;
			settings->initImpulses = false;
			epLogEnabled = true;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
density=7800\n\
hz=0\n\
10000000 kg mass\n\
hx=3\n\
");
		}
		if (bo::b1 && uihx!=0.f) {
			settings->pause = true;
			hx = uihx;
			so_count = 1;
			baseHz = 0.f;
			settings->gravityRampUpTime = 0.f;
			settings->bombShape = RECTANGLE;
			settings->bombWidth = 1.f;
			settings->bombMass = 10e6f;
			settings->bombSpawnPoint = b2Vec2
			(0.5f*settings->bombWidth + 0.02f,
				by + 0.5f*settings->bombWidth + hy);
			settings->bombVelocity = b2Vec2(0, 0);
			density = 7800.f;
			horizontal = true;
			addSoft = false;
			addHard = false;
			addFriction = false;
			addElasticPlastic = true;
			firstIsHinge = false;
			openLists = true;
			restartPending = true;
		}
		if (ImGui::RadioButton("B2-1", &bo::b2, 1)) {
			bo::reset();
			bo::b2 = 1;
			uihx = 1.f;
			epLogEnabled = true;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
density=7800\n\
hz=0\n\
hx=1\n\
10 bodies\n\
");
		}
		if (bo::b2 && uihx!=0.f) {
			settings->pause = true;
			hx = uihx;
			baseHz = 0.f;
			density = 7800.f;
			so_count = 2;
			horizontal = true;
			addSoft = false;
			addHard = false;
			addFriction = false;
			addElasticPlastic = true;
			firstIsHinge = false;
			openLists = true;
			restartPending = true;
		}
		if (ImGui::RadioButton("B3-1", &bo::b3, 1)) {
			bo::reset();
			bo::b3 = 1;
			uihx = 40.f;
			epLogEnabled = true;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
density=7800\n\
rigidPlastic\n\
hx=1\n\
one 80 m body\n\
epDebug and log active\n\
");
		}
		ImGui::SameLine();
		if (ImGui::RadioButton("B3-11", &bo::b3, 11)) {
			bo::reset();
			bo::b3 =11;
			uihx = 40.f;
			epLogEnabled = true;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
density=7800\n\
elasticPlastic 10Hz\n\
hx=1\n\
one 80 m body\n\
epDebug and log active\n\
");
		}
		ImGui::SameLine();
		if (ImGui::RadioButton("B3-2", &bo::b3, 2)) {
			bo::reset();
			bo::b3 = 2;
			uihx = 10.f;
			epLogEnabled = false;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
density=7800\n\
rigidPlastic\n\
hx=1\n\
4 10 m bodies\n\
epDebug and log not active\n\
");
		}
		if (bo::b3 && uihx != 0.f) {
			settings->pause = true;
			hx = uihx;
			density = 7800.f;
			so_count = (int)(40.f/uihx);
			horizontal = true;
			addSoft = false;
			addHard = false;
			addFriction = false;
			switch (bo::b3) {
			case 11:
				addElasticPlastic = true;
				addRigidPlastic = false;
				baseHz = 10;
				break;
			default:
				addElasticPlastic = false;
				addRigidPlastic = true;
			}
			firstIsHinge = false;
			openLists = true;
			restartPending = true;
		}
		if (ImGui::RadioButton("B4-1", &bo::b4, 1)) {
			bo::reset();
			bo::b4 = 1;
			uihx = 10.f;
			epLogEnabled = true;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Horizontal\n\
rigidPlastic\n\
hx=10\n\
1 10 m body\n\
epLog active\n\
addMass=500 000\n\
addMassPoint=(10,29)\n\
");
		}
		if (bo::b4 && uihx != 0.f) {
			settings->pause = true;
			hx = uihx;
			so_count =1;
			horizontal = true;
			addSoft = false;
			addHard = false;
			addFriction = false;
			addElasticPlastic = false;
			addRigidPlastic = true;
			firstIsHinge = false;
			openLists = true;
			restartPending = true;
			settings->addMass = 500000;
		}
	}
}

void Beam::reset(){
	so_count = 1;
	baseHz = 2;
	baseDampingRatio = 0.2f;
	density = 7800;
	hx = 10.f;
	hy = 1.f;
	fy = 350;
	maxRotation = 3.f;
	maxElasticRotation = 0.f;
	maxStrain = 0.2f;
	addSoft = false;
	addHard = false;
	addFriction = false;
	addElasticPlastic = true;
	firstIsHinge = false;
	horizontal = false;
	openLists = false;
	beamType = Cantilever;
	settings->reset();
}

float Beam::getFloorMinX() {
	return -5 * hy - so_count*hx - 10;
}

float Beam::getFloorMaxX() {
	return so_count*hx * 4 + 5 * hy + 10;
}

void Beam::build(){
	if (!isMyType()){
		reset();
	}
	float totalLength = so_count * 2 * hx + 2 * hy;
	b2Body* ground = NULL;
	{
		b2BodyDef bd;
		ground = m_world->CreateBody(&bd);

		b2EdgeShape shape;
		float floorLevel = 0;
		shape.SetTwoSided(b2Vec2(getFloorMinX(), floorLevel),
			b2Vec2(getFloorMaxX(), floorLevel));
		ground->CreateFixture(&shape, 0.0f);
	}
	b2PolygonShape staticShape;
	staticShape.SetAsBox(hy, hy);
	float sy = 0;
	b2RevoluteJointDef hd;
	if (addHard)
	{
		sy += 5 + totalLength;
		b2Body* sbody = getStaticBody(staticShape, sy);

		b2PolygonShape shape;
		shape.SetAsBox(hx, hy);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = density;

		b2WeldJointDef jd;

		b2Body* prevBody = sbody;
		noteWeld1.Set(-hy, sy + 2 + hy);
		for (int32 i = 0; i < so_count; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set((2 * i + 1)*hx, sy);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);
			b2Vec2 anchor((2 * i)*hx, sy);
			if (i == 0 && firstIsHinge){
				hd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&hd);
			}
			else{
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);
			}
			prevBody = body;
		}
	}
	if (addFriction)
	{
		sy += 5 + totalLength;
		b2Body* sbody = getStaticBody(staticShape, sy);

		b2PolygonShape shape;
		shape.SetAsBox(hx, hy);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = density;

		b2FrictionJointDef jd;
		// select smaller, either axial or cutting
		if (horizontal) {
			jd.maxForce = b2Min(2 * hy, hx)*fy*1e6f;
		}
		else {
			jd.maxForce = b2Min(2 * hx, hy)*fy*1e6f;
		}
		jd.maxTorque = hy * hy* fy*1e6f;

		b2Body* prevBody = sbody;
		noteFriction1.Set(-hy, sy + 2 + hy);
		for (int32 i = 0; i < so_count; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set((2 * i + 1)*hx, sy);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);
			b2Vec2 anchor((2 * i)*hx, sy);
			if (i == 0 && firstIsHinge) {
				hd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&hd);
			}
			else {
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);
			}
			prevBody = body;
		}
	}
	if (addSoft)
	{
		sy += 5 + totalLength;
		b2Body* sbody = getStaticBody(staticShape, sy);

		b2PolygonShape shape;
		shape.SetAsBox(hx, hy);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = density;

		b2WeldJointDef jd;

		b2Body* prevBody = sbody;
		noteSoftWeld1.Set(-hy, sy + 2 + hy);
		for (int32 i = 0; i < so_count; ++i)
		{
			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set((2 * i + 1)*hx, sy);
			b2Body* body = m_world->CreateBody(&bd);
			body->CreateFixture(&fd);
			b2Vec2 anchor((2 * i)*hx, sy);
			if (i == 0 && firstIsHinge){
				hd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&hd);
			}
			else{
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);
			}
			prevBody = body;
		}
	}
if (addElasticPlastic)
{
	showElasticPlastic = true;
	sy += 5 + totalLength;
	b2Body* sbody = getStaticBody(staticShape, sy);

	b2PolygonShape shape;
	if (horizontal) {
		shape.SetAsBox(hx, hy);
	}
	else {
		shape.SetAsBox(hy, hx);
	}

	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = density;

	b2ElasticPlasticJointDef jd;
	// select smaller, either axial or cutting
	if (horizontal) {
		jd.maxForce.x = b2Min(2 * hy, hx)*fy*1e6f;
		jd.maxForce.y = b2Min(2 * hx, hy)*fy*1e6f;
	}
	else {
		jd.maxForce.x = b2Min(2 * hx, hy)*fy*1e6f;
		jd.maxForce.y = b2Min(2 * hy, hx)*fy*1e6f;
	}
	jd.maxTorque = hy*hy* fy*1e6f;
	jd.maxElasticRotation = maxElasticRotation;
	jd.maxRotation = maxRotation;
	jd.maxStrain = maxStrain*b2Min(hx, hy);
	jd.frequencyHz = baseHz;
	jd.dampingRatio = baseDampingRatio;

	b2Body* prevBody = sbody;
	noteElasticPlastic1.Set(-hy, sy + 2 + hy);
	for (int32 i = 0; i < so_count; ++i)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		float x, y;
		if (horizontal) {
			x = (2 * i + 1)*hx;
			y = sy;
		}
		else {
			x = -hy;
			y = sy - hy - (2 * i + 1)*hx;
		}
		bd.position.Set(x, y);
		b2Body* body = m_world->CreateBody(&bd);
		body->CreateFixture(&fd);
		float ax, ay;
		if (horizontal) {
			ax = 2 * i*hx;
			ay = sy;
		}
		else {
			ax = -hy;
			ay = sy - hy - 2 * i*hx;
		}
		b2Vec2 anchor(ax, ay);
		b2ElasticPlasticJoint* joint = NULL;
		if (i == 0 && firstIsHinge) {
			hd.Initialize(prevBody, body, anchor);
			m_world->CreateJoint(&hd);
		}
		else {
			jd.Initialize(prevBody, body, anchor);
			joint=(b2ElasticPlasticJoint*)m_world->CreateJoint(&jd);
		}
		if (bo::b1) {
			if (prevBody == sbody && joint) {
				loggedBody = body;
				SelectedEPJoint *sp = AddSelectedJoint(joint);
				sp->StartDebug();
			}
		}
		else if (bo::b2||bo::b3) {
			SelectedEPJoint *sp = AddSelectedJoint(joint);
			sp->StartDebug();
		}
		prevBody = body;
	}
}
if (addRigidPlastic)
{
	showElasticPlastic = true;
	sy += 5 + totalLength;
	b2Body* sbody = getStaticBody(staticShape, sy);
	b2PolygonShape shape;
	if (horizontal) {
		shape.SetAsBox(hx, hy);
	}
	else {
		shape.SetAsBox(hy, hx);
	}
	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = density;
	b2RigidPlasticJointDef jd;
	// select smaller, either axial or cutting
	if (horizontal) {
		jd.maxForce.x = b2Min(2 * hy, hx)*fy*1e6f;
		jd.maxForce.y = b2Min(2 * hx, hy)*fy*1e6f;
	}
	else {
		jd.maxForce.x = b2Min(2 * hx, hy)*fy*1e6f;
		jd.maxForce.y = b2Min(2 * hy, hx)*fy*1e6f;
	}
	jd.maxTorque = hy * hy* fy*1e6f;
	jd.maxElasticRotation = maxElasticRotation;
	jd.maxRotation = maxRotation;
	jd.maxStrain = maxStrain * b2Min(hx, hy);
	jd.frequencyHz = baseHz;
	jd.dampingRatio = baseDampingRatio;
	b2Body* prevBody = sbody;
	noteRigidPlastic1.Set(-hy, sy + 2 + hy);
	for (int32 i = 0; i < so_count; ++i)
	{
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		float x, y;
		if (horizontal) {
			x = (2 * i + 1)*hx;
			y = sy;
		}
		else {
			x = -hy;
			y = sy - hy - (2 * i + 1)*hx;
		}
		bd.position.Set(x, y);
		b2Body* body = m_world->CreateBody(&bd);
		body->CreateFixture(&fd);
		float ax, ay;
		if (horizontal) {
			ax = 0.f;
			ay = sy;
		}
		else {
			ax = -hy;
			ay = sy - hy;
		}
		b2Vec2 anchor(ax, ay);
		b2RigidPlasticJoint* joint = NULL;
		if (i == 0 && firstIsHinge) {
			hd.Initialize(prevBody, body, anchor);
			m_world->CreateJoint(&hd);
		}
		else {
			jd.Initialize(prevBody, body, anchor);
			joint = (b2RigidPlasticJoint*)m_world->CreateJoint(&jd);
		}
		if (bo::b1) {
			if (prevBody == sbody && joint) {
				loggedBody = body;
				SelectedEPJoint *sp = AddSelectedJoint(joint);
				sp->StartDebug();
			}
		}
		else if (bo::b2||bo::b3) {
			SelectedEPJoint *sp = AddSelectedJoint(joint);
			switch(bo::b3){
			case 1:
				sp->StartDebug();
			}
		}
	}
}
{ // rescale using m_zoom and m_center if dimensions change
	float cx = 2 * so_count*hx;
	float cy = sy / 2 + hy + 1;
	if (lastCy != cy || lastCx != cx) {
		float margin = 2 * hy + 2;
		g_camera.m_center.Set(cx, cy);
		g_camera.m_zoom = 1.1f*(b2Max(cx, cy) + margin) / 25.f;
		lastCx = cx;
		lastCy = cy;
	}
	// keep notes above beams
	float noteMove = g_camera.m_zoom;
	if (addHard) {
		noteWeld1.y += noteMove;
	}
	if (addFriction) {
		noteFriction1.y += noteMove;
	}
	if (addSoft) {
		noteSoftWeld1.y += noteMove;
	}
	if (addElasticPlastic) {
		noteElasticPlastic1.y += noteMove;
	}
	if (addRigidPlastic) {
		noteRigidPlastic1.y += noteMove;
	}
}
}


class AxialBeam : public Beam
{
public:
	AxialBeam(Settings *settings):Beam(settings){
	}
	~AxialBeam(){
	}
	virtual void reset();
	virtual bool isMyType();
	static Test* Create(Settings *settings)
	{
		Beam* t = new AxialBeam(settings);
		t->build();
		t->CommonEpInit();
		return t;
	}
};

bool AxialBeam::isMyType(){
	return beamType == Axial;
}

void AxialBeam::reset(){
	Beam::reset();
	hx = 1.f;
	fy = 10;
	beamType = Axial;
	settings->mouseJointForceScale = 10000;
	settings->forceScale = 10;
}

class ElasticBeam : public Beam
{
public:
	ElasticBeam(Settings *settings) :Beam(settings){
	}
	~ElasticBeam(){
	}
	virtual void reset();
	virtual bool isMyType();
	static Test* Create(Settings *settings)
	{
		Beam* t = new ElasticBeam(settings);
		t->build();
		t->CommonEpInit();
		return t;
	}
};

bool ElasticBeam::isMyType() {
	return beamType == Elastic;
}

void ElasticBeam::reset() {
	Beam::reset();
	hx = 30.f;
	baseHz = 1.f;
	beamType = Elastic;
}

namespace ebo {
	bool cs1, cs2, cs3; // for ui checkboxes
}

class EmptyBeam : public Beam
{
public:
	EmptyBeam(Settings *settings) :Beam(settings) {
		createDynamicItems();
	}
	~EmptyBeam() {
		DeleteEPBeams();
		settings->pause = false;
	}
	virtual void reset();
	virtual bool isMyType();
	virtual void BeamExtraUi();
	virtual bool OpenEPJoints() { return true; }
	virtual float getFloorMinX(), getFloorMaxX();
	static Test* Create(Settings *settings)
	{
		Beam* t = new EmptyBeam(settings);
		t->build();
		return t;
	}
	void createDynamicItems() {
		if (ebo::cs1) {
			EPBeam *epb = GetLastEPBeam();
			float hxb = settings->epbX / 2;
			if (NULL == epb->body) {
				b2Vec2 p = b2Vec2(hxb + 0.01f, 0);
				epb=AddEPBeam(p);
			}
			loggedBody = epb->body;
			settings->mouseJointForceScale = 300.f;
			settings->bombMass = 200;
			settings->addMass = 10 * settings->bombMass;
			settings->epbMaxForce = 275 / 2 * 1530; // N/mm^2, mm^2 RHS 100x100x4 
			settings->epbMaxMoment = 275 * 54900 / 1000; // N/mm^2, mm^3, /1000 to get Nm
			settings->pause = true;
			settings->gravityRampUpTime = 0.f;
			float br = 0.5f*settings->bombWidth;
			if (hxb>0.7f*br) {
				epb->translate(b2Vec2(0, -hxb));
			}
			settings->addMassSize = 5 * br;
			b2Vec2 p = b2Vec2(-(1.5f * settings->bombWidth + settings->addMassSize),
				settings->addMassSize / 2);
			AddMass(p);
			p.x = settings->epbX + settings->epbY + settings->addMassSize;
			AddMass(p);
			/** update view if bombSpawnPoint is modified */
			if (settings->bombSpawnPoint.y != br) {
				g_camera.m_zoom = 0.25f*br;
				g_camera.m_center = b2Vec2(0, br);
			}
			settings->bombSpawnPoint = b2Vec2(-br, br);
			m_bombSpawnPoint = settings->bombSpawnPoint;
			settings->epbMass = 12 * settings->epbY; // 12 kg/m
			SelectedEPJoint *sp=AddSelectedJoint(epb->joint);
			sp->StartDebug();
			epb->position[0] = settings->epbX / 2 + 0.01f;
			LaunchBomb();
		}
	}
};

float EmptyBeam::getFloorMinX() {
	return -(1.5f * settings->bombWidth + 2*settings->addMassSize);
}

float EmptyBeam::getFloorMaxX() {
	return 2*(settings->epbX+settings->epbY+ settings->addMassSize);
}

void EmptyBeam::reset()
{
	addElasticPlastic = false;
	addHard = false;
	addFriction = false;
	addSoft = false;
	showElasticPlastic = true;
	openLists = true;
	g_camera.m_center = b2Vec2(0,settings->epbY/2);
	beamType = Empty;
}

inline bool EmptyBeam::isMyType()
{
	return beamType == Empty;
}


void EmptyBeam::BeamExtraUi()
{
	if (ImGui::CollapsingHeader("EmptyBeamOptions",0,true)) {
		if (ImGui::Checkbox("CS1", &ebo::cs1)) {
			settings->bombShape = CIRCLE;
			settings->bombVelocity = b2Vec2(5, 0);
			restartPending = true;
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("AddEPBeams\n\
mjForceScale=300\n\
bomb=200kg circle, 5 m/s\n\
zoom=0.25\n\
pause=true");
		}
		ImGui::SameLine();
	}
}



#endif
