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
// It is difficult to make a cantilever made of links completely rigid with weld joints.
// You will have to use a high number of iterations to make them stiff.
// So why not go ahead and use soft weld joints? They behave like a revolute
// joint with a rotational spring.
//
// Simo searches way to 
// * make iterations more efficient
// * how to automatically decompose object into appropriate pieces
//

enum BeamType {
	None=0,
	Cantilever,
	Axial,
	Elastic
};
namespace {
	BeamType beamType = None;
	int so_count;
	float baseHz;
	float baseDampingRatio;
	float density;
	float32 hx;
	float32 hy;
	float E; // GPa
	float fy; //MPa
	float maxRotation, maxStrain;
	bool addSoft, addHard, addElasticPlastic, firstIsHinge;
	float32 lastCx,lastCy;
}
class Beam : public Test
{
public:
	b2Vec2 noteWeld1, noteSoftWeld1, noteElasticPlastic1;
	virtual b2Body* getStaticBody(b2PolygonShape staticShape, float32 sy){
		b2FixtureDef sfd;
		sfd.shape = &staticShape;
		b2BodyDef sbd;
		sbd.type = b2_staticBody;
		sbd.position.Set(-hy, sy);
		b2Body* sbody = m_world->CreateBody(&sbd);
		sbody->CreateFixture(&sfd);
		return sbody;
	}
	virtual float getBombRadius(){
		return hy;
	}
	virtual float getBombDensity(){
		return density;
	}
	virtual bool isMyType();
	virtual void reset();
	virtual void build();
	Beam(Settings* sp) :Test(sp)
	{
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
				if (ImGui::CollapsingHeader("Settings", "BeamSettings"))
				{
					ImGui::Text("sub body count");
					ImGui::SliderInt("##sub body count", &so_count, 1, 50);
					ImGui::Text("sub body half length");
					ImGui::SliderFloat("##sub body half length", &hx, 0.1f, 30, "%.2f", 2.f);
					ImGui::Text("sub body half height");
					ImGui::SliderFloat("##sub body half height", &hy, 0.05f, 3, "%.2f");
					if (addSoft || addElasticPlastic){
						ImGui::Text("Frequency for soft joints");
						ImGui::SliderFloat("Hz##Hertz", &baseHz, 0.f, 60.f, "%.0f");
						ImGui::Text("DampingRatio for soft joints");
						ImGui::SliderFloat("##dratio", &baseDampingRatio, 0.f, 1.0f, "%.3f");
					}
					ImGui::Text("density");
					ImGui::SliderFloat("kg/m^3##density", &density, 1000.f, 20000.f, "%.0f");
					ImGui::Text("Elastic modulus");
					ImGui::SliderFloat("GPa##E", &E, 10.f, 1000.f, "%.0f");
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
					ImGui::Checkbox("ElasticPlastic", &addElasticPlastic);
					ImGui::Checkbox("FirstIsHinge", &firstIsHinge);
				}
				if (ImGui::CollapsingHeader("Joint forces MN/MNm"))
				{
					float locs[4] = { 40, 80, 120, 160 };
					ImGui::Text(" x-f"); ImGui::SameLine(locs[0]);
					ImGui::Text(" y-f"); ImGui::SameLine(locs[1]);
					ImGui::Text(" z-m"); ImGui::SameLine(locs[2]);
					ImGui::Text(" j-x"); ImGui::SameLine(locs[3]);
					ImGui::Text(" j-y");
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						LogJoint(j, 1e-6f, 1e-6f, locs);
					}
				}
				if (addElasticPlastic && ImGui::CollapsingHeader("Capasity usage [%]"))
				{
					float locs[4] = { 40, 80, 120, 160 };
					ImGui::Text(" x"); ImGui::SameLine(locs[0]);
					ImGui::Text(" y"); ImGui::SameLine(locs[1]);
					ImGui::Text(" z"); ImGui::SameLine(locs[2]);
					ImGui::Text(" s"); ImGui::SameLine(locs[3]);
					ImGui::Text(" r");
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						switch (j->GetType()){
						case e_elasticPlasticJoint:
							LogEpCapasity((b2ElasticPlasticJoint*)j, locs);
							break;
						}
					}
				}
			}
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
		if (addSoft){
			g_debugDraw.DrawString(noteSoftWeld1, "Soft WeldJoint");
		}
		if (addElasticPlastic){
			g_debugDraw.DrawString(noteElasticPlastic1, "ElasticPlasticJoint");
		}
	}
	static Test* Create(Settings *settings)
	{
		Beam* t = new Beam(settings);
		t->build();
		t->CreateRigidTriangles();
		return t;
	}

	b2Body* m_middle;
};

bool Beam::isMyType(){
	return beamType == Cantilever;
}

void Beam::reset(){
	so_count = 1;
	baseHz = 30;
	baseDampingRatio = 0.2f;
	density = 7800;
	hx = 10.f;
	hy = 1.f;
	fy = 350;
	E = 200;
	maxRotation = 3.f;
	maxStrain = 0.2f;
	addSoft = false;
	addHard = false;
	addElasticPlastic = true;
	firstIsHinge = false;
	beamType = Cantilever;
	settings->reset();
}

void Beam::build(){
	if (!isMyType()){
		reset();
	}
	float32 totalLength = so_count * 2 * hx + 2 * hy;
	b2Body* ground = NULL;
	{
		b2BodyDef bd;
		ground = m_world->CreateBody(&bd);

		b2EdgeShape shape;
		float32 floorLevel = 0;
		shape.Set(b2Vec2(-5 * hy - so_count*hx - 10, floorLevel),
			b2Vec2(so_count*hx * 4 + 5 * hy + 10, floorLevel));
		ground->CreateFixture(&shape, 0.0f);
	}
	b2PolygonShape staticShape;
	staticShape.SetAsBox(hy, hy);
	float32 sy = 0;
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
		jd.frequencyHz = baseHz;
		jd.dampingRatio = baseDampingRatio;

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
		sy += 5 + totalLength;
		b2ElasticPlasticJoint::resetEpId();
		b2Body* sbody = getStaticBody(staticShape, sy);

		b2PolygonShape shape;
		shape.SetAsBox(hx, hy);

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = density;

		b2ElasticPlasticJointDef jd;
		// select smaller, either axial or cutting
		jd.maxForce.x = b2Min(2 * hy,hx)*fy*1e6f;
		jd.maxForce.y = b2Min(2 * hx,hy)*fy*1e6f;
		jd.maxTorque = hy*hy* fy*1e6f;
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
	{ // rescale using m_zoom and m_center if dimensions change
		float32 cx = 2 * so_count*hx;
		float32 cy = sy / 2 + hy + 1;
		if (lastCy != cy || lastCx != cx){
			float32 margin = 2 * hy + 2;
			g_camera.m_center.Set(cx, cy);
			g_camera.m_zoom = 1.1f*(b2Max(cx, cy) + margin) / 25.f;
			lastCx = cx;
			lastCy = cy;
		}
		// keep notes above beams
		float32 noteMove = g_camera.m_zoom;
		if (addHard){
			noteWeld1.y += noteMove;
		}
		if (addSoft){
			noteSoftWeld1.y += noteMove;
		}
		if (addElasticPlastic){
			noteElasticPlastic1.y += noteMove;
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
		t->CreateRigidTriangles();
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
		t->CreateRigidTriangles();
		return t;
	}
};

bool ElasticBeam::isMyType(){
	return beamType == Elastic;
}

void ElasticBeam::reset(){
	Beam::reset();
	hx = 30.f;
	baseHz = 1.f;
	beamType = Elastic;
}

#endif
