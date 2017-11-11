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
* Simo Nikula based on Frame.h
*/

#ifndef FRAME_H
#define FRAME_H
#include <imgui/imgui.h>
#include "Test.h"
enum FrameType {
	NoFrame = 0,
	SimpleFrame,
};

namespace b2Frame {
	FrameType frameType;
	int so_count;
	float baseHz;
	float baseDampingRatio;
	float density;
	float32 hx;
	float32 fh,hy;
	float E; // GPa
	float fy; //MPa
	float maxRotation, maxStrain;
	float bombRadius, bombDensity;
	float32 lastCx, lastCy;
}
class Frame : public Test
{
public:
	int bodyCount;
	b2Body **bodies;
	virtual bool isMyType();
	virtual float getBombRadius() {
		return b2Frame::bombRadius;
	}
	virtual float getBombDensity() {
		return b2Frame::bombDensity;
	}
	virtual void reset();
	virtual void build();
	Frame(Settings* sp) :Test(sp)
	{
	}
	~Frame() {
		free(bodies);
	}
	bool showMenu = true;
	virtual void Ui() {
		int menuWidth = 220;
		if (showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2((float)g_camera.m_width - menuWidth - 200 - 10, 10));
			ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)g_camera.m_height - 20));
			if (ImGui::Begin("Frame Controls##Bean", &showMenu,
				ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
				if (ImGui::CollapsingHeader("Settings", "FrameSettings"))
				{
					ImGui::Text("sub body count");
					ImGui::SliderInt("##sub body count", &b2Frame::so_count, 1, 50);
					ImGui::Text("frame height");
					ImGui::SliderFloat("##frame height", &b2Frame::fh, 0.1f, 30, "%.2f", 2.f);
					ImGui::Text("sub body half height");
					ImGui::SliderFloat("##sub body half height", &b2Frame::hy, 0.01f, 0.1f, "%.2f");
					ImGui::Text("Frequency for soft joints");
					ImGui::SliderFloat("Hz##Hertz", &b2Frame::baseHz, 0.f, 60.f, "%.1f");
					if (b2Frame::baseHz > 0.f) {
						ImGui::Text("DampingRatio for soft joints");
						ImGui::SliderFloat("##dratio", &b2Frame::baseDampingRatio, 0.f, 1.0f, "%.3f");
					}
					ImGui::Text("frame density");
					ImGui::SliderFloat("kg/m^3##frameDensity", &b2Frame::density, 1000.f, 20000.f, "%.0f");
					ImGui::Text("bomb density");
					ImGui::SliderFloat("kg/m^3##bombDensity", &b2Frame::bombDensity, 1000.f, 20000.f, "%.0f");
					ImGui::Text("bomb radius");
					ImGui::SliderFloat("m##bombRadius", &b2Frame::bombRadius, 0.05f, 0.2f, "%.2f");
					// E is not currently used as elastic behaviour is based on frequency
					// ImGui::Text("Elastic modulus");
					// ImGui::SliderFloat("GPa##E", &b2Frame::E, 10.f, 1000.f, "%.0f");
					ImGui::Text("yield stress");
					ImGui::SliderFloat("MPa##fy", &b2Frame::fy, 10.f, 1000.f, "%.0f");
					ImGui::Text("maxRotation");
					ImGui::SliderFloat("radians##maxRotation",
						&b2Frame::maxRotation, 0.01f, 10.f, "%.2f");
					ImGui::Text("maxStrain");
					ImGui::SliderFloat("##maxStrain",
						&b2Frame::maxStrain, 0.01f, 10.f, "%.2f");
				}
				float locs[4] = { 40, 80, 120, 160 };
				if (ImGui::CollapsingHeader("Joint forces MN/MNm"))
				{
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
				else {
					LogSelectedJoints(1e-6f, 1e-6f, locs);
				}
				if (ImGui::CollapsingHeader("Capacity usage [%]"))
				{
					ImGui::Text(" x"); ImGui::SameLine(locs[0]);
					ImGui::Text(" y"); ImGui::SameLine(locs[1]);
					ImGui::Text(" z"); ImGui::SameLine(locs[2]);
					ImGui::Text(" s"); ImGui::SameLine(locs[3]);
					ImGui::Text(" r");
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						switch (j->GetType()) {
						case e_elasticPlasticJoint:
							LogEpCapasity((b2ElasticPlasticJoint*)j, locs);
							break;
						}
					}
				}
				else {
					LogEpCapasityForSelectedJoints(locs);
				}
				float jelocs[] = { 100 };
				if (ImGui::CollapsingHeader("Joint errors"))
				{
					ImGui::Text(" p"); ImGui::SameLine(jelocs[0]);
					ImGui::Text(" a"); 
					for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
					{
						switch (j->GetType()) {
						case e_elasticPlasticJoint:
							LogEpJointErrors((b2ElasticPlasticJoint*)j, jelocs);
							break;
						}
					}
				}
				else{
					LogEpJointErrorsForSelectedJoints(jelocs);
				}
			}
			ImGui::End();
		}
		if (settings->drawNotes) {
			drawNotes();
		}
	}
	virtual void drawNotes() {
	}
	static Test* Create(Settings *settings)
	{
		Frame* t = new Frame(settings);
		t->build();
		t->CreateRigidTriangles();
		return t;
	}

	b2Body* m_middle;
};


inline bool Frame::isMyType()
{
	return b2Frame::frameType==SimpleFrame;
}

void Frame::reset() {
	b2Frame::so_count = 4;
	b2Frame::baseHz = 0;
	b2Frame::baseDampingRatio = 0.2f;
	b2Frame::density = 7800;
	b2Frame::fh = 10.f;
	b2Frame::hy = 0.05f;
	b2Frame::fy = 350;
	b2Frame::E = 200; // not used
	b2Frame::maxRotation = 3.f;
	b2Frame::maxStrain = 0.2f;
	b2Frame::bombRadius = 0.1f;
	b2Frame::bombDensity = 7800;
	b2Frame::frameType = SimpleFrame;
	settings->reset();
}

void Frame::build() {
	if (!isMyType()) {
		reset();
	}
	int soc2 = b2Frame::so_count*b2Frame::so_count;
	b2Frame::hx = b2Frame::fh / 2 / soc2;
	float32 totalLength = b2Frame::fh;
	b2Body* ground = NULL;
	{
		int n = b2Frame::so_count;
		bodyCount = (2 * n + 1)*n*n;
		bodies = (b2Body **)b2Alloc(bodyCount * sizeof(b2Body*));
	}
	{
		b2BodyDef bd;
		ground = m_world->CreateBody(&bd);

		b2EdgeShape shape;
		float32 floorLevel = 0;
		shape.Set(b2Vec2(-totalLength, floorLevel),
			b2Vec2(2*totalLength, floorLevel));
		ground->CreateFixture(&shape, 0.0f);
	}
	b2PolygonShape staticShape;
	staticShape.SetAsBox(b2Frame::hy, b2Frame::hy);
	float32 sy = 0;
	b2RevoluteJointDef hd;
	b2PolygonShape shape;
	shape.SetAsBox(b2Frame::hx, b2Frame::hy);

	b2FixtureDef fd;
	fd.shape = &shape;
	fd.density = b2Frame::density;

	b2ElasticPlasticJointDef jd;
	// select smaller, either axial or cutting
	jd.maxForce.x = b2Min(2 * b2Frame::hy, b2Frame::hx)*b2Frame::fy*1e6f;
	jd.maxForce.y = b2Min(2 * b2Frame::hx, b2Frame::hy)*b2Frame::fy*1e6f;
	jd.maxTorque = b2Frame::hy*b2Frame::hy* b2Frame::fy*1e6f;
	jd.maxRotation = b2Frame::maxRotation;
	jd.maxStrain = b2Frame::maxStrain*b2Min(b2Frame::hx, b2Frame::hy);
	jd.frequencyHz = b2Frame::baseHz;
	jd.dampingRatio = b2Frame::baseDampingRatio;
	b2Body *prevBody = NULL,*body;
	int n = 0;
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	// verticals bodies
	bd.angle = b2_pi / 2;
	float32 sx;
	float32 step = b2Frame::so_count * 2 * b2Frame::hx;
	for (int32 j = 0; j < b2Frame::so_count; ++j) { // floors
		sx = 0;
		for (int32 i = 0; i <= b2Frame::so_count; ++i) // walls
		{
			prevBody = NULL;
			for (int32 k = 0; k < b2Frame::so_count; ++k) { 
				// subparts with joints
				bd.position.Set(sx, sy+(2 * k + 1)*b2Frame::hx);
				body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
				bodies[n++] = body;
				if (prevBody != NULL) {
					b2Vec2 anchor(sx,sy+2 * k*b2Frame::hx);
					jd.Initialize(prevBody, body, anchor);
					m_world->CreateJoint(&jd);
				}
				prevBody = body;
			}
			sx += step;
		}
		sy += step;
	}
	// horizontal bodies
	bd.angle = 0.f;
	sy = step;
	for (int32 j = 0; j < b2Frame::so_count; ++j) { // floors
		sx = 0;
		prevBody = NULL;
		for (int32 i = 0; i < b2Frame::so_count; ++i) // walls
		{
			for (int32 k = 0; k < b2Frame::so_count; ++k) {
				// subparts with joints
				bd.position.Set(sx + (2 * k + 1)*b2Frame::hx, sy );
				body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
				bodies[n++] = body;
				if (prevBody != NULL) {
					b2Vec2 anchor(sx+2 * k*b2Frame::hx, sy);
					jd.Initialize(prevBody, body, anchor);
					m_world->CreateJoint(&jd);
				}
				prevBody = body;
			}
			sx += step;
		}
		sy += step;
	}
	// joints for corners
	// vertical anchors joining floors or to ground
	sy = 0.f;
	for (int32 j = 0; j < b2Frame::so_count; ++j) { // floors
		sx = 0;
		for (int32 i = 0; i <=b2Frame::so_count; ++i) { // walls
			int ai=i*b2Frame::so_count+
				j*b2Frame::so_count*(b2Frame::so_count+1);
			int bi=ai-soc2-1;
			prevBody = bodies[ai];
			if (bi >= 0) {
				body = bodies[bi];
			}
			else {
				body = ground;
			}
			b2Vec2 anchor(sx, sy);
			jd.Initialize(prevBody, body, anchor);
			m_world->CreateJoint(&jd);
			sx += step;
		}
		sy += step;
	}
	// corner joints
	// a is vertical and b horizontal
	int vbc = soc2*(b2Frame::so_count+1); // vertical body count
	sy = step;
	for (int32 j = 0; j < b2Frame::so_count; ++j) { // floors
		sx = 0.f;
		for (int32 i = 0; i <=b2Frame::so_count; ++i) { // walls
			int ai = (i+1)*b2Frame::so_count-1 +
				j*b2Frame::so_count*(b2Frame::so_count + 1);
			int bi= ai + vbc - (j+1)*b2Frame::so_count + 1;
			if (i == b2Frame::so_count) {
				bi -= 1;
			}
			prevBody = bodies[ai];
			body = bodies[bi];
			b2Vec2 anchor(sx, sy);
			jd.Initialize(prevBody, body, anchor);
			m_world->CreateJoint(&jd);
			sx += step;
		}
		sy += step;
	}
	{ // rescale using m_zoom and m_center if dimensions change
		float32 cx = 2 * b2Frame::so_count*b2Frame::hx;
		float32 cy = sy / 2 + b2Frame::hy + 1;
		if (b2Frame::lastCy != cy || b2Frame::lastCx != cx) {
			float32 margin = 2 * b2Frame::hy + 2;
			g_camera.m_center.Set(cx, cy);
			g_camera.m_zoom = 1.1f*(b2Max(cx, cy) + margin) / 25.f;
			b2Frame::lastCx = cx;
			b2Frame::lastCy = cy;
		}
	}
}
#endif
