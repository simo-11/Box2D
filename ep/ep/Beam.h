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
// It is difficult to make a cantilever made of links completely rigid with weld joints.
// You will have to use a high number of iterations to make them stiff.
// So why not go ahead and use soft weld joints? They behave like a revolute
// joint with a rotational spring.
//
// Simo searches way to 
// * make iterations more efficient
// * how to automatically decompose object into appropriate pieces
//

namespace {
	int so_count;
	float baseHz;
	float baseDampingRatio;
	float density;
	float32 hx;
	float32 hy;
	float fy; //MPa
	bool addSoft, addHard, addParts, addMotor, addElasticPlastic;
}
class Beam : public Test
{
public:
	b2Vec2 noteWeld1, noteSoftWeld1, noteMotor1, noteElasticPlastic1;
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
		return hx;
	}
	virtual float getBombDensity(){
		return density;
	}
	virtual void reset(){
		so_count = 10;
		baseHz = 30;
		baseDampingRatio = 0.2f;
		density = 7800;
		hx = 1.f;
		hy = 1.f;
		fy = 350;
		addSoft = false;
		addHard = false;
		addMotor = false;
		addElasticPlastic = true;
		addParts = false;
	}
	Beam()
	{
		if (so_count < 1){
			reset(); // initial run
		}
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;
			float32 floorLevel = -so_count*2*hx;
			shape.Set(b2Vec2(-5*hy, floorLevel), b2Vec2(so_count*hx*2+5*hy, floorLevel));
			ground->CreateFixture(&shape, 0.0f);
		}
		b2PolygonShape staticShape;
		staticShape.SetAsBox(hy, hy);
		if (addHard)
		{
			float32 sy = 5;
			b2Body* sbody = getStaticBody(staticShape, sy);

			b2PolygonShape shape;
			shape.SetAsBox(hx, hy);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = density;

			b2WeldJointDef jd;

			b2Body* prevBody = sbody;
			noteWeld1.Set(-hy,sy+2+hy);
			for (int32 i = 0; i < so_count; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set((2*i+1)*hx, sy);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
				b2Vec2 anchor((2 * i)*hx, sy);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);
				prevBody = body;
			}
		}
		if (addSoft)
		{
			float32 sy = 15;
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
			noteWeld1.Set(-hy, sy + 2 + hy);
			for (int32 i = 0; i < so_count; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set((2 * i + 1)*hx, sy);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
				b2Vec2 anchor((2 * i)*hx, sy);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);
				prevBody = body;
			}
		}
		if (addMotor)
		{
			float32 sy = 25;
			b2Body* sbody = getStaticBody(staticShape, sy);

			b2PolygonShape shape;
			shape.SetAsBox(hx, hy);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = density;

			b2MotorJointDef jd;
			jd.maxForce = 2 * hy*fy*1e6f;
			jd.maxTorque = hy*hy / 4 * fy*1e6f;

			b2Body* prevBody = sbody;
			noteWeld1.Set(-hy, sy + 2 + hy);
			for (int32 i = 0; i < so_count; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set((2 * i + 1)*hx, sy);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
				jd.Initialize(prevBody, body);
				m_world->CreateJoint(&jd);
				prevBody = body;
			}
		}
		if (addElasticPlastic)
		{
			b2ElasticPlasticJoint::resetEpId();
			float32 sy = 35;
			b2Body* sbody = getStaticBody(staticShape, sy);

			b2PolygonShape shape;
			shape.SetAsBox(hx, hy);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = density;

			b2ElasticPlasticJointDef jd;
			jd.maxForce = 2 * hy*fy*1e6f;
			jd.maxTorque = hy*hy / 4 * fy*1e6f;

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
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);
				prevBody = body;
			}
		}

		if (addParts)
		{
			b2PolygonShape shape;
			shape.SetAsBox(hx, hy);
			float32 sx = so_count*hx*2+5;
			float32 sy = 5;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = density;

			b2WeldJointDef jd;

			b2Body* prevBody = NULL;
			for (int32 i = 0; i < so_count; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(sx + (2 * i + 1)*hx, sy);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				if (i > 0)
				{
					b2Vec2 anchor(sx + (2 * i)*hx, sy);
					jd.Initialize(prevBody, body, anchor);
					m_world->CreateJoint(&jd);
				}

				prevBody = body;
			}
		}
		if (addParts)
		{
			b2PolygonShape shape;
			shape.SetAsBox(hx, hy);
			float32 sx = so_count*hx * 2 + 5;
			float32 sy = 15;
			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = density;

			b2WeldJointDef jd;
			jd.frequencyHz = baseHz;
			jd.dampingRatio = baseDampingRatio;

			b2Body* prevBody = NULL;
			for (int32 i = 0; i < so_count; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(sx + (2 * i + 1)*hx, sy);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				if (i > 0)
				{
					b2Vec2 anchor(sx + (2 * i)*hx, sy);
					jd.Initialize(prevBody, body, anchor);
					m_world->CreateJoint(&jd);
				}

				prevBody = body;
			}
		}
		if (addParts){
			for (int32 i = 0; i < 2; ++i)
			{
				b2Vec2 vertices[3];
				vertices[0].Set(-0.5f, 0.0f);
				vertices[1].Set(0.5f, 0.0f);
				vertices[2].Set(0.0f, 1.5f);

				b2PolygonShape shape;
				shape.Set(vertices, 3);

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = density;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-8.0f + 8.0f * i, 12.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}

			for (int32 i = 0; i < 2; ++i)
			{
				b2CircleShape shape;
				shape.m_radius = 0.5f;

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = density;

				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(-6.0f + 6.0f * i, 10.0f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
		}
	}
	bool showMenu=true;
	virtual void Ui(Settings* settings){
		int menuWidth = 220;
		if (showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2((float)g_camera.m_width - menuWidth-200 - 10, 10));
			ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)g_camera.m_height - 20));
			if (ImGui::Begin("Beam Controls##Bean", &showMenu, 
				ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)){
				ImGui::Text("sub body count");
				ImGui::SliderInt("##sub body count", &so_count, 1, 50);
				ImGui::Text("sub body half length");
				ImGui::SliderFloat("##sub body half length", &hx, 0.1f, 3, "%.2f");
				ImGui::Text("sub body half height");
				ImGui::SliderFloat("##sub body half height", &hy, 0.05f, 3, "%.2f");
				if (addSoft){
					ImGui::Text("Frequency for soft joints");
					ImGui::SliderFloat("Hz##Hertz", &baseHz, 1.f, 60.f, "%.0f");
					ImGui::Text("DampingRatio for soft joints");
					ImGui::SliderFloat("##dratio", &baseDampingRatio, 0.f, 1.0f, "%.3f");
				}
				ImGui::Text("density");
				ImGui::SliderFloat("kg/m^3##density", &density, 1000.f, 20000.f, "%.0f");
				ImGui::Text("yield stress");
				ImGui::SliderFloat("MPa##fy", &fy, 10.f, 1000.f, "%.0f");
				ImGui::Separator();
				ImGui::Checkbox("Soft", &addSoft);
				ImGui::SameLine();
				ImGui::Checkbox("Hard", &addHard);
				ImGui::SameLine();
				ImGui::Checkbox("Parts", &addParts);
				ImGui::Checkbox("Motor", &addMotor);
				ImGui::SameLine();
				ImGui::Checkbox("ElasticPlastic", &addElasticPlastic);
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
						LogJoint(j,1e-6f,1e-6f,locs);
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
		g_debugDraw.DrawString(noteWeld1, "WeldJoint");
		g_debugDraw.DrawString(noteSoftWeld1, "Soft WeldJoint");
		g_debugDraw.DrawString(noteMotor1, "MotorJoint");
		g_debugDraw.DrawString(noteElasticPlastic1, "ElasticPlasticJoint");
	}
	static Test* Create()
	{
		return new Beam;
	}

	b2Body* m_middle;
};

#endif
