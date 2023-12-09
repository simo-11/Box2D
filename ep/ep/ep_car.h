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
* Simo Nikula based on Standard Box2D demo Car.h
*
*/

#ifndef EPCAR_H
#define EPCAR_H
#include "Test.h"

enum CarType {
	NoCar=0,
	EpCar,
};
namespace epCar{
	CarType carType;
	float hz;
	float zeta;
	float speed;
	float length, carDensity;
	float wheelRadius, wheelDensity;
	float motorTorque;
	float brakeTorque, wheelFrictionTorque;
	float groundFriction,wheelFriction, carBodyFriction;
}
class EPCar : public Test
{
public:
	EPCar(Settings* sp) :Test(sp)
	{
	}
	bool isMyType() {
		return (epCar::carType == EpCar);
	}
	void reset() {
		epCar::hz = 4.0f;
		epCar::zeta = 0.7f;
		epCar::speed = 40.0f;
		epCar::length = 3.f;
		epCar::carDensity = 200.f;
		epCar::wheelRadius = 0.4f;
		epCar::wheelDensity = 100.f;
		epCar::groundFriction = 0.9f;
		epCar::wheelFriction = 0.9f;
		epCar::carBodyFriction = 0.9f;
		epCar::motorTorque = 1000.f;
		epCar::brakeTorque = 1500.f;
		epCar::wheelFrictionTorque = 50.f;
		epCar::carType = EpCar;
		settings->reset();
	}
	void build() {
		if (!isMyType()) {
			reset();
			g_camera.m_center.y = 5.f;
			g_camera.m_zoom = 0.3f;
		}
		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = epCar::groundFriction;

			shape.SetTwoSided(b2Vec2(-50.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground->CreateFixture(&fd);

			float hs[10] = { 0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f };

			float x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (int32 i = 0; i < 10; ++i)
			{
				float y2 = hs[i];
				shape.SetTwoSided(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			shape.SetTwoSided(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.SetTwoSided(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.SetTwoSided(b2Vec2(x, 0.0f), b2Vec2(x + 20.0f, 5.0f));
			ground->CreateFixture(&fd);

			x += 20.0f;
			shape.SetTwoSided(b2Vec2(x, 5.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.SetTwoSided(b2Vec2(x, 0.0f), b2Vec2(x+100.f, 0.0f));
			ground->CreateFixture(&fd);

			{
				// upper level
				// ramp is dynamic body with joint to ramp fixpoint rf.
				// joint at x,y
				// fhx is half length of fixed horizontal part
				// hx is half length of dynamic part of ramp
				float xb = 60.f,fhx=25.f;
				float y = 5.f;
				float hxb = 9.f;
				// fixed horizontal part
				b2BodyDef bdf;
				bdf.position.Set(xb+fhx, y);
				b2FixtureDef fdb;
				b2PolygonShape shapeb;
				shapeb.SetAsBox(fhx, 0.02f);
				fdb.shape = &shapeb;
				fdb.density = 0.f;
				b2Body* rf = m_world->CreateBody(&bdf);
				rf->CreateFixture(&fdb);
				// dynamic part of ramp
				shapeb.SetAsBox(hxb, 0.02f);
				fdb.shape = &shapeb;
				fdb.density = 3000.f;
				fdb.friction = epCar::groundFriction;
				bdf.type = b2_dynamicBody;
				bdf.position.Set(xb-hxb, y);
				b2Body* body = m_world->CreateBody(&bdf);
				body->CreateFixture(&fdb);
				b2ElasticPlasticJointDef jd;
				jd.maxForce = b2Vec2(0.02f*300.e6f,0.02*150e6f);
				jd.maxElasticRotation = 0.12f;
				jd.maxRotation = 0.4f;
				jd.maxStrain = 0.5f;
				jd.frequencyHz = 1;
				jd.dampingRatio = 0.2f;
				b2Vec2 anchor(x, y);
				jd.Initialize(body, rf, anchor);
				m_world->CreateJoint(&jd);
			}
		}

		// EPCar
		{
			b2PolygonShape chassis;
			b2Vec2 vertices[8]{};
			vertices[0].Set(-0.5f*epCar::length, -0.5f);
			vertices[1].Set(0.5f*epCar::length, -0.5f);
			vertices[2].Set(0.5f*epCar::length, 0.0f);
			vertices[3].Set(0.0f, 0.9f);
			vertices[4].Set(-0.38f*epCar::length, 0.9f);
			vertices[5].Set(-0.5f*epCar::length, 0.2f);
			chassis.Set(vertices, 6);

			b2CircleShape circle;
			circle.m_radius = epCar::wheelRadius; 

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 1.0f);
			m_Car = m_world->CreateBody(&bd);
			{
				b2FixtureDef fd;
				fd.shape = &chassis;
				fd.density = epCar::carDensity;
				fd.friction = epCar::carBodyFriction;
				m_Car->CreateFixture(&fd);
			}
			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = epCar::wheelDensity;
			fd.friction = epCar::wheelFriction;

			bd.position.Set(-0.3f*epCar::length, 0.35f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&fd);

			bd.position.Set(0.3f*epCar::length, 0.4f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&fd);

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);
			float mass1 = m_wheel1->GetMass();
			float mass2 = m_wheel2->GetMass();

			float hertz = 4.0f;
			float dampingRatio = 0.7f;
			float omega = 2.0f * b2_pi * hertz;
			jd.Initialize(m_Car, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = epCar::wheelFrictionTorque;
			jd.enableMotor = true;
			jd.stiffness = mass1 * omega * omega;
			jd.damping = 2.0f * mass1 * dampingRatio * omega;
			jd.lowerTranslation = -0.25f;
			jd.upperTranslation = 0.25f;
			jd.enableLimit = true;
			m_spring1 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_Car, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = epCar::wheelFrictionTorque;
			jd.enableMotor = true;
			jd.stiffness = mass2 * omega * omega;
			jd.damping = 2.0f * mass2 * dampingRatio * omega;
			jd.lowerTranslation = -0.25f;
			jd.upperTranslation = 0.25f;
			jd.enableLimit = true;
			m_spring2 = (b2WheelJoint*)m_world->CreateJoint(&jd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			m_spring1->SetMotorSpeed(epCar::speed);
			m_spring1->SetMaxMotorTorque(epCar::motorTorque);
			break;
		case GLFW_KEY_C:
			cruiseControl = !cruiseControl;
			if (!cruiseControl) {
				KeyboardUp(GLFW_KEY_S);
				KeyboardUp(GLFW_KEY_A);
				KeyboardUp(GLFW_KEY_D);
			}
			break;

		case GLFW_KEY_S:
			m_spring1->SetMotorSpeed(0.0f);
			m_spring1->SetMaxMotorTorque(epCar::brakeTorque);
			m_spring2->SetMaxMotorTorque(epCar::brakeTorque);
			break;

		case GLFW_KEY_D:
			m_spring1->SetMotorSpeed(-epCar::speed);
			m_spring1->SetMaxMotorTorque(epCar::motorTorque);
			break;
		}
	}
	void KeyboardUp(int key)
	{
		if (cruiseControl) {
			return;
		}
		switch (key)
		{
		case GLFW_KEY_A:
			m_spring1->SetMaxMotorTorque(epCar::wheelFrictionTorque);
			m_spring1->SetMotorSpeed(0.f);
			break;

		case GLFW_KEY_S:
			m_spring1->SetMaxMotorTorque(epCar::wheelFrictionTorque);
			m_spring1->SetMotorSpeed(0.f);
			m_spring2->SetMaxMotorTorque(epCar::wheelFrictionTorque);
			m_spring2->SetMotorSpeed(0.f);
			break;

		case GLFW_KEY_D:
			m_spring1->SetMaxMotorTorque(epCar::wheelFrictionTorque);
			m_spring1->SetMotorSpeed(0.f);
			break;
		}
	}
	void updateX(b2Body* b, float xmove) {
		b2Vec2 p = b->GetPosition();
		p.x += xmove;
		float a = b->GetAngle();
		b->SetTransform(p, a);
	}
	void Step()
	{
		/** Keep car in range -lowerLimit - upperLimit+2*lowerLimit
		-10 - 250
		*/
		float upperLimit = 270, lowerLimit=-10;
		float move=0.f;
		float cx = m_Car->GetPosition().x;
		if ( cx> upperLimit+2*lowerLimit) {
			move = lowerLimit-cx;
		}else if (cx <lowerLimit) {
			move = upperLimit + cx;
		}
		if(move!=0.f){
			updateX(m_Car,move);
			updateX(m_wheel1, move);
			updateX(m_wheel2, move);
		}
		Test::Step();
	}
	void UpdateCamera() {
		g_camera.m_center.x = m_Car->GetPosition().x;
	}
	static Test* Create(Settings *settings)
	{
		EPCar* t = new EPCar(settings);
		t->build();
		t->CommonEpInit();
		return t;
	}

	b2Body* m_Car;
	b2Body* m_wheel1;
	b2Body* m_wheel2;

	b2WheelJoint* m_spring1;
	b2WheelJoint* m_spring2;
	bool showMenu = true;
	bool cruiseControl = false;
	void drawNotes() {
		float s = m_Car->GetLinearVelocity().Length();
		float rs1 = b2Abs(m_spring1->GetJointAngularSpeed());
		float rs2 = b2Abs(m_spring2->GetJointAngularSpeed());
		char * tc = "";
		if (b2Abs(rs1 - rs2) > 3) {
			tc = "!";
		}
		int32 tx = (g_camera.m_width - 200) / 2;
		int32 ty = g_camera.m_height / 2;
		g_debugDraw.DrawString(tx,ty,"Keys: left = a, brake = s, right = d");
		ty += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(tx, ty, "  cruise control = c, currently %s", (cruiseControl?"on":"off"));
		ty += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(tx,ty,
			"%s %.0f rad/s, %.0f rad/s, %.0f m/s, %0.f km/h",
			tc,rs1,rs2,s,3.6f*s);
	}
	virtual void Ui() {
		int menuWidth = 220;
		if (showMenu)
		{
			ImGui::SetNextWindowPos(ImVec2((float)g_camera.m_width - menuWidth - 200 - 10, 10));
			ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)g_camera.m_height - 20));
			if (ImGui::Begin("Car Controls##EPCar", &showMenu,
				ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize)) {
				if (ImGui::CollapsingHeader("Settings"))
				{
					ImGui::Text("Length");
					ImGui::SliderFloat("##length", &epCar::length, 3.f, 10.0f, "%.1f");
					ImGui::Text("Density");
					ImGui::SliderFloat("##density", &epCar::carDensity, 100.f, 300.f, "%.0f");
					ImGui::Text("Max speed rad/s");
					if (ImGui::SliderFloat("##speed",
						&epCar::speed, 0.f, 100.0f, "%.0f")) {
						float s = epCar::speed;
						float cs = m_spring1->GetMotorSpeed();
						if (cs != 0.f) {
							if (cs < 0) {
								s = -s;
							}
							m_spring1->SetMotorSpeed(s);
						}

					}
					ImGui::Text("Wheel radius");
					ImGui::SliderFloat("##wheelRadius", &epCar::wheelRadius, 0.2f, 1.f, "%.2f");
					ImGui::Text("motor torque");
					if (ImGui::SliderFloat("##motorTorque", 
						&epCar::motorTorque, 300.f, 3000.f, "%.0f", 2.f)) {
						float t = epCar::motorTorque;
						float ct = m_spring1->GetMaxMotorTorque();
						if (ct != 0.f) {
							if (ct < 0) {
								t = -t;
							}
							m_spring1->SetMaxMotorTorque(t);
						}
					}
					ImGui::Text("break torque");
					ImGui::SliderFloat("##breakTorque", &epCar::brakeTorque, 1000.f, 10000.f, "%.0f",2.f);
					ImGui::Text("wheel friction torque");
					ImGui::SliderFloat("##wheelFrictionTorque", 
						&epCar::wheelFrictionTorque, 1.f, 1000.f, "%.0f", 2.f);
					ImGui::Text("ground friction");
					ImGui::SliderFloat("##groundFriction", &epCar::groundFriction, 0.1f, 1.f, "%.2f");
					ImGui::Text("wheel friction");
					ImGui::SliderFloat("##wheelFriction", &epCar::wheelFriction, 0.1f, 1.f, "%.2f");
					ImGui::Text("car body friction");
					ImGui::SliderFloat("##carBodyFriction", &epCar::carBodyFriction, 0.1f, 1.f, "%.2f");
					ImGui::Text("wheel density");
					ImGui::SliderFloat("##wheelDensity", &epCar::wheelDensity, 10.f, 100.f, "%.1f");
				}
			}
			float locs[4] = { 40, 80, 120, 160 };
			if (ImGui::CollapsingHeader("Joint forces kN/kNm"))
			{
				ImGui::Text(" x-f"); ImGui::SameLine(locs[0]);
				ImGui::Text(" y-f"); ImGui::SameLine(locs[1]);
				ImGui::Text(" z-m"); ImGui::SameLine(locs[2]);
				ImGui::Text(" j-x"); ImGui::SameLine(locs[3]);
				ImGui::Text(" j-y");
				for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
				{
					LogJoint(j, 1.e-3f, 1.e-3f, locs,"%4.1f",999.9f);
				}
			}
			else  {
				LogSelectedJoints( 1.e-3f, 1.e-3f, locs, "%4.1f", 999.9f);
			}

			if (ImGui::CollapsingHeader("Joint forces MN/MNm"))
			{
				float locsb[4] = { 40, 80, 120, 160 };
				ImGui::Text(" x-f"); ImGui::SameLine(locsb[0]);
				ImGui::Text(" y-f"); ImGui::SameLine(locsb[1]);
				ImGui::Text(" z-m"); ImGui::SameLine(locsb[2]);
				ImGui::Text(" j-x"); ImGui::SameLine(locsb[3]);
				ImGui::Text(" j-y");
				for (b2Joint* j = m_world->GetJointList(); j; j = j->GetNext())
				{
					LogJoint(j, 1e-6f, 1e-6f, locsb, "%4.2f",(float)FLT_MAX,0.09999f);
				}
			}
			else {
				LogSelectedJoints(1e-6f, 1e-6f,
					locs, "%5.2f", (float)FLT_MAX, 0.099999f);
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
					case e_rigidPlasticJoint:
					case e_elasticPlasticJoint:
						LogEpCapasity((b2ElasticPlasticJoint*)j, locs);
						break;
					}
				}
			}
			else {
				LogEpCapasityForSelectedJoints(locs);
			}

			if (ImGui::CollapsingHeader("Contact forces N"))
			{
				float locsb[] = { 50, 100, 150};
				ImGui::Text(" x-f"); ImGui::SameLine(locsb[0]);
				ImGui::Text(" y-f"); ImGui::SameLine(locsb[1]);
				ImGui::Text(" c-x"); ImGui::SameLine(locsb[2]);
				ImGui::Text(" c-y");
				for (int i = 0; i < m_pointCount; i++) {
					LogContact(m_points+i, 1.0f, locsb, "%5.0f");
				}
			}
			ImGui::End();
			if (settings->drawNotes) {
				drawNotes();
			}
		}
	}
	float getEpBeamMaxForce() {
		return m_Car->GetMass()*b2Abs(m_world->GetGravity().y);
	}
};
#endif