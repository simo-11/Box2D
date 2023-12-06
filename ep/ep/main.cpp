/*
* Copyright (c) 2006-2016 Erin Catto http://www.box2d.org
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
* Simo Nikula modified starting 2017/02
*/
#define _CRT_SECURE_NO_WARNINGS
#include <imgui/imgui.h>

#include "draw.h"
#include "test.h"
#include "ep_joint.h"

#include <glfw/glfw3.h>
#include <stdio.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

// This include was added to support MinGW
#ifdef _WIN32
#include <crtdbg.h>
#endif
#include "../../testbed/imgui_impl_glfw.h"
#include "../../testbed/imgui_impl_opengl3.h"

//
struct UIState
{
	bool showMenu;
};

//
namespace
{
	GLFWwindow* mainWindow = NULL;
	UIState ui;

	int32 testIndex = 0;
	int32 testSelection = 0;
	int32 testCount = 0;
	TestEntry* entry;
	Test* test;
	Settings settings;
	bool rightMouseDown;
	b2Vec2 lastp;
}

static bool canRead(const char* path){
	FILE* f;
	if ((f = fopen(path, "r")) == NULL){
		return false;
	}
	fclose(f);
	return true;
}

//
static void sCreateUI(GLFWwindow* window)
{
	ui.showMenu = true;

	// Init UI
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	const char* fontName = "droid_sans.ttf";
	char* dirs[] = { "../../testbed/data/", "", NULL };
	const int blen = 50;
	char buff[blen];
	for (int i = 0; dirs[i]; i++){
		snprintf(buff, blen, "%s%s", dirs[i], fontName);
		if (canRead(buff)){
			ImGui::GetIO().Fonts->AddFontFromFileTTF(buff, 15.f);
			fprintf(stdout, "using font file %s\n", buff);
			goto fontSearchDone;
		}
		else{
			fprintf(stdout, "font file %s is not available\n",buff);
		}
	}
fontSearchDone:
	if (ImGui_ImplGlfw_InitForOpenGL(window, false) == false)
	{
		fprintf(stderr, "Could not init GUI renderer.\n");
		assert(false);
		return;
	}

	ImGuiStyle& style = ImGui::GetStyle();
	style.FrameRounding = style.GrabRounding = style.ScrollbarRounding = 2.0f;
	style.FramePadding = ImVec2(4, 2);
	style.DisplayWindowPadding = ImVec2(0, 0);
	style.DisplaySafeAreaPadding = ImVec2(0, 0);
}

//
static void sResizeWindow(GLFWwindow*, int width, int height)
{
	g_camera.m_width = width;
	g_camera.m_height = height;
}

//
static void sKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
	bool keys_for_ui = ImGui::GetIO().WantCaptureKeyboard;
	if (keys_for_ui)
		return;

	if (action == GLFW_PRESS)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			// Quit
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);
			break;

		case GLFW_KEY_LEFT:
			// Pan left
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(2.0f, 0.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.x -= 0.5f;
			}
			break;

		case GLFW_KEY_RIGHT:
			// Pan right
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(-2.0f, 0.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.x += 0.5f;
			}
			break;

		case GLFW_KEY_DOWN:
			// Pan down
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(0.0f, 2.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.y -= 0.5f;
			}
			break;

		case GLFW_KEY_UP:
			// Pan up
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(0.0f, -2.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				g_camera.m_center.y += 0.5f;
			}
			break;

		case GLFW_KEY_HOME:
			// Reset view
			g_camera.m_zoom = 1.0f;
			g_camera.m_center.Set(0.0f, 20.0f);
			break;

		case GLFW_KEY_Z:
			// Zoom out
			g_camera.m_zoom = b2Min(1.1f * g_camera.m_zoom, 20.0f);
			break;

		case GLFW_KEY_X:
			// Zoom in
			g_camera.m_zoom = b2Max(0.9f * g_camera.m_zoom, 0.02f);
			break;

		case GLFW_KEY_R:
			// Reset test
			delete test;
			test = entry->createFcn(&settings);
			break;

		case GLFW_KEY_SPACE:
			// Launch a bomb.
			if (test)
			{
				test->LaunchBomb();
			}
			break;

		case GLFW_KEY_O:
			settings.singleStep = true;
			break;

		case GLFW_KEY_P:
			settings.pause = !settings.pause;
			break;

		case GLFW_KEY_LEFT_BRACKET:
			// Switch to previous test
			--testSelection;
			if (testSelection < 0)
			{
				testSelection = testCount - 1;
			}
			break;

		case GLFW_KEY_RIGHT_BRACKET:
			// Switch to next test
			++testSelection;
			if (testSelection == testCount)
			{
				testSelection = 0;
			}
			break;

		case GLFW_KEY_TAB:
			ui.showMenu = !ui.showMenu;

		default:
			if (test)
			{
				test->Keyboard(key);
			}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		test->KeyboardUp(key);
	}
	// else GLFW_REPEAT
}

//
static void sCharCallback(GLFWwindow* window, unsigned int c)
{
	ImGui_ImplGlfw_CharCallback(window, c);
}

//
static void sMouseButton(GLFWwindow* window, int32 button, int32 action, int32 mods)
{
	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);

	double xd, yd;
	glfwGetCursorPos(mainWindow, &xd, &yd);
	b2Vec2 ps((float)xd, (float)yd);

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1)
	{
        //<##>
        //ps.Set(0, 0);
		b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
		if (action == GLFW_PRESS)
		{
			if (mods == GLFW_MOD_SHIFT)
			{
				test->ShiftMouseDown(pw);
			}
			else if (mods == GLFW_MOD_CONTROL){
				test->ControlMouseDown(pw);
			}
			else if (mods == GLFW_MOD_ALT) {
				test->AltMouseDown(pw);
			}
			else
			{
				test->MouseDown(pw, mods);
			}
		}
		
		if (action == GLFW_RELEASE)
		{
			test->MouseUp(pw);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)
	{
		if (action == GLFW_PRESS)
		{	
			lastp = g_camera.ConvertScreenToWorld(ps);
			rightMouseDown = true;
		}

		if (action == GLFW_RELEASE)
		{
			rightMouseDown = false;
		}
	}
}

//
static void sMouseMotion(GLFWwindow*, double xd, double yd)
{
	b2Vec2 ps((float)xd, (float)yd);

	b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
	test->MouseMove(pw);
	
	if (rightMouseDown)
	{
		b2Vec2 diff = pw - lastp;
		g_camera.m_center.x -= diff.x;
		g_camera.m_center.y -= diff.y;
		lastp = g_camera.ConvertScreenToWorld(ps);
	}
}

//
static void sScrollCallback(GLFWwindow* window, double dx, double dy)
{
	ImGui_ImplGlfw_ScrollCallback(window, dx, dy);
	bool mouse_for_ui = ImGui::GetIO().WantCaptureMouse;

	if (!mouse_for_ui)
	{
		if (dy > 0)
		{
			g_camera.m_zoom /= 1.1f;
		}
		else
		{
			g_camera.m_zoom *= 1.1f;
		}
	}
}

//
static void sRestart()
{
	delete test;
	entry = g_testEntries + testIndex;
	test = entry->createFcn(&settings);
}

//
static void sSimulate()
{
	glEnable(GL_DEPTH_TEST);
	test->Step();

	test->DrawTitle(entry->name);
	glDisable(GL_DEPTH_TEST);

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete test;
		entry = g_testEntries + testIndex;
		test = entry->createFcn(&settings);
		g_camera.m_zoom = 1.0f;
		g_camera.m_center.Set(0.0f, 20.0f);
	}
}

//
static bool sTestEntriesGetName(void*, int idx, const char** out_name)
{
	*out_name = g_testEntries[idx].name;
	return true;
}

//
static void sInterface()
{
	int menuWidth = 200;
	if (ui.showMenu)
	{
		ImGui::SetNextWindowPos(ImVec2((float)g_camera.m_width - menuWidth - 10, 10));
		ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)g_camera.m_height - 20));
		ImGui::Begin("Common Controls##cc",
			&ui.showMenu, ImGuiWindowFlags_NoMove|ImGuiWindowFlags_NoResize);
		ImGui::PushAllowKeyboardFocus(false); // Disable TAB

		ImGui::PushItemWidth(-1.0f);

		ImGui::Text("Test");
		if (ImGui::Combo("##Test", &testIndex, sTestEntriesGetName, NULL, testCount, testCount))
		{
			delete test;
			entry = g_testEntries + testIndex;
#ifdef EP_LOG
			epLogClose();
			epLogActive = true;
			epLog("starting %s",entry->name);
#endif
			test = entry->createFcn(&settings);
			testSelection = testIndex;
		}
		ImGui::Separator();
		if (ImGui::CollapsingHeader("General Settings")){
			ImGui::Text("TargetTime");
			ImGui::SameLine();
			ImGui::SliderFloat("##TargetTime", &settings.targetTime, 0.0f, 100.0f, "%.3f s", 2.0f);
			ImGui::Text("Vel Iters");
			ImGui::SameLine();
			ImGui::SliderInt("##Vel Iters", &settings.velocityIterations, 0, 50);
			ImGui::Text("Pos Iters");
			ImGui::SameLine();
			ImGui::SliderInt("##Pos Iters", &settings.positionIterations, 0, 50);
			ImGui::Text("Hertz");
			ImGui::SameLine();
			ImGui::SliderFloat("##Hertz", &settings.hz, 5.0f, 10000.0f, "%.0f hz", 2.0f);
			ImGui::Text("GravityRampUpTime");
			ImGui::SameLine();
			ImGui::SliderFloat("##GravityRampUpTime",
				&settings.gravityRampUpTime, 0.0f, 10.0f, "%.1f s");
			ImGui::Text("Mouse joint force scale");
			ImGui::SliderFloat("##mouseJointForceScale", &settings.mouseJointForceScale,
				1.0f, 1000.0f, "%.0f * mass", 3.0f);
			ImGui::Text("visual joint reaction force");
			ImGui::SliderFloat("##forceScale", &settings.forceScale, 0.001f, 10000.0f, "%.3f", 3.f);
			ImGui::Text("visual joint reaction moment");
			ImGui::SliderFloat("##momentScale", &settings.momentScale, 0.001f, 10000.0f, "%.3f", 3.f);

			ImGui::PopItemWidth();

			ImGui::Checkbox("Sleep", &settings.enableSleep);
			ImGui::Checkbox("Warm Starting", &settings.enableWarmStarting);
			ImGui::Checkbox("Time of Impact", &settings.enableContinuous);
			ImGui::Checkbox("Sub-Stepping", &settings.enableSubStepping);

			ImGui::Separator();

			ImGui::Checkbox("Shapes", &settings.drawShapes);
			ImGui::Checkbox("Joints", &settings.drawJoints);
			ImGui::SameLine();
			ImGui::Checkbox("Joint Reactions", &settings.drawJointReactions);
			ImGui::Checkbox("AABBs", &settings.drawAABBs);
			ImGui::Checkbox("Contact Points", &settings.drawContactPoints);
			ImGui::Checkbox("Contact Normals", &settings.drawContactNormals);
			ImGui::Checkbox("Contact Impulses", &settings.drawContactImpulse);
			ImGui::Checkbox("Friction Impulses", &settings.drawFrictionImpulse);
			ImGui::Checkbox("Center of Masses", &settings.drawCOMs);
			ImGui::Checkbox("Statistics", &settings.drawStats);
			ImGui::SameLine();
			ImGui::Checkbox("Profile", &settings.drawProfile);
			ImGui::Checkbox("Notes", &settings.drawNotes);
			ImGui::Checkbox("Init Impulses", &settings.initImpulses);
			ImGui::Checkbox("Ep-logging", &epLogEnabled);
		}
		if (ImGui::CollapsingHeader("Bomb settings")) {
			ImGui::Text("Mass");
			ImGui::SliderFloat("kg##bombMass", &settings.bombMass, 100.f, 1000000.f, "%.0f",3);
			ImGui::Text("Shape");
			ImGui::SameLine();
			ImGui::RadioButton("Circle", (int *)&settings.bombShape, CIRCLE);
			ImGui::SameLine();
			ImGui::RadioButton("Rectangle", (int *)&settings.bombShape, RECTANGLE);
			switch (settings.bombShape) {
			case CIRCLE:
				ImGui::Text("Diameter");
				break;
			case RECTANGLE:
				ImGui::Text("Width");
				break;
			}
			ImGui::SliderFloat("m##bombRadius", &settings.bombWidth, 0.05f, 10.f, "%.2f");
			ImGui::Text("VelocityScale");
			ImGui::SliderFloat("m##bombMultiplier",
				&settings.bombMultiplier, 0.1f, 120.f, "%.2f");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Drag with Shift-MB1");
			}
			float v[2];
			v[0] = settings.bombVelocity.x;
			v[1] = settings.bombVelocity.y;
			ImGui::Text("Velocity");
			ImGui::SameLine();
			if (ImGui::InputFloat2("Velocity", v,3)) {
				settings.bombVelocity.x = v[0];
				settings.bombVelocity.y = v[1];
			}
			b2Vec2 sp = test->getBombSpawnPoint();
			v[0] = sp.x;
			v[1] = sp.y;
			ImGui::Text("Spawn@");
			ImGui::SameLine();
			if (ImGui::InputFloat2("Start", v,3)) {
				sp.x = v[0];
				sp.y = v[1];
				test->setBombSpawnPoint(sp);
				settings.bombSpawnPoint = sp;
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Launch with <space>");
			}
		}
		if (ImGui::CollapsingHeader("ElasticPlastic Joints",
				test->OpenEPJoints())) {
			ImGui::BeginGroup();
			ImGui::Text("Epd steps");
			ImGui::SameLine();
			ImGui::SliderInt("##EpDebugSteps", &settings.epDebugSteps, 1, 10);
			ImGui::EndGroup();
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Show upto # steps\nin EpDebug Window");
			}
			if (ImGui::Checkbox("Select current EPJoint(s)",
					&settings.selectEPJoint)) {
				settings.addRigidTriangles = false;
				settings.addEPBeams = false;
				settings.addMasses = false;
			}
			if (ImGui::IsItemHovered() && settings.selectEPJoint) {
				ImGui::SetTooltip("Use ALT-MB1");
			}
			if (nullptr != test->GetSelectedJointList()) {
				if (ImGui::SmallButton("Empty current EPJoint list")) {
					test->DeleteSelectedJoints();
				}
				else {
					b2ElasticPlasticJoint *ep = NULL;
					for (SelectedEPJoint* j = test->GetSelectedJointList();
						j != nullptr; j = j->next) {
						if (j->highlight) {
							test->HighLightJoint(j->joint);
							j->highlight = false;
						}
						char buff[20];
						const char* label = buff;
						sprintf(buff, "X %d##xepj-%d", j->id,j->id);
						if (ImGui::SmallButton(label)) {
							ep = j->joint;
						}
						if (ImGui::IsItemHovered()) {
							test->HighLightJoint(j->joint);
							ImGui::SetTooltip("Remove from current EPJoint list");
						}
						ImGui::SameLine();
						sprintf(buff, "EpDebug##epd-%d", j->id);
						if (ImGui::Checkbox(label, &(j->dep))) {
							if (j->dep) {
								j->StartDebug();
							}
							else {
								j->StopDebug();
							}
						}
						if (ImGui::IsItemHovered()) {
							test->HighLightJoint(j->joint);
							ImGui::SetTooltip("EpDebug listener and UI");
						}
						if (j->dep) {
							EpDebug::Ui(test,j);
						}
						if (NULL == j->forces) {
							if (ImGui::SmallButton("Plot forces")) {
								j->forces = new float[3 * EP_MAX_VALUES]();
								for (int i = 0; i < 3 * EP_MAX_VALUES; i++) {
									j->forces[i] = 0.f;
								}
								if (j->joint!=NULL) {
									j->maxForce = j->joint->GetRotatedMaxForce();
									j->maxTorque = j->joint->GetMaxTorque();
								}
							}
						}
						else {
							if (ImGui::SmallButton("Don't plot forces")) {
								delete j->forces;
								j->forces = NULL;
							}
							else {
								b2Vec2 mf = j->maxForce;
								float mm = j->maxTorque;
								ImGui::PlotLines("", &j->forces[0], EP_MAX_VALUES,
									0,"x-f",-mf.x,mf.x);
								ImGui::PlotLines("", &j->forces[EP_MAX_VALUES], EP_MAX_VALUES,
									0, "y-f", -mf.y, mf.y);
								ImGui::PlotLines("",&j->forces[2* EP_MAX_VALUES], EP_MAX_VALUES,
									0, "z-m",-mm,mm);
							}
						}
						if (NULL == j->capacities) {
							if (ImGui::SmallButton("Plot capacities")) {
								j->capacities = new float[5 * EP_MAX_VALUES]();
								for (int i = 0; i < 5 * EP_MAX_VALUES; i++) {
									j->capacities[i] = 0.f;
								}
							}
						}
						else {
							if (ImGui::SmallButton("Don't plot capacities")) {
								delete j->capacities;
								j->capacities = NULL;
							}
							else {
								ImGui::PlotLines("", &j->capacities[0], EP_MAX_VALUES,
									0, "x", 0, 100);
								ImGui::PlotLines("", &j->capacities[EP_MAX_VALUES], EP_MAX_VALUES,
									0, "y", 0, 100);
								ImGui::PlotLines("", &j->capacities[2 * EP_MAX_VALUES], EP_MAX_VALUES,
									0, "z", 0, 100);
								ImGui::PlotLines("", &j->capacities[3 * EP_MAX_VALUES], EP_MAX_VALUES,
									0, "s", 0, 100);
								ImGui::PlotLines("", &j->capacities[4 * EP_MAX_VALUES], EP_MAX_VALUES,
									0, "r", 0, 100);
							}
						}
					}
					if (ep != NULL) {
						test->DeleteSelectedJoint(ep);
					}
				}
			}
		}
		if (test->WantRigidTriangles() && ImGui::CollapsingHeader("RigidTriangles")){
			if (ImGui::Checkbox("Add RigidTriangles", &settings.addRigidTriangles)) {
				settings.selectEPJoint = false;
				settings.addEPBeams = false;
				settings.addMasses = false;
			}
			if (ImGui::IsItemHovered() && settings.addRigidTriangles){
				ImGui::SetTooltip("Use ALT-MB1");
			}
			bool deleteRigidTriangles = 
				ImGui::SmallButton("Delete all RigidTriangles");
			unsigned char labelForDelete = 0;
			for (RigidTriangle* rt = test->GetRigidTriangleList(); 
				rt!=nullptr; rt = rt->next)
			{ // draw labels and update positions
				bool movingByMouse = false;
				bool valueChanged = false;
				if (test->m_movingBody==rt->body){
					movingByMouse = true;
				}
				b2Body *body = rt->body;
				b2Vec2 p = body->GetTransform().p;
				rt->position[0] = p.x;
				rt->position[1] = p.y;
				char buff[4];
				_itoa(rt->label, buff, 10);
				ImGui::TextDisabled("%d", rt->label);
				ImGui::SameLine();
				int decimals = 3;
				if (movingByMouse){
					ImGui::TextDisabled("%.3f", p.x);
					ImGui::SameLine();
					ImGui::TextDisabled("%.3f", p.y);
				}
				else{
					char bufa[20];
					const char* label = bufa;
					sprintf(bufa, "X##drt-%d", rt->label);
					if (ImGui::SmallButton(label)){
						labelForDelete = rt->label;
					}
					ImGui::SameLine();
					valueChanged = ImGui::InputFloat2(bufa, rt->position, decimals);
				}
				float zoom = g_camera.m_zoom;
				p.x -= 1.5f*zoom;
				p.y += 1.f*zoom;
				g_debugDraw.DrawString(p, "%s", buff);
				b2Vec2 np;
				if (valueChanged){
					np.x = rt->position[0];
					np.y = rt->position[1];
					body->SetTransform(np, 0);
				}
			}
			if (labelForDelete){
				Test::DeleteRigidTriangle(labelForDelete);
			}
			if (deleteRigidTriangles){
				Test::DeleteRigidTriangles();
			}
		}
		else {
			settings.addRigidTriangles = false;
		}
		if (test->WantMasses() && ImGui::CollapsingHeader("AddMasses")) {
			if (ImGui::Checkbox("Add Masses", &settings.addMasses)) {
				settings.selectEPJoint = false;
				settings.addRigidTriangles = false;
				settings.addEPBeams = false;
			}
			if (ImGui::Button("Add at @")) {
				test->AddMass(settings.addMassPoint);
			}
			ImGui::SameLine();
			float mv[2];
			mv[0] = settings.addMassPoint.x;
			mv[1] = settings.addMassPoint.y;
			if (ImGui::InputFloat2("", mv,3)) {
				settings.addMassPoint.x = mv[0];
				settings.addMassPoint.y = mv[1];
			}
			if (ImGui::IsItemHovered() && settings.addMasses) {
				ImGui::SetTooltip("Use ALT-MB1");
			}
			ImGui::Text("size [m]");
			ImGui::SameLine();
			ImGui::SliderFloat
			("size [m]", &settings.addMassSize, 0.1f, 100);
			ImGui::InputFloat
				("Mass [kg]", &settings.addMass, 1000,100000,0);
		}
		else {
			settings.addMasses = false;
		}
		if (test->WantEPBeams() && 
			ImGui::CollapsingHeader("ElasticPlasticBeams")) {
			ImGui::Text("MaxF");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Maximum force for joint");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbMaxForce",
				&settings.epbMaxForce, 0.1f, 1000000.f,"%.0f",3.f);
			ImGui::Text("MaxM");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Maximum moment for joint");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbMaxMoment",
				&settings.epbMaxMoment, 0.1f, 1000000.f, "%.0f", 3.f);
			ImGui::Text("MaxS");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Maximum strain for joint");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbMaxStrain",
				&settings.epbMaxStrain, 0.1f, 10.f, "%.2f", 2.f);
			ImGui::Text("MaxR");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Maximum rotation for joint");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbMaxRotation",
				&settings.epbMaxRotation, 0.f, 10.f, "%.2f", 2.f);
			ImGui::Text("Frequency");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Frequency for joints");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbHz", 
				&settings.epbHz, 0.f, settings.hz, "%.2f Hz");
			if (settings.epbHz > 0) {
				ImGui::Text("MaxElasticRotation");
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip
					("MaxElasticRotation for elastic joints in radians");
				}
				ImGui::SameLine();
				ImGui::SliderFloat("##epbMaxElasticRotation",
					&settings.epbMaxElasticRotation, 0.f, 1.f, "%.2f rad");
			}
			ImGui::Text("Mass");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("mass for beam");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbMass", 
				&settings.epbMass, 0.1f, 100000.f, "%.2f kg", 3.f);
			ImGui::Text("X");
			ImGui::SameLine();
			ImGui::SliderFloat("##epbX",
				&settings.epbX, 0.1f, 10.f, "%.2f m");
			ImGui::Text("Y");
			ImGui::SameLine();
			ImGui::SliderFloat("##epbY",
				&settings.epbY, 0.1f, 10.f, "%.2f m");
			ImGui::Checkbox("DebugListener",
				&settings.epbDebugListener);
			if (ImGui::Checkbox("Add ElasticPlastic Beams",
				&settings.addEPBeams)) {
				settings.selectEPJoint = false;
				settings.addRigidTriangles = false;
				settings.addMasses = false;
			}
			if (ImGui::IsItemHovered() && settings.addEPBeams) {
				ImGui::SetTooltip("Use ALT-MB1");
			}
			bool deleteEPBeams =
				ImGui::SmallButton("Delete all ElasticPlastic Beams");
			unsigned char labelForDelete = 0;
			for (EPBeam* rt = test->GetEPBeamList();
				rt != nullptr; rt = rt->next)
			{ // draw labels and update positions
				b2Body *body = rt->sBody;
				if (rt->deleteSbody) {
					// turn to sensor
					body->GetFixtureList()->SetSensor(true);
					rt->deleteSbody = false;
				}
				char lbuff[4];
				_itoa(rt->label, lbuff, 10);
				ImGui::TextDisabled("%d", rt->label);
				ImGui::SameLine();
				int decimals = 3;
				char buff[20];
				const char* label = buff;
				sprintf(buff, "X##depb-%d", rt->label);
				if (ImGui::SmallButton(label)) {
					labelForDelete = rt->label;
				}
				ImGui::SameLine();
				b2Vec2 p = body->GetTransform().p;
				rt->position[0] = p.x;
				rt->position[1] = p.y;
				bool valueChanged = ImGui::InputFloat2(buff, rt->position, decimals);
				float zoom = g_camera.m_zoom;
				b2Vec2 lp = p;
				lp.x -= 1.5f*zoom;
				lp.y += 1.f*zoom;
				g_debugDraw.DrawString(lp, "%s", lbuff);
				b2Vec2 np;
				if (valueChanged) {
					np.x = rt->position[0];
					np.y = rt->position[1];
					body->SetTransform(np, 0);
				}
			}
			if (labelForDelete) {
				Test::DeleteEPBeam(labelForDelete);
			}
			if (deleteEPBeams) {
				Test::DeleteEPBeams();
			}
		}
		else{
			settings.addEPBeams = false;
		}
		if (ImGui::SmallButton("Pause (P)"))
			settings.pause = !settings.pause;
		ImGui::SameLine();
		if (ImGui::SmallButton("Single Step (O)"))
			settings.singleStep = !settings.singleStep;

		if (ImGui::SmallButton("Restart (R)") || test->restartPending) {
			sRestart();
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("Quit"))
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);
		if (ImGui::SmallButton("ResetCommon")){
			settings.reset();
		}
		ImGui::SameLine();
		if (ImGui::SmallButton("ResetTest")){
			test->reset();
		}

		ImGui::PopAllowKeyboardFocus();
		ImGui::End();
	}
	test->Ui();
	//ImGui::ShowTestWindow(NULL);
}
//
void glfwErrorCallback(int error, const char *description)
{
	fprintf(stderr, "GLFW error occured. Code: %d. Description: %s\n", error, description);
}
//
int main(int, char**)
{
#if defined(_WIN32)
	// Enable memory-leak reports
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif
	glfwSetErrorCallback(glfwErrorCallback);
	g_camera.m_width = 1280;
	g_camera.m_height = 840;

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	char title[64];
	sprintf(title, "Box2D ElasticPlastic Version 0.2.0");

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title, NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);
	// Load OpenGL functions using glad
	int version = gladLoadGL(glfwGetProcAddress);
	printf("GL %d.%d\n", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));
	printf("OpenGL %s, GLSL %s\n", 
		glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetCharCallback(mainWindow, sCharCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

	g_debugDraw.Create();

	sCreateUI(mainWindow);

	testCount = 0;
	while (g_testEntries[testCount].createFcn != NULL)
	{
		++testCount;
	}

	testIndex = b2Clamp(testIndex, 0, testCount - 1);
	testSelection = testIndex;

	entry = g_testEntries + testIndex;
	test = entry->createFcn(&settings);
	const ImVec4 bgc=ImVec4(0.0f,0.0f,0.0f,0.7f);
	// ImGui::PushStyleColor(ImGuiCol_WindowBg,bgc);
	// Control the frame rate. One draw per monitor refresh.
	glfwSwapInterval(1);
	double time1 = glfwGetTime();
	double frameTime = 0.0;
	glClearColor(0.3f, 0.3f, 0.3f, 1.f);
	while (!glfwWindowShouldClose(mainWindow))
	{
		bool opened;
		glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
		glViewport(0, 0, g_camera.m_width, g_camera.m_height);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImVec2((float)g_camera.m_width, (float)g_camera.m_height));
		opened = ImGui::Begin("Overlay", NULL, ImVec2(0, 0), 0.0f,
			ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs |
			ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
		if (opened){
			ImGui::SetCursorPos(ImVec2(5, (float)g_camera.m_height - 20));
			b2Vec2 mp = test->GetMouseWorld();
			ImGui::Text("%.1f ms, steppedTime=%.4f s mouse @(%.3f,%.3f)", 
				1000.0 * frameTime, test->steppedTime, mp.x,mp.y);
		}
		ImGui::End();
		sSimulate();
		sInterface();
		// Measure speed
		double time2 = glfwGetTime();
		double alpha = 0.9f;
		frameTime = alpha * frameTime + (1.0 - alpha) * (time2 - time1);
		time1 = time2;
		ImGui::Render();
		glfwSwapBuffers(mainWindow);
		glfwPollEvents();
	}

	if (test)
	{
		test->DeleteSelectedJoints();
		delete test;
		test = nullptr;
	}
	Test::DeleteRigidTriangles();
	Test::DeleteEPBeams();
	g_debugDraw.Destroy();
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	glfwTerminate();
	return 0;
}
