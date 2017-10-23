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

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#endif

#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw_gl3.h>
#include "DebugDraw.h"
#include "Test.h"
#include "RigidTriangle.h"

#include <glfw/glfw3.h>
#include <stdio.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

// This include was added to support MinGW
#ifdef _WIN32
#include <crtdbg.h>
#endif

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
	const char* fontName = "DroidSans.ttf";
	char* dirs[] = { "../Data/", "", NULL };
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
	if (ImGui_ImplGlfwGL3_Init(window, false) == false)
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
	ImGui_ImplGlfwGL3_KeyCallback(window, key, scancode, action, mods);
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
	ImGui_ImplGlfwGL3_CharCallback(window, c);
}

//
static void sMouseButton(GLFWwindow* window, int32 button, int32 action, int32 mods)
{
	ImGui_ImplGlfwGL3_MouseButtonCallback(window, button, action, mods);

	double xd, yd;
	glfwGetCursorPos(mainWindow, &xd, &yd);
	b2Vec2 ps((float32)xd, (float32)yd);

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
	ImGui_ImplGlfwGL3_ScrollCallback(window, dx, dy);
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
			test = entry->createFcn(&settings);
			testSelection = testIndex;
		}
		ImGui::Separator();
		if (ImGui::CollapsingHeader("Settings","MainSettings")){
			ImGui::Text("Vel Iters");
			ImGui::SameLine();
			ImGui::SliderInt("##Vel Iters", &settings.velocityIterations, 0, 50);
			ImGui::Text("Pos Iters");
			ImGui::SameLine();
			ImGui::SliderInt("##Pos Iters", &settings.positionIterations, 0, 50);
			ImGui::Text("Hertz");
			ImGui::SameLine();
			ImGui::SliderFloat("##Hertz", &settings.hz, 5.0f, 10000.0f, "%.0f hz", 2.0f);
			ImGui::Text("Mouse joint force scale");
			ImGui::SliderFloat("##mouseJointForceScale", &settings.mouseJointForceScale,
				1.0f, 1000.0f, "%.0f * mass", 3.0f);
			ImGui::Text("visual joint reaction force");
			ImGui::SliderFloat("##forceScale", &settings.forceScale, 0.001f, 10000.0f, "%.3f", 3.f);
			ImGui::Text("visual joint reaction moment");
			ImGui::SliderFloat("##momentScale", &settings.momentScale, 0.001f, 1000.0f, "%.3f", 2.f);
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
		}
		if (ImGui::CollapsingHeader("ElasticPlastic Joints")) {
			if (ImGui::Checkbox("Select current EPJoint(s)", &settings.selectEPJoint)) {
				settings.addRigidTriangles = false;
				settings.addEPBeams = false;
			}
			if (ImGui::IsItemHovered() && settings.selectEPJoint) {
				ImGui::SetTooltip("Use ALT-MB1");
			}
			if (nullptr != test->GetSelectedJointList()) {
				if (ImGui::SmallButton("Empty current EPJoint list")) {
					test->DeleteSelectedJoints();
				}
				else {
					b2ElasticPlasticJoint* ep = NULL;
					for (SelectedEPJoint* j = test->GetSelectedJointList();
						j != nullptr; j = j->next) {
						char buff[20];
						const char* label = buff;
						sprintf(buff, "X##depj-%d", j->id);
						if (ImGui::SmallButton(label)) {
							ep = j->joint;
						}
						if (ImGui::IsItemHovered()) {
							test->HighLightJoint(j->joint);
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
					char buff[20];
					const char* label = buff;
					sprintf(buff, "X##drt-%d", rt->label);
					if (ImGui::SmallButton(label)){
						labelForDelete = rt->label;
					}
					ImGui::SameLine();
					valueChanged = ImGui::InputFloat2(buff, rt->position, decimals);
				}
				float32 zoom = g_camera.m_zoom;
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
			if (ImGui::IsItemHovered() && settings.addMasses) {
				ImGui::SetTooltip("Use ALT-MB1");
			}
			bool valueChanged = ImGui::InputFloat
				("Mass [kg]", &settings.addMass, 1000,1000000,0);
		}
		else {
			settings.addMasses = false;
		}
		if (test->WantEPBeams() && ImGui::CollapsingHeader("ElasticPlasticBeams")) {
			ImGui::Text("Scale");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Scale maximum force for joint");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbScale", &settings.epbScale, 0.1f, 100.f,"%.2f",3.f);
			ImGui::Text("Mass scale");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Scale mass for beam");
			}
			ImGui::SameLine();
			ImGui::SliderFloat("##epbMassScale", &settings.epbMassScale, 0.1f, 100.f, "%.2f", 3.f);
			if (ImGui::Checkbox("Add Elastic Plastic Beams", &settings.addEPBeams)) {
				settings.selectEPJoint = false;
				settings.addRigidTriangles = false;
				settings.addMasses = false;
			}
			if (ImGui::IsItemHovered() && settings.addEPBeams) {
				ImGui::SetTooltip("Use ALT-MB1");
			}
			bool deleteEPBeams =
				ImGui::SmallButton("Delete all ElasticPlasticBeams");
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
				float32 zoom = g_camera.m_zoom;
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

		if (ImGui::SmallButton("Restart (R)"))
			sRestart();
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
int main(int, char**)
{
#if defined(_WIN32)
	// Enable memory-leak reports
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

	g_camera.m_width = 1280;
	g_camera.m_height = 840;

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	char title[64];
	sprintf(title, "Box2D ElasticPlastic Version 0.2.0");

#if defined(__APPLE__)
	// Not sure why, but these settings cause glewInit below to crash.
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

	mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title, NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);
	printf("OpenGL %s, GLSL %s\n", 
		glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetCharCallback(mainWindow, sCharCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

#if defined(__APPLE__) == FALSE
	//glewExperimental = GL_TRUE;
	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(EXIT_FAILURE);
	}
#endif

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
		ImGui_ImplGlfwGL3_NewFrame();
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImVec2((float)g_camera.m_width, (float)g_camera.m_height));
		opened = ImGui::Begin("Overlay", NULL, ImVec2(0, 0), 0.0f,
			ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs |
			ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
		if (opened){
			ImGui::SetCursorPos(ImVec2(5, (float)g_camera.m_height - 20));
			ImGui::Text("%.1f ms, steppedTime=%.4f s", 1000.0 * frameTime, test->steppedTime);
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
	ImGui_ImplGlfwGL3_Shutdown();
	glfwTerminate();

	return 0;
}
