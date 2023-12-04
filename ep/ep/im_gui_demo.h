/**
Simple hook to provide instructions on imgui.
*/
#ifndef IM_GUI_DEMO_H
#define IM_GUI_DEMO_H
#include <imgui/imgui.h>
class ImGuiDemo : public Test
{
protected:
	bool open, m_pause;
public:
	ImGuiDemo(Settings * sp):Test(sp){
		settings = sp;
		m_pause = settings->pause;
		settings->pause = true;
		open = true;
	}
	~ImGuiDemo(){
		settings->pause = m_pause;
	}
	virtual void Ui(){
		if (open)
		{
			ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiSetCond_FirstUseEver);
			ImGui::ShowTestWindow();
		}
	}
	virtual bool WantRigidTriangles(){
		return false;
	}
	virtual bool WantEPBeams() {
		return false;
	}
	virtual bool WantMasses() {
		return false;
	}
	static Test* Create(Settings *settings)
	{
		return new ImGuiDemo(settings);
	}
};

#endif