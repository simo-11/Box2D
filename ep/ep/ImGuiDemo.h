/**
Simple hook to provide user instructions of imgui usage.
*/
#ifndef IM_GUI_DEMO_H
#define IM_GUI_DEMO_H
#include <imgui/imgui.h>
class ImGuiDemo : public Test
{
protected:
	bool open, m_pause;
	Settings* m_settings = nullptr;
public:
	ImGuiDemo(){
		open = true;
	}
	~ImGuiDemo(){
		if (m_settings != nullptr){
			m_settings->pause = m_pause;
			m_settings = nullptr;
		}
	}
	virtual void Ui(Settings* settings){
		if (m_settings == nullptr){
			m_settings = settings;
			m_pause = settings->pause;
			settings->pause = true;
		}
		if (open)
		{
			ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiSetCond_FirstUseEver);
			ImGui::ShowTestWindow(&open);
		}
	}
	virtual bool WantRigidTriangles(){
		return false;
	}

	static Test* Create()
	{
		return new ImGuiDemo;
	}
};

#endif