
#ifndef IM_GUI_DEMO_H
#define IM_GUI_DEMO_H
#include <imgui/imgui.h>
namespace {
	bool p_opened=true;
}
class ImGuiDemo : public Test
{
public:
	ImGuiDemo(){
		ImGui::NewFrame();
		ImGui::ShowTestWindow(&p_opened);
	}
	static Test* Create()
	{
		return new ImGuiDemo;
	}
};

#endif