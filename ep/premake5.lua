-- Box2D -ep (elastic-plastic) extensions premake5 script.
-- https://premake.github.io/

workspace "0_Box2D-EP"
	location 'Build'
	architecture "x86_64"
	configurations { "Debug", "Release" }
	startproject 'ep'
	cppdialect 'C++11'
	symbols 'On'
	warnings 'Extra'
	
	configuration "vs*"
		defines { "_CRT_SECURE_NO_WARNINGS" }	
		
	filter "configurations:Debug"
	 	defines { "DEBUG" }

	filter "configurations:Release"
		defines { "NDEBUG" }
		optimize "On"

project "Box2D"
	kind "StaticLib"
	language "C++"
	files { "../Box2D/**" }
	includedirs { ".."}
	
		 
local glfw_common = {
	"../Testbed/glfw/internal.h",
	"../Testbed/glfw/glfw_config.h",
	"../Testbed/glfw/glfw3.h",
	"../Testbed/glfw/glfw3native.h",
	"../Testbed/glfw/context.c",
	"../Testbed/glfw/init.c",
	"../Testbed/glfw/input.c",
	"../Testbed/glfw/monitor.c",
	"../Testbed/glfw/vulkan.c",
	"../Testbed/glfw/window.c" }

project "GLFW"
	kind "StaticLib"
	language "C"
	configuration { "windows" }
		local f = {
			'../Testbed/glad/*',
			"../Testbed/glfw/win32_platform.h",
			"../Testbed/glfw/win32_joystick.h",
			"../Testbed/glfw/wgl_context.h",
			"../Testbed/glfw/egl_context.h",
			"../Testbed/glfw/win32_init.c",
            "../Testbed/glfw/win32_joystick.c",
			"../Testbed/glfw/win32_monitor.c",
			"../Testbed/glfw/win32_time.c",
            "../Testbed/glfw/win32_tls.c",
            "../Testbed/glfw/win32_window.c",
        	"../Testbed/glfw/wgl_context.c",
        	"../Testbed/glfw/egl_context.c"
		}
        for i, v in ipairs(glfw_common) do
        	f[#f + 1] = glfw_common[i]
        end
    	files(f)

project "IMGUI"
	kind "StaticLib"
	language "C++"
	files { "../Testbed/imgui/*.h", "../Testbed/imgui/*.cpp" }
	includedirs { ".." }

project "ep"
	kind "ConsoleApp"
	debugdir 'ep'
	files { "ep/**.h", 
	"ep/**.cpp",
	}
	includedirs { ".","..","../Testbed" }
	links { "Box2D", "GLFW", "IMGUI"}
	configuration { "windows" }
		links { "glu32", "opengl32", "winmm" }