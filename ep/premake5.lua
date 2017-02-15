-- Box2D -ep (elastic-plastic) extensions premake5 script.
-- https://premake.github.io/

workspace "0_Box2D-EP"
	location ( "Build/" .. _ACTION )
	architecture "x86_64"
	configurations { "Debug", "Release" }
	
	configuration "vs*"
		defines { "_CRT_SECURE_NO_WARNINGS" }	
		
	filter "configurations:Debug"
		targetdir ( "Build/" .. _ACTION .. "/bin/Debug" )
	 	defines { "DEBUG" }
		symbols "On"

	filter "configurations:Release"
		targetdir ( "Build/" .. _ACTION .. "/bin/Release" )
		defines { "NDEBUG" }
		optimize "On"

project "Box2D"
	kind "StaticLib"
	language "C++"
	files { "../Box2D/Box2D/**.h", "../Box2D/Box2D/**.cpp" }
	includedirs { "../Box2D" }
	
project "GLEW"
	kind "StaticLib"
	language "C++"
	defines {"GLEW_STATIC"}
	files { "../Box2D/glew/*.h", "../Box2D/glew/*.c" }
	includedirs { "../Box2D" }
		 
local glfw_common = {
	"../Box2D/glfw/internal.h",
	"../Box2D/glfw/glfw_config.h",
	"../Box2D/glfw/glfw3.h",
	"../Box2D/glfw/glfw3native.h",
	"../Box2D/glfw/context.c",
	"../Box2D/glfw/init.c",
	"../Box2D/glfw/input.c",
	"../Box2D/glfw/monitor.c",
	"../Box2D/glfw/window.c" }

project "GLFW"
	kind "StaticLib"
	language "C"
	configuration { "windows" }
		local f = {
			"../Box2D/glfw/win32_platform.h",
			"../Box2D/glfw/win32_tls.h",
			"../Box2D/glfw/winmm_joystick.h",
			"../Box2D/glfw/wglext.h",
			"../Box2D/glfw/win32_init.c",
			"../Box2D/glfw/win32_monitor.c",
			"../Box2D/glfw/win32_time.c",
            "../Box2D/glfw/win32_tls.c",
            "../Box2D/glfw/win32_window.c",
            "../Box2D/glfw/winmm_joystick.c",
        	"../Box2D/glfw/wgl_context.c"}
   
        for i, v in ipairs(glfw_common) do
        	f[#f + 1] = glfw_common[i]
        end
    	files(f)

project "IMGUI"
	kind "StaticLib"
	language "C++"
	defines {"GLEW_STATIC"}
	files { "../Box2D/imgui/*.h", "../Box2D/imgui/*.cpp" }
	includedirs { "../Box2D" }

project "ep"
	kind "ConsoleApp"
	language "C++"
	defines {"GLEW_STATIC"}
	files { "ep/**.h", 
	"ep/**.cpp",
	}
	includedirs { ".","../Box2D","../../Box2D/Box2D" }
	links { "Box2D", "GLEW", "GLFW", "IMGUI"}
	configuration { "windows" }
		links { "glu32", "opengl32", "winmm" }