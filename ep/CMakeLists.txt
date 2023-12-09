cmake_minimum_required(VERSION 3.8)

# https://cmake.org/cmake/help/latest/command/project.html
project(ep VERSION 0.2.0)

# set(CMAKE_CONFIGURATION_TYPES "Debug;RelWithDebInfo" CACHE STRING "" FORCE)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

option(BUILD_SHARED_LIBS "Build ep as a shared library" OFF)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
set(BOX2D_BIN_DIR "../../build")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

add_subdirectory(ep)
add_subdirectory(.. "${BOX2D_BIN_DIR}")

# default startup project for Visual Studio
if (MSVC)
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ep)
	set_property(TARGET ep PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/ep")
	add_compile_options("/ZI")
endif()
