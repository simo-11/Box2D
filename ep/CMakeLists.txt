cmake_minimum_required(VERSION 3.22)
include(FetchContent)
include(CMakeDependentOption)

# https://cmake.org/cmake/help/latest/command/project.html
project(ep VERSION 0.3.0)

# stuff to help debug cmake
#message(STATUS "cmake tool chain: ${CMAKE_TOOLCHAIN_FILE}")
#message(STATUS "cmake source dir: ${CMAKE_SOURCE_DIR}")
#message(STATUS "library postfix: ${CMAKE_DEBUG_POSTFIX}")
#message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
#message(STATUS "CMake C++ compiler: ${CMAKE_CXX_COMPILER_ID}")
#message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")
#message(STATUS "CMake host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

# static link
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

if (MSVC OR APPLE)
	option(BOX2D_SANITIZE "Enable sanitizers for some builds" OFF)
	if(BOX2D_SANITIZE)
		message(STATUS "Box2D Sanitize")
		# sanitizers need to apply to all compiled libraries to work well
		if(MSVC)
			# address sanitizer only in the debug build
			add_compile_options("$<$<CONFIG:Debug>:/fsanitize=address>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL:NO>")
		elseif(APPLE)
			# more sanitizers on Apple clang
			# add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
			add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
			add_link_options(-fsanitize=address -fsanitize-address-use-after-scope -fsanitize=undefined)
		endif()
	else()
		if(MSVC)
			# enable hot reloading
			add_compile_options("$<$<CONFIG:Debug>:/ZI>")
			add_link_options("$<$<CONFIG:Debug>:/INCREMENTAL>")
		endif()
	endif()
endif()

# Deterministic math
# https://box2d.org/posts/2024/08/determinism/
if (MINGW OR APPLE OR UNIX)
	add_compile_options(-ffp-contract=off)
endif()

option(BOX2D_ENABLE_SIMD "Enable SIMD math (faster)" ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
	cmake_dependent_option(BOX2D_AVX2 "Enable AVX2" OFF "BOX2D_ENABLE_SIMD" OFF)
endif()

# C++17 needed for imgui
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

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
