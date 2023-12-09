set(EP_SOURCE_FILES
	ep_joint.cpp
	main.cpp
	test_entries.cpp
	test.cpp
	)

set(SHARED_TESTBED_SOURCE_FILES
	../../testbed/draw.cpp
	../../testbed/draw.h
	../../testbed/imgui_impl_glfw.cpp
	../../testbed/imgui_impl_glfw.h
	../../testbed/imgui_impl_opengl3.cpp
	../../testbed/imgui_impl_opengl3.h
)
set(EP_HEADER_FILES
	beam.h
	draw.h
	ep_car.h
	frame.h
	im_gui_demo.h
	test.h
	../../include/box2d/b2ep_joint.h
	../../include/box2d/b2_api.h
	../../include/box2d/b2_block_allocator.h
	../../include/box2d/b2_body.h
	../../include/box2d/b2_broad_phase.h
	../../include/box2d/b2_chain_shape.h
	../../include/box2d/b2_circle_shape.h
	../../include/box2d/b2_collision.h
	../../include/box2d/b2_common.h
	../../include/box2d/b2_contact.h
	../../include/box2d/b2_contact_manager.h
	../../include/box2d/b2_distance.h
	../../include/box2d/b2_distance_joint.h
	../../include/box2d/b2_draw.h
	../../include/box2d/b2_dynamic_tree.h
	../../include/box2d/b2_edge_shape.h
	../../include/box2d/b2_fixture.h
	../../include/box2d/b2_friction_joint.h
	../../include/box2d/b2_gear_joint.h
	../../include/box2d/b2_growable_stack.h
	../../include/box2d/b2_joint.h
	../../include/box2d/b2_math.h
	../../include/box2d/b2_motor_joint.h
	../../include/box2d/b2_mouse_joint.h
	../../include/box2d/b2_polygon_shape.h
	../../include/box2d/b2_prismatic_joint.h
	../../include/box2d/b2_pulley_joint.h
	../../include/box2d/b2_revolute_joint.h
	../../include/box2d/b2_rope.h
	../../include/box2d/b2_settings.h
	../../include/box2d/b2_shape.h
	../../include/box2d/b2_stack_allocator.h
	../../include/box2d/b2_time_of_impact.h
	../../include/box2d/b2_timer.h
	../../include/box2d/b2_time_step.h
	../../include/box2d/b2_types.h
	../../include/box2d/b2_weld_joint.h
	../../include/box2d/b2_wheel_joint.h
	../../include/box2d/b2_world.h
	../../include/box2d/b2_world_callbacks.h
	../../include/box2d/box2d.h
	../../src/dynamics/b2_island.h
	)

add_executable(ep)
target_include_directories(ep PRIVATE 
	../../include 
	../../src
	../../testbed
	../../extern/glad/include 
	../../extern/glfw/include 
	../../extern
	${CMAKE_CURRENT_SOURCE_DIR})
#	 TYPE CXX_MODULES
target_sources(ep 
 PRIVATE
 ${EP_SOURCE_FILES}
 ${SHARED_TESTBED_SOURCE_FILES}
)	
target_link_libraries(ep PUBLIC box2d glfw imgui sajson glad)
set_target_properties(ep PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

if(MSVC)
  set_target_properties(ep PROPERTIES
    COMPILE_PDB_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMPILE_PDB_NAME ep
  )

endif()


source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${EP_SOURCE_FILES})

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/epConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
)