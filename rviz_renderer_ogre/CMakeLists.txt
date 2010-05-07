cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


pkg_check_modules(OGRE OGRE)
include_directories( ${OGRE_INCLUDE_DIRS} )
link_directories( ${OGRE_LIBRARY_DIRS} )

include_directories(include)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/camera.cpp
									 src/convert.cpp
									 src/render_window.cpp
									 src/renderer.cpp
									 src/scene.cpp
									 src/transform_node.cpp
									 src/simple_shape.cpp
									 src/material.cpp
									 src/mesh_instance.cpp
									 src/simple_color_material.cpp
									 src/server.cpp
									 src/init.cpp
									 src/disable_rendering_scheme_listener.cpp)
target_link_libraries(${PROJECT_NAME} ${OGRE_LIBRARIES})
rosbuild_add_boost_directories()
