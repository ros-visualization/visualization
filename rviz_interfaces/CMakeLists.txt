cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(rviz_interface_gen)
include(${rviz_interface_gen_PACKAGE_PATH}/cmake/interfacebuild.cmake)
rviz_add_interface(interfaces/camera.interface)
rviz_add_interface(interfaces/render_window.interface)
rviz_add_interface(interfaces/scene.interface)
rviz_add_interface(interfaces/simple_shape.interface)
rviz_add_interface(interfaces/transform_node.interface)
rviz_add_interface(interfaces/material.interface)
rviz_add_interface(interfaces/mesh_instance.interface)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories(interface_gen/cpp/include)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} interface_gen/cpp/src/Camera.cpp
								     interface_gen/cpp/src/Scene.cpp
								     interface_gen/cpp/src/RenderWindow.cpp
								     interface_gen/cpp/src/SimpleShape.cpp
								     interface_gen/cpp/src/TransformNode.cpp
								     interface_gen/cpp/src/Material.cpp
								     interface_gen/cpp/src/MeshInstance.cpp)
rosbuild_add_boost_directories()
