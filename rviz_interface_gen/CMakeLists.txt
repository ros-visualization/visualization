cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(cmake/interfacebuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rviz_add_interface(interfaces/test.interface)

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
rosbuild_add_library(interfaces interface_gen/cpp/src/Test.cpp)
rosbuild_add_boost_directories()

rosbuild_add_executable(test_interface EXCLUDE_FROM_ALL  test/test_interface.cpp)
target_link_libraries(test_interface interfaces)
rosbuild_add_gtest_build_flags(test_interface)
rosbuild_add_rostest(test/test_interface.test)