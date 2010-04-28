cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
# CMake 2.4.6 doesn't include FindPkgConfig, so we provide our own copy
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} )

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# We need GTK2 and OpenGL for the wx render window
pkg_check_modules(GTK2 gtk+-2.0)

include_directories( ${GTK2_INCLUDE_DIRS} )
add_definitions( ${GTK2_DEFINITIONS} )
link_directories( ${GTK2_LIBRARY_DIRS} )

find_package(OpenGL REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIR} )

set(X11_LIBS "")
if (UNIX AND NOT APPLE)
  include(FindX11)
  if (NOT X11_FOUND)
    message(send_error "X11 not found")
  endif(NOT X11_FOUND)
  
  set(X11_LIBS ${X11_LIBRARIES})
endif(UNIX AND NOT APPLE)

include_directories(src)

add_subdirectory(src)